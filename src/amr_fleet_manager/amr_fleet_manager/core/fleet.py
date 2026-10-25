"""Fleet manager core: task allocation, charging, priorities and traffic.

`FleetCore` is deliberately free of ROS so that it can be unit tested and
run in the headless simulator.  The ROS node only converts messages.

Robot life cycle::

    PARKED/IDLE --assign--> TO_PICKUP -> LOADING -> TO_DROPOFF -> UNLOADING
         ^                                                          |
         +------ TO_PARK <---- (no task) <---- next task? <---------+
                 TO_CHARGER -> CHARGING   (battery low / opportunistic)
                 WAIT_CHARGER (all 5 docks busy -> wait on a parking bay)

    any moving state --(conflict with higher priority robot)--> YIELDING
          (drive into a yield bay / off the other robot's path, wait until it
           has passed, then rejoin the *original* route and resume)
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Set, Tuple

from .graph import CHARGER, INTERSECTION, LANE, PARKING, STATION, YIELD, TrafficGraph
from .layout import build_graph
from .production import Production
from .tasks import (ASSIGNED, IN_TRANSIT, LOADING as T_LOADING, MANUAL, QUEUED,
                    TO_PICKUP as T_TO_PICKUP, UNLOADING as T_UNLOADING, Task, TaskBook)
from .traffic import TrafficAgent, TrafficController, TrafficParams

# robot states
OFFLINE = 'OFFLINE'
IDLE = 'IDLE'
PARKED = 'PARKED'
TO_PARK = 'TO_PARK'
TO_PICKUP = 'TO_PICKUP'
LOADING = 'LOADING'
TO_DROPOFF = 'TO_DROPOFF'
UNLOADING = 'UNLOADING'
TO_CHARGER = 'TO_CHARGER'
CHARGING = 'CHARGING'
WAIT_CHARGER = 'WAIT_CHARGER'
YIELDING = 'YIELDING'

TASK_STATES = (TO_PICKUP, LOADING, TO_DROPOFF, UNLOADING)
WORK_STATES = (LOADING, UNLOADING, CHARGING)

STATE_TEXT = {
    OFFLINE: 'Mất kết nối',
    IDLE: 'Rảnh',
    PARKED: 'Đang đỗ, chờ task',
    TO_PARK: 'Về bãi đỗ',
    TO_PICKUP: 'Đi tới điểm lấy hàng',
    LOADING: 'Đang nhận hàng',
    TO_DROPOFF: 'Đang chở hàng tới điểm trả',
    UNLOADING: 'Đang trả hàng',
    TO_CHARGER: 'Đi tới trạm sạc',
    CHARGING: 'Đang sạc pin',
    WAIT_CHARGER: 'Chờ dock sạc trống',
    YIELDING: 'Nhường đường',
}

INFO, WARN, ERROR = 0, 1, 2


@dataclass
class FleetParams:
    station_capacity: int = 2          # robots booked per station (service + queue slot)
    battery_low: float = 30.0
    battery_critical: float = 15.0
    battery_full: float = 95.0
    battery_opportunistic: float = 50.0
    opportunistic_max_trip: float = 70.0  # s, only charge 'on the way' when a dock is near
    charge_release: float = 60.0       # opportunistic charger may leave for a task
    charge_release_low: float = 80.0   # robot that came in low may leave for a task
    assign_period: float = 1.0
    yield_wait: float = 3.0            # higher priority robot stalled this long -> ask lower to yield
    cycle_wait: float = 1.5
    reroute_wait: float = 20.0
    yield_timeout: float = 45.0
    yield_max_cost: float = 45.0
    status_timeout: float = 5.0
    nominal_speed: float = 0.8
    turn_time_per_rad: float = 0.8
    priority_mode: str = 'dynamic'     # dynamic | static
    w_task: float = 1000.0
    w_cargo: float = 400.0
    w_charge: float = 600.0
    w_charge_critical: float = 3500.0
    w_base: float = 15.0
    w_aging: float = 4.0
    aging_cap: float = 160.0
    lookahead: float = 4.5


@dataclass
class RobotStatusIn:
    robot_id: str
    x: float
    y: float
    yaw: float
    v: float = 0.0
    w: float = 0.0
    command_id: int = 0
    reached_index: int = -1
    route_complete: bool = False
    battery: float = 100.0
    charging: bool = False
    obstacle_stop: bool = False
    min_obstacle_distance: float = math.inf
    stamp: float = 0.0


@dataclass
class Command:
    robot_id: str
    command_id: int
    mode: int
    node_ids: List[str]
    waypoints: List[Tuple[float, float]]
    speed_limits: List[float]
    granted_index: int
    final_yaw: float
    charge: bool

    def key(self) -> tuple:
        return (self.command_id, self.mode, self.granted_index, self.charge)


@dataclass
class RobotRecord:
    id: str
    base_rank: int
    state: str = OFFLINE
    online: bool = False
    paused: bool = False
    needs_init: bool = True
    # last status
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    battery: float = 100.0
    charging: bool = False
    obstacle_stop: bool = False
    last_status: float = -math.inf
    # navigation
    route: List[int] = field(default_factory=list)
    prefix_len: int = 1
    command_id: int = 0
    acked: bool = True
    progress: int = 0
    route_done: bool = False
    granted: int = 0
    held: List[int] = field(default_factory=list)
    held_end: int = 0
    final_yaw: float = float('nan')
    current_node: Optional[int] = None
    blocked_by: Optional[str] = None
    blocked_node: Optional[int] = None
    stall_since: Optional[float] = None
    last_reroute: float = -math.inf
    # job
    task: Optional[Task] = None
    next_task: Optional[Task] = None
    work_until: float = 0.0
    cargo: str = ''
    dock: Optional[str] = None
    parking: Optional[str] = None
    opportunistic: bool = False
    yield_ctx: Optional[dict] = None
    # priority
    priority: float = 0.0
    priority_static: float = 0.0
    rank: int = 0
    # stats
    tasks_done: int = 0
    odometer: float = 0.0

    @property
    def job_state(self) -> str:
        if self.state == YIELDING and self.yield_ctx:
            return self.yield_ctx.get('state') or IDLE
        return self.state

    def waiting(self, now: float) -> float:
        return now - self.stall_since if self.stall_since is not None else 0.0


class FleetCore:
    def __init__(self, layout: dict, robot_ids: Sequence[str],
                 params: Optional[FleetParams] = None,
                 graph: Optional[TrafficGraph] = None) -> None:
        self.layout = layout
        self.g = graph or build_graph(layout)
        self.p = params or FleetParams()
        self.production = Production(layout.get('process', {}) or {})
        self.tasks = TaskBook()
        self.traffic = TrafficController(self.g, TrafficParams(lookahead=self.p.lookahead))
        self.robots: Dict[str, RobotRecord] = {}
        for i, rid in enumerate(robot_ids):
            self.robots[rid] = RobotRecord(rid, base_rank=i + 1)
        self.station_bookings: Dict[str, List[str]] = {s: [] for s in self.g.stations}
        self.dock_booking: Dict[str, Optional[str]] = {c: None for c in self.g.chargers}
        self.parking_booking: Dict[str, Optional[str]] = {p: None for p in self.g.parking}
        self.yield_booking: Dict[int, str] = {}
        self.charge_queue: List[str] = []
        self.events: Deque[dict] = deque(maxlen=400)
        self.event_seq = 0
        self.paused = False
        self.now = 0.0
        self._last_tick: Optional[float] = None
        self._t0: Optional[float] = None
        self._last_assign = -math.inf
        self.yields = 0
        self.deadlocks = 0
        self.reroutes = 0
        self.physical_conflicts = 0
        self._agents: Dict[str, TrafficAgent] = {}

    # ================================================================ API
    @property
    def elapsed(self) -> float:
        return self.now - self._t0 if self._t0 is not None else 0.0

    def event(self, level: int, category: str, robot: str, text: str) -> None:
        self.event_seq += 1
        self.events.append({'seq': self.event_seq, 't': self.elapsed, 'level': level,
                            'category': category, 'robot': robot, 'text': text})

    def update_status(self, st: RobotStatusIn) -> None:
        r = self.robots.get(st.robot_id)
        if r is None:
            return
        if not r.online:
            r.online = True
            if r.state == OFFLINE:
                r.needs_init = True
        if r.last_status > -math.inf:
            r.odometer += math.hypot(st.x - r.x, st.y - r.y)
        r.x, r.y, r.yaw, r.v = st.x, st.y, st.yaw, st.v
        r.battery = st.battery
        r.charging = st.charging
        r.obstacle_stop = st.obstacle_stop
        r.last_status = st.stamp if st.stamp else self.now
        if r.route and st.command_id == r.command_id:
            r.acked = True
            r.progress = max(0, min(st.reached_index, len(r.route) - 1))
            r.route_done = bool(st.route_complete)

    def submit_task(self, pickup: str, dropoff: str, priority: int = 1,
                    description: str = '') -> Tuple[bool, str, str]:
        if pickup not in self.g.stations or dropoff not in self.g.stations:
            return False, '', 'unknown station'
        if pickup == dropoff:
            return False, '', 'pickup == dropoff'
        t = Task(MANUAL, pickup, dropoff, priority=max(0, min(3, int(priority))),
                 description=description or f'Task thủ công {pickup} -> {dropoff}',
                 cargo='Hàng thủ công', created=self.now)
        self.tasks.add(t)
        self.event(INFO, 'TASK', '', f'Tạo task thủ công {t.id}: {pickup} → {dropoff} (ưu tiên {t.priority})')
        return True, t.id, 'ok'

    def set_base_rank(self, robot_id: str, rank: int) -> bool:
        r = self.robots.get(robot_id)
        if r is None or rank <= 0:
            return False
        # move robot to `rank` and shift the others
        order = sorted(self.robots.values(), key=lambda x: x.base_rank)
        order.remove(r)
        order.insert(min(rank, len(order) + 1) - 1, r)
        for i, x in enumerate(order):
            x.base_rank = i + 1
        self.event(INFO, 'TRAFFIC', robot_id, f'Hạng ưu tiên cơ bản của {robot_id} -> {r.base_rank}')
        return True

    def set_robot_paused(self, robot_id: str, paused: bool) -> bool:
        r = self.robots.get(robot_id)
        if r is None:
            return False
        r.paused = paused
        self.event(WARN, 'SYSTEM', robot_id, f'{robot_id} {"tạm dừng" if paused else "tiếp tục"}')
        return True

    def set_paused(self, paused: bool) -> None:
        self.paused = paused
        self.event(WARN, 'SYSTEM', '', 'Toàn bộ fleet TẠM DỪNG' if paused else 'Fleet tiếp tục hoạt động')

    # =============================================================== tick
    def tick(self, now: float) -> Dict[str, Command]:
        dt = 0.0 if self._last_tick is None else max(0.0, min(now - self._last_tick, 1.0))
        self._last_tick = now
        self.now = now
        if self._t0 is None:
            self._t0 = now
        for r in self.robots.values():
            if r.online and now - r.last_status > self.p.status_timeout:
                r.online = False
                self.event(ERROR, 'SYSTEM', r.id, f'{r.id} mất kết nối')
            if r.online and r.needs_init:
                self._init_robot(r)
        if not self.paused:
            supply = {s: len(b) for s, b in self.station_bookings.items()}
            for t in self.tasks.active():
                if t.status == QUEUED:
                    supply[t.pickup] = supply.get(t.pickup, 0) + 1
                    supply[t.dropoff] = supply.get(t.dropoff, 0) + 1
            for t in self.production.update(dt, now, dict(supply), dict(supply)):
                self.tasks.add(t)
            while self.production.events:
                self.event(INFO, 'PRODUCTION', '', self.production.events.pop(0))
        for r in self.robots.values():
            if not r.online:
                continue
            if r.route and r.acked and r.route_done:
                self._arrived(r)
            self._update_work(r)
            self._battery_watch(r)
        self._serve_charge_queue()
        if now - self._last_assign >= self.p.assign_period and not self.paused:
            self._last_assign = now
            self._assign()
        self._priorities()
        self._run_traffic()
        if not self.paused:
            self._resolve_conflicts()
            self._release_yields()
            self._reroute_stalled()
        return self._commands()

    # ============================================================ robots
    def _init_robot(self, r: RobotRecord) -> None:
        r.needs_init = False
        node = self.g.nearest_node(r.x, r.y)
        r.current_node = node
        r.route = []
        n = self.g.nodes[node]
        r.state = IDLE
        if n.kind == PARKING and math.hypot(n.x - r.x, n.y - r.y) < 0.8 \
                and self.parking_booking.get(n.group) in (None, r.id):
            self.parking_booking[n.group] = r.id
            r.parking = n.group
            r.state = PARKED
        elif n.kind == CHARGER and self.dock_booking.get(n.group) in (None, r.id):
            self.dock_booking[n.group] = r.id
            r.dock = n.group
            r.state = CHARGING
            r.opportunistic = True
        else:
            self._go_idle(r)
        self.event(INFO, 'SYSTEM', r.id, f'{r.id} online tại {n.id} (pin {r.battery:.0f}%)')

    def _prefix(self, r: RobotRecord) -> List[int]:
        if not r.route:
            return [r.current_node if r.current_node is not None
                    else self.g.nearest_node(r.x, r.y)]
        if not r.acked:
            return list(r.route[:max(1, r.prefix_len)])
        k = min(r.progress, len(r.route) - 1)
        end = max(k, min(r.held_end, len(r.route) - 1))
        return list(r.route[k:end + 1])

    def _penalties(self, r: RobotRecord) -> Dict[int, float]:
        pen: Dict[int, float] = {}
        for o in self.robots.values():
            if o.id == r.id or not o.online:
                continue
            stuck = o.stall_since is not None and self.now - o.stall_since > 8.0
            parked = not o.route
            for n in o.held:
                w = 25.0 if (stuck or parked) else 8.0
                for c in self.g.conflicts[n]:
                    pen[c] = max(pen.get(c, 0.0), w)
        for n, owner in self.traffic.owner.items():
            if owner != r.id:
                pen[n] = max(pen.get(n, 0.0), 2.0)
        return pen

    def _set_route(self, r: RobotRecord, route: List[int], prefix_len: int,
                   state: str, final_yaw: float) -> None:
        r.route = route
        r.prefix_len = max(1, prefix_len)
        r.command_id += 1
        r.acked = False
        r.progress = 0
        r.route_done = False
        r.granted = r.prefix_len - 1
        r.held_end = r.prefix_len - 1
        r.final_yaw = final_yaw
        r.state = state
        r.stall_since = None
        if r.parking is not None and self.g.parking.get(r.parking) != route[-1]:
            self._release_parking(r)

    def _navigate(self, r: RobotRecord, goal: int, state: str,
                  final_yaw: Optional[float] = None,
                  blocked: Optional[Set[int]] = None) -> bool:
        prefix = self._prefix(r)
        start = prefix[-1]
        if start == goal:
            path = [goal]
        else:
            path = self.g.plan(start, goal, node_penalty=self._penalties(r), blocked=blocked)
        if path is None:
            self.event(WARN, 'TRAFFIC', r.id,
                       f'{r.id}: không tìm được đường {self.g.nodes[start].id} → {self.g.nodes[goal].id}')
            return False
        if final_yaw is None:
            h = self.g.nodes[goal].heading
            final_yaw = h if h is not None else float('nan')
        self._set_route(r, prefix[:-1] + path, len(prefix), state, final_yaw)
        return True

    def _station_goal(self, sid: str) -> int:
        return self.g.stations[sid].service

    # ------------------------------------------------------------ arrival
    def _arrived(self, r: RobotRecord) -> None:
        r.current_node = r.route[-1]
        r.route = []
        r.route_done = False
        r.held = [r.current_node]
        r.stall_since = None
        st = r.state
        if st == YIELDING:
            if r.yield_ctx is not None and r.yield_ctx.get('arrived') is None:
                r.yield_ctx['arrived'] = self.now
            return
        if st == TO_PICKUP:
            r.state = LOADING
            r.work_until = self.now + self.production.load_time
            if r.task:
                r.task.status = T_LOADING
        elif st == TO_DROPOFF:
            r.state = UNLOADING
            r.work_until = self.now + self.production.unload_time
            if r.task:
                r.task.status = T_UNLOADING
        elif st == TO_CHARGER:
            r.state = CHARGING
            self.event(INFO, 'CHARGE', r.id, f'{r.id} vào dock {r.dock}, bắt đầu sạc ({r.battery:.0f}%)')
        elif st == TO_PARK:
            r.state = PARKED
        elif st == WAIT_CHARGER:
            pass
        else:
            r.state = IDLE

    def _update_work(self, r: RobotRecord) -> None:
        now = self.now
        if r.state == LOADING and now >= r.work_until and r.task:
            t = r.task
            self.production.on_pickup(t)
            t.status = IN_TRANSIT
            t.picked_at = now
            r.cargo = t.cargo
            self._unbook_station(t.pickup, r.id)
            self.event(INFO, 'TASK', r.id, f'{r.id} đã nhận "{t.cargo}" tại {t.pickup} → chở tới {t.dropoff}')
            if not self._navigate(r, self._station_goal(t.dropoff), TO_DROPOFF):
                r.state = LOADING
                r.work_until = now + 2.0
        elif r.state == UNLOADING and now >= r.work_until and r.task:
            t = r.task
            self.production.on_dropoff(t)
            self._unbook_station(t.dropoff, r.id)
            r.cargo = ''
            r.task = None
            r.tasks_done += 1
            self.tasks.finish(t, now)
            self.event(INFO, 'TASK', r.id, f'{r.id} hoàn thành {t.id} ({t.description}) '
                                           f'trong {now - (t.assigned_at if t.assigned_at is not None else now):.0f}s')
            self._after_task(r)
        elif r.state == CHARGING:
            if r.battery >= self.p.battery_full:
                self.event(INFO, 'CHARGE', r.id, f'{r.id} sạc đầy ({r.battery:.0f}%), rời dock {r.dock}')
                self._release_dock(r)
                self._go_idle(r, allow_charge=False)
        elif r.state == IDLE and not r.route and now - r.work_until > 2.0:
            r.work_until = now
            self._go_idle(r)

    def _after_task(self, r: RobotRecord) -> None:
        nt = r.next_task
        if r.battery < self.p.battery_low:
            if nt is not None:
                self._unassign_next(r)
            self.event(WARN, 'CHARGE', r.id, f'{r.id} pin thấp ({r.battery:.0f}%) → đi sạc')
            self._request_charge(r, opportunistic=False)
            return
        if nt is not None:
            r.next_task = None
            nt.next_for = None
            self._start_task(r, nt, already_booked=True)
            return
        self._go_idle(r)

    def _go_idle(self, r: RobotRecord, allow_charge: bool = True) -> None:
        if allow_charge and r.battery < self.p.battery_low:
            self._request_charge(r, opportunistic=False)
            return
        if allow_charge and r.battery < self.p.battery_opportunistic:
            dock = self._free_dock(r)
            here = r.current_node if not r.route else self._prefix(r)[-1]
            if dock is not None and self.g.travel_time(here, self.g.chargers[dock]) \
                    <= self.p.opportunistic_max_trip:
                self._request_charge(r, opportunistic=True)
                return
        self._go_park(r)

    # ------------------------------------------------------------ parking
    def _go_park(self, r: RobotRecord, near: Optional[int] = None) -> None:
        here = r.current_node if not r.route else self._prefix(r)[-1]
        if r.parking is not None and not r.route and self.g.parking[r.parking] == r.current_node:
            r.state = PARKED
            return
        best, best_c = None, math.inf
        ref = near if near is not None else here
        for pid, node in self.g.parking.items():
            if self.parking_booking[pid] not in (None, r.id):
                continue
            c = self.g.travel_time(ref, node) if near is None else self.g.distance(ref, node)
            if c < best_c:
                best, best_c = pid, c
        if best is None:
            r.state = IDLE
            return
        self._release_parking(r)
        self.parking_booking[best] = r.id
        if self._navigate(r, self.g.parking[best], TO_PARK):
            r.parking = best
        else:
            self.parking_booking[best] = None
            r.state = IDLE

    def _release_parking(self, r: RobotRecord) -> None:
        if r.parking is not None and self.parking_booking.get(r.parking) == r.id:
            self.parking_booking[r.parking] = None
        r.parking = None

    # ------------------------------------------------------------ charging
    def _free_dock(self, r: RobotRecord) -> Optional[str]:
        here = r.current_node if not r.route else self._prefix(r)[-1]
        best, best_c = None, math.inf
        for cid, node in self.g.chargers.items():
            if self.dock_booking[cid] is not None:
                continue
            c = self.g.travel_time(here, node)
            if c < best_c:
                best, best_c = cid, c
        return best

    def _request_charge(self, r: RobotRecord, opportunistic: bool) -> None:
        dock = self._free_dock(r)
        if dock is not None and (opportunistic or not self.charge_queue):
            self.dock_booking[dock] = r.id
            r.dock = dock
            r.opportunistic = opportunistic
            if self._navigate(r, self.g.chargers[dock], TO_CHARGER):
                kind = 'sạc tranh thủ' if opportunistic else 'sạc'
                self.event(INFO, 'CHARGE', r.id, f'{r.id} → dock {dock} ({kind}, pin {r.battery:.0f}%)')
                if r.id in self.charge_queue:
                    self.charge_queue.remove(r.id)
                return
            self.dock_booking[dock] = None
            r.dock = None
        if opportunistic:
            self._go_park(r)
            return
        if r.id not in self.charge_queue:
            self.charge_queue.append(r.id)
            self.event(WARN, 'CHARGE', r.id, f'{r.id} chờ dock sạc trống (hàng đợi {len(self.charge_queue)})')
        # wait on the parking bay closest to the charging station
        charger_center = next(iter(self.g.chargers.values()))
        self._go_park(r, near=charger_center)
        r.state = WAIT_CHARGER

    def _release_dock(self, r: RobotRecord) -> None:
        if r.dock is not None and self.dock_booking.get(r.dock) == r.id:
            self.dock_booking[r.dock] = None
        r.dock = None
        r.opportunistic = False

    def _serve_charge_queue(self) -> None:
        if not self.charge_queue:
            return
        # opportunistic chargers make room for robots that really need it
        free = [c for c, o in self.dock_booking.items() if o is None]
        if not free:
            for cid, rid in self.dock_booking.items():
                o = self.robots.get(rid) if rid else None
                if o and o.opportunistic and o.state == CHARGING and o.battery >= self.p.charge_release:
                    self.event(INFO, 'CHARGE', o.id, f'{o.id} nhường dock {cid} cho robot pin thấp')
                    self._release_dock(o)
                    self._go_park(o)
                    break
        queue = sorted((self.robots[i] for i in self.charge_queue if i in self.robots),
                       key=lambda x: x.battery)
        for r in queue:
            if r.state != WAIT_CHARGER:
                self.charge_queue.remove(r.id)
                continue
            dock = self._free_dock(r)
            if dock is None:
                break
            self.charge_queue.remove(r.id)
            self.dock_booking[dock] = r.id
            r.dock = dock
            r.opportunistic = False
            if self._navigate(r, self.g.chargers[dock], TO_CHARGER):
                self.event(INFO, 'CHARGE', r.id, f'{r.id} được cấp dock {dock} (pin {r.battery:.0f}%)')
            else:
                self.dock_booking[dock] = None
                r.dock = None
                self.charge_queue.append(r.id)

    def _battery_watch(self, r: RobotRecord) -> None:
        if r.battery < self.p.battery_low and r.state in (PARKED, TO_PARK) and r.yield_ctx is None:
            self.event(WARN, 'CHARGE', r.id, f'{r.id} pin thấp ({r.battery:.0f}%) → đi sạc')
            self._request_charge(r, opportunistic=False)
            return
        if r.battery >= self.p.battery_critical:
            return
        if r.state == TO_PICKUP and r.task is not None and not r.cargo:
            t = r.task
            self.event(ERROR, 'CHARGE', r.id, f'{r.id} pin nguy hiểm ({r.battery:.0f}%) → huỷ {t.id}, đi sạc')
            self._unassign(r, t)
            if r.next_task:
                self._unassign_next(r)
            self._request_charge(r, opportunistic=False)

    # ------------------------------------------------------------ tasks
    def _book_station(self, sid: str, rid: str) -> None:
        self.station_bookings[sid].append(rid)

    def _unbook_station(self, sid: str, rid: str) -> None:
        if rid in self.station_bookings.get(sid, []):
            self.station_bookings[sid].remove(rid)

    def _station_free(self, sid: str) -> bool:
        return len(self.station_bookings[sid]) < self.p.station_capacity

    def _anchor(self, r: RobotRecord) -> int:
        return self._prefix(r)[-1]

    def _available(self, r: RobotRecord) -> bool:
        if not r.online or r.paused or r.task is not None or r.yield_ctx is not None:
            return False
        if r.state in (PARKED, TO_PARK, IDLE):
            return r.battery >= self.p.battery_low
        if r.state == CHARGING:
            need = self.p.charge_release if r.opportunistic else self.p.charge_release_low
            return r.battery >= need
        return False

    def _assign(self) -> None:
        open_tasks = self.tasks.open_tasks()
        if not open_tasks:
            return
        for t in open_tasks:
            if not (self._station_free(t.pickup) and self._station_free(t.dropoff)):
                continue
            target = self._station_goal(t.pickup)
            best, best_c = None, math.inf
            for r in self.robots.values():
                if not self._available(r):
                    continue
                c = self.g.travel_time(self._anchor(r), target)
                if r.state == CHARGING:
                    c += 15.0  # prefer robots that are not charging
                if c < best_c:
                    best, best_c = r, c
            busy, busy_c = None, math.inf
            for r in self.robots.values():
                if (not r.online or r.paused or r.next_task is not None or r.task is None
                        or r.job_state not in (TO_DROPOFF, UNLOADING)
                        or r.battery < self.p.battery_low + 8.0):
                    continue
                c = self._eta_task(r) + self.g.travel_time(self._station_goal(r.task.dropoff), target)
                if c < busy_c:
                    busy, busy_c = r, c
            if busy is not None and busy_c + 3.0 < best_c:
                self._book_station(t.pickup, busy.id)
                self._book_station(t.dropoff, busy.id)
                busy.next_task = t
                t.next_for = busy.id
                self.event(INFO, 'TASK', busy.id, f'{busy.id} nhận trước task tiếp theo {t.id} ({t.description})')
            elif best is not None:
                self._start_task(best, t)

    def _start_task(self, r: RobotRecord, t: Task, already_booked: bool = False) -> None:
        if not already_booked:
            self._book_station(t.pickup, r.id)
            self._book_station(t.dropoff, r.id)
        if r.state == CHARGING:
            self.event(INFO, 'CHARGE', r.id, f'{r.id} rời dock {r.dock} để nhận task ({r.battery:.0f}%)')
            self._release_dock(r)
        t.robot = r.id
        t.status = T_TO_PICKUP
        t.assigned_at = self.now
        r.task = t
        if not self._navigate(r, self._station_goal(t.pickup), TO_PICKUP):
            self._unassign(r, t)
            self._go_idle(r)
            return
        self.event(INFO, 'TASK', r.id, f'{r.id} nhận {t.id}: {t.description} '
                                       f'(ưu tiên {t.priority}, ETA {self._eta_task(r):.0f}s)')

    def _unassign(self, r: RobotRecord, t: Task) -> None:
        self._unbook_station(t.pickup, r.id)
        self._unbook_station(t.dropoff, r.id)
        t.status = QUEUED
        t.robot = None
        t.assigned_at = None
        r.task = None

    def _unassign_next(self, r: RobotRecord) -> None:
        t = r.next_task
        if t is None:
            return
        self._unbook_station(t.pickup, r.id)
        self._unbook_station(t.dropoff, r.id)
        t.next_for = None
        r.next_task = None

    # =========================================================== priority
    def _priorities(self) -> None:
        p = self.p
        N = len(self.robots)
        for r in self.robots.values():
            base = (N + 1 - r.base_rank) * p.w_base
            s = 0.0
            js = r.job_state
            if r.task is not None and js in TASK_STATES:
                s += p.w_task * (r.task.priority + 1)
            if r.cargo:
                s += p.w_cargo
            if js in (TO_CHARGER, WAIT_CHARGER):
                s += p.w_charge_critical if r.battery < p.battery_critical else p.w_charge
            if p.priority_mode == 'static':
                s = 0.0
            r.priority_static = s + base
            aging = 0.0
            if p.priority_mode != 'static' and r.stall_since is not None:
                aging = min((self.now - r.stall_since) * p.w_aging, p.aging_cap)
            r.priority = r.priority_static + aging
        for i, r in enumerate(sorted(self.robots.values(), key=lambda x: (-x.priority, x.base_rank))):
            r.rank = i + 1

    # ============================================================ traffic
    def _run_traffic(self) -> None:
        agents = []
        for r in self.robots.values():
            a = self._agents.get(r.id)
            if a is None:
                a = TrafficAgent(r.id, 0.0)
                self._agents[r.id] = a
            a.priority = r.priority
            a.x, a.y, a.speed = r.x, r.y, r.v
            a.route = r.route
            a.progress = r.progress
            a.acked = r.acked
            a.prefix_len = r.prefix_len
            a.prev_granted = r.granted
            a.current_node = r.current_node if r.current_node is not None else None
            if a.current_node is None and r.online:
                a.current_node = self.g.nearest_node(r.x, r.y)
            a.hold = self.paused or r.paused or not r.online
            if not r.online and r.current_node is None:
                continue
            agents.append(a)
        self.traffic.compute(agents)
        if self.traffic.physical_conflicts:
            self.physical_conflicts += len(self.traffic.physical_conflicts)
        for a in agents:
            r = self.robots[a.id]
            r.held = a.held
            r.held_end = a.held_end
            r.granted = a.granted
            r.blocked_by = a.blocked_by
            r.blocked_node = a.blocked_node
            stalled = (bool(r.route) and not r.route_done and a.blocked_by is not None
                       and a.granted <= a.held_end and abs(r.v) < 0.06)
            if stalled:
                if r.stall_since is None:
                    r.stall_since = self.now
            else:
                r.stall_since = None

    def _stalled_for(self, r: RobotRecord) -> float:
        return r.waiting(self.now)

    def _resolve_conflicts(self) -> None:
        robots = self.robots
        waits = {r.id: r.blocked_by for r in robots.values()
                 if r.blocked_by and self._stalled_for(r) >= self.p.cycle_wait}
        handled: Set[str] = set()
        done: Set[str] = set()
        for start in list(waits):
            if start in done:
                continue
            path: List[str] = []
            pos: Dict[str, int] = {}
            cur = start
            while cur in waits and cur not in pos and cur not in done:
                pos[cur] = len(path)
                path.append(cur)
                cur = waits[cur]
            if cur in pos:
                cycle = path[pos[cur]:]
                if not any(robots[c].state == YIELDING and robots[c].route for c in cycle):
                    if self._break_cycle(cycle):
                        handled.update(cycle)
            done.update(path)
        # priority yielding (overtaking / head-on with a stopped lower priority robot)
        for h in sorted(robots.values(), key=lambda x: -x.priority_static):
            l_id = h.blocked_by
            if not l_id or h.id in handled or l_id in handled:
                continue
            if self._stalled_for(h) < self.p.yield_wait:
                continue
            L = robots.get(l_id)
            if L is None or L.state in WORK_STATES or L.paused or not L.online:
                continue
            if L.priority_static >= h.priority_static:
                continue
            if L.route and (abs(L.v) > 0.08 or L.stall_since is None):
                continue  # L is moving away, just follow it
            if L.state == YIELDING and L.yield_ctx and L.yield_ctx.get('arrived') is None and L.route:
                continue
            if not self._helps(h, L):
                continue
            if self._start_yield(L, {h.id}, reason='nhường robot ưu tiên cao',
                                 note=f'{h.id} (P{h.rank}) bị {L.id} (P{L.rank}) chặn '
                                      f'{self._stalled_for(h):.0f}s'):
                handled.update({h.id, L.id})

    def _helps(self, h: RobotRecord, L: RobotRecord) -> bool:
        a = self._agents.get(h.id)
        return a is not None and a.block_owners == {L.id}

    def _break_cycle(self, cycle: List[str]) -> bool:
        members = [self.robots[c] for c in cycle]
        names = ' ⇄ '.join(cycle) if len(cycle) == 2 else ' → '.join(cycle + [cycle[0]])
        kind = 'đối đầu' if len(cycle) == 2 else 'deadlock vòng'
        for L in sorted(members, key=lambda x: (x.priority_static, -x.base_rank)):
            if L.state in WORK_STATES or L.paused:
                continue
            others = {m.id for m in members if m.id != L.id}
            note = f'Phát hiện {kind} {names}: {L.id} có ưu tiên thấp nhất nên phải nhường'
            if self._start_yield(L, others, reason=kind, note=note):
                self.deadlocks += 1
                return True
        # nobody can yield: try to reroute the lowest priority robot around
        for L in sorted(members, key=lambda x: x.priority_static):
            if L.route and self._reroute(L, force=True):
                self.deadlocks += 1
                return True
        return False

    def _refuge_score(self, L: RobotRecord, u: int, avoid: Set[int]) -> Optional[float]:
        n = self.g.nodes[u]
        if u in avoid:
            return None
        if n.kind == YIELD:
            return 0.0 if self.yield_booking.get(u) in (None, L.id) else None
        if n.kind == PARKING:
            pid = n.group
            return 3.0 if self.parking_booking.get(pid) in (None, L.id) else None
        if n.kind == LANE:
            if self.g.conflicts[u] & avoid:
                return None
            for e in self.g.out_edges[u]:
                if self.g.is_intersection(e.v):
                    return None
            for e in self.g.in_edges[u]:
                if self.g.is_intersection(e.u):
                    return None
            return 10.0
        return None

    def _find_refuge(self, L: RobotRecord, avoid_ids: Set[str]) -> Optional[List[int]]:
        g = self.g
        avoid: Set[int] = set()
        for hid in avoid_ids:
            H = self.robots[hid]
            seq = H.route[H.progress:H.progress + 40] if H.route else list(H.held)
            seq = list(seq) + list(H.held)
            avoid |= self.traffic.closure(seq)
        blocked = self.traffic.closure(self.traffic.nodes_owned_by_others(L.id))
        start = self._prefix(L)[-1]
        dist = {start: 0.0}
        parent: Dict[int, int] = {}
        heap = [(0.0, start)]
        best, best_score = None, math.inf
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, math.inf):
                continue
            if d > best_score or d > self.p.yield_max_cost:
                break
            if u != start:
                sc = self._refuge_score(L, u, avoid)
                if sc is not None and d + sc < best_score:
                    best, best_score = u, d + sc
                if not g.nodes[u].transit:
                    continue
            for e in g.out_edges[u]:
                v = e.v
                if v in blocked:
                    continue
                nv = g.nodes[v]
                if not nv.transit and nv.kind not in (YIELD, PARKING):
                    continue
                nd = d + g.edge_cost(e)
                pu = parent.get(u)
                if pu is not None:
                    nd += self.p.turn_time_per_rad * g.turn_angle(pu, u, v)
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    parent[v] = u
                    heapq.heappush(heap, (nd, v))
        if best is None:
            return None
        return g.unwind(parent, start, best)

    def _start_yield(self, L: RobotRecord, to: Set[str], reason: str,
                     note: Optional[str] = None) -> bool:
        path = self._find_refuge(L, to)
        if path is None or len(path) < 2:
            return False
        if note:
            self.event(WARN, 'TRAFFIC', L.id, note)
        prefix = self._prefix(L)
        ctx = L.yield_ctx
        if ctx is None:
            ctx = {'state': L.state, 'route': list(L.route) if L.route else None,
                   'index': L.progress if L.route else 0,
                   'final_yaw': L.final_yaw, 'since': self.now}
            L.yield_ctx = ctx
        ctx['to'] = set(to)
        ctx['arrived'] = None
        ctx['reason'] = reason
        old_refuge = ctx.get('refuge')
        if old_refuge is not None and self.yield_booking.get(old_refuge) == L.id:
            del self.yield_booking[old_refuge]
        refuge = path[-1]
        ctx['refuge'] = refuge
        if self.g.nodes[refuge].kind == YIELD:
            self.yield_booking[refuge] = L.id
        elif self.g.nodes[refuge].kind == PARKING:
            self.parking_booking[self.g.nodes[refuge].group] = L.id
            ctx['parking'] = self.g.nodes[refuge].group
        heading = self.g.nodes[refuge].heading
        self._set_route(L, prefix[:-1] + path, len(prefix), YIELDING,
                        float('nan') if heading is None else heading)
        self.yields += 1
        tos = ', '.join(sorted(to))
        to_rank = min(self.robots[t].rank for t in to)
        self.event(WARN, 'YIELD', L.id,
                   f'{L.id} (P{L.rank}) né đường cho {tos} (P{to_rank}) [{reason}] → '
                   f'tránh vào {self.g.nodes[refuge].id}, sau đó bám lại route cũ')
        return True

    def _release_yields(self) -> None:
        for L in self.robots.values():
            ctx = L.yield_ctx
            if L.state != YIELDING or ctx is None or L.route:
                if L.state == YIELDING and ctx is not None and L.route \
                        and self._stalled_for(L) > 6.0 and self.now - ctx.get('replanned', -1e9) > 6.0:
                    ctx['replanned'] = self.now
                    self._start_yield(L, ctx['to'], ctx.get('reason', 'ưu tiên'))
                continue
            arrived = ctx.get('arrived') or self.now
            ctx['arrived'] = arrived
            if self.now - arrived < 1.0:
                continue
            plan = self._rejoin_plan(L)
            if plan is None:
                if self.now - arrived > self.p.yield_timeout:
                    self._finish_yield(L, None)
                continue
            path, _ = plan
            check = self.traffic.closure(path[:8])
            clear = True
            for hid in ctx['to']:
                H = self.robots.get(hid)
                if H is None or not H.online:
                    continue
                upcoming = set(H.held)
                if H.route:
                    upcoming |= set(H.route[H.progress:H.progress + 15])
                if upcoming & check:
                    clear = False
                    break
            if clear or self.now - arrived > self.p.yield_timeout:
                self._finish_yield(L, plan)

    def _rejoin_plan(self, L: RobotRecord) -> Optional[Tuple[List[int], int]]:
        """Path from the refuge back onto the original route (and the index)."""
        ctx = L.yield_ctx
        orig = ctx.get('route') if ctx else None
        start = L.current_node
        if not orig or start is None:
            return None
        k = min(ctx.get('index', 0), len(orig) - 1)
        goal = orig[-1]
        dist, parent = self.g.dijkstra_from(start, {self.g.nodes[goal].group}, max_cost=90.0)
        suffix = [0.0] * (len(orig) + 1)
        for m in range(len(orig) - 2, -1, -1):
            e = self.g.edge(orig[m], orig[m + 1])
            suffix[m] = suffix[m + 1] + (self.g.edge_cost(e) if e else self.g.distance(orig[m], orig[m + 1]))
        best, best_c = None, math.inf
        for m in range(k, len(orig)):
            n = orig[m]
            if n == start:
                c = suffix[m]
            elif n in dist:
                c = dist[n] + suffix[m]
            else:
                continue
            if c < best_c:
                best, best_c = m, c
        if best is None:
            return None
        if orig[best] == start:
            return [start] + list(orig[best + 1:]), best
        path = self.g.unwind(parent, start, orig[best])
        return path + list(orig[best + 1:]), best

    def _finish_yield(self, L: RobotRecord, plan: Optional[Tuple[List[int], int]]) -> None:
        ctx = L.yield_ctx or {}
        refuge = ctx.get('refuge')
        if refuge is not None and self.yield_booking.get(refuge) == L.id:
            del self.yield_booking[refuge]
        if ctx.get('parking') and self.parking_booking.get(ctx['parking']) == L.id \
                and L.parking != ctx['parking']:
            self.parking_booking[ctx['parking']] = None
        L.yield_ctx = None
        state = ctx.get('state') or IDLE
        if plan is not None and ctx.get('route'):
            route, idx = plan
            self._set_route(L, route, 1, state, ctx.get('final_yaw', float('nan')))
            self.event(INFO, 'YIELD', L.id,
                       f'{L.id} bám lại route cũ tại {self.g.nodes[ctx["route"][idx]].id} '
                       f'→ tiếp tục {STATE_TEXT.get(state, state).lower()}')
            return
        L.state = state if state in WORK_STATES else IDLE
        if state in (TO_PICKUP, TO_DROPOFF) and L.task is not None:
            sid = L.task.pickup if state == TO_PICKUP else L.task.dropoff
            self._navigate(L, self._station_goal(sid), state)
        elif state == TO_CHARGER and L.dock is not None:
            self._navigate(L, self.g.chargers[L.dock], TO_CHARGER)
        else:
            self._go_idle(L)
        self.event(INFO, 'YIELD', L.id, f'{L.id} kết thúc nhường đường, lập lại lộ trình')

    def _reroute(self, r: RobotRecord, force: bool = False) -> bool:
        if not r.route or r.state == YIELDING:
            return False
        goal = r.route[-1]
        blocked: Set[int] = set()
        if r.blocked_by:
            o = self.robots.get(r.blocked_by)
            if o is not None:
                blocked = self.traffic.closure(o.held)
        blocked.discard(goal)
        prefix = self._prefix(r)
        start = prefix[-1]
        blocked.discard(start)
        path = self.g.plan(start, goal, node_penalty=self._penalties(r), blocked=blocked)
        if path is None:
            return False
        old_rem = r.route[r.route.index(start):] if start in r.route else []
        if path == old_rem:
            return False
        old_t = self.g.path_time(old_rem) if old_rem else math.inf
        new_t = self.g.path_time(path)
        if not force and new_t > old_t + 40.0:
            return False
        self._set_route(r, prefix[:-1] + path, len(prefix), r.state, r.final_yaw)
        self.reroutes += 1
        self.event(INFO, 'TRAFFIC', r.id, f'{r.id} đổi lộ trình tránh {r.blocked_by or "ùn tắc"} '
                                          f'({old_t:.0f}s → {new_t:.0f}s)')
        return True

    def _reroute_stalled(self) -> None:
        for r in self.robots.values():
            if not r.route or r.state == YIELDING or r.stall_since is None:
                continue
            if self._stalled_for(r) < self.p.reroute_wait or self.now - r.last_reroute < self.p.reroute_wait:
                continue
            blocker = self.robots.get(r.blocked_by) if r.blocked_by else None
            if blocker is not None and blocker.state in WORK_STATES and r.blocked_node is not None \
                    and self.g.nodes[r.blocked_node].group == self.g.nodes[r.route[-1]].group:
                continue  # just queueing for its own station
            r.last_reroute = self.now
            self._reroute(r)

    # =========================================================== commands
    def _speed_limits(self, route: List[int]) -> List[float]:
        out = []
        for i, n in enumerate(route):
            if i == 0:
                e = None
            else:
                e = self.g.edge(route[i - 1], n)
            if e is not None:
                out.append(e.speed)
            else:
                out.append(0.45 if not self.g.nodes[n].transit else 1.0)
        return out

    def _commands(self) -> Dict[str, Command]:
        cmds = {}
        for r in self.robots.values():
            if not r.route:
                cmds[r.id] = Command(r.id, r.command_id, 0, [], [], [], -1, float('nan'),
                                     r.state == CHARGING)
                continue
            mode = 2 if (self.paused or r.paused) else 1
            cmds[r.id] = Command(
                r.id, r.command_id, mode,
                [self.g.nodes[n].id for n in r.route],
                [self.g.nodes[n].xy for n in r.route],
                self._speed_limits(r.route),
                r.granted, r.final_yaw, False)
        return cmds

    # ================================================================ ETA
    def _eta_route(self, r: RobotRecord) -> Tuple[float, float]:
        if not r.route:
            return 0.0, 0.0
        k = min(r.progress, len(r.route) - 1)
        rem = r.route[k:]
        if len(rem) > 1:
            n1 = self.g.nodes[rem[1]]
            d = math.hypot(n1.x - r.x, n1.y - r.y) + self.g.path_length(rem[1:])
            turns = sum(self.g.turn_angle(a, b, c) for a, b, c in zip(rem, rem[1:], rem[2:]))
        else:
            n0 = self.g.nodes[rem[0]]
            d = math.hypot(n0.x - r.x, n0.y - r.y)
            turns = 0.0
        t = d / self.p.nominal_speed + turns * self.p.turn_time_per_rad
        if r.stall_since is not None:
            t += min(self.now - r.stall_since, 30.0) * 0.5
        return t, d

    def _eta_task(self, r: RobotRecord) -> float:
        t = r.task
        if t is None:
            return self._eta_route(r)[0]
        js = r.job_state
        load, unload = self.production.load_time, self.production.unload_time
        leg = self.g.travel_time(self._station_goal(t.pickup), self._station_goal(t.dropoff))
        if r.state == YIELDING:
            eta_r = 10.0 + (self._eta_route(r)[0] if r.route else 5.0)
            orig = (r.yield_ctx or {}).get('route') or []
            if orig:
                eta_r += self.g.path_time(orig[(r.yield_ctx or {}).get('index', 0):]) / 0.8
        else:
            eta_r = self._eta_route(r)[0]
        if js == TO_PICKUP:
            return eta_r + load + leg + unload
        if js == LOADING:
            return max(r.work_until - self.now, 0.0) + leg + unload
        if js == TO_DROPOFF:
            return eta_r + unload
        if js == UNLOADING:
            return max(r.work_until - self.now, 0.0)
        return eta_r

    # =========================================================== snapshot
    def _goal_name(self, r: RobotRecord) -> str:
        if not r.route:
            return self.g.nodes[r.current_node].id if r.current_node is not None else ''
        n = self.g.nodes[r.route[-1]]
        return n.group or n.id

    def _detail(self, r: RobotRecord) -> str:
        txt = STATE_TEXT.get(r.state, r.state)
        if r.state == YIELDING and r.yield_ctx:
            tos = ', '.join(sorted(r.yield_ctx.get('to', [])))
            where = self.g.nodes[r.yield_ctx['refuge']].id if r.yield_ctx.get('refuge') is not None else ''
            phase = 'đang chờ tại' if not r.route else 'đang tránh vào'
            txt = f'Nhường đường cho {tos} ({phase} {where}), sau đó quay lại route cũ'
        elif r.task and r.state in TASK_STATES:
            if r.state == TO_PICKUP:
                txt = f'Đi tới {r.task.pickup} lấy "{r.task.cargo}"'
            elif r.state == TO_DROPOFF:
                txt = f'Chở "{r.task.cargo}" tới {r.task.dropoff}'
            elif r.state == LOADING:
                txt = f'Đang nhận hàng tại {r.task.pickup} ({max(r.work_until - self.now, 0):.0f}s)'
            elif r.state == UNLOADING:
                txt = f'Đang trả hàng tại {r.task.dropoff} ({max(r.work_until - self.now, 0):.0f}s)'
        elif r.state == CHARGING:
            txt = f'Đang sạc tại {r.dock} ({r.battery:.0f}%)'
        if r.stall_since is not None and r.blocked_by:
            node = self.g.nodes[r.blocked_node].id if r.blocked_node is not None else ''
            txt += f' | chờ {r.blocked_by} nhường {node} ({self.now - r.stall_since:.0f}s)'
        if r.paused:
            txt = 'TẠM DỪNG | ' + txt
        return txt

    def _next_text(self, r: RobotRecord) -> Tuple[str, str, float]:
        if r.next_task is not None:
            nt = r.next_task
            eta = self._eta_task(r) + self.g.travel_time(
                self._station_goal(r.task.dropoff) if r.task else self._anchor(r),
                self._station_goal(nt.pickup))
            return nt.id, f'{nt.id}: {nt.description}', eta
        if r.task is not None:
            if r.battery - 6.0 < self.p.battery_low:
                return '', 'Sau task: đi sạc (pin sắp thấp)', self._eta_task(r)
            return '', 'Sau task: nhận task mới hoặc về bãi đỗ', self._eta_task(r)
        if r.state in (TO_CHARGER, CHARGING):
            return '', f'Sạc tới {self.p.battery_full:.0f}% rồi nhận task', 0.0
        if r.state == WAIT_CHARGER:
            pos = self.charge_queue.index(r.id) + 1 if r.id in self.charge_queue else 0
            return '', f'Chờ dock sạc (vị trí {pos} trong hàng đợi)', 0.0
        return '', 'Sẵn sàng nhận task', 0.0

    def snapshot(self) -> dict:
        robots = []
        for r in sorted(self.robots.values(), key=lambda x: x.id):
            eta_goal, dist_rem = self._eta_route(r)
            k = min(r.progress, len(r.route) - 1) if r.route else 0
            rem = r.route[k:] if r.route else []
            nid, ntext, neta = self._next_text(r)
            t = r.task
            robots.append({
                'robot_id': r.id, 'state': r.state, 'state_detail': self._detail(r),
                'priority_rank': r.rank, 'priority_score': round(r.priority, 1),
                'base_rank': r.base_rank,
                'pose': {'x': r.x, 'y': r.y, 'theta': r.yaw}, 'speed': r.v,
                'battery_percent': r.battery, 'charging': r.charging, 'cargo': r.cargo,
                'task_id': t.id if t else '', 'task_type': t.type if t else '',
                'task_description': t.description if t else '',
                'pickup': t.pickup if t else '', 'dropoff': t.dropoff if t else '',
                'current_goal': self._goal_name(r),
                'eta_goal_s': eta_goal, 'eta_task_s': self._eta_task(r) if t else eta_goal,
                'distance_remaining_m': dist_rem,
                'route_node_ids': [self.g.nodes[n].id for n in rem],
                'route_points': [self.g.nodes[n].xy for n in rem],
                'granted_count': max(0, r.granted - k + 1) if r.route else 0,
                'route_summary': self.g.route_summary(rem) if rem else '',
                'next_task_id': nid, 'next_task_description': ntext,
                'next_task_eta_start_s': neta,
                'blocked_by': r.blocked_by if r.stall_since is not None else '',
                'yielding_to': ', '.join(sorted(r.yield_ctx['to'])) if r.yield_ctx else '',
                'waiting_time_s': r.waiting(self.now),
                'tasks_completed': r.tasks_done, 'odometer_m': r.odometer,
                'online': r.online, 'paused': r.paused,
                'held_nodes': [self.g.nodes[n].id for n in r.held],
            })
        tasks = []
        for t in sorted(self.tasks.active(), key=lambda x: (-x.priority, x.created)):
            eta = 0.0
            if t.robot and t.robot in self.robots:
                eta = self._eta_task(self.robots[t.robot])
            tasks.append({'task_id': t.id, 'task_type': t.type, 'description': t.description,
                          'priority': t.priority, 'status': t.status if not t.next_for else 'NEXT',
                          'pickup': t.pickup, 'dropoff': t.dropoff,
                          'robot_id': t.robot or '', 'next_for_robot': t.next_for or '',
                          'age_s': self.now - t.created, 'eta_s': eta})
        return {
            'paused': self.paused, 'sim_time_s': self.elapsed, 'robots': robots, 'tasks': tasks,
            'charger_ids': list(self.dock_booking),
            'charger_occupants': [o or '' for o in self.dock_booking.values()],
            'charger_queue': list(self.charge_queue),
            'tasks_completed': self.tasks.completed,
            'yield_maneuvers': self.yields, 'deadlocks_resolved': self.deadlocks,
            'reroutes': self.reroutes,
        }
