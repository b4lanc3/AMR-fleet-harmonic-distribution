"""Fleet manager core: task allocation, charging, priorities and traffic.

`FleetCore` is deliberately free of ROS so that it can be unit tested and
run in the headless simulator.  The ROS node only converts messages.

Robot life cycle::

    PARKED/IDLE --assign--> TO_PICKUP -> LOADING -> TO_DROPOFF -> UNLOADING
         ^                                                          |
         +------ TO_PARK <---- (no task) <---- next task? <---------+
                 TO_CHARGER -> CHARGING   (battery low / opportunistic)
                 WAIT_CHARGER (all 5 docks busy -> wait on a parking bay)

    Two robots meeting head-on on a two-way lane pass each other without
    stopping: once they are `pass_detect` metres apart, both shift
    `pass_offset` to their right along their own route, drive past and come
    back to the lane centre (smooth, no yield bay).

    any moving state --(deadlock / blocked by a stopped lower priority robot)
          --> YIELDING (drive off the other robot's path, wait until it has
          passed, then rejoin the *original* route and resume) - fallback only
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Set, Tuple

from .assign import hungarian
from .graph import CHARGER, LANE, PARKING, YIELD, TrafficGraph
from .layout import LINE_CELLS, build_graph
from .production import Production
from .tasks import (IN_TRANSIT, LOADING as T_LOADING, MANUAL, QUEUED,
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
DEMO = 'DEMO'

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
    DEMO: 'Kịch bản demo đối đầu',
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
    max_away_share: float = 0.4        # opportunistic charging: at most this share of the
                                       # fleet away charging at the same time (staggered)
    charge_release: float = 60.0       # opportunistic charger may leave for a task
    charge_release_low: float = 80.0   # robot that came in low may leave for a task
    assign_period: float = 1.0
    yield_wait: float = 3.0            # higher priority robot stalled this long -> ask lower to yield
    cycle_wait: float = 1.5
    reroute_wait: float = 20.0
    yield_timeout: float = 45.0
    yield_max_cost: float = 45.0
    status_timeout: float = 5.0
    nominal_speed: float = 1.2
    turn_time_per_rad: float = 0.8
    priority_mode: str = 'task'        # task | dynamic | static
    w_task: float = 1000.0
    w_cargo: float = 400.0
    w_charge: float = 600.0
    w_charge_critical: float = 3500.0
    w_base: float = 15.0
    w_aging: float = 4.0
    aging_cap: float = 160.0
    lookahead: float = 7.0
    pass_detect: float = 8.0           # m: robots meeting head-on start shifting to their right
    pass_after: float = 2.5            # m: stay shifted until this far past the other robot
    pass_speed: float = 1.1            # m/s while shifted (lower priority robot: 0.8 of it)
    # experimental: blocked by a moving robot -> arrive when it has cleared.
    # Measured (tune_sim, 5-12 robots): slightly fewer stops but about twice
    # the hard braking and less throughput with 10 robots -> off by default
    advisory: bool = False
    advisory_margin: float = 0.3       # s after the other robot has cleared
    advisory_min: float = 0.35         # m/s
    production: bool = True            # False: no MES tasks, only manual tasks / demos
    dispatch: str = 'batch'            # batch: optimal robot <-> task matching with line
                                       # deadlines | greedy: nearest robot, task by task
    # cost per second the served line would stand still before the goods arrive
    # (production.deadline).  Measured with tune_sim: any weight > 0 lowered
    # the throughput with 4-5 robots (the line buffers absorb it, shorter
    # total travel wins) -> 0: optimal matching on travel time + task priority
    w_late: float = 0.0
    w_priority: float = 60.0           # s of travel a task priority level is worth
    station_handover: bool = False     # Gazebo: the station arms end loading / unloading
    handover_timeout: float = 30.0     # s: give up waiting for the arm


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
    carrying: bool = False
    shape: tuple = ()            # route indices shifted for passing (re-send when it changes)

    def key(self) -> tuple:
        return (self.command_id, self.mode, self.granted_index, self.charge, self.carrying,
                self.shape)


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
    passing_with: Optional[str] = None
    handover_wait: bool = False        # loading / unloading ends when the station arm says so
    advisory: Optional[float] = None   # m/s: time the arrival at a busy node instead of stopping
    # priority
    priority: float = 0.0
    priority_static: float = 0.0
    rank: int = 0
    # stats
    tasks_done: int = 0
    odometer: float = 0.0
    moving: bool = False

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
        self.production = Production(layout.get('process', {}) or {}, layout.get('lines'))
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
        self.passes = 0
        # stations served by a transfer arm (line inputs / outputs, Gazebo)
        self.arm_stations = {st['id'] for st in layout.get('stations', []) or []
                             if st.get('object') in LINE_CELLS}
        self.unplanned_stops = 0
        self.passing: Dict[frozenset, dict] = {}
        self.deadlocks = 0
        self.reroutes = 0
        self.physical_conflicts = 0
        self._agents: Dict[str, TrafficAgent] = {}
        self.demo: Optional[dict] = None

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
        # smoothness KPI: the robot stopped on its way (not at the end of its route)
        if abs(st.v) > 0.3:
            r.moving = True
        elif abs(st.v) < 0.05:
            if r.moving and r.route and not r.route_done and r.progress < len(r.route) - 1:
                self.unplanned_stops += 1
            r.moving = False
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
        if not self.paused and self.p.production:
            self.production.set_fleet_size(sum(r.online for r in self.robots.values()), now)
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
        self._run_demo()
        if now - self._last_assign >= self.p.assign_period and not self.paused:
            self._last_assign = now
            self._assign()
        self._priorities()
        self._update_passing()
        self._run_traffic()
        self._speed_advisories()
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
        for n, owners in self.traffic.owners.items():
            if owners - {r.id}:
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
            r.work_until = self.now + self._work_time(r, r.task.pickup if r.task else '',
                                                      self.production.load_time)
            if r.task:
                r.task.status = T_LOADING
        elif st == TO_DROPOFF:
            r.state = UNLOADING
            r.work_until = self.now + self._work_time(r, r.task.dropoff if r.task else '',
                                                      self.production.unload_time)
            if r.task:
                r.task.status = T_UNLOADING
        elif st == TO_CHARGER:
            r.state = CHARGING
            self.event(INFO, 'CHARGE', r.id, f'{r.id} vào dock {r.dock}, bắt đầu sạc ({r.battery:.0f}%)')
        elif st == TO_PARK:
            r.state = PARKED
        elif st in (WAIT_CHARGER, DEMO):
            pass
        else:
            r.state = IDLE

    def _work_time(self, r: RobotRecord, station: str, nominal: float) -> float:
        """Loading / unloading time: fixed, or until the station arm hands over."""
        r.handover_wait = self.p.station_handover and station in self.arm_stations
        return self.p.handover_timeout if r.handover_wait else nominal

    def station_handover(self, station: str, robot_id: str) -> bool:
        """The transfer arm of `station` has taken the load off / put it on the AMR."""
        r = self.robots.get(robot_id)
        if r is None or r.task is None or not r.handover_wait:
            return False
        if (r.state == LOADING and r.task.pickup == station) or \
                (r.state == UNLOADING and r.task.dropoff == station):
            r.handover_wait = False
            r.work_until = self.now
            what = 'đặt hàng lên' if r.state == LOADING else 'nhấc hàng khỏi'
            self.event(INFO, 'STATION', r.id, f'Tay máy {station} đã {what} {r.id}')
            return True
        return False

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
        if allow_charge and r.battery < self.p.battery_opportunistic and self._can_top_up():
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

    def _can_top_up(self) -> bool:
        """Opportunistic charging is staggered: only while the fleet has spare
        robots (fewer queued tasks than free robots) and only a share of it
        away charging at the same time."""
        robots = [r for r in self.robots.values() if r.online]
        away = sum(r.state in (TO_CHARGER, CHARGING, WAIT_CHARGER) for r in robots)
        if away + 1 > max(1, math.ceil(self.p.max_away_share * len(robots))):
            return False
        waiting = sum(1 for t in self.tasks.open_tasks() if t.status == QUEUED)
        free = sum(1 for r in robots if self._available(r))
        return waiting < free

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
        if self.p.dispatch == 'batch':
            open_tasks = self._assign_batch(open_tasks)
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

    def _assign_batch(self, open_tasks: List[Task]) -> List[Task]:
        """Match the free robots and the open tasks all at once (Hungarian):
        minimum total of (travel to the pick-up + w_late * seconds the served
        line would stand still before the goods arrive).  A busy robot that
        will be done soon may still take a task as its next task: the tasks it
        would serve better are returned for the task-by-task pass."""
        robots = [r for r in self.robots.values() if self._available(r)]
        tasks = [t for t in open_tasks
                 if self._station_free(t.pickup) and self._station_free(t.dropoff)]
        if not robots or not tasks:
            return open_tasks
        prod = self.production
        deadline = {t.id: prod.deadline(t) for t in tasks}
        leg = {t.id: self.g.travel_time(self._station_goal(t.pickup), self._station_goal(t.dropoff))
               for t in tasks}
        cost = []
        for r in robots:
            row = []
            a = self._anchor(r)
            for t in tasks:
                eta = self.g.travel_time(a, self._station_goal(t.pickup))
                if eta == math.inf:
                    row.append(math.inf)
                    continue
                if r.state == CHARGING:
                    eta += 15.0
                arrive = eta + prod.load_time + leg[t.id]
                late = max(0.0, arrive - deadline[t.id])
                row.append(eta + self.p.w_late * late - self.p.w_priority * t.priority)
            cost.append(row)
        # the cheapest pairs: a robot about to finish may serve a task sooner
        rest = []
        matched = set()
        for i, j in hungarian(cost):
            t, r = tasks[j], robots[i]
            matched.add(t.id)
            eta = cost[i][j] + self.p.w_priority * t.priority
            if self._busy_eta(t) + 3.0 < eta:
                rest.append(t)
                continue
            self._start_task(r, t)
        return rest + [t for t in open_tasks if t.id not in matched]

    def _busy_eta(self, t: Task) -> float:
        target = self._station_goal(t.pickup)
        best = math.inf
        for r in self.robots.values():
            if (not r.online or r.paused or r.next_task is not None or r.task is None
                    or r.job_state not in (TO_DROPOFF, UNLOADING)
                    or r.battery < self.p.battery_low + 8.0):
                continue
            best = min(best, self._eta_task(r)
                       + self.g.travel_time(self._station_goal(r.task.dropoff), target))
        return best

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

    # ============================================================== demo
    def start_demo_head_on(self, lane: str = 'link') -> Tuple[bool, str]:
        """Send two idle robots to both ends of a two-way lane, then let them
        drive towards each other at the same time: they detect each other,
        both shift to their right, pass and keep their route without stopping."""
        if self.demo is not None:
            return False, 'Kịch bản demo đang chạy'
        bays = sorted((i for i in self.g.yield_bays if self.g.nodes[i].id.startswith(f'Y.{lane}.')),
                      key=lambda i: (self.g.nodes[i].x, self.g.nodes[i].y))
        if len(bays) < 3:
            return False, f'Làn {lane} không có đủ yield bay'
        ends = self._demo_ends(lane, bays)
        if ends is None:
            return False, f'Làn {lane} không có đoạn đủ dài để hai xe tránh nhau'
        free = [r for r in self.robots.values() if self._available(r) and r.state != CHARGING]
        if len(free) < 2:
            return False, 'Cần ít nhất 2 robot rảnh (đang đỗ) để chạy demo'
        picked = []
        for end in ends:
            r = min((x for x in free if x not in picked),
                    key=lambda x: self.g.travel_time(self._anchor(x), end))
            picked.append(r)
        for r, end in zip(picked, ends):
            self.yield_booking[end] = r.id
            self._navigate(r, end, DEMO, final_yaw=self.g.nodes[end].heading)
        self.demo = {'robots': [r.id for r in picked], 'ends': ends, 'phase': 'staging',
                     'since': self.now}
        self.event(WARN, 'YIELD', '', f'Demo đối đầu: {picked[0].id} và {picked[1].id} ra hai đầu '
                                      f'làn 2 chiều "{lane}" rồi chạy ngược chiều nhau')
        return True, f'{picked[0].id}, {picked[1].id}'

    def _demo_ends(self, lane: str, bays: List[int], half: float = 5.0) -> Optional[List[int]]:
        """The two yield bays farthest apart whose midpoint (where the robots
        meet) lies on a stretch of passing nodes: no intersection within
        `half` metres, where the robots could not shift aside."""
        lanes = getattr(self.g, 'lanes', {}) or {}
        ln = lanes.get(lane)
        if ln is None:
            return [bays[0], bays[-1]]
        nodes = [n for n in self.g.nodes if n.kind == LANE and n.lanes == [lane]]

        def along(n) -> float:
            return ln.project((n.x, n.y))

        pos = sorted((along(n), n.idx in self.g.pass_nodes) for n in nodes)
        gap = float((self.layout.get('traffic', {}) or {}).get('node_spacing', 2.0)) + 0.5

        def passable(t: float) -> bool:
            near = [p for p in pos if abs(p[0] - t) <= half]
            if not near or not all(ok for _, ok in near):
                return False
            ts = [t - half] + [p[0] for p in near] + [t + half]
            return all(b - a <= gap for a, b in zip(ts, ts[1:]))

        best, best_d = None, 0.0
        for i, a in enumerate(bays):
            for b in bays[i + 1:]:
                ta, tb = along(self.g.nodes[a]), along(self.g.nodes[b])
                if abs(tb - ta) > best_d and passable((ta + tb) / 2):
                    best, best_d = [a, b] if ta < tb else [b, a], abs(tb - ta)
        return best

    def _run_demo(self) -> None:
        d = self.demo
        if d is None:
            return
        robots = [self.robots[i] for i in d['robots']]
        if any(r.state not in (DEMO, YIELDING) or not r.online for r in robots) \
                or self.now - d['since'] > 900:
            self._end_demo('kịch bản bị gián đoạn')
            return
        idle = all(not r.route and r.state == DEMO for r in robots)
        if d['phase'] == 'staging' and idle:
            d['phase'] = 'swap'
            for r, goal in zip(robots, reversed(d['ends'])):
                self.yield_booking[goal] = r.id
                path = self.g.plan(r.current_node, goal)
                if path is not None:
                    self._set_route(r, path, 1, DEMO, self.g.nodes[goal].heading or float('nan'))
            hi, lo = sorted(robots, key=lambda x: -x.priority_static)
            self.event(WARN, 'YIELD', '', f'Demo: {hi.id} (P{hi.rank}) và {lo.id} (P{lo.rank}) chạy '
                                          f'ngược chiều trên cùng một làn → hai xe lệch phải tránh nhau, không dừng')
        elif d['phase'] == 'swap' and idle:
            self._end_demo('hoàn tất')

    def _end_demo(self, why: str) -> None:
        d = self.demo
        self.demo = None
        if d is None:
            return
        for end in d['ends']:
            self.yield_booking.pop(end, None)
        for rid in d['robots']:
            r = self.robots[rid]
            if r.state == DEMO and not r.route:
                r.state = IDLE
                self._go_idle(r)
        self.event(INFO, 'YIELD', '', f'Demo đối đầu {why}')

    # =========================================================== priority
    def _priorities(self) -> None:
        p = self.p
        N = len(self.robots)
        # MES tasks follow the state of the line they serve
        for t in self.tasks.active():
            if t.type != MANUAL:
                t.priority = self.production.urgency(t)
        if p.priority_mode == 'task':
            self._task_priorities()
            return
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
        # the displayed rank is the one used to decide who gives way; the waiting
        # time bonus only re-orders grants at intersections (anti-starvation)
        for i, r in enumerate(sorted(self.robots.values(),
                                     key=lambda x: (-x.priority_static, x.base_rank))):
            r.rank = i + 1

    def _task_priorities(self) -> None:
        """Priority follows the task: level of the task (URGENT > HIGH > NORMAL
        > LOW), then robots already carrying the goods of their task, then the
        older task.  Robots without a task come last, except a robot that must
        reach a charger with a critical battery.  The robot id only breaks ties."""
        p = self.p
        N = len(self.robots)
        for r in self.robots.values():
            js = r.job_state
            t = r.task
            s = 0.0
            if t is not None and js in TASK_STATES:
                s = 1000.0 * (t.priority + 1)
                if r.cargo:
                    s += 400.0
                s += min(self.now - t.created, 600.0) * 0.5
            elif js in (TO_CHARGER, WAIT_CHARGER):
                s = 3500.0 if r.battery < p.battery_critical else 300.0
            elif js == DEMO:
                s = 1000.0
            s += (N + 1 - r.base_rank) * 0.01   # deterministic tie-break only
            r.priority_static = s
            aging = min(r.waiting(self.now) * p.w_aging, p.aging_cap) if r.stall_since else 0.0
            r.priority = s + aging
        for i, r in enumerate(sorted(self.robots.values(),
                                     key=lambda x: (-x.priority_static, x.base_rank))):
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

    # ============================================================ passing
    def _lane_tracks(self, r: RobotRecord) -> Dict[str, Tuple[int, float, float]]:
        """Two-way lanes on the next metres of r's route: lane -> (direction along
        the lane axis, axis coordinate of the robot, farthest coordinate reached
        on the lane; +-inf when the route is still on the lane at the end of the
        look-ahead)."""
        g = self.g
        out: Dict[str, Tuple[int, float, float]] = {}
        if not r.route:
            return out
        k = min(max(r.progress, 0), len(r.route) - 1)
        dist = math.dist((r.x, r.y), g.nodes[r.route[k]].xy)
        last_lane = None                      # lane of the last edge looked at
        for i in range(k, len(r.route) - 1):
            u, v = r.route[i], r.route[i + 1]
            if dist > 2.0 * self.p.lookahead + 8.0:
                if last_lane in out:          # beyond the look-ahead, still on it
                    d, s0, _ = out[last_lane]
                    out[last_lane] = (d, s0, d * math.inf)
                break
            dist += g.distance(u, v)
            lane = g.pass_lane.get(u)
            last_lane = lane if lane is not None and g.pass_lane.get(v) == lane else None
            if last_lane is None:
                continue
            ax, ay = g.pass_axis[u]
            nu, nv = g.nodes[u], g.nodes[v]
            d = 1 if (nv.x - nu.x) * ax + (nv.y - nu.y) * ay > 0 else -1
            far = nv.x * ax + nv.y * ay
            if lane in out:
                if out[lane][0] != d:
                    continue          # the route turns back on this lane: keep the first stretch
                out[lane] = (d, out[lane][1], far)
            else:
                out[lane] = (d, r.x * ax + r.y * ay, far)
        return out

    def _update_passing(self) -> None:
        """Pair the robots that meet head-on on a two-way lane: they pass each
        other, both shifted to their right, instead of stopping and yielding."""
        p = self.p
        live = [r for r in self.robots.values()
                if r.online and r.route and not r.paused and r.state not in WORK_STATES]
        tracks = {r.id: self._lane_tracks(r) for r in live}
        old = getattr(self, 'passing', {})
        new: Dict[frozenset, dict] = {}
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                ta, tb = tracks[a.id], tracks[b.id]
                for lane in ta.keys() & tb.keys():
                    da, sa, fa = ta[lane]
                    db, sb, fb = tb[lane]
                    if da != -db:
                        continue
                    # both must really drive that way now: a robot leaving a lay-by
                    # or about to turn back on the lane is not passing anybody
                    ax, ay = self.g.lanes[lane].dir
                    if (math.cos(a.yaw) * ax + math.sin(a.yaw) * ay) * da < 0.7 or \
                            (math.cos(b.yaw) * ax + math.sin(b.yaw) * ay) * db < 0.7:
                        continue
                    gap = (sb - sa) * da              # b ahead of a, along a's direction
                    key = frozenset((a.id, b.id))
                    prev = old.get(key)
                    # both stay on the lane until they are past each other (a robot
                    # turning off before that crosses the other one's shifted path)
                    meet = (fa - sa) * da >= gap + p.pass_after and \
                        (fb - sb) * db >= gap + p.pass_after
                    if gap < -p.pass_after or not meet or (prev is None and gap < 0.5):
                        continue          # passed each other (a new pair must still meet)
                    # paired early enough that their traffic grants never collide
                    # (no braking); the visible shift only starts at pass_detect
                    pair_range = 2.0 * p.lookahead + 2.0
                    if gap > pair_range + (2.0 if prev else 0.0):
                        continue
                    if gap < 3.5 and not (self._shifting(a, lane) and self._shifting(b, lane)):
                        continue          # one of them stands still on the lane centre
                    shift = gap <= p.pass_detect or bool(prev and prev['shift'])
                    new[key] = {'lane': lane, 'shift': shift, 'dirs': {a.id: da, b.id: db},
                                'since': prev['since'] if prev else self.now}
                    if shift and not (prev and prev['shift']):
                        self.passes += 1
                        name = self.g.lane_names.get(lane, lane)
                        self.event(INFO, 'PASS', a.id,
                                   f'{a.id} và {b.id} gặp nhau trên {name} (cách {gap:.1f} m): '
                                   f'cùng lệch phải để tránh, không dừng, giữ route')
                    break
        # a pair that has started shifting stays a pair until the robots are
        # really past each other, even if one route leaves the lane meanwhile
        # (turning at the next intersection): never back to the centre line
        # while the other robot is still alongside
        for key, info in old.items():
            if key in new or not info.get('shift'):
                continue
            x, y = sorted(key)
            rx, ry = self.robots[x], self.robots[y]
            if not (rx.online and ry.online):
                continue
            ax, ay = self.g.lanes[info['lane']].dir
            gap = ((ry.x - rx.x) * ax + (ry.y - rx.y) * ay) * info['dirs'][x]
            if gap > -p.pass_after:
                new[key] = info
        self.passing = new
        for r in self.robots.values():
            r.passing_with = None
        for key, info in new.items():
            if not info['shift']:
                continue
            x, y = sorted(key)
            self.robots[x].passing_with = y
            self.robots[y].passing_with = x
        self.traffic.passing = {k: bool(v['shift']) for k, v in new.items()}

    # ========================================================== advisory
    def _speed_advisories(self) -> None:
        """Space-time arrival instead of stop-and-go: a robot whose grant ends
        because a *moving* robot uses the nodes ahead (crossing at an
        intersection, merging, a robot ahead on the same lane) gets a speed that
        brings it to its stop point just after the other robot has cleared them.
        The grant is extended meanwhile, so it never has to stop."""
        p, g = self.p, self.g
        for r in self.robots.values():
            prev, r.advisory = r.advisory, None
            if not p.advisory or not r.route or r.blocked_by is None or r.blocked_node is None:
                continue
            # hysteresis: an advisory stays while the robot is still held up
            # (on / off flicker would mean accelerating and braking again)
            r.advisory = prev
            b = self.robots.get(r.blocked_by)
            # only a reliable prediction: b drives steadily and may leave the zone
            if b is None or not b.route or abs(b.v) < 0.5 or b.paused or b.stall_since is not None:
                continue
            zone = self.traffic.closure([r.blocked_node])
            # time for b to leave the zone along its own route
            k = min(max(b.progress, 0), len(b.route) - 1)
            dist = math.dist((b.x, b.y), g.nodes[b.route[k]].xy)
            inside = False
            clear = None
            for i in range(k, len(b.route)):
                n = b.route[i]
                if i > k:
                    dist += g.distance(b.route[i - 1], n)
                if n in zone:
                    inside = True
                elif inside or i - k > 6:
                    clear = dist if inside and b.granted >= i else None
                    break
            if clear is None:
                continue                      # b stays in the way: plain stop
            t_clear = clear / max(abs(b.v), 0.3) + p.advisory_margin
            # distance of r to the end of its grant
            kr = min(max(r.progress, 0), len(r.route) - 1)
            d = math.dist((r.x, r.y), g.nodes[r.route[kr]].xy)
            for i in range(kr + 1, min(r.granted, len(r.route) - 1) + 1):
                d += g.distance(r.route[i - 1], r.route[i])
            if d < 0.5:
                continue
            v = d / t_clear
            if v < abs(r.v) + 0.05 or prev is not None:
                r.advisory = min(self.g.lane_speed, max(p.advisory_min, round(v, 1)))

    def _shifting(self, r: RobotRecord, lane: str) -> bool:
        """r moves, or already stands shifted off the centre line of lane."""
        if abs(r.v) > 0.05:
            return True
        n = self.g.nearest_node(r.x, r.y)
        if self.g.pass_lane.get(n) != lane:
            return True
        ax, ay = self.g.pass_axis[n]
        node = self.g.nodes[n]
        return abs((r.x - node.x) * ay - (r.y - node.y) * ax) > 0.3

    def _pass_shape(self, r: RobotRecord) -> Tuple[List[Tuple[float, float]], List[float], tuple]:
        """Waypoints of r shifted to its right where it passes another robot,
        the matching speed limits and a signature of the shift."""
        g, p = self.g, self.p
        pts = [g.nodes[n].xy for n in r.route]
        limits = self._speed_limits(r.route)
        adv: tuple = ()
        if r.advisory is not None:
            # from where the robot can slow down gently: the follower / driver
            # speed profiles plan the deceleration to it
            k0 = min(max(r.progress, 0), len(r.route) - 1)
            need = max(0.0, (r.v * r.v - r.advisory ** 2) / (2.0 * 0.6))
            d = math.dist((r.x, r.y), g.nodes[r.route[k0]].xy)
            i = k0 + 1
            while i < len(limits) and d < need:
                d += g.distance(r.route[i - 1], r.route[i])
                i += 1
            for j in range(i, len(limits)):
                limits[j] = min(limits[j], r.advisory)
            adv = (('adv', r.advisory),)
        if r.passing_with is None:
            return pts, limits, adv
        o = self.robots[r.passing_with]
        info = self.passing.get(frozenset((r.id, o.id)))
        if info is None or not info['shift']:
            return pts, limits, adv
        lane = info['lane']
        k = min(max(r.progress, 0), len(r.route) - 1)
        speed = p.pass_speed * (0.8 if r.priority_static < o.priority_static else 1.0)
        sig = []
        for i in range(max(k, 1), len(r.route) - 1):
            n = r.route[i]
            if g.pass_lane.get(n) != lane:
                if sig:
                    break
                continue
            prv, nxt = g.nodes[r.route[i - 1]], g.nodes[r.route[i + 1]]
            ax, ay = g.pass_axis[n]
            d = 1 if (nxt.x - prv.x) * ax + (nxt.y - prv.y) * ay > 0 else -1
            node = g.nodes[n]
            s_n = node.x * ax + node.y * ay
            if (s_n - (r.x * ax + r.y * ay)) * d < -0.5:
                continue                                  # already behind the robot
            if (s_n - (o.x * ax + o.y * ay)) * d > p.pass_after:
                break                                     # past the other robot: back to the centre
            tx, ty = d * ax, d * ay
            off = g.pass_offset
            pts[i] = (node.x + off * ty, node.y - off * tx)
            limits[i] = min(limits[i], speed)
            if i + 1 < len(limits):
                limits[i + 1] = min(limits[i + 1], speed)
            sig.append(i)
        return pts, limits, tuple(sig) + adv

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
                # same rule as the grant: the node and everything the edge sweeps
                if self.traffic.free_for(L.id, {v} | g.sweep(u, v)) is not None:
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
                    ctx['attempts'] = ctx.get('attempts', 0) + 1
                    if ctx['attempts'] >= 2 or not self._start_yield(
                            L, ctx['to'], ctx.get('reason', 'ưu tiên')):
                        self._swap_yield(L)
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

    def _swap_yield(self, L: RobotRecord) -> None:
        """L cannot get out of the way: let it resume and ask the robot(s) it
        was yielding to (normally higher priority) to back off instead."""
        ctx = L.yield_ctx or {}
        to = set(ctx.get('to', set()))
        self._finish_yield(L, None)   # re-plans L towards its original goal
        for hid in sorted(to, key=lambda i: self.robots[i].priority_static):
            H = self.robots.get(hid)
            if H is None or H.state in WORK_STATES or H.yield_ctx is not None:
                continue
            if self._start_yield(H, {L.id}, reason='robot kia không thể tránh',
                                 note=f'{L.id} không có chỗ tránh → {H.id} lùi lại nhường'):
                self.deadlocks += 1
                return

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
                out.append(self.g.lane_speed if self.g.nodes[n].transit else self.g.bay_speed)
        return out

    def _commands(self) -> Dict[str, Command]:
        cmds = {}
        for r in self.robots.values():
            if not r.route:
                cmds[r.id] = Command(r.id, r.command_id, 0, [], [], [], -1, float('nan'),
                                     r.state == CHARGING, bool(r.cargo))
                continue
            mode = 2 if (self.paused or r.paused) else 1
            pts, limits, shape = self._pass_shape(r)
            cmds[r.id] = Command(
                r.id, r.command_id, mode,
                [self.g.nodes[n].id for n in r.route],
                pts, limits, r.granted, r.final_yaw, False, bool(r.cargo), shape)
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
                txt = f'Đang nhận hàng tại {r.task.pickup} ' + \
                      ('(tay máy đang đặt hàng)' if r.handover_wait
                       else f'({max(r.work_until - self.now, 0):.0f}s)')
            elif r.state == UNLOADING:
                txt = f'Đang trả hàng tại {r.task.dropoff} ' + \
                      ('(tay máy đang nhấc hàng)' if r.handover_wait
                       else f'({max(r.work_until - self.now, 0):.0f}s)')
        elif r.state == CHARGING:
            txt = f'Đang sạc tại {r.dock} ({r.battery:.0f}%)'
        if r.passing_with:
            txt += f' | tránh {r.passing_with} (lệch phải, không dừng)'
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
            'yield_maneuvers': self.yields, 'pass_maneuvers': self.passes,
            'unplanned_stops': self.unplanned_stops,
            'fleet_km': sum(r.odometer for r in self.robots.values()) / 1000.0, 'deadlocks_resolved': self.deadlocks,
            'reroutes': self.reroutes,
        }
