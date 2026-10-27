"""Simulated MES: production lines, finished goods warehouse, packaging
lines and shipping gates.

Material flow (one carton per AMR trip):

  A_i (carton with a kit of components) --KIT_DELIVERY-->  B_k input pedestal
  B_k starts one product every cycle_time (takt) from the open kit carton: the
      loader arm puts the component on the belt, the line machines it, the
      loader arm at the end packs the product into the carton on the output
      pedestal (lead time = loader + belt + drop); full carton -> output buffer
  B_k output (carton)     --FG_STORE-->  D inbound station (warehouse stock)
  D outbound station      --FG_PICK--->  C_m input pedestal
  C_m wraps every product of the carton the same way (pack_time takt) into a
      shipping carton; full carton -> output buffer
  C_m output (carton)     --SHIPMENT-->  G_j shipping gate (truck)

The warehouse decouples production and packaging: it starts with stock (the
packaging lines run from t = 0), a full warehouse holds the production lines
back, an empty one the packaging lines.  A truck leaves its gate when it is
full and the next one docks.  The model raises task requests when buffers
need service and escalates the task priority when a line is about to starve
or block.  With active_lines: auto only as many lines run as the fleet can
serve (west lines first).
"""

from __future__ import annotations

import math

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .layout import line_lead_time
from .tasks import FG_PICK, FG_STORE, KIT_DELIVERY, SHIPMENT, Task

RUNNING = 'RUNNING'
STARVED = 'STARVED'
BLOCKED = 'BLOCKED'
IDLE = 'IDLE'

AUTO_DOWN_DELAY = 20.0      # s the fleet must stay smaller before lines are switched off


@dataclass
class Flow:
    """Pipeline shared by both line kinds: an open input carton (`open_left`
    products), products on the way (ages since their start, oldest first) and
    the output carton being filled."""
    takt: float = 20.0
    lead: float = 100.0
    t_load: float = 5.0
    t_belt: float = 90.0
    items: List[float] = field(default_factory=list)
    progress: float = 0.0       # time since the last product start
    active: bool = True

    def arrivals(self) -> List[float]:
        """Seconds until the products on the way reach the output carton."""
        return [max(0.0, self.lead - a) for a in self.items]


@dataclass
class ProductionLine(Flow):
    id: str = ''
    name: str = ''
    input: str = ''
    output: str = ''
    cycle_time: float = 22.0
    products_per_kit: int = 12
    box_capacity: int = 12
    input_capacity: int = 2
    output_capacity: int = 2
    target_input: int = 2
    input_kits: int = 1
    material: int = 0           # products still in the open kit carton
    box_fill: int = 0
    out_boxes: int = 0
    inbound: int = 0            # kit deliveries created and not yet unloaded
    outbound: int = 0           # box transfers created and not yet picked
    produced: int = 0
    status: str = STARVED
    busy_time: float = 0.0


@dataclass
class PackagingLine(Flow):
    id: str = ''
    name: str = ''
    input: str = ''
    output: str = ''
    pack_time: float = 21.0
    input_capacity: int = 2
    output_capacity: int = 2
    input_boxes: int = 0
    current: int = 0            # products still in the open input carton
    pallet_fill: int = 0        # products in the finished goods carton being filled
    pallet_capacity: int = 12
    out_pallets: int = 0
    inbound: int = 0
    outbound: int = 0
    packaged: int = 0
    status: str = STARVED
    busy_time: float = 0.0


class Production:
    def __init__(self, process: dict, lines: Optional[List[dict]] = None) -> None:
        self.load_time = float(process.get('load_time', 8.0))
        self.unload_time = float(process.get('unload_time', 8.0))
        self.supply = list(process.get('supply_stations', []))
        wh = process.get('warehouse', {}) or {}
        self.wh_in = list(wh.get('inbound', []))
        self.wh_out = list(wh.get('outbound', []))
        self.wh_capacity = int(wh.get('capacity', 48))        # cartons
        self.stock = min(self.wh_capacity, int(wh.get('initial_stock', 0)))
        self.wh_incoming = 0        # FG_STORE tasks created and not yet unloaded
        self.wh_reserved = 0        # FG_PICK tasks created and not yet loaded
        self.gates = list(process.get('gates', []))
        self.truck_capacity = int(process.get('truck_capacity', 20))     # cartons
        self.truck_load: Dict[str, int] = {g: 0 for g in self.gates}
        self.trucks_out: Dict[str, int] = {g: 0 for g in self.gates}
        self.auto_lines = str(process.get('active_lines', 'all')).lower() == 'auto'
        self.robots_per_line = max(0.1, float(process.get('robots_per_line', 2.0)))
        geo = {ln['id']: ln for ln in lines or []}

        def timing(fl: Flow, lid: str, takt: float) -> None:
            fl.takt = takt
            if lid in geo:
                fl.t_load, fl.t_belt, drop = line_lead_time(process, geo[lid])
            else:           # no geometry: the old fixed cycle (lead = takt)
                fl.t_load, fl.t_belt, drop = 0.0, takt, 0.0
            fl.lead = fl.t_load + fl.t_belt + drop

        self.lines: Dict[str, ProductionLine] = {}
        for c in process.get('production_lines', []):
            ln = ProductionLine(
                id=c['id'], name=c.get('name', c['id']), input=c['input'], output=c['output'],
                cycle_time=float(c.get('cycle_time', 12.0)),
                products_per_kit=int(c.get('products_per_kit', 6)),
                box_capacity=int(c.get('box_capacity', 6)),
                input_capacity=int(c.get('input_capacity', 2)),
                output_capacity=int(c.get('output_capacity', 2)),
                target_input=int(c.get('target_input', 2)))
            timing(ln, ln.id, ln.cycle_time)
            self.lines[ln.id] = ln
        box_cap = max([ln.box_capacity for ln in self.lines.values()] or [6])
        self.packers: Dict[str, PackagingLine] = {}
        for c in process.get('packaging_lines', []):
            pk = PackagingLine(
                id=c['id'], name=c.get('name', c['id']), input=c['input'], output=c['output'],
                pack_time=float(c.get('pack_time', 8.0)),
                input_capacity=int(c.get('input_capacity', 2)),
                output_capacity=int(c.get('output_capacity', 2)),
                pallet_capacity=int(c.get('pallet_capacity', box_cap)))
            timing(pk, pk.id, pk.pack_time)
            self.packers[pk.id] = pk
        # the factory is already running when the simulation starts: staggered
        # work in progress so every line is busy from t = 0
        if process.get('initial_wip', True):
            n = max(len(self.lines), 1)
            for i, ln in enumerate(self.lines.values()):
                ln.material = ln.products_per_kit // 2
                ln.box_fill = (ln.box_capacity * (i + 1)) // (n + 1)
                ln.items = self._wip(ln)
            for pk in self.packers.values():
                pk.input_boxes = min(1, pk.input_capacity)
                pk.current = (pk.pallet_capacity * 2) // 3
                pk.items = self._wip(pk)
        self.elapsed = 0.0
        self.kits_delivered = 0
        self.cartons_stored = 0     # cartons put away in the warehouse
        self.cartons_picked = 0     # cartons taken out of it to packaging
        self.cartons_shipped = 0
        self.products_shipped = 0
        self._rr_supply = 0
        self.events: List[str] = []
        self.supply_picked: Dict[str, int] = {a: 0 for a in self.supply}
        self.fleet_size = -1
        self._active_n = len(self.lines)
        self._down_since: Optional[float] = None

    @staticmethod
    def _wip(fl: Flow) -> List[float]:
        """Products already on the way, one every takt (oldest first)."""
        n = int(fl.lead // fl.takt)
        return [fl.lead - (k + 0.5) * fl.takt for k in range(n) if fl.lead - (k + 0.5) * fl.takt > 0]

    # ------------------------------------------------------- active lines
    def set_fleet_size(self, robots: int, now: float) -> None:
        """active_lines: auto - run ceil(robots / robots_per_line) production
        lines and as many packaging lines, west first.  More robots switch lines
        on at once; fewer switch them off after AUTO_DOWN_DELAY (a robot that
        reconnects does not stop a line)."""
        if not self.auto_lines or robots <= 0:
            return
        want = min(len(self.lines), max(1, math.ceil(robots / self.robots_per_line - 1e-9)))
        if self.fleet_size < 0:
            self._apply_active(want)
            # the factory was running with this fleet: no work in progress on
            # the lines that are off (it would only flood the fleet with tasks)
            for fl in list(self.lines.values()) + list(self.packers.values()):
                if not fl.active:
                    self._clear(fl)
        elif want > self._active_n:
            self._down_since = None
            self._apply_active(want)
        elif want < self._active_n:
            if self._down_since is None:
                self._down_since = now
            elif now - self._down_since >= AUTO_DOWN_DELAY:
                self._down_since = None
                self._apply_active(want)
        else:
            self._down_since = None
        self.fleet_size = robots

    @staticmethod
    def _clear(fl: Flow) -> None:
        fl.items, fl.progress = [], 0.0
        if isinstance(fl, ProductionLine):
            fl.material = fl.box_fill = fl.out_boxes = 0
            fl.input_kits = 0
        else:
            fl.current = fl.pallet_fill = fl.out_pallets = fl.input_boxes = 0

    def _apply_active(self, n: int) -> None:
        if n != self._active_n:
            self.events.append(f'Đội AMR thay đổi → chạy {n}/{len(self.lines)} dây chuyền')
            self._active_n = n
        for group in (self.lines, self.packers):
            for i, fl in enumerate(sorted(group.values(), key=lambda f: (_order(f.id)))):
                fl.active = i < n

    @property
    def active_lines(self) -> int:
        return sum(ln.active for ln in self.lines.values())

    # ----------------------------------------------------------- simulate
    def update(self, dt: float, now: float, supply_load: Dict[str, int],
               storage_load: Dict[str, int]) -> List[Task]:
        """Advance the process by dt and return new transport requests."""
        self.elapsed += dt
        for ln in self.lines.values():
            self._run_line(ln, dt)
        for pk in self.packers.values():
            self._run_packer(pk, dt)
        return self._requests(now, supply_load, storage_load)

    @staticmethod
    def _flow(fl: Flow, dt: float, out_full: bool, can_close: bool, close, deliver,
              has_material, start) -> Tuple[str, bool]:
        """One step of a line: close the full output carton when there is room,
        stop the whole line while the next product cannot be packed, move the
        products, start a new one every takt.  Returns (status, fed): fed =
        the line had material for the whole step (busy time)."""
        if out_full and can_close:
            close()
            out_full = False
        if fl.items and fl.items[0] + dt >= fl.lead and out_full:
            return BLOCKED, False
        fl.items = [a + dt for a in fl.items]
        while fl.items and fl.items[0] >= fl.lead:
            fl.items.pop(0)
            deliver()
        fed = False
        if has_material():
            fed = True
            fl.progress += dt
            if fl.progress >= fl.takt:
                fl.progress -= fl.takt
                start()
                fl.items.append(fl.progress)
        if fed or fl.items:
            return RUNNING, fed
        return (STARVED if fl.active else IDLE), False

    def _run_line(self, ln: ProductionLine, dt: float) -> None:
        def close():
            ln.out_boxes += 1
            ln.box_fill = 0

        def deliver():
            ln.box_fill += 1
            ln.produced += 1

        def has_material():
            return ln.material > 0 or ln.input_kits > 0

        def start():
            if ln.material <= 0:
                ln.input_kits -= 1
                ln.material = ln.products_per_kit
            ln.material -= 1

        ln.status, fed = self._flow(ln, dt, ln.box_fill >= ln.box_capacity,
                                    ln.out_boxes < ln.output_capacity, close, deliver,
                                    has_material, start)
        if fed:
            ln.busy_time += dt

    def _run_packer(self, pk: PackagingLine, dt: float) -> None:
        def close():
            pk.out_pallets += 1
            pk.pallet_fill = 0

        def deliver():
            pk.pallet_fill += 1
            pk.packaged += 1

        def has_material():
            return pk.current > 0 or pk.input_boxes > 0

        def start():
            if pk.current <= 0:
                pk.input_boxes -= 1
                pk.current = self._box_size()
            pk.current -= 1

        pk.status, fed = self._flow(pk, dt, pk.pallet_fill >= pk.pallet_capacity,
                                    pk.out_pallets < pk.output_capacity, close, deliver,
                                    has_material, start)
        if fed:
            pk.busy_time += dt

    def _box_size(self) -> int:
        return max([ln.box_capacity for ln in self.lines.values()] or [6])

    # ---------------------------------------------------------- requests
    def _requests(self, now: float, supply_load: Dict[str, int],
                  storage_load: Dict[str, int]) -> List[Task]:
        out: List[Task] = []
        # component kits for the running production lines
        for ln in self.lines.values():
            while (ln.active and ln.input_kits + ln.inbound < ln.target_input
                   and ln.input_kits + ln.inbound < ln.input_capacity):
                src = self._pick_supply(supply_load)
                if src is None:
                    break
                starving = ln.input_kits == 0 and ln.material <= 1
                t = Task(KIT_DELIVERY, src, ln.input, priority=2 if starving else 1,
                         description=f'Kit linh kien {src} -> {ln.id}',
                         cargo=f'Kit linh kien ({ln.products_per_kit} bo)',
                         meta={'line': ln.id}, created=now)
                ln.inbound += 1
                supply_load[src] = supply_load.get(src, 0) + 1
                out.append(t)
        # full cartons of the production lines into the warehouse (fullest first)
        for ln in sorted(self.lines.values(), key=lambda x: (-x.out_boxes, _order(x.id))):
            while (ln.out_boxes - ln.outbound > 0 and self.wh_in
                   and self.stock + self.wh_incoming < self.wh_capacity):
                dst = self._least_loaded(self.wh_in, storage_load)
                blocked = ln.out_boxes >= ln.output_capacity
                t = Task(FG_STORE, ln.output, dst, priority=2 if blocked else 1,
                         description=f'Nhap kho {ln.id} -> {dst}',
                         cargo=f'Thung {ln.box_capacity} thanh pham',
                         meta={'line': ln.id}, created=now)
                ln.outbound += 1
                self.wh_incoming += 1
                storage_load[dst] = storage_load.get(dst, 0) + 1
                out.append(t)
        # warehouse -> packaging lines (the emptiest first)
        while self.stock - self.wh_reserved > 0 and self.wh_out:
            pk = self._pick_packer()
            if pk is None:
                break
            src = self._least_loaded(self.wh_out, storage_load)
            starving = pk.input_boxes == 0 and pk.current <= 2
            t = Task(FG_PICK, src, pk.input, priority=2 if starving else 1,
                     description=f'Xuat kho {src} -> {pk.id}',
                     cargo=f'Thung {self._box_size()} thanh pham',
                     meta={'packer': pk.id}, created=now)
            pk.inbound += 1
            self.wh_reserved += 1
            storage_load[src] = storage_load.get(src, 0) + 1
            out.append(t)
        # packed goods to the shipping gates
        for pk in self.packers.values():
            while pk.out_pallets - pk.outbound > 0 and self.gates:
                dst = self._least_loaded(self.gates, storage_load)
                blocked = pk.out_pallets >= pk.output_capacity
                t = Task(SHIPMENT, pk.output, dst, priority=2 if blocked else 1,
                         description=f'Xuat hang {pk.id} -> cong {dst}',
                         cargo=f'Thung {pk.pallet_capacity} SP da dong goi',
                         meta={'packer': pk.id}, created=now)
                pk.outbound += 1
                storage_load[dst] = storage_load.get(dst, 0) + 1
                out.append(t)
        return out

    def _pick_supply(self, load: Dict[str, int]) -> Optional[str]:
        if not self.supply:
            return None
        n = len(self.supply)
        order = [self.supply[(self._rr_supply + i) % n] for i in range(n)]
        best = min(order, key=lambda s: load.get(s, 0))
        self._rr_supply = (self.supply.index(best) + 1) % n
        return best

    @staticmethod
    def _least_loaded(stations: List[str], load: Dict[str, int]) -> str:
        """The station with the fewest robots booked / tasks queued (first one
        on a tie: the stations are listed in the order of the traffic flow)."""
        return min(stations, key=lambda s: load.get(s, 0))

    def _pick_packer(self) -> Optional[PackagingLine]:
        cands = [pk for pk in self.packers.values()
                 if pk.active and pk.input_boxes + pk.inbound < pk.input_capacity]
        if not cands:
            return None
        return min(cands, key=lambda pk: (pk.input_boxes + pk.inbound
                                          + (1 if pk.current > 0 else 0), _order(pk.id)))

    # ------------------------------------------------------ transfers
    def on_pickup(self, task: Task) -> None:
        if task.type == KIT_DELIVERY and task.pickup in self.supply_picked:
            self.supply_picked[task.pickup] += 1
        elif task.type == FG_STORE:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln:
                ln.out_boxes = max(0, ln.out_boxes - 1)
                ln.outbound = max(0, ln.outbound - 1)
        elif task.type == FG_PICK:
            self.stock = max(0, self.stock - 1)
            self.wh_reserved = max(0, self.wh_reserved - 1)
            self.cartons_picked += 1
        elif task.type == SHIPMENT:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk:
                pk.out_pallets = max(0, pk.out_pallets - 1)
                pk.outbound = max(0, pk.outbound - 1)

    def on_dropoff(self, task: Task) -> None:
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln:
                ln.input_kits += 1
                ln.inbound = max(0, ln.inbound - 1)
                self.kits_delivered += 1
        elif task.type == FG_STORE:
            self.stock = min(self.wh_capacity, self.stock + 1)
            self.wh_incoming = max(0, self.wh_incoming - 1)
            self.cartons_stored += 1
        elif task.type == FG_PICK:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk:
                pk.input_boxes += 1
                pk.inbound = max(0, pk.inbound - 1)
        elif task.type == SHIPMENT:
            pk = self.packers.get(task.meta.get('packer', ''))
            n = pk.pallet_capacity if pk else self._box_size()
            gate = task.dropoff
            self.cartons_shipped += 1
            self.products_shipped += n
            self.truck_load[gate] = self.truck_load.get(gate, 0) + 1
            if self.truck_load[gate] >= self.truck_capacity:
                self.truck_load[gate] = 0
                self.trucks_out[gate] = self.trucks_out.get(gate, 0) + 1
                self.events.append(f'Cổng {gate}: xe tải đủ {self.truck_capacity} thùng → xuất bến, '
                                   f'xe mới vào')

    def urgency(self, task: Task) -> int:
        """Current priority level (0..3) of an MES task from the line state:
        HIGH (2) when the line it serves is about to starve or is blocked."""
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is not None and ln.input_kits == 0 and ln.material <= 2:
                return 2
        elif task.type == FG_STORE:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is not None and ln.out_boxes >= ln.output_capacity:
                return 2
        elif task.type == FG_PICK:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is not None and (pk.status == STARVED
                                   or (pk.input_boxes == 0 and pk.current <= 2)):
                return 2
        elif task.type == SHIPMENT:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is not None and pk.out_pallets >= pk.output_capacity:
                return 2
        else:
            return task.priority
        return 1

    @staticmethod
    def _block_time(fl: Flow, fill: int, cap: int, out: int, out_cap: int) -> float:
        """Seconds until the line blocks: the product that finds the output
        carton full while every output slot is taken."""
        room = (cap - fill) + max(0, out_cap - out - 1) * cap
        arrivals = sorted(fl.arrivals())
        if room < len(arrivals):
            return arrivals[room]
        k = room - len(arrivals)          # products not started yet
        return fl.lead + max(0.0, fl.takt - fl.progress) + k * fl.takt

    def deadline(self, task: Task) -> float:
        """Seconds from now until the line this task serves stops without it:
        a production line runs out of material (kit) or blocks on a full output
        (warehouse put-away), a packer runs out of cartons (warehouse pick) or
        blocks on a full output (shipment).  inf: no constraint (manual task)."""
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is None:
                return math.inf
            left = ln.material + ln.input_kits * ln.products_per_kit
            return max(0.0, left * ln.takt - ln.progress)
        if task.type == FG_STORE:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is None:
                return math.inf
            return self._block_time(ln, ln.box_fill, ln.box_capacity, ln.out_boxes,
                                    ln.output_capacity)
        if task.type == FG_PICK:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is None:
                return math.inf
            left = pk.current + pk.input_boxes * self._box_size()
            return max(0.0, left * pk.takt - pk.progress)
        if task.type == SHIPMENT:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is None:
                return math.inf
            return self._block_time(pk, pk.pallet_fill, pk.pallet_capacity, pk.out_pallets,
                                    pk.output_capacity)
        return math.inf

    def on_cancel(self, task: Task) -> None:
        """Task cancelled before pick-up: roll the bookkeeping back."""
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln:
                ln.inbound = max(0, ln.inbound - 1)
        elif task.type == FG_STORE:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln:
                ln.outbound = max(0, ln.outbound - 1)
            self.wh_incoming = max(0, self.wh_incoming - 1)
        elif task.type == FG_PICK:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk:
                pk.inbound = max(0, pk.inbound - 1)
            self.wh_reserved = max(0, self.wh_reserved - 1)
        elif task.type == SHIPMENT:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk:
                pk.outbound = max(0, pk.outbound - 1)

    # ---------------------------------------------------------- state
    @property
    def products_made(self) -> int:
        return sum(ln.produced for ln in self.lines.values())

    @property
    def products_packaged(self) -> int:
        return sum(pk.packaged for pk in self.packers.values())

    def throughput_per_hour(self) -> float:
        """Products shipped (packed and loaded onto a truck) per hour."""
        if self.elapsed < 60.0:
            return 0.0
        return self.products_shipped / self.elapsed * 3600.0

    def gz_state(self) -> str:
        """Line state for the FactoryLines system of Gazebo, one entry per line:
        "B1=run,in,open,fill,out|age age ...;C1=..." - run 0 while blocked,
        `in` full cartons waiting on the input pedestal, `open` products left in
        the open carton, `fill` products in the output carton, `out` full
        output cartons, ages [s] of the products on the way (oldest first)."""
        parts = []
        for ln in self.lines.values():
            parts.append(_gz(ln.id, ln.status, ln.input_kits, ln.material, ln.box_fill,
                             ln.out_boxes, ln.items))
        for pk in self.packers.values():
            parts.append(_gz(pk.id, pk.status, pk.input_boxes, pk.current, pk.pallet_fill,
                             pk.out_pallets, pk.items))
        return ';'.join(parts)

    def snapshot(self) -> dict:
        lines = []
        for ln in self.lines.values():
            lines.append({
                'line_id': ln.id, 'line_type': 'production', 'name': ln.name,
                'status': ln.status, 'input_buffer': ln.input_kits,
                'input_capacity': ln.input_capacity, 'inbound': ln.inbound,
                'work_in_progress': ln.box_fill, 'output_buffer': ln.out_boxes,
                'output_capacity': ln.output_capacity, 'produced': ln.produced,
                'utilization': ln.busy_time / self.elapsed if self.elapsed > 0 else 0.0,
                'active': ln.active, 'on_belt': len(ln.items)})
        for pk in self.packers.values():
            lines.append({
                'line_id': pk.id, 'line_type': 'packaging', 'name': pk.name,
                'status': pk.status, 'input_buffer': pk.input_boxes,
                'input_capacity': pk.input_capacity, 'inbound': pk.inbound,
                'work_in_progress': pk.pallet_fill, 'output_buffer': pk.out_pallets,
                'output_capacity': pk.output_capacity, 'produced': pk.packaged,
                'utilization': pk.busy_time / self.elapsed if self.elapsed > 0 else 0.0,
                'active': pk.active, 'on_belt': len(pk.items)})
        if self.wh_in or self.wh_out:
            lines.append({
                'line_id': 'D', 'line_type': 'warehouse', 'name': 'Kho thanh pham',
                'status': BLOCKED if self.stock >= self.wh_capacity else
                          (STARVED if self.stock == 0 else RUNNING),
                'input_buffer': self.stock, 'input_capacity': self.wh_capacity,
                'inbound': self.wh_incoming, 'work_in_progress': 0,
                'output_buffer': self.wh_reserved, 'output_capacity': 0,
                'produced': self.cartons_stored, 'utilization': self.stock / max(1, self.wh_capacity)})
        for g in self.gates:
            lines.append({
                'line_id': g, 'line_type': 'gate', 'name': f'Cong xuat hang {g}',
                'status': RUNNING, 'input_buffer': self.truck_load.get(g, 0),
                'input_capacity': self.truck_capacity, 'inbound': 0,
                'work_in_progress': 0, 'output_buffer': self.trucks_out.get(g, 0),
                'output_capacity': 0, 'produced': self.trucks_out.get(g, 0), 'utilization': 0.0})
        for a in self.supply:
            lines.append({
                'line_id': a, 'line_type': 'supply', 'name': f'Kho linh kien {a}',
                'status': RUNNING, 'input_buffer': 0, 'input_capacity': 0, 'inbound': 0,
                'work_in_progress': 0, 'output_buffer': 0, 'output_capacity': 0,
                'produced': self.supply_picked.get(a, 0), 'utilization': 0.0})
        return {
            'elapsed_s': self.elapsed, 'lines': lines,
            'kits_delivered': self.kits_delivered,
            'products_made': self.products_made,
            'cartons_stored': self.cartons_stored,
            'cartons_picked': self.cartons_picked,
            'warehouse_stock': self.stock,
            'warehouse_capacity': self.wh_capacity,
            'products_packaged': self.products_packaged,
            'cartons_shipped': self.cartons_shipped,
            'products_shipped': self.products_shipped,
            'trucks_departed': sum(self.trucks_out.values()),
            'throughput_per_hour': self.throughput_per_hour(),
            'active_lines': self.active_lines,
        }


def _order(line_id: str):
    """B2 < B10: lines are numbered west -> east."""
    digits = ''.join(ch for ch in line_id if ch.isdigit())
    return (line_id.rstrip('0123456789'), int(digits) if digits else 0)


def _gz(lid, status, n_in, open_left, fill, n_out, items) -> str:
    run = 0 if status == BLOCKED else 1
    ages = ' '.join(f'{a:.2f}' for a in items)
    return f'{lid}={run},{n_in},{open_left},{fill},{n_out}|{ages}'
