"""Simulated MES: production lines, packaging lines and storage.

Material flow (one transport unit per AMR trip):

  A_i (kit of components) --KIT_DELIVERY-->  B_k input buffer
  B_k produces 1 product / cycle_time, products are put into a box;
      full box -> B_k output buffer
  B_k output (box)        --BOX_TRANSFER-->  C_m input buffer
  C_m packages every product of the box (pack_time each) onto a pallet;
      full pallet -> C_m output buffer
  C_m output (pallet)     --FG_DELIVERY-->   D_j finished goods storage

The model raises task requests when buffers need service and escalates the
task priority when a line is about to starve or block.
"""

from __future__ import annotations

import math

from dataclasses import dataclass
from typing import Dict, List, Optional

from .tasks import BOX_TRANSFER, FG_DELIVERY, KIT_DELIVERY, Task

RUNNING = 'RUNNING'
STARVED = 'STARVED'
BLOCKED = 'BLOCKED'
IDLE = 'IDLE'


@dataclass
class ProductionLine:
    id: str
    name: str
    input: str
    output: str
    cycle_time: float
    products_per_kit: int
    box_capacity: int
    input_capacity: int
    output_capacity: int
    target_input: int
    input_kits: int = 1
    material: int = 0           # products that can still be made from the opened kit
    box_fill: int = 0
    out_boxes: int = 0
    inbound: int = 0            # kit deliveries created and not yet unloaded
    outbound: int = 0           # box transfers created and not yet picked
    progress: float = 0.0
    produced: int = 0
    status: str = STARVED
    busy_time: float = 0.0


@dataclass
class PackagingLine:
    id: str
    name: str
    input: str
    output: str
    pack_time: float
    input_capacity: int
    output_capacity: int
    input_boxes: int = 0
    current: int = 0            # unpacked products of the box being processed
    pallet_fill: int = 0
    pallet_capacity: int = 6
    out_pallets: int = 0
    inbound: int = 0
    outbound: int = 0
    progress: float = 0.0
    packaged: int = 0
    status: str = STARVED
    busy_time: float = 0.0


class Production:
    def __init__(self, process: dict) -> None:
        self.load_time = float(process.get('load_time', 8.0))
        self.unload_time = float(process.get('unload_time', 8.0))
        self.supply = list(process.get('supply_stations', []))
        self.storage = list(process.get('storage_stations', []))
        self.storage_capacity = int(process.get('storage_capacity', 240))
        self.lines: Dict[str, ProductionLine] = {}
        for c in process.get('production_lines', []):
            ln = ProductionLine(
                id=c['id'], name=c.get('name', c['id']), input=c['input'], output=c['output'],
                cycle_time=float(c.get('cycle_time', 12.0)),
                products_per_kit=int(c.get('products_per_kit', 6)),
                box_capacity=int(c.get('box_capacity', 6)),
                input_capacity=int(c.get('input_capacity', 3)),
                output_capacity=int(c.get('output_capacity', 2)),
                target_input=int(c.get('target_input', 2)))
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
            self.packers[pk.id] = pk
        # the factory is already running when the simulation starts: staggered
        # work in progress so every line is busy from t = 0
        if process.get('initial_wip', True):
            n = max(len(self.lines), 1)
            for i, ln in enumerate(self.lines.values()):
                ln.material = ln.products_per_kit // 2
                ln.box_fill = (ln.box_capacity * (i + 1)) // (n + 1)
            for pk in self.packers.values():
                pk.input_boxes = min(1, pk.input_capacity)
                pk.current = (pk.pallet_capacity * 2) // 3
        self.stored: Dict[str, int] = {d: 0 for d in self.storage}
        self.shipped = 0
        self.elapsed = 0.0
        self.kits_delivered = 0
        self.boxes_transferred = 0
        self.finished_stored = 0
        self._rr_supply = 0
        self._rr_storage = 0
        self.events: List[str] = []
        self.supply_picked: Dict[str, int] = {a: 0 for a in self.supply}

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

    def _run_line(self, ln: ProductionLine, dt: float) -> None:
        if ln.box_fill >= ln.box_capacity:
            if ln.out_boxes < ln.output_capacity:
                ln.out_boxes += 1
                ln.box_fill = 0
            else:
                ln.status = BLOCKED
                return
        if ln.material <= 0:
            if ln.input_kits > 0:
                ln.input_kits -= 1
                ln.material = ln.products_per_kit
            else:
                ln.status = STARVED
                return
        ln.status = RUNNING
        ln.busy_time += dt
        ln.progress += dt
        if ln.progress >= ln.cycle_time:
            ln.progress -= ln.cycle_time
            ln.material -= 1
            ln.box_fill += 1
            ln.produced += 1

    def _run_packer(self, pk: PackagingLine, dt: float) -> None:
        if pk.pallet_fill >= pk.pallet_capacity:
            if pk.out_pallets < pk.output_capacity:
                pk.out_pallets += 1
                pk.pallet_fill = 0
            else:
                pk.status = BLOCKED
                return
        if pk.current <= 0:
            if pk.input_boxes > 0:
                pk.input_boxes -= 1
                pk.current = self._box_size()
            else:
                pk.status = STARVED
                return
        pk.status = RUNNING
        pk.busy_time += dt
        pk.progress += dt
        if pk.progress >= pk.pack_time:
            pk.progress -= pk.pack_time
            pk.current -= 1
            pk.pallet_fill += 1
            pk.packaged += 1

    def _box_size(self) -> int:
        return max([ln.box_capacity for ln in self.lines.values()] or [6])

    # ---------------------------------------------------------- requests
    def _requests(self, now: float, supply_load: Dict[str, int],
                  storage_load: Dict[str, int]) -> List[Task]:
        out: List[Task] = []
        # component kits for production lines
        for ln in self.lines.values():
            while (ln.input_kits + ln.inbound < ln.target_input
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
        # boxes of unpacked products to packaging (fullest line first)
        for ln in sorted(self.lines.values(), key=lambda x: (-x.out_boxes, x.id)):
            while ln.out_boxes - ln.outbound > 0:
                dst = self._pick_packer()
                if dst is None:
                    break
                blocked = ln.out_boxes >= ln.output_capacity
                t = Task(BOX_TRANSFER, ln.output, dst.input, priority=2 if blocked else 1,
                         description=f'Thung {ln.box_capacity} SP chua dong goi {ln.id} -> {dst.id}',
                         cargo=f'Thung {ln.box_capacity} SP chua dong goi',
                         meta={'line': ln.id, 'packer': dst.id}, created=now)
                ln.outbound += 1
                dst.inbound += 1
                out.append(t)
        # finished pallets to storage
        for pk in self.packers.values():
            while pk.out_pallets - pk.outbound > 0:
                dst = self._pick_storage(storage_load)
                blocked = pk.out_pallets >= pk.output_capacity
                t = Task(FG_DELIVERY, pk.output, dst, priority=2 if blocked else 1,
                         description=f'{pk.pallet_capacity} thanh pham {pk.id} -> {dst}',
                         cargo=f'{pk.pallet_capacity} SP thanh pham',
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

    def _pick_packer(self) -> Optional[PackagingLine]:
        cands = [pk for pk in self.packers.values()
                 if pk.input_boxes + pk.inbound < pk.input_capacity]
        if not cands:
            return None
        return min(cands, key=lambda pk: (pk.input_boxes + pk.inbound
                                          + (1 if pk.current > 0 else 0), pk.id))

    def _pick_storage(self, load: Dict[str, int]) -> str:
        n = len(self.storage)
        order = [self.storage[(self._rr_storage + i) % n] for i in range(n)]
        best = min(order, key=lambda s: load.get(s, 0))
        self._rr_storage = (self.storage.index(best) + 1) % n
        return best

    # ------------------------------------------------------ transfers
    def on_pickup(self, task: Task) -> None:
        if task.type == KIT_DELIVERY and task.pickup in self.supply_picked:
            self.supply_picked[task.pickup] += 1
        elif task.type == BOX_TRANSFER:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln:
                ln.out_boxes = max(0, ln.out_boxes - 1)
                ln.outbound = max(0, ln.outbound - 1)
        elif task.type == FG_DELIVERY:
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
        elif task.type == BOX_TRANSFER:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk:
                pk.input_boxes += 1
                pk.inbound = max(0, pk.inbound - 1)
                self.boxes_transferred += 1
        elif task.type == FG_DELIVERY:
            n = self.packers[task.meta['packer']].pallet_capacity \
                if task.meta.get('packer') in self.packers else 0
            self.stored[task.dropoff] = self.stored.get(task.dropoff, 0) + n
            self.finished_stored += n
            if self.stored[task.dropoff] >= self.storage_capacity:
                self.shipped += self.stored[task.dropoff]
                self.stored[task.dropoff] = 0
                self.events.append(f'{task.dropoff}: kho day -> xuat hang')

    def urgency(self, task: Task) -> int:
        """Current priority level (0..3) of an MES task from the line state:
        HIGH (2) when the line it serves is about to starve or is blocked."""
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is not None and ln.input_kits == 0 and ln.material <= 2:
                return 2
        elif task.type == BOX_TRANSFER:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is not None and ln.out_boxes >= ln.output_capacity:
                return 2
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is not None and pk.status == STARVED:
                return 2
        elif task.type == FG_DELIVERY:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is not None and pk.out_pallets >= pk.output_capacity:
                return 2
        else:
            return task.priority
        return 1

    def deadline(self, task: Task) -> float:
        """Seconds from now until the line this task serves stops without it:
        a production line runs out of material (kit), a production line blocks
        on a full output or the packer runs out of boxes (box transfer), a
        packer blocks on a full pallet output (finished goods).  inf: no
        constraint (manual task)."""
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is None:
                return math.inf
            left = ln.material + ln.input_kits * ln.products_per_kit
            return max(0.0, left * ln.cycle_time - ln.progress)
        if task.type == BOX_TRANSFER:
            out = math.inf
            ln = self.lines.get(task.meta.get('line', ''))
            if ln is not None:
                room = (ln.box_capacity - ln.box_fill) + \
                    max(0, ln.output_capacity - ln.out_boxes - 1) * ln.box_capacity
                out = max(0.0, room * ln.cycle_time - ln.progress)
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is not None:
                left = pk.current + pk.input_boxes * self._box_size()
                out = min(out, max(0.0, left * pk.pack_time - pk.progress))
            return out
        if task.type == FG_DELIVERY:
            pk = self.packers.get(task.meta.get('packer', ''))
            if pk is None:
                return math.inf
            room = (pk.pallet_capacity - pk.pallet_fill) + \
                max(0, pk.output_capacity - pk.out_pallets - 1) * pk.pallet_capacity
            return max(0.0, room * pk.pack_time - pk.progress)
        return math.inf

    def on_cancel(self, task: Task) -> None:
        """Task cancelled before pick-up: roll the bookkeeping back."""
        if task.type == KIT_DELIVERY:
            ln = self.lines.get(task.meta.get('line', ''))
            if ln:
                ln.inbound = max(0, ln.inbound - 1)
        elif task.type == BOX_TRANSFER:
            ln = self.lines.get(task.meta.get('line', ''))
            pk = self.packers.get(task.meta.get('packer', ''))
            if ln:
                ln.outbound = max(0, ln.outbound - 1)
            if pk:
                pk.inbound = max(0, pk.inbound - 1)
        elif task.type == FG_DELIVERY:
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
        if self.elapsed < 60.0:
            return 0.0
        return self.finished_stored / self.elapsed * 3600.0

    def snapshot(self) -> dict:
        lines = []
        for ln in self.lines.values():
            lines.append({
                'line_id': ln.id, 'line_type': 'production', 'name': ln.name,
                'status': ln.status, 'input_buffer': ln.input_kits,
                'input_capacity': ln.input_capacity, 'inbound': ln.inbound,
                'work_in_progress': ln.box_fill, 'output_buffer': ln.out_boxes,
                'output_capacity': ln.output_capacity, 'produced': ln.produced,
                'utilization': ln.busy_time / self.elapsed if self.elapsed > 0 else 0.0})
        for pk in self.packers.values():
            lines.append({
                'line_id': pk.id, 'line_type': 'packaging', 'name': pk.name,
                'status': pk.status, 'input_buffer': pk.input_boxes,
                'input_capacity': pk.input_capacity, 'inbound': pk.inbound,
                'work_in_progress': pk.pallet_fill, 'output_buffer': pk.out_pallets,
                'output_capacity': pk.output_capacity, 'produced': pk.packaged,
                'utilization': pk.busy_time / self.elapsed if self.elapsed > 0 else 0.0})
        for d in self.storage:
            lines.append({
                'line_id': d, 'line_type': 'storage', 'name': f'Kho thanh pham {d}',
                'status': IDLE, 'input_buffer': self.stored.get(d, 0),
                'input_capacity': self.storage_capacity, 'inbound': 0,
                'work_in_progress': 0, 'output_buffer': 0, 'output_capacity': 0,
                'produced': self.stored.get(d, 0), 'utilization': 0.0})
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
            'boxes_transferred': self.boxes_transferred,
            'products_packaged': self.products_packaged,
            'finished_stored': self.finished_stored,
            'throughput_per_hour': self.throughput_per_hour(),
        }
