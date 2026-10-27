"""Transport tasks."""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Dict, List, Optional

KIT_DELIVERY = 'KIT_DELIVERY'      # A  -> B_IN  (component kit)
FG_STORE = 'FG_STORE'              # B_OUT -> D inbound (carton of finished products)
FG_PICK = 'FG_PICK'                # D outbound -> C_IN (finished products to packaging)
SHIPMENT = 'SHIPMENT'              # C_OUT -> G gate (packed goods onto the truck)
MANUAL = 'MANUAL'

QUEUED = 'QUEUED'
ASSIGNED = 'ASSIGNED'
TO_PICKUP = 'TO_PICKUP'
LOADING = 'LOADING'
IN_TRANSIT = 'IN_TRANSIT'
UNLOADING = 'UNLOADING'
DONE = 'DONE'
CANCELLED = 'CANCELLED'

PRIORITY_NAMES = {0: 'LOW', 1: 'NORMAL', 2: 'HIGH', 3: 'URGENT'}

_ids = itertools.count(1)


def reset_ids() -> None:
    global _ids
    _ids = itertools.count(1)


@dataclass
class Task:
    type: str
    pickup: str
    dropoff: str
    priority: int = 1
    description: str = ''
    cargo: str = ''
    meta: Dict[str, str] = field(default_factory=dict)
    created: float = 0.0
    id: str = ''
    status: str = QUEUED
    robot: Optional[str] = None
    next_for: Optional[str] = None
    assigned_at: Optional[float] = None
    picked_at: Optional[float] = None
    done_at: Optional[float] = None

    def __post_init__(self) -> None:
        if not self.id:
            self.id = f'T{next(_ids):04d}'

    @property
    def active(self) -> bool:
        return self.status not in (DONE, CANCELLED)

    @property
    def open(self) -> bool:
        """Not yet assigned to any robot (neither as current nor as next task)."""
        return self.status == QUEUED and self.next_for is None


class TaskBook:
    def __init__(self) -> None:
        self.tasks: Dict[str, Task] = {}
        self.history: List[Task] = []
        self.completed = 0

    def add(self, task: Task) -> Task:
        self.tasks[task.id] = task
        return task

    def open_tasks(self) -> List[Task]:
        ts = [t for t in self.tasks.values() if t.open]
        ts.sort(key=lambda t: (-t.priority, t.created))
        return ts

    def active(self) -> List[Task]:
        return [t for t in self.tasks.values() if t.active]

    def finish(self, task: Task, now: float, cancelled: bool = False) -> None:
        task.status = CANCELLED if cancelled else DONE
        task.done_at = now
        self.tasks.pop(task.id, None)
        self.history.append(task)
        if len(self.history) > 300:
            self.history = self.history[-300:]
        if not cancelled:
            self.completed += 1
