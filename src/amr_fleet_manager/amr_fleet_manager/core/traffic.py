"""Priority based traffic control (node reservation).

Every control cycle:

1. *Held* nodes are computed for every robot: the node it stands on, the
   node it is driving into and every node it can no longer stop before
   (braking distance, never stopping inside an intersection).  Held nodes are
   hard locks - they belong to the robot whatever its priority.
2. Robots are visited in descending priority.  Each robot extends its
   *granted* segment along its route node by node (an intersection and the
   node after it are granted atomically: "don't block the box") until the
   look-ahead distance is covered or a node is owned by another robot.  A node
   is free only if no node of its conflict set (nodes closer than the robot
   footprint) and no node swept by the edge leading to it is owned by
   somebody else.
3. The first owner that stops a robot is recorded as `blocked_by`; this
   builds the wait-for graph used by the fleet manager to resolve head-on
   conflicts, overtaking and deadlocks (lower priority robot yields).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set

from .graph import TrafficGraph


@dataclass
class TrafficParams:
    lookahead: float = 4.5        # metres granted ahead of the robot
    min_nodes: int = 2            # always try to grant at least this many nodes ahead
    decel: float = 0.9
    brake_margin: float = 0.35
    leave_tol: float = 0.25       # robot has left its node when farther than this


@dataclass
class TrafficAgent:
    """What the traffic controller needs to know about one robot."""
    id: str
    priority: float
    x: float = 0.0
    y: float = 0.0
    speed: float = 0.0
    route: List[int] = field(default_factory=list)
    progress: int = 0             # last reached route index
    acked: bool = True            # robot already follows `route`
    prefix_len: int = 1           # nodes of `route` held when the route was issued
    prev_granted: int = 0
    current_node: Optional[int] = None
    hold: bool = False            # paused: no new grants
    # outputs
    held: List[int] = field(default_factory=list)
    held_end: int = 0             # route index of the last held node
    granted: int = 0
    blocked_by: Optional[str] = None
    blocked_node: Optional[int] = None
    block_owners: Set[str] = field(default_factory=set)


class TrafficController:
    def __init__(self, graph: TrafficGraph, params: Optional[TrafficParams] = None) -> None:
        self.g = graph
        self.p = params or TrafficParams()
        self.owner: Dict[int, str] = {}
        self.physical_conflicts: List[tuple] = []

    # ---------------------------------------------------------- held nodes
    def _held(self, a: TrafficAgent) -> None:
        g, p = self.g, self.p
        if not a.route:
            node = a.current_node if a.current_node is not None else g.nearest_node(a.x, a.y)
            a.held = [node]
            a.held_end = 0
            a.granted = 0
            return
        if not a.acked:
            n = max(1, min(a.prefix_len, len(a.route)))
            a.held = list(a.route[:n])
            a.held_end = n - 1
            return
        route = a.route
        k = min(max(a.progress, 0), len(route) - 1)
        held = [route[k]]
        end = k
        if k + 1 < len(route) and a.prev_granted > k:
            nk = g.nodes[route[k]]
            left = math.hypot(a.x - nk.x, a.y - nk.y) > p.leave_tol or abs(a.speed) > 0.05
            if left:
                d_brake = a.speed * a.speed / (2.0 * p.decel) + p.brake_margin
                end = k + 1
                held.append(route[end])
                while end < min(a.prev_granted, len(route) - 1):
                    n = g.nodes[route[end]]
                    dist = math.hypot(a.x - n.x, a.y - n.y)
                    if dist < d_brake or g.is_intersection(route[end]):
                        end += 1
                        held.append(route[end])
                    else:
                        break
        a.held = held
        a.held_end = end

    # ---------------------------------------------------------------- core
    def free_for(self, robot_id: str, nodes: Iterable[int]) -> Optional[Set[str]]:
        """Return None if all nodes are free for robot_id, else the owners."""
        owners: Set[str] = set()
        for n in nodes:
            for c in self.g.conflicts[n]:
                o = self.owner.get(c)
                if o is not None and o != robot_id:
                    owners.add(o)
        return owners or None

    def _claim(self, robot_id: str, nodes: Iterable[int]) -> None:
        for n in nodes:
            self.owner[n] = robot_id

    def _edge_nodes(self, route: Sequence[int], i: int, j: int) -> Set[int]:
        out: Set[int] = set()
        for m in range(i, j):
            out |= self.g.sweep(route[m], route[m + 1])
        return out

    def compute(self, agents: Sequence[TrafficAgent]) -> None:
        g, p = self.g, self.p
        self.owner = {}
        self.physical_conflicts = []
        for a in agents:
            self._held(a)
            a.blocked_by = None
            a.blocked_node = None
            a.block_owners = set()
        # hard locks first (physical occupation), conflict check for safety
        for a in agents:
            nodes = set(a.held)
            if a.route:
                nodes |= self._edge_nodes(a.route, max(0, a.held_end - len(a.held) + 1), a.held_end)
            clash = self.free_for(a.id, nodes)
            if clash:
                self.physical_conflicts.append((a.id, tuple(sorted(clash))))
            self._claim(a.id, nodes)

        for a in sorted(agents, key=lambda r: -r.priority):
            if not a.route:
                continue
            route = a.route
            i = a.held_end
            if a.hold:
                a.granted = i
                continue
            last = len(route) - 1
            n_i = g.nodes[route[i]]
            dist = math.hypot(a.x - n_i.x, a.y - n_i.y)
            while i < last:
                if dist >= p.lookahead and i - a.held_end >= p.min_nodes:
                    break
                j = i + 1
                while j < last and g.is_intersection(route[j]):
                    j += 1
                block = route[i + 1:j + 1]
                needed = set(block) | self._edge_nodes(route, i, j)
                owners = self.free_for(a.id, needed)
                if owners:
                    a.blocked_node = route[i + 1]
                    a.block_owners = owners
                    # the owner of the first blocked node is the main blocker
                    first = None
                    for c in g.conflicts[route[i + 1]]:
                        o = self.owner.get(c)
                        if o is not None and o != a.id:
                            first = o
                            break
                    a.blocked_by = first or sorted(owners)[0]
                    break
                self._claim(a.id, needed)
                for m in range(i, j):
                    dist += g.distance(route[m], route[m + 1])
                i = j
            a.granted = i

    # ------------------------------------------------------------- helpers
    def nodes_owned_by_others(self, robot_id: str) -> Set[int]:
        return {n for n, o in self.owner.items() if o != robot_id}

    def closure(self, nodes: Iterable[int]) -> Set[int]:
        out: Set[int] = set()
        for n in nodes:
            out |= self.g.conflicts[n]
        return out
