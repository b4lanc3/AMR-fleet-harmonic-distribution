"""Traffic graph used by the fleet manager.

The graph is directed.  Lane and intersection nodes are *transit* nodes that
any route may traverse; lay-by slots, docks, parking spots and yield bays are
*non-transit*: a route may only use them when they belong to the route's start
or goal group (e.g. the queue slot of the station the robot is driving to).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

LANE = 'lane'
INTERSECTION = 'intersection'
QUEUE = 'queue'
STATION = 'station'
CHARGER = 'charger'
PARKING = 'parking'
YIELD = 'yield'

TRANSIT_KINDS = (LANE, INTERSECTION)


@dataclass
class Node:
    idx: int
    id: str
    x: float
    y: float
    kind: str
    lanes: List[str] = field(default_factory=list)
    group: Optional[str] = None      # station / dock / bay id for non-transit nodes
    heading: Optional[float] = None  # preferred yaw when parked on this node

    @property
    def transit(self) -> bool:
        return self.kind in TRANSIT_KINDS

    @property
    def xy(self) -> Tuple[float, float]:
        return (self.x, self.y)


@dataclass
class Edge:
    u: int
    v: int
    length: float
    speed: float
    lane: Optional[str] = None
    two_way: bool = False

    @property
    def time(self) -> float:
        return self.length / max(self.speed, 1e-3)


@dataclass
class StationNodes:
    station_id: str
    service: int            # S slot
    queue: Optional[int]    # Q slot (upstream of S)
    entry: int              # lane node used to enter the lay-by
    exit: int               # lane node used to leave the lay-by
    heading: float          # yaw of a robot standing in the slot


class TrafficGraph:
    """Directed traffic graph with conflict information."""

    def __init__(self) -> None:
        self.nodes: List[Node] = []
        self.out_edges: List[List[Edge]] = []
        self.in_edges: List[List[Edge]] = []
        self.index: Dict[str, int] = {}
        self.stations: Dict[str, StationNodes] = {}
        self.chargers: Dict[str, int] = {}
        self.parking: Dict[str, int] = {}
        self.yield_bays: List[int] = []
        self.lane_names: Dict[str, str] = {}
        self.lane_two_way: Dict[str, bool] = {}
        self.conflicts: List[Set[int]] = []
        self.sweeps: Dict[Tuple[int, int], Set[int]] = {}
        self.two_way_cost_factor = 1.0
        self._cost_to_cache: Dict[int, Dict[int, float]] = {}
        self._xy: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ build
    def add_node(self, node_id: str, x: float, y: float, kind: str,
                 lane: Optional[str] = None, group: Optional[str] = None,
                 heading: Optional[float] = None) -> int:
        if node_id in self.index:
            raise ValueError(f'duplicate node id {node_id}')
        idx = len(self.nodes)
        node = Node(idx, node_id, float(x), float(y), kind,
                    [lane] if lane else [], group, heading)
        self.nodes.append(node)
        self.out_edges.append([])
        self.in_edges.append([])
        self.index[node_id] = idx
        return idx

    def add_edge(self, u: int, v: int, speed: float, lane: Optional[str] = None,
                 two_way: bool = False) -> None:
        if u == v:
            return
        for e in self.out_edges[u]:
            if e.v == v:
                return
        length = math.dist(self.nodes[u].xy, self.nodes[v].xy)
        edge = Edge(u, v, length, speed, lane, two_way)
        self.out_edges[u].append(edge)
        self.in_edges[v].append(edge)

    def finalize(self, conflict_radius: float, sweep_radius: float) -> None:
        """Pre-compute node conflict sets and edge sweep sets."""
        xy = np.array([[n.x, n.y] for n in self.nodes], dtype=float)
        self._xy = xy
        d = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=2)
        self.conflicts = [set(np.nonzero(d[i] < conflict_radius)[0].tolist())
                          for i in range(len(self.nodes))]
        self.sweeps = {}
        for u, edges in enumerate(self.out_edges):
            for e in edges:
                key = (min(e.u, e.v), max(e.u, e.v))
                if key in self.sweeps:
                    continue
                a = xy[e.u]
                b = xy[e.v]
                ab = b - a
                L2 = float(ab @ ab)
                t = np.clip(((xy - a) @ ab) / max(L2, 1e-9), 0.0, 1.0)
                proj = a + t[:, None] * ab
                dist = np.linalg.norm(xy - proj, axis=1)
                hits = set(np.nonzero(dist < sweep_radius)[0].tolist())
                hits.discard(e.u)
                hits.discard(e.v)
                self.sweeps[key] = hits
        # Edges that geometrically cross each other (without sharing a node)
        # are mutually exclusive: add the end nodes of one to the sweep of
        # the other.
        keys = list(self.sweeps)
        if keys:
            seg = np.array([[xy[a, 0], xy[a, 1], xy[b, 0], xy[b, 1]] for a, b in keys])
            for i, (a, b) in enumerate(keys):
                p, r = seg[i, :2], seg[i, 2:] - seg[i, :2]
                q, s = seg[:, :2], seg[:, 2:] - seg[:, :2]
                den = r[0] * s[:, 1] - r[1] * s[:, 0]
                qp = q - p
                with np.errstate(divide='ignore', invalid='ignore'):
                    t = (qp[:, 0] * s[:, 1] - qp[:, 1] * s[:, 0]) / den
                    u = (qp[:, 0] * r[1] - qp[:, 1] * r[0]) / den
                hit = (np.abs(den) > 1e-9) & (t > 0.02) & (t < 0.98) & (u > 0.02) & (u < 0.98)
                for j in np.nonzero(hit)[0]:
                    c, d = keys[j]
                    if len({a, b, c, d}) < 4:
                        continue
                    self.sweeps[(a, b)].update((c, d))
        self._cost_to_cache.clear()

    # ---------------------------------------------------------------- queries
    def __len__(self) -> int:
        return len(self.nodes)

    def node(self, ref) -> Node:
        if isinstance(ref, str):
            return self.nodes[self.index[ref]]
        return self.nodes[ref]

    def edge(self, u: int, v: int) -> Optional[Edge]:
        for e in self.out_edges[u]:
            if e.v == v:
                return e
        return None

    def sweep(self, u: int, v: int) -> Set[int]:
        return self.sweeps.get((min(u, v), max(u, v)), set())

    def is_intersection(self, idx: int) -> bool:
        return self.nodes[idx].kind == INTERSECTION

    def nearest_node(self, x: float, y: float,
                     kinds: Optional[Iterable[str]] = None) -> int:
        if self._xy is None:
            raise RuntimeError('graph not finalized')
        d = np.hypot(self._xy[:, 0] - x, self._xy[:, 1] - y)
        if kinds is not None:
            kinds = set(kinds)
            mask = np.array([n.kind in kinds for n in self.nodes])
            d = np.where(mask, d, np.inf)
        return int(np.argmin(d))

    def distance(self, a: int, b: int) -> float:
        return math.dist(self.nodes[a].xy, self.nodes[b].xy)

    def allowed(self, idx: int, groups: Set[Optional[str]]) -> bool:
        n = self.nodes[idx]
        return n.transit or n.group in groups

    # --------------------------------------------------------------- planning
    def edge_cost(self, e: Edge) -> float:
        c = e.time
        if e.two_way and e.lane is not None:
            c *= self.two_way_cost_factor
        return c

    def plan(self, start: int, goal: int,
             node_penalty: Optional[Dict[int, float]] = None,
             blocked: Optional[Set[int]] = None,
             turn_penalty: float = 1.2) -> Optional[List[int]]:
        """A* from start to goal.  Returns the node index list or None."""
        if start == goal:
            return [start]
        groups = {self.nodes[start].group, self.nodes[goal].group}
        groups.discard(None)
        node_penalty = node_penalty or {}
        blocked = blocked or set()
        gx, gy = self.nodes[goal].xy
        vmax = 1.0

        def h(i: int) -> float:
            n = self.nodes[i]
            return math.hypot(n.x - gx, n.y - gy) / vmax

        open_heap: List[Tuple[float, float, int]] = [(h(start), 0.0, start)]
        g = {start: 0.0}
        parent: Dict[int, int] = {}
        closed: Set[int] = set()
        while open_heap:
            _, cost, u = heapq.heappop(open_heap)
            if u in closed:
                continue
            if u == goal:
                path = [u]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                path.reverse()
                return path
            closed.add(u)
            pu = parent.get(u)
            for e in self.out_edges[u]:
                v = e.v
                if v in closed or (v in blocked and v != goal):
                    continue
                if v != goal and not self.allowed(v, groups):
                    continue
                c = cost + self.edge_cost(e) + node_penalty.get(v, 0.0)
                if pu is not None and turn_penalty > 0.0:
                    c += turn_penalty * self.turn_angle(pu, u, v) / math.pi
                if c < g.get(v, math.inf):
                    g[v] = c
                    parent[v] = u
                    heapq.heappush(open_heap, (c + h(v), c, v))
        return None

    def dijkstra_from(self, start: int, groups: Set[Optional[str]],
                      blocked: Optional[Set[int]] = None,
                      max_cost: float = math.inf) -> Tuple[Dict[int, float], Dict[int, int]]:
        """Forward Dijkstra honouring the transit rules (used for rejoining)."""
        groups = set(groups) | {self.nodes[start].group}
        blocked = blocked or set()
        dist = {start: 0.0}
        parent: Dict[int, int] = {}
        heap = [(0.0, start)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, math.inf) or d > max_cost:
                continue
            for e in self.out_edges[u]:
                v = e.v
                if v in blocked or not self.allowed(v, groups):
                    continue
                nd = d + self.edge_cost(e)
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    parent[v] = u
                    heapq.heappush(heap, (nd, v))
        return dist, parent

    @staticmethod
    def unwind(parent: Dict[int, int], start: int, goal: int) -> List[int]:
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def turn_angle(self, a: int, b: int, c: int) -> float:
        na, nb, nc = self.nodes[a], self.nodes[b], self.nodes[c]
        h1 = math.atan2(nb.y - na.y, nb.x - na.x)
        h2 = math.atan2(nc.y - nb.y, nc.x - nb.x)
        return abs(wrap_angle(h2 - h1))

    def cost_to(self, target: int) -> Dict[int, float]:
        """Travel time from every node to `target` (reverse Dijkstra, cached).

        Non-transit nodes are only expanded when they belong to the target's
        group, which matches what `plan` allows for routes ending at target.
        Start nodes that are non-transit are handled by the caller adding the
        exit leg (see `travel_time`)."""
        cached = self._cost_to_cache.get(target)
        if cached is not None:
            return cached
        groups = {self.nodes[target].group}
        dist = {target: 0.0}
        heap = [(0.0, target)]
        while heap:
            d, v = heapq.heappop(heap)
            if d > dist.get(v, math.inf):
                continue
            if v != target and not self.allowed(v, groups):
                continue
            for e in self.in_edges[v]:
                nd = d + self.edge_cost(e)
                if nd < dist.get(e.u, math.inf):
                    dist[e.u] = nd
                    heapq.heappush(heap, (nd, e.u))
        self._cost_to_cache[target] = dist
        return dist

    def travel_time(self, start: int, goal: int) -> float:
        """Estimated travel time start -> goal (uses cached reverse Dijkstra)."""
        if start == goal:
            return 0.0
        d = self.cost_to(goal).get(start)
        if d is not None:
            return d
        path = self.plan(start, goal, turn_penalty=0.0)
        if path is None:
            return math.inf
        return self.path_time(path)

    def path_length(self, path: Sequence[int]) -> float:
        return sum(self.distance(a, b) for a, b in zip(path, path[1:]))

    def path_time(self, path: Sequence[int], turn_time_per_rad: float = 0.8) -> float:
        t = 0.0
        for i, (a, b) in enumerate(zip(path, path[1:])):
            e = self.edge(a, b)
            t += e.time if e else self.distance(a, b)
            if i > 0:
                t += turn_time_per_rad * self.turn_angle(path[i - 1], a, b)
        return t

    def lane_of(self, a: int, b: int) -> Optional[str]:
        e = self.edge(a, b)
        return e.lane if e else None

    def route_summary(self, path: Sequence[int], max_items: int = 7) -> str:
        """Human readable route: 'P03 > west- > north+ > A2'."""
        if not path:
            return ''
        items: List[str] = []

        def push(name: Optional[str]) -> None:
            if name and (not items or items[-1] != name):
                items.append(name)

        first = self.nodes[path[0]]
        push(first.group or (first.lanes[0] if first.lanes else first.id))
        for a, b in zip(path, path[1:]):
            lane = self.lane_of(a, b)
            nb = self.nodes[b]
            if lane:
                push(lane)
            elif nb.group:
                push(nb.group)
        last = self.nodes[path[-1]]
        push(last.group or last.id)
        if len(items) > max_items:
            items = items[:2] + ['...'] + items[-(max_items - 3):]
        return ' > '.join(items)

    def is_strongly_connected(self, nodes: Iterable[int]) -> Tuple[bool, List[str]]:
        """Check that every node in `nodes` can reach every other one."""
        nodes = list(nodes)
        if not nodes:
            return True, []
        bad: List[str] = []
        ref = nodes[0]
        for n in nodes[1:]:
            if self.plan(ref, n, turn_penalty=0.0) is None:
                bad.append(f'{self.nodes[ref].id}->{self.nodes[n].id}')
            if self.plan(n, ref, turn_penalty=0.0) is None:
                bad.append(f'{self.nodes[n].id}->{self.nodes[ref].id}')
        return not bad, bad


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
