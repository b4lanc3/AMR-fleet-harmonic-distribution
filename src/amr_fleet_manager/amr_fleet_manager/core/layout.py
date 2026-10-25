"""Build the traffic graph from the factory layout YAML.

Aisles are expanded into lanes (right-hand traffic), lanes are intersected,
discretised into nodes, and lay-bys / docks / yield bays are attached on the
right-hand side of their lane.  The same layout dictionary is used by the
world generator, so Gazebo, RViz, the dashboard and the planner always agree.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

from .graph import (CHARGER, INTERSECTION, LANE, PARKING, QUEUE, STATION, YIELD,
                    StationNodes, TrafficGraph)

Vec = Tuple[float, float]

# priority of key points when two of them fall within the merge tolerance
_KEY_RANK = {'end': 3, 'cross': 4, 'attach': 2, 'fill': 0}


def line_objects(layout: dict) -> List[dict]:
    """Footprints of the production / packaging lines (conveyor + machines).

    Every line runs north (y_in, input) -> south (y_out, output).  The world
    generator dresses these footprints with detailed geometry and moving
    products; the traffic graph, maps and simulators use the rectangles."""
    out: List[dict] = []
    for ln in layout.get('lines', []) or []:
        x, yi, yo = float(ln['x']), float(ln['y_in']), float(ln['y_out'])
        mid = (yi + yo) / 2.0
        lid = ln['id']

        def add(kind, x0, y0, x1, y1, h, **kw):
            o = {'kind': kind, 'rect': [x0, y0, x1, y1], 'height': h, 'line': lid}
            o.update(kw)
            out.append(o)

        add('line_conveyor', x - 0.5, yo, x + 0.5, yi, 0.9, flow=[[x, yi], [x, yo]])
        if ln.get('kind', 'production') == 'production':
            add('cnc', x - 3.4, yi - 7.5, x - 0.8, yi - 3.5, 2.4)
            add('control_cabinet', x + 0.9, yi - 3.0, x + 2.4, yi - 2.2, 2.0)
            add('robot_cell', x + 0.8, mid - 1.0, x + 3.4, mid + 2.5, 2.0)
            add('assembly', x - 3.2, mid - 5.5, x - 0.8, mid - 2.0, 1.0)
            add('qc_gate', x - 0.9, yo + 4.6, x + 0.9, yo + 5.4, 2.2)
            add('boxing', x + 0.8, yo + 1.2, x + 3.2, yo + 4.0, 1.8)
        else:
            add('unboxer', x - 3.0, yi - 5.0, x - 0.8, yi - 2.0, 1.6)
            add('wrapper', x - 1.4, mid + 0.5, x + 1.4, mid + 4.5, 2.2)
            add('case_packer', x + 0.8, mid - 4.5, x + 3.2, mid - 1.0, 2.0)
            add('palletizer', x - 3.4, yo + 1.0, x - 0.8, yo + 4.5, 2.6)
            add('pallet_out', x + 0.8, yo + 0.6, x + 2.2, yo + 2.0, 1.2)
    return out


def static_objects(layout: dict) -> List[dict]:
    """Layout objects + line footprints (everything but station side objects)."""
    return list(layout.get('objects', []) or []) + line_objects(layout)


def load_layout(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


@dataclass
class Lane:
    id: str
    name: str
    p0: Vec
    p1: Vec
    two_way: bool
    aisle: str
    yield_side: Optional[str] = 'right'
    yield_spacing: Optional[float] = None
    keys: Dict[float, str] = field(default_factory=dict)   # t -> key kind
    node_at: Dict[float, int] = field(default_factory=dict)

    @property
    def length(self) -> float:
        return math.dist(self.p0, self.p1)

    @property
    def dir(self) -> Vec:
        L = self.length
        return ((self.p1[0] - self.p0[0]) / L, (self.p1[1] - self.p0[1]) / L)

    @property
    def right(self) -> Vec:
        dx, dy = self.dir
        return (dy, -dx)

    @property
    def heading(self) -> float:
        dx, dy = self.dir
        return math.atan2(dy, dx)

    def point(self, t: float) -> Vec:
        dx, dy = self.dir
        return (self.p0[0] + dx * t, self.p0[1] + dy * t)

    def project(self, p: Sequence[float]) -> float:
        dx, dy = self.dir
        t = (p[0] - self.p0[0]) * dx + (p[1] - self.p0[1]) * dy
        return min(max(t, 0.0), self.length)

    def distance_to(self, p: Sequence[float]) -> float:
        q = self.point(self.project(p))
        return math.dist(q, p)

    def add_key(self, t: float, kind: str, tol: float) -> float:
        """Insert key point t; snap to an existing key within tol."""
        t = min(max(t, 0.0), self.length)
        for k in list(self.keys):
            if abs(k - t) < tol:
                if _KEY_RANK[kind] > _KEY_RANK[self.keys[k]]:
                    self.keys[k] = kind
                return k
        self.keys[t] = kind
        return t

    def node_near(self, t: float) -> int:
        best = min(self.node_at, key=lambda k: abs(k - t))
        return self.node_at[best]


def _segment_intersection(a0: Vec, a1: Vec, b0: Vec, b1: Vec,
                          eps: float = 1e-6) -> Optional[Tuple[float, float]]:
    """Return (ta, tb) as fractions along both segments, or None."""
    rx, ry = a1[0] - a0[0], a1[1] - a0[1]
    sx, sy = b1[0] - b0[0], b1[1] - b0[1]
    den = rx * sy - ry * sx
    if abs(den) < 1e-12:
        return None  # parallel lanes never get connected implicitly
    qpx, qpy = b0[0] - a0[0], b0[1] - a0[1]
    ta = (qpx * sy - qpy * sx) / den
    tb = (qpx * ry - qpy * rx) / den
    if -eps <= ta <= 1 + eps and -eps <= tb <= 1 + eps:
        return min(max(ta, 0.0), 1.0), min(max(tb, 0.0), 1.0)
    return None


def _rect_distance(p: Vec, rect: Sequence[float]) -> float:
    x0, y0, x1, y1 = rect
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


class LayoutBuilder:
    def __init__(self, layout: dict) -> None:
        self.layout = layout
        tr = layout.get('traffic', {})
        self.lane_offset = float(tr.get('lane_offset', 1.0))
        self.spacing = float(tr.get('node_spacing', 2.0))
        self.bay_offset = float(tr.get('bay_offset', 2.0))
        self.dock_offset = float(tr.get('dock_offset', 2.5))
        self.conflict_radius = float(tr.get('conflict_radius', 1.3))
        self.sweep_radius = float(tr.get('sweep_radius', 1.1))
        self.lane_speed = float(tr.get('lane_speed', 1.0))
        self.bay_speed = float(tr.get('bay_speed', 0.45))
        self.two_way_factor = float(tr.get('two_way_cost_factor', 1.35))
        self.yield_spacing = float(tr.get('yield_bay_spacing', 14.0))
        self.yield_clearance = float(tr.get('yield_bay_clearance', 0.75))
        self.size = layout.get('factory', {}).get('size', [100.0, 60.0])
        self.merge_tol = 0.6
        self.lanes: Dict[str, Lane] = {}
        self.graph = TrafficGraph()
        self.graph.two_way_cost_factor = self.two_way_factor
        self.warnings: List[str] = []

    # -------------------------------------------------------------- lanes
    def _make_lanes(self) -> None:
        for a in self.layout.get('aisles', []):
            p0 = tuple(map(float, a['from']))
            p1 = tuple(map(float, a['to']))
            name = a.get('name', a['id'])
            L = math.dist(p0, p1)
            d = ((p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L)
            r = (d[1], -d[0])
            ys = a.get('yield_side', 'right')
            sp = a.get('yield_bay_spacing')
            if a.get('type', 'dual') == 'dual':
                o = self.lane_offset
                fwd = Lane(f"{a['id']}+", f'{name} (+)',
                           (p0[0] + r[0] * o, p0[1] + r[1] * o),
                           (p1[0] + r[0] * o, p1[1] + r[1] * o), False, a['id'], ys, sp)
                bwd = Lane(f"{a['id']}-", f'{name} (-)',
                           (p1[0] - r[0] * o, p1[1] - r[1] * o),
                           (p0[0] - r[0] * o, p0[1] - r[1] * o), False, a['id'], ys, sp)
                self.lanes[fwd.id] = fwd
                self.lanes[bwd.id] = bwd
            else:
                ln = Lane(a['id'], name, p0, p1, True, a['id'], ys, sp)
                self.lanes[ln.id] = ln
        for ln in self.lanes.values():
            ln.add_key(0.0, 'end', self.merge_tol)
            ln.add_key(ln.length, 'end', self.merge_tol)
            self.graph.lane_names[ln.id] = ln.name
            self.graph.lane_two_way[ln.id] = ln.two_way

    def _cross_lanes(self) -> None:
        lanes = list(self.lanes.values())
        self.crossings: List[Tuple[str, float, str, float]] = []
        for i, a in enumerate(lanes):
            for b in lanes[i + 1:]:
                hit = _segment_intersection(a.p0, a.p1, b.p0, b.p1)
                if hit is None:
                    continue
                ta = a.add_key(hit[0] * a.length, 'cross', self.merge_tol)
                tb = b.add_key(hit[1] * b.length, 'cross', self.merge_tol)
                self.crossings.append((a.id, ta, b.id, tb))

    # -------------------------------------------------------- attachments
    def _lane(self, lane_id: str) -> Lane:
        if lane_id not in self.lanes:
            raise ValueError(f'unknown lane "{lane_id}" (lanes: {sorted(self.lanes)})')
        return self.lanes[lane_id]

    def _plan_attachments(self) -> None:
        self.layby_specs = []
        for st in self.layout.get('stations', []):
            ln = self._lane(st['lane'])
            t = ln.project(st['at'])
            keys = {
                'entry': ln.add_key(t - 2 * self.bay_offset, 'attach', self.merge_tol),
                'q': ln.add_key(t - self.bay_offset, 'attach', self.merge_tol),
                'at': ln.add_key(t, 'attach', self.merge_tol),
                'exit': ln.add_key(t + self.bay_offset, 'attach', self.merge_tol),
            }
            self.layby_specs.append((st, ln, t, keys))
        self.dock_specs = []
        for kind, items in ((CHARGER, self.layout.get('chargers', [])),
                            (PARKING, self.layout.get('parking', []))):
            for it in items:
                ln = self._lane(it['lane'])
                t = ln.project(it['at'])
                tk = ln.add_key(t, 'attach', self.merge_tol)
                self.dock_specs.append((kind, it, ln, tk))

    def _special_points(self) -> List[Vec]:
        pts: List[Vec] = []
        for st, ln, t, _ in self.layby_specs:
            r = ln.right
            for tt in (t, t - self.bay_offset):
                p = ln.point(tt)
                pts.append((p[0] + r[0] * self.bay_offset, p[1] + r[1] * self.bay_offset))
        for kind, it, ln, t in self.dock_specs:
            p = ln.point(t)
            r = ln.right
            pts.append((p[0] + r[0] * self.dock_offset, p[1] + r[1] * self.dock_offset))
        return pts

    def _plan_yield_bays(self) -> None:
        self.yield_specs = []
        explicit = self.layout.get('yield_bays', []) or []
        objects = static_objects(self.layout)
        W, H = self.size
        specials = self._special_points()
        placed: List[Vec] = []

        def ok_point(p: Vec, lane: Lane) -> bool:
            if not (1.0 <= p[0] <= W - 1.0 and 1.0 <= p[1] <= H - 1.0):
                return False
            for o in objects:
                if _rect_distance(p, o['rect']) < self.yield_clearance:
                    return False
            for s in specials:
                if math.dist(s, p) < 3.5:
                    return False
            for s in placed:
                if math.dist(s, p) < 4.5:
                    return False
            for other in self.lanes.values():
                if other.id != lane.id and other.distance_to(p) < 1.9:
                    return False
            return True

        for spec in explicit:
            ln = self._lane(spec['lane'])
            t = ln.project(spec['at'])
            side = spec.get('side', 'right')
            self.yield_specs.append((ln, t, side))
            placed.append(self._bay_point(ln, t, side))

        for ln in self.lanes.values():
            side = ln.yield_side
            if side in (None, 'none', False):
                continue
            spacing = float(ln.yield_spacing or self.yield_spacing)
            cross_ts = [k for k, kind in ln.keys.items() if kind in ('cross', 'end')]
            # lay-bys and docks are on the right side: a bay on the same side
            # must stay clear of them; on the other side its lane nodes must
            # either coincide with existing ones or keep a normal spacing
            same_side = side in ('right', 'both')
            min_gap = 2.5 if same_side else 1.9
            other_keys = [k for k, kind in ln.keys.items() if kind == 'attach']
            last = -math.inf
            t = 2 * self.bay_offset + 1.0
            while t <= ln.length - 2 * self.bay_offset - 1.0:
                good = (t - last >= spacing
                        and all(abs(t - c) >= 2 * self.bay_offset + 0.5 for c in cross_ts)
                        and all(min(abs(t + o - k) for o in (-self.bay_offset, 0.0, self.bay_offset))
                                >= min_gap
                                or (not same_side and all(
                                    abs(t + o - k) < 0.05 or abs(t + o - k) >= min_gap
                                    for o in (-self.bay_offset, 0.0, self.bay_offset)))
                                for k in other_keys))
                if good:
                    sides = ['right', 'left'] if side == 'both' else [side]
                    for sd in sides:
                        p = self._bay_point(ln, t, sd)
                        if ok_point(p, ln):
                            self.yield_specs.append((ln, t, sd))
                            placed.append(p)
                            last = t
                t += 0.5

        for ln, t, side in self.yield_specs:
            for o in (-self.bay_offset, 0.0, self.bay_offset):
                ln.add_key(t + o, 'attach', self.merge_tol)

    def _bay_point(self, ln: Lane, t: float, side: str) -> Vec:
        p = ln.point(t)
        r = ln.right
        s = 1.0 if side == 'right' else -1.0
        return (p[0] + s * r[0] * self.bay_offset, p[1] + s * r[1] * self.bay_offset)

    # ------------------------------------------------------------- nodes
    def _make_lane_nodes(self) -> None:
        g = self.graph
        coord_index: Dict[Tuple[int, int], int] = {}

        def key(p: Vec) -> Tuple[int, int]:
            return (int(round(p[0] * 100)), int(round(p[1] * 100)))

        for ln in self.lanes.values():
            keys = sorted(ln.keys)
            ts: List[float] = []
            for a, b in zip(keys, keys[1:]):
                n = max(1, math.ceil((b - a) / self.spacing - 1e-9))
                ts.extend(a + (b - a) * i / n for i in range(n))
            ts.append(keys[-1])
            prev = None
            for i, t in enumerate(ts):
                p = ln.point(t)
                k = key(p)
                if k in coord_index:
                    idx = coord_index[k]
                    node = g.nodes[idx]
                    if ln.id not in node.lanes:
                        node.lanes.append(ln.id)
                    node.kind = INTERSECTION
                else:
                    idx = g.add_node(f'{ln.id}.{i:02d}', p[0], p[1], LANE, ln.id)
                    coord_index[k] = idx
                ln.node_at[t] = idx
                if prev is not None:
                    g.add_edge(prev, idx, self.lane_speed, ln.id, ln.two_way)
                    if ln.two_way:
                        g.add_edge(idx, prev, self.lane_speed, ln.id, ln.two_way)
                prev = idx
        # readable names for intersections
        for n in g.nodes:
            if n.kind == INTERSECTION:
                new_id = 'X.' + '/'.join(sorted(n.lanes))
                if new_id in g.index:
                    new_id = f'{new_id}#{n.idx}'
                del g.index[n.id]
                n.id = new_id
                g.index[new_id] = n.idx

    def _make_laybys(self) -> None:
        g = self.graph
        for st, ln, t, keys in self.layby_specs:
            r = ln.right
            sid = st['id']
            heading = ln.heading
            ps = ln.point(keys['at'])
            pq = ln.point(keys['q'])
            s = g.add_node(sid, ps[0] + r[0] * self.bay_offset, ps[1] + r[1] * self.bay_offset,
                           STATION, group=sid, heading=heading)
            q = g.add_node(f'{sid}.q', pq[0] + r[0] * self.bay_offset,
                           pq[1] + r[1] * self.bay_offset, QUEUE, group=sid, heading=heading)
            entry = ln.node_near(keys['entry'])
            exit_ = ln.node_near(keys['exit'])
            if g.nodes[entry].kind == INTERSECTION or g.nodes[exit_].kind == INTERSECTION:
                self.warnings.append(f'station {sid}: lay-by touches an intersection')
            # one-way even on a two-way lane: a robot coming from the other side
            # passes the lay-by and turns into Q, it never waits on the lane
            g.add_edge(entry, q, self.bay_speed)
            g.add_edge(q, s, self.bay_speed)
            g.add_edge(s, exit_, self.bay_speed)
            g.stations[sid] = StationNodes(sid, s, q, entry, exit_, heading)

    def _make_docks(self) -> None:
        g = self.graph
        for kind, it, ln, t in self.dock_specs:
            p = ln.point(t)
            r = ln.right
            heading = math.atan2(r[1], r[0])
            idx = g.add_node(it['id'], p[0] + r[0] * self.dock_offset,
                             p[1] + r[1] * self.dock_offset, kind,
                             group=it['id'], heading=heading)
            lane_node = ln.node_near(t)
            g.add_edge(lane_node, idx, self.bay_speed)
            g.add_edge(idx, lane_node, self.bay_speed)
            if kind == CHARGER:
                g.chargers[it['id']] = idx
            else:
                g.parking[it['id']] = idx

    def _make_yield_bays(self) -> None:
        g = self.graph
        for i, (ln, t, side) in enumerate(self.yield_specs):
            p = self._bay_point(ln, t, side)
            yid = f'Y.{ln.id}.{i:02d}'
            idx = g.add_node(yid, p[0], p[1], YIELD, group=yid, heading=ln.heading)
            a = ln.node_near(t - self.bay_offset)
            b = ln.node_near(t + self.bay_offset)
            g.add_edge(a, idx, self.bay_speed)
            g.add_edge(idx, b, self.bay_speed)
            if ln.two_way:
                g.add_edge(b, idx, self.bay_speed)
                g.add_edge(idx, a, self.bay_speed)
            g.yield_bays.append(idx)

    # -------------------------------------------------------------- build
    def build(self) -> TrafficGraph:
        self._make_lanes()
        self._cross_lanes()
        self._plan_attachments()
        self._plan_yield_bays()
        self._make_lane_nodes()
        self._make_laybys()
        self._make_docks()
        self._make_yield_bays()
        self.graph.finalize(self.conflict_radius, self.sweep_radius)
        for n in self.graph.nodes:
            if not self.graph.out_edges[n.idx] or not self.graph.in_edges[n.idx]:
                self.warnings.append(f'node {n.id} is a dead end (no in- or out-edge)')
        return self.graph


def build_graph(layout: dict) -> TrafficGraph:
    builder = LayoutBuilder(layout)
    graph = builder.build()
    graph.build_warnings = builder.warnings  # type: ignore[attr-defined]
    graph.lanes = builder.lanes              # type: ignore[attr-defined]
    return graph


def station_objects(layout: dict, graph: TrafficGraph) -> List[dict]:
    """Objects standing next to every station slot (kit rack, conveyor end,
    storage table), on the far side of the lay-by.  Returned in the same
    format as layout['objects'] (axis aligned rect) plus 'yaw' and 'station'."""
    out = []
    kinds = {'kit_rack': (0.9, 0.55, 1.2), 'conveyor_in': (1.0, 0.6, 0.55),
             'conveyor_out': (1.0, 0.6, 0.55), 'storage_table': (1.0, 0.6, 0.7)}
    lanes = getattr(graph, 'lanes', {})
    for st in layout.get('stations', []) or []:
        sn = graph.stations.get(st['id'])
        ln = lanes.get(st['lane'])
        if sn is None or ln is None:
            continue
        along, across, h = kinds.get(st.get('object', 'storage_table'), (1.0, 0.6, 0.7))
        s = graph.nodes[sn.service]
        r = ln.right
        cx = s.x + r[0] * (0.75 + across / 2)
        cy = s.y + r[1] * (0.75 + across / 2)
        d = ln.dir
        hx = abs(d[0]) * along / 2 + abs(r[0]) * across / 2
        hy = abs(d[1]) * along / 2 + abs(r[1]) * across / 2
        out.append({'kind': st.get('object', 'storage_table'), 'rect': [cx - hx, cy - hy, cx + hx, cy + hy],
                    'height': h, 'yaw': math.atan2(d[1], d[0]), 'station': st['id'],
                    'size': [along, across]})
    return out


def all_objects(layout: dict, graph: TrafficGraph) -> List[dict]:
    return static_objects(layout) + station_objects(layout, graph)


def graph_to_dict(graph: TrafficGraph) -> dict:
    """Static description of the graph for the dashboard / debugging."""
    nodes = [{'id': n.id, 'x': round(n.x, 3), 'y': round(n.y, 3), 'kind': n.kind,
              'group': n.group} for n in graph.nodes]
    edges = []
    for es in graph.out_edges:
        for e in es:
            if e.two_way and e.u > e.v and graph.edge(e.v, e.u) is not None:
                continue
            edges.append({'u': e.u, 'v': e.v, 'lane': e.lane, 'two_way': e.two_way})
    return {'nodes': nodes, 'edges': edges,
            'lanes': {k: {'name': v, 'two_way': graph.lane_two_way.get(k, False)}
                      for k, v in graph.lane_names.items()}}
