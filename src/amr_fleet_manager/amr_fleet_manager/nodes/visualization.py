"""RViz markers and occupancy grid generated from the layout + fleet state."""

from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..core.graph import CHARGER, INTERSECTION, PARKING, QUEUE, STATION, YIELD, TrafficGraph
from ..core.colors import robot_rgb
from ..core.layout import all_objects

OBJECT_COLORS = {
    'rack': (0.30, 0.45, 0.70), 'conveyor': (0.25, 0.25, 0.28), 'machine': (0.85, 0.50, 0.15),
    'packer': (0.55, 0.30, 0.75), 'pallet': (0.65, 0.50, 0.30), 'column': (0.6, 0.6, 0.6),
    'charger': (0.95, 0.80, 0.10), 'kit_rack': (0.20, 0.45, 0.85), 'line_in': (0.95, 0.52, 0.10),
    'line_out': (0.10, 0.70, 0.30), 'storage_table': (0.55, 0.55, 0.50),
}


def robot_color(i: int, n: int) -> Tuple[float, float, float]:
    return robot_rgb(i)


def _c(rgb, a=1.0) -> ColorRGBA:
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def _p(x, y, z=0.0) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def _marker(ns: str, mid: int, mtype: int, frame: str, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 1.0
    return m


def static_markers(layout: dict, graph: TrafficGraph, frame: str, stamp) -> MarkerArray:
    arr = MarkerArray()
    mid = 0
    W, H = layout.get('factory', {}).get('size', [100.0, 60.0])
    # floor
    m = _marker('floor', mid, Marker.CUBE, frame, stamp)
    m.pose.position = _p(W / 2, H / 2, -0.10)
    m.scale.x, m.scale.y, m.scale.z = W, H, 0.04
    m.color = _c((0.82, 0.82, 0.80), 1.0)
    arr.markers.append(m)
    # walls
    t = layout.get('factory', {}).get('wall_thickness', 0.3)
    wh = 2.0
    for i, (cx, cy, sx, sy) in enumerate([(W / 2, 0, W, t), (W / 2, H, W, t), (0, H / 2, t, H), (W, H / 2, t, H)]):
        m = _marker('walls', i, Marker.CUBE, frame, stamp)
        m.pose.position = _p(cx, cy, wh / 2)
        m.scale.x, m.scale.y, m.scale.z = sx, sy, wh
        m.color = _c((0.55, 0.57, 0.60), 0.6)
        arr.markers.append(m)
    # zones
    for i, z in enumerate(layout.get('zones', [])):
        x0, y0, x1, y1 = z['rect']
        col = z.get('color', [0.5, 0.5, 0.5])
        m = _marker('zones', i, Marker.CUBE, frame, stamp)
        m.pose.position = _p((x0 + x1) / 2, (y0 + y1) / 2, -0.04)
        m.scale.x, m.scale.y, m.scale.z = x1 - x0, y1 - y0, 0.02
        m.color = _c(col, 0.35)
        arr.markers.append(m)
        tm = _marker('zone_labels', i, Marker.TEXT_VIEW_FACING, frame, stamp)
        tm.pose.position = _p((x0 + x1) / 2, (y0 + y1) / 2, 4.5)
        tm.scale.z = 1.6
        tm.color = _c(col, 1.0)
        tm.text = z.get('name', z['id'])
        arr.markers.append(tm)
    # objects (layout objects + kit racks / line work cells next to the stations)
    for i, o in enumerate(all_objects(layout, graph)):
        x0, y0, x1, y1 = o['rect']
        h = float(o.get('height', 1.0))
        m = _marker('objects', i, Marker.CUBE, frame, stamp)
        m.pose.position = _p((x0 + x1) / 2, (y0 + y1) / 2, h / 2)
        m.scale.x, m.scale.y, m.scale.z = x1 - x0, y1 - y0, h
        m.color = _c(OBJECT_COLORS.get(o.get('kind'), (0.5, 0.5, 0.5)), 0.85)
        arr.markers.append(m)
    # lanes
    one = _marker('lanes', 0, Marker.LINE_LIST, frame, stamp)
    one.scale.x = 0.06
    one.color = _c((0.15, 0.45, 0.95), 0.9)
    two = _marker('lanes', 1, Marker.LINE_LIST, frame, stamp)
    two.scale.x = 0.10
    two.color = _c((1.0, 0.55, 0.0), 0.95)
    bay = _marker('lanes', 2, Marker.LINE_LIST, frame, stamp)
    bay.scale.x = 0.04
    bay.color = _c((0.3, 0.3, 0.3), 0.8)
    heads = _marker('lane_arrows', 0, Marker.TRIANGLE_LIST, frame, stamp)
    heads.color = _c((0.15, 0.45, 0.95), 0.9)
    done = set()
    for es in graph.out_edges:
        for e in es:
            key = (min(e.u, e.v), max(e.u, e.v))
            a, b = graph.nodes[e.u], graph.nodes[e.v]
            if key not in done:
                done.add(key)
                target = bay if e.lane is None else (two if e.two_way else one)
                target.points += [_p(a.x, a.y, 0.02), _p(b.x, b.y, 0.02)]
            if e.lane is not None and not e.two_way and e.length > 1.2:
                dx, dy = (b.x - a.x) / e.length, (b.y - a.y) / e.length
                mx, my = (a.x + b.x) / 2, (a.y + b.y) / 2
                s = 0.28
                heads.points += [_p(mx + dx * s, my + dy * s, 0.03),
                                 _p(mx - dx * s - dy * s * 0.8, my - dy * s + dx * s * 0.8, 0.03),
                                 _p(mx - dx * s + dy * s * 0.8, my - dy * s - dx * s * 0.8, 0.03)]
    arr.markers += [one, two, bay, heads]
    # nodes
    colors = {INTERSECTION: (0.9, 0.2, 0.2), STATION: (0.1, 0.8, 0.3), QUEUE: (0.5, 0.9, 0.5),
              CHARGER: (1.0, 0.85, 0.0), PARKING: (0.55, 0.55, 0.55), YIELD: (1.0, 0.6, 0.1)}
    nodes = _marker('nodes', 0, Marker.SPHERE_LIST, frame, stamp)
    nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.18
    for n in graph.nodes:
        nodes.points.append(_p(n.x, n.y, 0.05))
        nodes.colors.append(_c(colors.get(n.kind, (0.2, 0.4, 0.9)), 0.9))
    arr.markers.append(nodes)
    # station / dock pads + labels
    k = 0
    for sid, st in graph.stations.items():
        for idx, alpha in ((st.service, 0.55), (st.queue, 0.25)):
            if idx is None:
                continue
            n = graph.nodes[idx]
            m = _marker('pads', k, Marker.CUBE, frame, stamp)
            m.pose.position = _p(n.x, n.y, 0.005)
            m.pose.orientation.z = math.sin(st.heading / 2)
            m.pose.orientation.w = math.cos(st.heading / 2)
            m.scale.x, m.scale.y, m.scale.z = 1.0, 0.8, 0.01
            m.color = _c((0.1, 0.8, 0.3), alpha)
            arr.markers.append(m)
            k += 1
        n = graph.nodes[st.service]
        tm = _marker('station_labels', k, Marker.TEXT_VIEW_FACING, frame, stamp)
        tm.pose.position = _p(n.x, n.y, 1.4)
        tm.scale.z = 0.7
        tm.color = _c((0.05, 0.35, 0.1), 1.0)
        tm.text = sid
        arr.markers.append(tm)
        k += 1
    for group, kind_col in ((graph.chargers, (1.0, 0.85, 0.0)), (graph.parking, (0.5, 0.5, 0.5))):
        for did, idx in group.items():
            n = graph.nodes[idx]
            m = _marker('pads', k, Marker.CUBE, frame, stamp)
            m.pose.position = _p(n.x, n.y, 0.005)
            yaw = n.heading or 0.0
            m.pose.orientation.z = math.sin(yaw / 2)
            m.pose.orientation.w = math.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = 1.0, 0.8, 0.01
            m.color = _c(kind_col, 0.5)
            arr.markers.append(m)
            k += 1
            tm = _marker('station_labels', k, Marker.TEXT_VIEW_FACING, frame, stamp)
            tm.pose.position = _p(n.x, n.y, 1.0)
            tm.scale.z = 0.5
            tm.color = _c((0.2, 0.2, 0.2), 1.0)
            tm.text = did
            arr.markers.append(tm)
            k += 1
    return arr


def dynamic_markers(snapshot: dict, robot_index: Dict[str, int], frame: str, stamp,
                    graph: TrafficGraph) -> MarkerArray:
    arr = MarkerArray()
    life = Duration(sec=1, nanosec=0)
    n = max(len(robot_index), 1)
    for r in snapshot['robots']:
        i = robot_index.get(r['robot_id'], 0)
        col = robot_color(i, n)
        x, y, th = r['pose']['x'], r['pose']['y'], r['pose']['theta']
        # label
        tm = _marker('robot_labels', i, Marker.TEXT_VIEW_FACING, frame, stamp)
        tm.lifetime = life
        tm.pose.position = _p(x, y, 2.2)
        tm.scale.z = 0.65
        tm.color = _c(col, 1.0)
        goal = r['current_goal']
        eta = r['eta_task_s'] if r['task_id'] else r['eta_goal_s']
        line2 = f"{r['state']} -> {goal}" if goal else r['state']
        line3 = f"ETA {eta:.0f}s | {r['battery_percent']:.0f}%"
        if r['yielding_to']:
            line3 += f" | YIELD->{r['yielding_to']}"
        elif r['blocked_by']:
            line3 += f" | wait {r['blocked_by']}"
        tid = f" {r['task_id']}" if r['task_id'] else ''
        name = r['robot_id'].upper().replace('_', '-')
        tm.text = f"{name} P{r['priority_rank']}{tid}\n{line2}\n{line3}"
        arr.markers.append(tm)
        # colour disc under the robot
        d = _marker('robot_discs', i, Marker.CYLINDER, frame, stamp)
        d.lifetime = life
        d.pose.position = _p(x, y, 0.01)
        d.scale.x = d.scale.y = 1.6
        d.scale.z = 0.02
        d.color = _c(col, 0.55)
        arr.markers.append(d)
        # route (remaining) and granted part
        pts = r['route_points']
        line = _marker('routes', i, Marker.LINE_STRIP, frame, stamp)
        line.lifetime = life
        line.scale.x = 0.10
        line.color = _c(col, 0.75)
        line.points = [_p(x, y, 0.08)] + [_p(px, py, 0.08) for px, py in pts[1:]]
        if len(line.points) < 2:
            line.action = Marker.DELETE
        arr.markers.append(line)
        g = _marker('granted', i, Marker.LINE_STRIP, frame, stamp)
        g.lifetime = life
        g.scale.x = 0.28
        g.color = _c(col, 0.95)
        gc = max(0, min(r['granted_count'], len(pts)))
        g.points = [_p(x, y, 0.1)] + [_p(px, py, 0.1) for px, py in pts[1:gc]]
        if len(g.points) < 2:
            g.action = Marker.DELETE
        arr.markers.append(g)
        # goal flag
        gm = _marker('goals', i, Marker.CYLINDER, frame, stamp)
        gm.lifetime = life
        if pts:
            gx, gy = pts[-1]
            gm.pose.position = _p(gx, gy, 0.6)
            gm.scale.x = gm.scale.y = 0.25
            gm.scale.z = 1.2
            gm.color = _c(col, 0.8)
        else:
            gm.action = Marker.DELETE
        arr.markers.append(gm)
        # cargo box on top of the robot
        cm = _marker('cargo', i, Marker.CUBE, frame, stamp)
        cm.lifetime = life
        if r['cargo']:
            cm.pose.position = _p(x, y, 0.55)
            cm.pose.orientation.z = math.sin(th / 2)
            cm.pose.orientation.w = math.cos(th / 2)
            cm.scale.x, cm.scale.y, cm.scale.z = 0.6, 0.45, 0.4
            cm.color = _c((0.80, 0.55, 0.25), 0.95)
        else:
            cm.action = Marker.DELETE
        arr.markers.append(cm)
        # reserved nodes
        res = _marker('reservations', i, Marker.CUBE_LIST, frame, stamp)
        res.lifetime = life
        res.scale.x = res.scale.y = 0.45
        res.scale.z = 0.03
        res.color = _c(col, 0.6)
        for nid in r.get('held_nodes', []):
            node = graph.node(nid)
            res.points.append(_p(node.x, node.y, 0.04))
        if not res.points:
            res.action = Marker.DELETE
        arr.markers.append(res)
    # charger status
    for k, (cid, occ) in enumerate(zip(snapshot['charger_ids'], snapshot['charger_occupants'])):
        node = graph.nodes[graph.chargers[cid]]
        m = _marker('charger_status', k, Marker.CYLINDER, frame, stamp)
        m.lifetime = life
        m.pose.position = _p(node.x, node.y + 0.0, 1.3)
        m.scale.x = m.scale.y = 0.3
        m.scale.z = 0.3
        m.color = _c((0.9, 0.1, 0.1) if occ else (0.1, 0.9, 0.1), 0.9)
        arr.markers.append(m)
    return arr


def occupancy_grid(layout: dict, frame: str, stamp, resolution: float = 0.1,
                   objects=None) -> OccupancyGrid:
    W, H = layout.get('factory', {}).get('size', [100.0, 60.0])
    t = layout.get('factory', {}).get('wall_thickness', 0.3)
    nx, ny = int(math.ceil(W / resolution)) + 1, int(math.ceil(H / resolution)) + 1
    grid = np.zeros((ny, nx), dtype=np.int8)

    def fill(x0, y0, x1, y1):
        i0 = max(0, int(math.floor(x0 / resolution)))
        i1 = min(nx, int(math.ceil(x1 / resolution)))
        j0 = max(0, int(math.floor(y0 / resolution)))
        j1 = min(ny, int(math.ceil(y1 / resolution)))
        grid[j0:j1, i0:i1] = 100

    fill(0, 0, W, t / 2)
    fill(0, H - t / 2, W, H + resolution)
    fill(0, 0, t / 2, H)
    fill(W - t / 2, 0, W + resolution, H)
    for o in (objects if objects is not None else layout.get('objects', []) or []):
        fill(*o['rect'])
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.info.resolution = float(resolution)
    msg.info.width = nx
    msg.info.height = ny
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    return msg
