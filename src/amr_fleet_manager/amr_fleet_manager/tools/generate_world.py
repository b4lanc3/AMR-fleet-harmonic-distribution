"""Generate the Gazebo (Harmonic / Jetty) SDF world from factory_layout.yaml.

  ros2 run amr_fleet_manager generate_world --layout factory_layout.yaml --out factory.sdf

Everything in the hall is built from primitives so the world has no external
model dependencies: walls, floor zones and lane markings, component racks,
production lines (conveyors, machines, light towers), packaging machines,
finished goods racks, building columns, station conveyors / kit racks and the
5-dock automatic charging station.
"""

from __future__ import annotations

import argparse
import math
from typing import List, Sequence
from xml.sax.saxutils import escape

from ..core.layout import build_graph, load_layout, station_objects

# ---------------------------------------------------------------- helpers
_MAT = {
    'floor': (0.62, 0.63, 0.62, 1), 'wall': (0.80, 0.81, 0.83, 1), 'steel': (0.35, 0.37, 0.40, 1),
    'rack_up': (0.10, 0.30, 0.65, 1), 'rack_beam': (0.95, 0.45, 0.05, 1), 'shelf': (0.72, 0.72, 0.70, 1),
    'carton': (0.72, 0.55, 0.33, 1), 'bin_blue': (0.15, 0.40, 0.80, 1), 'bin_red': (0.80, 0.20, 0.15, 1),
    'belt': (0.08, 0.08, 0.09, 1), 'machine': (0.88, 0.89, 0.90, 1), 'machine_accent': (0.95, 0.55, 0.10, 1),
    'packer': (0.55, 0.40, 0.80, 1), 'glass': (0.55, 0.75, 0.90, 0.5), 'pallet': (0.62, 0.45, 0.25, 1),
    'column': (0.70, 0.70, 0.68, 1), 'charger': (0.95, 0.80, 0.10, 1), 'screen': (0.05, 0.10, 0.15, 1),
    'green': (0.10, 0.75, 0.25, 1), 'red': (0.85, 0.15, 0.10, 1), 'amber': (1.0, 0.70, 0.0, 1),
    'lane': (0.98, 0.85, 0.10, 1), 'lane_two': (0.98, 0.50, 0.05, 1), 'pad_station': (0.10, 0.70, 0.30, 1),
    'pad_dock': (0.98, 0.85, 0.10, 1), 'pad_park': (0.92, 0.92, 0.92, 1), 'pad_yield': (1.0, 0.55, 0.0, 1),
    'black': (0.1, 0.1, 0.1, 1),
}


def _mat(name_or_rgba) -> str:
    r, g, b, a = _MAT[name_or_rgba] if isinstance(name_or_rgba, str) else name_or_rgba
    return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>'
            f'<diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


class Model:
    def __init__(self, name: str) -> None:
        self.name = name
        self.visuals: List[str] = []
        self.collisions: List[str] = []
        self.k = 0

    def box(self, cx, cy, cz, sx, sy, sz, mat, yaw=0.0, collide=False, transparency=0.0):
        self.k += 1
        pose = f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>'
        geo = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
        tr = f'<transparency>{transparency}</transparency>' if transparency else ''
        self.visuals.append(f'<visual name="v{self.k}">{pose}{geo}{_mat(mat)}{tr}'
                            f'<cast_shadows>false</cast_shadows></visual>')
        if collide:
            self.collisions.append(f'<collision name="c{self.k}">{pose}{geo}</collision>')

    def cyl(self, cx, cy, cz, r, h, mat, collide=False):
        self.k += 1
        pose = f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>'
        geo = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>'
        self.visuals.append(f'<visual name="v{self.k}">{pose}{geo}{_mat(mat)}</visual>')
        if collide:
            self.collisions.append(f'<collision name="c{self.k}">{pose}{geo}</collision>')

    def collision_box(self, cx, cy, cz, sx, sy, sz):
        self.k += 1
        self.collisions.append(
            f'<collision name="c{self.k}"><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>')

    def sdf(self) -> str:
        if not self.visuals and not self.collisions:
            return ''
        return (f'<model name="{escape(self.name)}"><static>true</static><link name="link">'
                + ''.join(self.collisions) + ''.join(self.visuals) + '</link></model>\n')


def _rect(o):
    x0, y0, x1, y1 = o['rect']
    return x0, y0, x1, y1, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0


# ------------------------------------------------------------- furniture
def add_rack(m: Model, o, idx: int) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 3.0))
    m.collision_box(cx, cy, h / 2, sx, sy, h)
    along_x = sx >= sy
    L = sx if along_x else sy
    D = sy if along_x else sx
    bays = max(1, int(round(L / 2.5)))
    for i in range(bays + 1):
        t = -L / 2 + L * i / bays
        for side in (-1, 1):
            px = cx + (t if along_x else side * (D / 2 - 0.05))
            py = cy + (side * (D / 2 - 0.05) if along_x else t)
            m.box(px, py, h / 2, 0.08, 0.08, h, 'rack_up')
    levels = [0.15, h * 0.37, h * 0.68, h - 0.05]
    for z in levels:
        m.box(cx, cy, z, sx, sy, 0.05, 'shelf')
        for side in (-1, 1):
            px = cx if along_x else cx + side * (D / 2 - 0.05)
            py = cy + side * (D / 2 - 0.05) if along_x else cy
            m.box(px, py, z + 0.06, sx if along_x else 0.06, 0.06 if along_x else sy, 0.10, 'rack_beam')
    # cartons / bins on the shelves (deterministic pattern)
    for li, z in enumerate(levels[:-1]):
        for i in range(bays):
            if (i * 7 + li * 3 + idx) % 5 == 0:
                continue
            t = -L / 2 + L * (i + 0.5) / bays
            px, py = (cx + t, cy) if along_x else (cx, cy + t)
            w = L / bays * 0.75
            hh = 0.35 + 0.15 * ((i + li) % 3)
            mat = 'carton' if (i + li + idx) % 3 else ('bin_blue' if li % 2 else 'bin_red')
            m.box(px, py, z + 0.03 + hh / 2, w if along_x else D * 0.8, D * 0.8 if along_x else w, hh, mat)


def add_conveyor(m: Model, o) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 0.9))
    m.collision_box(cx, cy, h / 2, sx, sy, h)
    along_x = sx >= sy
    L = max(sx, sy)
    m.box(cx, cy, h - 0.03, sx, sy, 0.06, 'belt')
    for side in (-1, 1):
        if along_x:
            m.box(cx, cy + side * (sy / 2 - 0.03), h + 0.02, sx, 0.05, 0.08, 'steel')
        else:
            m.box(cx + side * (sx / 2 - 0.03), cy, h + 0.02, 0.05, sy, 0.08, 'steel')
    legs = max(2, int(L / 2.0))
    for i in range(legs + 1):
        t = -L / 2 + 0.1 + (L - 0.2) * i / legs
        for side in (-1, 1):
            px = cx + t if along_x else cx + side * (sx / 2 - 0.06)
            py = cy + side * (sy / 2 - 0.06) if along_x else cy + t
            m.box(px, py, (h - 0.06) / 2, 0.06, 0.06, h - 0.06, 'steel')
    # products travelling on the belt
    n = int(L / 1.6)
    for i in range(n):
        t = -L / 2 + 0.8 + 1.6 * i
        px, py = (cx + t, cy) if along_x else (cx, cy + t)
        m.box(px, py, h + 0.12, 0.35, 0.35, 0.18, 'carton')


def add_machine(m: Model, o, accent='machine_accent', body='machine') -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 2.0))
    m.box(cx, cy, h / 2, sx, sy, h, body, collide=True)
    m.box(cx, cy, h + 0.03, sx * 0.9, sy * 0.9, 0.06, accent)
    m.box(cx + sx / 2 + 0.01, cy, h * 0.55, 0.02, sy * 0.5, h * 0.35, 'glass')
    m.box(cx - sx / 2 - 0.01, cy, h * 0.55, 0.02, sy * 0.5, h * 0.35, 'glass')
    # light tower
    tx, ty = x1 - 0.2, y1 - 0.2
    for i, mat in enumerate(('green', 'amber', 'red')):
        m.cyl(tx, ty, h + 0.12 + i * 0.1, 0.05, 0.09, mat)


def add_pallet(m: Model, o) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 1.0))
    m.box(cx, cy, 0.07, sx, sy, 0.14, 'pallet', collide=True)
    m.box(cx, cy, 0.14 + (h - 0.14) / 2, sx * 0.95, sy * 0.95, h - 0.14, 'carton', collide=True)


def add_charger(m: Model, o) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 1.1))
    m.box(cx, cy, h / 2, sx, sy, h, 'charger', collide=True)
    m.box(cx, cy, h * 0.7, sx * 1.02, sy * 0.6, h * 0.25, 'screen')
    m.cyl(cx, cy, h + 0.08, 0.05, 0.12, 'green')


def add_station_object(m: Model, o) -> None:
    along, across = o['size']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 0.6))
    yaw = o['yaw']
    kind = o['kind']
    m.collision_box(cx, cy, h / 2, sx, sy, h)
    if kind == 'kit_rack':
        for z in (0.1, 0.55, 1.0):
            m.box(cx, cy, z, along, across, 0.04, 'shelf', yaw)
            m.box(cx, cy, z + 0.12, along * 0.8, across * 0.7, 0.2,
                  'bin_blue' if z < 0.5 else 'bin_red', yaw)
        for sxs in (-1, 1):
            for sys_ in (-1, 1):
                dx = sxs * (along / 2 - 0.03)
                dy = sys_ * (across / 2 - 0.03)
                px = cx + dx * math.cos(yaw) - dy * math.sin(yaw)
                py = cy + dx * math.sin(yaw) + dy * math.cos(yaw)
                m.box(px, py, h / 2, 0.04, 0.04, h, 'rack_up', yaw)
    else:
        top = 'belt' if kind.startswith('conveyor') else 'shelf'
        m.box(cx, cy, h / 2, along, across, h, 'steel', yaw)
        m.box(cx, cy, h + 0.02, along, across, 0.04, top, yaw)
        if kind == 'conveyor_out' or kind == 'storage_table':
            m.box(cx, cy, h + 0.17, 0.45, 0.4, 0.26, 'carton', yaw)


# ----------------------------------------------------------------- world
def world_sdf(layout: dict, step: float = 0.004, rtf: float = 1.0) -> str:
    graph = build_graph(layout)
    W, H = layout.get('factory', {}).get('size', [100.0, 60.0])
    wall_h = float(layout.get('factory', {}).get('wall_height', 6.0))
    wt = float(layout.get('factory', {}).get('wall_thickness', 0.3))
    out: List[str] = []
    out.append(f'''<?xml version="1.0"?>
<!-- generated by amr_fleet_manager/generate_world from factory_layout.yaml - do not edit by hand -->
<sdf version="1.9">
<world name="amr_factory">
  <physics name="fleet" type="ignored">
    <max_step_size>{step}</max_step_size>
    <real_time_factor>{rtf}</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <gravity>0 0 -9.81</gravity>
  <scene>
    <ambient>0.75 0.75 0.75 1</ambient>
    <background>0.70 0.75 0.80 1</background>
    <shadows>false</shadows>
    <grid>false</grid>
  </scene>
  <light type="directional" name="sun">
    <cast_shadows>false</cast_shadows>
    <pose>{W / 2} {H / 2} 20 0 0 0</pose>
    <diffuse>0.85 0.85 0.85 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.3 0.2 -0.9</direction>
  </light>
  <light type="directional" name="fill">
    <cast_shadows>false</cast_shadows>
    <pose>{W / 2} {H / 2} 20 0 0 0</pose>
    <diffuse>0.35 0.35 0.38 1</diffuse>
    <specular>0 0 0 1</specular>
    <direction>0.4 -0.3 -0.8</direction>
  </light>
  <gui fullscreen="0">
    <camera name="user_camera">
      <pose>{W / 2} {-18} 42 0 0.85 1.5708</pose>
    </camera>
  </gui>
''')
    # floor
    floor = Model('factory_floor')
    floor.collisions.append(f'<collision name="plane"><geometry><plane><normal>0 0 1</normal>'
                            f'<size>{W + 40} {H + 40}</size></plane></geometry>'
                            f'<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
                            f'</surface></collision>')
    floor.box(W / 2, H / 2, -0.05, W + 40, H + 40, 0.1, 'floor')
    out.append(floor.sdf())

    # walls
    walls = Model('walls')
    for cx, cy, sx, sy in ((W / 2, 0, W + wt, wt), (W / 2, H, W + wt, wt),
                           (0, H / 2, wt, H + wt), (W, H / 2, wt, H + wt)):
        walls.box(cx, cy, wall_h / 2, sx, sy, wall_h, 'wall', collide=True)
    out.append(walls.sdf())

    # floor markings: zones, lanes, pads
    mk = Model('floor_markings')
    for z in layout.get('zones', []) or []:
        x0, y0, x1, y1 = z['rect']
        r, g, b = z.get('color', [0.5, 0.5, 0.5])
        mk.box((x0 + x1) / 2, (y0 + y1) / 2, 0.001, x1 - x0, y1 - y0, 0.002,
               (0.62 + 0.25 * (r - 0.5), 0.63 + 0.25 * (g - 0.5), 0.62 + 0.25 * (b - 0.5), 1))
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x0, y1, x1, y1), (x0, y0, x0, y1), (x1, y0, x1, y1)):
            L = math.hypot(bx - ax, by - ay)
            mk.box((ax + bx) / 2, (ay + by) / 2, 0.003, L if ay == by else 0.08,
                   0.08 if ay == by else L, 0.002, (r, g, b, 1))
    lanes = getattr(graph, 'lanes', {})
    for ln in lanes.values():
        (ax, ay), (bx, by) = ln.p0, ln.p1
        L = ln.length
        yaw = math.atan2(by - ay, bx - ax)
        mat = 'lane_two' if ln.two_way else 'lane'
        if ln.two_way:
            n = int(L / 1.5)
            for i in range(n):
                t = (i + 0.5) * L / n
                mk.box(ax + (bx - ax) * t / L, ay + (by - ay) * t / L, 0.004, 0.9, 0.10, 0.002, mat, yaw)
        else:
            mk.box((ax + bx) / 2, (ay + by) / 2, 0.004, L, 0.08, 0.002, mat, yaw)
            # direction chevrons every 6 m
            for i in range(1, int(L / 6)):
                t = i * 6.0
                px, py = ax + (bx - ax) * t / L, ay + (by - ay) * t / L
                for s in (-1, 1):
                    mk.box(px - 0.15 * math.cos(yaw) - s * 0.12 * math.sin(yaw),
                           py - 0.15 * math.sin(yaw) + s * 0.12 * math.cos(yaw),
                           0.004, 0.40, 0.07, 0.002, mat, yaw + s * 0.7)
    pads = [(graph.nodes[s.service], 'pad_station', s.heading) for s in graph.stations.values()]
    pads += [(graph.nodes[s.queue], 'pad_station', s.heading) for s in graph.stations.values() if s.queue is not None]
    pads += [(graph.nodes[i], 'pad_dock', graph.nodes[i].heading or 0.0) for i in graph.chargers.values()]
    pads += [(graph.nodes[i], 'pad_park', graph.nodes[i].heading or 0.0) for i in graph.parking.values()]
    pads += [(graph.nodes[i], 'pad_yield', graph.nodes[i].heading or 0.0) for i in graph.yield_bays]
    for n, mat, yaw in pads:
        # outline of a 1.0 x 0.8 m pad
        for dx, dy, sx, sy in ((0.5, 0, 0.06, 0.8), (-0.5, 0, 0.06, 0.8), (0, 0.4, 1.0, 0.06), (0, -0.4, 1.0, 0.06)):
            px = n.x + dx * math.cos(yaw) - dy * math.sin(yaw)
            py = n.y + dx * math.sin(yaw) + dy * math.cos(yaw)
            mk.box(px, py, 0.005, sx, sy, 0.002, mat, yaw)
    out.append(mk.sdf())

    # furniture
    for i, o in enumerate(layout.get('objects', []) or []):
        m = Model(f'{o.get("kind", "object")}_{i:02d}' + (f'_{o["line"]}' if o.get('line') else ''))
        kind = o.get('kind')
        if kind == 'rack':
            add_rack(m, o, i)
        elif kind == 'conveyor':
            add_conveyor(m, o)
        elif kind == 'machine':
            add_machine(m, o)
        elif kind == 'packer':
            add_machine(m, o, accent='packer', body='machine')
        elif kind == 'pallet':
            add_pallet(m, o)
        elif kind == 'charger':
            add_charger(m, o)
        else:
            x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
            h = float(o.get('height', 1.0))
            m.box(cx, cy, h / 2, sx, sy, h, 'column', collide=True)
        out.append(m.sdf())
    for o in station_objects(layout, graph):
        m = Model(f'station_{o["station"]}_{o["kind"]}')
        add_station_object(m, o)
        out.append(m.sdf())
    out.append('</world>\n</sdf>\n')
    return ''.join(out)


def main(argv: Sequence[str] = None) -> None:
    ap = argparse.ArgumentParser(description='Generate the Gazebo factory world from the layout')
    ap.add_argument('--layout', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--step', type=float, default=0.004, help='physics max step size [s]')
    ap.add_argument('--rtf', type=float, default=1.0, help='target real time factor')
    args = ap.parse_args(argv)
    sdf = world_sdf(load_layout(args.layout), args.step, args.rtf)
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(sdf)
    print(f'wrote {args.out} ({len(sdf) / 1024:.0f} kB)')


if __name__ == '__main__':
    main()
