"""Generate the Gazebo (Harmonic / Jetty) factory world from factory_layout.yaml.

  ros2 run amr_fleet_manager generate_world --layout factory_layout.yaml \\
       --out factory.sdf --map factory_map

Everything is built from primitives (no external model downloads): an
insulated-panel hall with concrete base, window band, roller doors and steel
columns, roof beams with light fixtures and real lights, epoxy floor zones and
lane markings, component racks full of bins and cartons, pallet racks with
wrapped pallets, 4 production lines (conveyor, CNC, robot cell, assembly
bench, QC gate, case packer, control cabinet) and 4 packaging lines (unboxer,
flow wrapper, case packer, robot palletizer, output pallet), the charging
station, office annex, trucks at the docks, fire extinguishers, signs...

Products move along every conveyor through the LineAnimator system
(amr_gazebo); conveyors stop when the MES reports the line starved/blocked.

With --map the generator also writes an occupancy map (PGM + YAML) for
map_server / AMCL by slicing every visual at the 2D lidar plane, so the map
matches what the robots' lidars really see.

All opaque static parts are baked into one OBJ mesh (one sub-mesh per
material): a few dozen draw calls instead of ~5000 visuals, so every lidar
and RGB-D camera of the fleet renders the full factory cheaply.  Lamps, LEDs,
glass and stretch wrap stay SDF visuals (emissive / transparent).
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import numpy as np

from ..core.layout import build_graph, line_objects, load_layout, station_objects

LIDAR_Z = (0.16, 0.24)      # z band of the 2D lidar plane (mounted at 0.20 m)

# ----------------------------------------------------------------- materials
_MAT = {
    'asphalt': (0.22, 0.22, 0.23, 1), 'concrete': (0.66, 0.66, 0.64, 1),
    'floor': (0.70, 0.71, 0.70, 1), 'floor_joint': (0.55, 0.55, 0.54, 1),
    'wall_base': (0.58, 0.58, 0.56, 1), 'wall_panel': (0.84, 0.86, 0.88, 1),
    'wall_top': (0.30, 0.42, 0.58, 1), 'window': (0.55, 0.72, 0.88, 0.45),
    'door': (0.78, 0.80, 0.82, 1), 'door_frame': (0.95, 0.75, 0.05, 1), 'dark': (0.12, 0.12, 0.13, 1),
    'steel': (0.38, 0.40, 0.43, 1), 'steel_light': (0.62, 0.64, 0.66, 1), 'galv': (0.72, 0.74, 0.76, 1),
    'beam_blue': (0.12, 0.28, 0.55, 1), 'yellow': (0.98, 0.78, 0.05, 1), 'black': (0.06, 0.06, 0.06, 1),
    'rack_up': (0.10, 0.30, 0.65, 1), 'rack_beam': (0.95, 0.45, 0.05, 1), 'shelf': (0.72, 0.72, 0.70, 1),
    'carton': (0.74, 0.56, 0.34, 1), 'carton_dark': (0.62, 0.45, 0.26, 1), 'wrap': (0.86, 0.90, 0.95, 0.85),
    'bin_blue': (0.15, 0.40, 0.80, 1), 'bin_red': (0.80, 0.20, 0.15, 1), 'bin_green': (0.15, 0.60, 0.30, 1),
    'bin_yellow': (0.95, 0.75, 0.10, 1), 'pallet': (0.62, 0.45, 0.25, 1),
    'belt': (0.09, 0.09, 0.10, 1), 'roller': (0.55, 0.57, 0.60, 1),
    'machine': (0.90, 0.91, 0.92, 1), 'machine_dark': (0.30, 0.32, 0.35, 1),
    'machine_orange': (0.95, 0.52, 0.10, 1), 'machine_blue': (0.15, 0.38, 0.70, 1),
    'packer': (0.55, 0.40, 0.80, 1), 'glass': (0.60, 0.78, 0.92, 0.45), 'screen': (0.04, 0.12, 0.20, 1),
    'robot_orange': (0.98, 0.45, 0.05, 1), 'fence': (0.95, 0.80, 0.10, 0.55),
    'wood': (0.70, 0.52, 0.32, 1), 'charger': (0.95, 0.80, 0.10, 1), 'cabinet': (0.62, 0.64, 0.68, 1),
    'green': (0.10, 0.80, 0.25, 1), 'red': (0.88, 0.12, 0.10, 1), 'amber': (1.0, 0.70, 0.0, 1),
    'white': (0.95, 0.95, 0.95, 1), 'lane': (0.98, 0.85, 0.10, 1), 'lane_two': (0.98, 0.50, 0.05, 1),
    'walkway': (0.15, 0.65, 0.30, 1), 'pad_station': (0.10, 0.70, 0.30, 1), 'pad_dock': (0.98, 0.85, 0.10, 1),
    'pad_park': (0.95, 0.95, 0.95, 1), 'pad_yield': (1.0, 0.55, 0.0, 1),
    'office_glass': (0.45, 0.65, 0.85, 0.55), 'office_wall': (0.90, 0.90, 0.88, 1),
    'truck_cab': (0.80, 0.10, 0.10, 1), 'truck_box': (0.92, 0.92, 0.92, 1), 'tyre': (0.05, 0.05, 0.05, 1),
    'grass': (0.30, 0.50, 0.25, 1),
}
_EMISSIVE = {'lamp': (1.0, 0.98, 0.90), 'led_green': (0.1, 1.0, 0.3), 'led_red': (1.0, 0.1, 0.1),
             'led_amber': (1.0, 0.7, 0.0), 'screen_on': (0.2, 0.6, 1.0)}


def _mat(name) -> str:
    if isinstance(name, str) and name in _EMISSIVE:
        r, g, b = _EMISSIVE[name]
        return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
                f'<emissive>{r} {g} {b} 1</emissive></material>')
    r, g, b, a = _MAT[name] if isinstance(name, str) else name
    return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>'
            f'<diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>'
            f'<specular>0.15 0.15 0.15 1</specular></material>')


def _alpha(name) -> float:
    if isinstance(name, str):
        return _MAT.get(name, (0, 0, 0, 1))[3] if name in _MAT else 1.0
    return name[3]


def rot(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c - dy * s, dx * s + dy * c


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


_BOX_V = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
# faces of the unit box (corner indices, counter-clockwise seen from outside) and normals
_BOX_F = [((0, 1, 3, 2), (-1, 0, 0)), ((4, 6, 7, 5), (1, 0, 0)), ((0, 4, 5, 1), (0, -1, 0)),
          ((2, 3, 7, 6), (0, 1, 0)), ((0, 2, 6, 4), (0, 0, -1)), ((1, 5, 7, 3), (0, 0, 1))]


class MeshBaker:
    """Merges opaque static primitives into one OBJ (+ MTL), one object per material."""

    def __init__(self) -> None:
        self.v: List[Tuple[float, float, float]] = []
        self.vn: List[Tuple[float, float, float]] = []
        self.vn_index: Dict[Tuple[float, float, float], int] = {}
        self.faces: Dict[str, List[Tuple[Tuple[int, int], ...]]] = {}
        self.colors: Dict[str, Tuple[float, float, float]] = {}
        self.count = 0

    @staticmethod
    def accepts(mat) -> bool:
        if isinstance(mat, str):
            return mat in _MAT and _MAT[mat][3] >= 0.999
        return mat[3] >= 0.999

    def _key(self, mat) -> str:
        if isinstance(mat, str):
            self.colors[mat] = _MAT[mat][:3]
            return mat
        key = 'c_' + ''.join(f'{int(round(c * 255)):02x}' for c in mat[:3])
        self.colors[key] = tuple(mat[:3])
        return key

    def _normal(self, n) -> int:
        key = (round(float(n[0]), 3), round(float(n[1]), 3), round(float(n[2]), 3))
        idx = self.vn_index.get(key)
        if idx is None:
            self.vn.append(key)
            idx = self.vn_index[key] = len(self.vn)
        return idx

    def _add(self, mat, verts, normals, faces) -> None:
        """faces: tuples of (vertex index, normal index) local to verts / normals."""
        v0 = len(self.v)
        self.v.extend(map(tuple, verts))
        nidx = [self._normal(n) for n in normals]
        out = self.faces.setdefault(self._key(mat), [])
        for f in faces:
            out.append(tuple((v0 + a + 1, nidx[b]) for a, b in f))
        self.count += 1

    def box(self, cx, cy, cz, sx, sy, sz, mat, roll=0.0, pitch=0.0, yaw=0.0) -> None:
        R = _rpy_matrix(roll, pitch, yaw)
        verts = (_BOX_V * (sx, sy, sz)) @ R.T + (cx, cy, cz)
        normals = np.array([n for _, n in _BOX_F], dtype=float) @ R.T
        faces = []
        for k, (q, _) in enumerate(_BOX_F):
            faces.append(((q[0], k), (q[1], k), (q[2], k)))
            faces.append(((q[0], k), (q[2], k), (q[3], k)))
        self._add(mat, verts, normals, faces)

    def cyl(self, cx, cy, cz, r, h, mat, roll=0.0, pitch=0.0) -> None:
        n = 10 if r < 0.08 else (16 if r < 0.4 else 24)
        R = _rpy_matrix(roll, pitch, 0.0)
        ang = np.arange(n) * 2.0 * math.pi / n
        ring = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros(n)], axis=1)
        verts = np.concatenate([ring + (0, 0, -h / 2), ring + (0, 0, h / 2),
                                [[0, 0, -h / 2], [0, 0, h / 2]]])
        normals = np.concatenate([np.stack([np.cos(ang), np.sin(ang), np.zeros(n)], axis=1),
                                  [[0, 0, -1], [0, 0, 1]]])
        verts = verts @ R.T + (cx, cy, cz)
        normals = normals @ R.T
        faces = []
        bc, tc = 2 * n, 2 * n + 1
        for i in range(n):
            j = (i + 1) % n
            faces.append(((i, i), (j, j), (n + j, j)))
            faces.append(((i, i), (n + j, j), (n + i, i)))
            faces.append(((bc, n), (j, n), (i, n)))
            faces.append(((tc, n + 1), (n + i, n + 1), (n + j, n + 1)))
        self._add(mat, verts, normals, faces)

    def write(self, obj_path: str) -> str:
        import os
        mtl_path = os.path.splitext(obj_path)[0] + '.mtl'
        with open(mtl_path, 'w', encoding='utf-8') as f:
            for key in self.faces:
                r, g, b = self.colors[key]
                f.write(f'newmtl {key}\nKa {r:.3f} {g:.3f} {b:.3f}\nKd {r:.3f} {g:.3f} {b:.3f}\n'
                        f'Ks 0.150 0.150 0.150\nNs 20\nd 1.0\nillum 2\n\n')
        with open(obj_path, 'w', encoding='utf-8') as f:
            f.write(f'# AMR factory static geometry ({self.count} primitives)\n'
                    f'mtllib {os.path.basename(mtl_path)}\n')
            f.write(''.join(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.v))
            f.write(''.join(f'vn {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.vn))
            for key, faces in self.faces.items():
                f.write(f'o {key}\nusemtl {key}\n')
                f.write(''.join('f ' + ' '.join(f'{a}//{b}' for a, b in fc) + '\n'
                                for fc in faces))
        return mtl_path


_BAKER: Optional[MeshBaker] = None      # active while world_sdf() runs with a mesh


# --------------------------------------------------------------- primitives
class MapSlices:
    """Collects the footprint of every primitive crossing the lidar plane."""

    def __init__(self) -> None:
        self.boxes: List[Tuple[float, float, float, float, float]] = []   # cx, cy, sx, sy, yaw
        self.circles: List[Tuple[float, float, float]] = []

    def add_box(self, cx, cy, cz, sx, sy, sz, yaw, roll=0.0, pitch=0.0) -> None:
        if roll or pitch:      # tilted primitives: use their bounding sphere height
            half = 0.5 * math.sqrt(sx * sx + sy * sy + sz * sz)
            if cz + half < LIDAR_Z[0] or cz - half > LIDAR_Z[1]:
                return
        elif cz + sz / 2 < LIDAR_Z[0] or cz - sz / 2 > LIDAR_Z[1]:
            return
        self.boxes.append((cx, cy, sx, sy, yaw))

    def add_cyl(self, cx, cy, cz, r, h) -> None:
        if cz + h / 2 < LIDAR_Z[0] or cz - h / 2 > LIDAR_Z[1]:
            return
        self.circles.append((cx, cy, r))


class Model:
    def __init__(self, name: str, slices: Optional[MapSlices] = None) -> None:
        self.name = name
        self.visuals: List[str] = []
        self.collisions: List[str] = []
        self.k = 0
        self.slices = slices
        self.baker = _BAKER

    def box(self, cx, cy, cz, sx, sy, sz, mat, yaw=0.0, collide=False, roll=0.0, pitch=0.0,
            in_map=True):
        self.k += 1
        pose = f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'
        geo = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
        if self.baker is not None and self.baker.accepts(mat):
            self.baker.box(cx, cy, cz, sx, sy, sz, mat, roll, pitch, yaw)
        else:
            self.visuals.append(f'<visual name="v{self.k}">{pose}{geo}{_mat(mat)}'
                                f'<cast_shadows>false</cast_shadows></visual>')
        if collide:
            self.collisions.append(f'<collision name="c{self.k}">{pose}{geo}</collision>')
        if self.slices is not None and in_map and _alpha(mat) > 0.3:
            self.slices.add_box(cx, cy, cz, sx, sy, sz, yaw, roll, pitch)

    def cyl(self, cx, cy, cz, r, h, mat, collide=False, roll=0.0, pitch=0.0, in_map=True):
        self.k += 1
        pose = f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} {roll:.4f} {pitch:.4f} 0</pose>'
        geo = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>'
        if self.baker is not None and self.baker.accepts(mat):
            self.baker.cyl(cx, cy, cz, r, h, mat, roll, pitch)
        else:
            self.visuals.append(f'<visual name="v{self.k}">{pose}{geo}{_mat(mat)}'
                                f'<cast_shadows>false</cast_shadows></visual>')
        if collide:
            self.collisions.append(f'<collision name="c{self.k}">{pose}{geo}</collision>')
        if self.slices is not None and in_map and not roll and not pitch:
            self.slices.add_cyl(cx, cy, cz, r, h)
        elif self.slices is not None and in_map:
            self.slices.add_box(cx, cy, cz, 2 * r, h, 2 * r, 0.0, roll, pitch)

    def collision_box(self, cx, cy, cz, sx, sy, sz, yaw=0.0):
        self.k += 1
        self.collisions.append(
            f'<collision name="c{self.k}"><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>'
            f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>')

    def sdf(self) -> str:
        if not self.visuals and not self.collisions:
            return ''
        return (f'<model name="{escape(self.name)}"><static>true</static><link name="link">'
                + ''.join(self.collisions) + ''.join(self.visuals) + '</link></model>\n')


class Local:
    """Helper to place parts in a local frame (origin x0, y0, rotation yaw)."""

    def __init__(self, m: Model, x0: float, y0: float, yaw: float = 0.0) -> None:
        self.m, self.x0, self.y0, self.yaw = m, x0, y0, yaw

    def box(self, dx, dy, z, sx, sy, sz, mat, dyaw=0.0, **kw):
        ox, oy = rot(dx, dy, self.yaw)
        self.m.box(self.x0 + ox, self.y0 + oy, z, sx, sy, sz, mat, self.yaw + dyaw, **kw)

    def cyl(self, dx, dy, z, r, h, mat, **kw):
        ox, oy = rot(dx, dy, self.yaw)
        self.m.cyl(self.x0 + ox, self.y0 + oy, z, r, h, mat, **kw)


def _rect(o):
    x0, y0, x1, y1 = o['rect']
    return x0, y0, x1, y1, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0


# ========================================================== building shell
def build_ground(W, H, out: List[str]) -> None:
    g = Model('ground')
    g.collisions.append(f'<collision name="plane"><geometry><plane><normal>0 0 1</normal>'
                        f'<size>{W + 80} {H + 80}</size></plane></geometry>'
                        f'<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
                        f'</surface></collision>')
    g.box(W / 2 + 6, H / 2, -0.06, W + 80, H + 80, 0.1, 'grass', in_map=False)
    g.box(W / 2 + 6, H / 2, -0.05, W + 40, H + 36, 0.1, 'asphalt', in_map=False)
    # parking lines for trucks / cars outside
    for i in range(10):
        g.box(8 + i * 4.0, -16.0, 0.005, 0.12, 5.0, 0.01, 'white', in_map=False)
    out.append(g.sdf())


def build_floor(layout, W, H, out: List[str]) -> None:
    f = Model('hall_floor')
    f.box(W / 2, H / 2, 0.0, W, H, 0.002, 'floor', in_map=False)
    for x in np.arange(6.0, W, 6.0):
        f.box(float(x), H / 2, 0.0015, 0.02, H, 0.002, 'floor_joint', in_map=False)
    for y in np.arange(6.0, H, 6.0):
        f.box(W / 2, float(y), 0.0015, W, 0.02, 0.002, 'floor_joint', in_map=False)
    out.append(f.sdf())


def build_walls(layout, slices: MapSlices, out: List[str]) -> None:
    W, H = layout['factory']['size']
    wh = float(layout['factory'].get('wall_height', 10.0))
    t = float(layout['factory'].get('wall_thickness', 0.4))
    doors = (layout.get('decor', {}) or {}).get('doors', []) or []
    m = Model('hall_walls', slices)
    walls = {'south': ((0.0, -t / 2), (W, -t / 2)), 'north': ((0.0, H + t / 2), (W, H + t / 2)),
             'west': ((-t / 2, 0.0), (-t / 2, H)), 'east': ((W + t / 2, 0.0), (W + t / 2, H))}
    for name, (p0, p1) in walls.items():
        horiz = p0[1] == p1[1]
        L = (p1[0] - p0[0]) if horiz else (p1[1] - p0[1])
        cuts = sorted((d['at'] - d['width'] / 2, d['at'] + d['width'] / 2)
                      for d in doors if d['wall'] == name)
        segs, s = [], -t
        for a, b in cuts:
            segs.append((s, a))
            s = b
        segs.append((s, L + t))

        def place(a, b, z0, z1, mat, inset=0.0, collide=True):
            if b - a < 0.05:
                return
            c = (a + b) / 2
            cx, cy = (c, p0[1]) if horiz else (p0[0], c)
            if inset:
                if horiz:
                    cy += inset if name == 'south' else -inset
                else:
                    cx += inset if name == 'west' else -inset
            sx, sy = (b - a, t) if horiz else (t, b - a)
            m.box(cx, cy, (z0 + z1) / 2, sx, sy, z1 - z0, mat, collide=collide)

        for a, b in segs:
            place(a, b, 0.0, 1.2, 'wall_base')
            place(a, b, 1.2, 6.4, 'wall_panel')
            place(a, b, 6.4, 7.9, 'window', collide=False)
            place(a, b, 7.9, wh, 'wall_panel')
            place(a, b, wh - 0.4, wh, 'wall_top')
            # vertical panel ribs every 1.5 m
            for r in np.arange(a + 0.75, b, 1.5):
                if horiz:
                    m.box(float(r), p0[1] + (0.22 if name == 'south' else -0.22), 3.8, 0.05, 0.03, 5.2,
                          'steel_light', in_map=False)
                else:
                    m.box(p0[0] + (0.22 if name == 'west' else -0.22), float(r), 3.8, 0.03, 0.05, 5.2,
                          'steel_light', in_map=False)
        # doors: closed roller shutters with a yellow frame, lintel above
        for a, b in cuts:
            place(a, b, 0.0, 5.0, 'door', inset=0.1)
            for r in np.arange(0.3, 5.0, 0.3):
                place(a, b, r, r + 0.04, 'steel_light', inset=0.05, collide=False)
            place(a, b, 5.0, wh, 'wall_panel')
            for edge in (a, b):
                place(edge - 0.15, edge + 0.15, 0.0, 5.2, 'door_frame', inset=-0.05)
            place(a, b, 0.0, 0.25, 'black', inset=-0.35)     # dock bumper
    # steel columns along the walls (inside)
    for x in np.arange(0.0, W + 0.1, 12.0):
        for y in (0.15, H - 0.15):
            m.box(float(x) if 0 < x < W else min(max(float(x), 0.15), W - 0.15), y, wh / 2,
                  0.3, 0.3, wh, 'steel')
    for y in np.arange(12.0, H, 12.0):
        for x in (0.15, W - 0.15):
            m.box(x, float(y), wh / 2, 0.3, 0.3, wh, 'steel')
    out.append(m.sdf())


def build_roof(layout, out: List[str], lights: List[str]) -> None:
    """Roof beams with light fixtures (emissive) and a grid of real lights."""
    W, H = layout['factory']['size']
    wh = float(layout['factory'].get('wall_height', 10.0))
    cfg = (layout.get('decor', {}) or {}).get('lights', {}) or {}
    sx = float(cfg.get('spacing_x', 12.0))
    sy = float(cfg.get('spacing_y', 12.0))
    hz = float(cfg.get('height', wh - 0.8))
    m = Model('roof_beams')
    for y in np.arange(sy / 2, H, sy):
        m.box(W / 2, float(y), wh - 0.25, W, 0.18, 0.35, 'steel', in_map=False)
        for x in np.arange(sx / 2, W, sx / 2):
            m.box(float(x), float(y), hz + 0.12, 0.02, 0.02, wh - hz - 0.3, 'steel', in_map=False)
            m.box(float(x), float(y), hz, 2.4, 0.35, 0.08, 'steel_light', in_map=False)
            m.box(float(x), float(y), hz - 0.045, 2.3, 0.28, 0.01, 'lamp', in_map=False)
    out.append(m.sdf())
    k = 0
    for y in np.arange(sy / 2, H, sy * 2):
        for x in np.arange(sx / 2, W, sx * 2):
            k += 1
            lights.append(
                f'<light type="point" name="hall_light_{k}"><pose>{x:.1f} {y:.1f} {hz - 0.3:.1f} 0 0 0</pose>'
                f'<diffuse>0.55 0.55 0.52 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
                f'<attenuation><range>30</range><constant>0.6</constant><linear>0.02</linear>'
                f'<quadratic>0.0005</quadratic></attenuation><cast_shadows>false</cast_shadows></light>')


def _clear_of_traffic(graph, rect, margin: float) -> bool:
    x0, y0, x1, y1 = rect
    xy = graph._xy
    dx = np.maximum(np.maximum(x0 - xy[:, 0], 0.0), xy[:, 0] - x1)
    dy = np.maximum(np.maximum(y0 - xy[:, 1], 0.0), xy[:, 1] - y1)
    if np.any(np.hypot(dx, dy) < margin):
        return False
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    r = 0.5 * math.hypot(x1 - x0, y1 - y0)
    for es in graph.out_edges:
        for e in es:
            a, b = xy[e.u], xy[e.v]
            ab = b - a
            L2 = float(ab @ ab) or 1e-9
            t = min(max(((cx - a[0]) * ab[0] + (cy - a[1]) * ab[1]) / L2, 0.0), 1.0)
            if math.hypot(cx - (a[0] + t * ab[0]), cy - (a[1] + t * ab[1])) < r + margin:
                return False
    return True


def _overlaps(rect, objects, margin: float) -> bool:
    x0, y0, x1, y1 = rect
    for o in objects:
        a0, b0, a1, b1 = o['rect']
        if x0 < a1 + margin and a0 - margin < x1 and y0 < b1 + margin and b0 - margin < y1:
            return True
    return False


def build_columns(layout, graph, objects, slices, out: List[str]) -> None:
    W, H = layout['factory']['size']
    wh = float(layout['factory'].get('wall_height', 10.0))
    cfg = (layout.get('decor', {}) or {}).get('columns', {}) or {}
    m = Model('steel_columns', slices)
    for x in np.arange(12.0, W - 1, float(cfg.get('spacing_x', 12.0))):
        for y in np.arange(12.0, H - 1, float(cfg.get('spacing_y', 12.0))):
            rect = (x - 0.25, y - 0.25, x + 0.25, y + 0.25)
            if not _clear_of_traffic(graph, rect, 0.9) or _overlaps(rect, objects, 0.6):
                continue
            m.box(float(x), float(y), wh / 2, 0.45, 0.45, wh, 'steel')
            for i, z in enumerate(np.arange(0.1, 1.5, 0.25)):     # yellow/black guard
                m.box(float(x), float(y), float(z), 0.62, 0.62, 0.25, 'yellow' if i % 2 == 0 else 'black')
    out.append(m.sdf())


# ============================================================ floor marks
def build_markings(layout, graph, out: List[str]) -> None:
    mk = Model('floor_markings')
    for z in layout.get('zones', []) or []:
        x0, y0, x1, y1 = z['rect']
        r, g, b = z.get('color', [0.5, 0.5, 0.5])
        mk.box((x0 + x1) / 2, (y0 + y1) / 2, 0.003, x1 - x0, y1 - y0, 0.002,
               (0.70 + 0.22 * (r - 0.5), 0.71 + 0.22 * (g - 0.5), 0.70 + 0.22 * (b - 0.5), 1), in_map=False)
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x0, y1, x1, y1), (x0, y0, x0, y1), (x1, y0, x1, y1)):
            L = math.hypot(bx - ax, by - ay)
            mk.box((ax + bx) / 2, (ay + by) / 2, 0.005, L if ay == by else 0.1,
                   0.1 if ay == by else L, 0.002, (r, g, b, 1), in_map=False)
    for ln in getattr(graph, 'lanes', {}).values():
        (ax, ay), (bx, by) = ln.p0, ln.p1
        L = ln.length
        yaw = math.atan2(by - ay, bx - ax)
        if ln.two_way:
            n = int(L / 1.5)
            for i in range(n):
                t = (i + 0.5) * L / n
                mk.box(ax + (bx - ax) * t / L, ay + (by - ay) * t / L, 0.006, 0.9, 0.10, 0.002,
                       'lane_two', yaw, in_map=False)
            # aisle edge lines
            for s in (-1, 1):
                ox, oy = rot(0, s * 1.6, yaw)
                mk.box((ax + bx) / 2 + ox, (ay + by) / 2 + oy, 0.006, L, 0.06, 0.002, 'white', yaw,
                       in_map=False)
        else:
            mk.box((ax + bx) / 2, (ay + by) / 2, 0.006, L, 0.08, 0.002, 'lane', yaw, in_map=False)
            for i in range(1, int(L / 6)):
                t = i * 6.0
                px, py = ax + (bx - ax) * t / L, ay + (by - ay) * t / L
                for s in (-1, 1):
                    ox, oy = rot(-0.15, s * 0.12, yaw)
                    mk.box(px + ox, py + oy, 0.006, 0.40, 0.07, 0.002, 'lane', yaw + s * 0.7, in_map=False)
    pads = [(graph.nodes[s.service], 'pad_station', s.heading) for s in graph.stations.values()]
    pads += [(graph.nodes[s.queue], 'pad_station', s.heading) for s in graph.stations.values()
             if s.queue is not None]
    pads += [(graph.nodes[i], 'pad_dock', graph.nodes[i].heading or 0.0) for i in graph.chargers.values()]
    pads += [(graph.nodes[i], 'pad_park', graph.nodes[i].heading or 0.0) for i in graph.parking.values()]
    pads += [(graph.nodes[i], 'pad_yield', graph.nodes[i].heading or 0.0) for i in graph.yield_bays]
    for n, mat, yaw in pads:
        loc = Local(mk, n.x, n.y, yaw)
        for dx, dy, sx, sy in ((0.55, 0, 0.06, 0.9), (-0.55, 0, 0.06, 0.9), (0, 0.45, 1.1, 0.06),
                               (0, -0.45, 1.1, 0.06)):
            loc.box(dx, dy, 0.007, sx, sy, 0.002, mat, in_map=False)
    out.append(mk.sdf())


# ============================================================ storage
def add_rack(m: Model, o, idx: int, pallet: bool = False) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 3.0))
    m.collision_box(cx, cy, h / 2, sx, sy, h)
    along_x = sx >= sy
    L = sx if along_x else sy
    D = sy if along_x else sx
    bays = max(1, int(round(L / (2.7 if pallet else 2.5))))
    up = 'rack_up'
    for i in range(bays + 1):
        t = -L / 2 + L * i / bays
        for side in (-1, 1):
            px = cx + (t if along_x else side * (D / 2 - 0.05))
            py = cy + (side * (D / 2 - 0.05) if along_x else t)
            m.box(px, py, h / 2, 0.09, 0.09, h, up)
            m.box(px, py, 0.04, 0.2, 0.2, 0.08, 'steel')
    nlev = 4 if pallet else 5
    levels = [0.12 + (h - 0.4) * k / (nlev - 1) for k in range(nlev)]
    for z in levels:
        if not pallet:
            m.box(cx, cy, z, sx, sy, 0.04, 'shelf')
        for side in (-1, 1):
            px = cx if along_x else cx + side * (D / 2 - 0.05)
            py = cy + side * (D / 2 - 0.05) if along_x else cy
            m.box(px, py, z + 0.07, sx if along_x else 0.07, 0.07 if along_x else sy, 0.12, 'rack_beam')
    # goods on every level (deterministic pattern so the map is reproducible)
    for li, z in enumerate(levels[:-1]):
        for i in range(bays):
            t = -L / 2 + L * (i + 0.5) / bays
            px, py = (cx + t, cy) if along_x else (cx, cy + t)
            w = L / bays
            if pallet:
                if (i * 3 + li + idx) % 7 == 0:
                    continue
                for k in (-0.25, 0.25):
                    qx, qy = (px + k * w, py) if along_x else (px, py + k * w)
                    m.box(qx, qy, z + 0.21, w * 0.42 if along_x else D * 0.9,
                          D * 0.9 if along_x else w * 0.42, 0.14, 'pallet')
                    hh = 0.7 + 0.25 * ((i + li + idx) % 3)
                    m.box(qx, qy, z + 0.28 + hh / 2, w * 0.40 if along_x else D * 0.85,
                          D * 0.85 if along_x else w * 0.40, hh,
                          'wrap' if (i + li) % 2 else 'carton')
            else:
                n_items = 3
                for k in range(n_items):
                    if (i * 7 + li * 3 + k + idx) % 6 == 0:
                        continue
                    off = (k - 1) * w / 3.2
                    qx, qy = (px + off, py) if along_x else (px, py + off)
                    hh = 0.22 + 0.12 * ((i + li + k) % 3)
                    mat = ['carton', 'bin_blue', 'carton_dark', 'bin_red', 'bin_green', 'carton'][
                        (i + li * 2 + k + idx) % 6]
                    m.box(qx, qy, z + 0.03 + hh / 2, w * 0.28 if along_x else D * 0.8,
                          D * 0.8 if along_x else w * 0.28, hh, mat)
    # end protection
    for end in (-1, 1):
        px, py = (cx + end * (L / 2 + 0.15), cy) if along_x else (cx, cy + end * (L / 2 + 0.15))
        m.box(px, py, 0.25, 0.12 if along_x else D + 0.2, D + 0.2 if along_x else 0.12, 0.5, 'yellow')


def add_pallet(m: Model, o, idx: int = 0) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 1.0))
    m.box(cx, cy, 0.07, sx, sy, 0.14, 'pallet', collide=True)
    layers = max(1, int((h - 0.14) / 0.3))
    for k in range(layers):
        for i, (dx, dy) in enumerate(((-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25))):
            if k == layers - 1 and (i + idx) % 3 == 0:
                continue
            m.box(cx + dx * sx, cy + dy * sy, 0.14 + 0.15 + k * 0.3, sx * 0.47, sy * 0.47, 0.28,
                  'carton' if (k + i) % 2 else 'carton_dark')
    m.box(cx, cy, 0.14 + layers * 0.15, sx * 0.98, sy * 0.98, layers * 0.3 + 0.02, 'wrap', in_map=False)


def add_cabinet(m: Model, o) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 2.0))
    m.box(cx, cy, h / 2, sx, sy, h, 'cabinet', collide=True)
    n = max(1, int(sx / 0.8)) if sx > sy else max(1, int(sy / 0.8))
    for i in range(n):
        t = -0.5 + (i + 0.5) / n
        if sx > sy:
            m.box(cx + t * sx, cy - sy / 2 - 0.01, h / 2, sx / n - 0.06, 0.02, h - 0.1, 'steel_light')
        else:
            m.box(cx - sx / 2 - 0.01, cy + t * sy, h / 2, 0.02, sy / n - 0.06, h - 0.1, 'steel_light')


def add_charger(m: Model, o) -> None:
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 1.1))
    m.box(cx, cy, h / 2, sx, sy, h, 'charger', collide=True)
    m.box(cx, cy - sy / 2 - 0.01, h * 0.72, sx * 0.7, 0.02, h * 0.25, 'screen_on')
    m.box(cx, cy - sy / 2 - 0.05, 0.25, sx * 0.5, 0.1, 0.12, 'steel')      # charging contacts
    m.cyl(cx, cy, h + 0.08, 0.05, 0.12, 'led_green')


# ============================================================ production lines
def add_conveyor(m: Model, x, y0, y1, h=0.9, width=1.0) -> None:
    L = y1 - y0
    cy = (y0 + y1) / 2
    m.box(x, cy, h - 0.04, width, L, 0.06, 'belt')
    for s in (-1, 1):
        m.box(x + s * (width / 2 - 0.03), cy, h + 0.03, 0.05, L, 0.1, 'galv')
    for yy in np.arange(y0 + 0.2, y1, 0.5):
        m.box(x, float(yy), h - 0.075, width - 0.1, 0.06, 0.03, 'roller', in_map=False)
    for yy in np.arange(y0 + 0.1, y1, 1.6):
        for s in (-1, 1):
            m.box(x + s * (width / 2 - 0.06), float(yy), (h - 0.1) / 2, 0.06, 0.06, h - 0.1, 'galv')
        m.box(x, float(yy), 0.10, width - 0.1, 0.05, 0.05, 'galv')


def add_light_tower(m: Model, x, y, z) -> None:
    m.cyl(x, y, z + 0.1, 0.015, 0.2, 'steel')
    for i, mat in enumerate(('led_green', 'amber', 'red')):
        m.cyl(x, y, z + 0.25 + i * 0.09, 0.045, 0.08, mat)


def add_robot_arm(m: Model, x, y, base_z, yaw, color='robot_orange', reach=1.0) -> None:
    loc = Local(m, x, y, yaw)
    loc.cyl(0, 0, base_z + 0.15, 0.22, 0.3, 'machine_dark')
    loc.cyl(0, 0, base_z + 0.42, 0.16, 0.25, color)
    m.box(x + rot(0.0, 0, yaw)[0], y + rot(0, 0, yaw)[1], base_z + 0.95, 0.16, 0.16, 0.9, color,
          yaw, pitch=0.35)
    ex, ey = rot(0.45 * reach, 0, yaw)
    m.box(x + ex, y + ey, base_z + 1.35, 0.8 * reach, 0.13, 0.13, color, yaw, pitch=-0.25)
    gx, gy = rot(0.85 * reach, 0, yaw)
    m.box(x + gx, y + gy, base_z + 1.05, 0.08, 0.2, 0.35, 'machine_dark', yaw)


def add_fence(m: Model, x0, y0, x1, y1, h=1.8, gap_side: Optional[str] = None) -> None:
    sides = {'s': ((x0, y0), (x1, y0)), 'n': ((x0, y1), (x1, y1)),
             'w': ((x0, y0), (x0, y1)), 'e': ((x1, y0), (x1, y1))}
    for k, ((ax, ay), (bx, by)) in sides.items():
        if k == gap_side:
            continue
        L = math.hypot(bx - ax, by - ay)
        horiz = ay == by
        # mesh panels start above the lidar plane: the lidar only sees the posts
        m.box((ax + bx) / 2, (ay + by) / 2, (h + 0.35) / 2, L if horiz else 0.03, 0.03 if horiz else L,
              h - 0.35, 'fence', in_map=False)
        for t in np.arange(0.0, L + 0.01, 1.2):
            px, py = (ax + t, ay) if horiz else (ax, ay + t)
            m.box(float(px), float(py), h / 2, 0.05, 0.05, h, 'yellow')


def add_production_line(m: Model, ln: dict, objs: Dict[str, dict]) -> None:
    x, yi, yo = float(ln['x']), float(ln['y_in']), float(ln['y_out'])
    add_conveyor(m, x, yo, yi)
    o = objs['cnc']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    m.box(cx, cy, 1.15, sx, sy, 2.3, 'machine', collide=True)
    m.box(cx, cy, 2.33, sx, sy, 0.06, 'machine_blue')
    m.box(x1 + 0.01, cy, 1.3, 0.02, sy * 0.55, 0.9, 'glass')
    m.box(x1 + 0.03, cy - sy * 0.38, 1.4, 0.04, 0.35, 0.45, 'screen_on')
    m.box(cx, cy, 0.05, sx + 0.1, sy + 0.1, 0.1, 'machine_dark')
    add_light_tower(m, x0 + 0.2, y1 - 0.2, 2.4)
    o = objs['control_cabinet']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    m.box(cx, cy, 1.0, sx, sy, 2.0, 'cabinet', collide=True)
    m.box(x0 - 0.01, cy, 1.4, 0.02, sy * 0.6, 0.35, 'screen_on')
    o = objs['robot_cell']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    add_fence(m, x0, y0, x1, y1, gap_side='w')
    m.collision_box(cx, cy, 1.0, sx, sy, 2.0)
    add_robot_arm(m, cx + 0.3, cy, 0.0, math.pi, reach=1.1)
    m.box(cx + 0.5, y1 - 0.5, 0.45, 0.6, 0.5, 0.9, 'machine_dark')          # parts feeder
    o = objs['assembly']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    m.box(cx, cy, 0.85, sx, sy, 0.05, 'wood', collide=True)
    for dx in (-sx / 2 + 0.08, sx / 2 - 0.08):
        for dy in (-sy / 2 + 0.08, sy / 2 - 0.08):
            m.box(cx + dx, cy + dy, 0.42, 0.06, 0.06, 0.84, 'galv')
    m.box(x0 + 0.15, cy, 1.35, 0.06, sy * 0.9, 0.9, 'galv')                  # tool board
    m.box(cx - 0.3, cy + 0.6, 1.15, 0.05, 0.45, 0.3, 'screen_on')
    for k, mat in enumerate(('bin_blue', 'bin_red', 'bin_yellow', 'bin_green')):
        m.box(x0 + 0.3, y0 + 0.4 + k * 0.7, 1.05, 0.3, 0.45, 0.2, mat)
    m.box(cx + 0.2, cy - 0.4, 0.95, 0.35, 0.3, 0.18, 'machine_orange')        # work piece
    o = objs['qc_gate']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    for s in (x0 + 0.08, x1 - 0.08):
        m.box(s, cy, 1.1, 0.16, sy, 2.2, 'machine_dark', collide=True)
    m.box(cx, cy, 2.1, sx, sy, 0.2, 'machine_dark')
    m.box(cx, cy, 1.95, sx * 0.7, 0.05, 0.05, 'led_green')
    o = objs['boxing']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    m.box(cx, cy, 0.9, sx, sy, 1.8, 'machine', collide=True)
    m.box(cx, cy, 1.82, sx, sy, 0.05, 'machine_orange')
    m.box(x0 - 0.01, cy, 1.2, 0.02, sy * 0.5, 0.6, 'glass')
    add_light_tower(m, x1 - 0.2, y1 - 0.2, 1.85)
    for k in range(4):                                                        # flat cartons
        m.box(x1 - 0.4, y0 - 0.0 + 0.1, 0.1 + k * 0.05, 0.6, 0.15, 0.04, 'carton')


def add_packaging_line(m: Model, ln: dict, objs: Dict[str, dict]) -> None:
    x, yi, yo = float(ln['x']), float(ln['y_in']), float(ln['y_out'])
    add_conveyor(m, x, yo, yi)
    o = objs['unboxer']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    m.box(cx, cy, 0.85, sx, sy, 0.05, 'galv', collide=True)
    for dx in (-sx / 2 + 0.08, sx / 2 - 0.08):
        for dy in (-sy / 2 + 0.08, sy / 2 - 0.08):
            m.box(cx + dx, cy + dy, 0.42, 0.06, 0.06, 0.84, 'galv')
    add_robot_arm(m, cx - 0.3, cy, 0.87, 0.0, color='machine_blue', reach=0.8)
    m.box(cx + 0.4, y0 + 0.4, 1.05, 0.45, 0.4, 0.35, 'carton')
    o = objs['wrapper']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    for s in (x0 + 0.15, x1 - 0.15):
        m.box(s, cy, 1.1, 0.3, sy, 2.2, 'packer', collide=True)
    m.box(cx, cy, 2.05, sx, sy, 0.3, 'packer')
    m.box(cx, y1 + 0.01, 1.5, sx * 0.6, 0.02, 0.8, 'glass', in_map=False)
    m.box(cx, y0 - 0.01, 1.5, sx * 0.6, 0.02, 0.8, 'glass', in_map=False)
    m.cyl(cx, cy, 1.55, 0.25, 0.12, 'white', roll=math.pi / 2)              # film roll
    add_light_tower(m, x1 - 0.2, y1 - 0.2, 2.2)
    o = objs['case_packer']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    m.box(cx, cy, 1.0, sx, sy, 2.0, 'machine', collide=True)
    m.box(cx, cy, 2.02, sx, sy, 0.05, 'packer')
    m.box(x0 - 0.01, cy, 1.25, 0.02, sy * 0.5, 0.7, 'glass')
    m.box(x1 - 0.3, y1 + 0.25, 0.6, 0.6, 0.4, 1.2, 'carton_dark')          # carton magazine
    o = objs['palletizer']
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    add_fence(m, x0, y0, x1, y1, gap_side='e')
    m.collision_box(cx, cy, 1.0, sx, sy, 2.0)
    add_robot_arm(m, cx - 0.2, cy + 0.6, 0.0, 0.0, reach=1.2)
    m.box(cx - 0.2, cy - 0.9, 0.07, 1.0, 1.0, 0.14, 'pallet')
    for k in range(3):
        m.box(cx - 0.2, cy - 0.9, 0.29 + k * 0.3, 0.9, 0.9, 0.28, 'carton')
    o = objs['pallet_out']
    add_pallet(m, o, 1)


def build_lines(layout, slices, out: List[str], segments: List[dict]) -> None:
    by_line: Dict[str, Dict[str, dict]] = {}
    for o in line_objects(layout):
        by_line.setdefault(o['line'], {})[o['kind']] = o
    for ln in layout.get('lines', []) or []:
        m = Model(f'line_{ln["id"]}', slices)
        objs = by_line[ln['id']]
        x, yi, yo = float(ln['x']), float(ln['y_in']), float(ln['y_out'])
        mid = (yi + yo) / 2
        if ln.get('kind', 'production') == 'production':
            add_production_line(m, ln, objs)
            segs = [(yi - 0.3, yi - 7.7, (0.22, 0.22, 0.12), (0.55, 0.60, 0.68, 1), 0.9),
                    (yi - 7.9, yo + 5.7, (0.30, 0.30, 0.20), (0.95, 0.52, 0.10, 1), 1.2),
                    (yo + 4.3, yo + 0.3, (0.42, 0.36, 0.30), (0.74, 0.56, 0.34, 1), 1.0)]
        else:
            add_packaging_line(m, ln, objs)
            segs = [(yi - 0.3, mid + 4.7, (0.30, 0.30, 0.20), (0.95, 0.52, 0.10, 1), 1.1),
                    (mid + 0.3, mid - 4.7, (0.32, 0.32, 0.22), (0.86, 0.90, 0.95, 1), 1.0),
                    (mid - 4.9, yo + 0.3, (0.45, 0.40, 0.32), (0.74, 0.56, 0.34, 1), 1.0)]
        for k, (sa, sb, size, color, spacing) in enumerate(segs):
            z = 0.9 + size[2] / 2 + 0.01
            segments.append({'line': ln['id'], 'name': f'{ln["id"]}_s{k}', 'start': (x, sa, z),
                             'end': (x, sb, z), 'size': size, 'color': color, 'spacing': spacing,
                             'speed': 0.22})
        out.append(m.sdf())


def build_station_objects(layout, graph, slices, out: List[str]) -> None:
    for o in station_objects(layout, graph):
        m = Model(f'station_{o["station"]}', slices)
        along, across = o['size']
        x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
        h = float(o.get('height', 0.6))
        yaw = o['yaw']
        loc = Local(m, cx, cy, yaw)
        m.collision_box(cx, cy, h / 2, sx, sy, h)
        kind = o['kind']
        if kind == 'kit_rack':
            for z in (0.12, 0.6, 1.08):
                loc.box(0, 0, z, along, across, 0.04, 'shelf')
                for k, mat in enumerate(('bin_blue', 'bin_red', 'bin_green')):
                    loc.box((k - 1) * along / 3.2, 0, z + 0.13, along / 3.6, across * 0.75, 0.2, mat)
            for dx in (-along / 2 + 0.03, along / 2 - 0.03):
                for dy in (-across / 2 + 0.03, across / 2 - 0.03):
                    loc.box(dx, dy, h / 2, 0.04, 0.04, h, 'rack_up')
        elif kind.startswith('conveyor'):
            loc.box(0, 0, 0.27, along, across, 0.04, 'belt')
            for dx in (-along / 2 + 0.06, along / 2 - 0.06):
                for dy in (-across / 2 + 0.06, across / 2 - 0.06):
                    loc.box(dx, dy, 0.13, 0.05, 0.05, 0.26, 'galv')
            for dy in (-across / 2 + 0.02, across / 2 - 0.02):
                loc.box(0, dy, 0.33, along, 0.03, 0.08, 'galv')
            if kind == 'conveyor_out':
                loc.box(0, 0, 0.45, 0.42, 0.36, 0.3, 'carton')
            loc.box(along / 2 + 0.1, across / 2, 1.2, 0.05, 0.05, 2.4, 'steel')    # sign post
            loc.box(along / 2 + 0.1, across / 2, 2.3, 0.6, 0.04, 0.3,
                    'machine_orange' if kind == 'conveyor_in' else 'pad_station')
        else:   # storage table
            loc.box(0, 0, h - 0.03, along, across, 0.06, 'galv')
            for dx in (-along / 2 + 0.05, along / 2 - 0.05):
                for dy in (-across / 2 + 0.05, across / 2 - 0.05):
                    loc.box(dx, dy, h / 2, 0.05, 0.05, h, 'galv')
            loc.box(0, 0, h + 0.2, 0.5, 0.42, 0.34, 'wrap')
        out.append(m.sdf())


def build_objects(layout, slices, out: List[str]) -> None:
    for i, o in enumerate(layout.get('objects', []) or []):
        m = Model(f'{o.get("kind", "object")}_{i:02d}', slices)
        kind = o.get('kind')
        if kind == 'rack':
            add_rack(m, o, i)
        elif kind == 'pallet_rack':
            add_rack(m, o, i, pallet=True)
        elif kind == 'pallet':
            add_pallet(m, o, i)
        elif kind == 'cabinet':
            add_cabinet(m, o)
        elif kind == 'charger':
            add_charger(m, o)
        else:
            x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
            h = float(o.get('height', 1.0))
            m.box(cx, cy, h / 2, sx, sy, h, 'steel', collide=True)
        out.append(m.sdf())


def build_charging_area(layout, graph, slices, out: List[str]) -> None:
    if not graph.chargers:
        return
    m = Model('charging_station', slices)
    xs = sorted(graph.nodes[i].x for i in graph.chargers.values())
    ys = [graph.nodes[i].y for i in graph.chargers.values()]
    y_back = max(ys) + 1.85
    # one back panel (cable trays, canopy) per group of neighbouring docks: an
    # aisle running between two groups (the link lane) must stay open
    groups = [[xs[0]]]
    for x in xs[1:]:
        if x - groups[-1][-1] > 4.5:
            groups.append([])
        groups[-1].append(x)
    for grp in groups:
        x0, x1 = grp[0] - 1.2, grp[-1] + 1.2
        m.box((x0 + x1) / 2, y_back, 1.2, x1 - x0, 0.1, 2.4, 'wall_top')
        m.box((x0 + x1) / 2, y_back - 0.6, 2.9, x1 - x0, 1.6, 0.08, 'galv', in_map=False)
        for x in (x0 + 0.1, x1 - 0.1):
            m.box(x, y_back - 1.3, 1.45, 0.08, 0.08, 2.9, 'steel')   # canopy post: lidar sees it
    out.append(m.sdf())


def build_office(layout, out: List[str]) -> None:
    cfg = (layout.get('decor', {}) or {}).get('office')
    if not cfg:
        return
    x0, y0, x1, y1 = cfg['rect']
    floors = int(cfg.get('floors', 2))
    fh = 3.5
    m = Model('office_annex')
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    for f in range(floors):
        z = f * fh
        m.box(cx, cy, z + 0.1, x1 - x0, y1 - y0, 0.2, 'concrete', collide=True)
        m.box(x1 - 0.1, cy, z + fh / 2 + 0.1, 0.2, y1 - y0, fh - 0.2, 'office_glass')
        for y in (y0 + 0.1, y1 - 0.1):
            m.box(cx, y, z + fh / 2 + 0.1, x1 - x0, 0.2, fh - 0.2, 'office_glass')
        for yy in np.arange(y0 + 3, y1 - 1, 3.0):
            m.box(x1 - 0.1, float(yy), z + fh / 2 + 0.1, 0.25, 0.12, fh - 0.2, 'steel_light')
            # desks + monitors
            m.box(cx, float(yy), z + 0.95, 1.6, 0.8, 0.05, 'wood')
            m.box(cx + 0.3, float(yy) + 0.2, z + 1.2, 0.05, 0.5, 0.32, 'screen_on')
    m.box(cx, cy, floors * fh + 0.15, x1 - x0 + 0.4, y1 - y0 + 0.4, 0.3, 'wall_top', collide=True)
    out.append(m.sdf())


def build_trucks(layout, out: List[str]) -> None:
    for i, t in enumerate((layout.get('decor', {}) or {}).get('trucks', []) or []):
        m = Model(f'truck_{i}')
        loc = Local(m, float(t['x']), float(t['y']), float(t.get('yaw', 0.0)))
        loc.box(-1.5, 0, 2.2, 9.0, 2.5, 2.9, 'truck_box', collide=True)        # trailer (back to the door)
        loc.box(4.3, 0, 1.6, 2.2, 2.4, 2.4, 'truck_cab', collide=True)
        loc.box(5.42, 0, 2.1, 0.02, 2.0, 0.9, 'glass')
        for dx in (-4.5, -3.3, 3.8):
            for dy in (-1.1, 1.1):
                loc.cyl(dx, dy, 0.5, 0.5, 0.35, 'tyre', roll=math.pi / 2)
        out.append(m.sdf())


def build_wall_items(layout, slices, out: List[str]) -> None:
    """Fire extinguishers, hose reels, electrical panels and safety signs."""
    W, H = layout['factory']['size']
    m = Model('wall_items', slices)
    k = 0
    for x in np.arange(9.0, W - 3, 18.0):
        for y, s in ((0.35, 1), (H - 0.35, -1)):
            k += 1
            m.cyl(float(x), y + s * 0.05, 0.35, 0.09, 0.6, 'red')
            m.box(float(x), y, 1.6, 0.4, 0.04, 0.4, 'red', in_map=False)        # sign
            if k % 2 == 0:
                m.box(float(x) + 2.0, y + s * 0.05, 1.4, 0.8, 0.25, 1.2, 'cabinet')  # electrical panel
                m.box(float(x) + 2.0, y + s * 0.19, 1.8, 0.2, 0.02, 0.2, 'yellow', in_map=False)
    out.append(m.sdf())


# ============================================================ occupancy map
def rasterize(slices: MapSlices, size, resolution: float = 0.05, margin: float = 1.0):
    W, H = size
    nx = int(math.ceil((W + 2 * margin) / resolution))
    ny = int(math.ceil((H + 2 * margin) / resolution))
    occ = np.zeros((ny, nx), dtype=bool)
    xs = (np.arange(nx) + 0.5) * resolution - margin
    ys = (np.arange(ny) + 0.5) * resolution - margin
    for cx, cy, sx, sy, yaw in slices.boxes:
        r = 0.5 * math.hypot(sx, sy)
        i0 = max(0, int((cx - r + margin) / resolution))
        i1 = min(nx, int((cx + r + margin) / resolution) + 2)
        j0 = max(0, int((cy - r + margin) / resolution))
        j1 = min(ny, int((cy + r + margin) / resolution) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        gx, gy = np.meshgrid(xs[i0:i1] - cx, ys[j0:j1] - cy)
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = gx * c - gy * s
        ly = gx * s + gy * c
        half = resolution * 0.5
        occ[j0:j1, i0:i1] |= (np.abs(lx) <= sx / 2 + half) & (np.abs(ly) <= sy / 2 + half)
    for cx, cy, r in slices.circles:
        i0 = max(0, int((cx - r + margin) / resolution))
        i1 = min(nx, int((cx + r + margin) / resolution) + 2)
        j0 = max(0, int((cy - r + margin) / resolution))
        j1 = min(ny, int((cy + r + margin) / resolution) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        gx, gy = np.meshgrid(xs[i0:i1] - cx, ys[j0:j1] - cy)
        occ[j0:j1, i0:i1] |= np.hypot(gx, gy) <= r + resolution * 0.5
    # outside the hall walls = unknown
    inside = (np.abs(xs[None, :] - W / 2) < W / 2 + 0.45) & (np.abs(ys[:, None] - H / 2) < H / 2 + 0.45)
    img = np.where(occ, 0, np.where(inside, 254, 205)).astype(np.uint8)
    return img, (-margin, -margin), resolution


def write_map(base: str, img, origin, resolution) -> Tuple[str, str]:
    import os
    pgm, yml = base + '.pgm', base + '.yaml'
    h, w = img.shape
    with open(pgm, 'wb') as f:
        f.write(f'P5\n# AMR factory map (lidar plane slice)\n{w} {h}\n255\n'.encode())
        f.write(np.flipud(img).tobytes())
    with open(yml, 'w') as f:
        f.write(f'image: {os.path.basename(pgm)}\nmode: trinary\nresolution: {resolution}\n'
                f'origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm, yml


# ===================================================================== world
def world_sdf(layout: dict, step: float = 0.004, rtf: float = 1.0, animate: bool = True,
              line_topic: str = '/factory/line_state', mesh: Optional[Tuple[str, str]] = None):
    """Return (sdf_text, map_slices).

    mesh=(obj_file, uri): bake the opaque static geometry into obj_file (written
    here) and reference it as uri; without it every primitive is an SDF visual."""
    global _BAKER
    _BAKER = MeshBaker() if mesh else None
    try:
        return _world_sdf(layout, step, rtf, animate, line_topic, mesh)
    finally:
        _BAKER = None


def _world_sdf(layout, step, rtf, animate, line_topic, mesh):
    graph = build_graph(layout)
    W, H = layout.get('factory', {}).get('size', [100.0, 60.0])
    slices = MapSlices()
    lights: List[str] = []
    body: List[str] = []
    segments: List[dict] = []
    build_ground(W, H, body)
    build_floor(layout, W, H, body)
    build_walls(layout, slices, body)
    build_roof(layout, body, lights)
    objects = list(layout.get('objects', []) or []) + line_objects(layout) + \
        station_objects(layout, graph)
    build_columns(layout, graph, objects, slices, body)
    build_markings(layout, graph, body)
    build_objects(layout, slices, body)
    build_lines(layout, slices, body, segments)
    build_station_objects(layout, graph, slices, body)
    build_charging_area(layout, graph, slices, body)
    build_wall_items(layout, slices, body)
    build_office(layout, body)
    build_trucks(layout, body)

    if mesh and _BAKER is not None and _BAKER.count:
        obj_file, uri = mesh
        _BAKER.write(obj_file)
        body.append(f'<model name="factory_static"><static>true</static><link name="link">'
                    f'<visual name="baked"><geometry><mesh><uri>{escape(uri)}</uri></mesh>'
                    f'</geometry><cast_shadows>false</cast_shadows></visual></link></model>\n')
    anim = ''
    if animate and segments:
        segs = ''.join(
            f'<segment line="{s["line"]}" name="{s["name"]}">'
            f'<start>{s["start"][0]:.3f} {s["start"][1]:.3f} {s["start"][2]:.3f}</start>'
            f'<end>{s["end"][0]:.3f} {s["end"][1]:.3f} {s["end"][2]:.3f}</end>'
            f'<size>{s["size"][0]} {s["size"][1]} {s["size"][2]}</size>'
            f'<color>{s["color"][0]} {s["color"][1]} {s["color"][2]} {s["color"][3]}</color>'
            f'<spacing>{s["spacing"]}</spacing><speed>{s["speed"]}</speed></segment>'
            for s in segments)
        anim = (f'  <plugin filename="amr_line_animator" name="amr_gazebo::LineAnimator">'
                f'<topic>{line_topic}</topic>{segs}</plugin>\n')
    head = f'''<?xml version="1.0"?>
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
{anim}  <gravity>0 0 -9.81</gravity>
  <scene>
    <ambient>0.62 0.62 0.62 1</ambient>
    <background>0.62 0.72 0.82 1</background>
    <shadows>false</shadows>
    <grid>false</grid>
  </scene>
  <light type="directional" name="sun">
    <cast_shadows>false</cast_shadows>
    <pose>{W / 2} {H / 2} 40 0 0 0</pose>
    <diffuse>0.75 0.75 0.72 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.3 0.25 -0.9</direction>
  </light>
  {''.join(lights)}
  <gui fullscreen="0">
    <camera name="user_camera">
      <pose>{W / 2} {-26} 46 0 0.82 1.5708</pose>
    </camera>
  </gui>
'''
    return head + ''.join(body) + '</world>\n</sdf>\n', slices


def write_world_and_map(layout: dict, world_file: str, map_base: str = '',
                        step: float = 0.004, rtf: float = 1.0, animate: bool = True) -> Optional[str]:
    """Write the world SDF and (optionally) the matching map; return the map yaml."""
    import os
    obj = os.path.join(os.path.dirname(os.path.abspath(world_file)), 'factory_static.obj')
    sdf, slices = world_sdf(layout, step, rtf, animate=animate, mesh=(obj, 'file://' + obj))
    with open(world_file, 'w', encoding='utf-8') as f:
        f.write(sdf)
    if not map_base:
        return None
    img, origin, res = rasterize(slices, layout['factory']['size'])
    return write_map(map_base, img, origin, res)[1]


def write_factory_map(layout: dict, map_base: str) -> str:
    """Only the AMCL / map_server map of the generated world; return the yaml."""
    _, slices = world_sdf(layout, animate=False)
    img, origin, res = rasterize(slices, layout['factory']['size'])
    return write_map(map_base, img, origin, res)[1]


def main(argv: Sequence[str] = None) -> None:
    ap = argparse.ArgumentParser(description='Generate the Gazebo factory world from the layout')
    ap.add_argument('--layout', required=True)
    ap.add_argument('--out', required=True, help='world SDF file')
    ap.add_argument('--map', default='', help='also write <base>.pgm/.yaml occupancy map')
    ap.add_argument('--step', type=float, default=0.004, help='physics max step size [s]')
    ap.add_argument('--rtf', type=float, default=1.0, help='target real time factor')
    ap.add_argument('--no-animation', action='store_true')
    ap.add_argument('--mesh', default='',
                    help='bake static geometry into this OBJ file (default: next to --out)')
    ap.add_argument('--mesh-uri', default='', help='URI of the OBJ in the world '
                    '(default file://<abs path>; e.g. model://amr_factory_static/meshes/...)')
    ap.add_argument('--no-mesh', action='store_true', help='plain SDF visuals only (slow)')
    args = ap.parse_args(argv)
    layout = load_layout(args.layout)
    import os
    mesh = None
    if not args.no_mesh:
        obj = os.path.abspath(args.mesh or os.path.join(os.path.dirname(os.path.abspath(args.out)),
                                                        'factory_static.obj'))
        os.makedirs(os.path.dirname(obj), exist_ok=True)
        mesh = (obj, args.mesh_uri or 'file://' + obj)
    sdf, slices = world_sdf(layout, args.step, args.rtf, animate=not args.no_animation,
                            mesh=mesh)
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(sdf)
    print(f'wrote {args.out} ({len(sdf) / 1024:.0f} kB)')
    if args.map:
        img, origin, res = rasterize(slices, layout['factory']['size'])
        pgm, yml = write_map(args.map, img, origin, res)
        print(f'wrote {pgm} {yml} ({img.shape[1]}x{img.shape[0]})')


if __name__ == '__main__':
    main()
