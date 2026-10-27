"""Generate the Gazebo (Harmonic / Jetty) factory world from factory_layout.yaml.

  ros2 run amr_fleet_manager generate_world --layout factory_layout.yaml \\
       --out factory.sdf --map factory_map

Everything is built from primitives (no external model downloads): an
insulated-panel hall with concrete base, window band, roller doors and steel
columns, roof beams with light fixtures and real lights, epoxy floor zones and
lane markings, component racks full of bins and cartons, pallet racks with
wrapped pallets, production lines (conveyor, CNC, robot cell, assembly
bench, QC gate, case packer, control cabinet) and packaging lines (unboxer,
flow wrapper, case packer, robot palletizer), a work cell at both ends of every
line, the chargers, office annex, trucks at the docks, fire extinguishers, signs...

The FactoryLines system (amr_gazebo) runs the work cells from the MES state:
the station arm moves the cartons between the AMR and the pedestal, the loader
arm puts every product of the open carton on the belt, the product crosses
the line and is packed into the carton on the output pedestal.

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

from ..core.docking import CONTACT_DEPTH
from ..core.layout import (BELT_MARGIN, BELT_TOP, CELL_ARM, CELL_LOADER, CELL_SLOTS,
                           CELL_STACK, LINE_CELLS, PEDESTAL_TOP, build_graph, cell_point,
                           line_lead_time, line_objects, load_layout, people_routes,
                           people_segments, station_objects)

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
    'robot_orange': (0.98, 0.45, 0.05, 1), 'copper': (0.80, 0.45, 0.20, 1), 'fence': (0.95, 0.80, 0.10, 0.55),
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
        cuts = sorted((d['at'] - d['width'] / 2, d['at'] + d['width'] / 2, bool(d.get('open')))
                      for d in doors if d['wall'] == name)
        segs, s = [], -t
        for a, b, _ in cuts:
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
        # doors: roller shutters with a yellow frame, lintel above; an open
        # one (shipping gate) has its shutter rolled up under the lintel
        for a, b, is_open in cuts:
            if is_open:
                place(a, b, 4.55, 5.0, 'door', inset=-0.15, collide=False)
                place(a, b, 0.0, 0.03, 'steel_light', inset=0.0, collide=False)   # threshold
            else:
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
    """Charger unit facing -y (towards its dock node).  The flat face is what the
    AMR's lidar detects for docking; the contact plate sticks out CONTACT_DEPTH at
    bumper height, below the lidar plane, where the docked AMR's bumper touches it."""
    x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
    h = float(o.get('height', 1.1))
    m.box(cx, cy, h / 2, sx, sy, h, 'charger', collide=True)
    m.box(cx, cy - sy / 2 - 0.01, h * 0.72, sx * 0.7, 0.02, h * 0.25, 'screen_on')
    plate_y = cy - sy / 2 - CONTACT_DEPTH / 2
    m.box(cx, plate_y, 0.10, sx * 0.6, CONTACT_DEPTH, 0.08, 'dark')           # contact arm
    for dx in (-0.12, 0.12):                                                   # copper pads
        m.box(cx + dx, plate_y - CONTACT_DEPTH / 2 - 0.004, 0.10, 0.12, 0.008, 0.05,
              (0.85, 0.55, 0.25, 1))
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


# ------------------------------------------------------------- robot arms
# 6-axis industrial arm.  Every link is a list of parts in its own frame:
#   turret  origin on top of the pedestal, rotates about z (J1)
#   upper   origin on the shoulder axis (J2), the link points along +z
#   fore    origin on the elbow axis (J3), the link points along +x
#   tool    origin on the wrist, the tool hangs along -z (vertical flange)
# The same tables draw the static arms of the line cells (baked into the
# factory mesh) and the animated station arms (one OBJ per link, posed by the
# FactoryLines system of amr_gazebo with the same inverse kinematics).
ARM_DIMS = {'a1': 0.10, 'd1': 0.30, 'l2': 0.85, 'l3': 0.90, 'tool': 0.19}
CARTON = (0.56, 0.40, 0.22)           # AMR load (amr_description cargo_link)
DECK_TOP = 0.515                      # top of the load on the AMR deck
ARM_PEDESTAL = 0.45                   # station arms stand on a pedestal
LOADER_SCALE = 0.85                   # the loader arms (products only) are smaller
LOADER_HOME = (0.40, 1.25)            # loader rest: this far from its base towards the slots, z
# a move that turns the turret by more than SWING_MIN first pulls the tool in to
# this radius around the base, turns, then reaches out (station arm, loader):
# each arm keeps to its own half of the work cell (FactoryLines, cell_clearance)
SAFE_RHO = (0.50, 0.60)
SWING_MIN = 0.35                      # rad


def loader_home(base, slot) -> Tuple[float, float, float]:
    """Rest point of a loader arm: above the floor between its base and the
    middle slot (same as amr_gazebo/FactoryLines)."""
    dx, dy = slot[0] - base[0], slot[1] - base[1]
    d = math.hypot(dx, dy) or 1.0
    return (base[0] + dx / d * LOADER_HOME[0], base[1] + dy / d * LOADER_HOME[0], LOADER_HOME[1])


def arm_dims(k: float = 1.0) -> Dict[str, float]:
    return {n: v * k for n, v in ARM_DIMS.items()}


def arm_links(k: float = 1.0, color='robot_orange', tool: str = 'vacuum') -> Dict[str, list]:
    """Parts of every link: ('box', (x, y, z), (sx, sy, sz), mat) or
    ('cyl', (x, y, z), r, h, axis, mat) in the link frame."""
    d = arm_dims(k)
    a1, d1, l2, l3 = d['a1'], d['d1'], d['l2'], d['l3']
    dark, steel = 'machine_dark', 'steel_light'
    links = {
        'base': [('cyl', (0, 0, 0.05 * k), 0.20 * k, 0.10 * k, 'z', dark),
                 ('cyl', (0, 0, 0.105 * k), 0.17 * k, 0.012 * k, 'z', steel)],
        'turret': [('cyl', (0, 0, 0.14 * k), 0.18 * k, 0.07 * k, 'z', color),
                   ('box', (0.03 * k, 0, 0.23 * k), (0.36 * k, 0.32 * k, 0.16 * k), color),
                   ('box', (-0.17 * k, 0, 0.21 * k), (0.12 * k, 0.20 * k, 0.15 * k), dark),
                   ('cyl', (a1, 0.19 * k, d1), 0.09 * k, 0.07 * k, 'y', dark),
                   ('cyl', (a1, -0.17 * k, d1), 0.10 * k, 0.03 * k, 'y', steel)],
        'upper': [('cyl', (0, 0, 0), 0.125 * k, 0.32 * k, 'y', color),
                  ('box', (0, 0, l2 / 2), (0.15 * k, 0.20 * k, l2), color),
                  ('box', (0, 0.104 * k, l2 / 2), (0.08 * k, 0.012 * k, 0.7 * l2), dark),
                  ('box', (0, -0.104 * k, l2 / 2), (0.08 * k, 0.012 * k, 0.7 * l2), dark),
                  ('cyl', (-0.12 * k, 0, 0.32 * l2), 0.035 * k, 0.5 * l2, 'z', steel),
                  ('cyl', (-0.12 * k, 0, 0.62 * l2), 0.05 * k, 0.16 * l2, 'z', dark),
                  ('cyl', (0, 0, l2), 0.105 * k, 0.28 * k, 'y', color)],
        'fore': [('box', (-0.08 * k, 0, 0), (0.42 * k, 0.22 * k, 0.21 * k), color),
                 ('box', (-0.36 * k, 0, 0), (0.15 * k, 0.17 * k, 0.17 * k), dark),
                 ('cyl', ((0.13 * k + l3 - 0.08 * k) / 2, 0, 0), 0.07 * k, l3 - 0.21 * k, 'x', color),
                 ('cyl', (l3 - 0.06 * k, 0, 0), 0.075 * k, 0.05 * k, 'x', dark),
                 ('cyl', (l3, 0, 0), 0.065 * k, 0.17 * k, 'y', dark),
                 ('box', (0.45 * l3, 0, 0.09 * k), (0.5 * l3, 0.05 * k, 0.03 * k), dark)],
        'tool': [('cyl', (0, 0, -0.045 * k), 0.055 * k, 0.09 * k, 'z', color),
                 ('cyl', (0, 0, -0.10 * k), 0.065 * k, 0.025 * k, 'z', steel)],
    }
    t = d['tool']
    if tool == 'vacuum':            # suction plate sized for the AMR carton
        links['tool'] += [('box', (0, 0, -0.125 * k), (0.46 * k, 0.30 * k, 0.03 * k), dark)]
        for sx in (-0.17, 0.17):
            for sy in (-0.10, 0.10):
                links['tool'].append(('cyl', (sx * k, sy * k, -t + 0.025 * k), 0.035 * k,
                                      0.05 * k, 'z', 'black'))
    elif tool == 'gripper':         # fork gripper of a palletizer
        links['tool'] += [('box', (0, 0, -0.125 * k), (0.62 * k, 0.16 * k, 0.04 * k), dark)]
        for sx in (-0.30, 0.30):
            links['tool'].append(('box', (sx * k, 0, -t), (0.03 * k, 0.30 * k, 0.15 * k), steel))
    elif tool == 'pincer':          # small parallel gripper of a loader arm
        links['tool'] += [('box', (0, 0, -0.12 * k), (0.07 * k, 0.16 * k, 0.03 * k), dark)]
        for sy in (-0.065, 0.065):
            links['tool'].append(('box', (0, sy * k, -t + 0.03 * k), (0.04 * k, 0.015 * k, 0.07 * k),
                                  steel))
    elif tool == 'torch':           # welding torch with its cable
        links['tool'] += [('box', (0, 0, -0.13 * k), (0.12 * k, 0.12 * k, 0.04 * k), dark),
                          ('cyl', (0, 0, -0.18 * k), 0.022 * k, 0.10 * k, 'z', 'copper'),
                          ('cyl', (0, 0, -t + 0.005 * k), 0.012 * k, 0.02 * k, 'z', 'dark')]
    return links


def _rot_z(a: float) -> np.ndarray:
    return _rpy_matrix(0.0, 0.0, a)


def arm_ik(base, p, k: float = 1.0) -> Tuple[float, float, float, Tuple[float, ...]]:
    """Joint angles (q1 azimuth, q2 shoulder lean, q3 forearm down pitch) and the
    wrist position for the tool tip p, wrist above the tip, elbow up.  Same
    solution as amr_gazebo/FactoryLines."""
    d = arm_dims(k)
    bx, by, bz = base
    q1 = math.atan2(p[1] - by, p[0] - bx)
    r = max(0.05, math.hypot(p[0] - bx, p[1] - by) - d['a1'])
    z = p[2] + d['tool'] - (bz + d['d1'])
    l2, l3 = d['l2'], d['l3']
    phi = math.atan2(r, z)
    dist = min(max(math.hypot(r, z), abs(l2 - l3) + 1e-3), l2 + l3 - 1e-3)
    r, z = dist * math.sin(phi), dist * math.cos(phi)
    cos_a = (l2 * l2 + dist * dist - l3 * l3) / (2.0 * l2 * dist)
    q2 = phi - math.acos(min(1.0, max(-1.0, cos_a)))
    er, ez = l2 * math.sin(q2), l2 * math.cos(q2)
    q3 = math.atan2(-(z - ez), r - er)
    c, s = math.cos(q1), math.sin(q1)
    wrist = (bx + (d['a1'] + r) * c, by + (d['a1'] + r) * s, bz + d['d1'] + z)
    return q1, q2, q3, wrist


def arm_frames(base, p, tool_yaw: float, k: float = 1.0) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """World frame (origin, rotation) of every link for the tool tip p."""
    d = arm_dims(k)
    q1, q2, q3, w = arm_ik(base, p, k)
    radial = np.array([math.cos(q1), math.sin(q1), 0.0])
    b = np.array(base, dtype=float)
    s = b + radial * d['a1'] + (0, 0, d['d1'])
    e = s + radial * d['l2'] * math.sin(q2) + (0, 0, d['l2'] * math.cos(q2))
    return {'base': (b, np.eye(3)), 'turret': (b, _rot_z(q1)),
            'upper': (s, _rpy_matrix(0.0, q2, q1)), 'fore': (e, _rpy_matrix(0.0, q3, q1)),
            'tool': (np.array(w), _rot_z(tool_yaw))}


_AXIS = {'z': np.eye(3), 'x': _rpy_matrix(0.0, math.pi / 2, 0.0), 'y': _rpy_matrix(-math.pi / 2, 0.0, 0.0)}


def draw_parts(sink, parts, origin=(0.0, 0.0, 0.0), R=None, **kw) -> None:
    """Draw link parts into a Model (kw: in_map) or a MeshBaker, placed in the
    frame (origin, R)."""
    R = np.eye(3) if R is None else R
    o = np.asarray(origin, dtype=float)
    model = isinstance(sink, Model)
    for part in parts:
        c = o + R @ np.asarray(part[1], dtype=float)
        if part[0] == 'box':
            M = R
            roll, pitch, yaw = (math.atan2(M[2, 1], M[2, 2]), -math.asin(max(-1.0, min(1.0, M[2, 0]))),
                                math.atan2(M[1, 0], M[0, 0]))
            sx, sy, sz = part[2]
            if model:
                sink.box(*c, sx, sy, sz, part[3], yaw, roll=roll, pitch=pitch, **kw)
            else:
                sink.box(*c, sx, sy, sz, part[3], roll, pitch, yaw)
        else:
            _, _, r, h, axis, mat = part
            dx, dy, dz = R @ _AXIS[axis] @ (0.0, 0.0, 1.0)
            roll = math.asin(max(-1.0, min(1.0, -dy)))
            pitch = math.atan2(dx, dz) if math.hypot(dx, dz) > 1e-9 else 0.0
            if model:
                sink.cyl(*c, r, h, mat, roll=roll, pitch=pitch, **kw)
            else:
                sink.cyl(*c, r, h, mat, roll, pitch)


def add_robot_arm(m: Model, x, y, base_z, target, tool_yaw=0.0, color='robot_orange', k=1.0,
                  tool='vacuum', pedestal=0.0, carry=False) -> None:
    """Static arm on a pedestal at (x, y, base_z), posed with its tool tip on target."""
    if pedestal > 0.0:
        m.box(x, y, base_z + 0.02, 0.56 * k, 0.56 * k, 0.04, 'machine_dark')
        m.cyl(x, y, base_z + pedestal / 2, 0.17 * k, pedestal, 'machine_dark')
    base = (x, y, base_z + pedestal)
    links = arm_links(k, color, tool)
    for name, (o, R) in arm_frames(base, target, tool_yaw, k).items():
        draw_parts(m, links[name], o, R)
    if carry:
        m.box(target[0], target[1], target[2] - CARTON[2] / 2, *CARTON, 'carton', tool_yaw)


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
    # welding robot on a riser, torch on the seam of a part clamped on the positioner
    m.box(cx - 0.45, cy - 0.3, 0.40, 0.9, 0.7, 0.8, 'machine_dark')          # positioner
    m.box(cx - 0.45, cy - 0.3, 0.82, 1.0, 0.8, 0.04, 'steel')
    m.box(cx - 0.45, cy - 0.3, 0.93, 0.55, 0.35, 0.18, 'machine_orange')     # work piece
    for dy in (-0.22, 0.22):
        m.box(cx - 0.45, cy - 0.3 + dy, 0.9, 0.08, 0.06, 0.12, 'black')       # clamps
    add_robot_arm(m, cx + 0.55, cy - 0.3, 0.0, (cx - 0.25, cy - 0.3, 1.04), math.pi / 2,
                  k=0.85, tool='torch', pedestal=0.3)
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
    m.box(cx + 0.4, y0 + 0.4, 1.05, 0.45, 0.4, 0.35, 'carton')
    add_robot_arm(m, cx - 0.1, cy - 0.3, 0.87, (cx + 0.4, y0 + 0.4, 1.225), 0.0,
                  color='machine_blue', k=0.7)
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
    add_robot_arm(m, cx - 0.2, cy + 0.6, 0.0, (cx - 0.2, cy - 0.75, 1.50), math.pi / 2,
                  tool='gripper', pedestal=0.3, carry=True)
    m.box(cx - 0.2, cy - 0.9, 0.07, 1.0, 1.0, 0.14, 'pallet')
    for k in range(3):
        m.box(cx - 0.2, cy - 0.9, 0.29 + k * 0.3, 0.9, 0.9, 0.28, 'carton')
    o = objs['pallet_out']
    add_pallet(m, o, 1)


def build_lines(layout, slices, out: List[str]) -> None:
    by_line: Dict[str, Dict[str, dict]] = {}
    for o in line_objects(layout):
        by_line.setdefault(o['line'], {})[o['kind']] = o
    for ln in layout.get('lines', []) or []:
        m = Model(f'line_{ln["id"]}', slices)
        objs = by_line[ln['id']]
        if ln.get('kind', 'production') == 'production':
            add_production_line(m, ln, objs)
        else:
            add_packaging_line(m, ln, objs)
        out.append(m.sdf())


def build_station_objects(layout, graph, slices, out: List[str]) -> None:
    """Kit racks, storage tables and the static part of the line work cells."""
    for o in station_objects(layout, graph):
        m = Model(f'station_{o["station"]}', slices)
        along, across = o['size']
        x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
        h = float(o.get('height', 0.6))
        yaw = o['yaw']
        loc = Local(m, cx, cy, yaw)
        if o['kind'] not in LINE_CELLS:
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
        elif kind in LINE_CELLS:
            add_work_cell(m, o)
        elif kind in ('wh_inbound', 'wh_outbound', 'ship_dock'):
            add_transfer_table(m, o, loc, along, across, h)
        else:   # storage table
            loc.box(0, 0, h - 0.03, along, across, 0.06, 'galv')
            for dx in (-along / 2 + 0.05, along / 2 - 0.05):
                for dy in (-across / 2 + 0.05, across / 2 - 0.05):
                    loc.box(dx, dy, h / 2, 0.05, 0.05, h, 'galv')
            loc.box(0, 0, h + 0.2, 0.5, 0.42, 0.34, 'wrap')
        out.append(m.sdf())


def add_transfer_table(m: Model, o: dict, loc: Local, along: float, across: float, h: float) -> None:
    """Roller table the AMR unloads onto / loads from (local y = left of the
    lane, the table stands on the right).  wh_inbound: put-away table in
    front of the warehouse racks, wh_outbound: picking table with the next
    cartons of finished products waiting, ship_dock: dock table, its roller
    conveyor runs on through the open dock door into the truck."""
    kind = o['kind']
    loc.box(0, 0, h - 0.06, along, across, 0.04, 'steel')
    for k in range(int(along / 0.12)):
        loc.box(-along / 2 + 0.06 + k * 0.12, 0, h - 0.025, 0.05, across - 0.08, 0.05, 'roller',
                in_map=False)
    for dx in (-along / 2 + 0.05, along / 2 - 0.05):
        for dy in (-across / 2 + 0.05, across / 2 - 0.05):
            loc.box(dx, dy, (h - 0.08) / 2, 0.05, 0.05, h - 0.08, 'galv')
    # side guide on the far side, sign post
    loc.box(0, -across / 2 + 0.02, h + 0.06, along, 0.03, 0.08, 'yellow', in_map=False)
    sign = {'wh_inbound': 'green', 'wh_outbound': 'beam_blue', 'ship_dock': 'amber'}[kind]
    loc.box(along / 2 + 0.12, -across / 2, 1.1, 0.05, 0.05, 2.2, 'steel')
    loc.box(along / 2 + 0.12, -across / 2, 2.15, 0.04, 0.55, 0.28, sign, in_map=False)
    if kind == 'wh_outbound':
        # two more cartons waiting on the upper level of the flow rack
        loc.box(0, -0.05, h + 0.55, along, across * 0.8, 0.03, 'steel', in_map=False)
        for dx in (-0.3, 0.3):
            loc.box(dx, -0.05, h + 0.68, 0.5, 0.36, 0.22, 'carton', in_map=False)
        for dx in (-along / 2 + 0.05, along / 2 - 0.05):
            loc.box(dx, -across / 2 + 0.05, h + 0.3, 0.05, 0.05, 0.6, 'galv', in_map=False)
    elif kind == 'ship_dock':
        # conveyor from the table through the door to the trailer, dock plate
        x0, y0, x1, y1, cx, cy, sx, sy = _rect(o)
        out_len = y0 + 2.6                      # the hall wall is at y = 0, the trailer at -2.5
        m.box(cx, y0 - out_len / 2, h - 0.06, 0.8, out_len, 0.04, 'steel')
        for k in range(int(out_len / 0.12)):
            m.box(cx, y0 - 0.06 - k * 0.12, h - 0.025, 0.72, 0.05, 0.05, 'roller', in_map=False)
        for yy in np.arange(y0 - 0.3, -2.4, -0.9):
            for dx in (-0.35, 0.35):
                m.box(cx + dx, float(yy), (h - 0.08) / 2, 0.05, 0.05, h - 0.08, 'galv',
                      in_map=yy > 0)
        m.box(cx, -1.4, 0.02, 3.0, 2.0, 0.04, 'steel_light', in_map=False)    # dock apron
        # safety bollards both sides of the conveyor inside the door
        for dx in (-1.25, 1.25):
            m.cyl(cx + dx, 0.45, 0.55, 0.08, 1.1, 'yellow', collide=True)


def _link_model(name: str, pose, geometry: str) -> str:
    x, y, z, roll, pitch, yaw = pose
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'
            f'<link name="link">{geometry}</link></model>\n')


def _vis(name, size, mat, pose=(0.0, 0.0, 0.0)) -> str:
    return (f'<visual name="{name}"><pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>'
            f'<geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box>'
            f'</geometry>{_mat(mat)}<cast_shadows>false</cast_shadows></visual>')


# carton styles: (box, band) - kit of components, carton of unpacked products,
# carton of finished goods
CARTON_STYLE = {'kit': ('carton', 'bin_blue'), 'semi': ('carton', 'dark'),
                'fg': ('carton_dark', 'green')}
# product looks along the lines: (size, body material, cap material, film)
# (16 per carton: 4 x 4 cells of 0.13 x 0.09 m)
ITEM_LOOK = {'part': ((0.10, 0.078, 0.06), 'steel_light', 'steel', False),
             'product': ((0.10, 0.078, 0.08), 'machine_orange', 'machine_dark', False),
             'wrapped': ((0.11, 0.084, 0.09), 'machine_orange', 'machine_dark', True)}
TRAY_WALL = 0.015


def _carton_geometry(style: str = 'semi') -> str:
    """Closed carton, origin in its centre."""
    sx, sy, sz = CARTON
    body, band = CARTON_STYLE[style]
    return (_vis('box', CARTON, body)
            + _vis('tape', (sx + 0.01, 0.06, 0.004), band, (0, 0, sz / 2 + 0.001))
            + _vis('label', (0.004, 0.16, 0.10), 'white', (sx / 2 + 0.002, 0, 0.0)))


def _tray_geometry(style: str = 'semi') -> str:
    """Open carton (flaps folded down), origin in the centre of its volume."""
    sx, sy, sz = CARTON
    body, band = CARTON_STYLE[style]
    t = TRAY_WALL
    return (_vis('floor', (sx, sy, t), body, (0, 0, -sz / 2 + t / 2))
            + _vis('w1', (sx, t, sz), body, (0, sy / 2 - t / 2, 0))
            + _vis('w2', (sx, t, sz), body, (0, -sy / 2 + t / 2, 0))
            + _vis('w3', (t, sy, sz), body, (sx / 2 - t / 2, 0, 0))
            + _vis('w4', (t, sy, sz), body, (-sx / 2 + t / 2, 0, 0))
            + _vis('band', (0.004, 0.16, 0.06), band, (sx / 2 + 0.002, 0, 0.0)))


def _item_geometry(look: str) -> str:
    """One product, origin in its centre."""
    (sx, sy, sz), body, cap, film = ITEM_LOOK[look]
    if film:        # shrink-wrapped product: the product inside a film
        core = (sx - 0.02, sy - 0.02, sz - 0.02)
        return (_vis('core', core, body) + _vis('cap', (core[0], core[1] * 0.5, 0.006), cap,
                                                (0, 0, core[2] / 2 + 0.003))
                + _vis('film', (sx, sy, sz), 'wrap'))
    return _vis('body', (sx, sy, sz * 0.8), body, (0, 0, -sz * 0.1)) + \
        _vis('cap', (sx * 0.7, sy * 0.7, sz * 0.2), cap, (0, 0, sz * 0.4))


def add_robot_arm_pedestal(m: Model, base, k: float = 1.0) -> None:
    x, y, top = base
    m.box(x, y, 0.02, 0.56 * k, 0.56 * k, 0.04, 'machine_dark')
    m.cyl(x, y, top / 2, 0.17 * k, top, 'machine_dark')
    for dx in (-0.22 * k, 0.22 * k):
        for dy in (-0.22 * k, 0.22 * k):
            m.cyl(x + dx, y + dy, 0.05, 0.025, 0.03, 'steel_light')
    draw_parts(m, arm_links(k)['base'], base)


def add_work_cell(m: Model, o: dict) -> None:
    """Static part of a line work cell: the 3-slot pedestal (a row parallel
    to the lane), the pedestals of the station arm (AMR side) and of the
    loader arm (belt side), the stack of folded cartons and a sign post
    (moving parts: build_line_cells)."""
    yaw = o['yaw']
    pts = [cell_point(o, u, v) for u, v in CELL_SLOTS]
    (ax, ay), (bx, by) = pts[0], pts[-1]
    cx, cy = (ax + bx) / 2, (ay + by) / 2
    length = math.hypot(bx - ax, by - ay) + CARTON[0] + 0.20      # slots along u
    width = CARTON[1] + 0.16
    loc = Local(m, cx, cy, yaw)
    top = PEDESTAL_TOP
    # pedestal: u runs along the lane = x of the Local frame
    loc.box(0, 0, top - 0.025, length, width, 0.05, 'machine_blue', collide=True)
    for du in (-length / 2 + 0.06, length / 2 - 0.06):
        for dv in (-width / 2 + 0.04, width / 2 - 0.04):
            loc.box(du, dv, (top - 0.05) / 2, 0.05, 0.05, top - 0.05, 'galv')
    loc.box(0, 0, 0.12, length - 0.12, width - 0.08, 0.03, 'galv', in_map=False)
    for (px, py) in pts:                                   # slot outlines on the top
        Local(m, px, py, yaw).box(0, 0, top + 0.002, CARTON[0] + 0.04, CARTON[1] + 0.04, 0.004,
                                  'yellow')
        Local(m, px, py, yaw).box(0, 0, top + 0.004, CARTON[0] - 0.02, CARTON[1] - 0.02, 0.004,
                                  'machine_blue')
    for (u, v), k in ((CELL_ARM, 1.0), (CELL_LOADER, LOADER_SCALE)):
        x, y = cell_point(o, u, v)
        add_robot_arm_pedestal(m, (x, y, ARM_PEDESTAL), k)
    # stack of folded cartons on a small cart (spent ones in, new ones out)
    x, y = cell_point(o, *CELL_STACK)
    st = Local(m, x, y, yaw)
    st.box(0, 0, 0.30, 0.62, 0.46, 0.04, 'steel', collide=True)
    for du in (-0.27, 0.27):
        for dv in (-0.19, 0.19):
            st.box(du, dv, 0.15, 0.04, 0.04, 0.28, 'steel')
    for k in range(5):
        st.box(0, 0, 0.335 + k * 0.03, 0.58, 0.42, 0.025, 'carton' if k % 2 else 'carton_dark')
    # sign post on the corner of the cell away from the AMR and the arms
    x, y = cell_point(o, -1.65, 3.10)
    m.box(x, y, 1.2, 0.05, 0.05, 2.4, 'steel')
    Local(m, x, y, yaw).box(0, 0, 2.3, 0.6, 0.04, 0.3,
                            'machine_orange' if o['kind'] == 'line_in' else 'pad_station')


STACK_TOP = 0.335 + 5 * 0.03


def _arm_geometry(mesh, prefix: str, color: str, tool: str, k: float = 1.0) -> Dict[str, str]:
    """Visual of every moving arm link (one OBJ per link with a mesh)."""
    links = arm_links(k, color, tool)
    geometry = {}
    for name in ('turret', 'upper', 'fore', 'tool'):
        if mesh:
            import os
            obj_file, uri = mesh
            baker = MeshBaker()
            draw_parts(baker, links[name])
            baker.write(os.path.join(os.path.dirname(os.path.abspath(obj_file)),
                                     f'{prefix}_{name}.obj'))
            geometry[name] = (f'<visual name="v"><geometry><mesh><uri>{escape(uri.rsplit("/", 1)[0])}'
                              f'/{prefix}_{name}.obj</uri></mesh></geometry>'
                              f'<cast_shadows>false</cast_shadows></visual>')
        else:
            m = Model(name)
            m.baker = None
            draw_parts(m, links[name], in_map=False)
            geometry[name] = ''.join(m.visuals)
    return geometry


def _posed_arm(name: str, base, tip, yaw: float, geometry: Dict[str, str], k: float = 1.0) -> str:
    out = []
    for link, (o, R) in arm_frames(base, tip, yaw, k).items():
        if link not in geometry:
            continue
        rpy = (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
               math.atan2(R[1, 0], R[0, 0]))
        out.append(_link_model(f'{name}_{link}', (*o, *rpy), geometry[link]))
    return ''.join(out)


HIDDEN = -20.0


def line_cells(layout: dict, graph) -> List[dict]:
    """Everything the FactoryLines system needs per line: belt hand-off
    points, timing, product looks and both work cells."""
    process = layout.get('process', {}) or {}
    takt = {c['id']: float(c.get('cycle_time', 22.0)) for c in process.get('production_lines', [])}
    takt.update({c['id']: float(c.get('pack_time', 21.0)) for c in process.get('packaging_lines', [])})
    # products per carton: the production line's box, the packer's pallet (= box)
    box = {c['id']: int(c.get('box_capacity', 12)) for c in process.get('production_lines', [])}
    size = max(box.values() or [12])
    box.update({c['id']: int(c.get('pallet_capacity', size)) for c in process.get('packaging_lines', [])})
    cells = {o['station']: o for o in station_objects(layout, graph) if o['kind'] in LINE_CELLS}
    ends = {}
    for c in process.get('production_lines', []) + process.get('packaging_lines', []):
        ends[c['id']] = (c['input'], c['output'])
    out = []
    for ln in layout.get('lines', []) or []:
        lid = ln['id']
        if lid not in ends or ends[lid][0] not in cells or ends[lid][1] not in cells:
            continue
        prod = ln.get('kind', 'production') == 'production'
        x, yi, yo = float(ln['x']), float(ln['y_in']), float(ln['y_out'])
        s = 1.0 if yi > yo else -1.0
        t_load, t_belt, t_drop = line_lead_time(process, ln)
        lead = t_load + t_belt + t_drop
        n_way = int(math.ceil(lead / takt.get(lid, 20.0))) + 2
        cap = box.get(lid, 12)
        out.append({
            'id': lid, 'kind': 'production' if prod else 'packaging',
            'belt': ((x, yi - s * BELT_MARGIN), (x, yo + s * BELT_MARGIN)),
            'timing': (t_load, t_belt, t_drop), 'cap': cap,
            'transform': 0.5 if prod else 0.4,
            'looks': ('part', 'product') if prod else ('product', 'wrapped'),
            'styles': (('kit', 'semi') if prod else ('semi', 'fg')),
            'pool': (cap + n_way, n_way + cap),
            'in': cells[ends[lid][0]], 'out': cells[ends[lid][1]]})
    return out


def build_line_cells(layout, graph, out: List[str], topic: str, mesh=None) -> str:
    """Moving parts of the line work cells (arms, cartons, open cartons,
    products) and the FactoryLines plugin element that drives them."""
    lines = line_cells(layout, graph)
    if not lines:
        return ''
    station_geo = _arm_geometry(mesh, 'arm', 'robot_orange', 'vacuum')
    loader_geo = _arm_geometry(mesh, 'arm_loader', 'machine_blue', 'pincer', LOADER_SCALE)
    hidden = (0.0, 0.0, HIDDEN, 0, 0, 0)
    cfg = []

    def v3(p) -> str:
        return f'{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}'

    for L in lines:
        ends = []
        for side, style in (('in', L['styles'][0]), ('out', L['styles'][1])):
            o = L[side]
            sid, yaw = o['station'], o['yaw']
            arm = (*cell_point(o, *CELL_ARM), ARM_PEDESTAL)
            loader = (*cell_point(o, *CELL_LOADER), ARM_PEDESTAL)
            slots = [(*cell_point(o, u, v), PEDESTAL_TOP) for u, v in CELL_SLOTS]
            stack = (*cell_point(o, *CELL_STACK), STACK_TOP)
            service = o['service']
            mid = cell_point(o, CELL_SLOTS[0][0] / 2, CELL_SLOTS[0][1] / 2)
            out.append(_posed_arm(f'fl_{sid}_arm', arm, (*mid, 1.25), yaw, station_geo))
            out.append(_posed_arm(f'fl_{sid}_ld', loader, loader_home(loader, slots[1]), yaw,
                                  loader_geo, LOADER_SCALE))
            for k in range(len(slots)):
                out.append(_link_model(f'fl_{sid}_box{k}', hidden, _carton_geometry(style)))
                out.append(_link_model(f'fl_{sid}_tray{k}', hidden, _tray_geometry(style)))
            out.append(_link_model(f'fl_{sid}_hbox', hidden, _carton_geometry(style)))
            out.append(_link_model(f'fl_{sid}_htray', hidden, _tray_geometry(style)))
            ends.append(f'<end side="{side}" station="{sid}">'
                        f'<service>{service[0]:.3f} {service[1]:.3f}</service>'
                        f'<yaw>{yaw:.4f}</yaw><arm>{v3(arm)}</arm><loader>{v3(loader)}</loader>'
                        + ''.join(f'<slot>{v3(p)}</slot>' for p in slots)
                        + f'<stack>{v3(stack)}</stack></end>')
        heights = []
        for pool, (n, look) in zip('ab', zip(L['pool'], L['looks'])):
            heights.append(ITEM_LOOK[look][0][2])
            for k in range(n):
                out.append(_link_model(f'fl_{L["id"]}_{pool}{k}', hidden, _item_geometry(look)))
        (x0, y0), (x1, y1) = L['belt']
        t = L['timing']
        cfg.append(f'<line id="{L["id"]}" cap="{L["cap"]}" transform="{L["transform"]}">'
                   f'<belt>{x0:.3f} {y0:.3f} {BELT_TOP:.3f} {x1:.3f} {y1:.3f} {BELT_TOP:.3f}</belt>'
                   f'<timing>{t[0]:.3f} {t[1]:.3f} {t[2]:.3f}</timing>'
                   f'<pool_a>{L["pool"][0]} {heights[0]:.3f}</pool_a>'
                   f'<pool_b>{L["pool"][1]} {heights[1]:.3f}</pool_b>{"".join(ends)}</line>')
    d = ARM_DIMS
    return (f'  <plugin filename="amr_factory_lines" name="amr_gazebo::FactoryLines">'
            f'<topic>{topic}</topic>'
            f'<a1>{d["a1"]}</a1><d1>{d["d1"]}</d1><l2>{d["l2"]}</l2><l3>{d["l3"]}</l3>'
            f'<tool>{d["tool"]}</tool><loader_scale>{LOADER_SCALE}</loader_scale>'
            f'<loader_home>{LOADER_HOME[0]} {LOADER_HOME[1]}</loader_home>'
            f'<safe_rho>{SAFE_RHO[0]} {SAFE_RHO[1]}</safe_rho><swing_min>{SWING_MIN}</swing_min><carton>{CARTON[0]} {CARTON[1]} {CARTON[2]}</carton>'
            f'<deck_top>{DECK_TOP}</deck_top>{"".join(cfg)}</plugin>\n')


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


def _seg_intersection(a, b, c, d) -> Optional[float]:
    """Parameter t on a->b where it crosses c->d (None if they do not cross)."""
    r = (b[0] - a[0], b[1] - a[1])
    q = (d[0] - c[0], d[1] - c[1])
    den = r[0] * q[1] - r[1] * q[0]
    if abs(den) < 1e-9:
        return None
    t = ((c[0] - a[0]) * q[1] - (c[1] - a[1]) * q[0]) / den
    u = ((c[0] - a[0]) * r[1] - (c[1] - a[1]) * r[0]) / den
    return t if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0 else None


def walkway_crossings(graph, a, b, half: float = 1.3) -> List[Tuple[float, float]]:
    """Spans (metres along a->b) where a walkway crosses AMR lanes, merged."""
    L = math.dist(a, b)
    hits = []
    for es in graph.out_edges:
        for e in es:
            t = _seg_intersection(a, b, (graph.nodes[e.u].x, graph.nodes[e.u].y),
                                  (graph.nodes[e.v].x, graph.nodes[e.v].y))
            if t is not None:
                hits.append(t * L)
    spans: List[Tuple[float, float]] = []
    for h in sorted(hits):
        lo, hi = max(0.0, h - half), min(L, h + half)
        if spans and lo <= spans[-1][1] + 0.3:
            spans[-1] = (spans[-1][0], max(spans[-1][1], hi))
        else:
            spans.append((lo, hi))
    return spans


def build_people(layout, graph, out: List[str]) -> str:
    """Painted walkways / zebra crossings along the workers' routes and the
    Pedestrians system block (returned) that animates the workers."""
    routes = people_routes(layout)
    if not routes:
        return ''
    m = Model('walkways')
    for route in routes:
        for a, b in people_segments(route):
            L = math.dist(a, b)
            if L < 0.2:
                continue
            ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            yaw = math.atan2(uy, ux)
            spans = walkway_crossings(graph, a, b)
            # zebra crossings over the lanes: white bars across the walking direction
            for lo, hi in spans:
                k = lo + 0.2
                while k < hi - 0.15:
                    m.box(a[0] + ux * k, a[1] + uy * k, 0.007, 0.3, 1.4, 0.002, 'white', yaw,
                          in_map=False)
                    k += 0.6
            # green border lines on both sides elsewhere
            free, last = [], 0.0
            for lo, hi in spans:
                if lo - last > 0.3:
                    free.append((last, lo))
                last = hi
            if L - last > 0.3:
                free.append((last, L))
            for lo, hi in free:
                cx, cy = a[0] + ux * (lo + hi) / 2, a[1] + uy * (lo + hi) / 2
                for side in (-0.7, 0.7):
                    m.box(cx - uy * side, cy + ux * side, 0.006, hi - lo, 0.08, 0.002,
                          'walkway', yaw, in_map=False)
    out.append(m.sdf())
    cfg = layout.get('people') or {}
    persons = []
    for r in routes:
        wps = ''.join(f'<waypoint dwell="{d:g}">{x:.2f} {y:.2f}</waypoint>'
                      for (x, y), d in zip(r['waypoints'], r['dwell']))
        vest = ' '.join(f'{c:g}' for c in r['vest'])
        helmet = ' '.join(f'{c:g}' for c in r['helmet'])
        persons.append(f'<person name="{escape(r["name"])}" speed="{r["speed"]:g}" '
                       f'loop="{str(r["loop"]).lower()}" vest="{vest}" helmet="{helmet}">'
                       f'{wps}</person>')
    return (f'  <plugin filename="amr_pedestrians" name="amr_gazebo::Pedestrians">'
            f'<topic>/factory/people</topic><robot_prefix>amr_</robot_prefix>'
            f'<stop_distance>{float(cfg.get("stop_distance", 2.0)):g}</stop_distance>'
            f'{"".join(persons)}</plugin>\n')


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
              line_topic: str = '/factory/line_state', mesh: Optional[Tuple[str, str]] = None,
              people: bool = True):
    """Return (sdf_text, map_slices).

    mesh=(obj_file, uri): bake the opaque static geometry into obj_file (written
    here) and reference it as uri; without it every primitive is an SDF visual."""
    global _BAKER
    _BAKER = MeshBaker() if mesh else None
    try:
        return _world_sdf(layout, step, rtf, animate, line_topic, mesh, people)
    finally:
        _BAKER = None


def _world_sdf(layout, step, rtf, animate, line_topic, mesh, people):
    graph = build_graph(layout)
    W, H = layout.get('factory', {}).get('size', [100.0, 60.0])
    slices = MapSlices()
    lights: List[str] = []
    body: List[str] = []
    build_ground(W, H, body)
    build_floor(layout, W, H, body)
    build_walls(layout, slices, body)
    build_roof(layout, body, lights)
    objects = list(layout.get('objects', []) or []) + line_objects(layout) + \
        station_objects(layout, graph)
    build_columns(layout, graph, objects, slices, body)
    build_markings(layout, graph, body)
    build_objects(layout, slices, body)
    build_lines(layout, slices, body)
    build_station_objects(layout, graph, slices, body)
    cells = build_line_cells(layout, graph, body, line_topic, mesh) if animate else ''
    build_charging_area(layout, graph, slices, body)
    build_wall_items(layout, slices, body)
    build_office(layout, body)
    build_trucks(layout, body)
    workers = build_people(layout, graph, body) if people else ''

    if mesh and _BAKER is not None and _BAKER.count:
        obj_file, uri = mesh
        _BAKER.write(obj_file)
        body.append(f'<model name="factory_static"><static>true</static><link name="link">'
                    f'<visual name="baked"><geometry><mesh><uri>{escape(uri)}</uri></mesh>'
                    f'</geometry><cast_shadows>false</cast_shadows></visual></link></model>\n')
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
{cells}{workers}  <gravity>0 0 -9.81</gravity>
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
                        step: float = 0.004, rtf: float = 1.0, animate: bool = True,
                        people: bool = True) -> Optional[str]:
    """Write the world SDF and (optionally) the matching map; return the map yaml."""
    import os
    obj = os.path.join(os.path.dirname(os.path.abspath(world_file)), 'factory_static.obj')
    sdf, slices = world_sdf(layout, step, rtf, animate=animate, mesh=(obj, 'file://' + obj),
                            people=people)
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
    ap.add_argument('--no-people', action='store_true', help='no walking workers')
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
                            mesh=mesh, people=not args.no_people)
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(sdf)
    print(f'wrote {args.out} ({len(sdf) / 1024:.0f} kB)')
    if args.map:
        img, origin, res = rasterize(slices, layout['factory']['size'])
        pgm, yml = write_map(args.map, img, origin, res)
        print(f'wrote {pgm} {yml} ({img.shape[1]}x{img.shape[0]})')


if __name__ == '__main__':
    main()
