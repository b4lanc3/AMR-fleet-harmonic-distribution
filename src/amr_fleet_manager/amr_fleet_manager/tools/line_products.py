"""Products of the lines and their look at every assembly / packing stage.

The production lines assemble a small electric motor on jigs that ride a
closed-loop (over-under) conveyor; the packaging lines pack it.  Every stage
is one look (one model) whose parts add to the previous stage:

  production:  m0 base with feet and cradle  (kit carton -> jig)
               m1 + stator housing with cooling fins
               m2 + front end shield, shaft and key
               m3 + fan cover, terminal box, cable gland
               m4 + name plate and QC sticker  (-> carton of finished motors)
  packaging:   m4 finished motor               (carton -> jig)
               p1 + foam end caps
               p2 in an open product box
               p3 box closed and taped
               p4 + shipping label with barcode (-> shipping carton)

Every look is baked into ONE OBJ with ONE textured material (a small colour
palette image), so a product is a single draw call whatever its detail.
Sizes fit a cell of the 4 x 4 carton grid (0.13 x 0.09 m); origin in the
centre of the bounding box, length along x.
"""

from __future__ import annotations

import math
import os
import struct
import zlib
from typing import Dict, List, Sequence, Tuple

import numpy as np

H = 0.10                          # height of every look (bounding box)
JIG = (0.20, 0.14, 0.03)          # jig (pallet carrier) on the loop conveyor
LIFT = 0.30                       # length of the lift carriage (belt section at each end)
Z0 = -H / 2

C = {
    'steel': (0.38, 0.40, 0.43), 'steel_light': (0.70, 0.72, 0.74), 'dark': (0.16, 0.17, 0.19),
    'black': (0.05, 0.05, 0.05), 'motor': (0.13, 0.36, 0.68), 'white': (0.95, 0.95, 0.93),
    'foam': (0.97, 0.97, 0.95), 'box': (0.93, 0.92, 0.88), 'band': (0.12, 0.38, 0.75),
    'logo': (0.95, 0.50, 0.10), 'tape': (0.80, 0.68, 0.42), 'label': (1.00, 0.99, 0.88),
    'green': (0.10, 0.75, 0.25), 'yellow': (0.98, 0.78, 0.05), 'brass': (0.78, 0.62, 0.25),
    'jig': (0.30, 0.45, 0.55), 'rubber': (0.10, 0.10, 0.10),
}


def _box(c, s, col, roll=0.0):
    return ('box', c, s, col, roll)


def _cyl(c, r, h, axis, col):
    return ('cyl', c, r, h, axis, col)


def motor_parts(stage: int) -> List[tuple]:
    """Parts of the motor after assembly stage 0..4."""
    p = [_box((0, 0, Z0 + 0.006), (0.110, 0.076, 0.012), 'steel')]             # base plate
    for sx in (-0.04, 0.04):
        for sy in (-0.03, 0.03):
            p.append(_box((sx, sy, Z0 + 0.0125), (0.018, 0.010, 0.002), 'black'))  # foot slots
    for sx in (-0.026, 0.026):
        p.append(_box((sx, 0, Z0 + 0.017), (0.012, 0.052, 0.010), 'steel_light'))  # cradles
    zc = Z0 + 0.016 + 0.032                                                      # stator axis
    if stage >= 1:
        p.append(_cyl((0, 0, zc), 0.032, 0.075, 'x', 'motor'))
        for k in range(8):                                                       # cooling fins
            a = math.radians(22.5 + 45 * k)
            if math.sin(a) < -0.5:
                continue
            p.append(_box((0, 0.034 * math.cos(a), zc + 0.034 * math.sin(a)),
                          (0.068, 0.003, 0.006), 'motor', roll=a))
    if stage >= 2:
        p.append(_cyl((0.041, 0, zc), 0.030, 0.007, 'x', 'dark'))               # end shield
        p.append(_cyl((0.0445, 0, zc), 0.012, 0.004, 'x', 'steel'))             # bearing cap
        p.append(_cyl((0.058, 0, zc), 0.006, 0.024, 'x', 'steel_light'))        # shaft
        p.append(_box((0.060, 0, zc + 0.0062), (0.014, 0.003, 0.002), 'brass'))  # key
    if stage >= 3:
        p.append(_cyl((-0.046, 0, zc), 0.031, 0.016, 'x', 'black'))             # fan cover
        for r in (0.012, 0.022):
            p.append(_cyl((-0.0545, 0, zc), r, 0.002, 'x', 'steel'))            # grille rings
        p.append(_box((0.006, 0, zc + 0.039), (0.032, 0.028, 0.016), 'dark'))  # terminal box
        p.append(_box((0.006, 0, zc + 0.048), (0.034, 0.030, 0.003), 'steel'))  # its lid
        p.append(_cyl((0.006, 0.018, zc + 0.038), 0.004, 0.010, 'y', 'black'))  # cable gland
    if stage >= 4:
        p.append(_box((0, -0.0335, zc - 0.004), (0.030, 0.002, 0.016), 'white'))  # name plate
        p.append(_box((0, -0.0347, zc - 0.004), (0.012, 0.001, 0.004), 'dark'))
        p.append(_box((0.016, 0, zc + 0.0505), (0.008, 0.008, 0.001), 'green'))  # QC sticker
    return p


def package_parts(stage: int) -> List[tuple]:
    """Parts after packing stage 1..4 (stage 0 is the finished motor)."""
    if stage == 0:
        return motor_parts(4)
    L, W, Hb, t = 0.125, 0.088, 0.098, 0.003
    if stage == 1:
        p = motor_parts(4)
        for x in (-0.050, 0.055):                                               # foam end caps
            p.append(_box((x, 0, Z0 + 0.047), (0.024, 0.084, 0.092), 'foam'))
        return p
    if stage == 2:                                                               # open product box
        p = [q for q in package_parts(1)]
        p.append(_box((0, 0, Z0 + t / 2), (L, W, t), 'box'))
        wall = 0.085
        for sy in (-1, 1):
            p.append(_box((0, sy * (W / 2 - t / 2), Z0 + wall / 2), (L, t, wall), 'box'))
            p.append(_box((0, sy * (W / 2 + 0.001), Z0 + 0.035), (L, 0.001, 0.018), 'band'))
            p.append(_box((0, sy * (W / 2 + 0.012), Z0 + wall + 0.010), (L, 0.024, t), 'box', roll=sy * 1.0))
        for sx in (-1, 1):
            p.append(_box((sx * (L / 2 - t / 2), 0, Z0 + wall / 2), (t, W, wall), 'box'))
        return p
    p = [_box((0, 0, Z0 + Hb / 2), (L, W, Hb), 'box')]                          # closed box
    for sy in (-1, 1):
        p.append(_box((0, sy * (W / 2 + 0.0005), Z0 + 0.035), (L, 0.001, 0.018), 'band'))
        p.append(_box((-0.03, sy * (W / 2 + 0.0008), Z0 + 0.068), (0.022, 0.001, 0.022), 'logo'))
    p.append(_box((0, 0, Z0 + Hb + 0.0008), (L + 0.002, 0.022, 0.0016), 'tape'))       # top tape
    for sx in (-1, 1):
        p.append(_box((sx * (L / 2 + 0.0008), 0, Z0 + Hb - 0.018), (0.0016, 0.022, 0.036), 'tape'))
    if stage >= 4:                                                               # shipping label
        p.append(_box((0.028, 0, Z0 + Hb + 0.0012), (0.045, 0.060, 0.0012), 'label'))
        for k in range(7):
            p.append(_box((0.015 + k * 0.0045, -0.012, Z0 + Hb + 0.0021), (0.0018, 0.022, 0.0006), 'black'))
    return p


def jig_parts() -> List[tuple]:
    sx, sy, sz = JIG
    p = [_box((0, 0, 0), (sx, sy, sz), 'jig')]
    for x in (-0.07, 0.07):
        p.append(_cyl((x, 0.05, sz / 2 + 0.008), 0.006, 0.016, 'z', 'yellow'))    # locating pins
    for x in (-0.085, 0.085):
        p.append(_box((x, 0, sz / 2 + 0.002), (0.012, 0.10, 0.004), 'rubber'))
    return p


def lift_parts() -> List[tuple]:
    """Carriage of an end lift: a short twin-strand belt section with its guide
    rails that carries one jig between the two levels; x along the belt,
    origin on top of the strands."""
    L = LIFT
    p = []
    for y in (-0.07, 0.07):
        p.append(_box((0, y, -0.01), (L, 0.05, 0.02), 'black'))              # belt strands
        p.append(_cyl((L / 2 - 0.02, y, -0.012), 0.016, 0.06, 'y', 'steel'))  # end rollers
        p.append(_cyl((-L / 2 + 0.02, y, -0.012), 0.016, 0.06, 'y', 'steel'))
    for y in (-0.12, 0.12):
        p.append(_box((0, y, -0.035), (L, 0.025, 0.07), 'steel_light'))      # side profiles
        p.append(_box((0, y * 0.708, 0.035), (L - 0.04, 0.012, 0.02), 'steel'))   # jig guide rails
        p.append(_box((0, y * 1.1, -0.07), (0.12, 0.012, 0.12), 'yellow'))   # slider plates
    p.append(_box((0, 0, -0.078), (L, 0.22, 0.012), 'dark'))                 # base plate
    p.append(_box((0, 0.11, -0.05), (0.06, 0.03, 0.04), 'dark'))             # strand motor
    p.append(_box((L / 2 - 0.01, 0.10, 0.03), (0.02, 0.02, 0.03), 'yellow'))   # jig stop
    return p


STAGES = {
    'production': [('m0', motor_parts(0)), ('m1', motor_parts(1)), ('m2', motor_parts(2)),
                   ('m3', motor_parts(3)), ('m4', motor_parts(4))],
    'packaging': [('m4', motor_parts(4)), ('p1', package_parts(1)), ('p2', package_parts(2)),
                  ('p3', package_parts(3)), ('p4', package_parts(4))],
}


# ------------------------------------------------------------------ baking
_BOX_V = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
_BOX_F = [((0, 1, 3, 2), (-1, 0, 0)), ((4, 6, 7, 5), (1, 0, 0)), ((0, 4, 5, 1), (0, -1, 0)),
          ((2, 3, 7, 6), (0, 1, 0)), ((0, 2, 6, 4), (0, 0, -1)), ((1, 5, 7, 3), (0, 0, 1))]
_AXIS = {'z': np.eye(3),
         'x': np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float),     # z -> x
         'y': np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)}     # z -> y


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _triangles(part):
    """(vertices, normals, faces[(vi, ni) x3]) of one part."""
    if part[0] == 'box':
        _, c, s, _col, roll = part
        R = _rx(roll)
        v = (_BOX_V * s) @ R.T + c
        n = np.array([nn for _, nn in _BOX_F], dtype=float) @ R.T
        f = []
        for k, (q, _) in enumerate(_BOX_F):
            f += [((q[0], k), (q[1], k), (q[2], k)), ((q[0], k), (q[2], k), (q[3], k))]
        return v, n, f
    _, c, r, h, axis, _col = part
    m = 8 if r < 0.01 else 16
    a = np.arange(m) * 2 * math.pi / m
    ring = np.stack([r * np.cos(a), r * np.sin(a), np.zeros(m)], axis=1)
    v = np.concatenate([ring + (0, 0, -h / 2), ring + (0, 0, h / 2), [[0, 0, -h / 2], [0, 0, h / 2]]])
    n = np.concatenate([np.stack([np.cos(a), np.sin(a), np.zeros(m)], axis=1), [[0, 0, -1], [0, 0, 1]]])
    R = _AXIS[axis]
    v = v @ R.T + c
    n = n @ R.T
    f = []
    for i in range(m):
        j = (i + 1) % m
        f += [((i, i), (j, j), (m + j, j)), ((i, i), (m + j, j), (m + i, i)),
              ((2 * m, m), (j, m), (i, m)), ((2 * m + 1, m + 1), (m + i, m + 1), (m + j, m + 1))]
    return v, n, f


PALETTE = list(C)                 # colour order in the palette image
CELL = 8                          # pixels per colour


def write_palette(path: str) -> None:
    """Palette image: one CELL x CELL square per colour (RGB PNG, no extra deps)."""
    w, h = CELL * len(PALETTE), CELL
    rows = []
    for _ in range(h):
        row = bytearray([0])
        for name in PALETTE:
            r, g, b = (int(round(x * 255)) for x in C[name])
            row += bytes((r, g, b)) * CELL
        rows.append(bytes(row))
    raw = zlib.compress(b''.join(rows), 9)

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', raw) + chunk(b'IEND', b''))


def write_obj(path: str, parts: Sequence[tuple], palette_png: str) -> None:
    """One OBJ + MTL, one material textured with the palette."""
    base = os.path.splitext(path)[0]
    mtl = base + '.mtl'
    name = os.path.basename(base)
    with open(mtl, 'w', encoding='utf-8') as f:
        f.write(f'newmtl {name}\nKa 1 1 1\nKd 1 1 1\nKs 0.1 0.1 0.1\nNs 20\nd 1.0\nillum 2\n'
                f'map_Kd {os.path.basename(palette_png)}\n')
    vs, vns, out = [], [], []
    vts = [((PALETTE.index(c) + 0.5) / len(PALETTE), 0.5) for c in PALETTE]
    for part in parts:
        v, n, faces = _triangles(part)
        col = part[3] if part[0] == 'box' else part[5]
        t = PALETTE.index(col) + 1
        v0, n0 = len(vs), len(vns)
        vs += [tuple(x) for x in v]
        vns += [tuple(x) for x in n]
        for fc in faces:
            out.append(' '.join(f'{v0 + a + 1}/{t}/{n0 + b + 1}' for a, b in fc))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'# {name}: {len(parts)} parts, one material\nmtllib {os.path.basename(mtl)}\n')
        f.write(''.join(f'v {x:.5f} {y:.5f} {z:.5f}\n' for x, y, z in vs))
        f.write(''.join(f'vt {u:.5f} {w:.5f}\n' for u, w in vts))
        f.write(''.join(f'vn {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in vns))
        f.write(f'o {name}\nusemtl {name}\n')
        f.write(''.join(f'f {fc}\n' for fc in out))


def bounding_box(parts: Sequence[tuple]) -> Tuple[float, float, float]:
    pts = np.concatenate([_triangles(p)[0] for p in parts])
    return tuple(float(x) for x in pts.max(axis=0) - pts.min(axis=0))


def bake_all(directory: str) -> Dict[str, str]:
    """Write every look and the jig next to the factory mesh; returns file names."""
    os.makedirs(directory, exist_ok=True)
    png = os.path.join(directory, 'item_palette.png')
    write_palette(png)
    files = {}
    looks = {}
    for kind in STAGES.values():
        for name, parts in kind:
            looks[name] = parts
    looks['jig'] = jig_parts()
    looks['lift'] = lift_parts()
    for name, parts in looks.items():
        fn = f'item_{name}.obj'
        write_obj(os.path.join(directory, fn), parts, png)
        files[name] = fn
    return files
