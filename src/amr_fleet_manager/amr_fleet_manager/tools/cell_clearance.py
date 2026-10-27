"""Clearance between the two arms of a line work cell.

Replays the motions of the FactoryLines system (amr_gazebo) in the frame of a
work cell: every job of the station arm (AMR deck <-> slot k, slot k <-> stack
of cartons) against every pick of the loader arm (product cell of the open
carton on slot j <-> belt), j != k - the loader only works on the open carton
and the station arm never touches it.  Links are capsules, the carton held by
the station arm a box.  Returns the smallest gap [m] (< 0: the arms overlap).

    python3 -m amr_fleet_manager.tools.cell_clearance
"""

from __future__ import annotations

import itertools
import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from ..core.layout import (BELT_TOP, CELL_ARM, CELL_BELT_V, CELL_LOADER,
                           CELL_SLOTS, CELL_STACK, PEDESTAL_TOP)
from .generate_world import (ARM_PEDESTAL, CARTON, DECK_TOP, ITEM_LOOK, LOADER_SCALE, SAFE_RHO,
                             STACK_TOP, SWING_MIN, arm_dims, arm_frames, loader_home)

ABOVE = 0.30            # FactoryLines kAbove
KEYS = (0.0, 0.15, 0.30, 0.42, 0.70, 0.85, 1.0)    # FactoryLines Loader() keyframes
Tip = Tuple[float, float, float]      # az, rho, z around the turret axis


def _tip(base, p) -> Tip:
    dx, dy = p[0] - base[0], p[1] - base[1]
    return (math.atan2(dy, dx), math.hypot(dx, dy), p[2])


def _pos(base, t: Tip):
    return (base[0] + t[1] * math.cos(t[0]), base[1] + t[1] * math.sin(t[0]), t[2])


def _lerp(a: Tip, b: Tip, k: float) -> Tip:
    daz = math.atan2(math.sin(b[0] - a[0]), math.cos(b[0] - a[0]))
    return (a[0] + k * daz, a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]))


def _smooth(t: float) -> float:
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def move(a: Tip, b: Tip, k: float, safe: float) -> Tip:
    """Tool tip at k (0..1, time) of a move: like FactoryLines Move() - a big
    turn of the turret pulls the tool in to `safe` first, turns, reaches out."""
    daz = math.atan2(math.sin(b[0] - a[0]), math.cos(b[0] - a[0]))
    if abs(daz) < SWING_MIN or (a[1] <= safe and b[1] <= safe):
        return _lerp(a, b, _smooth(k))
    ra, rb = min(a[1], safe), min(b[1], safe)
    l1, l3 = a[1] - ra, b[1] - rb
    l2 = abs(daz) * 0.5 * (ra + rb) + abs(rb - ra)
    tot = l1 + l2 + l3
    k1, k2 = l1 / tot, (l1 + l2) / tot
    z = a[2] + _smooth(k) * (b[2] - a[2])
    if k < k1:
        t = _smooth(k / k1)
        return (a[0], a[1] + t * (ra - a[1]), z)
    if k < k2:
        t = _smooth((k - k1) / (k2 - k1))
        return (a[0] + t * daz, ra + t * (rb - ra), z)
    t = _smooth((k - k2) / (1.0 - k2)) if k2 < 1.0 else 1.0
    return (b[0], rb + t * (b[1] - rb), z)


def _path(base, points: Sequence, safe: float, n: int = 10) -> List[Tuple[float, float, float]]:
    """Tool tip positions along the waypoints."""
    out = []
    tips = [_tip(base, p) for p in points]
    for a, b in zip(tips, tips[1:]):
        for i in range(n):
            out.append(_pos(base, move(a, b, i / (n - 1), safe)))
    return out


def _capsules(base, tip, k: float) -> List[Tuple[np.ndarray, np.ndarray, float]]:
    d = arm_dims(k)
    f = arm_frames(base, tip, 0.0, k)
    b, s, e, w = (np.array(f[n][0], dtype=float) for n in ('turret', 'upper', 'fore', 'tool'))
    fore = (w - e) / (np.linalg.norm(w - e) or 1.0)
    return [(b + (0, 0, 0.1 * k), s, 0.18 * k),             # turret / shoulder
            (s, e, 0.125 * k),                              # upper arm
            (e - fore * 0.36 * k, w, 0.11 * k),             # forearm with its back end
            (w, w - (0, 0, d['tool']), 0.075 * k)]          # wrist and flange


def _seg_dist(p0, p1, q0, q1) -> float:
    u, v, w0 = p1 - p0, q1 - q0, p0 - q0
    a, b, c, d, e = u @ u, u @ v, v @ v, u @ w0, v @ w0
    den = a * c - b * b
    s = 0.0 if den < 1e-12 else min(1.0, max(0.0, (b * e - c * d) / den))
    t = 0.0 if c < 1e-12 else (b * s + e) / c
    t = min(1.0, max(0.0, t))
    s = 0.0 if a < 1e-12 else min(1.0, max(0.0, (b * t - d) / a))
    return float(np.linalg.norm(w0 + u * s - v * t))


def _box_points(center, yaw: float, size) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for fx, fy, fz in itertools.product((-0.5, 0.0, 0.5), repeat=3):
        if (fx, fy, fz) == (0.0, 0.0, 0.0):
            continue
        x, y, z = fx * size[0], fy * size[1], fz * size[2]
        pts.append((center[0] + c * x - s * y, center[1] + s * x + c * y, center[2] + z))
    return np.array(pts)


LINKS = ('turret', 'upper arm', 'forearm', 'wrist')


def _gap(station, loader, carton) -> Tuple[float, str]:
    best, what = math.inf, ''
    for a, (p0, p1, r0) in enumerate(station):
        for b, (q0, q1, r1) in enumerate(loader):
            g = _seg_dist(p0, p1, q0, q1) - r0 - r1
            if g < best:
                best, what = g, f'{LINKS[a]} / loader {LINKS[b]}'
    if carton is not None:
        for b, (q0, q1, r1) in enumerate(loader):
            for p in carton:
                g = _seg_dist(p, p, q0, q1) - r1
                if g < best:
                    best, what = g, f'carton / loader {LINKS[b]}'
    return best, what


def min_clearance(samples: int = 8) -> Tuple[float, str]:
    """Smallest gap between the two arms of a work cell and where it occurs."""
    # cell frame: S = origin, u = x, v = y (the mirror image is symmetric)
    arm = (CELL_ARM[0], CELL_ARM[1], ARM_PEDESTAL)
    ld = (CELL_LOADER[0], CELL_LOADER[1], ARM_PEDESTAL)
    slots = [(u, v, PEDESTAL_TOP) for u, v in CELL_SLOTS]
    h = CARTON[2]
    up = np.array((0.0, 0.0, ABOVE))
    deck = np.array((0.0, 0.0, DECK_TOP))
    stack = np.array((CELL_STACK[0], CELL_STACK[1], STACK_TOP + h))
    belt = np.array((0.0, CELL_BELT_V, BELT_TOP))
    home = np.array(((0.0 + slots[0][0]) / 2, (0.0 + slots[0][1]) / 2, 1.25))
    lhome = np.array(loader_home(ld, slots[1]))
    item = ITEM_LOOK['part'][0]
    worst, where = math.inf, ''
    for k, slot in enumerate(slots):
        top = np.array(slot) + (0, 0, h)
        jobs = {
            'serve': ([home, deck + up, deck, deck + up, top + up, top, top + up, home],
                      (2, 6)),                                  # holds the carton
            'tray': ([home, top + up, top, top + up, stack + up, stack, stack + up, home],
                     (2, 6)),
        }
        for job, (pts, (g0, g1)) in jobs.items():
            path = _path(arm, pts, SAFE_RHO[0], samples)
            st = []
            for i, p in enumerate(path):
                held = g0 * samples <= i < g1 * samples
                carton = _box_points((p[0], p[1], p[2] - h / 2), 0.0, CARTON) if held else None
                st.append((_capsules(arm, p, 1.0), carton))
            for j, other in enumerate(slots):
                if j == k:
                    continue
                for cu, cv in ((-0.21, -0.13), (0.21, -0.13), (-0.21, 0.13), (0.21, 0.13), (0, 0)):
                    cell = np.array((other[0] + cu, other[1] + cv, other[2] + 0.015 + item[2]))
                    pts_l = [lhome, cell + up, cell, cell + up, belt + up + (0, 0, item[2]),
                             belt + (0, 0, item[2]), lhome]
                    for p in _path(ld, pts_l, SAFE_RHO[1], samples):
                        lc = _capsules(ld, p, LOADER_SCALE)
                        for caps, carton in st:
                            g, what = _gap(caps, lc, carton)
                            if g < worst:
                                worst = g
                                where = (f'station {job} slot {k} vs loader slot {j}: {what}, '
                                         f'loader tip {np.round(p, 2)}')
    return worst, where


def reach_error() -> float:
    """Largest distance [m] between a pick / place point of a work cell and
    where the arm can bring its tool (> 0: out of reach)."""
    arm = (CELL_ARM[0], CELL_ARM[1], ARM_PEDESTAL)
    ld = (CELL_LOADER[0], CELL_LOADER[1], ARM_PEDESTAL)
    h = CARTON[2]
    item = ITEM_LOOK['part'][0]
    worst = 0.0
    targets = [(arm, 1.0, (0.0, 0.0, DECK_TOP)), (arm, 1.0, (CELL_STACK[0], CELL_STACK[1], STACK_TOP + h))]
    for u, v in CELL_SLOTS:
        targets.append((arm, 1.0, (u, v, PEDESTAL_TOP + h + ABOVE)))
        for cu, cv in itertools.product((-0.2, 0.2), (-0.14, 0.14)):
            targets.append((ld, LOADER_SCALE, (u + cu, v + cv, PEDESTAL_TOP + 0.015 + item[2])))
    targets.append((ld, LOADER_SCALE, (0.0, CELL_BELT_V, BELT_TOP + item[2])))
    for base, k, p in targets:
        w = arm_frames(base, p, 0.0, k)['tool'][0]
        want = np.array(p) + (0, 0, arm_dims(k)['tool'])
        worst = max(worst, float(np.linalg.norm(np.array(w) - want)))
    return worst


def main() -> None:
    gap, where = min_clearance()
    print(f'smallest gap between the arms: {gap:.3f} m ({where})')
    print(f'largest reach error: {reach_error():.4f} m')


if __name__ == '__main__':
    main()
