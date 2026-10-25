import math

import numpy as np

from amr_fleet_manager.core.docking import (DOCK_DEPTH, FACE_TO_CENTER, FACE_WIDTH,
                                            detect_dock_face, docked_pose)
from amr_fleet_manager.core.layout import all_objects

LIDAR_X = 0.43


def scan_of_face(dist, lateral, tilt, back_panel=True, noise=0.0, seed=1):
    """270 deg scan (0.5 deg) of a charger face centred `dist` ahead of the base,
    shifted `lateral`, rotated by `tilt`; plus the station's back panel behind it."""
    rng = np.random.default_rng(seed)
    ang = np.radians(np.arange(-135.0, 135.0001, 0.5))
    ranges = np.full(ang.size, np.inf)
    c = np.array([dist, lateral])
    t = np.array([-math.sin(tilt), math.cos(tilt)])         # along the face
    segs = [(c - t * FACE_WIDTH / 2, c + t * FACE_WIDTH / 2)]
    if back_panel:
        n = np.array([math.cos(tilt), math.sin(tilt)])
        segs.append((c + n * 0.75 - t * 3.0, c + n * 0.75 + t * 3.0))
    o = np.array([LIDAR_X, 0.0])
    for i, a in enumerate(ang):
        d = np.array([math.cos(a), math.sin(a)])
        for p, q in segs:
            e = q - p
            den = d[0] * -e[1] - d[1] * -e[0]
            if abs(den) < 1e-12:
                continue
            w = p - o
            s = (w[0] * -e[1] - w[1] * -e[0]) / den
            u = (d[0] * w[1] - d[1] * w[0]) / den
            if s > 0 and 0 <= u <= 1:
                ranges[i] = min(ranges[i], s + noise * rng.standard_normal())
    return ranges, math.radians(-135.0), math.radians(0.5)


def test_detects_the_dock_pose_from_the_face():
    tilt, lateral = math.radians(4.0), 0.06
    ranges, amin, inc = scan_of_face(FACE_TO_CENTER + DOCK_DEPTH, lateral, tilt, noise=0.005)
    x, y, yaw = detect_dock_face(ranges, amin, inc, lidar_x=LIDAR_X)
    # docked base pose: FACE_TO_CENTER in front of the face, facing it
    exp_x = FACE_TO_CENTER + DOCK_DEPTH - FACE_TO_CENTER * math.cos(tilt)
    exp_y = lateral - FACE_TO_CENTER * math.sin(tilt)
    assert abs(x - exp_x) < 0.02 and abs(y - exp_y) < 0.02
    assert abs(yaw - tilt) < math.radians(1.0)


def test_detection_works_until_contact_and_rejects_other_shapes():
    # nearly docked: the face is 0.12 m in front of the lidar
    ranges, amin, inc = scan_of_face(FACE_TO_CENTER, 0.0, 0.0)
    x, y, yaw = detect_dock_face(ranges, amin, inc, lidar_x=LIDAR_X)
    assert abs(x) < 0.02 and abs(y) < 0.02 and abs(yaw) < 0.02
    # a plain wall (no 0.8 m wide face) is not a dock
    ang = np.radians(np.arange(-135.0, 135.0001, 0.5))
    wall = np.where(np.cos(ang) > 0.05, 1.2 / np.maximum(np.cos(ang), 1e-3), np.inf)
    assert detect_dock_face(wall, ang[0], math.radians(0.5), lidar_x=LIDAR_X) is None


def test_chargers_match_the_docking_geometry(layout, graph):
    """Charger unit face at DOCK_DEPTH + FACE_TO_CENTER in front of every dock node."""
    units = [o['rect'] for o in all_objects(layout, graph) if o.get('kind') == 'charger']
    for cid, i in graph.chargers.items():
        n = graph.nodes[i]
        dx, dy, h = docked_pose((n.x, n.y), n.heading)
        fx, fy = dx + FACE_TO_CENTER * math.cos(h), dy + FACE_TO_CENTER * math.sin(h)
        hit = [r for r in units if r[0] - 0.02 <= fx <= r[2] + 0.02 and r[1] - 0.02 <= fy <= r[3] + 0.02]
        assert hit, f'{cid}: no charger face at ({fx:.2f}, {fy:.2f})'
        assert abs(hit[0][2] - hit[0][0] - FACE_WIDTH) < 0.05
