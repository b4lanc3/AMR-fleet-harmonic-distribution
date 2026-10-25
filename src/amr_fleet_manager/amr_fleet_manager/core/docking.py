"""Charger docking geometry and lidar-based dock detection.

The route of an AMR going to charge ends on the charger's dock node of the
traffic graph: that is the *staging pose*.  Nav2's docking server
(opennav_docking) then drives the last DOCK_DEPTH metres forward until the
robot's bumper touches the contact plate of the charger.

Precise docking needs the dock pose relative to the robot, not the AMCL map
pose (10-30 cm error along an aisle).  The charger unit presents a flat face
FACE_WIDTH wide at the height of the 2D lidar; `detect_dock_face` fits that
face in the scan and returns the pose the robot base must reach, which the
agent publishes on `detected_dock_pose` for the docking server.

    staging (dock node) --DOCK_DEPTH--> docked base pose --FACE_TO_CENTER--> charger face
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

DOCK_DEPTH = 0.50        # m from the dock node (staging) to the docked base pose
FACE_TO_CENTER = 0.55    # m from the charger face to the robot base when docked
FACE_WIDTH = 0.80        # m, width of the charger unit
CONTACT_DEPTH = 0.13     # m, the contact plate sticks out of the face (below the lidar plane)


def docked_pose(node_xy: Tuple[float, float], heading: float) -> Tuple[float, float, float]:
    """Docked base pose (map) of a charger whose dock node is node_xy, facing heading."""
    return (node_xy[0] + DOCK_DEPTH * math.cos(heading),
            node_xy[1] + DOCK_DEPTH * math.sin(heading), heading)


def detect_dock_face(ranges: Sequence[float], angle_min: float, angle_increment: float,
                     lidar_x: float = 0.43, max_dist: float = 2.5,
                     face_width: float = FACE_WIDTH,
                     face_to_center: float = FACE_TO_CENTER) -> Optional[Tuple[float, float, float]]:
    """Docked base pose (x, y, yaw) in the robot base frame from a 2D scan, or None.

    The charger face is the nearest flat segment straight ahead: points of the
    central corridor give its distance, a thin band around it gives the
    segment, a least-squares line x = a + b*y its orientation, the extreme
    points its centre.  The face must be about face_width wide."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + np.arange(r.size) * angle_increment
    ok = np.isfinite(r) & (r > 0.05) & (r < max_dist)
    x = lidar_x + r[ok] * np.cos(ang[ok])
    y = r[ok] * np.sin(ang[ok])
    front = (x > lidar_x - 0.05) & (np.abs(y) < face_width)
    x, y = x[front], y[front]
    central = np.abs(y) < 0.2
    if central.sum() < 5:
        return None
    x0 = float(np.min(x[central]))
    band = (x < x0 + 0.06 + 0.25 * np.abs(y)) & (x > x0 - 0.06 - 0.25 * np.abs(y))
    xs, ys = x[band], y[band]
    if xs.size < 15:
        return None
    b, a = np.polyfit(ys, xs, 1)               # x = a + b * y
    resid = xs - (a + b * ys)
    keep = np.abs(resid) < 0.03
    xs, ys = xs[keep], ys[keep]
    if xs.size < 15:
        return None
    b, a = np.polyfit(ys, xs, 1)
    y_lo, y_hi = float(np.min(ys)), float(np.max(ys))
    width = (y_hi - y_lo) * math.sqrt(1.0 + b * b)
    if abs(width - face_width) > 0.15:
        return None
    yc = 0.5 * (y_lo + y_hi)
    xc = a + b * yc
    norm = math.hypot(1.0, b)
    fx, fy = 1.0 / norm, -b / norm             # from the robot into the face
    return (xc - fx * face_to_center, yc - fy * face_to_center, math.atan2(fy, fx))
