"""Light-weight kinematic simulation of the AMR fleet.

Used by the `kinematic_sim` ROS node (run the whole stack without Gazebo on a
weak PC) and by the headless test harness.  Each robot is a differential
drive with acceleration limits, a battery model and a simple 2D lidar that
sees the walls, the static objects of the layout and the other robots.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence

import numpy as np

ROBOT_LENGTH = 0.80
ROBOT_WIDTH = 0.56
LIDAR_X = 0.43          # lidar mounted at the front centre of the chassis


@dataclass
class BatteryModel:
    idle_drain: float = 0.004      # %/s
    move_drain: float = 0.030      # %/s per m/s
    charge_rate: float = 0.25      # %/s on a dock


@dataclass
class SimRobot:
    id: str
    x: float
    y: float
    yaw: float
    v: float = 0.0
    w: float = 0.0
    cmd_v: float = 0.0
    cmd_w: float = 0.0
    battery: float = 100.0
    charging: bool = False
    wheel_left: float = 0.0
    wheel_right: float = 0.0

    def footprint(self, margin: float = 0.0) -> np.ndarray:
        hl, hw = ROBOT_LENGTH / 2 + margin, ROBOT_WIDTH / 2 + margin
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        pts = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        rot = np.array([[c, -s], [s, c]])
        return pts @ rot.T + np.array([self.x, self.y])


class KinematicWorld:
    WHEEL_RADIUS = 0.08
    WHEEL_SEPARATION = 0.52

    def __init__(self, layout: dict, battery: Optional[BatteryModel] = None,
                 max_accel: float = 1.0, max_alpha: float = 3.0,
                 objects: Optional[Sequence[dict]] = None) -> None:
        self.layout = layout
        self.battery = battery or BatteryModel()
        self.max_accel = max_accel
        self.max_alpha = max_alpha
        self.robots: Dict[str, SimRobot] = {}
        W, H = layout.get('factory', {}).get('size', [100.0, 60.0])
        t = layout.get('factory', {}).get('wall_thickness', 0.3) / 2.0
        rects = [[-t, -t, W + t, t], [-t, H - t, W + t, H + t],
                 [-t, -t, t, H + t], [W - t, -t, W + t, H + t]]
        for o in (objects if objects is not None else layout.get('objects', []) or []):
            rects.append(list(o['rect']))
        self.rects = np.array(rects, dtype=float)

    def add_robot(self, rid: str, x: float, y: float, yaw: float, battery: float) -> SimRobot:
        r = SimRobot(rid, x, y, yaw, battery=battery)
        self.robots[rid] = r
        return r

    # ---------------------------------------------------------- dynamics
    def step(self, dt: float, charge_enable: Dict[str, bool]) -> None:
        bm = self.battery
        for r in self.robots.values():
            dv = max(-self.max_accel * 1.6 * dt, min(self.max_accel * dt, r.cmd_v - r.v))
            dw = max(-self.max_alpha * dt, min(self.max_alpha * dt, r.cmd_w - r.w))
            r.v += dv
            r.w += dw
            r.yaw = (r.yaw + r.w * dt + math.pi) % (2 * math.pi) - math.pi
            r.x += r.v * math.cos(r.yaw) * dt
            r.y += r.v * math.sin(r.yaw) * dt
            vl = (r.v - r.w * self.WHEEL_SEPARATION / 2) / self.WHEEL_RADIUS
            vr = (r.v + r.w * self.WHEEL_SEPARATION / 2) / self.WHEEL_RADIUS
            r.wheel_left += vl * dt
            r.wheel_right += vr * dt
            r.charging = bool(charge_enable.get(r.id)) and abs(r.v) < 0.02
            if r.charging:
                r.battery = min(100.0, r.battery + bm.charge_rate * dt)
            else:
                drain = bm.idle_drain + bm.move_drain * abs(r.v) + 0.01 * abs(r.w)
                r.battery = max(0.0, r.battery - drain * dt)

    # ------------------------------------------------------------- lidar
    def scan(self, rid: str, angle_min: float = -0.75 * math.pi, angle_max: float = 0.75 * math.pi,
             n: int = 271, range_max: float = 12.0) -> np.ndarray:
        """Ray cast from the lidar of robot rid against boxes and robots."""
        r = self.robots[rid]
        ox = r.x + LIDAR_X * math.cos(r.yaw)
        oy = r.y + LIDAR_X * math.sin(r.yaw)
        ang = r.yaw + np.linspace(angle_min, angle_max, n)
        dx, dy = np.cos(ang), np.sin(ang)
        ranges = np.full(n, range_max)
        # static boxes (slab method), only those near the robot
        rc = self.rects
        near = ((rc[:, 0] - range_max < ox) & (rc[:, 2] + range_max > ox)
                & (rc[:, 1] - range_max < oy) & (rc[:, 3] + range_max > oy))
        boxes = rc[near]
        if len(boxes):
            with np.errstate(divide='ignore', invalid='ignore'):
                inv_dx = 1.0 / np.where(np.abs(dx) < 1e-9, 1e-9, dx)
                inv_dy = 1.0 / np.where(np.abs(dy) < 1e-9, 1e-9, dy)
            tx1 = (boxes[:, 0][:, None] - ox) * inv_dx[None, :]
            tx2 = (boxes[:, 2][:, None] - ox) * inv_dx[None, :]
            ty1 = (boxes[:, 1][:, None] - oy) * inv_dy[None, :]
            ty2 = (boxes[:, 3][:, None] - oy) * inv_dy[None, :]
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = (tmax >= np.maximum(tmin, 0.0))
            t = np.where(tmin > 0, tmin, tmax)
            t = np.where(hit & (t > 0), t, np.inf)
            ranges = np.minimum(ranges, t.min(axis=0))
        # other robots as circles of radius 0.33 (good enough for a 0.8x0.56 body)
        for o in self.robots.values():
            if o.id == rid:
                continue
            cx, cy = o.x - ox, o.y - oy
            if cx * cx + cy * cy > (range_max + 1) ** 2:
                continue
            b = cx * dx + cy * dy
            c = cx * cx + cy * cy - 0.33 ** 2
            disc = b * b - c
            t = np.where(disc >= 0, b - np.sqrt(np.maximum(disc, 0)), np.inf)
            t = np.where(t > 0, t, np.inf)
            ranges = np.minimum(ranges, t)
        return ranges

    def front_clearance(self, rid: str, half_angle: float = math.radians(35)) -> float:
        """Closest obstacle in the driving sector (cheap analytic version)."""
        r = self.robots[rid]
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        best = math.inf
        for o in self.robots.values():
            if o.id == rid:
                continue
            dx, dy = o.x - r.x, o.y - r.y
            fx = dx * c + dy * s
            fy = -dx * s + dy * c
            if fx <= 0:
                continue
            ang = math.atan2(fy, fx - LIDAR_X)
            if abs(ang) > half_angle and abs(fy) > ROBOT_WIDTH / 2 + 0.3:
                continue
            d = math.hypot(fx - LIDAR_X, fy) - 0.33
            best = min(best, d)
        return best


def footprints_overlap(a: SimRobot, b: SimRobot, margin: float = 0.0) -> bool:
    """Separating axis test for two oriented rectangles."""
    pa, pb = a.footprint(margin), b.footprint(margin)
    for poly in (pa, pb):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            axis = np.array([-e[1], e[0]])
            ra = pa @ axis
            rb = pb @ axis
            if ra.max() < rb.min() or rb.max() < ra.min():
                return False
    return True
