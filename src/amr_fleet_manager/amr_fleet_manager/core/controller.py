"""On-board path follower of one AMR (differential drive).

The fleet manager sends a polyline of traffic-graph waypoints and a *granted*
index: the robot may drive up to that waypoint and must be able to stop there.
The follower is a carrot (pure-pursuit style) controller with:

* speed profile that brakes for the granted stop point and for corners,
* rotate-in-place when the heading error is large (diff drive),
* final yaw alignment at docks / stations,
* lidar safety field (slow-down and stop zones in the driving direction).

It is pure Python so it can run inside the ROS agent node, the kinematic
simulator and the headless test harness.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

MODE_IDLE = 0
MODE_FOLLOW = 1
MODE_HOLD = 2


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class FollowerParams:
    max_speed: float = 1.0
    max_accel: float = 0.7
    max_decel: float = 0.9
    max_omega: float = 1.3
    omega_accel: float = 3.0
    k_omega: float = 2.4
    lookahead_min: float = 0.45
    lookahead_gain: float = 0.5
    reach_tol: float = 0.30
    final_tol: float = 0.10
    yaw_tol: float = 0.06
    rotate_threshold: float = 0.85   # rad: above this heading error -> rotate in place
    corner_speed: float = 0.35
    safety_stop: float = 0.30        # m in front of the lidar
    safety_slow: float = 0.85
    slow_speed: float = 0.25


@dataclass
class FollowerState:
    command_id: int = 0
    mode: int = MODE_IDLE
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    speed_limits: List[float] = field(default_factory=list)
    cum: List[float] = field(default_factory=list)
    granted: int = -1
    final_yaw: float = float('nan')
    reached: int = -1
    complete: bool = False
    v: float = 0.0
    w: float = 0.0
    obstacle_stop: bool = False


class PathFollower:
    def __init__(self, params: Optional[FollowerParams] = None) -> None:
        self.p = params or FollowerParams()
        self.s = FollowerState()

    # ----------------------------------------------------------- commands
    def set_command(self, command_id: int, mode: int,
                    waypoints: Sequence[Tuple[float, float]],
                    speed_limits: Sequence[float], granted: int,
                    final_yaw: float) -> None:
        s = self.s
        if command_id != s.command_id:
            s.command_id = command_id
            s.waypoints = [(float(x), float(y)) for x, y in waypoints]
            s.speed_limits = list(speed_limits) or [self.p.max_speed] * len(s.waypoints)
            s.cum = [0.0]
            for a, b in zip(s.waypoints, s.waypoints[1:]):
                s.cum.append(s.cum[-1] + math.dist(a, b))
            s.reached = 0 if s.waypoints else -1
            s.complete = len(s.waypoints) <= 1 and math.isnan(final_yaw)
        s.mode = mode
        s.granted = min(granted, len(s.waypoints) - 1)
        s.final_yaw = final_yaw

    # --------------------------------------------------------------- step
    def step(self, x: float, y: float, yaw: float, dt: float,
             front_clearance: float = math.inf) -> Tuple[float, float]:
        """Return (v, w) for the current pose."""
        p, s = self.p, self.s
        s.obstacle_stop = False
        if s.mode != MODE_FOLLOW or not s.waypoints or s.granted < 0:
            return self._ramp(0.0, 0.0, dt)

        wps = s.waypoints
        last = len(wps) - 1
        self._update_reached(x, y)

        stop_idx = s.granted
        at_end = s.reached >= last
        if at_end or (s.reached >= stop_idx and stop_idx == last):
            return self._finish(x, y, yaw, dt)

        # position along the polyline
        i0 = max(s.reached, 0)
        i1 = min(i0 + 1, last)
        a, b = wps[i0], wps[i1]
        seg_len = max(math.dist(a, b), 1e-6)
        t = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / (seg_len ** 2)
        t = min(max(t, 0.0), 1.0)
        s_robot = s.cum[i0] + t * seg_len
        s_stop = s.cum[stop_idx]
        d_stop = s_stop - s_robot
        if stop_idx > i0:
            d_stop = max(d_stop, 0.0)
        # also measure the straight distance to the stop point (corner cutting)
        d_stop = min(d_stop, math.dist((x, y), wps[stop_idx]) + 0.02) \
            if stop_idx == i1 else d_stop

        if s.reached >= stop_idx or d_stop < 0.03:
            # waiting at a granted stop point
            return self._ramp(0.0, 0.0, dt)

        # carrot
        look = min(max(p.lookahead_min + p.lookahead_gain * abs(s.v), p.lookahead_min), 1.2)
        s_carrot = min(s_robot + look, s_stop)
        cx, cy = self._point_at(s_carrot)
        if math.dist((x, y), (cx, cy)) < 0.05:
            cx, cy = wps[i1]
        err = wrap(math.atan2(cy - y, cx - x) - yaw)

        # speed profile
        v_lim = min(p.max_speed, s.speed_limits[i1] if i1 < len(s.speed_limits) else p.max_speed)
        v = v_lim
        v = min(v, math.sqrt(2.0 * p.max_decel * max(d_stop - 0.02, 0.0)))
        # corners ahead (until the stop point)
        for j in range(i1, min(stop_idx, last)):
            if j <= 0:
                continue
            ang = abs(wrap(math.atan2(wps[j + 1][1] - wps[j][1], wps[j + 1][0] - wps[j][0])
                           - math.atan2(wps[j][1] - wps[j - 1][1], wps[j][0] - wps[j - 1][0])))
            if ang < 0.3:
                continue
            dist_c = max(s.cum[j] - s_robot, 0.0)
            if dist_c > 3.0:
                break
            vc = max(p.corner_speed, p.max_speed * (1.0 - ang / math.pi * 1.4))
            v = min(v, math.sqrt(vc * vc + 2.0 * p.max_decel * dist_c))
        if abs(err) > p.rotate_threshold:
            v = 0.0
        else:
            v *= max(0.0, math.cos(err)) ** 2

        # safety field (front lidar)
        if front_clearance < p.safety_stop:
            v = 0.0
            s.obstacle_stop = True
        elif front_clearance < p.safety_slow:
            frac = (front_clearance - p.safety_stop) / (p.safety_slow - p.safety_stop)
            v = min(v, p.slow_speed * frac + 0.05)

        w = max(-p.max_omega, min(p.max_omega, p.k_omega * err))
        return self._ramp(v, w, dt)

    # ------------------------------------------------------------ helpers
    def _update_reached(self, x: float, y: float) -> None:
        s, p = self.s, self.p
        wps = s.waypoints
        last = len(wps) - 1
        while s.reached < min(s.granted, last):
            i = s.reached + 1
            d = math.dist((x, y), wps[i])
            prev = wps[i - 1]
            seg = (wps[i][0] - prev[0], wps[i][1] - prev[1])
            seg_len2 = seg[0] ** 2 + seg[1] ** 2
            proj = ((x - prev[0]) * seg[0] + (y - prev[1]) * seg[1]) / max(seg_len2, 1e-9)
            if i == last:
                ok = d < p.final_tol or (proj >= 1.0 and d < 0.35)
            else:
                ok = d < p.reach_tol or (proj >= 1.0 and d < 0.8)
            if not ok:
                break
            s.reached = i

    def _finish(self, x: float, y: float, yaw: float, dt: float) -> Tuple[float, float]:
        s, p = self.s, self.p
        if s.complete:
            return self._ramp(0.0, 0.0, dt)
        if math.isnan(s.final_yaw):
            s.complete = True
            return self._ramp(0.0, 0.0, dt)
        err = wrap(s.final_yaw - yaw)
        if abs(err) < p.yaw_tol and abs(s.w) < 0.15:
            s.complete = True
            return self._ramp(0.0, 0.0, dt)
        w = max(-p.max_omega * 0.7, min(p.max_omega * 0.7, p.k_omega * err))
        return self._ramp(0.0, w, dt)

    def _point_at(self, dist: float) -> Tuple[float, float]:
        s = self.s
        for i in range(len(s.cum) - 1):
            if s.cum[i + 1] >= dist:
                L = max(s.cum[i + 1] - s.cum[i], 1e-9)
                f = (dist - s.cum[i]) / L
                a, b = s.waypoints[i], s.waypoints[i + 1]
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        return s.waypoints[-1]

    def _ramp(self, v: float, w: float, dt: float) -> Tuple[float, float]:
        s, p = self.s, self.p
        dv = v - s.v
        dv = max(-p.max_decel * 1.6 * dt, min(p.max_accel * dt, dv)) if v >= 0 else dv
        if v == 0.0 and abs(s.v) < p.max_decel * 1.6 * dt:
            s.v = 0.0
        else:
            s.v += dv
        dw = w - s.w
        dw = max(-p.omega_accel * dt, min(p.omega_accel * dt, dw))
        s.w += dw
        return s.v, s.w

    # ------------------------------------------------------------ queries
    @property
    def reached_index(self) -> int:
        return self.s.reached

    @property
    def route_complete(self) -> bool:
        return self.s.complete
