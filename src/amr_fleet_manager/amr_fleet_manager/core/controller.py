"""On-board path follower of one AMR (differential drive).

The fleet manager sends a polyline of traffic-graph waypoints and a *granted*
index: the robot may drive up to that waypoint and must be able to stop there.
The follower is a carrot (pure-pursuit style) controller with:

* speed profile that brakes for the granted stop point and for corners,
* corners rounded with circular arcs (`fillet_path`, same geometry as the
  path sent to Nav2), cornering speed from the lateral acceleration limit,
* rotate-in-place when the heading error is large (diff drive, U-turns),
* final yaw alignment at docks / stations,
* lidar safety field (slow-down and stop zones in the driving direction).

It is pure Python so it can run inside the ROS agent node, the kinematic
simulator and the headless test harness.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

MODE_IDLE = 0
MODE_FOLLOW = 1
MODE_HOLD = 2


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def speed_ahead(limits: Sequence[float], cum: Sequence[float], s_robot: float, i1: int,
                decel: float, horizon: float = 6.0) -> float:
    """Highest speed that still lets the robot slow down, at `decel`, to the
    limit of every segment ahead (limits[j] holds on the segment ending at
    waypoint j, which starts at cum[j - 1])."""
    if not limits:
        return math.inf
    v = limits[min(i1, len(limits) - 1)]
    for j in range(i1 + 1, len(limits)):
        d = cum[j - 1] - s_robot
        if d > horizon:
            break
        v = min(v, math.sqrt(limits[j] ** 2 + 2.0 * decel * max(d, 0.0)))
    return v


def fillet_path(pts: Sequence[Tuple[float, float]], radius: float, step: float = 0.1,
                max_turn: float = 2.5) -> Tuple[List[Tuple[float, float]], List[float], List[float]]:
    """Polyline with every interior corner replaced by a circular arc.

    Returns the dense points (spacing <= step), the arc length of the curve at
    every input vertex (middle of its arc) and the radius used at every vertex
    (0: not rounded).  The radius shrinks so that an arc never takes more than
    half of either adjacent segment; turns sharper than max_turn (U-turns) are
    left as they are (the robot rotates in place there)."""
    n = len(pts)
    out: List[Tuple[float, float]] = [tuple(pts[0])] if n else []
    vs = [0.0] * n
    radii = [0.0] * n
    cum = [0.0]

    def add(p):
        cum.append(cum[-1] + math.dist(out[-1], p))
        out.append(p)

    def line_to(q):
        a = out[-1]
        k = max(1, int(math.ceil(math.dist(a, q) / step)))
        for j in range(1, k + 1):
            add((a[0] + (q[0] - a[0]) * j / k, a[1] + (q[1] - a[1]) * j / k))

    for i in range(1, n):
        b = pts[i]
        if i == n - 1:
            line_to(b)
            vs[i] = cum[-1]
            continue
        a, c = pts[i - 1], pts[i + 1]
        l1, l2 = math.dist(a, b), math.dist(b, c)
        turn = 0.0
        if l1 > 1e-6 and l2 > 1e-6:
            u1 = ((b[0] - a[0]) / l1, (b[1] - a[1]) / l1)
            u2 = ((c[0] - b[0]) / l2, (c[1] - b[1]) / l2)
            turn = math.atan2(u1[0] * u2[1] - u1[1] * u2[0], u1[0] * u2[0] + u1[1] * u2[1])
        if abs(turn) < 0.05 or abs(turn) > max_turn:
            line_to(b)
            vs[i] = cum[-1]
            continue
        tn = math.tan(abs(turn) / 2.0)
        t = min(radius * tn, l1 / 2.0, l2 / 2.0)
        r = t / tn
        p1 = (b[0] - u1[0] * t, b[1] - u1[1] * t)
        line_to(p1)
        side = 1.0 if turn > 0 else -1.0                 # left / right turn
        cx, cy = p1[0] - side * u1[1] * r, p1[1] + side * u1[0] * r
        a0 = math.atan2(p1[1] - cy, p1[0] - cx)
        k = max(2, int(math.ceil(r * abs(turn) / step)))
        for j in range(1, k + 1):
            ang = a0 + turn * j / k
            add((cx + r * math.cos(ang), cy + r * math.sin(ang)))
            if j == k // 2:
                vs[i] = cum[-1]
        radii[i] = r
    return out, vs, radii


@dataclass
class FollowerParams:
    max_speed: float = 1.5
    max_accel: float = 0.9
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
    corner_radius: float = 1.0       # m: corners are rounded with arcs of this radius
    lat_accel: float = 0.6           # m/s^2: cornering speed sqrt(lat_accel * radius)
    corner_speed: float = 0.5
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
    rev: int = 0                 # waypoints / speed limits updated within the same command
    geom: List[Tuple[float, float]] = field(default_factory=list)   # rounded path
    geom_cum: List[float] = field(default_factory=list)
    geom_at: List[float] = field(default_factory=list)   # arc length at every waypoint
    radii: List[float] = field(default_factory=list)     # corner radius at every waypoint
    geom_hint: int = 0


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
            s.rev += 1
            self._build_geometry()
        elif len(waypoints) == len(s.waypoints):
            # same route re-shaped (shifted to pass another robot) or new speed
            # limits (speed advisory): keep the progress
            wps = [(float(x), float(y)) for x, y in waypoints]
            s.speed_limits = list(speed_limits) or [self.p.max_speed] * len(wps)
            if wps != s.waypoints:
                s.waypoints = wps
                s.cum = [0.0]
                for a, b in zip(wps, wps[1:]):
                    s.cum.append(s.cum[-1] + math.dist(a, b))
                s.rev += 1                # the geometry changed: the path is re-sent
                self._build_geometry()
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

        # carrot on the rounded path, never beyond the stop point
        look = min(max(p.lookahead_min + p.lookahead_gain * abs(s.v), p.lookahead_min), 1.2)
        g_robot = self._geom_project(x, y)
        cx, cy = self._geom_point(min(g_robot + look, s.geom_at[stop_idx]))
        if math.dist((x, y), (cx, cy)) < 0.05:
            cx, cy = wps[i1]
        err = wrap(math.atan2(cy - y, cx - x) - yaw)

        # speed profile
        # speed profiles evaluated where the robot will be at the next control
        # step: braking starts one step earlier instead of one step too late
        brake = p.max_decel
        ahead = abs(s.v) * dt
        v_lim = min(p.max_speed, speed_ahead(s.speed_limits, s.cum, s_robot + ahead, i1, brake))
        v = v_lim
        v = min(v, math.sqrt(2.0 * brake * max(d_stop - 0.02 - ahead, 0.0)))
        # corners ahead (until the stop point)
        for j in range(i1, min(stop_idx, last)):
            if j <= 0:
                continue
            ang = abs(wrap(math.atan2(wps[j + 1][1] - wps[j][1], wps[j + 1][0] - wps[j][0])
                           - math.atan2(wps[j][1] - wps[j - 1][1], wps[j][0] - wps[j - 1][0])))
            if ang < 0.3:
                continue
            dist_c = max(s.cum[j] - s_robot - s.radii[j] - ahead, 0.0)
            if dist_c > 3.0:
                break
            if s.radii[j] > 0.0:     # rounded corner: lateral acceleration limit
                vc = max(p.corner_speed, math.sqrt(p.lat_accel * s.radii[j]))
            else:
                vc = max(p.corner_speed, p.max_speed * (1.0 - ang / math.pi * 1.4))
            v = min(v, math.sqrt(vc * vc + 2.0 * brake * dist_c))
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
    def _build_geometry(self) -> None:
        s = self.s
        if not s.waypoints:
            s.geom, s.geom_cum, s.geom_at, s.radii = [], [], [], []
            return
        s.geom, s.geom_at, s.radii = fillet_path(s.waypoints, self.p.corner_radius)
        s.geom_cum = [0.0]
        for a, b in zip(s.geom, s.geom[1:]):
            s.geom_cum.append(s.geom_cum[-1] + math.dist(a, b))
        s.geom_hint = 0

    def _geom_project(self, x: float, y: float, local: bool = False) -> float:
        """Arc length of the robot projected on the rounded path (local search)."""
        s = self.s
        g = s.geom
        if len(g) < 2:
            return 0.0
        # never behind the last reached waypoint: on a route that turns back on
        # itself (U-turn) both legs overlap and the projection must not jump
        floor = 0
        if 0 <= s.reached < len(s.geom_at):
            back = 0.3 if s.radii[s.reached] > 0.0 else 0.0     # arc middle / exact vertex
            floor = bisect.bisect_left(s.geom_cum, s.geom_at[s.reached] - back)
        floor = min(floor, len(g) - 1)
        lo, hi = max(floor, s.geom_hint - 5), min(len(g), max(floor, s.geom_hint) + 40)
        best = min(range(lo, hi), key=lambda k: (g[k][0] - x) ** 2 + (g[k][1] - y) ** 2)
        d2 = (g[best][0] - x) ** 2 + (g[best][1] - y) ** 2
        if local:                        # progress: only a nearby, local match counts
            return s.geom_cum[best] if d2 < 1.0 else -1.0
        if d2 > 4.0:
            # far from the expected place (new shape / big jump): wider search
            best = min(range(floor, len(g)), key=lambda k: (g[k][0] - x) ** 2 + (g[k][1] - y) ** 2)
        s.geom_hint = best
        return s.geom_cum[best]

    def _geom_point(self, dist: float) -> Tuple[float, float]:
        s = self.s
        c = s.geom_cum
        k = s.geom_hint
        while k + 1 < len(c) and c[k + 1] < dist:
            k += 1
        if k + 1 >= len(c):
            return s.geom[-1]
        f = min(max((dist - c[k]) / max(c[k + 1] - c[k], 1e-9), 0.0), 1.0)
        a, b = s.geom[k], s.geom[k + 1]
        return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))

    def _update_reached(self, x: float, y: float) -> None:
        s, p = self.s, self.p
        wps = s.waypoints
        last = len(wps) - 1
        # a rounded corner cuts its waypoint: passing the middle of its arc counts
        g = self._geom_project(x, y, local=True) if len(s.geom_at) == len(wps) else -1.0
        while s.reached < min(s.granted, last):
            i = s.reached + 1
            if i < last and 0.0 <= s.geom_at[i] - 0.05 <= g:
                s.reached = i
                continue
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
