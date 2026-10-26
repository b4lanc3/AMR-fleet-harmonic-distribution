"""Headless (no ROS, no Gazebo) simulation of the complete fleet.

Runs FleetCore + one PathFollower per robot + the kinematic world much faster
than real time and reports KPIs and safety metrics:

    ros2 run amr_fleet_manager headless_sim --robots 15 --minutes 60
    python3 -m amr_fleet_manager.sim.headless --robots 12 --minutes 20
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from ..core.controller import FollowerParams, PathFollower
from ..core.fleet import FleetCore, FleetParams, RobotStatusIn
from ..core.layout import all_objects, build_graph, load_layout
from .kinematic import KinematicWorld, footprints_overlap


def initial_battery(i: int) -> float:
    return float(38 + (i * 37) % 60)


def spawn_poses(graph, n: int):
    """Robots spawn on the parking bays P01..Pn facing the bay."""
    ids = sorted(graph.parking)
    if n > len(ids):
        raise ValueError(f'{n} robots but only {len(ids)} parking bays')
    poses = []
    for pid in ids[:n]:
        node = graph.nodes[graph.parking[pid]]
        poses.append((node.x, node.y, node.heading or 0.0))
    return poses


def robot_name(i: int) -> str:
    return f'amr_{i + 1:02d}'


@dataclass
class Metrics:
    sim_time: float = 0.0
    wall_time: float = 0.0
    tasks_completed: int = 0
    finished_stored: int = 0
    products_made: int = 0
    yields: int = 0
    deadlocks: int = 0
    reroutes: int = 0
    collisions: int = 0
    min_distance: float = math.inf
    max_stall: float = 0.0
    max_stall_robot: str = ''
    obstacle_stops: int = 0
    charges: int = 0
    min_battery: float = 100.0
    throughput_per_hour: float = 0.0
    collision_pairs: List[str] = field(default_factory=list)
    # smoothness (robots with a route only)
    distance_km: float = 0.0
    route_time: float = 0.0            # robot-seconds with a route
    unplanned_stops: int = 0           # stopped before the end of the route
    blocked_time: float = 0.0          # robot-seconds standing still before the route end
    hard_brakes: int = 0               # braking > HARD_BRAKE m/s^2 that loses >= 0.3 m/s
    acc_sq: float = 0.0
    jerk_sq: float = 0.0
    passes: int = 0
    line_utilization: float = 0.0      # mean busy share of the production lines
    packer_utilization: float = 0.0

    @property
    def stops_per_km(self) -> float:
        return self.unplanned_stops / self.distance_km if self.distance_km > 1e-3 else 0.0

    @property
    def avg_speed(self) -> float:
        return self.distance_km * 1000.0 / self.route_time if self.route_time > 0 else 0.0

    @property
    def rms_accel(self) -> float:
        return math.sqrt(self.acc_sq / self.route_time) if self.route_time > 0 else 0.0

    @property
    def rms_jerk(self) -> float:
        return math.sqrt(self.jerk_sq / self.route_time) if self.route_time > 0 else 0.0

    @property
    def blocked_share(self) -> float:
        return self.blocked_time / self.route_time if self.route_time > 0 else 0.0

    def smoothness(self) -> str:
        return (f'smooth | {self.distance_km:.2f} km | avg {self.avg_speed:.2f} m/s '
                f'| stops {self.unplanned_stops} ({self.stops_per_km:.1f}/km) '
                f'| blocked {100 * self.blocked_share:.1f}% | hard brakes {self.hard_brakes} '
                f'| rms acc {self.rms_accel:.2f} jerk {self.rms_jerk:.2f} | passes {self.passes} '
                f'| lines {100 * self.line_utilization:.0f}% packers {100 * self.packer_utilization:.0f}%')

    def report(self) -> str:
        return (f'sim {self.sim_time / 60:.1f} min in {self.wall_time:.1f}s | tasks {self.tasks_completed} '
                f'| stored {self.finished_stored} | made {self.products_made} '
                f'| thr {self.throughput_per_hour:.0f}/h | yields {self.yields} '
                f'| deadlocks {self.deadlocks} | reroutes {self.reroutes} '
                f'| collisions {self.collisions} | min dist {self.min_distance:.2f} m '
                f'| max stall {self.max_stall:.0f}s ({self.max_stall_robot}) '
                f'| lidar stops {self.obstacle_stops} | charges {self.charges} '
                f'| min batt {self.min_battery:.0f}%')


HARD_BRAKE = 1.2      # m/s^2
MOVING, STOPPED = 0.3, 0.05


class HeadlessSim:
    def __init__(self, layout: dict, n_robots: int, fleet_params: Optional[FleetParams] = None,
                 dt: float = 0.1, fleet_period: float = 0.1) -> None:
        self.layout = layout
        self.graph = build_graph(layout)
        self.ids = [robot_name(i) for i in range(n_robots)]
        self.fleet = FleetCore(layout, self.ids, fleet_params, graph=self.graph)
        self.world = KinematicWorld(layout, objects=all_objects(layout, self.graph))
        self.followers: Dict[str, PathFollower] = {}
        for i, (rid, pose) in enumerate(zip(self.ids, spawn_poses(self.graph, n_robots))):
            self.world.add_robot(rid, pose[0], pose[1], pose[2], initial_battery(i))
            self.followers[rid] = PathFollower(FollowerParams())
        self.dt = dt
        self.fleet_period = fleet_period
        self.t = 0.0
        self._next_fleet = 0.0
        self.commands = {}
        self.metrics = Metrics()
        self._last_progress: Dict[str, tuple] = {}
        self._progress_time: Dict[str, float] = {}
        self._was_charging: Dict[str, bool] = {}
        self._colliding: set = set()
        self._v: Dict[str, float] = {}
        self._a: Dict[str, float] = {}
        self._moving: Dict[str, bool] = {}
        self._braking: Dict[str, float] = {}     # speed lost in the current hard braking

    def step(self) -> None:
        dt = self.dt
        if self.t >= self._next_fleet:
            self._next_fleet = self.t + self.fleet_period
            for rid in self.ids:
                r = self.world.robots[rid]
                f = self.followers[rid]
                self.fleet.update_status(RobotStatusIn(
                    rid, r.x, r.y, r.yaw, r.v, r.w, f.s.command_id, f.reached_index,
                    f.route_complete, r.battery, r.charging, f.s.obstacle_stop, 0.0, self.t))
            self.commands = self.fleet.tick(self.t)
            for rid, c in self.commands.items():
                self.followers[rid].set_command(c.command_id, c.mode, c.waypoints,
                                                c.speed_limits, c.granted_index, c.final_yaw)
        charge = {rid: c.charge for rid, c in self.commands.items()}
        for rid in self.ids:
            r = self.world.robots[rid]
            f = self.followers[rid]
            clr = self.world.front_clearance(rid)
            v, w = f.step(r.x, r.y, r.yaw, dt, clr)
            if f.s.obstacle_stop:
                self.metrics.obstacle_stops += 1
            r.cmd_v, r.cmd_w = v, w
        self.world.step(dt, charge)
        self.t += dt
        self._check_safety()
        self._check_smoothness()

    def _check_smoothness(self) -> None:
        m, dt = self.metrics, self.dt
        for rid in self.ids:
            r = self.world.robots[rid]
            rec = self.fleet.robots[rid]
            f = self.followers[rid]
            v = abs(r.v)
            a = (v - self._v.get(rid, v)) / dt
            jerk = (a - self._a.get(rid, a)) / dt
            self._v[rid], self._a[rid] = v, a
            m.distance_km += v * dt / 1000.0
            if not rec.route or f.route_complete:
                self._moving[rid] = False
                continue
            m.route_time += dt
            m.acc_sq += a * a * dt
            m.jerk_sq += jerk * jerk * dt
            if a < -HARD_BRAKE:
                lost = self._braking.get(rid, 0.0)
                if lost < 0.3 <= lost - a * dt:
                    m.hard_brakes += 1
                self._braking[rid] = lost - a * dt
            else:
                self._braking[rid] = 0.0
            at_end = f.s.reached >= len(f.s.waypoints) - 1
            if v < STOPPED and not at_end:
                m.blocked_time += dt
                if self._moving.get(rid):
                    m.unplanned_stops += 1
                self._moving[rid] = False
            elif v > MOVING:
                self._moving[rid] = True

    def _check_safety(self) -> None:
        m = self.metrics
        robots = list(self.world.robots.values())
        colliding = set()
        for i, a in enumerate(robots):
            m.min_battery = min(m.min_battery, a.battery)
            if a.charging and not self._was_charging.get(a.id):
                m.charges += 1
            self._was_charging[a.id] = a.charging
            for b in robots[i + 1:]:
                d = math.hypot(a.x - b.x, a.y - b.y)
                m.min_distance = min(m.min_distance, d)
                if d < 1.0 and footprints_overlap(a, b):
                    pair = f'{a.id}/{b.id}'
                    colliding.add(pair)
                    if pair not in self._colliding:
                        m.collisions += 1
                        m.collision_pairs.append(f'{pair}@{self.t:.0f}s')
        self._colliding = colliding
        # stall tracking: no progress along the route / no state change
        for rid in self.ids:
            rec = self.fleet.robots[rid]
            key = (rec.state, rec.command_id, rec.progress)
            if self._last_progress.get(rid) != key or not rec.route:
                self._last_progress[rid] = key
                self._progress_time[rid] = self.t
            stall = self.t - self._progress_time[rid]
            if stall > m.max_stall:
                m.max_stall = stall
                m.max_stall_robot = rid

    def run(self, seconds: float, verbose: bool = False, report_every: float = 300.0) -> Metrics:
        wall0 = time.time()
        next_report = report_every
        while self.t < seconds:
            self.step()
            if verbose and self.t >= next_report:
                next_report += report_every
                self._fill_metrics(time.time() - wall0)
                print(self.metrics.report(), flush=True)
                print('   ', self.metrics.smoothness(), flush=True)
        self._fill_metrics(time.time() - wall0)
        return self.metrics

    def _fill_metrics(self, wall: float) -> None:
        m = self.metrics
        m.sim_time = self.t
        m.wall_time = wall
        m.tasks_completed = self.fleet.tasks.completed
        m.finished_stored = self.fleet.production.finished_stored
        m.products_made = self.fleet.production.products_made
        m.yields = self.fleet.yields
        m.deadlocks = self.fleet.deadlocks
        m.reroutes = self.fleet.reroutes
        m.throughput_per_hour = self.fleet.production.throughput_per_hour()
        m.passes = self.fleet.passes
        snap = self.fleet.production.snapshot()
        lines = [x['utilization'] for x in snap.get('lines', []) if x.get('line_type') == 'production']
        packs = [x['utilization'] for x in snap.get('lines', []) if x.get('line_type') == 'packaging']
        m.line_utilization = sum(lines) / len(lines) if lines else 0.0
        m.packer_utilization = sum(packs) / len(packs) if packs else 0.0


def default_layout_path() -> str:
    import os
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('amr_bringup'), 'config',
                            'factory_layout.yaml')
    except Exception:  # pragma: no cover - fallback for source checkouts
        return os.path.join(os.path.dirname(__file__), '..', '..', '..', 'amr_bringup',
                            'config', 'factory_layout.yaml')


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description='Headless AMR fleet simulation')
    ap.add_argument('--layout', default=None, help='factory_layout.yaml')
    ap.add_argument('--robots', type=int, default=12)
    ap.add_argument('--minutes', type=float, default=20.0)
    ap.add_argument('--events', action='store_true', help='print fleet events')
    args = ap.parse_args(argv)
    path = args.layout or default_layout_path()
    sim = HeadlessSim(load_layout(path), args.robots)
    seen = 0
    if args.events:
        while sim.t < args.minutes * 60:
            sim.step()
            for e in list(sim.fleet.events):
                if e['seq'] > seen:
                    seen = e['seq']
                    print(f"[{e['t']:7.1f}] {e['category']:<10} {e['text']}")
        sim._fill_metrics(0.0)
        print(sim.metrics.report())
    else:
        m = sim.run(args.minutes * 60, verbose=True)
        print(m.report())
        print(m.smoothness())


if __name__ == '__main__':
    main()
