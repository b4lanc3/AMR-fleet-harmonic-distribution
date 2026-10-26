"""Parameter sweep on the headless simulation: production and smoothness KPIs
for every combination of the given fleet / follower parameters.

    ros2 run amr_fleet_manager tune_sim --robots 7 --minutes 20 \\
        --fleet pass_detect=6,8,10 --fleet lookahead=5,7 --follower max_decel=0.7,0.9

Score (higher is better): finished goods per hour, minus 20 per unplanned stop
per km and 2 per hard brake per km; a collision disqualifies the setting.
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import replace
from typing import Dict, List, Tuple

from ..core.controller import FollowerParams
from ..core.fleet import FleetParams
from ..core.layout import load_layout
from .headless import HeadlessSim, Metrics, default_layout_path


def _grid(specs: List[str]) -> List[Tuple[str, List[float]]]:
    out = []
    for spec in specs or []:
        key, vals = spec.split('=', 1)
        out.append((key.strip(), [float(v) for v in vals.split(',')]))
    return out


def score(m: Metrics) -> float:
    if m.collisions:
        return -1e9
    per_km = max(m.distance_km, 1e-3)
    return m.throughput_per_hour - 20.0 * m.stops_per_km - 2.0 * m.hard_brakes / per_km


def run_one(layout: dict, robots: int, minutes: float, fleet: Dict[str, float],
            follower: Dict[str, float]) -> Metrics:
    fp = replace(FleetParams(), **fleet)
    sim = HeadlessSim(layout, robots, fp)
    for f in sim.followers.values():
        f.p = replace(FollowerParams(), **follower)
    return sim.run(minutes * 60.0)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description='Sweep fleet / follower parameters (headless)')
    ap.add_argument('--layout', default=None)
    ap.add_argument('--robots', type=int, nargs='+', default=[7])
    ap.add_argument('--minutes', type=float, default=20.0)
    ap.add_argument('--fleet', action='append', help='FleetParams field=v1,v2,...')
    ap.add_argument('--follower', action='append', help='FollowerParams field=v1,v2,...')
    args = ap.parse_args(argv)
    layout = load_layout(args.layout or default_layout_path())
    fg, wg = _grid(args.fleet), _grid(args.follower)
    keys = [('fleet', k) for k, _ in fg] + [('follower', k) for k, _ in wg]
    rows = []
    for n in args.robots:
        for combo in itertools.product(*([v for _, v in fg] + [v for _, v in wg])):
            fleet = {k: v for (kind, k), v in zip(keys, combo) if kind == 'fleet'}
            follower = {k: v for (kind, k), v in zip(keys, combo) if kind == 'follower'}
            m = run_one(layout, n, args.minutes, fleet, follower)
            rows.append((score(m), n, combo, m))
            print(f'{n:2d} robots {dict(zip([k for _, k in keys], combo))} -> score {score(m):7.1f} '
                  f'| thr {m.throughput_per_hour:4.0f}/h stored {m.finished_stored} '
                  f'| stops {m.stops_per_km:.1f}/km brakes {m.hard_brakes} '
                  f'| avg {m.avg_speed:.2f} m/s | coll {m.collisions}', flush=True)
    print('\nbest first:')
    for sc, n, combo, m in sorted(rows, key=lambda r: -r[0])[:10]:
        print(f'  {sc:7.1f}  {n:2d} robots  {dict(zip([k for _, k in keys], combo))}')
        print(f'           {m.smoothness()}')


if __name__ == '__main__':
    main()
