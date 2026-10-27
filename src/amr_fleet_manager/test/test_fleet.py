import math

from amr_fleet_manager.core.fleet import YIELDING, RobotStatusIn
from amr_fleet_manager.sim.headless import HeadlessSim


def test_headless_fleet_is_safe_and_productive(layout):
    sim = HeadlessSim(layout, 6)
    m = sim.run(12 * 60)
    assert m.collisions == 0, m.collision_pairs
    assert m.tasks_completed >= 10
    assert m.finished_stored > 0
    assert m.max_stall < 150.0, (m.max_stall, m.max_stall_robot)


def test_eight_robots_no_collisions(layout):
    sim = HeadlessSim(layout, 8)
    m = sim.run(8 * 60)
    assert m.collisions == 0, m.collision_pairs
    assert m.tasks_completed >= 10


def _status(fleet, rid, x, y, yaw=0.0, t=0.0):
    r = fleet.robots[rid]
    fleet.update_status(RobotStatusIn(rid, x, y, yaw, 0.0, 0.0, r.command_id,
                                      r.progress, False, 90.0, False, False, 9.9, t))


def test_head_on_robots_pass_each_other_without_stopping(layout, graph):
    """Two robots meet head-on on the two-way mid lane: they detect each
    other, both shift to their right, pass and keep their route - nobody stops
    or drives into a yield bay."""
    sim = HeadlessSim(layout, 2)
    fleet = sim.fleet
    g = sim.graph
    # put both robots on the mid lane facing each other
    a, b = sim.world.robots['amr_01'], sim.world.robots['amr_02']
    na = g.nearest_node(35.0, 14.0)
    nb = g.nearest_node(35.0, 27.0)
    a.x, a.y, a.yaw = g.nodes[na].x, g.nodes[na].y, math.pi / 2
    b.x, b.y, b.yaw = g.nodes[nb].x, g.nodes[nb].y, -math.pi / 2
    fleet.production.lines.clear()
    fleet.production.packers.clear()
    sim.step()
    ra, rb = fleet.robots['amr_01'], fleet.robots['amr_02']
    # goals: the yield bays at both ends of the lane (like the head-on demo)
    bays = sorted((i for i in g.yield_bays if g.nodes[i].id.startswith('Y.mid.')),
                  key=lambda i: g.nodes[i].y)
    goal_a, goal_b = bays[-1], bays[0]
    fleet._release_parking(ra)
    fleet._release_parking(rb)
    route_a = g.plan(ra.current_node, goal_a)
    route_b = g.plan(rb.current_node, goal_b)
    shared = set(route_a) & set(route_b)
    assert len(shared) >= 4, 'test setup: routes must overlap on the two-way lane'
    fleet.yield_booking[goal_a], fleet.yield_booking[goal_b] = 'amr_01', 'amr_02'
    fleet._set_route(ra, route_a, 1, 'TO_PARK', float('nan'))
    fleet._set_route(rb, route_b, 1, 'TO_PARK', float('nan'))
    yielded = stalled = False
    closest = math.inf
    passing_speed = math.inf
    for _ in range(int(240 / sim.dt)):
        sim.step()
        yielded |= YIELDING in (ra.state, rb.state)
        stalled |= ra.stall_since is not None or rb.stall_since is not None
        d = math.hypot(a.x - b.x, a.y - b.y)
        closest = min(closest, d)
        if d < 4.0:
            passing_speed = min(passing_speed, abs(a.v), abs(b.v))
        if not ra.route and not rb.route and ra.state == 'PARKED' and rb.state == 'PARKED':
            break
    assert ra.current_node == goal_a and rb.current_node == goal_b
    assert not yielded, 'a robot drove into a yield bay'
    assert not stalled, 'a robot stopped for the other one'
    assert fleet.passes >= 1
    assert passing_speed > 0.3, f'robots slowed down to {passing_speed:.2f} m/s while passing'
    assert closest > 1.0, f'robots passed {closest:.2f} m apart'
    assert sim.metrics.collisions == 0
    texts = ' '.join(e['text'] for e in fleet.events)
    assert 'lệch phải' in texts


def test_demo_head_on_scenario(layout):
    sim = HeadlessSim(layout, 4)
    sim.fleet.production.lines.clear()
    sim.fleet.production.packers.clear()
    sim.run(30)
    ok, msg = sim.fleet.start_demo_head_on()
    assert ok, msg
    sim.run(sim.t + 420)
    assert sim.fleet.demo is None, 'demo did not finish'
    assert sim.fleet.passes >= 1
    assert sim.fleet.yields == 0
    assert sim.metrics.collisions == 0
    texts = ' '.join(e['text'] for e in sim.fleet.events)
    assert 'lệch phải' in texts and 'Demo đối đầu hoàn tất' in texts


def test_station_arm_handover_ends_loading_and_unloading(layout):
    """With station arms (Gazebo) a robot at a line station waits for the arm's
    handover instead of a fixed time; without the handover it would wait for
    the timeout."""
    from amr_fleet_manager.core.fleet import LOADING, UNLOADING, FleetParams
    sim = HeadlessSim(layout, 3, FleetParams(station_handover=True))
    fleet = sim.fleet
    seen = set()
    while sim.t < 600 and len(seen) < 2:
        sim.step()
        for r in fleet.robots.values():
            st = r.task.pickup if r.state == LOADING else r.task.dropoff if r.state == UNLOADING else None
            if st in fleet.arm_stations and r.handover_wait and (r.id, st) not in seen:
                assert r.work_until - fleet.now > 20.0, 'still on the fixed loading time'
                state = r.state
                assert fleet.station_handover(st, r.id)
                assert not fleet.station_handover(st, r.id)    # only once
                for _ in range(3):
                    sim.step()
                assert r.state != state, 'handover did not end the station work'
                seen.add((r.id, st))
    assert len(seen) == 2, seen
    texts = ' '.join(e['text'] for e in fleet.events)
    assert 'Tay máy' in texts


def test_hungarian_matches_brute_force():
    import itertools
    import random
    from amr_fleet_manager.core.assign import hungarian
    rng = random.Random(3)
    for _ in range(200):
        rows, cols = rng.randint(1, 5), rng.randint(1, 5)
        cost = [[rng.choice([math.inf, rng.uniform(0, 50), rng.uniform(0, 50)]) for _ in range(cols)]
                for _ in range(rows)]
        got = hungarian(cost)
        best = (0, 0.0)
        for perm in itertools.permutations(range(max(rows, cols))):
            pairs = [(i, perm[i]) for i in range(rows) if perm[i] < cols and cost[i][perm[i]] != math.inf]
            cand = (len(pairs), -sum(cost[i][j] for i, j in pairs))
            if cand > (best[0], -best[1]) or best == (0, 0.0):
                best = (len(pairs), -cand[1])
        assert len(got) == best[0]
        assert math.isclose(sum(cost[i][j] for i, j in got), best[1], abs_tol=1e-6)
