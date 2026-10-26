import math

from amr_fleet_manager.core.fleet import YIELDING, RobotStatusIn
from amr_fleet_manager.sim.headless import HeadlessSim


def test_headless_fleet_is_safe_and_productive(layout):
    sim = HeadlessSim(layout, 12)
    m = sim.run(12 * 60)
    assert m.collisions == 0, m.collision_pairs
    assert m.tasks_completed >= 10
    assert m.finished_stored > 0
    assert m.max_stall < 150.0, (m.max_stall, m.max_stall_robot)


def test_fifteen_robots_no_collisions(layout):
    sim = HeadlessSim(layout, 15)
    m = sim.run(8 * 60)
    assert m.collisions == 0, m.collision_pairs
    assert m.tasks_completed >= 10


def _status(fleet, rid, x, y, yaw=0.0, t=0.0):
    r = fleet.robots[rid]
    fleet.update_status(RobotStatusIn(rid, x, y, yaw, 0.0, 0.0, r.command_id,
                                      r.progress, False, 90.0, False, False, 9.9, t))


def test_head_on_robots_pass_each_other_without_stopping(layout, graph):
    """Two robots meet head-on on the two-way north aisle: they detect each
    other, both shift to their right, pass and keep their route - nobody stops
    or drives into a yield bay."""
    sim = HeadlessSim(layout, 2)
    fleet = sim.fleet
    g = sim.graph
    # put both robots on the north aisle facing each other
    a, b = sim.world.robots['amr_01'], sim.world.robots['amr_02']
    na = g.nearest_node(40.0, 67.0)
    nb = g.nearest_node(53.0, 67.0)
    a.x, a.y, a.yaw = g.nodes[na].x, g.nodes[na].y, 0.0
    b.x, b.y, b.yaw = g.nodes[nb].x, g.nodes[nb].y, math.pi
    fleet.production.lines.clear()
    fleet.production.packers.clear()
    sim.step()
    ra, rb = fleet.robots['amr_01'], fleet.robots['amr_02']
    goal_a = g.parking['P19']                 # far east
    goal_b = g.parking['P01']                 # far west
    fleet._release_parking(ra)
    fleet._release_parking(rb)
    # force the shortest (head-on) routes, ignoring congestion-aware penalties
    route_a = g.plan(ra.current_node, goal_a)
    route_b = g.plan(rb.current_node, goal_b)
    shared = set(route_a) & set(route_b)
    assert len(shared) >= 4, 'test setup: routes must overlap on the two-way aisle'
    fleet._set_route(ra, route_a, 1, 'TO_PARK', float('nan'))
    fleet._set_route(rb, route_b, 1, 'TO_PARK', float('nan'))
    ra.parking, rb.parking = 'P19', 'P01'
    fleet.parking_booking['P19'], fleet.parking_booking['P01'] = 'amr_01', 'amr_02'
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
