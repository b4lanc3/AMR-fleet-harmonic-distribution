import math

from amr_fleet_manager.core.controller import MODE_FOLLOW, PathFollower


def simulate(f, x, y, yaw, steps, dt=0.05):
    v = w = 0.0
    for _ in range(steps):
        v, w = f.step(x, y, yaw, dt)
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
    return x, y, yaw


def test_follows_route_and_aligns():
    f = PathFollower()
    wps = [(0, 0), (2, 0), (4, 0), (4, 2), (4, 4)]
    f.set_command(1, MODE_FOLLOW, wps, [1.0] * 5, 4, math.pi / 2)
    x, y, yaw = simulate(f, 0.0, 0.0, 0.0, 600)
    assert f.route_complete
    assert math.hypot(x - 4, y - 4) < 0.15
    assert abs(yaw - math.pi / 2) < 0.1


def test_stops_at_granted_waypoint():
    f = PathFollower()
    wps = [(0, 0), (2, 0), (4, 0), (6, 0)]
    f.set_command(1, MODE_FOLLOW, wps, [1.0] * 4, 1, float('nan'))
    x, y, _ = simulate(f, 0.0, 0.0, 0.0, 400)
    assert not f.route_complete
    assert f.reached_index == 1
    assert abs(x - 2.0) < 0.3
    # extend the grant on the same command
    f.set_command(1, MODE_FOLLOW, wps, [1.0] * 4, 3, float('nan'))
    x, y, _ = simulate(f, x, y, 0.0, 400)
    assert f.route_complete and abs(x - 6.0) < 0.2


def test_safety_field_stops_robot():
    f = PathFollower()
    f.set_command(1, MODE_FOLLOW, [(0, 0), (5, 0)], [1.0, 1.0], 1, float('nan'))
    for _ in range(40):
        v, _ = f.step(0.0, 0.0, 0.0, 0.05, front_clearance=0.2)
    assert v == 0.0 and f.s.obstacle_stop
