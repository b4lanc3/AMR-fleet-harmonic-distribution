import math

import pytest

from amr_fleet_manager.core.controller import MODE_FOLLOW, PathFollower

nav2_driver = pytest.importorskip('amr_fleet_manager.nodes.nav2_driver')

# west lane -> corner -> north aisle, granted up to (12, 67)
ROUTE = [(6, 61.5), (6, 63.5), (6, 65.25), (6, 67), (8, 67), (10, 67), (12, 67), (14, 67)]


def tracker(granted=6):
    f = PathFollower()
    f.set_command(2, MODE_FOLLOW, ROUTE, [], granted, 0.0)
    d = nav2_driver.Nav2Driver.__new__(nav2_driver.Nav2Driver)   # no ROS node needed
    d.f = f
    return f, d


def advance(f, d, x, y):
    f._update_reached(x, y)
    d._advance(x, y)
    return f.s.reached


def test_progress_survives_corner_cutting():
    """Pure pursuit passes the corner node (6, 67) ~0.5 m away: still counted."""
    f, d = tracker()
    for x, y in [(6, 62), (6, 64), (6.2, 65.6), (6.9, 66.6)]:
        advance(f, d, x, y)
    assert f.s.reached == 3
    assert advance(f, d, 9.9, 67.0) == 5
    assert advance(f, d, 11.93, 67.0) == 6          # end of the grant


def test_progress_never_passes_the_grant_or_jumps_far():
    f, d = tracker(granted=4)
    assert advance(f, d, 13.9, 67.0) == 0           # far from the route window: no jump
    f, d = tracker(granted=4)
    for x, y in [(6, 63), (6, 65), (6.6, 66.8), (7.9, 67.0)]:
        advance(f, d, x, y)
    assert f.s.reached == 4


def test_pose_composition_matches_tf():
    """map->base = map->odom * odom->base, map->odom from the AMCL pose at scan time."""
    odom_at_scan = (2.0, 1.0, 0.3)
    amcl = (12.0, 40.0, 1.2)
    map_odom = nav2_driver._compose(amcl, nav2_driver._inverse(odom_at_scan))
    back = nav2_driver._compose(map_odom, odom_at_scan)
    assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(back, amcl))
