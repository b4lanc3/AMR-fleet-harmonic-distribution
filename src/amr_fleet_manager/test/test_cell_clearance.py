from amr_fleet_manager.tools.cell_clearance import min_clearance, reach_error


def test_work_cell_arms_never_touch():
    """The station arm (every carton job) and the loader arm (every product
    pick on another slot) keep clear of each other."""
    gap, where = min_clearance(samples=5)
    assert gap > 0.01, where


def test_work_cell_points_are_within_reach():
    assert reach_error() < 1e-3
