import numpy as np

from amr_fleet_manager.tools.generate_world import rasterize, world_sdf

ROBOT_HALF_WIDTH = 0.30
MARGIN = 0.05


def test_world_leaves_every_lane_open(layout, graph):
    """Nothing the world generator builds may stand on a traffic edge: the lidar
    (and Nav2's costmap) would see it and the robots could not drive the route
    that traffic control granted.  Checked on the lidar-plane occupancy map."""
    _, slices = world_sdf(layout, animate=False)
    img, origin, res = rasterize(slices, layout['factory']['size'])
    occ = img == 0
    k = int(np.ceil((ROBOT_HALF_WIDTH + MARGIN) / res))
    oy, ox = np.mgrid[-k:k + 1, -k:k + 1]
    disk = (ox ** 2 + oy ** 2) * res * res <= (ROBOT_HALF_WIDTH + MARGIN) ** 2
    blocked = []
    for es in graph.out_edges:
        for e in es:
            a, b = graph.nodes[e.u], graph.nodes[e.v]
            n = max(2, int(e.length / 0.1))
            for t in np.linspace(0.0, 1.0, n + 1):
                i = int((a.x + (b.x - a.x) * t - origin[0]) / res)
                j = int((a.y + (b.y - a.y) * t - origin[1]) / res)
                if (occ[j - k:j + k + 1, i - k:i + k + 1] & disk).any():
                    blocked.append((a.id, b.id))
                    break
    assert not blocked, f'edges blocked by world objects: {blocked[:10]}'


def test_worker_routes_keep_clear_of_machines(layout, graph):
    """Workers walk on free floor: 0.5 m from every rack, machine and conveyor
    footprint (they have no physics), and cross AMR lanes only on zebra crossings
    painted by the world generator."""
    from amr_fleet_manager.core.layout import all_objects, people_routes, people_segments
    from amr_fleet_manager.tools.generate_world import walkway_crossings
    routes = people_routes(layout)
    assert len(routes) >= 6
    rects = [o['rect'] for o in all_objects(layout, graph)]
    close = []
    crossings = 0
    for r in routes:
        for a, b in people_segments(r):
            n = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.1))
            for t in np.linspace(0.0, 1.0, n + 1):
                x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
                for x0, y0, x1, y1 in rects:
                    dx = max(x0 - x, 0.0, x - x1)
                    dy = max(y0 - y, 0.0, y - y1)
                    if np.hypot(dx, dy) < 0.5:
                        close.append((r['name'], round(x, 1), round(y, 1)))
            crossings += len(walkway_crossings(graph, a, b))
    assert not close, f'worker routes too close to objects: {close[:5]}'
    assert crossings >= 2      # the main aisle and the link lane are crossed
