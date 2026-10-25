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
