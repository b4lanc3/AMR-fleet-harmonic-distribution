from amr_fleet_manager.core.graph import INTERSECTION, YIELD


def test_layout_has_required_elements(layout, graph):
    zones = {z['id'] for z in layout['zones']}
    assert {'A', 'B', 'C', 'D'} <= zones
    assert len(graph.chargers) >= 6
    # chargers spread along the spine: no robot is far from a dock
    xs = sorted(graph.nodes[i].x for i in graph.chargers.values())
    assert max(b - a for a, b in zip(xs, xs[1:])) < 20.0
    assert len(graph.parking) >= 15
    assert len(graph.yield_bays) > 10
    assert not graph.build_warnings


def test_everything_is_reachable(graph):
    specials = ([s.service for s in graph.stations.values()] + list(graph.chargers.values())
                + list(graph.parking.values()) + list(graph.yield_bays))
    ok, bad = graph.is_strongly_connected(specials)
    assert ok, bad[:5]


def test_routes_only_use_transit_nodes(graph):
    a = graph.stations['A1'].service
    d = graph.stations['G2'].service
    path = graph.plan(d, a)
    assert path[0] == d and path[-1] == a
    for n in path[1:-1]:
        node = graph.nodes[n]
        assert node.transit or node.group in ('A1', 'G2'), node.id


def test_conflict_sets_are_symmetric(graph):
    for i, c in enumerate(graph.conflicts):
        assert i in c
        for j in c:
            assert i in graph.conflicts[j]


def test_yield_bays_are_off_lane(graph):
    for y in graph.yield_bays:
        node = graph.nodes[y]
        assert node.kind == YIELD
        assert not node.transit
        # a bay is reachable from a lane and leads back to a lane
        assert graph.in_edges[y] and graph.out_edges[y]
    assert any(n.kind == INTERSECTION for n in graph.nodes)
