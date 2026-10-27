import copy

from amr_fleet_manager.core.layout import line_lead_time
from amr_fleet_manager.core.production import BLOCKED, Production
from amr_fleet_manager.core.tasks import FG_PICK, FG_STORE, SHIPMENT


def _mes(layout, **process):
    p = copy.deepcopy(layout['process'])
    p.update(process)
    return Production(p, layout['lines'])


def test_products_cross_the_line_in_the_lead_time(layout):
    """A product is started every takt and reaches the output carton after
    loader + belt + drop seconds."""
    mes = _mes(layout, initial_wip=False, active_lines='all')
    ln = mes.lines['B1']
    ln.input_kits = 1
    lead = sum(line_lead_time(layout['process'], layout['lines'][0]))
    assert abs(ln.lead - lead) < 1e-9 and ln.lead > ln.takt
    t, dt, first = 0.0, 0.1, None
    while t < ln.takt + ln.lead + 1.0:
        mes.update(dt, t, {}, {})
        t += dt
        if first is None and ln.box_fill:
            first = t
    assert abs(first - (ln.takt + ln.lead)) < 0.5, first
    assert ln.material == ln.products_per_kit - len(ln.items) - ln.box_fill


def test_full_output_blocks_the_whole_line(layout):
    mes = _mes(layout, initial_wip=False, active_lines='all')
    ln = mes.lines['B1']
    ln.input_kits, ln.box_fill, ln.out_boxes = 2, ln.box_capacity, ln.output_capacity
    ln.items = [ln.lead - 0.05]
    mes.update(0.1, 0.0, {}, {})
    assert ln.status == BLOCKED
    assert ln.items == [ln.lead - 0.05]       # nothing moves
    ln.out_boxes -= 1                         # an AMR took a carton
    mes.update(0.1, 0.1, {}, {})
    assert ln.status != BLOCKED and ln.box_fill == 1 and ln.out_boxes == ln.output_capacity


def test_auto_lines_follow_the_fleet_size(layout):
    mes = _mes(layout, active_lines='auto', robots_per_line=2.0)
    mes.set_fleet_size(5, 0.0)
    assert mes.active_lines == 3
    assert [ln.id for ln in mes.lines.values() if ln.active] == ['B1', 'B2', 'B3']
    assert sum(pk.active for pk in mes.packers.values()) == 3
    mes.set_fleet_size(8, 1.0)
    assert mes.active_lines == 4              # more robots: at once
    mes.set_fleet_size(5, 2.0)
    assert mes.active_lines == 4              # fewer: only after a while
    mes.set_fleet_size(5, 30.0)
    assert mes.active_lines == 3
    # a switched off line gets no kits, its packer no cartons
    tasks = mes.update(0.1, 31.0, {}, {})
    assert all(t.meta.get('line') in ('B1', 'B2', 'B3') for t in tasks if t.type == 'KIT_DELIVERY')


def test_gazebo_state_lists_every_line(layout):
    mes = _mes(layout)
    state = dict(part.split('=') for part in mes.gz_state().split(';'))
    assert len(state) == len(layout['lines']) + len(mes.gates)
    assert all(state[g] == '0' for g in mes.gates)            # empty trucks
    head, ages = state['B1'].split('|')
    run, n_in, open_left, fill, n_out = map(int, head.split(','))
    ln = mes.lines['B1']
    assert (n_in, open_left, fill, n_out) == (ln.input_kits, ln.material, ln.box_fill, ln.out_boxes)
    assert [float(a) for a in ages.split()] == [round(a, 2) for a in ln.items]


def _serve(mes, task):
    mes.on_pickup(task)
    mes.on_dropoff(task)


def test_material_flows_through_the_warehouse_to_the_gates(layout):
    """B_OUT -> warehouse (FG_STORE), warehouse -> C_IN (FG_PICK),
    C_OUT -> gate (SHIPMENT); a full truck leaves."""
    mes = _mes(layout, initial_wip=False, active_lines='all', truck_capacity=2,
               warehouse={'inbound': ['DI1', 'DI2'], 'outbound': ['DO1', 'DO2'],
                          'capacity': 3, 'initial_stock': 0})
    ln, pk = mes.lines['B1'], mes.packers['C1']
    ln.out_boxes = 1
    tasks = mes.update(0.1, 0.0, {}, {})
    store = [t for t in tasks if t.type == FG_STORE]
    assert len(store) == 1 and store[0].pickup == ln.output and store[0].dropoff in ('DI1', 'DI2')
    assert not [t for t in tasks if t.type == FG_PICK]          # warehouse empty
    _serve(mes, store[0])
    assert mes.stock == 1 and ln.out_boxes == 0
    picks = [t for t in mes.update(0.1, 0.1, {}, {}) if t.type == FG_PICK]
    assert len(picks) == 1 and picks[0].pickup in ('DO1', 'DO2')
    assert picks[0].dropoff == mes.packers[picks[0].meta['packer']].input
    _serve(mes, picks[0])
    assert mes.stock == 0 and mes.packers[picks[0].meta['packer']].input_boxes == 1
    pk.out_pallets = 2
    ships = [t for t in mes.update(0.1, 0.2, {}, {}) if t.type == SHIPMENT]
    assert len(ships) == 2 and all(t.dropoff in layout['process']['gates'] for t in ships)
    for t in ships:
        t.dropoff = 'G1'
        _serve(mes, t)
    assert mes.cartons_shipped == 2 and mes.trucks_out['G1'] == 1 and mes.truck_load['G1'] == 0
    assert mes.products_shipped == 2 * pk.pallet_capacity


def test_full_warehouse_holds_the_production_back(layout):
    mes = _mes(layout, initial_wip=False, active_lines='all',
               warehouse={'inbound': ['DI1'], 'outbound': ['DO1'], 'capacity': 2,
                          'initial_stock': 2})
    mes.lines['B1'].out_boxes = 1
    for pk in mes.packers.values():
        pk.active = False                                       # nobody takes stock out
    assert not [t for t in mes.update(0.1, 0.0, {}, {}) if t.type == FG_STORE]
