import copy

from amr_fleet_manager.core.layout import line_lead_time
from amr_fleet_manager.core.production import BLOCKED, Production


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
    assert len(state) == len(layout['lines'])
    head, ages = state['B1'].split('|')
    run, n_in, open_left, fill, n_out = map(int, head.split(','))
    ln = mes.lines['B1']
    assert (n_in, open_left, fill, n_out) == (ln.input_kits, ln.material, ln.box_fill, ln.out_boxes)
    assert [float(a) for a in ages.split()] == [round(a, 2) for a in ln.items]
