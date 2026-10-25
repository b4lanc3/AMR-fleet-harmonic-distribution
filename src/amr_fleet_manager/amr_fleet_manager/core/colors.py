"""Robot identity colours shared by Gazebo, RViz and the dashboard.

Eight categorical slots in a fixed order (the dashboard uses the same order);
robots 9..16 reuse a slot and are told apart by a darker body / hollow marker
and, everywhere, by their id label.
"""

SLOTS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']


def hex_to_rgb(h: str):
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def robot_rgb(i: int):
    r, g, b = hex_to_rgb(SLOTS[i % len(SLOTS)])
    if i >= len(SLOTS):
        r, g, b = r * 0.6, g * 0.6, b * 0.6
    return (r, g, b)


def robot_rgba_str(i: int) -> str:
    r, g, b = robot_rgb(i)
    return f'{r:.3f} {g:.3f} {b:.3f} 1'
