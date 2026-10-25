"""Generate an RViz2 config for a fleet of N robots.

  ros2 run amr_fleet_manager rviz_config --robots 12 --out fleet.rviz
"""

from __future__ import annotations

import argparse
from typing import List

import yaml

from ..core.colors import robot_rgb


def _topic(name: str, durability: str = 'Volatile', reliability: str = 'Reliable') -> dict:
    return {'Depth': 5, 'Durability Policy': durability, 'History Policy': 'Keep Last',
            'Reliability Policy': reliability, 'Value': name}


def make_rviz_config(ids: List[str], camera_robot: str = '', center=(50.0, 30.0)) -> str:
    robots = []
    scans = []
    for i, rid in enumerate(ids):
        r, g, b = robot_rgb(i)
        rgb = f'{int(r * 255)}; {int(g * 255)}; {int(b * 255)}'
        robots.append({
            'Class': 'rviz_default_plugins/RobotModel', 'Name': rid, 'Enabled': True, 'Alpha': 1,
            'Description Source': 'Topic',
            'Description Topic': _topic(f'/{rid}/robot_description', 'Transient Local'),
            'TF Prefix': rid, 'Visual Enabled': True, 'Collision Enabled': False,
            'Update Interval': 0})
        scans.append({
            'Class': 'rviz_default_plugins/LaserScan', 'Name': f'{rid} lidar', 'Enabled': True,
            'Topic': _topic(f'/{rid}/scan', reliability='Best Effort'),
            'Color Transformer': 'FlatColor', 'Color': rgb, 'Size (m)': 0.05,
            'Style': 'Points', 'Size (Pixels)': 3, 'Decay Time': 0, 'Queue Size': 5,
            'Position Transformer': 'XYZ', 'Use Fixed Frame': True})
    displays = [
        {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid', 'Enabled': False,
         'Cell Size': 5, 'Plane Cell Count': 24, 'Offset': {'X': center[0], 'Y': center[1], 'Z': 0}},
        {'Class': 'rviz_default_plugins/Map', 'Name': 'Factory map', 'Enabled': True, 'Alpha': 0.35,
         'Color Scheme': 'map', 'Draw Behind': True,
         'Topic': _topic('/map', 'Transient Local'),
         'Update Topic': _topic('/map_updates')},
        {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Factory layout & traffic graph',
         'Enabled': True, 'Namespaces': {},
         'Topic': _topic('/fleet/static_markers', 'Transient Local')},
        {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Fleet: labels, routes, reservations',
         'Enabled': True, 'Namespaces': {}, 'Topic': _topic('/fleet/markers')},
        {'Class': 'rviz_common/Group', 'Name': 'Robots', 'Enabled': True, 'Displays': robots},
        {'Class': 'rviz_common/Group', 'Name': 'Lidar 2D', 'Enabled': True, 'Displays': scans},
    ]
    if camera_robot:
        displays.append({'Class': 'rviz_common/Group', 'Name': f'3D camera ({camera_robot})',
                         'Enabled': True, 'Displays': [
            {'Class': 'rviz_default_plugins/PointCloud2', 'Name': 'points', 'Enabled': True,
             'Topic': _topic(f'/{camera_robot}/camera/points', reliability='Best Effort'),
             'Color Transformer': 'RGB8', 'Style': 'Points', 'Size (Pixels)': 2,
             'Size (m)': 0.02, 'Decay Time': 0, 'Position Transformer': 'XYZ',
             'Use Fixed Frame': True},
            {'Class': 'rviz_default_plugins/Image', 'Name': 'rgb image', 'Enabled': True,
             'Topic': _topic(f'/{camera_robot}/camera/image', reliability='Best Effort'),
             'Normalize Range': True, 'Max Value': 1, 'Min Value': 0},
        ]})
    cfg = {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Name': 'Displays', 'Help Height': 0},
            {'Class': 'rviz_common/Views', 'Name': 'Views'},
        ],
        'Visualization Manager': {
            'Class': '', 'Displays': displays, 'Enabled': True,
            'Global Options': {'Background Color': '42; 44; 48', 'Fixed Frame': 'map',
                               'Frame Rate': 30},
            'Tools': [{'Class': 'rviz_default_plugins/Interact'},
                      {'Class': 'rviz_default_plugins/MoveCamera'},
                      {'Class': 'rviz_default_plugins/Select'},
                      {'Class': 'rviz_default_plugins/Measure'}],
            'Views': {
                'Current': {'Class': 'rviz_default_plugins/Orbit', 'Name': 'Factory',
                            'Distance': 78.0, 'Pitch': 1.05, 'Yaw': 4.71,
                            'Focal Point': {'X': center[0], 'Y': center[1] - 4.0, 'Z': 0.0},
                            'Field of View': 0.785, 'Near Clip Distance': 0.01,
                            'Target Frame': '<Fixed Frame>', 'Value': 'Orbit (rviz)'},
                'Saved': [
                    {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Top down',
                     'Scale': 14.0, 'X': center[0], 'Y': center[1], 'Angle': 0,
                     'Target Frame': '<Fixed Frame>', 'Value': 'TopDownOrtho (rviz_default_plugins)'},
                ]},
        },
        'Window Geometry': {'Width': 1600, 'Height': 960, 'Hide Left Dock': False,
                            'Displays': {'collapsed': False}},
    }
    return yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', type=int, default=12)
    ap.add_argument('--camera-robot', default='amr_01')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    ids = [f'amr_{i + 1:02d}' for i in range(a.robots)]
    with open(a.out, 'w') as f:
        f.write(make_rviz_config(ids, a.camera_robot))
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
