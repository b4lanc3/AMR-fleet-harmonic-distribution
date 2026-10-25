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


def make_rviz_config(ids: List[str], camera_robot: str = '', center=(60.0, 36.0),
                     nav2: bool = False) -> str:
    robots = []
    scans = []
    particles = []
    safety = []
    plans = []
    for i, rid in enumerate(ids):
        r, g, b = robot_rgb(i)
        rgb = f'{int(r * 255)}; {int(g * 255)}; {int(b * 255)}'
        if nav2:
            particles.append({
                'Class': 'nav2_rviz_plugins/ParticleCloud', 'Name': f'{rid} AMCL', 'Enabled': True,
                'Topic': _topic(f'/{rid}/particle_cloud', reliability='Best Effort'),
                'Color': rgb, 'Alpha': 1, 'Shape': 'Arrow (Flat)', 'Arrow Length': 0.18,
                'Min Arrow Length': 0.1, 'Max Arrow Length': 0.3})
            for poly, col in (('polygon_stop', '239; 41; 41'), ('polygon_slowdown', '252; 175; 62')):
                safety.append({
                    'Class': 'rviz_default_plugins/Polygon', 'Name': f'{rid} {poly}',
                    'Enabled': True, 'Color': col, 'Alpha': 1,
                    'Topic': _topic(f'/{rid}/{poly}')})
            plans.append({
                'Class': 'rviz_default_plugins/Path', 'Name': f'{rid} FollowPath', 'Enabled': True,
                'Topic': _topic(f'/{rid}/received_global_plan'), 'Color': rgb, 'Alpha': 1,
                'Line Style': 'Lines', 'Line Width': 0.03, 'Buffer Length': 1,
                'Offset': {'X': 0, 'Y': 0, 'Z': 0.05}, 'Pose Style': 'None'})
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
    displays.append({
        'Class': 'rviz_default_plugins/PoseArray', 'Name': 'Workers (Gazebo)', 'Enabled': True,
        'Topic': _topic('/factory/people'), 'Shape': 'Arrow (3D)', 'Color': '255; 120; 0',
        'Alpha': 1, 'Arrow Length': 0.6, 'Head Length': 0.15, 'Head Radius': 0.12,
        'Shaft Length': 0.45, 'Shaft Radius': 0.06})
    if nav2:
        first = ids[0]
        displays += [
            {'Class': 'rviz_common/Group', 'Name': 'Nav2: AMCL particles', 'Enabled': True,
             'Displays': particles},
            {'Class': 'rviz_common/Group', 'Name': 'Nav2: paths sent to FollowPath',
             'Enabled': False, 'Displays': plans},
            {'Class': 'rviz_common/Group', 'Name': 'Nav2: collision monitor zones',
             'Enabled': True, 'Displays': safety},
            {'Class': 'rviz_default_plugins/Map', 'Name': f'Nav2: local costmap ({first})',
             'Enabled': True, 'Alpha': 0.6, 'Color Scheme': 'costmap', 'Draw Behind': False,
             'Topic': _topic(f'/{first}/local_costmap/costmap'),
             'Update Topic': _topic(f'/{first}/local_costmap/costmap_updates')},
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
                            'Distance': 96.0, 'Pitch': 1.05, 'Yaw': 4.71,
                            'Focal Point': {'X': center[0], 'Y': center[1] - 4.0, 'Z': 0.0},
                            'Field of View': 0.785, 'Near Clip Distance': 0.5,
                            'Target Frame': '<Fixed Frame>', 'Value': 'Orbit (rviz)'},
                'Saved': [
                    {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Top down',
                     'Scale': 11.0, 'X': center[0], 'Y': center[1], 'Angle': 0,
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
    ap.add_argument('--nav2', action='store_true', help='AMCL particles, safety zones, costmap')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    ids = [f'amr_{i + 1:02d}' for i in range(a.robots)]
    with open(a.out, 'w') as f:
        f.write(make_rviz_config(ids, a.camera_robot, nav2=a.nav2))
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
