"""Nav2 + AMCL for every AMR of the fleet (on top of Gazebo).

  ros2 launch amr_bringup navigation.launch.py num_robots:=12

* one global map_server (+ lifecycle manager) serves /map: the lidar-plane
  slice of the generated Gazebo world, so AMCL sees exactly the walls, racks,
  machines and conveyors the lidar sees
* per robot, in its namespace and its own component container:
    amcl               localisation on /map, publishes map -> <id>/odom
    controller_server  Regulated Pure Pursuit + local costmap (2D lidar + RGB-D
                       point cloud), FollowPath action, cmd_vel -> cmd_vel_nav
    collision_monitor  lidar + RGB-D stop / slow-down zones, cmd_vel_nav -> cmd_vel
    docking_server     precise docking on the chargers (opennav_docking), dock pose
                       detected from the lidar by the robot agent
    lifecycle_manager_navigation
  full_stack:=true adds planner_server, behavior_server and bt_navigator
  (NavigateToPose for manual tests; the fleet does not need them).

The robot agents (driver:=nav2) send the lane route granted by the fleet
manager's traffic control to FollowPath, so Nav2 drives and localises while
the fleet manager keeps the traffic rules (one-way lanes, reservations,
yielding by task priority).
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from amr_fleet_manager.core.docking import docked_pose
from amr_fleet_manager.core.layout import build_graph, load_layout
from amr_fleet_manager.sim.headless import robot_name, spawn_poses
from amr_fleet_manager.tools.generate_world import write_factory_map

CORE = ['amcl', 'controller_server', 'collision_monitor', 'docking_server']
FULL = ['planner_server', 'behavior_server', 'bt_navigator']


def charger_docks(graph) -> dict:
    """Dock database of the docking server: docked base pose of every charger."""
    docks = {}
    for cid, i in sorted(graph.chargers.items()):
        n = graph.nodes[i]
        x, y, yaw = docked_pose((n.x, n.y), n.heading)
        docks[cid] = {'type': 'charger', 'frame': 'map',
                      'pose': [round(x, 3), round(y, 3), round(yaw, 4)]}
    return docks


def robot_params(template: str, rid: str, pose, camera: bool, full_stack: bool,
                 docks: dict = None) -> dict:
    """Nav2 parameters of one robot from the nav2_amr.yaml template."""
    data = yaml.safe_load(template.replace('ROBOT', rid))
    x, y, yaw = pose
    amcl = data[f'/{rid}/amcl']['ros__parameters']
    amcl['initial_pose'] = {'x': float(x), 'y': float(y), 'z': 0.0, 'yaw': float(yaw)}
    dock = data[f'/{rid}/docking_server']['ros__parameters']
    dock['docks'] = list((docks or {}).keys())
    dock.update(docks or {})
    data[f'/{rid}/lifecycle_manager_navigation']['ros__parameters']['node_names'] = \
        CORE + (FULL if full_stack else [])
    if not camera:
        lc = data[f'/{rid}/local_costmap/local_costmap']['ros__parameters']['obstacle_layer']
        lc['observation_sources'] = 'scan'
        lc.pop('points', None)
        cm = data[f'/{rid}/collision_monitor']['ros__parameters']
        cm['observation_sources'] = ['scan']
        cm.pop('points', None)
    if not full_stack:
        for k in ('planner_server', 'global_costmap/global_costmap', 'behavior_server',
                  'bt_navigator'):
            data.pop(f'/{rid}/{k}', None)
    return data


def humble_collision_monitor(cm: dict) -> None:
    """Collision monitor parameters in the Humble (Nav2 1.1) format: polygon points
    as a flat [x0, y0, x1, y1, ...] array, `max_points` (points allowed inside the
    zone) instead of `min_points`, one global source_timeout, no state topic."""
    cm.pop('state_topic', None)
    for name in cm.get('polygons', []):
        poly = cm[name]
        if isinstance(poly.get('points'), str):
            poly['points'] = [float(v) for xy in yaml.safe_load(poly['points']) for v in xy]
        if 'min_points' in poly:
            poly['max_points'] = int(poly.pop('min_points')) - 1
    for name in cm.get('observation_sources', []):
        cm.get(name, {}).pop('source_timeout', None)


def robot_navigation(rid: str, params_file: str, full_stack: bool, log_level: str):
    nodes = [
        ComposableNode(package='nav2_amcl', plugin='nav2_amcl::AmclNode', name='amcl',
                       namespace=rid, parameters=[params_file]),
        ComposableNode(package='nav2_controller', plugin='nav2_controller::ControllerServer',
                       name='controller_server', namespace=rid, parameters=[params_file],
                       remappings=[('cmd_vel', 'cmd_vel_nav')]),
        ComposableNode(package='nav2_collision_monitor',
                       plugin='nav2_collision_monitor::CollisionMonitor',
                       name='collision_monitor', namespace=rid, parameters=[params_file]),
        ComposableNode(package='opennav_docking', plugin='opennav_docking::DockingServer',
                       name='docking_server', namespace=rid, parameters=[params_file]),
    ]
    if full_stack:
        nodes += [
            ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer',
                           name='planner_server', namespace=rid, parameters=[params_file]),
            ComposableNode(package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
                           name='behavior_server', namespace=rid, parameters=[params_file],
                           remappings=[('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
                           name='bt_navigator', namespace=rid, parameters=[params_file]),
        ]
    nodes.append(ComposableNode(
        package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation', namespace=rid, parameters=[params_file]))
    # the costmaps are child nodes created inside controller_server / planner_server:
    # they read their parameters from the process-wide --params-file
    return ComposableNodeContainer(
        name='nav2_container', namespace=rid, package='rclcpp_components',
        executable='component_container_isolated', output='log',
        arguments=['--ros-args', '--params-file', params_file, '--log-level', log_level],
        composable_node_descriptions=nodes)


def launch_setup(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    n = int(lc('num_robots'))
    share = get_package_share_directory('amr_bringup')
    layout_file = lc('layout_file') or os.path.join(share, 'config', 'factory_layout.yaml')
    layout = load_layout(layout_file)
    graph = build_graph(layout)
    poses = spawn_poses(graph, n)
    docks = charger_docks(graph)
    tmp = os.path.join(tempfile.gettempdir(), 'amr_fleet')
    os.makedirs(tmp, exist_ok=True)
    sim_time = lc('use_sim_time').lower() == 'true'
    full = lc('full_stack').lower() == 'true'
    camera = lc('camera').lower() == 'true'

    map_yaml = lc('map')
    if not map_yaml:
        map_yaml = write_factory_map(layout, os.path.join(tmp, 'factory_map'))
    actions = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='log', parameters=[{'yaml_filename': map_yaml, 'topic_name': 'map',
                                        'frame_id': 'map', 'use_sim_time': sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='log',
             parameters=[{'autostart': True, 'node_names': ['map_server'],
                          'use_sim_time': sim_time}]),
    ]

    with open(os.path.join(share, 'config', 'nav2_amr.yaml'), encoding='utf-8') as f:
        template = f.read()
    delay, stagger = float(lc('start_delay')), float(lc('stagger'))
    for i, pose in enumerate(poses):
        rid = robot_name(i)
        data = robot_params(template, rid, pose, camera, full, docks)
        if os.environ.get('ROS_DISTRO') == 'humble':
            humble_collision_monitor(data[f'/{rid}/collision_monitor']['ros__parameters'])
        if not sim_time:
            for v in data.values():
                v['ros__parameters']['use_sim_time'] = False
        params_file = os.path.join(tmp, f'nav2_{rid}.yaml')
        with open(params_file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        actions.append(TimerAction(period=delay + stagger * i, actions=[
            robot_navigation(rid, params_file, full, lc('log_level'))]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='12'),
        DeclareLaunchArgument('layout_file', default_value=''),
        DeclareLaunchArgument('map', default_value='',
                              description='map yaml (default: generated from the layout)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='RGB-D point cloud in the costmap / collision monitor'),
        DeclareLaunchArgument('full_stack', default_value='false',
                              description='also planner, behaviors and bt_navigator per robot'),
        DeclareLaunchArgument('start_delay', default_value='6.0',
                              description='s before the first robot stack (robots must be spawned)'),
        DeclareLaunchArgument('stagger', default_value='1.0', description='s between robots'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        OpaqueFunction(function=launch_setup),
    ])
