"""Gazebo Harmonic (or Jetty) factory + N AMRs + one ros_gz_bridge for the fleet.

  ros2 launch amr_gazebo gazebo_fleet.launch.py num_robots:=12

The world is regenerated from factory_layout.yaml at every start (so the
traffic graph, the 3D world and the AMCL map always match); pass
world:=<file.sdf> to use a fixed one.  Robots spawn on the parking bays P01..Pn.

odom:=wheel        DiffDrive wheel odometry on /<id>/odom (+ odom -> base_footprint
                   TF); AMCL provides map -> odom.  The exact pose is kept on
                   /<id>/ground_truth for monitoring.
odom:=ground_truth the exact pose is the odometry (map aligned, no AMCL needed).
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_manager.core.colors import robot_rgba_str
from amr_fleet_manager.core.layout import build_graph, load_layout
from amr_fleet_manager.sim.headless import robot_name, spawn_poses
from amr_fleet_manager.tools.generate_world import write_world_and_map


def _bridge_config(ids, cameras, points, ground_truth):
    def entry(ros, gz, ros_t, gz_t, direction, **extra):
        e = {'ros_topic_name': ros, 'gz_topic_name': gz, 'ros_type_name': ros_t,
             'gz_type_name': gz_t, 'direction': direction}
        e.update(extra)
        return e

    cfg = [entry('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
           # MES line state -> conveyors of the LineAnimator system
           entry('/factory/line_state', '/factory/line_state', 'std_msgs/msg/String',
                 'gz.msgs.StringMsg', 'ROS_TO_GZ'),
           # walking workers (Pedestrians system) -> dashboard / RViz
           entry('/factory/people', '/factory/people', 'geometry_msgs/msg/PoseArray',
                 'gz.msgs.Pose_V', 'GZ_TO_ROS')]
    for i, rid in enumerate(ids):
        cfg += [
            entry(f'/{rid}/cmd_vel', f'/{rid}/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
            entry(f'/{rid}/cargo_lift', f'/{rid}/cargo_lift', 'std_msgs/msg/Float64',
                  'gz.msgs.Double', 'ROS_TO_GZ'),
            entry(f'/{rid}/odom', f'/{rid}/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
            entry('/tf', f'/{rid}/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS'),
            entry(f'/{rid}/joint_states', f'/{rid}/joint_states', 'sensor_msgs/msg/JointState',
                  'gz.msgs.Model', 'GZ_TO_ROS'),
            entry(f'/{rid}/scan', f'/{rid}/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan',
                  'GZ_TO_ROS'),
        ]
        if ground_truth:
            cfg.append(entry(f'/{rid}/ground_truth', f'/{rid}/ground_truth',
                             'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'))
        images = cameras == 'all' or (cameras == 'first' and i == 0)
        if images:
            cfg += [
                entry(f'/{rid}/camera/image', f'/{rid}/camera/image', 'sensor_msgs/msg/Image',
                      'gz.msgs.Image', 'GZ_TO_ROS'),
                entry(f'/{rid}/camera/depth_image', f'/{rid}/camera/depth_image',
                      'sensor_msgs/msg/Image', 'gz.msgs.Image', 'GZ_TO_ROS'),
                entry(f'/{rid}/camera/camera_info', f'/{rid}/camera/camera_info',
                      'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', 'GZ_TO_ROS'),
            ]
        if images or points == 'all':
            cfg.append(entry(f'/{rid}/camera/points', f'/{rid}/camera/points',
                             'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked',
                             'GZ_TO_ROS'))
    return cfg


def rsp_container(name, robots, use_sim_time):
    """All robot_state_publishers of the fleet in one component container."""
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    return ComposableNodeContainer(
        name=name, namespace='', package='rclcpp_components', executable='component_container',
        output='log',
        composable_node_descriptions=[ComposableNode(
            package='robot_state_publisher', plugin='robot_state_publisher::RobotStatePublisher',
            name='robot_state_publisher', namespace=rid,
            parameters=[{'robot_description': urdf, 'frame_prefix': f'{rid}/',
                         'use_sim_time': use_sim_time}]) for rid, urdf in robots])


def launch_setup(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    n = int(lc('num_robots'))
    layout_file = lc('layout_file') or os.path.join(
        get_package_share_directory('amr_bringup'), 'config', 'factory_layout.yaml')
    layout = load_layout(layout_file)
    graph = build_graph(layout)
    poses = spawn_poses(graph, n)
    ids = [robot_name(i) for i in range(n)]
    tmp = os.path.join(tempfile.gettempdir(), 'amr_fleet')
    os.makedirs(tmp, exist_ok=True)

    world = lc('world')
    if not world:
        world = os.path.join(tmp, 'factory.sdf')
        write_world_and_map(layout, world, os.path.join(tmp, 'factory_map'),
                            step=float(lc('physics_step')),
                            people=lc('people').lower() == 'true')
    wheel_odom = lc('odom') == 'wheel'

    # server only: a GUI started together with the server is still loading the
    # large factory scene while the robots are spawned and misses most of them
    # (only one AMR rendered); the GUI client joins after the last spawn and
    # receives the complete scene
    gui = lc('gui').lower() == 'true'
    gz_args = f'-s -r -v {lc("gz_verbosity")} {world}'
    if not gui and lc('headless_rendering').lower() == 'true':
        gz_args = '--headless-rendering ' + gz_args
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                             'gz_sim.launch.py')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())]
    if gui:
        actions.append(TimerAction(
            period=3.0 + float(lc('spawn_delay')) * n + 4.0, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={'gz_args': f'-g -v {lc("gz_verbosity")}'}.items())]))

    cameras = lc('bridge_cameras')
    bridge_file = os.path.join(tmp, 'bridge.yaml')
    with open(bridge_file, 'w') as f:
        yaml.safe_dump(_bridge_config(ids, cameras, lc('bridge_points'), wheel_odom), f)
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='fleet_bridge',
                        output='screen',
                        parameters=[{'config_file': bridge_file, 'use_sim_time': True}]))

    xacro_file = os.path.join(get_package_share_directory('amr_description'), 'urdf',
                              'amr.urdf.xacro')
    delay = float(lc('spawn_delay'))
    descriptions = []
    for i, rid in enumerate(ids):
        descriptions.append((rid, xacro.process_file(xacro_file, mappings={
            'robot_name': rid, 'color': robot_rgba_str(i), 'gazebo': 'true',
            'camera': lc('camera'), 'camera_rate': lc('camera_rate'),
            'lidar_rate': lc('lidar_rate'), 'odom': lc('odom')}).toxml()))
    actions.append(rsp_container('fleet_state_publishers', descriptions, True))
    for i, (rid, (x, y, yaw)) in enumerate(zip(ids, poses)):
        spawn = Node(package='ros_gz_sim', executable='create', output='log',
                     arguments=['-world', 'amr_factory', '-name', rid,
                                '-topic', f'/{rid}/robot_description',
                                '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', '0.01',
                                '-Y', f'{yaw:.4f}'])
        actions.append(TimerAction(period=3.0 + delay * i, actions=[spawn]))
    return actions


def generate_launch_description():
    share_models = os.path.join(get_package_share_directory('amr_gazebo'), 'models')
    plugins = os.path.join(get_package_prefix('amr_gazebo'), 'lib', 'amr_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='12', description='number of AMRs (<= 22)'),
        DeclareLaunchArgument('layout_file', default_value='', description='factory_layout.yaml'),
        DeclareLaunchArgument('world', default_value='', description='use this SDF instead of generating one'),
        DeclareLaunchArgument('gui', default_value='true', description='start the Gazebo GUI'),
        DeclareLaunchArgument('headless_rendering', default_value='true',
                              description='EGL headless rendering when gui:=false (sensors still work)'),
        DeclareLaunchArgument('camera', default_value='true', description='simulate the RGB-D camera'),
        DeclareLaunchArgument('camera_rate', default_value='5'),
        DeclareLaunchArgument('lidar_rate', default_value='10'),
        DeclareLaunchArgument('bridge_cameras', default_value='first',
                              description='bridge camera images to ROS: none | first | all'),
        DeclareLaunchArgument('bridge_points', default_value='all',
                              description='bridge the RGB-D point clouds: all | images '
                                          '(only robots whose images are bridged)'),
        DeclareLaunchArgument('people', default_value='true',
                              description='workers walking in the factory'),
        DeclareLaunchArgument('odom', default_value='wheel',
                              description='wheel (AMCL / Nav2) | ground_truth'),
        DeclareLaunchArgument('physics_step', default_value='0.004'),
        DeclareLaunchArgument('spawn_delay', default_value='0.8', description='s between robot spawns'),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_models + os.pathsep
                               + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        # LineAnimator system (moving products on the conveyors)
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugins + os.pathsep
                               + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        OpaqueFunction(function=launch_setup),
    ])
