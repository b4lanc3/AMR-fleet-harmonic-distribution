"""Gazebo Harmonic (or Jetty) factory + N AMRs + one ros_gz_bridge for the fleet.

  ros2 launch amr_gazebo gazebo_fleet.launch.py num_robots:=12

The world is regenerated from factory_layout.yaml at every start (so the
traffic graph and the 3D world always match); pass world:=<file.sdf> to use a
fixed one.  Robots spawn on the parking bays P01..Pn.
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_manager.core.colors import robot_rgba_str
from amr_fleet_manager.core.layout import build_graph, load_layout
from amr_fleet_manager.sim.headless import robot_name, spawn_poses
from amr_fleet_manager.tools.generate_world import world_sdf


def _bridge_config(ids, cameras):
    def entry(ros, gz, ros_t, gz_t, direction, **extra):
        e = {'ros_topic_name': ros, 'gz_topic_name': gz, 'ros_type_name': ros_t,
             'gz_type_name': gz_t, 'direction': direction}
        e.update(extra)
        return e

    cfg = [entry('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS')]
    for i, rid in enumerate(ids):
        cfg += [
            entry(f'/{rid}/cmd_vel', f'/{rid}/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
            entry(f'/{rid}/odom', f'/{rid}/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
            entry('/tf', f'/{rid}/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS'),
            entry(f'/{rid}/joint_states', f'/{rid}/joint_states', 'sensor_msgs/msg/JointState',
                  'gz.msgs.Model', 'GZ_TO_ROS'),
            entry(f'/{rid}/scan', f'/{rid}/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan',
                  'GZ_TO_ROS'),
        ]
        if cameras == 'all' or (cameras == 'first' and i == 0):
            cfg += [
                entry(f'/{rid}/camera/image', f'/{rid}/camera/image', 'sensor_msgs/msg/Image',
                      'gz.msgs.Image', 'GZ_TO_ROS'),
                entry(f'/{rid}/camera/depth_image', f'/{rid}/camera/depth_image',
                      'sensor_msgs/msg/Image', 'gz.msgs.Image', 'GZ_TO_ROS'),
                entry(f'/{rid}/camera/points', f'/{rid}/camera/points',
                      'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', 'GZ_TO_ROS'),
                entry(f'/{rid}/camera/camera_info', f'/{rid}/camera/camera_info',
                      'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', 'GZ_TO_ROS'),
            ]
    return cfg


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
        with open(world, 'w', encoding='utf-8') as f:
            f.write(world_sdf(layout, step=float(lc('physics_step'))))

    gz_args = f'-r -v {lc("gz_verbosity")} {world}'
    if lc('gui').lower() != 'true':
        gz_args = '-s ' + gz_args
        if lc('headless_rendering').lower() == 'true':
            gz_args = '--headless-rendering ' + gz_args
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())]

    cameras = lc('bridge_cameras')
    bridge_file = os.path.join(tmp, 'bridge.yaml')
    with open(bridge_file, 'w') as f:
        yaml.safe_dump(_bridge_config(ids, cameras), f)
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='fleet_bridge',
                        output='screen',
                        parameters=[{'config_file': bridge_file, 'use_sim_time': True}]))

    xacro_file = os.path.join(get_package_share_directory('amr_description'), 'urdf',
                              'amr.urdf.xacro')
    delay = float(lc('spawn_delay'))
    for i, (rid, (x, y, yaw)) in enumerate(zip(ids, poses)):
        urdf = xacro.process_file(xacro_file, mappings={
            'robot_name': rid, 'color': robot_rgba_str(i), 'gazebo': 'true',
            'camera': lc('camera'), 'camera_rate': lc('camera_rate'),
            'lidar_rate': lc('lidar_rate')}).toxml()
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=rid, name='robot_state_publisher', output='log',
                            parameters=[{'robot_description': urdf, 'frame_prefix': f'{rid}/',
                                         'use_sim_time': True}]))
        spawn = Node(package='ros_gz_sim', executable='create', output='log',
                     arguments=['-world', 'amr_factory', '-name', rid,
                                '-topic', f'/{rid}/robot_description',
                                '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', '0.01',
                                '-Y', f'{yaw:.4f}'])
        actions.append(TimerAction(period=3.0 + delay * i, actions=[spawn]))
    return actions


def generate_launch_description():
    share_models = os.path.join(get_package_share_directory('amr_gazebo'), 'models')
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
                              description='bridge camera topics to ROS: none | first | all'),
        DeclareLaunchArgument('physics_step', default_value='0.004'),
        DeclareLaunchArgument('spawn_delay', default_value='0.8', description='s between robot spawns'),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_models + os.pathsep
                               + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        OpaqueFunction(function=launch_setup),
    ])
