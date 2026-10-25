"""Complete AMR fleet simulation.

  # Gazebo Harmonic + RViz + dashboard (http://localhost:8080)
  ros2 launch amr_bringup bringup.launch.py num_robots:=12

  # same fleet software on the light kinematic simulator (no Gazebo)
  ros2 launch amr_bringup bringup.launch.py sim:=kinematic num_robots:=15
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from amr_fleet_manager.core.colors import robot_rgba_str
from amr_fleet_manager.sim.headless import robot_name
from amr_fleet_manager.tools.rviz_config import make_rviz_config


def rsp_container(robots):
    """All robot_state_publishers of the fleet in one component container."""
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    return ComposableNodeContainer(
        name='fleet_state_publishers', namespace='', package='rclcpp_components',
        executable='component_container', output='log',
        composable_node_descriptions=[ComposableNode(
            package='robot_state_publisher', plugin='robot_state_publisher::RobotStatePublisher',
            name='robot_state_publisher', namespace=rid,
            parameters=[{'robot_description': urdf, 'frame_prefix': f'{rid}/'}])
            for rid, urdf in robots])


def launch_setup(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    sim = lc('sim')
    n = int(lc('num_robots'))
    if not 1 <= n <= 22:
        raise RuntimeError('num_robots must be between 1 and 22 (parking bays)')
    gazebo = sim == 'gazebo'
    sim_time = 'true' if gazebo else 'false'
    share = get_package_share_directory('amr_bringup')
    layout = lc('layout_file') or os.path.join(share, 'config', 'factory_layout.yaml')
    ids = [robot_name(i) for i in range(n)]
    actions = []

    if gazebo:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('amr_gazebo'), 'launch', 'gazebo_fleet.launch.py')),
            launch_arguments={
                'num_robots': str(n), 'layout_file': layout, 'gui': lc('gui'),
                'camera': lc('camera'), 'bridge_cameras': lc('bridge_cameras'),
                'world': lc('world')}.items()))
    elif sim == 'kinematic':
        actions.append(Node(package='amr_fleet_manager', executable='kinematic_sim',
                            name='kinematic_sim', output='screen',
                            parameters=[{'layout_file': layout, 'num_robots': n}]))
        xacro_file = os.path.join(get_package_share_directory('amr_description'), 'urdf',
                                  'amr.urdf.xacro')
        robots = [(rid, xacro.process_file(xacro_file, mappings={
            'robot_name': rid, 'color': robot_rgba_str(i), 'gazebo': 'false'}).toxml())
            for i, rid in enumerate(ids)]
        actions.append(rsp_container(robots))
    else:
        raise RuntimeError(f'unknown sim "{sim}" (gazebo | kinematic)')

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'fleet.launch.py')),
        launch_arguments={'num_robots': str(n), 'use_sim_time': sim_time, 'layout_file': layout,
                          'priority_mode': lc('priority_mode'), 'dashboard': lc('dashboard'),
                          'production': lc('production'), 'agents': lc('agents'),
                          'dashboard_port': lc('dashboard_port')}.items()))

    if lc('rviz').lower() == 'true':
        tmp = os.path.join(tempfile.gettempdir(), 'amr_fleet')
        os.makedirs(tmp, exist_ok=True)
        cfg = os.path.join(tmp, f'fleet_{n}.rviz')
        cam = ids[0] if gazebo and lc('camera') == 'true' and lc('bridge_cameras') != 'none' else ''
        with open(cfg, 'w') as f:
            f.write(make_rviz_config(ids, cam))
        actions.append(TimerAction(period=2.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', cfg], parameters=[{'use_sim_time': gazebo}])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='gazebo', description='gazebo | kinematic'),
        DeclareLaunchArgument('num_robots', default_value='12', description='10..15 recommended'),
        DeclareLaunchArgument('layout_file', default_value=''),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true', description='RGB-D camera sensors'),
        DeclareLaunchArgument('bridge_cameras', default_value='first',
                              description='none | first | all  (camera topics bridged to ROS)'),
        DeclareLaunchArgument('priority_mode', default_value='dynamic', description='dynamic | static'),
        DeclareLaunchArgument('production', default_value='true',
                              description='false: robots only run manual tasks / the demo'),
        DeclareLaunchArgument('agents', default_value='single_process',
                              description='single_process (simulation) | per_robot (like real AMRs)'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        OpaqueFunction(function=launch_setup),
    ])
