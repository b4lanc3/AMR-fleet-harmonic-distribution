"""Fleet software only: fleet manager, one agent per robot, web dashboard.

Use it on top of any simulator (Gazebo or the kinematic one) or real robots
that provide /<id>/odom, /<id>/scan and accept /<id>/cmd_vel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_manager.sim.headless import initial_battery, robot_name


def launch_setup(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    n = int(lc('num_robots'))
    sim_time = lc('use_sim_time').lower() == 'true'
    share = get_package_share_directory('amr_bringup')
    params = os.path.join(share, 'config', 'fleet_params.yaml')
    layout = lc('layout_file') or os.path.join(share, 'config', 'factory_layout.yaml')
    actions = [Node(
        package='amr_fleet_manager', executable='fleet_manager', name='fleet_manager',
        output='screen',
        parameters=[params, {'layout_file': layout, 'num_robots': n,
                             'priority_mode': lc('priority_mode'),
                             'production': lc('production').lower() == 'true',
                             'use_sim_time': sim_time}])]
    for i in range(n):
        rid = robot_name(i)
        actions.append(Node(
            package='amr_fleet_manager', executable='robot_agent', name='robot_agent',
            namespace=rid, output='log',
            parameters=[params, {'robot_id': rid, 'initial_battery': initial_battery(i),
                                 'use_sim_time': sim_time}]))
    if lc('dashboard').lower() == 'true':
        actions.append(Node(
            package='amr_fleet_manager', executable='dashboard', name='fleet_dashboard',
            output='screen', parameters=[{'port': int(lc('dashboard_port')),
                                          'use_sim_time': sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='12'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('layout_file', default_value=''),
        DeclareLaunchArgument('priority_mode', default_value='dynamic',
                              description='dynamic | static'),
        DeclareLaunchArgument('production', default_value='true',
                              description='false: no MES tasks (manual tasks / demo only)'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        OpaqueFunction(function=launch_setup),
    ])
