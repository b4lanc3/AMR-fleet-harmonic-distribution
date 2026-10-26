"""Fleet software only: fleet manager, one agent per robot, web dashboard.

Use it on top of any simulator (Gazebo or the kinematic one) or real robots.
driver:=simple  robots provide map-aligned /<id>/odom, /<id>/scan and accept
                /<id>/cmd_vel; the fleet manager serves /map and map -> odom.
driver:=nav2    every robot runs AMCL + Nav2 (navigation.launch.py); the agents
                send the granted lane route to FollowPath.
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
    driver = lc('driver')
    nav2 = driver == 'nav2'
    share = get_package_share_directory('amr_bringup')
    params = os.path.join(share, 'config', 'fleet_params.yaml')
    layout = lc('layout_file') or os.path.join(share, 'config', 'factory_layout.yaml')
    actions = [Node(
        package='amr_fleet_manager', executable='fleet_manager', name='fleet_manager',
        output='screen',
        parameters=[params, {'layout_file': layout, 'num_robots': n,
                             'priority_mode': lc('priority_mode'),
                             'production': lc('production').lower() == 'true',
                             'station_handover': lc('station_handover').lower() == 'true',
                             'publish_map': not nav2, 'publish_odom_tf': not nav2,
                             'use_sim_time': sim_time}])]
    if lc('agents') == 'per_robot':
        # one process per robot, like on real AMRs
        for i in range(n):
            rid = robot_name(i)
            actions.append(Node(
                package='amr_fleet_manager', executable='robot_agent', name='robot_agent',
                namespace=rid, output='log',
                parameters=[params, {'robot_id': rid, 'initial_battery': initial_battery(i),
                                     'driver': driver, 'use_sim_time': sim_time}]))
    else:
        # all agents in one process: one DDS participant, much less CPU in simulation.
        # No name= here: a node-name remap would rename every agent of the process and
        # the /**/robot_agent section of fleet_params.yaml would no longer apply.
        actions.append(Node(
            package='amr_fleet_manager', executable='fleet_agents',
            output='screen',
            parameters=[params, {'num_robots': n,
                                 'initial_batteries': [initial_battery(i) for i in range(n)],
                                 'driver': driver, 'use_sim_time': sim_time}]))
    if lc('dashboard').lower() == 'true':
        actions.append(Node(
            package='amr_fleet_manager', executable='dashboard', name='fleet_dashboard',
            output='screen', parameters=[{'port': int(lc('dashboard_port')),
                                          'host': lc('dashboard_host'),
                                          'use_sim_time': sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='12'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('layout_file', default_value=''),
        DeclareLaunchArgument('priority_mode', default_value='task',
                              description='task | dynamic | static'),
        DeclareLaunchArgument('production', default_value='true',
                              description='false: no MES tasks (manual tasks / demo only)'),
        DeclareLaunchArgument('agents', default_value='single_process',
                              description='single_process | per_robot'),
        DeclareLaunchArgument('driver', default_value='simple', description='simple | nav2'),
        DeclareLaunchArgument('station_handover', default_value='false',
                              description='loading / unloading ends when the Gazebo station arm '
                                          'hands over (true in Gazebo)'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        DeclareLaunchArgument('dashboard_host', default_value='0.0.0.0',
                              description='127.0.0.1: dashboard (which can create tasks and '
                                          'pause the fleet) only reachable from this machine'),
        OpaqueFunction(function=launch_setup),
    ])
