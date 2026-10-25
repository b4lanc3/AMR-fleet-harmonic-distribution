"""Show one AMR in RViz (no simulation):  ros2 launch amr_description view_robot.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('amr_description')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'amr.urdf.xacro'),
                              mappings={'robot_name': 'amr_01', 'gazebo': 'false'}).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'view_robot.rviz')]),
    ])
