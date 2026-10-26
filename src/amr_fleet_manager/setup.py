import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_fleet_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('amr_fleet_manager/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMR Fleet Team',
    maintainer_email='tranbinhminh615@gmail.com',
    description='AMR fleet manager, traffic control, robot agents and dashboard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager = amr_fleet_manager.nodes.fleet_manager_node:main',
            'robot_agent = amr_fleet_manager.nodes.robot_agent_node:main',
            'fleet_agents = amr_fleet_manager.nodes.robot_agent_node:main_fleet',
            'kinematic_sim = amr_fleet_manager.nodes.kinematic_sim_node:main',
            'dashboard = amr_fleet_manager.nodes.dashboard_node:main',
            'fleet_cli = amr_fleet_manager.tools.fleet_cli:main',
            'headless_sim = amr_fleet_manager.sim.headless:main',
            'tune_sim = amr_fleet_manager.sim.tune:main',
            'generate_world = amr_fleet_manager.tools.generate_world:main',
            'rviz_config = amr_fleet_manager.tools.rviz_config:main',
        ],
    },
)
