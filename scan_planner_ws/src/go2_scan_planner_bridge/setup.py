import os
from glob import glob

from setuptools import setup

package_name = 'go2_scan_planner_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@go2.local',
    description='GO2-W LIO-SAM adapter for SCAN-Planner',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lio_pose_adapter = go2_scan_planner_bridge.lio_pose_adapter:main',
            'forward_test_goal = go2_scan_planner_bridge.forward_test_goal:main',
            'static_navigation_map = go2_scan_planner_bridge.static_navigation_map:main',
            'realtime_recovery = go2_scan_planner_bridge.realtime_recovery:main',
        ],
    },
)
