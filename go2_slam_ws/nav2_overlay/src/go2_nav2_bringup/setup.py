from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'go2_nav2_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description='Shadow-safe Nav2 integration for the Unitree GO2-W.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_odom_bridge = go2_nav2_bringup.map_odom_bridge:main',
            'nav_goal_bridge = go2_nav2_bringup.nav_goal_bridge:main',
            'shadow_monitor = go2_nav2_bringup.shadow_monitor:main',
        ],
    },
)
