from setuptools import setup

package_name = 'go2_slam_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@go2.local',
    description='GO2-W SLAM core: unitree built-in slam RPC control + map relay + fallback python 2D SLAM',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_manager = go2_slam_core.slam_manager:main',
            'fallback_slam = go2_slam_core.fallback_slam:main',
            'imu_attitude_bridge = go2_slam_core.imu_attitude_bridge:main',
        ],
    },
)
