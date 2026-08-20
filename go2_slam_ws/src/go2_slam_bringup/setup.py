import os
from setuptools import setup

package_name = 'go2_slam_bringup'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
# launch 文件
launch_dir = os.path.join(os.path.dirname(__file__), 'launch')
data_files.append(('share/' + package_name + '/launch',
                   [os.path.join(launch_dir, f) for f in os.listdir(launch_dir)
                    if f.endswith('.launch.py')]))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@localhost',
    description='GO2-W SLAM bringup',
    license='Apache-2.0',
)
