import os
from setuptools import setup

package_name = 'go2_slam_web'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
# 前端页面
static_dir = os.path.join(os.path.dirname(__file__), 'go2_slam_web', 'static')
for root, dirs, files in os.walk(static_dir):
    rel = os.path.relpath(root, os.path.dirname(__file__))
    data_files.append(('share/' + package_name + '/' + rel, [os.path.join(root, f) for f in files]))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'tornado'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@go2.local',
    description='GO2-W SLAM web console',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_server = go2_slam_web.web_server:main',
            'chassis_safety_gate = go2_slam_web.chassis_safety_gate:main',
        ],
    },
)
