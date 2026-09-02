import os

from setuptools import setup


package_name = 'go2_remembr'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         [os.path.join(package_name, 'default_config.json')]),
    ],
    package_data={package_name: ['default_config.json']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@go2.local',
    description='Edge-side semantic memory for GO2-W navigation',
    license='Apache-2.0',
)
