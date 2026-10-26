from glob import glob

from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 2',
    maintainer_email='todo@todo.todo',
    description='Launch files that start the robot: drivers, cameras, lidar, web panel.',
    license='Apache-2.0',
)
