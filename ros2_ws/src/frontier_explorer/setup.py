from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'frontier_explorer'

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
    maintainer='Team 2',
    maintainer_email='todo@todo.todo',
    description='Lidar bring-up, live SLAM + Nav2 and frontier exploration for the Create 3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = frontier_explorer.frontier_explorer:main',
            'scan_timestamp_fixer = frontier_explorer.scan_timestamp_fixer:main',
            'odom_tf_republisher = frontier_explorer.odom_tf_republisher:main',
        ],
    },
)
