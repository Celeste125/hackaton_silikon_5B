from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'imdl_statemachine'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lennart Troesken',
    maintainer_email='lennart.troesken@tum.de',
    description='Waypoint state machine on top of Nav2 and a saved SLAM map.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'state_machine = imdl_statemachine.state_machine:main',
            'aruco_detector = imdl_statemachine.aruco_detector:main',
            'fake_markers = imdl_statemachine.fake_markers:main',
            'fake_robot = imdl_statemachine.fake_robot:main',
        ],
    },
)
