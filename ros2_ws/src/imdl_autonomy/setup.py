from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'imdl_autonomy'

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
        (os.path.join('share', package_name, 'config', 'missions'),
         glob('config/missions/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member B',
    maintainer_email='todo@todo.com',
    description='Nav2 based mission autonomy for the IMDL Create3 tasks.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_obstacle_projector = '
            'imdl_autonomy.depth_obstacle_projector:main',
            'generate_maps = imdl_autonomy.map_from_world:main',
            'mission_manager = imdl_autonomy.mission_manager:main',
            'object_registry = imdl_autonomy.object_registry:main',
            'prepare_trial = imdl_autonomy.real_trial:main',
            'prepare_real_trial = imdl_autonomy.real_trial:main',
            'localization_guard = imdl_autonomy.localization_guard:main',
            'safety_monitor = imdl_autonomy.safety_monitor:main',
            'simulation_manipulator = imdl_autonomy.simulation_manipulator:main',
        ],
    },
)
