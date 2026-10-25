from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*')),
        (os.path.join('share', package_name, 'docs'),
            glob('docs/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Member A',
    maintainer_email='todo@todo.com',
    description='Perception package for IMDL project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_align = perception.depth_align:main',
            'hsv_segmentation = perception.hsv_segmentation_node:main',
            'landmark_detector = perception.landmark_detector_node:main',
            'yolo_segmentation = perception.yolo_segmentation_node:main',
            'target_pose_estimator = perception.target_pose_estimator:main',
            'target_selector = perception.target_selector:main',
            'beacon_heading_controller = perception.beacon_heading_controller:main',
            'perception_visualizer = perception.perception_visualizer:main',
        ],
    },
)
