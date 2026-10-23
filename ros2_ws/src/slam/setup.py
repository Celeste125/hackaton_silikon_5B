from setuptools import setup
import os
from glob import glob

package_name = 'slam'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Member A',
    maintainer_email='todo@todo.com',
    description='SLAM package for IMDL project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_republisher = slam.odom_tf_republisher:main',
            'scan_timestamp_fixer = slam.scan_timestamp_fixer:main',
        ],
    },
)
