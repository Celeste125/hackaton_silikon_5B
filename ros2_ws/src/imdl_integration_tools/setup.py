import os
from glob import glob

from setuptools import setup


package_name = 'imdl_integration_tools'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='IMDL Team',
    maintainer_email='todo@todo.com',
    description='Hardware-free typed-object integration tools for IMDL.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mock_detected_objects_publisher = '
            'imdl_integration_tools.mock_detected_objects_publisher:main',
            'navigation_object_filter = '
            'imdl_integration_tools.navigation_object_filter:main',
            'control_object_filter = '
            'imdl_integration_tools.control_object_filter:main',
        ],
    },
)
