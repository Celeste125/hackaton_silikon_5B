from setuptools import find_packages, setup

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'gripper_server = robot_arm.Gripper:main',
        	'move_distance_server = robot_arm.MoveDistance:main',
        	'robot_arm_communication = robot_arm.Communication:main',
        ],
    },
)
