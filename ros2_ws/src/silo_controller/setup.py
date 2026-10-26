from setuptools import find_packages, setup


package_name = 'silo_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 2',
    maintainer_email='todo@todo.todo',
    description='Silo valve servos on the Raspberry Pi (PCA9685), and tools to command them.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'valve_controller = silo_controller.valve_controller:main',
            'valve_cmd = silo_controller.valve_cmd:main',
            'servo_sweep = silo_controller.servo_sweep:main',
        ],
    },
)
