from setuptools import find_packages, setup

package_name = 'mixer_esp_driver'

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
    maintainer='fudayl',
    maintainer_email='fudayl@todo.todo',
    description='ROS 2 driver for the mixer ESP32 over USB serial: load cells, mixer motors, door servo.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'esp_driver_node = mixer_esp_driver.esp_driver_node:main',
            'calibrate = mixer_esp_driver.calibrate:main',
        ],
    },
)
