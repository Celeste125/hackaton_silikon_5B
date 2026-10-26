from setuptools import find_packages, setup

package_name = 'web_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={package_name: ['web/*.html']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 2',
    maintainer_email='todo@todo.todo',
    description='Web control panel for the robot: mixer, valves, door, base, cameras.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_panel = web_panel.server:main',
        ],
    },
)
