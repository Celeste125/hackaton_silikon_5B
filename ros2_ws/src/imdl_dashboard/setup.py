from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'imdl_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariaceleste Fernandez Rios',
    maintainer_email='fernandezriosmariaceleste@gmail.com',
    description='Web control panel for the Team2 Create3 robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard_server = imdl_dashboard.dashboard_server:main',
        ],
    },
)
