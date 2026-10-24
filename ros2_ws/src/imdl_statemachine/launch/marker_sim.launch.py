"""Marker route on a saved map with no robot: fake robot + camera, Nav2, detector, mission.

    ros2 launch imdl_statemachine marker_sim.launch.py
    ros2 launch imdl_simulation real_rviz.launch.py      # in a second terminal

Defaults use the arena_task1_a map and config/waypoints_arena_test.yaml. For
another map, pass ``map:=`` and ``waypoints_file:=`` (marker_route.sim_markers
must lie on the inner face of that map's walls) and set ``start`` /
``initial_x/y/yaw`` to a free pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('imdl_statemachine')
    arena_map = os.path.join(
        get_package_share_directory('imdl_autonomy'), 'maps', 'arena_task1_a.yaml')
    waypoints = os.path.join(share, 'config', 'waypoints_arena_test.yaml')

    map_file = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('imdl_autonomy'), 'launch', 'map_navigation.launch.py')),
        launch_arguments={
            'map': map_file,
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }.items(),
    )

    fake_robot = Node(
        package='imdl_statemachine', executable='fake_robot', output='screen',
        parameters=[{
            'map': map_file,
            'waypoints_file': waypoints_file,
            'start_x': LaunchConfiguration('initial_x'),
            'start_y': LaunchConfiguration('initial_y'),
            'start_yaw': LaunchConfiguration('initial_yaw'),
        }],
    )
    detector = Node(
        package='imdl_statemachine', executable='aruco_detector', output='screen',
        parameters=[{'mount_xyz': [0.1, 0.0, 0.25], 'marker_size_m': 0.20}],
    )
    # Give AMCL and Nav2 time to come up before the mission starts.
    mission = TimerAction(period=20.0, actions=[Node(
        package='imdl_statemachine', executable='state_machine', output='screen',
        parameters=[{'waypoints_file': waypoints_file}],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=arena_map),
        DeclareLaunchArgument('waypoints_file', default_value=waypoints),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        navigation, fake_robot, detector, mission,
    ])
