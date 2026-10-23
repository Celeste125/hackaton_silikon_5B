#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('model_visual_yaw', default_value='0.0',
                          description='Yaw correction applied only to visual meshes.'),
    DeclareLaunchArgument('camera_update_rate', default_value='20.0',
                          description='Simulated RGB-D update rate in Hz.'),
    DeclareLaunchArgument('ultrasonic_update_rate', default_value='20.0',
                          description='Simulated front ultrasonic update rate in Hz.'),
    DeclareLaunchArgument('sensor_visualization', default_value='false',
                          choices=['true', 'false'],
                          description='Render simulated sensor helpers in Gazebo.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_create3_gazebo_bringup, 'launch', 'gazebo.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [get_package_share_directory('imdl_simulation'),
         'launch', 'imdl_create3_spawn.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world_path', LaunchConfiguration('world_path')),
            ('use_gazebo_gui', LaunchConfiguration('use_gazebo_gui'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('spawn_dock', LaunchConfiguration('spawn_dock')),
            ('model_visual_yaw', LaunchConfiguration('model_visual_yaw')),
            ('camera_update_rate', LaunchConfiguration('camera_update_rate')),
            ('ultrasonic_update_rate', LaunchConfiguration('ultrasonic_update_rate')),
            ('sensor_visualization', LaunchConfiguration('sensor_visualization')),
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))])

    ultrasonic_adapter = Node(
        package='imdl_simulation',
        executable='ultrasonic_range_adapter.py',
        name='ultrasonic_range_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/ultrasonic/front/scan_raw',
            'output_topic': '/ultrasonic/front/range',
        }],
    )
    scan_self_filter = Node(
        package='imdl_simulation',
        executable='scan_self_filter.py',
        name='scan_self_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
        }],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Gazebo
    ld.add_action(gazebo)
    # Gazebo ray sensors can see collision geometry from their own model. Keep
    # /scan as the stable real/sim interface while removing only points inside
    # the exact Create3 + camera footprint.
    ld.add_action(scan_self_filter)
    # Convert the reliable narrow LaserScan output to the public Arduino-like
    # sensor_msgs/Range contract before any task nodes consume it.
    ld.add_action(ultrasonic_adapter)
    # Robot spawn
    ld.add_action(robot_spawn)
    return ld
