"""The feeding mission, on top of navigation on the saved map.

    ros2 launch robot_bringup bringup.launch.py
    ros2 launch frontier_explorer navigation.launch.py
    ros2 launch feed_mission mission.launch.py

Starts the three feed_mission nodes:
  mission      the whole run in one go (web panel Mission tab, /mission/start)
  lap_driver   feeding laps around the cow box (Feed tab, /lap/start)
  relocalizer  finds the robot on the map from the lidar (Setup tab, /relocalize)

By default in one process with one shared TF listener (all_nodes.py), which
saves the Jetson a good part of a core; one_process:=false runs them apart,
e.g. to restart one of them on its own.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    one = LaunchConfiguration('one_process')
    poses = {'poses_file': LaunchConfiguration('poses_file')}
    speed = {'speed': LaunchConfiguration('speed')}
    return LaunchDescription([
        DeclareLaunchArgument(
            'poses_file', default_value='~/maps/arena_poses.yaml',
            description='Taught poses, written by the web panel Setup tab.',
        ),
        DeclareLaunchArgument(
            'speed', default_value='0.15',
            description='Lap speed on the straights (m/s).',
        ),
        DeclareLaunchArgument(
            'one_process', default_value='true', choices=['true', 'false'],
            description='All three nodes in one process, sharing one TF listener.',
        ),
        Node(  # no name=: it would rename all three nodes in the process
            package='feed_mission',
            executable='feed_mission',
            output='screen',
            parameters=[{**poses, **speed}],
            condition=IfCondition(one),
        ),
        Node(
            package='feed_mission', executable='lap_driver', name='lap_driver',
            output='screen', parameters=[{**poses, **speed}], condition=UnlessCondition(one),
        ),
        Node(
            package='feed_mission', executable='mission', name='mission',
            output='screen', parameters=[poses], condition=UnlessCondition(one),
        ),
        Node(
            package='feed_mission', executable='relocalizer', name='relocalizer',
            output='screen', condition=UnlessCondition(one),
        ),
    ])
