"""The feeding mission, on top of navigation on the saved map.

    ros2 launch robot_bringup bringup.launch.py
    ros2 launch frontier_explorer navigation.launch.py
    ros2 launch feed_mission mission.launch.py

Starts the three feed_mission nodes:
  mission      the whole run in one go (web panel Mission tab, /mission/start)
  lap_driver   feeding laps around the cow box (Feed tab, /lap/start)
  relocalizer  finds the robot on the map from the lidar (Setup tab, /relocalize)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'poses_file', default_value='~/maps/arena_poses.yaml',
            description='Taught poses, written by the web panel Setup tab.',
        ),
        DeclareLaunchArgument(
            'speed', default_value='0.15',
            description='Lap speed on the straights (m/s).',
        ),
        Node(
            package='feed_mission',
            executable='lap_driver',
            name='lap_driver',
            output='screen',
            parameters=[{
                'poses_file': LaunchConfiguration('poses_file'),
                'speed': LaunchConfiguration('speed'),
            }],
        ),
        Node(
            package='feed_mission',
            executable='mission',
            name='mission',
            output='screen',
            parameters=[{'poses_file': LaunchConfiguration('poses_file')}],
        ),
        Node(
            package='feed_mission',
            executable='relocalizer',
            name='relocalizer',
            output='screen',
        ),
    ])
