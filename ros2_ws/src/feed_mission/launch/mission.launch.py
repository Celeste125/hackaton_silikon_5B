"""The feeding mission, on top of navigation on the saved map.

    ros2 launch robot_bringup bringup.launch.py
    ros2 launch frontier_explorer navigation.launch.py
    ros2 launch feed_mission mission.launch.py

For now this is the lap driver only: four corners taught in the web panel
(Setup tab), laps started from the Unload tab or with
    ros2 service call /lap/start std_srvs/srv/Trigger
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
    ])
