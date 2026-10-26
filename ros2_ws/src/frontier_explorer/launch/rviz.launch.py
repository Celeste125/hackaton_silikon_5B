"""RViz with everything for mapping and navigation.

    ros2 launch frontier_explorer rviz.launch.py

Fixed frame map, top down. Shows the map, both costmaps, the lidar scan, the
robot footprint, TF (map -> odom -> base_link), Nav2's global and local plan,
the goal, the collision monitor's stop zone, and in saved-map mode AMCL's pose
and particles. "2D Pose Estimate" and "Nav2 Goal" in the toolbar set the
start pose and send goals; the Navigation 2 panel shows Nav2's state.

Needs a display: run it on the Jetson desktop, or on a laptop on the same
ROS_DOMAIN_ID with this package built (or pass config:=/path/to/navigation.rviz
to a plain `rviz2 -d`).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('frontier_explorer'), 'rviz', 'navigation.rviz']),
            description='RViz config file.',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('config')],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
    ])
