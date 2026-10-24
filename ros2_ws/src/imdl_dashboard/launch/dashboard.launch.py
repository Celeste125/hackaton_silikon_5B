"""Web control panel. Open http://<this machine>:8080 from any browser."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/Team2'),
        DeclareLaunchArgument('port', default_value='8080'),
        Node(
            package='imdl_dashboard',
            executable='dashboard_server',
            name='dashboard_server',
            output='screen',
            parameters=[{
                'namespace': LaunchConfiguration('namespace'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
