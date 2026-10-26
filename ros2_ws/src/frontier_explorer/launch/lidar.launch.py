"""Start only the SLAMTEC lidar, e.g. to test it: /scan_raw -> /scan.

    ros2 launch frontier_explorer lidar.launch.py
    ros2 topic hz /scan

scan_timestamp_fixer restamps each scan with this machine's clock, for SLAM.
robot_bringup's bringup.launch.py includes this file.

The scans are in the frame "laser", placed on base_link by a static TF. The
lidar is mounted turned 180 degrees: its 0-degree direction points to the
robot's back (measured by driving straight and aligning scans against odom).
With the scan straight in base_link, Nav2 and slam_toolbox saw the arena
mirrored front to back, so the robot drove into walls it believed were
behind it. If the lidar is remounted, change lidar_yaw (and lidar_x/lidar_y
if it moves off the centre).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device used by the physical SLAMTEC lidar.',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='460800',
            description='Serial baud rate used by the physical lidar.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame written into LaserScan messages.',
        ),
        DeclareLaunchArgument(
            'lidar_x', default_value='0.0',
            description='Lidar position ahead of base_link (m).',
        ),
        DeclareLaunchArgument(
            'lidar_y', default_value='0.0',
            description='Lidar position left of base_link (m).',
        ),
        DeclareLaunchArgument(
            'lidar_yaw', default_value='3.14159265',
            description="Lidar's 0-degree direction relative to the robot's front (rad).",
        ),
        DeclareLaunchArgument(
            'fix_timestamps',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Republish /scan_raw as /scan stamped with this machine\'s '
                'clock. Set false when SLAM runs elsewhere and restamps there.'
            ),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', LaunchConfiguration('lidar_x'),
                '--y', LaunchConfiguration('lidar_y'),
                '--yaw', LaunchConfiguration('lidar_yaw'),
                '--frame-id', 'base_link',
                '--child-frame-id', LaunchConfiguration('frame_id'),
            ],
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': ParameterValue(
                    LaunchConfiguration('serial_baudrate'),
                    value_type=int,
                ),
                'frame_id': LaunchConfiguration('frame_id'),
                'angle_compensate': True,
            }],
            remappings=[('/scan', '/scan_raw')],
            output='screen',
        ),
        Node(
            package='frontier_explorer',
            executable='scan_timestamp_fixer',
            name='scan_timestamp_fixer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fix_timestamps')),
        ),
    ])
