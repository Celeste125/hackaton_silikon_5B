"""Start only the physical SLAMTEC lidar and timestamp correction.

Pass fix_timestamps:=false when SLAM runs on another machine: the fixer stamps
/scan with *this* machine's clock, which only helps if the TF it will be
matched against is published here too. With SLAM on the laptop, the laptop runs
the fixer instead, so scan stamps and TF share one clock.
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
            default_value='base_link',
            description='Frame written into LaserScan messages.',
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
            package='slam',
            executable='scan_timestamp_fixer',
            name='scan_timestamp_fixer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fix_timestamps')),
        ),
    ])
