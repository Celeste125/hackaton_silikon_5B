"""Start only the physical SLAMTEC lidar and timestamp correction."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        ),
    ])
