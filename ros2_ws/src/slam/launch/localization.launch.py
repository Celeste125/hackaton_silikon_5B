from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

# Localization-only mode: load a saved map and localize with the AMCL particle
# filter, which is natively supported by Nav2.
# slam_toolbox localization mode is kept as a fallback.
# Prerequisite: ~/maps/arena_map.yaml and arena_map.pgm must exist.

def generate_launch_description():
    slam_dir = get_package_share_directory('slam')
    map_file = os.path.expanduser('~/maps/arena_map.yaml')

    return LaunchDescription([

        # LIDAR driver node.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 460800,
                'frame_id': 'base_link',
                'angle_compensate': True,
            }],
            remappings=[('/scan', '/scan_raw')],
            output='screen'
        ),

        # Timestamp correction node.
        Node(
            package='slam',
            executable='scan_timestamp_fixer',
            name='scan_timestamp_fixer',
            output='screen'
        ),

        # odom TF publisher node.
        Node(
            package='slam',
            executable='odom_tf_republisher',
            name='odom_tf_republisher',
            output='screen'
        ),

        # Map server: load the pgm/yaml map for AMCL.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_file,
            }],
            output='screen'
        ),

        # Lifecycle manager: automatically activate map_server and amcl.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen'
        ),

        # AMCL particle-filter localization node.
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                os.path.join(slam_dir, 'config', 'amcl_params.yaml'),
            ],
            remappings=[('/amcl_pose', '/robot_pose')],
            output='screen'
        ),

    ])
