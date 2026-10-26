from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_dir = get_package_share_directory('frontier_explorer')

    return LaunchDescription([

        # LIDAR driver node. It outputs /scan_raw, and scan_timestamp_fixer
        # republishes it on /scan with synchronized timestamps.
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

        # Rewrite scan timestamps with the Jetson clock to match TF time.
        Node(
            package='frontier_explorer',
            executable='scan_timestamp_fixer',
            name='scan_timestamp_fixer',
            output='screen'
        ),

        # /Team2/odom to odom -> base_link TF, published with the Jetson clock.
        Node(
            package='frontier_explorer',
            executable='odom_tf_republisher',
            name='odom_tf_republisher',
            output='screen'
        ),

        # slam_toolbox mapping node. Launch directly as a Node so the TF
        # listener initializes correctly.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(slam_dir, 'config', 'slam_toolbox_params.yaml'),
                {'use_sim_time': False},
            ],
            remappings=[('/pose', '/robot_pose')],
            output='screen'
        ),

    ])
