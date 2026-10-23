"""Camera-based food-dispenser localisation, on top of a live SLAM run.

Needs three things already running: slam_navigation.launch.py (for the map
frame), perception's camera.launch.py (for the RealSense and the
base_link -> camera_link extrinsics), and ANTHROPIC_API_KEY in the
environment. Take a view with:

    ros2 topic pub --once /dispenser/locate std_msgs/msg/Empty "{}"

Do that from two or three different angles. One view carries a bias from the
box centroid sitting on the dispenser's front face; viewing from elsewhere
cancels most of it, and the log prints the spread so you can see it settle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    locator = Node(
        package='imdl_autonomy',
        executable='dispenser_locator',
        name='dispenser_locator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'map_frame': LaunchConfiguration('map_frame'),
            'model': LaunchConfiguration('model'),
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'info_topic': LaunchConfiguration('info_topic'),
            'min_confidence': LaunchConfiguration('min_confidence'),
            'max_depth_m': LaunchConfiguration('max_depth_m'),
            'auto_interval_sec': LaunchConfiguration('auto_interval_sec'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'model', default_value='claude-opus-5-5',
            description='Claude model asked to find the dispenser in a frame.',
        ),
        DeclareLaunchArgument(
            'color_topic',
            default_value='/camera/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Must be depth aligned to colour, not raw depth.',
        ),
        DeclareLaunchArgument(
            'info_topic',
            default_value='/camera/camera/aligned_depth_to_color/camera_info',
        ),
        DeclareLaunchArgument(
            'min_confidence', default_value='0.4',
            description='Discard boxes Claude is less sure of than this.',
        ),
        DeclareLaunchArgument(
            'max_depth_m', default_value='3.0',
            description='Depth window; RealSense noise grows with range.',
        ),
        DeclareLaunchArgument(
            'auto_interval_sec', default_value='0.0',
            description='Above 0, take a view every N seconds unprompted.',
        ),
        locator,
    ])
