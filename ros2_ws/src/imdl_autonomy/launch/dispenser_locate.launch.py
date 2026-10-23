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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    # The depth topic is left pointing at the RealSense in both presets. With a
    # webcam nothing ever publishes it, the node sees no depth and falls back
    # to bearing-only geometry, so no conditional is needed.
    colour = PythonExpression([
        "'/webcam/image_raw' if '", camera, "' == 'webcam' else ",
        "'/camera/camera/color/image_raw'",
    ])
    info = PythonExpression([
        "'/webcam/camera_info' if '", camera, "' == 'webcam' else ",
        "'/camera/camera/aligned_depth_to_color/camera_info'",
    ])

    locator = Node(
        package='imdl_autonomy',
        executable='dispenser_locator',
        name='dispenser_locator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'map_frame': LaunchConfiguration('map_frame'),
            'model': LaunchConfiguration('model'),
            'color_topic': colour,
            'depth_topic': LaunchConfiguration('depth_topic'),
            'info_topic': info,
            'fallback_hfov_deg': LaunchConfiguration('fallback_hfov_deg'),
            'min_triangulation_angle_deg':
                LaunchConfiguration('min_triangulation_angle_deg'),
            'raycast_max_range_m': LaunchConfiguration('raycast_max_range_m'),
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
            'camera', default_value='webcam', choices=['webcam', 'realsense'],
            description=(
                'webcam: /webcam/* from perception webcam.launch.py, no depth, '
                'so range comes from triangulation or a wall cast. '
                'realsense: /camera/camera/*, depth does the range.'
            ),
        ),
        DeclareLaunchArgument(
            'fallback_hfov_deg', default_value='70.4',
            description='C920s horizontal FOV, used only while uncalibrated.',
        ),
        DeclareLaunchArgument(
            'min_triangulation_angle_deg', default_value='12.0',
            description='Refuse to intersect bearings closer than this.',
        ),
        DeclareLaunchArgument(
            'raycast_max_range_m', default_value='6.0',
            description='How far to cast a bearing looking for a wall.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Must be depth aligned to colour, not raw depth.',
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
