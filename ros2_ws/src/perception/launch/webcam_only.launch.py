"""Just the webcam, so you can see a picture. Nothing else.

Runnable by path, so the Jetson needs no up-to-date workspace and no colcon --
only `sudo apt install ros-humble-v4l2-camera`:

    ros2 launch ~/webcam_only.launch.py

The static transform is here solely so RViz has a frame for the image and
stays quiet. It is not the measured mounting pose; use webcam.launch.py (or
IMDL_CAM_* in the tmux script) when the geometry actually matters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam',
            namespace='webcam',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                # MJPG keeps a VGA stream to about a megabit on a busy WiFi.
                'pixel_format': 'MJPG',
                'output_encoding': 'rgb8',
                'camera_frame_id': 'webcam_optical_frame',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='webcam_frame',
            arguments=[
                '--frame-id', 'base_link',
                '--child-frame-id', 'webcam_optical_frame',
            ],
            output='screen',
        ),
    ])
