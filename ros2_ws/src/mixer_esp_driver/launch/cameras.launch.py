"""The two USB webcams (Logitech C920), as JPEG for the web panel.

  ros2 launch mixer_esp_driver cameras.launch.py

Each camera is a v4l2_camera node. With compressed_image_transport installed
it also publishes <ns>/image_raw/compressed (JPEG), which web_control streams
to the browser:

  /camera0/image_raw/compressed  USB port 2.2 (was /dev/video0)
  /camera1/image_raw/compressed  USB port 2.1 (/dev/video2)

/dev/videoN numbers change when a camera is replugged (each C920 takes two
numbers), so the defaults are the per-USB-port links in /dev/v4l/by-path/,
which stay put as long as each camera stays in its port. Pass a plain device
to override, e.g. camera0_device:=/dev/video0. A device can only be opened
once: stop any other v4l2_camera_node on it first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BY_PATH = '/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0'


def camera(name):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        namespace=name,
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration(f'{name}_device'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            # 15 fps is plenty for watching and halves the JPEG encoding load.
            'time_per_frame': [1, 15],
            'image_raw.jpeg_quality': ParameterValue(
                LaunchConfiguration('jpeg_quality'), value_type=int),
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera0_device', default_value=BY_PATH.format(port='2.2'),
            description='Video device of camera 0 (USB port 2.2).',
        ),
        DeclareLaunchArgument(
            'camera1_device', default_value=BY_PATH.format(port='2.1'),
            description='Video device of camera 1 (USB port 2.1).',
        ),
        DeclareLaunchArgument(
            'jpeg_quality', default_value='70',
            description='JPEG quality of image_raw/compressed, 1-100.',
        ),
        camera('camera0'),
        camera('camera1'),
    ])
