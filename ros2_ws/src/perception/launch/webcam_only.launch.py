"""Just the webcam, so you can see a picture. Nothing else.

Runnable by path, so the Jetson needs no up-to-date workspace and no colcon --
only `sudo apt install ros-humble-v4l2-camera`:

    ros2 launch ~/webcam_only.launch.py

The static transform is here solely so RViz has a frame for the image and
stays quiet. It is not the measured mounting pose; use webcam.launch.py (or
IMDL_CAM_* in the tmux script) when the geometry actually matters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    """Built here so width/height/fps can be plain integers on the command
    line. v4l2_camera wants lists, and a list argument cannot survive the
    ssh -> bash -ic quoting the Jetson is driven through."""
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps = int(LaunchConfiguration('fps').perform(context))
    return [Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        namespace='webcam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            # YUYV, not MJPG: this v4l2_camera sets MJPG happily and then
            # cannot decode it ('Unrecognized image encoding []').
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'image_size': [width, height],
            'time_per_frame': [1, fps],
            'camera_frame_id': 'webcam_optical_frame',
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        # rgb8 is 3 bytes a pixel and DDS fragments every frame into hundreds
        # of UDP packets; losing one loses the frame. On a congested network
        # 640x480 can drop nearly everything, so these are tunable:
        #   width:=320 height:=240  is a quarter of the bytes.
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='5'),
        OpaqueFunction(function=_nodes),
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
