"""Just the webcam, so you can see a picture. Nothing else.

Runnable by path, so the machine it runs on needs no colcon and no up-to-date
workspace -- only a checkout and `sudo apt install ros-humble-v4l2-camera`:

    ros2 launch ~/imdl-main/ros2_ws/src/perception/launch/webcam_only.launch.py

It publishes base_link -> webcam_optical_frame too, so one terminal is the
whole camera. The pose defaults are placeholders that only keep RViz quiet --
pass the measured one (the tmux script passes IMDL_CAM_*), because a bearing
is only as good as the transform it is expressed in:

    ros2 launch ~/webcam_only.launch.py x:=0.10 z:=0.10 roll:=-1.5707963

publish_tf:=false when something else already publishes that pair: two static
publishers for one parent/child leave RViz with whichever arrived last.

Watching the picture on ANOTHER machine? Shrink it first:

    ros2 launch ~/webcam_only.launch.py width:=320 height:=240

The defaults below suit a viewer on this same machine, which gets frames
through shared memory and never touches the network. A raw 640x480 rgb8 frame
is ~900 KB, which DDS splits into ~650 UDP packets, and a frame is discarded
whole if any one of them is lost. Over WiFi that means a remote RViz freezes
on its first picture while the local one stays live.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
        # Off when something else already publishes the measured mounting
        # pose: two static publishers for one parent/child pair leave RViz
        # picking whichever arrived last.
        DeclareLaunchArgument('publish_tf', default_value='true'),
        # Where the camera sits on the robot. MEASURE x/y/z: with bearing-only
        # geometry an angular error scales with range, and no averaging removes
        # it. The roll/yaw pair turns base_link (x forward, z up) into the
        # optical convention (z forward, y down) the locator deprojects in.
        DeclareLaunchArgument('x', default_value='0.10'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('roll', default_value='-1.5707963'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='-1.5707963'),
        OpaqueFunction(function=_nodes),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='webcam_frame',
            arguments=[
                '--x', LaunchConfiguration('x'),
                '--y', LaunchConfiguration('y'),
                '--z', LaunchConfiguration('z'),
                '--roll', LaunchConfiguration('roll'),
                '--pitch', LaunchConfiguration('pitch'),
                '--yaw', LaunchConfiguration('yaw'),
                '--frame-id', 'base_link',
                '--child-frame-id', 'webcam_optical_frame',
            ],
            condition=IfCondition(LaunchConfiguration('publish_tf')),
            output='screen',
        ),
    ])
