"""Logitech C920s (or any UVC webcam) as the robot's camera.

Runs on whichever machine the camera is plugged into -- the Jetson, if that is
where the USB cable goes. The frames cross the network to wherever the
dispenser locator runs, which is fine: an image topic tolerates WiFi lag in a
way a TF lookup inside a message filter does not.

Deliberately 640x480 at a low frame rate. The C920 will happily do 1080p30,
but the arena WiFi is congested and a VLM needs no more than VGA to put a box
around a dispenser. Raw YUYV rather than MJPEG, because this v4l2_camera
cannot decode MJPEG even though it will set it.

This camera has no depth. The locator therefore falls back to bearing-only
geometry (ray-cast onto the SLAM wall map, and triangulation across views),
which is why the mounting numbers below matter: a bearing is only as good as
the transform it is expressed in. Measure them on the real robot.

    ros2 launch perception webcam.launch.py video_device:=/dev/video0

Check what the camera actually supports first:
    v4l2-ctl --list-devices
    v4l2-ctl --list-formats-ext -d /dev/video0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    frame_id = LaunchConfiguration('frame_id')

    # v4l2_camera over usb_cam: it publishes camera_info alongside the image
    # without extra configuration, and the locator needs the intrinsics.
    driver = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        namespace=camera_name,
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': ParameterValue(
                LaunchConfiguration('image_size'), value_type=str),
            'camera_frame_id': frame_id,
            'pixel_format': LaunchConfiguration('pixel_format'),
            'output_encoding': 'rgb8',
            # An uncalibrated camera_info has a zeroed K; the locator notices
            # and falls back to the declared field of view.
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'use_sim_time': False,
        }],
    )

    # Where the camera sits on the robot. Without this the bearing is
    # meaningless, so it is on by default rather than opt-in.
    extrinsics = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_webcam_tf',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', frame_id,
        ],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='UVC device node; check with v4l2-ctl --list-devices.',
        ),
        DeclareLaunchArgument(
            'camera_name', default_value='webcam',
            description='Namespace, so topics are <camera_name>/image_raw.',
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='webcam_optical_frame',
            description=(
                'Optical frame of the webcam: x right, y down, z forward, '
                'which is what the locator deprojects in.'
            ),
        ),
        DeclareLaunchArgument(
            'image_size', default_value='[640,480]',
            description='VGA is plenty for a bounding box and kind to the WiFi.',
        ),
        DeclareLaunchArgument(
            'pixel_format', default_value='YUYV',
            description=(
                'YUYV, not MJPG: this v4l2_camera accepts MJPG and then dies '
                'decoding it (Unrecognized image encoding []).'
            ),
        ),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description=(
                'file:// path to a calibration YAML from camera_calibration. '
                'Empty means uncalibrated, and the locator uses the FOV '
                'fallback instead -- good for a few degrees, not better.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_camera_tf', default_value='true',
            description='Publish base_link -> the webcam optical frame.',
        ),
        # Placeholders in the same spirit as camera.launch.py's RealSense
        # numbers: forward-facing, slightly above the deck, tilted down.
        # MEASURE THESE. Extrinsic error is a bias no averaging removes.
        DeclareLaunchArgument('camera_x', default_value='0.10'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.10'),
        # Optical frame: roll -90 deg about x then yaw -90 deg puts z forward
        # and y down, from base_link's x-forward z-up convention.
        DeclareLaunchArgument('camera_roll', default_value='-1.5707963'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.5707963'),
        driver,
        extrinsics,
    ])
