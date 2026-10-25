"""Everything the Jetson runs: lidar, camera and its mounting TF, detector.

One command on the robot. Launch it BY PATH out of the checkout, because the
Jetson has no colcon workspace for this repo:

    ros2 launch ~/imdl-main/ros2_ws/src/perception/launch/jetson_bringup.launch.py \\
        backend:=ultralytics \\
        model:=~/imdl-main/ros2_ws/src/perception/models/cows_pipes_yolov8.pt

A git pull is therefore the whole deployment. Nothing here resolves a package
share directory on the happy path: sibling launch files are found relative to
this file, and the detector runs as a plain python3 process with PYTHONPATH
set, exactly as it was run by hand before. The ament index is consulted only
as a fallback, so the same file still works from an installed share tree.

The lidar deliberately publishes /scan_raw (fix_timestamps:=false). The laptop
owns /scan, odom -> base_link and map -> odom; see laptop_bringup.launch.py.
Do not start SLAM or an odometry TF publisher here.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_HERE = Path(__file__).resolve().parent
_PERCEPTION = _HERE.parent


def _locate(package, relative):
    """Checkout layout first, an installed share tree second.

    In the checkout this file is at src/perception/launch/, so a sibling
    package is two levels up. Falling back to the ament index keeps the file
    usable on the laptop, where perception is installed and src/ may be absent.
    """
    candidate = _PERCEPTION.parent / package / relative
    if candidate.exists():
        return str(candidate)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory(package)) / relative)


def generate_launch_description():
    detector = _PERCEPTION / 'perception' / 'landmark_detector_node.py'
    # Only the stub takes stub_landmark, and only the model backends take
    # model_path; passing the wrong one is harmless but reads as a mistake in
    # the process line, which is the first thing anyone debugging will see.
    backend = LaunchConfiguration('backend')
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='disabled',
            choices=['disabled', 'ultralytics', 'onnx', 'stub', 'claude'],
            description='disabled publishes empty frames: a camera heartbeat with no weights.'),
        DeclareLaunchArgument('model', default_value='',
                              description='Weights for the ultralytics/onnx backends.'),
        DeclareLaunchArgument('stub_landmark', default_value='cow_enclosure'),
        DeclareLaunchArgument('inference_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('width', default_value='320'),
        DeclareLaunchArgument('height', default_value='240'),
        DeclareLaunchArgument('fps', default_value='5'),
        # Measured 2026-10-25, relative to the LIDAR: lidar.launch.py stamps
        # scans base_link, so SLAM treats the lidar centre as base_link. roll
        # -pi/2 is a LEVEL camera here; pitch would spin the image, not tilt it.
        DeclareLaunchArgument('x', default_value='0.08'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.35'),
        DeclareLaunchArgument('roll', default_value='-1.5707963'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='-1.5707963'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_locate('slam', 'launch/lidar.launch.py')),
            launch_arguments={
                'fix_timestamps': 'false',
                'serial_port': LaunchConfiguration('serial_port'),
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_HERE / 'webcam_only.launch.py')),
            launch_arguments={
                name: LaunchConfiguration(name) for name in (
                    'width', 'height', 'fps', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
            }.items()),

        ExecuteProcess(
            # python3 by path, not Node(package=...): an executable entry point
            # would need perception pip/colcon-installed, which is the one
            # thing this machine does not have.
            cmd=['python3', str(detector), '--ros-args',
                 '-p', ['backend:=', backend],
                 '-p', ['model_path:=', LaunchConfiguration('model')],
                 '-p', ['stub_landmark:=', LaunchConfiguration('stub_landmark')],
                 '-p', 'landmark_config:=' + str(_PERCEPTION / 'config' / 'landmarks.yaml'),
                 '-p', 'image_topic:=/webcam/image_raw',
                 '-p', 'camera_info_topic:=/webcam/camera_info',
                 '-p', ['max_inference_rate_hz:=',
                        LaunchConfiguration('inference_rate_hz')]],
            additional_env={'PYTHONPATH': os.pathsep.join(
                [str(_PERCEPTION)] + [p for p in [os.environ.get('PYTHONPATH')] if p])},
            output='screen'),
    ])
