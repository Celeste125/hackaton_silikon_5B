"""Fuse landmark detections into map positions, while the robot explores.

Run this where SLAM runs -- the laptop -- because the fusion does TF lookups
at each image's own timestamp and the project's TF chain must not be split
across machines. The detector runs on the Jetson with the camera; only JSON
crosses the link.

The full run, three terminals:

    # laptop
    ros2 launch imdl_autonomy slam_navigation.launch.py run_slam:=true
    ros2 launch imdl_autonomy landmark_mapping.launch.py

    # jetson
    ros2 launch perception webcam.launch.py publish_camera_tf:=false
    ros2 launch perception landmark_detector.launch.py \
        backend:=ultralytics model_path:=~/models/imdl_landmarks.pt

    # laptop, once Nav2 reports "Managed nodes are active"
    ros2 launch imdl_autonomy frontier_explore.launch.py

publish_camera_tf:=false on the Jetson matters: the camera's static transform
is published here instead, by this launch file, so the whole TF chain stays on
one machine.

Watch it converge:

    ros2 topic echo /landmarks/state

and in RViz add MarkerArray on /landmarks/markers. Each landmark shows a
sphere, a translucent 2-sigma ellipse and a label. The ellipse starting long
and thin and then rounding off is the estimate improving as the robot sees the
landmark from new angles; an ellipse that stays a long streak means every view
so far came from the same side, and /landmarks/<name>/next_view carries a pose
to drive to that would fix it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapper = Node(
        package='imdl_autonomy',
        executable='landmark_mapper',
        name='landmark_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'detections_topic': LaunchConfiguration('detections_topic'),
            'landmark_config': LaunchConfiguration('landmark_config'),
            'map_frame': LaunchConfiguration('map_frame'),
            'map_topic': LaunchConfiguration('map_topic'),
            'max_sigma_m': LaunchConfiguration('max_sigma_m'),
            'nominal_range_m': LaunchConfiguration('nominal_range_m'),
            'use_wall_cast': LaunchConfiguration('use_wall_cast'),
            'wall_cast_max_range_m': LaunchConfiguration('wall_cast_max_range_m'),
            'tf_timeout_sec': LaunchConfiguration('tf_timeout_sec'),
            'next_view_standoff_m': LaunchConfiguration('next_view_standoff_m'),
            'output_path': LaunchConfiguration('output_path'),
            'base_frame': LaunchConfiguration('base_frame'),
            'prior_source': LaunchConfiguration('prior_source'),
            'prior_file': LaunchConfiguration('prior_file'),
            'prior_file_sigma_m': LaunchConfiguration('prior_file_sigma_m'),
            'prior_network_path': LaunchConfiguration('prior_network_path'),
            'prior_inflate': LaunchConfiguration('prior_inflate'),
            'prior_min_sigma_m': LaunchConfiguration('prior_min_sigma_m'),
            'prior_update_interval_sec':
                LaunchConfiguration('prior_update_interval_sec'),
            'dataset_dir': LaunchConfiguration('dataset_dir'),
            'dataset_interval_sec': LaunchConfiguration('dataset_interval_sec'),
        }],
    )

    # The camera extrinsics, published here rather than beside the camera, so
    # every transform the fusion needs is produced on this machine. Measure
    # these on the real robot: extrinsic error is a bias that no number of
    # views removes, unlike the box noise the fusion does average away.
    extrinsics = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('camera_frame'),
        ],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'detections_topic', default_value='/landmarks/detections',
            description='JSON from perception landmark_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'landmark_config', default_value='',
            description=(
                'Empty means the installed perception/config/landmarks.yaml. '
                'Must be the same file the detector reads.'
            ),
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument(
            'max_sigma_m', default_value='0.25',
            description=(
                'A landmark counts as localised once its worst axis is known '
                'to better than this. 0.25 m is about what a Create3 needs to '
                'drive to something and find it in frame.'
            ),
        ),
        DeclareLaunchArgument(
            'nominal_range_m', default_value='2.0',
            description=(
                'Assumed range before anything is known, used only to weight '
                'the first views. Roughly the arena scale.'
            ),
        ),
        DeclareLaunchArgument(
            'use_wall_cast', default_value='true',
            description=(
                'Cast a bearing at the SLAM wall behind the object to get a '
                "range from a single webcam view. Biased by the object's "
                'standoff from the wall, so it is dropped automatically once '
                'the views triangulate on their own.'
            ),
        ),
        DeclareLaunchArgument('wall_cast_max_range_m', default_value='6.0'),
        DeclareLaunchArgument(
            'tf_timeout_sec', default_value='0.5',
            description=(
                'How long to wait for the transform at an image stamp. The '
                'detections arrive over WiFi, so some slack is needed; too '
                'much and the node stalls on views it will never get.'
            ),
        ),
        DeclareLaunchArgument(
            'next_view_standoff_m', default_value='1.2',
            description='How far from a landmark to suggest standing.',
        ),
        DeclareLaunchArgument(
            'output_path', default_value='',
            description=(
                'Write the landmarks to this YAML as they settle, so a '
                'restart keeps them, e.g. ~/maps/arena_landmarks.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description=(
                'Used both for the camera extrinsics and to centre the '
                "location prior's window on the robot."
            ),
        ),

        # --- the learned location prior -------------------------------------
        # See landmark_prior/README.md for collecting data and training.
        DeclareLaunchArgument(
            'prior_source', default_value='none',
            choices=['none', 'file', 'network'],
            description=(
                'Where a belief about the landmarks comes from before the '
                'camera has seen them. none: geometry only. file: the '
                'positions a previous run wrote with output_path -- needs '
                'this run to share that map frame, i.e. AMCL on a saved map. '
                'network: the trained prior network, which reads the live '
                'map and needs no shared frame. Either way the prior is '
                'outvoted by real views and never counts as converged.'
            ),
        ),
        DeclareLaunchArgument(
            'prior_file', default_value='',
            description='Landmarks YAML from an earlier run, for prior_source:=file.',
        ),
        DeclareLaunchArgument(
            'prior_file_sigma_m', default_value='0.5',
            description=(
                'How far last run\'s position might have moved. Keep it '
                'generous: this is a hint, not a measurement.'
            ),
        ),
        DeclareLaunchArgument(
            'prior_network_path', default_value='',
            description=(
                'The .onnx from train_landmark_prior.py, for '
                'prior_source:=network. Loaded through onnxruntime, so the '
                'robot needs no torch.'
            ),
        ),
        DeclareLaunchArgument(
            'prior_inflate', default_value='1.5',
            description=(
                "Multiplies the network's own uncertainty. A network trained "
                'on one arena produces very peaked heatmaps; raise this to '
                '3.0 if the prior is not being overridden by good views.'
            ),
        ),
        DeclareLaunchArgument(
            'prior_min_sigma_m', default_value='0.15',
            description=(
                'Floor on how certain any prior may claim to be, so it can '
                'always be argued down by a couple of real observations.'
            ),
        ),

        DeclareLaunchArgument(
            'prior_update_interval_sec', default_value='2.0',
            description=(
                'Seconds between re-running the prior. The network reads the '
                'live map, so its guess improves as the map fills in; the '
                'file prior never changes and ignores this.'
            ),
        ),

        # --- recording training data for that network -----------------------
        DeclareLaunchArgument(
            'dataset_dir', default_value='',
            description=(
                'Set this to record prior-network training samples, one run '
                'per launch, e.g. ~/landmark_runs. Off when empty. Labels '
                'come from whatever this run converges on, so a run that '
                'localises nothing is discarded at dataset-build time.'
            ),
        ),
        DeclareLaunchArgument(
            'dataset_interval_sec', default_value='2.0',
            description=(
                'Seconds between recorded samples. Faster buys little: '
                'consecutive windows are nearly identical.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_frame', default_value='webcam_optical_frame',
            description=(
                'Must match the frame_id the camera driver stamps its images '
                'with; the detections carry it and the fusion looks it up.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_camera_tf', default_value='true',
            description=(
                'Publish base_link -> the camera optical frame here. Leave '
                'this on and pass publish_camera_tf:=false to the camera '
                'launch, so the TF chain stays on one machine.'
            ),
        ),
        # MEASURE THESE on the real robot. Same placeholders as
        # perception/launch/webcam.launch.py.
        DeclareLaunchArgument('camera_x', default_value='0.10'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.10'),
        # Optical frame: roll -90 deg then yaw -90 deg puts z forward and y
        # down, from base_link's x-forward z-up convention.
        DeclareLaunchArgument('camera_roll', default_value='-1.5707963'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.5707963'),
        extrinsics,
        mapper,
    ])
