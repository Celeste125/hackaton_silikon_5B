"""The landmark model, on the machine the camera is plugged into.

Run this on the Jetson. It publishes /landmarks/detections -- boxes and
intrinsics as JSON, a few hundred bytes a frame -- and no images, so the
frames never cross the arena WiFi and never leave the robot.

The model is an argument. Nothing in the pipeline changes when it arrives:

    # No weights yet: drive the geometry with a fixed box, to check the
    # extrinsics and the fusion against a landmark you place by hand.
    ros2 launch perception landmark_detector.launch.py backend:=stub \
        stub_detections:='[{"class":"food_dispenser","box":[0.40,0.40,0.60,0.70]}]'

    # The trained weights.
    ros2 launch perception landmark_detector.launch.py \
        backend:=ultralytics model_path:=/home/jetson/models/imdl_landmarks.pt

    # The same weights, exported to TensorRT on the Jetson itself.
    ros2 launch perception landmark_detector.launch.py \
        backend:=ultralytics model_path:=/home/jetson/models/imdl_landmarks.engine

    # No torch on this Jetson: ONNX through onnxruntime instead.
    ros2 launch perception landmark_detector.launch.py \
        backend:=onnx model_path:=/home/jetson/models/imdl_landmarks.onnx

With a RealSense, pass the depth topic as well and each single view gets its
own range, which localises a landmark without the robot having to move:

    ros2 launch perception landmark_detector.launch.py camera:=realsense \
        backend:=ultralytics model_path:=...

The companion is imdl_autonomy's landmark_mapping.launch.py, which does the
TF and the fusion and must run where SLAM runs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration('camera')

    image = PythonExpression([
        "'/camera/camera/color/image_raw' if '", camera, "' == 'realsense' ",
        "else '/webcam/image_raw'",
    ])
    info = PythonExpression([
        "'/camera/camera/aligned_depth_to_color/camera_info' if '", camera,
        "' == 'realsense' else '/webcam/camera_info'",
    ])
    # A webcam has no depth, so the topic stays empty and the node runs
    # bearing-only. Only the aligned-to-colour depth image is usable: a raw
    # depth frame has a different projection, so a box taken from the colour
    # image would sample the wrong pixels.
    depth = PythonExpression([
        "'/camera/camera/aligned_depth_to_color/image_raw' if '", camera,
        "' == 'realsense' else ''",
    ])

    detector = Node(
        package='perception',
        executable='landmark_detector',
        name='landmark_detector',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'backend': LaunchConfiguration('backend'),
            'model_path': LaunchConfiguration('model_path'),
            'landmark_config': LaunchConfiguration('landmark_config'),
            'image_topic': image,
            'camera_info_topic': info,
            'depth_topic': depth,
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'image_size': LaunchConfiguration('image_size'),
            'device': LaunchConfiguration('device'),
            'task': LaunchConfiguration('task'),
            'half': LaunchConfiguration('half'),
            'max_inference_rate_hz': LaunchConfiguration('rate'),
            'fallback_hfov_deg': LaunchConfiguration('fallback_hfov_deg'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'class_names_path': LaunchConfiguration('class_names_path'),
            'stub_detections': LaunchConfiguration('stub_detections'),
            'stub_landmark': LaunchConfiguration('stub_landmark'),
            'stub_box_fraction': LaunchConfiguration('stub_box_fraction'),
            'claude_model': LaunchConfiguration('claude_model'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='stub',
            choices=['ultralytics', 'onnx', 'stub', 'claude'],
            description=(
                'ultralytics: a trained .pt or TensorRT .engine, the normal '
                'path. onnx: the same model through onnxruntime, for a Jetson '
                'without a working torch. stub: fixed boxes and no model, to '
                'exercise the geometry before the weights exist. claude: '
                'boxes over the network, needs ANTHROPIC_API_KEY and sends '
                'frames off the robot -- a stand-in, not a deployment.'
            ),
        ),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Weights: .pt, .engine or .onnx. Required except for stub.',
        ),
        DeclareLaunchArgument(
            'landmark_config', default_value='',
            description=(
                'landmarks.yaml to use. Empty means the installed '
                'perception/config/landmarks.yaml. The mapper must read the '
                'same one.'
            ),
        ),
        DeclareLaunchArgument(
            'camera', default_value='webcam', choices=['webcam', 'realsense'],
            description=(
                'webcam: /webcam/*, no depth, so range comes from '
                'triangulation across views or a wall cast. '
                'realsense: /camera/camera/*, and every view carries its own '
                'depth-sampled range.'
            ),
        ),
        DeclareLaunchArgument(
            'rate', default_value='5.0',
            description=(
                'Inference rate cap in Hz. 5 is plenty while exploring: views '
                'from the same spot add nothing, so the limit is how fast the '
                'robot changes viewpoint, not how fast the model runs. Drop '
                'to ~0.3 for the claude backend.'
            ),
        ),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='0.25',
            description=(
                'Backend-level floor. The per-landmark min_confidence in '
                'landmarks.yaml is applied after this, so keep this the '
                'looser of the two.'
            ),
        ),
        DeclareLaunchArgument('iou_threshold', default_value='0.60'),
        DeclareLaunchArgument(
            'image_size', default_value='640',
            description='Must match what the model was trained at.',
        ),
        DeclareLaunchArgument(
            'device', default_value='0',
            description="GPU index, or 'cpu'. A TensorRT engine needs the GPU.",
        ),
        DeclareLaunchArgument(
            'task', default_value='detect',
            description=(
                'Only read for a .engine, which carries no task metadata. '
                "Use 'segment' for a segmentation export -- boxes are taken "
                'either way.'
            ),
        ),
        DeclareLaunchArgument(
            'half', default_value='false',
            description='FP16 inference; worth trying on the Jetson GPU.',
        ),
        DeclareLaunchArgument(
            'fallback_hfov_deg', default_value='70.4',
            description=(
                'C920s horizontal FOV, used only while camera_info is '
                'uncalibrated. Good to a couple of degrees; calibrate for '
                'better bearings.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_debug_image', default_value='false',
            description=(
                'A JPEG of the boxes on /landmarks/debug_image/compressed. '
                'Costs the bandwidth this node exists to save, so it is for '
                'checking the model, not for running.'
            ),
        ),
        DeclareLaunchArgument(
            'class_names_path', default_value='',
            description=(
                'onnx backend only: the dataset YAML or a JSON list of class '
                'names, in training order. An ONNX graph carries no labels.'
            ),
        ),
        DeclareLaunchArgument(
            'stub_landmark', default_value='',
            description=(
                'stub backend, the easy form: a landmark name, reported in a '
                'box at the centre of every frame. Aim the camera at the real '
                'object and this is exactly right -- and unlike '
                'stub_detections it needs no quoting, so it survives being '
                'typed through ssh.'
            ),
        ),
        DeclareLaunchArgument(
            'stub_box_fraction', default_value='0.2',
            description='How much of the frame the centred stub box covers.',
        ),
        DeclareLaunchArgument(
            'stub_detections', default_value='',
            description=(
                'stub backend, full control: JSON list of {"class", "box", '
                '"confidence"}, box normalised [x0,y0,x1,y1] in 0-1. '
                'Overrides stub_landmark.'
            ),
        ),
        DeclareLaunchArgument('claude_model', default_value='claude-opus-5-5'),
        detector,
    ])
