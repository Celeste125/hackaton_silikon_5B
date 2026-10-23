from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    debug_image_topic = LaunchConfiguration('debug_image_topic')
    debug_compressed_image_topic = LaunchConfiguration('debug_compressed_image_topic')
    debug_jpeg_quality = LaunchConfiguration('debug_jpeg_quality')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    iou_threshold = LaunchConfiguration('iou_threshold')
    image_size = LaunchConfiguration('image_size')
    device = LaunchConfiguration('device')
    max_inference_rate_hz = LaunchConfiguration('max_inference_rate_hz')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    yolo_node = Node(
        package='perception',
        executable='yolo_segmentation',
        name='yolo_segmentation',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'image_topic': image_topic,
            'detections_topic': detections_topic,
            'debug_image_topic': debug_image_topic,
            'debug_compressed_image_topic': debug_compressed_image_topic,
            'debug_jpeg_quality': ParameterValue(debug_jpeg_quality, value_type=int),
            'confidence_threshold': ParameterValue(
                confidence_threshold, value_type=float
            ),
            'iou_threshold': ParameterValue(iou_threshold, value_type=float),
            'image_size': ParameterValue(image_size, value_type=int),
            'device': device,
            'max_inference_rate_hz': ParameterValue(
                max_inference_rate_hz, value_type=float
            ),
            'publish_debug_image': ParameterValue(
                publish_debug_image, value_type=bool
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='YOLO .pt or TensorRT .engine path; empty uses package model.',
        ),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_raw'
        ),
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('debug_image_topic', default_value='/yolo/debug_image'),
        DeclareLaunchArgument(
            'debug_compressed_image_topic',
            default_value='/yolo/debug_image/compressed',
        ),
        DeclareLaunchArgument('debug_jpeg_quality', default_value='75'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.35'),
        DeclareLaunchArgument('iou_threshold', default_value='0.70'),
        DeclareLaunchArgument('image_size', default_value='640'),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('max_inference_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        yolo_node,
    ])
