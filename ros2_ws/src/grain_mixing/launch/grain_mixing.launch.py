from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('grain_mixing'), 'config', 'params.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path', default_value='',
            description='video file or camera index ("0") read directly by the analyzer; '
                        'empty: subscribe to the image_raw topic instead'),
        DeclareLaunchArgument('rate', default_value='2.0',
                              description='frames analysed per second'),
        Node(
            package='grain_mixing', executable='grain_analyzer', name='grain_analyzer',
            parameters=[config, {
                'video_path': LaunchConfiguration('video_path'),
                'max_rate': ParameterValue(LaunchConfiguration('rate'), value_type=float),
            }],
            output='screen',
        ),
    ])
