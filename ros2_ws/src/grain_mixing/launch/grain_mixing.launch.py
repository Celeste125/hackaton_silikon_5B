from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('grain_mixing'), 'config', 'params.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value='videos/clip_hd.mp4'),
        DeclareLaunchArgument('use_video_source', default_value='true',
                              description='false if a real camera already publishes image_raw'),
        Node(
            package='grain_mixing', executable='video_source', name='video_source',
            parameters=[{'video_path': LaunchConfiguration('video_path')}],
            condition=IfCondition(LaunchConfiguration('use_video_source')),
        ),
        Node(
            package='grain_mixing', executable='grain_analyzer', name='grain_analyzer',
            parameters=[config], output='screen',
        ),
    ])
