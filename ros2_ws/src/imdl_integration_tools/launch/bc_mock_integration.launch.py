"""Launch the hardware-free B/C typed-object interface demonstration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    scenario = LaunchConfiguration('scenario')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Optional namespace, for example /demo.',
        ),
        DeclareLaunchArgument(
            'scenario', default_value='mixed',
            description='Mock frame: mixed or empty.',
        ),
        DeclareLaunchArgument('publish_rate_hz', default_value='2.0'),
        Node(
            package='imdl_integration_tools',
            executable='mock_detected_objects_publisher',
            name='mock_detected_objects_publisher',
            namespace=namespace,
            parameters=[{
                'scenario': scenario,
                'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='imdl_integration_tools',
            executable='navigation_object_filter',
            name='navigation_object_filter',
            namespace=namespace,
            output='screen',
        ),
        Node(
            package='imdl_integration_tools',
            executable='control_object_filter',
            name='control_object_filter',
            namespace=namespace,
            output='screen',
        ),
    ])
