"""autonomy localization/navigation stack for the physical Team2 Create3."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_file',
            description='Measured real-trial mission YAML containing A/B/C and map.',
        ),
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument(
            'use_front_ultrasonic',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Enable after the Arduino publishes sensor_msgs/Range on '
                '/ultrasonic/front/range.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    import yaml

    package = get_package_share_directory('imdl_autonomy')
    mission_file = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('mission_file').perform(context)
    ))
    if not os.path.isfile(mission_file):
        raise RuntimeError(f'Real autonomy mission file does not exist: {mission_file}')
    with open(mission_file, 'r', encoding='utf-8') as stream:
        mission = yaml.safe_load(stream)
    map_yaml = os.path.expanduser(str(mission.get('map_yaml', '')))
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.abspath(os.path.join(os.path.dirname(mission_file), map_yaml))
    if not os.path.isfile(map_yaml):
        raise RuntimeError(f'Real autonomy map does not exist: {map_yaml}')

    params = os.path.join(package, 'config', 'solver_params.yaml')
    use_front_ultrasonic = LaunchConfiguration('use_front_ultrasonic')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
            'odom_topic': '/Team2/odom',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/Team2/cmd_vel',
            'use_front_ultrasonic': use_front_ultrasonic,
        }.items(),
    )
    remappings = [
        ('scan', '/scan'),
        ('odom', '/Team2/odom'),
        ('hazard_detection', '/Team2/hazard_detection'),
        ('kidnap_status', '/Team2/kidnap_status'),
    ]
    support = [
        Node(
            package='slam',
            executable='odom_tf_republisher',
            name='odom_tf_republisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='imdl_autonomy',
            executable='object_registry',
            name='object_registry',
            output='screen',
            parameters=[params, {'use_sim_time': False}],
        ),
        Node(
            package='imdl_autonomy',
            executable='depth_obstacle_projector',
            name='depth_obstacle_projector',
            output='screen',
            parameters=[params, {'use_sim_time': False}],
        ),
        Node(
            package='imdl_autonomy',
            executable='localization_guard',
            name='localization_guard',
            output='screen',
            parameters=[params, {
                'use_sim_time': False,
                'mission_file': mission_file,
            }],
            remappings=remappings,
        ),
        Node(
            package='imdl_autonomy',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[params, {'use_sim_time': False}],
            remappings=remappings,
        ),
    ]
    mission_manager = Node(
        package='imdl_autonomy',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[params, {
            'use_sim_time': False,
            'mission_file': mission_file,
            'ultrasonic_required_for_pick': ParameterValue(
                use_front_ultrasonic,
                value_type=bool,
            ),
        }],
        remappings=[
            ('cmd_vel_feedback', '/Team2/cmd_vel'),
            ('odom_feedback', '/Team2/odom'),
        ],
    )
    return [
        nav2,
        *support,
        TimerAction(
            period=6.0,
            actions=[mission_manager],
            condition=IfCondition(LaunchConfiguration('start_mission')),
        ),
    ]
