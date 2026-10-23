"""Start one mission against an already running autonomy stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from imdl_autonomy.mission_selection import resolve_mission_file


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='task1'),
        DeclareLaunchArgument(
            'task_number',
            default_value='',
            description='Optional numeric alias for task (1..5).',
        ),
        DeclareLaunchArgument('variant', default_value='a'),
        DeclareLaunchArgument(
            'mission_file',
            default_value='',
            description='Explicit mission YAML; overrides task/variant selection.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'require_ultrasonic',
            default_value='true',
            description=(
                'Require a fresh front Range measurement before simulated PICK.'
            ),
        ),
        DeclareLaunchArgument(
            'cmd_vel_feedback_topic',
            default_value='/Team2/cmd_vel',
        ),
        DeclareLaunchArgument(
            'odom_feedback_topic',
            default_value='/Team2/odom',
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    package = get_package_share_directory('imdl_autonomy')
    task = LaunchConfiguration('task').perform(context)
    task_number = LaunchConfiguration('task_number').perform(context)
    if task_number:
        task = task_number
    mission_file, _, _ = resolve_mission_file(
        package,
        task,
        LaunchConfiguration('variant').perform(context),
        LaunchConfiguration('mission_file').perform(context),
    )

    return [Node(
        package='imdl_autonomy',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            os.path.join(package, 'config', 'solver_params.yaml'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mission_file': mission_file,
                'ultrasonic_required_for_pick': ParameterValue(
                    LaunchConfiguration('require_ultrasonic'),
                    value_type=bool,
                ),
            },
        ],
        remappings=[
            ('cmd_vel_feedback', LaunchConfiguration('cmd_vel_feedback_topic')),
            ('odom_feedback', LaunchConfiguration('odom_feedback_topic')),
        ],
    )]
