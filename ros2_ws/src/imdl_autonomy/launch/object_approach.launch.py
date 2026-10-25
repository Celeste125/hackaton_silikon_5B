"""Frontier exploration interrupted by local-model detections from the Jetson.

Start the lidar, camera and landmark_detector on the Jetson separately. This
launch runs on the separate Linux laptop and owns the autonomous velocity path.
The lidar must publish /scan_raw (fix_timestamps:=false); the laptop owns
/scan timestamp correction, odometry TF, SLAM, Nav2 and frontier exploration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('imdl_autonomy')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/Team2'),
        DeclareLaunchArgument('positions_file', default_value='',
                              description='New JSON file on the laptop; empty creates a per-run file.'),
        DeclareLaunchArgument('map_id', default_value='',
                              description='Identifier of the SLAM map saved alongside these objects.'),
        DeclareLaunchArgument('start_navigation', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'require_detector', default_value='true', choices=['true', 'false'],
            description=('true stops the rover whenever detection frames go stale: '
                         'with no frames there is no way to notice a target. '
                         'false lets Nav2 drive without the camera machine, for '
                         'bringing up SLAM and goals on their own -- nothing can '
                         'be measured in that mode.')),
        DeclareLaunchArgument('approach_params_file', default_value=PathJoinSubstitution([
            share, 'config', 'object_approach.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'slam_navigation.launch.py'])),
            launch_arguments={
                'namespace': namespace,
                'collision_input_topic': '/approach/cmd_vel',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_navigation'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'frontier_explore.launch.py'])),
            launch_arguments={'namespace': namespace}.items()),
        Node(
            package='imdl_autonomy', executable='object_approach',
            name='object_approach', output='screen',
            parameters=[LaunchConfiguration('approach_params_file'), {
                'use_sim_time': False,
                'odom_topic': [namespace, '/odom'],
                'positions_file': ParameterValue(LaunchConfiguration('positions_file'), value_type=str),
                'map_id': ParameterValue(LaunchConfiguration('map_id'), value_type=str),
                'require_detector_for_navigation': ParameterValue(
                    LaunchConfiguration('require_detector'), value_type=bool),
            }]),
    ])
