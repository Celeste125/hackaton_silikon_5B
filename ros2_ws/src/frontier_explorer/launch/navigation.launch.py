"""Drive on a saved map: map_server + AMCL localization + Nav2.

    ros2 launch robot_bringup bringup.launch.py           # robot, lidar, panel
    ros2 launch frontier_explorer navigation.launch.py    # this

The map is the one explore.launch.py saved, ~/maps/arena.yaml by default
(map:=/path/to/other.yaml). Nothing is mapped any more; AMCL matches the lidar
against the saved map to know where the robot is.

Where the robot starts: the map's (0, 0, yaw 0) is where the robot stood when
it was mapped. Put it back on that start mark and AMCL starts from the right
pose, no RViz "2D Pose Estimate" needed. Started somewhere else, pass
initial_x:= initial_y:= initial_yaw:= (metres, radians, in the map frame), or
set the pose in RViz.

Goals come from RViz (2D Goal Pose) or any navigate_to_pose client, e.g.:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"

Teleop on <ns>/cmd_vel_teleop overrides Nav2 through twist_mux, as in
slam_navigation.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_twist_mux = LaunchConfiguration('use_twist_mux')
    share = FindPackageShare('frontier_explorer')

    nav_cmd_vel_topic = PythonExpression([
        "'", namespace, "/cmd_vel_nav' if '", use_twist_mux,
        "' == 'true' else '", namespace, "/cmd_vel'",
    ])

    params_file = RewrittenYaml(
        source_file=PathJoinSubstitution([share, 'config', 'nav2_params.yaml']),
        root_key='',
        param_rewrites={
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_yaw'),
            # Lidar only: nothing publishes the tracked_objects cloud.
            'collision_monitor.ros__parameters.tracked_objects.enabled': 'false',
        },
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'slam': 'false',  # map_server + AMCL instead of slam_toolbox
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': 'false',
            'odom_topic': [namespace, '/odom'],
            'scan_topic': '/scan',
            'cmd_vel_topic': nav_cmd_vel_topic,
        }.items(),
    )

    # <namespace>/odom -> odom -> base_link TF; AMCL adds map -> odom.
    odom_tf = Node(
        package='frontier_explorer',
        executable='odom_tf_republisher',
        name='odom_tf_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_topic': [namespace, '/odom'],
            'namespaced_tf_topic': [namespace, '/tf'],
        }],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        namespace=namespace,
        output='screen',
        parameters=[PathJoinSubstitution([share, 'config', 'twist_mux.yaml']), {'use_sim_time': False}],
        remappings=[('cmd_vel_out', 'cmd_vel')],
        condition=IfCondition(use_twist_mux),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/maps/arena.yaml'),
            description='Saved map (nav2 map_server yaml).',
        ),
        DeclareLaunchArgument('initial_x', default_value='0.0',
                              description='Start pose x in the map frame (m).'),
        DeclareLaunchArgument('initial_y', default_value='0.0',
                              description='Start pose y in the map frame (m).'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0',
                              description='Start heading in the map frame (rad).'),
        DeclareLaunchArgument('namespace', default_value='/Team2',
                              description='Namespace of the Create3 topics (odom, cmd_vel, tf).'),
        DeclareLaunchArgument(
            'use_twist_mux', default_value='true', choices=['true', 'false'],
            description='Route Nav2 through twist_mux so teleop can override it.',
        ),
        odom_tf,
        nav2,
        twist_mux,
    ])
