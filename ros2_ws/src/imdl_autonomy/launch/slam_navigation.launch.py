"""Nav2 on a map that slam_toolbox is still building: map and drive at once.

No saved map, no AMCL, no 2D Pose Estimate. slam_toolbox owns the /map topic
and the map -> odom TF, Nav2 plans on top of it, and goals come from RViz
(2D Goal Pose). The Jetson runs mapping.launch.py (lidar + scan timestamp fix
+ odom TF + slam_toolbox), this launch file runs the Nav2 servers, and
real_rviz.launch.py runs alongside it for base_link -> lidar_link.

Unexplored area counts as free (global_costmap track_unknown_space: false), so
goals may be placed outside the mapped region; the robot replans as the map
grows. Save the finished map while this is still running:

    ros2 run nav2_map_server map_saver_cli -f /root/maps/my_map

Teleop fallback (use_twist_mux, on by default): Nav2 drives <ns>/cmd_vel_nav
and twist_mux forwards it to <ns>/cmd_vel. Teleop has the higher priority, so
driving by hand overrides Nav2 instantly and Nav2 resumes 0.5 s after the last
key press -- useful for nudging the robot out of a corner, or for filling in a
part of the map Nav2 will not enter:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
      --ros-args -r cmd_vel:=/Team2/cmd_vel_teleop

Needs ros-humble-twist-mux. Pass use_twist_mux:=false to let Nav2 publish
<ns>/cmd_vel directly, as map_navigation.launch.py does.

The collision monitor runs on the lidar scan alone here: this mode is
lidar-only, so its camera-fed tracked_objects source is switched off rather
than left waiting for a cloud that never arrives.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    inflation_radius = LaunchConfiguration('inflation_radius')
    cost_scaling_factor = LaunchConfiguration('cost_scaling_factor')
    use_twist_mux = LaunchConfiguration('use_twist_mux')

    # With the mux in front of the robot, Nav2 may only reach cmd_vel_nav.
    # Without it, Nav2 keeps the direct cmd_vel wiring.
    nav_cmd_vel_topic = PythonExpression([
        "'", namespace, "/cmd_vel_nav' if '", use_twist_mux,
        "' == 'true' else '", namespace, "/cmd_vel'",
    ])

    params_file = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare('imdl_autonomy'), 'config', 'nav2_params.yaml',
        ]),
        root_key='',
        param_rewrites={
            'local_costmap.local_costmap.ros__parameters.inflation_layer.'
            'inflation_radius': inflation_radius,
            'local_costmap.local_costmap.ros__parameters.inflation_layer.'
            'cost_scaling_factor': cost_scaling_factor,
            'global_costmap.global_costmap.ros__parameters.inflation_layer.'
            'inflation_radius': inflation_radius,
            'global_costmap.global_costmap.ros__parameters.inflation_layer.'
            'cost_scaling_factor': cost_scaling_factor,
            'planner_server.ros__parameters.GridBased.cost_travel_multiplier':
                LaunchConfiguration('cost_travel_multiplier'),
            # Mapping with the lidar alone: there is no camera obstacle cloud,
            # so the collision monitor must not keep a source it will never
            # receive. (nav2_params.yaml enables it for the camera missions,
            # where imdl_autonomy's object_registry feeds it.)
            'collision_monitor.ros__parameters.tracked_objects.enabled':
                'false',
        },
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imdl_autonomy'),
                'launch',
                'nav2.launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'true',
            'params_file': params_file,
            'use_sim_time': 'false',
            'odom_topic': [namespace, '/odom'],
            'scan_topic': '/scan',
            'cmd_vel_topic': nav_cmd_vel_topic,
        }.items(),
    )

    # Teleop beats Nav2 (see config/twist_mux.yaml). Runs in the robot
    # namespace, so the relative topic names resolve to <ns>/cmd_vel_nav,
    # <ns>/cmd_vel_teleop and <ns>/cmd_vel.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        namespace=namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('imdl_autonomy'), 'config', 'twist_mux.yaml',
            ]),
            {'use_sim_time': False},
        ],
        remappings=[('cmd_vel_out', 'cmd_vel')],
        condition=IfCondition(use_twist_mux),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='/Team2',
            description='Namespace of the Create3 topics (odom, cmd_vel, tf).',
        ),
        # nav2_params.yaml uses 0.28 / 7.5 / 2.0. The footprint radius is about
        # 0.20 m, so inflation_radius must stay above that.
        DeclareLaunchArgument(
            'inflation_radius', default_value='0.23',
            description='Distance (m) around obstacles with extra cost.',
        ),
        DeclareLaunchArgument(
            'cost_scaling_factor', default_value='10.0',
            description='Higher = cost drops faster = robot may pass closer.',
        ),
        DeclareLaunchArgument(
            'cost_travel_multiplier', default_value='1.0',
            description='Lower = planned paths may run closer to walls.',
        ),
        DeclareLaunchArgument(
            'use_twist_mux', default_value='true', choices=['true', 'false'],
            description=(
                'Route Nav2 through twist_mux so teleop on '
                '<namespace>/cmd_vel_teleop can override it.'
            ),
        ),
        nav2,
        twist_mux,
    ])
