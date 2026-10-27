"""Nav2 on a map that slam_toolbox is still building: map and drive at once.

No saved map, no AMCL, no 2D Pose Estimate. slam_toolbox owns the /map topic
and the map -> odom TF, Nav2 plans on top of it, and goals come from the
frontier explorer (explore.launch.py) or RViz (2D Goal Pose).

Everything runs on the Jetson beside the lidar. The lidar, with its scan
restamping, comes from the robot bringup:

    ros2 launch robot_bringup bringup.launch.py
    ros2 launch frontier_explorer slam_navigation.launch.py

With run_slam (the default) this launch runs the rest of the TF chain --
odom -> base_link and slam_toolbox's map -> odom -- so scan stamps and TF share
the Jetson's clock. TF lookups inside a message-filter chain have tolerances of 0.2 s,
so they cannot cross a busy Wi-Fi link: splitting the chain between two
machines makes slam_toolbox drop every scan and Nav2 fail every transform.
Pass run_slam:=false to consume a slam_toolbox running elsewhere (e.g.
mapping.launch.py).

Unexplored area counts as free (global_costmap track_unknown_space: false), so
goals may be placed outside the mapped region; the robot replans as the map
grows. Save the finished map while this is still running:

    ros2 run nav2_map_server map_saver_cli -f ~/maps/arena

Teleop fallback (use_twist_mux, on by default): Nav2 drives <ns>/cmd_vel_nav
and twist_mux forwards it to <ns>/cmd_vel. Teleop has the higher priority, so
driving by hand overrides Nav2 instantly and Nav2 resumes 0.5 s after the last
key press -- useful for nudging the robot out of a corner, or for filling in a
part of the map Nav2 will not enter:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
      --ros-args -r cmd_vel:=/Team2/cmd_vel_teleop

Needs ros-humble-twist-mux. Pass use_twist_mux:=false to let Nav2 publish
<ns>/cmd_vel directly.

The collision monitor runs on the lidar scan alone: its point-cloud
tracked_objects source is switched off, since nothing publishes that cloud.
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
            FindPackageShare('frontier_explorer'), 'config', 'nav2_params.yaml',
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
            # Lidar only: nothing publishes the tracked_objects cloud, so the
            # collision monitor must not keep a source it will never receive.
            'collision_monitor.ros__parameters.tracked_objects.enabled':
                'false',
        },
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frontier_explorer'),
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

    # The whole TF chain, so every lookup is local and single-clock.
    # Mirrors mapping.launch.py, minus the lidar driver itself.
    run_slam = IfCondition(LaunchConfiguration('run_slam'))

    # /scan_raw (Jetson) -> /scan stamped with this machine's clock, the same
    # clock that stamps the TF slam_toolbox matches each scan against.
    scan_timestamp_fixer = Node(
        package='frontier_explorer',
        executable='scan_timestamp_fixer',
        name='scan_timestamp_fixer',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('fix_scan_timestamps')),
    )

    # <namespace>/odom -> odom -> base_link TF.
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
        condition=run_slam,
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('frontier_explorer'), 'config', 'slam_toolbox_params.yaml',
            ]),
            {'use_sim_time': False},
        ],
        # Keep slam_toolbox's own /pose off Nav2's topic, as mapping.launch.py does.
        remappings=[('/pose', '/robot_pose')],
        condition=run_slam,
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
                FindPackageShare('frontier_explorer'), 'config', 'twist_mux.yaml',
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
        # Same as nav2_params.yaml. The footprint's corners reach 0.32 m, so
        # inflation_radius must stay above that. 0.23 / 10 / 1.0 left little
        # cost past the footprint and the robot drove into walls.
        DeclareLaunchArgument(
            'inflation_radius', default_value='0.40',
            description='Distance (m) around obstacles with extra cost.',
        ),
        DeclareLaunchArgument(
            'cost_scaling_factor', default_value='5.0',
            description='Higher = cost drops faster = robot may pass closer.',
        ),
        DeclareLaunchArgument(
            'cost_travel_multiplier', default_value='2.0',
            description='Lower = planned paths may run closer to walls.',
        ),
        DeclareLaunchArgument(
            'run_slam', default_value='true', choices=['true', 'false'],
            description=(
                'Run slam_toolbox and the odom TF here, so the whole TF chain '
                'shares one machine and one clock. false expects them '
                'elsewhere, e.g. the Jetson running mapping.launch.py.'
            ),
        ),
        DeclareLaunchArgument(
            'fix_scan_timestamps', default_value='false',
            choices=['true', 'false'],
            description=(
                'Restamp /scan_raw as /scan here too. Only when the lidar runs '
                'with fix_timestamps:=false; the bringup already restamps.'
            ),
        ),
        DeclareLaunchArgument(
            'use_twist_mux', default_value='true', choices=['true', 'false'],
            description=(
                'Route Nav2 through twist_mux so teleop on '
                '<namespace>/cmd_vel_teleop can override it.'
            ),
        ),
        scan_timestamp_fixer,
        odom_tf,
        slam_toolbox,
        nav2,
        twist_mux,
    ])
