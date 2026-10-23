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
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    inflation_radius = LaunchConfiguration('inflation_radius')
    cost_scaling_factor = LaunchConfiguration('cost_scaling_factor')

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
            'cmd_vel_topic': [namespace, '/cmd_vel'],
        }.items(),
    )

    # The collision monitor in nav2_params.yaml lists
    # /autonomy/tracked_obstacle_cloud as a source; object_registry publishes
    # it at 10 Hz (empty without camera) so that source never goes stale.
    object_registry = Node(
        package='imdl_autonomy',
        executable='object_registry',
        name='object_registry',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('imdl_autonomy'), 'config', 'solver_params.yaml',
            ]),
            {'use_sim_time': False},
        ],
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
        nav2,
        object_registry,
    ])
