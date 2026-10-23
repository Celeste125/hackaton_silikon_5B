"""AMCL + Nav2 on a saved SLAM map for a physical Create3.

No mission file, solver or safety nodes: goals come from RViz (2D Goal Pose),
the command line or a state machine. On startup AMCL gets the start pose
from the initial_x/initial_y/initial_yaw arguments (default: the SLAM start
pose 0 0 0); correct it in RViz with 2D Pose Estimate if needed. Run
real_rviz.launch.py alongside it for base_link -> lidar_link.

Obstacle clearance is tunable without touching the shared nav2_params.yaml:
inflation_radius / cost_scaling_factor (both costmaps) and
cost_travel_multiplier (planner).
"""

import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _initial_pose(context, *args, **kwargs):
    """Publish the start pose a few times while AMCL comes up."""
    x = float(LaunchConfiguration('initial_x').perform(context))
    y = float(LaunchConfiguration('initial_y').perform(context))
    yaw = float(LaunchConfiguration('initial_yaw').perform(context))
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.05
    covariance[35] = 0.07
    message = (
        '{header: {frame_id: map}, pose: {pose: {'
        f'position: {{x: {x}, y: {y}}}, '
        f'orientation: {{z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}}}, '
        f'covariance: {covariance}}}}}'
    )
    return [ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--times', '5', '--rate', '1',
            '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', message,
        ],
        output='log',
    )]


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
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': 'false',
            'odom_topic': [namespace, '/odom'],
            'scan_topic': '/scan',
            'cmd_vel_topic': [namespace, '/cmd_vel'],
        }.items(),
    )

    # <namespace>/odom -> odom -> base_link TF. The Jetson runs only lidar.launch.py
    # in this mode, so this is the single odom TF publisher.
    odom_tf = Node(
        package='slam',
        executable='odom_tf_republisher',
        name='odom_tf_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_topic': [namespace, '/odom'],
            'namespaced_tf_topic': [namespace, '/tf'],
        }],
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
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='Start pose x in the map frame (m).',
        ),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='Start pose y in the map frame (m).',
        ),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Start heading in the map frame (rad).',
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
            'map',
            default_value=os.path.expanduser('~/maps/my_map.yaml'),
            description='Saved SLAM map (YAML) used by map_server and AMCL.',
        ),
        nav2,
        odom_tf,
        object_registry,
        # AMCL activates about 1 s after launch; repeat for 5 s so it is not missed.
        TimerAction(period=2.0, actions=[OpaqueFunction(function=_initial_pose)]),
    ])
