"""Nav2 autonomy stack: planner/controller/smoother/collision-monitor command chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package = FindPackageShare('imdl_autonomy')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_front_ultrasonic = LaunchConfiguration('use_front_ultrasonic')
    # With slam:=true the map and the map -> odom TF come from slam_toolbox,
    # so map_server and AMCL must stay off.
    without_slam = UnlessCondition(LaunchConfiguration('slam'))
    default_nav_to_pose_tree = PathJoinSubstitution([
        package, 'behavior_trees', 'navigate_rpp.xml',
    ])
    default_nav_through_poses_tree = PathJoinSubstitution([
        package, 'behavior_trees', 'navigate_through_poses.xml',
    ])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                (
                    'collision_monitor.ros__parameters.'
                    'front_ultrasonic.enabled'
                ): use_front_ultrasonic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    topic_remaps = [
        ('scan', scan_topic),
        ('/scan', scan_topic),
        ('odom', odom_topic),
        ('/odom', odom_topic),
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml}],
            remappings=topic_remaps,
            condition=without_slam,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps,
            condition=without_slam,
        ),
    ]
    localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=without_slam,
    )
    navigation_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {
                'default_nav_to_pose_bt_xml': default_nav_to_pose_tree,
                'default_nav_through_poses_bt_xml':
                    default_nav_through_poses_tree,
            }],
            remappings=topic_remaps,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_params],
            remappings=topic_remaps + [
                ('cmd_vel_safe', cmd_vel_topic),
            ],
        ),
    ]
    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                package, 'maps', 'arena_task1_a.yaml',
            ]),
            description='Known static map without task-object leakage.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                package, 'config', 'nav2_params.yaml',
            ]),
            description='Humble-compatible Nav2 autonomy parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Take map and map -> odom TF from a running slam_toolbox '
                'instead of starting map_server and AMCL.'
            ),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/Team2/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/Team2/cmd_vel'),
        DeclareLaunchArgument(
            'use_front_ultrasonic',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Use /ultrasonic/front/range as a short-range Collision Monitor '
                'and manipulation-distance source. It does not build the map.'
            ),
        ),
        DeclareLaunchArgument(
            'localization_manager_delay_sec',
            default_value='1.0',
            description=(
                'Delay lifecycle requests until map_server and AMCL have '
                'created their services.'
            ),
        ),
        DeclareLaunchArgument(
            'navigation_nodes_delay_sec',
            default_value='3.0',
            description='Start navigation servers after localization nodes.',
        ),
        DeclareLaunchArgument(
            'navigation_manager_delay_sec',
            default_value='6.0',
            description=(
                'Delay navigation lifecycle requests until all servers are '
                'responsive; useful on resource-constrained computers.'
            ),
        ),
        *localization_nodes,
        TimerAction(
            period=LaunchConfiguration('localization_manager_delay_sec'),
            actions=[localization_manager],
        ),
        TimerAction(
            period=LaunchConfiguration('navigation_nodes_delay_sec'),
            actions=navigation_nodes,
        ),
        TimerAction(
            period=LaunchConfiguration('navigation_manager_delay_sec'),
            actions=[navigation_manager],
        ),
    ])
