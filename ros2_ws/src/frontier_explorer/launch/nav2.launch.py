"""Nav2 autonomy stack: planner/controller/smoother/collision-monitor command chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package = FindPackageShare('frontier_explorer')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_front_ultrasonic = LaunchConfiguration('use_front_ultrasonic')
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

    # Every server once: run as its own process, or (use_composition, the
    # default) loaded as a component into one container. Each process is a
    # DDS participant that keeps every other one updated; on the Jetson eleven
    # Nav2 processes cost several cores of middleware traffic alone.
    localization = [
        dict(package='nav2_map_server', executable='map_server',
             plugin='nav2_map_server::MapServer', name='map_server',
             parameters=[configured_params, {'yaml_filename': map_yaml}]),
        dict(package='nav2_amcl', executable='amcl',
             plugin='nav2_amcl::AmclNode', name='amcl', parameters=[configured_params]),
    ]
    localization_manager = [dict(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
        remappings=[],
    )]
    navigation = [
        dict(package='nav2_controller', executable='controller_server',
             plugin='nav2_controller::ControllerServer', name='controller_server',
             parameters=[configured_params],
             remappings=topic_remaps + [('cmd_vel', 'cmd_vel_nav')]),
        dict(package='nav2_smoother', executable='smoother_server',
             plugin='nav2_smoother::SmootherServer', name='smoother_server',
             parameters=[configured_params]),
        dict(package='nav2_planner', executable='planner_server',
             plugin='nav2_planner::PlannerServer', name='planner_server',
             parameters=[configured_params]),
        dict(package='nav2_behaviors', executable='behavior_server',
             plugin='behavior_server::BehaviorServer', name='behavior_server',
             parameters=[configured_params],
             remappings=topic_remaps + [('cmd_vel', 'cmd_vel_nav')]),
        dict(package='nav2_bt_navigator', executable='bt_navigator',
             plugin='nav2_bt_navigator::BtNavigator', name='bt_navigator',
             parameters=[configured_params, {
                 'default_nav_to_pose_bt_xml': default_nav_to_pose_tree,
                 'default_nav_through_poses_bt_xml': default_nav_through_poses_tree,
             }]),
        dict(package='nav2_velocity_smoother', executable='velocity_smoother',
             plugin='nav2_velocity_smoother::VelocitySmoother', name='velocity_smoother',
             parameters=[configured_params],
             remappings=topic_remaps + [('cmd_vel', 'cmd_vel_nav'),
                                        ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        dict(package='nav2_collision_monitor', executable='collision_monitor',
             plugin='nav2_collision_monitor::CollisionMonitor', name='collision_monitor',
             parameters=[configured_params],
             remappings=topic_remaps + [('cmd_vel_safe', cmd_vel_topic)]),
    ]
    navigation_manager = [dict(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation',
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
        remappings=[],
    )]

    composition = LaunchConfiguration('use_composition')

    def when(composed, localization_only=False):
        expr = ["'", composition, "' == ('true' if ", str(composed), " else 'false')"]
        if localization_only:  # with slam:=true slam_toolbox gives the map and map -> odom
            expr += [" and '", LaunchConfiguration('slam'), "' != 'true'"]
        return IfCondition(PythonExpression(expr))

    def started(specs, localization_only=False):
        """The servers as processes, and the same servers as components."""
        processes = [
            Node(package=n['package'], executable=n['executable'], name=n['name'],
                 output='screen', parameters=n['parameters'],
                 remappings=n.get('remappings', topic_remaps),
                 condition=when(False, localization_only))
            for n in specs
        ]
        components = LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                ComposableNode(package=n['package'], plugin=n['plugin'], name=n['name'],
                               parameters=n['parameters'],
                               remappings=n.get('remappings', topic_remaps))
                for n in specs
            ],
            condition=when(True, localization_only),
        )
        return processes + [components]

    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',  # a thread per server, as separate processes had
        name='nav2_container',
        output='screen',
        parameters=[configured_params, {'autostart': autostart}],
        condition=when(True),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Saved map for map_server and AMCL; only used with slam:=false.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                package, 'config', 'nav2_params.yaml',
            ]),
            description='Nav2 parameters.',
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
                'source. It does not build the map.'
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
        DeclareLaunchArgument(
            'use_composition', default_value='true', choices=['true', 'false'],
            description='Run all Nav2 servers in one process (far less DDS traffic).',
        ),
        container,
        *started(localization, localization_only=True),
        TimerAction(
            period=LaunchConfiguration('localization_manager_delay_sec'),
            actions=started(localization_manager, localization_only=True),
        ),
        TimerAction(
            period=LaunchConfiguration('navigation_nodes_delay_sec'),
            actions=started(navigation),
        ),
        TimerAction(
            period=LaunchConfiguration('navigation_manager_delay_sec'),
            actions=started(navigation_manager),
        ),
    ])
