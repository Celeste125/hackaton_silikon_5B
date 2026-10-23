"""Real-like Gazebo, perception, Nav2, and autonomy support nodes without a task."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        DeclareLaunchArgument('mission_file', default_value=''),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument(
            'camera_update_rate',
            default_value='20.0',
            description='Simulated RGB-D update rate in Hz (30.0 is full rate).',
        ),
        DeclareLaunchArgument(
            'ultrasonic_update_rate',
            default_value='20.0',
            description='Simulated front ultrasonic update rate in Hz.',
        ),
        DeclareLaunchArgument(
            'sensor_visualization',
            default_value='false',
            choices=['true', 'false'],
            description='Render lidar rays and camera helpers in Gazebo.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional RViz config override; empty selects the lite view.',
        ),
        DeclareLaunchArgument('readiness_timeout_sec', default_value='120.0'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    import yaml

    simulation_package = get_package_share_directory('imdl_simulation')
    autonomy_package = get_package_share_directory('imdl_autonomy')
    task = LaunchConfiguration('task').perform(context)
    task_number = LaunchConfiguration('task_number').perform(context)
    if task_number:
        task = task_number
    variant = LaunchConfiguration('variant').perform(context)
    mission_file, task, variant = resolve_mission_file(
        autonomy_package,
        task,
        variant,
        LaunchConfiguration('mission_file').perform(context),
    )
    with open(mission_file, 'r', encoding='utf-8') as stream:
        mission = yaml.safe_load(stream)
    start = mission['poses']['A']
    map_yaml = str(mission['map_yaml'])
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.abspath(os.path.join(os.path.dirname(mission_file), map_yaml))
    world = LaunchConfiguration('world').perform(context)
    if not world:
        world = os.path.join(
            simulation_package,
            'worlds',
            f'arena_{task}_{variant}.world',
        )
    if not os.path.isfile(world):
        raise RuntimeError(f'Gazebo world does not exist: {world}')
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config:
        rviz_config = os.path.join(
            simulation_package,
            'rviz',
            'simulation.rviz',
        )
    if not os.path.isfile(rviz_config):
        raise RuntimeError(f'RViz config does not exist: {rviz_config}')

    create3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            simulation_package,
            'launch',
            'imdl_create3_gazebo.launch.py',
        )),
        launch_arguments={
            'world_path': world,
            'use_gazebo_gui': LaunchConfiguration('gui'),
            'use_rviz': 'false',
            'spawn_dock': 'false',
            # Bringup adds separators itself. A leading slash produces invalid
            # Gazebo transport names such as //Team2/create3/joint_cmd.
            'namespace': 'Team2',
            'x': str(start['x']),
            'y': str(start['y']),
            'z': '0.0',
            'yaw': str(start.get('yaw', 0.0)),
            'camera_update_rate': LaunchConfiguration('camera_update_rate'),
            'ultrasonic_update_rate': LaunchConfiguration(
                'ultrasonic_update_rate'
            ),
            'sensor_visualization': LaunchConfiguration('sensor_visualization'),
        }.items(),
    )
    arena_markers = Node(
        package='imdl_simulation',
        executable='arena_marker_publisher.py',
        name='arena_marker_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_path': world,
            'frame_id': 'map',
        }],
    )
    odom_tf = Node(
        package='slam',
        executable='odom_tf_republisher',
        name='odom_tf_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Broadcast at the continuously advancing Gazebo clock. Create3
            # odometry is slower than the laser; reusing its stamp creates a
            # sparse TF cache and drops otherwise valid scans between updates.
            'use_message_stamp': False,
            'stamp_backdate_sec': 0.0,
            # The official namespaced Create3 launch already publishes the
            # zero-offset bridge odom -> Team2/odom and
            # Team2/base_link -> base_link. Publish only the moving link.
            'parent_frame_id': 'Team2/odom',
            'child_frame_id': 'Team2/base_link',
        }],
    )
    static_tf_relay = Node(
        package='imdl_simulation',
        executable='static_tf_relay.py',
        name='static_tf_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    readiness_gate = Node(
        package='imdl_simulation',
        executable='simulation_readiness.py',
        name='simulation_readiness',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'timeout_sec': ParameterValue(
                LaunchConfiguration('readiness_timeout_sec'),
                value_type=float,
            ),
            'require_camera': LaunchConfiguration('perception'),
            'require_ultrasonic': task in {'task3', 'task4', 'task5'},
            'require_manipulation': task in {'task3', 'task4', 'task5'},
        }],
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('perception'),
            'launch',
            'camera.launch.py',
        )),
        launch_arguments={
            'use_realsense': 'false',
            'publish_camera_tf': 'false',
            'detector_backend': 'hsv',
            'target_frame': 'map',
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('perception')),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            autonomy_package,
            'launch',
            'nav2.launch.py',
        )),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'true',
            'odom_topic': '/Team2/odom',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/Team2/cmd_vel',
            'use_front_ultrasonic': 'true',
        }.items(),
    )
    params = os.path.join(autonomy_package, 'config', 'solver_params.yaml')
    common_remappings = [
        ('scan', '/scan'),
        ('odom', '/Team2/odom'),
        ('hazard_detection', '/Team2/hazard_detection'),
        ('kidnap_status', '/Team2/kidnap_status'),
    ]
    localization_support = Node(
        package='imdl_autonomy',
        executable='localization_guard',
        name='localization_guard',
        output='screen',
        parameters=[params, {
            'use_sim_time': True,
            'mission_file': mission_file,
        }],
        remappings=common_remappings,
    )
    support = [
        Node(
            package='imdl_autonomy',
            executable='depth_obstacle_projector',
            name='depth_obstacle_projector',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('perception')),
        ),
        Node(
            package='imdl_autonomy',
            executable='object_registry',
            name='object_registry',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='imdl_autonomy',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
            remappings=common_remappings,
        ),
        Node(
            package='imdl_autonomy',
            executable='simulation_manipulator',
            name='simulation_manipulator',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
    ]
    # The Gazebo spawn launch already owns robot_state_publisher and
    # joint_state_publisher. Start only RViz here to avoid duplicate TF and
    # robot-description work.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='simulation_rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    def start_autonomy_after_readiness(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=(
                    'ERROR: Gazebo readiness gate failed; Nav2 will not '
                    'start.'
                )),
                EmitEvent(event=Shutdown(
                    reason='Gazebo sensor/TF readiness failed'
                )),
            ]
        return [
            LogInfo(msg=(
                'Gazebo ready; starting localization-first Nav2 bringup.'
            )),
            nav2,
            TimerAction(period=2.0, actions=[localization_support]),
            TimerAction(period=5.0, actions=[perception]),
            TimerAction(period=7.0, actions=support),
            TimerAction(period=8.0, actions=[rviz]),
        ]

    return [
        create3,
        arena_markers,
        odom_tf,
        static_tf_relay,
        RegisterEventHandler(OnProcessExit(
            target_action=readiness_gate,
            on_exit=start_autonomy_after_readiness,
        )),
        readiness_gate,
    ]
