import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_domain_id',
            default_value='42',
            description='ROS_DOMAIN_ID used only by this real-like simulation launch.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description=(
                'Explicit Gazebo world file. If empty, task/variant or '
                'arena.world is used.'
            ),
        ),
        DeclareLaunchArgument(
            'task',
            default_value='',
            description='Optional task world selector: task1, task2, task3, task4, task5.',
        ),
        DeclareLaunchArgument(
            'variant',
            default_value='a',
            description='Task world variant when task is set.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz in the same simulation ROS_DOMAIN_ID.',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Run slam_toolbox from the simulated /scan topic.',
        ),
        DeclareLaunchArgument(
            'perception',
            default_value='true',
            description='Run the perception pipeline from simulated camera topics.',
        ),
        DeclareLaunchArgument(
            'camera_update_rate',
            default_value='20.0',
            description='Simulated RGB-D update rate in Hz.',
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
            description='Optional RViz config override.',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial Create3 x position in meters.',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial Create3 y position in meters.',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Initial Create3 z position in meters.',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial Create3 yaw in radians.',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('sim_domain_id')),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    pkg_sim_path = get_package_share_directory('imdl_simulation')
    pkg_slam_path = get_package_share_directory('slam')

    world = LaunchConfiguration('world').perform(context)
    task = LaunchConfiguration('task').perform(context)
    variant = LaunchConfiguration('variant').perform(context)

    if not world:
        if task:
            world = os.path.join(pkg_sim_path, 'worlds', f'arena_{task}_{variant}.world')
        else:
            world = os.path.join(pkg_sim_path, 'worlds', 'arena.world')
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not rviz_config:
        rviz_config = os.path.join(pkg_sim_path, 'rviz', 'simulation.rviz')
    if not os.path.isfile(rviz_config):
        raise RuntimeError(f'RViz config does not exist: {rviz_config}')

    create3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imdl_simulation'),
                'launch',
                'imdl_create3_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'world_path': world,
            'use_gazebo_gui': LaunchConfiguration('gui'),
            'use_rviz': 'false',
            'spawn_dock': 'false',
            # Bringup adds separators itself. A leading slash produces invalid
            # Gazebo transport names such as //Team2/create3/joint_cmd.
            'namespace': 'Team2',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'camera_update_rate': LaunchConfiguration('camera_update_rate'),
            'sensor_visualization': LaunchConfiguration('sensor_visualization'),
        }.items(),
    )

    arena_markers = Node(
        package='imdl_simulation',
        executable='arena_marker_publisher.py',
        name='arena_marker_publisher',
        parameters=[{
            'use_sim_time': True,
            'world_path': world,
        }],
        output='screen',
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'launch',
                'camera.launch.py',
            ])
        ),
        launch_arguments={
            'use_realsense': 'false',
            'publish_camera_tf': 'false',
            'detector_backend': 'hsv',
            'target_frame': 'map',
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('perception')),
    )

    odom_tf = Node(
        package='slam',
        executable='odom_tf_republisher',
        name='odom_tf_republisher',
        parameters=[{
            'use_sim_time': True,
            'use_message_stamp': False,
            'parent_frame_id': 'Team2/odom',
            'child_frame_id': 'Team2/base_link',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('slam')),
    )
    static_tf_relay = Node(
        package='imdl_simulation',
        executable='static_tf_relay.py',
        name='static_tf_relay',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(pkg_slam_path, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('/pose', '/robot_pose')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='simulation_rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        create3_gazebo,
        arena_markers,
        TimerAction(period=2.0, actions=[static_tf_relay]),
        TimerAction(period=5.0, actions=[odom_tf]),
        TimerAction(period=6.0, actions=[slam_toolbox]),
        TimerAction(period=8.0, actions=[perception]),
        TimerAction(period=9.0, actions=[rviz]),
    ]
