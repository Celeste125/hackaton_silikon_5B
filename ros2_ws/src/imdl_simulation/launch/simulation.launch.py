from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = FindPackageShare('imdl_simulation')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    rviz_config = LaunchConfiguration('rviz_config')
    camera_update_rate = LaunchConfiguration('camera_update_rate')
    sensor_visualization = LaunchConfiguration('sensor_visualization')

    create3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_sim,
                'launch',
                'imdl_create3_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'world_path': world,
            'use_gazebo_gui': gui,
            'use_rviz': 'false',
            'spawn_dock': 'false',
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'camera_update_rate': camera_update_rate,
            'sensor_visualization': sensor_visualization,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config,
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_sim, 'worlds', 'arena.world']),
            description='Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo client.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2 with the simulation view.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                pkg_sim,
                'rviz',
                'simulation.rviz',
            ]),
            description='RViz configuration file.',
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
        create3_gazebo,
        arena_markers,
        rviz_node,
    ])
