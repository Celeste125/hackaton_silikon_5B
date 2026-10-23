from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_mode = LaunchConfiguration('lidar_mode')
    use_localization = LaunchConfiguration('use_localization')
    start_rviz = LaunchConfiguration('start_rviz')
    target_frame = LaunchConfiguration('target_frame')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'launch',
                'camera.launch.py',
            ])
        ),
        launch_arguments={
            'use_realsense': 'true',
            'publish_camera_tf': 'true',
            'target_frame': target_frame,
        }.items(),
        condition=IfCondition(use_camera),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam'),
                'launch',
                'lidar.launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and '",
            lidar_mode, "' == 'driver' and '",
            use_localization, "' != 'true'",
        ])),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam'),
                'launch',
                'mapping.launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and '",
            lidar_mode, "' == 'mapping' and '",
            use_localization, "' != 'true'",
        ])),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam'),
                'launch',
                'localization.launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and ('",
            use_localization, "' == 'true' or '",
            lidar_mode, "' == 'localization')",
        ])),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imdl_simulation'),
                'launch',
                'real_rviz.launch.py',
            ])
        ),
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Start the physical RealSense camera and perception pipeline.',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the physical SLLIDAR pipeline.',
        ),
        DeclareLaunchArgument(
            'lidar_mode',
            default_value='driver',
            choices=['driver', 'mapping', 'localization'],
            description=(
                'driver publishes only /scan_raw and /scan; mapping and '
                'localization additionally start their respective stacks.'
            ),
        ),
        DeclareLaunchArgument(
            'use_localization',
            default_value='false',
            description=(
                'Deprecated compatibility switch. true is equivalent to '
                'lidar_mode:=localization.'
            ),
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Open RViz on this machine. Keep false for a headless USB sensor device.',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='Frame used by perception for detected object poses.',
        ),
        camera_launch,
        lidar_launch,
        mapping_launch,
        localization_launch,
        rviz_launch,
    ])
