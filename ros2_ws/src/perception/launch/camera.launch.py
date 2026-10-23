"""Formal IMDL RGB-D perception pipeline launch."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_config_file = LaunchConfiguration('perception_config_file')
    runtime_mode = LaunchConfiguration('runtime_mode')
    detector_backend = LaunchConfiguration('detector_backend')
    use_realsense = LaunchConfiguration('use_realsense')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    target_frame = LaunchConfiguration('target_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'unite_imu_method': '0',
            'initial_reset': 'true',
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
        }.items(),
    )
    # Isolate the RealSense launch arguments from perception-only arguments.
    realsense_node = GroupAction(
        actions=[realsense_launch],
        scoped=True,
        forwarding=False,
        condition=IfCondition(use_realsense),
    )

    depth_align_node = Node(
        package='perception',
        executable='depth_align',
        name='depth_align',
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'PYTHONNOUSERSITE': '1'},
        output='screen',
    )
    yolo_node = Node(
        package='perception',
        executable='yolo_segmentation',
        name='yolo_segmentation',
        parameters=[
            perception_config_file,
            {
                'publish_debug_image': ParameterValue(
                    PythonExpression(["'", runtime_mode, "' == 'debug'"]),
                    value_type=bool,
                ),
                'use_sim_time': use_sim_time,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", detector_backend, "' == 'yolo'",
        ])),
        output='screen',
    )
    hsv_node = Node(
        package='perception',
        executable='hsv_segmentation',
        name='hsv_segmentation',
        parameters=[{
            'publish_debug_image': ParameterValue(
                PythonExpression(["'", runtime_mode, "' == 'debug'"]),
                value_type=bool,
            ),
            'use_sim_time': use_sim_time,
        }],
        additional_env={'PYTHONNOUSERSITE': '1'},
        condition=IfCondition(PythonExpression([
            "'", detector_backend, "' == 'hsv'",
        ])),
        output='screen',
    )
    target_pose_node = Node(
        package='perception',
        executable='target_pose_estimator',
        name='target_pose_estimator',
        parameters=[
            perception_config_file,
            {
                'target_frame': target_frame,
                'use_sim_time': use_sim_time,
            },
        ],
        additional_env={'PYTHONNOUSERSITE': '1'},
        output='screen',
    )
    target_selector_node = Node(
        package='perception',
        executable='target_selector',
        name='target_selector',
        parameters=[
            perception_config_file,
            {
                'target_frame': target_frame,
                'use_sim_time': use_sim_time,
            },
        ],
        additional_env={'PYTHONNOUSERSITE': '1'},
        output='screen',
    )
    visualizer_node = Node(
        package='perception',
        executable='perception_visualizer',
        name='perception_visualizer',
        parameters=[
            perception_config_file,
            {
                'publish_fused_debug': ParameterValue(
                    PythonExpression(["'", runtime_mode, "' == 'debug'"]),
                    value_type=bool,
                ),
                'map_frame': target_frame,
                'use_sim_time': use_sim_time,
            },
        ],
        additional_env={'PYTHONNOUSERSITE': '1'},
        output='screen',
    )
    beacon_heading_node = Node(
        package='perception',
        executable='beacon_heading_controller',
        name='beacon_heading_controller',
        parameters=[
            perception_config_file,
            {'use_sim_time': use_sim_time},
        ],
        additional_env={'PYTHONNOUSERSITE': '1'},
        output='screen',
    )
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_camera_tf),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'perception_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('perception'), 'config', 'perception.yaml'
            ]),
            description='Shared runtime YAML for YOLO, geometry, selection, and beacon nodes.',
        ),
        DeclareLaunchArgument(
            'runtime_mode',
            default_value='deployment',
            description='Use debug to publish /yolo/debug_image; deployment disables it.',
        ),
        DeclareLaunchArgument(
            'detector_backend',
            default_value='yolo',
            choices=['yolo', 'hsv'],
            description='Detection frontend; Gazebo uses hsv, hardware uses yolo.',
        ),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Start the physical RealSense camera driver.',
        ),
        DeclareLaunchArgument(
            'publish_camera_tf',
            default_value='true',
            description='Publish the static base_link to camera_link transform.',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='Frame used for detected object poses.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for all perception nodes.',
        ),
        DeclareLaunchArgument('camera_x', default_value='0.19'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='-0.02'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.262'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        realsense_node,
        depth_align_node,
        yolo_node,
        hsv_node,
        target_pose_node,
        target_selector_node,
        visualizer_node,
        beacon_heading_node,
        camera_tf_node,
    ])
