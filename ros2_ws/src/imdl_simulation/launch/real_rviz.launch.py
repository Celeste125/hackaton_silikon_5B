"""RViz for a real run, on the machine that owns the TF tree.

No Image display here may point at a raw camera topic. RViz is the only thing
on the laptop that ever subscribes to one, and a subscriber is the whole cost:
DDS sends nothing until a reader matches, so the Jetson's /webcam/image_raw
stays on the Jetson -- where the detector reads it through shared memory --
purely because nothing here asks for it. Ticking such a display on pulls
320x240 rgb8 at 5 fps across the arena WiFi: 1.15 MB/s fragmented into ~825
UDP datagrams a second, which is most of this robot's share of the air, and
it starves the one stream that genuinely has to cross (/scan_raw).

The Image display is therefore disabled and aimed at the detector's annotated
JPEG, /landmarks/debug_image/compressed (~10 KB a frame, one datagram, and it
has the boxes drawn on it). Turn it on with publish_debug_image:=true on the
detector when you want to see what the model saw, and turn it off again. RViz
rewrites real_create3.rviz when you save a layout, so this note lives here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    model_visual_yaw = LaunchConfiguration('model_visual_yaw')
    lidar_yaw = LaunchConfiguration('lidar_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_sim = FindPackageShare('imdl_simulation')
    create3_xacro = PathJoinSubstitution([
        pkg_sim,
        'urdf',
        'create3_imdl.urdf.xacro',
    ])

    robot_description = {
        'robot_description': Command([
            'xacro',
            ' ',
            create3_xacro,
            ' ',
            'gazebo:=classic',
            ' ',
            'visualize_rays:=false',
            ' ',
            'namespace:=',
            namespace,
            ' ',
            'visual_yaw_offset:=',
            model_visual_yaw,
            ' ',
            'lidar_yaw:=',
            lidar_yaw,
        ])
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='real_create3_robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
        ],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='real_create3_joint_state_publisher',
        condition=IfCondition(use_joint_state_publisher),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='real_create3_rviz2',
        arguments=[
            '-d',
            rviz_config,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='/Team2',
            description='Namespace of the real Create3 robot topics.',
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description=(
                'Keep true for a complete visual robot model when no '
                '/joint_states are available.'
            ),
        ),
        DeclareLaunchArgument(
            'model_visual_yaw',
            default_value='3.14159265359',
            description=(
                'Yaw-only visual correction for the Create3 mesh in RViz. '
                'Does not rotate TF or sensors.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_yaw',
            default_value='0.0',
            description=(
                'Mounting yaw of the physical lidar in base_link (rad). '
                'Use 3.14159265359 when the lidar 0-degree direction '
                'points to the robot rear.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock for RobotModel and RViz.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                pkg_sim,
                'rviz',
                'real_create3.rviz',
            ]),
            description='RViz configuration file.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
