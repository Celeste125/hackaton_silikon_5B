"""Start the robot: everything except navigation.

  ros2 launch robot_bringup bringup.launch.py

  esp_driver_node   load cells, mixer motors, door servo (ESP32 over USB)
  web_panel         the web control panel, http://<jetson>:8000
  cameras           both webcams. camera_mode:=direct (default): the web panel
                    reads them itself, as the cameras' own JPEG frames (about
                    1% CPU each). camera_mode:=ros: v4l2_camera nodes
                    (cameras.launch.py), about 40% CPU each on the Jetson.
  lidar             sllidar_node + scan_timestamp_fixer -> /scan

Switch parts off with web:=false, cameras:=false, lidar:=false. teleop:=true
opens keyboard teleop in an xterm (needs a display, e.g. the Jetson desktop).
The Create 3 base needs nothing here: its firmware publishes /Team2/* and
listens on /Team2/cmd_vel.

Navigation goes on top, when you want it:
  ros2 launch frontier_explorer slam_navigation.launch.py
  ros2 launch frontier_explorer explore.launch.py

Launch from ~/imdl-main/ros2_ws (run sws first): the ESP driver reads
loadcell_calibration.json and door_calibration.json from the current directory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=(arguments or {}).items(),
        condition=IfCondition(LaunchConfiguration(condition)),
    )


BY_PATH = '/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0'


def when(expr):
    """IfCondition on a Python expression over launch arguments, e.g. when(['a', '==', 'b'])."""
    return IfCondition(PythonExpression(expr))


def generate_launch_description():
    web, cams, mode = (LaunchConfiguration(n) for n in ('web', 'cameras', 'camera_mode'))
    direct = ["'", web, "' == 'true' and '", cams, "' == 'true' and '", mode, "' == 'direct'"]
    not_direct = ["'", web, "' == 'true' and not ('", cams, "' == 'true' and '", mode, "' == 'direct')"]
    ros_cams = ["'", cams, "' == 'true' and '", mode, "' == 'ros'"]
    web_panel = dict(package='web_panel', executable='web_panel', name='web_panel', output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('web', default_value='true', choices=['true', 'false'],
                              description='Start the web panel on port 8000.'),
        DeclareLaunchArgument('cameras', default_value='true', choices=['true', 'false'],
                              description='Start both webcams.'),
        DeclareLaunchArgument('camera_mode', default_value='direct', choices=['direct', 'ros'],
                              description='direct: the web panel reads the webcams; ros: v4l2_camera nodes.'),
        DeclareLaunchArgument('camera0_device', default_value=BY_PATH.format(port='2.2'),
                              description='Camera 0 (USB port 2.2).'),
        DeclareLaunchArgument('camera1_device', default_value=BY_PATH.format(port='2.1'),
                              description='Camera 1 (USB port 2.1).'),
        DeclareLaunchArgument('lidar', default_value='true', choices=['true', 'false'],
                              description='Start the lidar (/scan).'),
        DeclareLaunchArgument('teleop', default_value='false', choices=['true', 'false'],
                              description='Open teleop_twist_keyboard in an xterm.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/Team2/cmd_vel',
                              description='Create 3 velocity command topic.'),
        Node(
            package='mixer_esp_driver',
            executable='esp_driver_node',
            name='esp_driver_node',
            output='screen',
        ),
        Node(
            **web_panel,
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'camera_devices': [LaunchConfiguration('camera0_device'),
                                   LaunchConfiguration('camera1_device')],
            }],
            condition=when(direct),
        ),
        Node(
            **web_panel,
            parameters=[{'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}],
            condition=when(not_direct),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'launch', 'cameras.launch.py'])),
            launch_arguments={'camera0_device': LaunchConfiguration('camera0_device'),
                              'camera1_device': LaunchConfiguration('camera1_device')}.items(),
            condition=when(ros_cams),
        ),
        include('frontier_explorer', 'lidar.launch.py', 'lidar'),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])
