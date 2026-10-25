"""Everything the laptop runs: scan fix, SLAM, Nav2, explorer, approach, RViz.

One command on the navigation machine, to pair with the Jetson's
jetson_bringup.launch.py:

    ros2 launch imdl_autonomy laptop_bringup.launch.py map_id:=my_map

Teleop stays a third terminal of its own, because it has to read the keyboard:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r cmd_vel:=/Team2/cmd_vel_teleop

Everything here belongs on ONE machine. slam_toolbox, Nav2 and the approach
controller all need the same TF tree, and splitting it across the WiFi made
slam_toolbox drop every scan while Nav2 missed transforms by whole seconds.
Sensor topics tolerate that lag; TF does not.

Before pressing Enter, confirm odometry is flowing:

    ros2 topic hz /Team2/odom

Nav2's local costmap activation fails *permanently* if odom is late, and none
flows while the Create3 is docked or asleep.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/Team2'),
        DeclareLaunchArgument('map_id', default_value='',
                              description='Name of the SLAM map these coordinates belong to.'),
        DeclareLaunchArgument('positions_file', default_value='',
                              description='Empty creates a per-run file under ~/maps/detected_objects.'),
        DeclareLaunchArgument(
            'require_detector', default_value='true', choices=['true', 'false'],
            description='false lets Nav2 drive without the Jetson detector; nothing is measured then.'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Set false and run real_rviz.launch.py elsewhere if X is awkward here.'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159265359'),

        # This pulls in slam_navigation (scan restamp, odom TF, SLAM, Nav2,
        # twist_mux) and the frontier explorer, and wires the Collision
        # Monitor's input to /approach/cmd_vel so the gate sits upstream of it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('imdl_autonomy'), 'launch', 'object_approach.launch.py'])),
            launch_arguments={
                'namespace': namespace,
                'start_navigation': 'true',
                'map_id': LaunchConfiguration('map_id'),
                'positions_file': LaunchConfiguration('positions_file'),
                'require_detector': LaunchConfiguration('require_detector'),
            }.items()),

        # imdl_simulation is deliberately NOT an exec_depend of this package:
        # it already depends on imdl_autonomy, so declaring the reverse makes a
        # colcon cycle. Nothing is needed at build time -- launch files are not
        # compiled -- and that existing dependency already builds this package
        # first, so the share directory is present by the time anyone runs this.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('imdl_simulation'), 'launch', 'real_rviz.launch.py'])),
            launch_arguments={
                'namespace': namespace,
                'model_visual_yaw': '0.0',
                'lidar_yaw': LaunchConfiguration('lidar_yaw'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
