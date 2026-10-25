"""The autonomy half: scan fix, odometry TF, SLAM, Nav2, explorer, approach.

Pairs with jetson_bringup.launch.py, which owns the sensors and the detector.
It was called laptop_bringup while that was the only place it ran; it runs on
whichever machine carries the autonomy, and on this branch that is usually the
Jetson:

    ros2 launch imdl_autonomy autonomy_bringup.launch.py map_id:=my_map

Two topologies, same file. ONBOARD puts this on the Jetson beside the sensors,
so the lidar scans and the detection JSON never leave the machine -- with
--net=host --ipc=host the container reaches the native detector over FastDDS
shared memory -- and only RViz crosses the WiFi, which is the one thing that
tolerates lag. SPLIT leaves this on the laptop, and then the scans and the
JSON do cross. Pass rviz:=false for ONBOARD and run real_rviz.launch.py on the
laptop instead; X over SSH from the Jetson is not worth the trouble.

What must NOT be split either way is this file's own contents. slam_toolbox,
Nav2 and the approach controller need one TF tree on one clock; splitting that
across the WiFi made slam_toolbox drop every scan while Nav2 missed transforms
by whole seconds. Sensor topics tolerate that lag; TF does not.

Teleop stays a terminal of its own, because it has to read the keyboard:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r cmd_vel:=/Team2/cmd_vel_teleop

Before pressing Enter, confirm odometry is flowing:

    ros2 topic hz /Team2/odom

Nav2 activates its local costmap during bringup and that needs odom ->
base_link. The retry loop does pick up once the frame appears, but the
Create3 publishes no odometry at all while docked or asleep, so check first
rather than watching it wait.
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
