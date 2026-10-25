"""Make sure the camera, not just the lidar, has looked at the whole arena.

The frontier explorer finishes when the lidar has mapped everything. The lidar
sees 360 degrees and the camera about 70 out of the front, so a finished
frontier run has usually had the camera over a third of the floor -- and the
dispenser may never have been in frame at all.

Start this at the same time as the explorer. It accumulates camera coverage
from the beginning whoever is driving, and only takes the wheel once the
explorer reports itself done:

    ros2 launch imdl_autonomy slam_navigation.launch.py run_slam:=true
    ros2 launch imdl_autonomy landmark_mapping.launch.py publish_camera_tf:=false
    ros2 launch imdl_autonomy frontier_explore.launch.py
    ros2 launch imdl_autonomy camera_coverage.launch.py

Watch it with:

    ros2 topic echo /camera_coverage/status

and in RViz add a Map display on /camera_coverage -- the navigation profile
already carries one as 'Camera Coverage'. Floor the camera has not looked at
shows as occupied; floor it has is clear. It should drain as the robot drives.

The two explorers never drive at once: this one waits for
/frontier_explorer/done. Use start_mode:=immediate only when the frontier
explorer is not running, and start_mode:=observe_only to measure coverage
without ever taking over.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    coverage = Node(
        package='imdl_autonomy',
        executable='camera_coverage_explorer',
        name='camera_coverage_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'camera_fov_deg': LaunchConfiguration('camera_fov_deg'),
            'camera_min_range_m': LaunchConfiguration('camera_min_range_m'),
            'camera_max_range_m': LaunchConfiguration('camera_max_range_m'),
            'camera_bearing_offset_deg':
                LaunchConfiguration('camera_bearing_offset_deg'),
            'coverage_target': LaunchConfiguration('coverage_target'),
            'start_mode': LaunchConfiguration('start_mode'),
            'standoff_m': LaunchConfiguration('standoff_m'),
            'clearance_m': LaunchConfiguration('clearance_m'),
            'nav_timeout_sec': LaunchConfiguration('nav_timeout_sec'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'camera_fov_deg', default_value='70.4',
            description='C920s horizontal field of view.',
        ),
        DeclareLaunchArgument(
            'camera_min_range_m', default_value='0.30',
            description='Nearer than this the object overflows the frame.',
        ),
        DeclareLaunchArgument(
            'camera_max_range_m', default_value='3.0',
            description=(
                'How far the MODEL can still recognise something, not how far '
                'the lens sees. Counting floor beyond that as covered would '
                'be a lie: at 5 m a dispenser is a handful of pixels. Lower '
                'it if the detector only fires close up.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_bearing_offset_deg', default_value='0.0',
            description=(
                'Where the camera points relative to base_link x. Zero for '
                'the front-facing mount this robot has.'
            ),
        ),
        DeclareLaunchArgument(
            'coverage_target', default_value='0.92',
            description=(
                'Stop at this fraction of known-free floor having been in '
                'frame. Not 1.0: the last few cells are usually tucked '
                'against a wall where the robot cannot get an angle, and '
                'chasing them costs more time than they are worth.'
            ),
        ),
        DeclareLaunchArgument(
            'start_mode', default_value='wait_for_frontier',
            choices=['wait_for_frontier', 'immediate', 'observe_only'],
            description=(
                'wait_for_frontier: accumulate coverage now, drive once the '
                'frontier explorer is done. immediate: drive at once -- only '
                'when the frontier explorer is NOT running, or the two will '
                'preempt each other at the Nav2 action server. observe_only: '
                'measure and publish, never drive.'
            ),
        ),
        DeclareLaunchArgument(
            'standoff_m', default_value='1.2',
            description='How far from unlooked-at floor to stand to see it.',
        ),
        DeclareLaunchArgument(
            'clearance_m', default_value='0.25',
            description='Free radius a standing pose needs. ~robot radius.',
        ),
        DeclareLaunchArgument(
            'nav_timeout_sec', default_value='40.0',
            description='Give up on one coverage goal after this long.',
        ),
        coverage,
    ])
