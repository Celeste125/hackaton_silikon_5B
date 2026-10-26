"""Autonomous frontier exploration on top of slam_navigation.launch.py.

Start slam_navigation.launch.py first and wait for "Managed nodes are active";
this adds the explorer, which repeatedly picks the best frontier between known
free space and unknown space and sends it to Nav2 as a navigate_to_pose goal.
It stops on its own once no actionable frontier is left, drives back to the
start, saves the map to <map_dir>/<map_name>.yaml (default ~/maps/arena) and
sets /frontier_explorer/status to "done".

The robot drives itself the moment this starts. Teleop still outranks it:
twist_mux gives <ns>/cmd_vel_teleop priority over everything Nav2 sends, so a
key press takes over, and Ctrl-C here cancels the running goal.

The upstream defaults in frontier_explorer.py were tuned for a car-like rover
in office corridors. The values below are for the Create3 in the arena: it is
small (~0.35 m across) and turns in place, and the arena is a few metres, so
the rover's 1.0 m minimum frontier distance would reject every frontier here
and declare the map finished before moving.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    explorer = Node(
        package='frontier_explorer',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # A frontier run shorter than this many cells is noise, not a gap.
            # 6 cells x 0.05 m = 0.30 m, about one robot width.
            'min_frontier_size': LaunchConfiguration('min_frontier_size'),
            # Upstream used 1.0 m, which exceeds most of this arena.
            'min_frontier_distance':
                LaunchConfiguration('min_frontier_distance'),
            # A few metres at 0.18 m/s, plus replanning; 90 s was for corridors.
            'nav_timeout': LaunchConfiguration('nav_timeout'),
            'goal_blacklist_radius': 0.35,
            'goal_max_dist_cells': 10,
            # The footprint is 0.34 x 0.52 m, and the costmap inflation already
            # keeps the robot off walls, so leave this to the costmap.
            'min_wall_clearance_m': 0.0,
            # Nav2's Spin recovery is valid on a differential drive, unlike the
            # Ackermann rover this came from, so give Nav2 room to recover
            # before the explorer cancels and replans.
            'stuck_dist_m': 0.12,
            'stuck_timeout_s': 12.0,
            'done_ticks_threshold': 5,
            'nav2_startup_delay': LaunchConfiguration('startup_delay'),
            'map_dir': LaunchConfiguration('map_dir'),
            'map_name': LaunchConfiguration('map_name'),
        }],
        # Vestigial on this platform (the startup wiggle is commented out), but
        # pointed through twist_mux rather than at the robot, just in case.
        remappings=[('cmd_vel', [namespace, '/cmd_vel_teleop'])],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='/Team2',
            description='Namespace of the Create3 topics.',
        ),
        DeclareLaunchArgument(
            'min_frontier_size', default_value='6',
            description='Smallest frontier worth driving to, in map cells.',
        ),
        DeclareLaunchArgument(
            'min_frontier_distance', default_value='0.40',
            description='Ignore frontiers closer than this (m).',
        ),
        DeclareLaunchArgument(
            'nav_timeout', default_value='40.0',
            description='Give up on one frontier goal after this long (s).',
        ),
        DeclareLaunchArgument(
            'startup_delay', default_value='5.0',
            description='Wait for Nav2 costmaps before the first goal (s).',
        ),
        DeclareLaunchArgument(
            'map_dir', default_value='~/maps',
            description='Where the finished map is saved.',
        ),
        DeclareLaunchArgument(
            'map_name', default_value='arena',
            description='Map file name: <map_dir>/<map_name>.yaml/.pgm/.posegraph.',
        ),
        explorer,
    ])
