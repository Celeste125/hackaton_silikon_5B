import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from imdl_autonomy.mission_manager import task_requires_manipulator


PACKAGE = Path(__file__).parents[1]
SIMULATION_PACKAGE = PACKAGE.parent / 'imdl_simulation'


def test_solvers_never_publish_raw_velocity_commands():
    for solver in (PACKAGE / 'imdl_autonomy' / 'solvers').glob('*.py'):
        source = solver.read_text(encoding='utf-8')
        assert 'create_publisher' not in source, solver.name
        assert 'geometry_msgs.msg import Twist' not in source, solver.name
        assert '.publish(' not in source, solver.name


def test_collision_monitor_is_only_final_velocity_output():
    launch = (PACKAGE / 'launch' / 'nav2.launch.py').read_text(
        encoding='utf-8'
    )

    assert "('cmd_vel', 'cmd_vel_nav')" in launch
    assert "('cmd_vel', 'cmd_vel_nav')," in launch
    assert "('cmd_vel_safe', cmd_vel_topic)" in launch
    assert "executable='collision_monitor'" in launch


def test_only_canonical_autonomy_names_remain():
    legacy_topic_prefix = '/' + 'ai' + '/'
    forbidden_names = {
        'mission_executor.py',
        'local_planner.py',
        'mission_model.py',
        'task_runner.py',
        'tasks.yaml',
    }
    source_roots = [
        PACKAGE / 'imdl_autonomy',
        PACKAGE / 'behavior_trees',
        PACKAGE / 'config',
        PACKAGE / 'launch',
        PACKAGE / 'test',
    ]
    for root in source_roots:
        for path in root.rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:
                continue
            assert '_AI' not in path.name, path
            assert path.name not in forbidden_names, path
            if path.suffix in {'.py', '.xml', '.yaml'}:
                source = path.read_text(encoding='utf-8')
                assert legacy_topic_prefix not in source, path


def test_public_and_internal_topic_contract():
    legacy_topic_prefix = '/' + 'ai' + '/'
    manager = (PACKAGE / 'imdl_autonomy' / 'mission_manager.py').read_text(
        encoding='utf-8'
    )
    runtime = (PACKAGE / 'imdl_autonomy' / 'runtime_context.py').read_text(
        encoding='utf-8'
    )

    for topic in (
        '/mission_state',
        '/task_result',
        '/performance_metrics',
    ):
        assert topic in manager
    assert '/tracked_objects' in runtime
    assert '/autonomy/' in runtime
    assert legacy_topic_prefix not in manager + runtime


def test_simulation_laptop_profile_avoids_duplicate_render_work():
    autonomy_launch = (
        SIMULATION_PACKAGE / 'launch' / 'autonomy.launch.py'
    ).read_text(encoding='utf-8')
    xacro = (
        SIMULATION_PACKAGE / 'urdf' / 'create3_imdl.urdf.xacro'
    ).read_text(encoding='utf-8')
    rviz = (
        SIMULATION_PACKAGE / 'rviz' / 'simulation.rviz'
    ).read_text(encoding='utf-8')

    assert "name='simulation_rviz2'" in autonomy_launch
    assert "'real_rviz.launch.py'" not in autonomy_launch
    assert 'value_type=float' in autonomy_launch
    assert 'camera_update_rate' in xacro
    assert '<visualize>$(arg sensor_visualization)</visualize>' in xacro
    assert 'Frame Rate: 20' in rviz
    assert 'Enabled: false\n      Frame Timeout: 15' in rviz

    worlds = list((SIMULATION_PACKAGE / 'worlds').glob('*.world'))
    assert worlds
    for world in worlds:
        source = world.read_text(encoding='utf-8')
        assert '<shadows>false</shadows>' in source, world.name


def test_simulated_camera_renders_from_published_optical_origin():
    xacro_path = (
        SIMULATION_PACKAGE / 'urdf' / 'create3_imdl.urdf.xacro'
    )
    root = ET.parse(xacro_path).getroot()
    optical_origin = root.find(
        "./joint[@name='camera_color_optical_joint']/origin"
    )
    sensor_pose = root.find(
        "./gazebo[@reference='camera_link']/sensor[@name='sim_camera']/pose"
    )
    frame_name = root.find(
        "./gazebo[@reference='camera_link']/sensor[@name='sim_camera']"
        '/plugin/frame_name'
    )

    assert optical_origin is not None
    assert sensor_pose is not None
    assert frame_name is not None
    assert frame_name.text == 'camera_color_optical_frame'
    optical_xyz = tuple(float(value) for value in optical_origin.get('xyz').split())
    sensor_xyz = tuple(float(value) for value in sensor_pose.text.split()[:3])
    assert sensor_xyz == optical_xyz


def test_simulated_lidar_self_filter_preserves_canonical_scan_topic():
    xacro_path = (
        SIMULATION_PACKAGE / 'urdf' / 'create3_imdl.urdf.xacro'
    )
    root = ET.parse(xacro_path).getroot()
    plugin = root.find(
        "./gazebo[@reference='lidar_link']/sensor[@name='lidar']"
        "/plugin[@name='gazebo_ros_laser']"
    )
    assert plugin is not None
    assert plugin.find('ros/remapping').text == '~/out:=/scan_raw'

    filter_script = (
        SIMULATION_PACKAGE / 'scripts' / 'scan_self_filter.py'
    ).read_text(encoding='utf-8')
    gazebo_launch = (
        SIMULATION_PACKAGE / 'launch' / 'imdl_create3_gazebo.launch.py'
    ).read_text(encoding='utf-8')
    nav2 = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(
            encoding='utf-8'
        )
    )
    assert "'/scan_raw'" in filter_script
    assert "'/scan'" in filter_script
    assert 'point_in_polygon' in filter_script
    assert "'scan_self_filter.py'" in gazebo_launch
    assert nav2['amcl']['ros__parameters']['scan_topic'] == 'scan'
    assert (
        nav2['collision_monitor']['ros__parameters']['scan']['topic']
        == 'scan'
    )


def test_front_ultrasonic_uses_canonical_range_contract():
    xacro_path = (
        SIMULATION_PACKAGE / 'urdf' / 'create3_imdl.urdf.xacro'
    )
    root = ET.parse(xacro_path).getroot()
    joint = root.find("./joint[@name='ultrasonic_front_joint']/origin")
    sensor = root.find(
        "./gazebo[@reference='ultrasonic_front_link']"
        "/sensor[@name='front_ultrasonic']"
    )

    assert joint is not None
    assert sensor is not None
    plugin = sensor.find("plugin[@name='gazebo_ros_front_ultrasonic']")
    assert plugin is not None
    assert plugin.find('output_type').text == 'sensor_msgs/LaserScan'
    assert plugin.find('frame_name').text == 'ultrasonic_front_link'
    assert (
        plugin.find('ros/remapping').text
        == '~/out:=/ultrasonic/front/scan_raw'
    )
    joint_xyz = tuple(float(value) for value in joint.get('xyz').split())
    ray_xyz = tuple(float(value) for value in sensor.find('pose').text.split()[:3])
    assert joint_xyz[0] + ray_xyz[0] == pytest.approx(0.190)
    assert joint_xyz[2] + ray_xyz[2] == pytest.approx(0.022)

    readiness = (
        SIMULATION_PACKAGE / 'scripts' / 'simulation_readiness.py'
    ).read_text(encoding='utf-8')
    adapter = (
        SIMULATION_PACKAGE / 'scripts' / 'ultrasonic_range_adapter.py'
    ).read_text(encoding='utf-8')
    gazebo_launch = (
        SIMULATION_PACKAGE / 'launch' / 'imdl_create3_gazebo.launch.py'
    ).read_text(encoding='utf-8')
    rviz = (
        SIMULATION_PACKAGE / 'rviz' / 'simulation.rviz'
    ).read_text(encoding='utf-8')
    assert "'/ultrasonic/front/scan_raw'" in adapter
    assert "'/ultrasonic/front/range'" in adapter
    assert 'Range.ULTRASOUND' in adapter
    assert 'return float(maximum)' in adapter
    assert "'ultrasonic_range_adapter.py'" in gazebo_launch
    assert "'/ultrasonic/front/range'" in readiness
    assert "'ultrasonic_front_link'" in readiness
    assert "'Unable to convert call argument'" in readiness
    assert readiness.count('raw=True') == 5
    assert 'Class: rviz_default_plugins/Range' in rviz
    assert 'Value: /ultrasonic/front/range' in rviz


def test_front_ultrasonic_is_fused_only_through_nav2_safety_layers():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(
            encoding='utf-8'
        )
    )
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config[
        'global_costmap'
    ]['global_costmap']['ros__parameters']
    collision = config['collision_monitor']['ros__parameters']
    nav2_launch = (PACKAGE / 'launch' / 'nav2.launch.py').read_text(
        encoding='utf-8'
    )
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch' / 'autonomy.launch.py'
    ).read_text(encoding='utf-8')
    real_launch = (
        PACKAGE / 'launch' / 'real_autonomy.launch.py'
    ).read_text(encoding='utf-8')

    layer = local['front_ultrasonic_layer']
    assert layer['plugin'] == 'nav2_costmap_2d::RangeSensorLayer'
    assert layer['topics'] == ['/ultrasonic/front/range']
    assert layer['input_sensor_type'] == 'VARIABLE'
    assert layer['clear_on_max_reading'] is True
    assert layer['enabled'] is False
    assert 'front_ultrasonic_layer' in local['plugins']

    # Ultrasound is a manipulation-distance check and fail-safe source, not a
    # mapping sensor. It must not create an inflated local or global trap after
    # an intentionally placed block remains directly in front of the robot.
    assert 'front_ultrasonic_layer' not in global_costmap['plugins']
    assert 'front_ultrasonic_layer' not in global_costmap
    assert (
        'local_costmap.local_costmap.ros__parameters.'
        'front_ultrasonic_layer.enabled'
    ) not in nav2_launch
    assert (
        'global_costmap.global_costmap.ros__parameters.'
        'front_ultrasonic_layer.enabled'
    ) not in nav2_launch
    assert collision['front_ultrasonic']['type'] == 'range'
    assert collision['front_ultrasonic']['topic'] == (
        '/ultrasonic/front/range'
    )
    assert collision['front_ultrasonic']['enabled'] is False
    assert collision['polygons'] == ['StopZone', 'SlowZone']
    assert 'FrontStopZone' not in collision
    assert 'use_front_ultrasonic' in nav2_launch
    assert "'use_front_ultrasonic': 'true'" in simulation_launch
    assert "default_value='false'" in real_launch
    assert "'ultrasonic_required_for_pick': ParameterValue(" in real_launch


def test_collision_monitor_uses_velocity_aware_footprint_approach():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(
            encoding='utf-8'
        )
    )
    collision = config['collision_monitor']['ros__parameters']
    stop = collision['StopZone']

    assert stop['action_type'] == 'approach'
    assert stop['footprint_topic'] == '/local_costmap/published_footprint'
    assert stop['time_before_collision'] == pytest.approx(1.0)
    assert stop['simulation_time_step'] == pytest.approx(0.05)
    assert 'points' not in stop


def test_confirmed_camera_tracks_feed_low_obstacle_safety():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(
            encoding='utf-8'
        )
    )
    registry = (
        PACKAGE / 'imdl_autonomy' / 'object_registry.py'
    ).read_text(encoding='utf-8')
    placed_objects = (
        PACKAGE / 'imdl_autonomy' / 'placed_objects.py'
    ).read_text(encoding='utf-8')
    local_obstacles = config[
        'local_costmap'
    ]['local_costmap']['ros__parameters']['obstacle_layer']
    global_obstacles = config[
        'global_costmap'
    ]['global_costmap']['ros__parameters']['obstacle_layer']
    collision = config['collision_monitor']['ros__parameters']

    assert '/autonomy/tracked_obstacle_cloud' in registry
    assert 'create_cloud_xyz32' in registry
    for layer in (local_obstacles, global_obstacles):
        assert 'tracked_objects' in layer['observation_sources']
        assert layer['tracked_objects']['data_type'] == 'PointCloud2'
        assert layer['tracked_objects']['clearing'] is False
        assert layer['tracked_objects']['marking'] is True
    assert collision['tracked_objects']['type'] == 'pointcloud'
    assert collision['tracked_objects']['topic'] == (
        '/autonomy/tracked_obstacle_cloud'
    )
    assert 'PLACED_OBJECT_EVENT_TOPIC' in registry
    assert '/autonomy/placed_object_event' in placed_objects


def test_sparse_depth_geometry_feeds_low_obstacle_safety():
    nav2 = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8')
    )
    setup_text = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch' / 'autonomy.launch.py'
    ).read_text(encoding='utf-8')

    assert 'depth_obstacle_projector' in setup_text
    assert "executable='depth_obstacle_projector'" in simulation_launch
    local_layer = nav2['local_costmap']['local_costmap']['ros__parameters'][
        'obstacle_layer'
    ]
    assert 'depth_obstacles' in local_layer['observation_sources']
    source = local_layer['depth_obstacles']
    assert source['topic'] == '/autonomy/depth_obstacle_cloud'
    assert source['data_type'] == 'PointCloud2'
    assert source['marking'] is True
    assert source['clearing'] is True

    # Short-lived RGB-D geometry belongs to the rolling local costmap. The
    # global planner uses the map, 360-degree lidar, and confirmed tracks so a
    # moving camera cannot leave transient obstacle trails across the arena.
    global_layer = nav2[
        'global_costmap'
    ]['global_costmap']['ros__parameters']['obstacle_layer']
    assert 'depth_obstacles' not in global_layer['observation_sources']
    assert 'depth_obstacles' not in global_layer
    collision = nav2['collision_monitor']['ros__parameters']
    # Camera delivery can pause under Gazebo load. It therefore informs the
    # rolling local planner but must not act as a hard watchdog that freezes
    # every command while scan, tracks, and ultrasonic remain healthy.
    assert 'depth_obstacles' not in collision['observation_sources']
    assert 'depth_obstacles' not in collision


def test_nav2_bringup_is_staggered_for_reliable_lifecycle_transitions():
    nav2_launch = (
        PACKAGE / 'launch' / 'nav2.launch.py'
    ).read_text(encoding='utf-8')
    simulation_launch = (
        SIMULATION_PACKAGE / 'launch' / 'autonomy.launch.py'
    ).read_text(encoding='utf-8')

    assert 'localization_manager_delay_sec' in nav2_launch
    assert 'navigation_nodes_delay_sec' in nav2_launch
    assert 'navigation_manager_delay_sec' in nav2_launch
    assert nav2_launch.count('TimerAction(') >= 3
    assert "TimerAction(period=5.0, actions=[perception])" in simulation_launch
    assert "TimerAction(period=7.0, actions=support)" in simulation_launch


def test_camera_mount_clears_lidar_and_sees_entire_block_at_pick_ring():
    xacro_path = (
        SIMULATION_PACKAGE / 'urdf' / 'create3_imdl.urdf.xacro'
    )
    root = ET.parse(xacro_path).getroot()
    pitch = float(root.find(
        "./{http://ros.org/wiki/xacro}arg[@name='camera_pitch']"
    ).get('default'))
    camera_origin = tuple(float(value) for value in root.find(
        "./joint[@name='camera_joint']/origin"
    ).get('xyz').split())
    optical_offset = tuple(float(value) for value in root.find(
        "./joint[@name='camera_color_optical_joint']/origin"
    ).get('xyz').split())
    camera_size = tuple(float(value) for value in root.find(
        "./link[@name='camera_link']/collision/geometry/box"
    ).get('size').split())
    lidar_z = float(root.find(
        "./joint[@name='lidar_joint']/origin"
    ).get('xyz').split()[2])

    camera_top = (
        camera_origin[2]
        + 0.5 * camera_size[0] * math.sin(pitch)
        + 0.5 * camera_size[2] * math.cos(pitch)
    )
    assert camera_top < lidar_z

    lens_x = camera_origin[0] + optical_offset[0] * math.cos(pitch)
    lens_z = camera_origin[2] - optical_offset[0] * math.sin(pitch)
    vertical_fov = 2.0 * math.atan(
        math.tan(1.211 * 0.5) * 480.0 / 640.0
    )
    for radius in (0.27, 0.36):
        for block_x in (radius - 0.0175, radius + 0.0175):
            for block_z in (0.0, 0.035):
                downward_angle = math.atan2(
                    lens_z - block_z,
                    block_x - lens_x,
                )
                assert abs(downward_angle - pitch) < vertical_fov * 0.5


def test_manipulator_reach_covers_navigation_goal_tolerance():
    solver = yaml.safe_load(
        (PACKAGE / 'config' / 'solver_params.yaml').read_text(encoding='utf-8')
    )
    nav2 = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8')
    )
    mission = solver['mission_manager']['ros__parameters']
    manipulator = solver['simulation_manipulator']['ros__parameters']
    controller = nav2['controller_server']['ros__parameters']
    general_tolerance = controller['general_goal_checker'][
        'xy_goal_tolerance'
    ]
    precise_tolerance = controller['precise_goal_checker'][
        'xy_goal_tolerance'
    ]

    assert (
        mission['approach_distance_m'] + general_tolerance
        <= manipulator['pick_max_distance_m']
    )
    assert (
        mission['drop_approach_distance_m'] + precise_tolerance
        <= manipulator['place_max_distance_m']
    )
    assert (
        mission['drop_terminal_max_distance_m']
        <= manipulator['place_max_distance_m']
    )
    assert (
        mission['pick_terminal_max_distance_m']
        < manipulator['pick_max_distance_m']
    )
    assert manipulator['entity_pose_tolerance_m'] <= 0.015
    assert manipulator['entity_yaw_tolerance_rad'] <= 0.10


def test_transport_bringup_requires_gazebo_manipulation_services():
    launch = (
        SIMULATION_PACKAGE / 'launch' / 'autonomy.launch.py'
    ).read_text(encoding='utf-8')
    readiness = (
        SIMULATION_PACKAGE / 'scripts' / 'simulation_readiness.py'
    ).read_text(encoding='utf-8')

    assert "'require_manipulation': task in {'task3', 'task4', 'task5'}" in launch
    assert "'/get_entity_state'" in readiness
    assert "'/set_entity_state'" in readiness


def test_only_transport_tasks_wait_for_manipulator_service():
    assert not task_requires_manipulator('task1')
    assert not task_requires_manipulator('task2')
    assert task_requires_manipulator('task3')
    assert task_requires_manipulator('task4')
    assert task_requires_manipulator('task5')

    solver = yaml.safe_load(
        (PACKAGE / 'config' / 'solver_params.yaml').read_text(encoding='utf-8')
    )
    timeout = solver['mission_manager']['ros__parameters'][
        'manipulator_startup_timeout_sec'
    ]
    assert 2.0 <= timeout <= 20.0


def test_manipulation_contract_can_rollback_an_aborted_pick():
    interface = (
        PACKAGE.parent / 'imdl_interfaces' / 'srv' / 'ManipulateObject.srv'
    ).read_text(encoding='utf-8')
    manager = (PACKAGE / 'imdl_autonomy' / 'mission_manager.py').read_text(
        encoding='utf-8'
    )
    manipulator = (
        PACKAGE / 'imdl_autonomy' / 'simulation_manipulator.py'
    ).read_text(encoding='utf-8')

    assert 'uint8 RELEASE=3' in interface
    assert 'self.solver.cancel()' in manager
    assert 'def _release(' in manipulator
    assert 'MutuallyExclusiveCallbackGroup' in manipulator


def test_unknown_obstacle_navigation_uses_live_lidar_and_stable_replanning():
    params = (PACKAGE / 'config' / 'nav2_params.yaml').read_text(
        encoding='utf-8'
    )
    tree = (PACKAGE / 'behavior_trees' / 'navigate_mppi.xml').read_text(
        encoding='utf-8'
    )

    assert 'observation_persistence: 4.0' in params
    assert params.count('inf_is_valid: true') == 2
    assert 'rotate_to_goal_heading: true' in params
    assert 'use_final_approach_orientation: false' in params
    assert 'vx_min: 0.0' in params
    assert 'vx_min: -0.14' in params
    assert 'deadband_velocities: [0.04, 0.0, 0.04]' in params
    assert '<RateController hz="0.5">' in tree

    precise_tree = (
        PACKAGE / 'behavior_trees' / 'navigate_mppi_precise.xml'
    ).read_text(encoding='utf-8')
    assert '<RateController' not in precise_tree
    assert 'NavigatePreciseCommittedPath' in precise_tree

    mission_tree = (
        PACKAGE / 'behavior_trees' / 'navigate_mppi_mission.xml'
    ).read_text(encoding='utf-8')
    assert '<RateController hz="0.5">' in mission_tree
    assert 'goal_checker_id="mission_pose_goal_checker"' in mission_tree

    rviz = (
        SIMULATION_PACKAGE / 'rviz' / 'simulation.rviz'
    ).read_text(encoding='utf-8')
    assert 'Name: LiveGlobalCostmap' in rviz
    assert 'Value: /global_costmap/costmap' in rviz


def test_active_search_closes_camera_heading_blind_spots():
    solver = yaml.safe_load(
        (PACKAGE / 'config' / 'solver_params.yaml').read_text(encoding='utf-8')
    )
    mission = solver['mission_manager']['ros__parameters']

    assert mission['search_sweep_viewpoints'] >= 2
    assert mission['search_sweep_segments'] >= 6
    assert 0.8 <= mission['search_sweep_step_rad'] <= 1.1
    assert (
        mission['search_sweep_segments']
        * mission['search_sweep_step_rad']
        >= 5.8
    )


def test_trial_cli_has_neutral_and_legacy_entry_points():
    setup = (PACKAGE / 'setup.py').read_text(encoding='utf-8')

    assert 'prepare_trial = imdl_autonomy.real_trial:main' in setup
    assert 'prepare_real_trial = imdl_autonomy.real_trial:main' in setup


def test_real_sensor_launch_defaults_to_driver_only_lidar():
    launch = (
        SIMULATION_PACKAGE / 'launch' / 'real_sensors.launch.py'
    ).read_text(encoding='utf-8')

    assert "'lidar_mode'" in launch
    assert "default_value='driver'" in launch
    assert "'lidar.launch.py'" in launch
