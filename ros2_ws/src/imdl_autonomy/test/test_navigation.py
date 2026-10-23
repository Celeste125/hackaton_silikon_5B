from pathlib import Path as FilePath
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
from imdl_autonomy.navigation_adapter import (
    MotionStats,
    NAV2_LIFECYCLE_NODES,
    path_length,
    reverse_path,
    VelocityStats,
)
from imdl_autonomy.next_best_view import NextBestViewPlanner
from nav_msgs.msg import Path
import pytest
import yaml


def _pose(x, yaw_z):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.orientation.z = yaw_z
    pose.pose.orientation.w = 1.0
    return pose


def test_reverse_path_reverses_positions_but_keeps_headings():
    path = Path()
    path.header.frame_id = 'map'
    path.poses = [_pose(0.0, 0.1), _pose(0.5, 0.2), _pose(1.0, 0.3)]

    result = reverse_path(path)

    assert [pose.pose.position.x for pose in result.poses] == [1.0, 0.5, 0.0]
    assert [pose.pose.orientation.z for pose in result.poses] == [0.3, 0.2, 0.1]
    assert result.header.frame_id == 'map'


def test_path_length_measures_planar_arc_length():
    path = Path()
    path.poses = [_pose(0.0, 0.0), _pose(3.0, 0.0)]
    path.poses[1].pose.position.y = 4.0

    assert path_length(path) == pytest.approx(5.0)


def test_navigation_waits_for_every_managed_lifecycle_node():
    assert NAV2_LIFECYCLE_NODES == (
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'collision_monitor',
    )


def test_reverse_mppi_profile_cannot_command_forward_motion():
    config_path = (
        FilePath(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    reverse = config['controller_server']['ros__parameters']['ReverseMPPI']

    assert reverse['vx_min'] < 0.0
    assert reverse['vx_max'] == 0.0
    assert reverse['vx_min'] <= -0.12
    assert reverse['vx_std'] >= 0.08
    assert reverse['PathAngleCritic']['forward_preference'] is False
    assert 'PreferForwardCritic' not in reverse['critics']
    assert 'VelocityDeadbandCritic' in reverse['critics']
    assert reverse['VelocityDeadbandCritic']['deadband_velocities'][0] >= 0.05
    task1_checker = config['controller_server']['ros__parameters'][
        'task1_goal_checker'
    ]
    assert task1_checker['xy_goal_tolerance'] == 0.10
    progress = config['controller_server']['ros__parameters'][
        'progress_checker'
    ]
    assert progress['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert 0.0 < progress['required_movement_angle'] <= 0.15
    assert progress['movement_time_allowance'] <= 8.0


def test_robust_mppi_uses_humble_rotation_shim_contract():
    config_path = (
        FilePath(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    robust = config['controller_server']['ros__parameters']['RobustMPPI']

    assert robust['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController'
    )
    assert robust['primary_controller'] == (
        'nav2_mppi_controller::MPPIController'
    )
    assert robust['angular_disengage_threshold'] < (
        robust['angular_dist_threshold']
    )
    assert robust['rotate_to_goal_heading'] is True
    assert robust['vx_min'] == 0.0
    assert robust['vx_max'] > 0.0
    assert 0.0 < robust['forward_sampling_distance'] <= robust['prune_distance']
    # Nav2 still performs full footprint collision checks inside this radius;
    # only the preferential inflation-gradient penalty is suppressed.
    assert robust['CostCritic']['near_goal_distance'] >= 0.35
    assert robust['GoalAngleCritic']['threshold_to_consider'] <= 0.15
    horizon_distance = (
        robust['time_steps'] * robust['model_dt'] * robust['vx_max']
    )
    assert 0.45 <= horizon_distance < 1.0
    assert robust['batch_size'] >= 1500
    assert robust['vx_std'] >= robust['vx_max']

    general_checker = config['controller_server']['ros__parameters'][
        'general_goal_checker'
    ]
    assert general_checker['xy_goal_tolerance'] <= 0.07
    assert 0.0 < general_checker['yaw_goal_tolerance'] <= 0.20


def test_precise_behavior_tree_selects_precise_goal_checker():
    tree_path = (
        FilePath(__file__).parents[1]
        / 'behavior_trees'
        / 'navigate_mppi_precise.xml'
    )
    tree = ET.parse(tree_path)
    follow_nodes = tree.findall('.//FollowPath')

    assert len(follow_nodes) == 1
    assert follow_nodes[0].attrib['controller_id'] == 'RobustMPPI'
    assert follow_nodes[0].attrib['goal_checker_id'] == (
        'precise_goal_checker'
    )


def test_mission_behavior_tree_stays_inside_full_score_pose_limits():
    package = FilePath(__file__).parents[1]
    tree = ET.parse(
        package / 'behavior_trees' / 'navigate_mppi_mission.xml'
    )
    follow_nodes = tree.findall('.//FollowPath')
    config = yaml.safe_load(
        (package / 'config' / 'nav2_params.yaml').read_text(
            encoding='utf-8'
        )
    )
    checker = config['controller_server']['ros__parameters'][
        'mission_pose_goal_checker'
    ]

    assert len(follow_nodes) == 1
    assert follow_nodes[0].attrib['controller_id'] == 'RobustMPPI'
    assert follow_nodes[0].attrib['goal_checker_id'] == (
        'mission_pose_goal_checker'
    )
    assert 0.0 < checker['xy_goal_tolerance'] < 0.10
    assert 0.0 < checker['yaw_goal_tolerance'] < 0.785398


def test_velocity_stats_report_range():
    stats = VelocityStats()
    for value in (0.0, -0.04, -0.08, -0.02):
        stats.observe(value)

    assert stats.samples == 4
    assert stats.minimum == -0.08
    assert stats.maximum == 0.0
    assert stats.mean == pytest.approx(-0.035)
    assert stats.nonzero_samples == 3
    assert stats.summary() == (
        'n=4,min=-0.080,max=0.000,mean=-0.035,active=3'
    )


def test_motion_stats_track_linear_and_angular_velocity_independently():
    stats = MotionStats()
    stats.observe(0.10, -0.40)
    stats.observe(0.05, 0.90)

    assert stats.linear.minimum == 0.05
    assert stats.linear.maximum == 0.10
    assert stats.linear.mean == pytest.approx(0.075)
    assert stats.angular.minimum == -0.40
    assert stats.angular.maximum == 0.90
    assert stats.angular.mean == pytest.approx(0.25)


def test_rotation_limits_are_fast_and_consistent():
    config_path = (
        FilePath(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    controller = config['controller_server']['ros__parameters']
    behaviors = config['behavior_server']['ros__parameters']
    smoother = config['velocity_smoother']['ros__parameters']

    assert controller['FastRPP']['rotate_to_heading_angular_vel'] == 0.90
    assert controller['FastRPP']['max_angular_accel'] == 2.0
    assert controller['FastRPP']['rotate_to_heading_min_angle'] >= 0.60
    assert controller['RobustMPPI']['rotate_to_heading_angular_vel'] == 0.90
    assert controller['RobustMPPI']['max_angular_accel'] == 2.0
    assert controller['RobustMPPI']['wz_max'] == 0.90
    assert controller['ReverseMPPI']['wz_max'] == 0.75
    assert behaviors['max_rotational_vel'] == 0.75
    assert behaviors['rotational_acc_lim'] == 2.0
    assert smoother['max_velocity'][2] == 0.90
    assert smoother['min_velocity'][2] == -0.90
    assert smoother['max_accel'][2] == 2.0
    assert smoother['max_decel'][2] == -4.0
    assert (
        NextBestViewPlanner.__init__.__defaults__[-1]
        == 0.90
    )
