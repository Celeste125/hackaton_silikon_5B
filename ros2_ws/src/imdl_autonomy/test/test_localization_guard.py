import math
from pathlib import Path

from geometry_msgs.msg import TransformStamped
import yaml

from imdl_autonomy.localization_guard import (
    amcl_update_overdue,
    pose_from_transform,
)


def test_old_amcl_pose_is_healthy_while_robot_is_stationary():
    overdue, distance, yaw = amcl_update_overdue(
        age_sec=10.0,
        timeout_sec=1.5,
        current_odom=(0.5, -0.2, 0.3),
        odom_at_last_amcl=(0.5, -0.2, 0.3),
        distance_limit_m=0.02,
        yaw_limit_rad=0.05,
    )

    assert not overdue
    assert distance == 0.0
    assert yaw == 0.0


def test_old_amcl_pose_stops_after_unlocalized_translation():
    overdue, distance, _ = amcl_update_overdue(
        age_sec=2.0,
        timeout_sec=1.5,
        current_odom=(0.04, 0.0, 0.0),
        odom_at_last_amcl=(0.0, 0.0, 0.0),
        distance_limit_m=0.02,
        yaw_limit_rad=0.05,
    )

    assert overdue
    assert math.isclose(distance, 0.04)


def test_old_amcl_pose_stops_after_unlocalized_rotation():
    overdue, _, yaw = amcl_update_overdue(
        age_sec=2.0,
        timeout_sec=1.5,
        current_odom=(0.0, 0.0, -math.pi + 0.04),
        odom_at_last_amcl=(0.0, 0.0, math.pi - 0.04),
        distance_limit_m=0.02,
        yaw_limit_rad=0.05,
    )

    assert overdue
    assert math.isclose(yaw, 0.08, abs_tol=1e-9)


def test_amcl_updates_before_guard_motion_limit():
    package = Path(__file__).parents[1]
    nav2 = yaml.safe_load(
        (package / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8')
    )
    solver = yaml.safe_load(
        (package / 'config' / 'solver_params.yaml').read_text(
            encoding='utf-8'
        )
    )
    amcl = nav2['amcl']['ros__parameters']
    guard = solver['localization_guard']['ros__parameters']

    assert amcl['update_min_d'] < guard['amcl_motion_limit_m']
    assert amcl['update_min_a'] < guard['amcl_motion_limit_rad']


def test_current_pose_is_composed_from_map_to_base_transform():
    transform = TransformStamped()
    transform.transform.translation.x = -0.4
    transform.transform.translation.y = 0.3
    transform.transform.rotation.z = math.sin(0.75 * 0.5)
    transform.transform.rotation.w = math.cos(0.75 * 0.5)

    pose = pose_from_transform(transform)

    assert math.isclose(pose.x, -0.4)
    assert math.isclose(pose.y, 0.3)
    assert math.isclose(pose.yaw, 0.75)
