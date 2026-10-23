from pathlib import Path

from imdl_autonomy.mission_spec import load_mission, Pose2D
from imdl_autonomy.real_trial import (
    build_trial_payload,
    relative_pose_to_absolute,
    write_validated_trial,
)
import pytest


MAP = Path(__file__).parents[1] / 'maps' / 'arena_unknown.yaml'


def test_body_relative_pose_is_transformed_by_start_yaw():
    start = Pose2D(0.20, -0.30, 0.5 * 3.141592653589793)

    absolute = relative_pose_to_absolute(start, Pose2D(0.40, 0.10, 0.25))

    assert absolute.x == pytest.approx(0.10)
    assert absolute.y == pytest.approx(0.10)
    assert absolute.yaw == pytest.approx(0.5 * 3.141592653589793 + 0.25)


def test_real_task2_file_contains_only_absolute_unknown_obstacle_contract(
    tmp_path,
):
    payload = build_trial_payload(
        mission_id='measured_task2',
        task='task2',
        map_yaml=MAP,
        a=(-0.50, -0.50, 0.0),
        b_relative_to_a=(0.40, 0.10, 0.2),
        c=(0.40, 0.45, 1.57),
    )

    output = write_validated_trial(payload, tmp_path / 'trial.yaml')
    mission = load_mission(output)

    assert mission.coordinate_mode == 'absolute'
    assert mission.obstacle_mode == 'unknown'
    assert mission.map_yaml.resolve() == MAP.resolve()
    assert mission.pose('B').x == pytest.approx(-0.10)
    assert mission.pose('B').y == pytest.approx(-0.40)
    assert mission.pose('B').yaw == pytest.approx(0.2)


def test_real_trial_rejects_pose_too_close_to_arena_wall(tmp_path):
    payload = build_trial_payload(
        mission_id='unsafe_task1',
        task='task1',
        map_yaml=MAP,
        a=(0.0, 0.0, 0.0),
        b=(0.75, 0.0, 0.0),
    )

    with pytest.raises(ValueError, match='Unsafe mission geometry'):
        write_validated_trial(payload, tmp_path / 'unsafe.yaml')
