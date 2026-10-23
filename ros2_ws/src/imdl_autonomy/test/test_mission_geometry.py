from dataclasses import replace
import math
from pathlib import Path

from imdl_autonomy.mission_geometry import validate_mission_geometry
from imdl_autonomy.mission_spec import load_mission, Pose2D
from imdl_autonomy.next_best_view import OccupancyGrid
import pytest


ROBOT_CLEARANCE_M = 0.205
APPROACH_DISTANCE_M = 0.27


def _issues(mission):
    grid = OccupancyGrid.from_yaml(mission.map_yaml)
    return validate_mission_geometry(
        mission,
        grid,
        robot_clearance_m=ROBOT_CLEARANCE_M,
        approach_distance_m=APPROACH_DISTANCE_M,
    )


def test_all_repository_ai_missions_pass_geometry_preflight():
    mission_dir = Path(__file__).parents[1] / 'config' / 'missions'

    for path in sorted(mission_dir.glob('*.yaml')):
        mission = load_mission(path)
        assert _issues(mission) == (), mission.mission_id


def test_task2_rejects_robot_goal_too_close_to_arena_wall():
    mission_path = (
        Path(__file__).parents[1]
        / 'config'
        / 'missions'
        / 'task2_a.yaml'
    )
    mission = load_mission(mission_path)
    unsafe_poses = dict(mission.poses)
    unsafe_poses['B'] = Pose2D(-0.70, -0.55, 1.570796)
    unsafe_mission = replace(mission, poses=unsafe_poses)

    issues = _issues(unsafe_mission)

    assert any(
        issue.code == 'ROBOT_POSE_IN_COLLISION' and issue.subject == 'B'
        for issue in issues
    )
    assert any(issue.code == 'NO_STATIC_MAP_PATH' for issue in issues)


@pytest.mark.parametrize(
    'task,variant',
    [
        (task, variant)
        for task in ('task1', 'task2')
        for variant in ('a', 'b')
    ],
)
def test_official_navigation_sequence_is_live_map_reachable(task, variant):
    package = Path(__file__).parents[1]
    mission = load_mission(
        package / 'config' / 'missions' / f'{task}_{variant}.yaml'
    )
    live_grid = OccupancyGrid.from_yaml(
        package / 'maps' / f'arena_{task}_{variant}.yaml'
    )
    names = ('A', 'B', 'A') if task == 'task1' else ('A', 'B', 'C')

    for start_name, goal_name in zip(names, names[1:]):
        start = mission.pose(start_name)
        goal = mission.pose(goal_name)
        assert live_grid.is_free(
            start.x,
            start.y,
            ROBOT_CLEARANCE_M,
        ), f'{mission.mission_id}/{start_name} is not footprint-safe'
        assert live_grid.is_free(
            goal.x,
            goal.y,
            ROBOT_CLEARANCE_M,
        ), f'{mission.mission_id}/{goal_name} is not footprint-safe'
        assert math.isfinite(live_grid.path_length(
            start,
            goal,
            ROBOT_CLEARANCE_M,
        )), f'{mission.mission_id}: no path {start_name}->{goal_name}'
