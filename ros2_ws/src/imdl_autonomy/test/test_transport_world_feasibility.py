"""Offline physical-solvability checks for official transport fixtures."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from imdl_autonomy.map_from_world import parse_pose
from imdl_autonomy.mission_spec import Pose2D, load_mission
from imdl_autonomy.next_best_view import NextBestViewPlanner, OccupancyGrid
from imdl_autonomy.solvers.transport import select_diverse_approaches

import pytest


ROOT = Path(__file__).parents[2]
AUTONOMY = Path(__file__).parents[1]
WORLDS = ROOT / 'imdl_simulation' / 'worlds'
ROBOT_CLEARANCE_M = 0.205


def _block_poses(world_path):
    result = {}
    root = ET.parse(world_path).getroot()
    for model in root.findall('.//model'):
        name = model.get('name', '')
        if not name.endswith('_block'):
            continue
        x, y, *_ = parse_pose(model.findtext('pose'))
        result[name.removesuffix('_block')] = Pose2D(x, y, 0.0)
    return result


def _first_live_reachable(
    planner,
    live_grid,
    current,
    target,
    radius_m,
    limit,
    separation_m,
):
    candidates = select_diverse_approaches(
        planner.approach_ring(current, target, radius_m=radius_m),
        limit=limit,
        minimum_separation_m=separation_m,
    )
    for candidate in candidates:
        path_length = live_grid.path_length(
            current,
            candidate.pose,
            ROBOT_CLEARANCE_M,
        )
        if math.isfinite(path_length):
            return candidate.pose
    return None


def _camera_sees(live_grid, pose, target):
    distance = pose.distance_to(target)
    if distance < 0.12 or distance > 0.90:
        return False
    target_yaw = math.atan2(target.y - pose.y, target.x - pose.x)
    bearing = math.atan2(
        math.sin(target_yaw - pose.yaw),
        math.cos(target_yaw - pose.yaw),
    )
    return (
        abs(bearing) <= 1.211 * 0.5
        and live_grid.line_of_sight(
            (pose.x, pose.y),
            (target.x, target.y),
        )
    )


def _observe_all_colors(live_grid, pose, blocks, cache):
    for color, target in blocks.items():
        if _camera_sees(live_grid, pose, target):
            cache.add(color)


@pytest.mark.parametrize(
    'task,variant',
    [
        (task, variant)
        for task in ('task3', 'task4', 'task5')
        for variant in ('a', 'b')
    ],
)
def test_official_transport_sequence_has_live_reachable_ring_poses(
    task,
    variant,
):
    """Guard fixture solvability without leaking world data into runtime."""
    stem = f'{task}_{variant}'
    mission = load_mission(AUTONOMY / 'config/missions' / f'{stem}.yaml')
    unknown_grid = OccupancyGrid.from_yaml(mission.map_yaml)
    live_grid = OccupancyGrid.from_yaml(
        AUTONOMY / 'maps' / f'arena_{stem}.yaml'
    )
    planner = NextBestViewPlanner(
        unknown_grid,
        robot_clearance_m=ROBOT_CLEARANCE_M,
    )
    blocks = _block_poses(WORLDS / f'arena_{stem}.world')
    order = (
        mission.color_order
        if task == 'task5'
        else (mission.target_color,)
    )
    current = mission.start

    for color in order:
        target = blocks[color]
        camera_pose = _first_live_reachable(
            planner,
            live_grid,
            current,
            target,
            radius_m=0.36,
            limit=8,
            separation_m=0.20,
        )
        assert camera_pose is not None, f'{stem}/{color}: camera ring blocked'

        pick_pose = _first_live_reachable(
            planner,
            live_grid,
            camera_pose,
            target,
            radius_m=0.27,
            limit=12,
            separation_m=0.14,
        )
        assert pick_pose is not None, f'{stem}/{color}: pick ring blocked'

        drop_pose = _first_live_reachable(
            planner,
            live_grid,
            pick_pose,
            mission.pose('C'),
            radius_m=0.31,
            limit=12,
            separation_m=0.14,
        )
        assert drop_pose is not None, f'{stem}/{color}: drop ring blocked'
        current = drop_pose


@pytest.mark.parametrize(
    'task,variant',
    [
        (task, variant)
        for task in ('task4', 'task5')
        for variant in ('a', 'b')
    ],
)
def test_next_best_view_search_discovers_official_unknown_objects(
    task,
    variant,
):
    """Exercise the real greedy belief update without exposing objects to it."""
    stem = f'{task}_{variant}'
    mission = load_mission(AUTONOMY / 'config/missions' / f'{stem}.yaml')
    unknown_grid = OccupancyGrid.from_yaml(mission.map_yaml)
    live_grid = OccupancyGrid.from_yaml(
        AUTONOMY / 'maps' / f'arena_{stem}.yaml'
    )
    planner = NextBestViewPlanner(
        unknown_grid,
        robot_clearance_m=ROBOT_CLEARANCE_M,
    )
    blocks = _block_poses(WORLDS / f'arena_{stem}.world')
    order = (
        mission.color_order
        if task == 'task5'
        else (mission.target_color,)
    )
    current = mission.start
    cached = set()
    max_viewpoints = 24 if task == 'task5' else 12

    for order_index, color in enumerate(order):
        # ActiveSearch first observes at A and then performs a short positive
        # scan. Checking both endpoints is conservative because the real camera
        # also sees every intermediate heading.
        _observe_all_colors(live_grid, current, blocks, cached)
        _observe_all_colors(
            live_grid,
            Pose2D(current.x, current.y, current.yaw + 0.55),
            blocks,
            cached,
        )
        rejected = []
        visited = 0
        while color not in cached and visited < max_viewpoints:
            ranked = planner.rank_candidates(
                current,
                limit=60,
                future_color_bonus=0.12 * (
                    len(order) - order_index - 1
                ),
            )
            candidates = [
                candidate for candidate in ranked
                if all(
                    candidate.pose.distance_to(position) >= 0.16
                    for position in rejected
                )
            ]
            if not candidates:
                break

            active = candidates[0]
            live_path = live_grid.path_length(
                current,
                active.pose,
                ROBOT_CLEARANCE_M,
            )
            if not math.isfinite(live_path):
                rejected.append(active.pose)
                continue

            current = active.pose
            visited += 1
            _observe_all_colors(live_grid, current, blocks, cached)
            if color not in cached:
                planner.observe_negative(current)

        assert color in cached, (
            f'{stem}: NBV did not discover {color} within '
            f'{max_viewpoints} reachable viewpoints'
        )

        # Continue from a physically reachable drop-ring pose, matching the
        # next Task 5 search more closely than teleporting the robot to C.
        camera_pose = _first_live_reachable(
            planner,
            live_grid,
            current,
            blocks[color],
            radius_m=0.36,
            limit=8,
            separation_m=0.20,
        )
        assert camera_pose is not None
        pick_pose = _first_live_reachable(
            planner,
            live_grid,
            camera_pose,
            blocks[color],
            radius_m=0.27,
            limit=12,
            separation_m=0.14,
        )
        assert pick_pose is not None
        drop_pose = _first_live_reachable(
            planner,
            live_grid,
            pick_pose,
            mission.pose('C'),
            radius_m=0.31,
            limit=12,
            separation_m=0.14,
        )
        assert drop_pose is not None
        current = drop_pose
