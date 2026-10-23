"""Conservative geometry preflight for autonomy mission files."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Tuple

from .mission_spec import MissionSpec
from .next_best_view import NextBestViewPlanner, OccupancyGrid


@dataclass(frozen=True)
class MissionGeometryIssue:
    code: str
    subject: str
    message: str

    def __str__(self) -> str:
        return f'{self.code}[{self.subject}]: {self.message}'


def _robot_waypoints(mission: MissionSpec) -> Tuple[str, ...]:
    if mission.task == 'task1':
        return ('A', 'B')
    if mission.task == 'task2':
        return ('A', 'B', 'C')
    return ('A',)


def _approach_targets(mission: MissionSpec) -> Tuple[str, ...]:
    if mission.task == 'task3':
        return ('B', 'C')
    if mission.task in {'task4', 'task5'}:
        return ('C',)
    return ()


def validate_mission_geometry(
    mission: MissionSpec,
    grid: OccupancyGrid,
    robot_clearance_m: float,
    approach_distance_m: float,
) -> Tuple[MissionGeometryIssue, ...]:
    """Reject unreachable robot poses before Nav2 spends the mission timeout.

    The map already contains obstacle radii. ``robot_clearance_m`` therefore
    represents the robot footprint radius, including its camera and padding.
    Object/drop centers are not robot-center goals; they are validated through
    collision-free approach rings instead.
    """
    issues = []
    clearance = max(0.0, float(robot_clearance_m))

    for name in _robot_waypoints(mission):
        pose = mission.pose(name)
        available = grid.clearance_at(pose.x, pose.y)
        if not grid.is_free(pose.x, pose.y, clearance):
            issues.append(MissionGeometryIssue(
                code='ROBOT_POSE_IN_COLLISION',
                subject=name,
                message=(
                    f'clearance={available:.3f} m, '
                    f'required={clearance:.3f} m'
                ),
            ))

    waypoint_names = _robot_waypoints(mission)
    for start_name, goal_name in zip(waypoint_names, waypoint_names[1:]):
        length = grid.path_length(
            mission.pose(start_name),
            mission.pose(goal_name),
            clearance,
        )
        if not math.isfinite(length):
            issues.append(MissionGeometryIssue(
                code='NO_STATIC_MAP_PATH',
                subject=f'{start_name}->{goal_name}',
                message=(
                    'no grid path with '
                    f'{clearance:.3f} m robot clearance'
                ),
            ))

    planner = NextBestViewPlanner(
        grid,
        robot_clearance_m=clearance,
    )
    for name in _approach_targets(mission):
        center = mission.pose(name)
        cell = grid.world_to_cell(center.x, center.y)
        if cell is None or grid.occupied[cell]:
            issues.append(MissionGeometryIssue(
                code='TARGET_CENTER_OCCUPIED',
                subject=name,
                message='object/drop center is outside free mapped space',
            ))
            continue
        candidates = planner.approach_ring(
            mission.start,
            center,
            radius_m=approach_distance_m,
        )
        accepted = [candidate for candidate in candidates if candidate.accepted]
        if not accepted:
            issues.append(MissionGeometryIssue(
                code='NO_APPROACH_POSE',
                subject=name,
                message=(
                    f'no reachable approach at radius '
                    f'{approach_distance_m:.3f} m'
                ),
            ))

    return tuple(_deduplicate(issues))


def _deduplicate(
    issues: Iterable[MissionGeometryIssue],
) -> Iterable[MissionGeometryIssue]:
    seen = set()
    for issue in issues:
        key = (issue.code, issue.subject)
        if key in seen:
            continue
        seen.add(key)
        yield issue
