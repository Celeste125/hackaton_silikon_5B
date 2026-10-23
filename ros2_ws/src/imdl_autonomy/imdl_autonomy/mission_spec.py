"""Validated, variant-free mission specifications for the autonomy solver stack."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

import yaml

SUPPORTED_TASKS = frozenset(f'task{index}' for index in range(1, 6))
SUPPORTED_COLORS = frozenset({'red', 'green', 'blue', 'yellow'})
SUPPORTED_COORDINATE_MODES = frozenset({'absolute'})
SUPPORTED_OBSTACLE_MODES = frozenset({'known', 'unknown'})


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, other: 'Pose2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class MissionSpec:
    mission_id: str
    task: str
    frame_id: str
    coordinate_mode: str
    obstacle_mode: str
    map_yaml: Path
    poses: Mapping[str, Pose2D]
    time_limit_sec: float
    target_color: Optional[str] = None
    color_order: Tuple[str, ...] = ()
    fixed_place_yaw: Optional[float] = None

    @property
    def start(self) -> Pose2D:
        return self.poses['A']

    def pose(self, name: str) -> Pose2D:
        try:
            return self.poses[name.upper()]
        except KeyError as error:
            raise KeyError(
                f'Mission {self.mission_id!r} has no pose {name!r}'
            ) from error


class MissionValidationError(ValueError):
    """Raised when a mission file is incomplete or unsafe to execute."""


def _finite_float(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise MissionValidationError(f'{field} must be numeric') from error
    if not math.isfinite(result):
        raise MissionValidationError(f'{field} must be finite')
    return result


def _parse_pose(name: str, value: Any) -> Pose2D:
    if not isinstance(value, Mapping):
        raise MissionValidationError(f'poses.{name} must be a mapping')
    if 'x' not in value or 'y' not in value:
        raise MissionValidationError(f'poses.{name} requires x and y')
    return Pose2D(
        x=_finite_float(value['x'], f'poses.{name}.x'),
        y=_finite_float(value['y'], f'poses.{name}.y'),
        yaw=_finite_float(value.get('yaw', 0.0), f'poses.{name}.yaw'),
    )


def _required_poses(task: str) -> Tuple[str, ...]:
    if task == 'task1':
        return ('A', 'B')
    if task in {'task2', 'task3'}:
        return ('A', 'B', 'C')
    return ('A', 'C')


def load_mission(path: str | Path) -> MissionSpec:
    """Load and strictly validate one mission YAML file."""
    mission_path = Path(path).expanduser().resolve()
    if not mission_path.is_file():
        raise MissionValidationError(f'Mission file does not exist: {mission_path}')

    try:
        raw = yaml.safe_load(mission_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise MissionValidationError(
            f'Cannot read mission file {mission_path}: {error}'
        ) from error
    if not isinstance(raw, Mapping):
        raise MissionValidationError('Mission root must be a mapping')

    mission_id = str(raw.get('mission_id', '')).strip()
    task = str(raw.get('task', '')).strip().lower()
    frame_id = str(raw.get('frame_id', 'map')).strip()
    if not mission_id:
        raise MissionValidationError('mission_id must not be empty')
    if task not in SUPPORTED_TASKS:
        raise MissionValidationError(
            f'task must be one of {sorted(SUPPORTED_TASKS)}, got {task!r}'
        )
    if not frame_id:
        raise MissionValidationError('frame_id must not be empty')

    coordinate_mode = str(
        raw.get('coordinate_mode', 'absolute')
    ).strip().lower()
    if coordinate_mode not in SUPPORTED_COORDINATE_MODES:
        raise MissionValidationError(
            'coordinate_mode must be absolute; convert relative measurements '
            'before starting autonomy'
        )
    obstacle_mode = str(
        raw.get('obstacle_mode', 'unknown')
    ).strip().lower()
    if obstacle_mode not in SUPPORTED_OBSTACLE_MODES:
        raise MissionValidationError(
            'obstacle_mode must be one of '
            f'{sorted(SUPPORTED_OBSTACLE_MODES)}, got {obstacle_mode!r}'
        )

    map_value = str(raw.get('map_yaml', '')).strip()
    if not map_value:
        raise MissionValidationError('map_yaml must not be empty')
    map_yaml = Path(map_value).expanduser()
    if not map_yaml.is_absolute():
        map_yaml = (mission_path.parent / map_yaml).resolve()
    if not map_yaml.is_file():
        raise MissionValidationError(f'Map YAML does not exist: {map_yaml}')

    raw_poses = raw.get('poses')
    if not isinstance(raw_poses, Mapping):
        raise MissionValidationError('poses must be a mapping')
    poses: Dict[str, Pose2D] = {}
    for name, value in raw_poses.items():
        normalized = str(name).upper()
        if normalized not in {'A', 'B', 'C'}:
            raise MissionValidationError(f'Unknown mission pose: {name!r}')
        poses[normalized] = _parse_pose(normalized, value)
    missing = [name for name in _required_poses(task) if name not in poses]
    if missing:
        raise MissionValidationError(
            f'{task} requires mission poses {missing}'
        )

    time_limit = _finite_float(
        raw.get('time_limit_sec', 600.0 if task == 'task5' else 180.0),
        'time_limit_sec',
    )
    if time_limit <= 0.0:
        raise MissionValidationError('time_limit_sec must be positive')

    target_color = raw.get('target_color')
    if target_color is not None:
        target_color = str(target_color).strip().lower()
        if target_color not in SUPPORTED_COLORS:
            raise MissionValidationError(
                f'Unsupported target_color {target_color!r}'
            )

    order_value = raw.get('color_order', [])
    if not isinstance(order_value, (list, tuple)):
        raise MissionValidationError('color_order must be a list')
    color_order = tuple(str(item).strip().lower() for item in order_value)
    if any(color not in SUPPORTED_COLORS for color in color_order):
        raise MissionValidationError(
            f'color_order contains unsupported colors: {color_order}'
        )
    if len(set(color_order)) != len(color_order):
        raise MissionValidationError('color_order must not contain duplicates')

    if task in {'task3', 'task4'} and target_color is None:
        raise MissionValidationError(f'{task} requires target_color')
    if task == 'task5' and len(color_order) != 3:
        raise MissionValidationError('task5 requires exactly three colors')

    fixed_place_yaw = raw.get('fixed_place_yaw')
    if fixed_place_yaw is not None:
        fixed_place_yaw = _finite_float(fixed_place_yaw, 'fixed_place_yaw')

    return MissionSpec(
        mission_id=mission_id,
        task=task,
        frame_id=frame_id,
        coordinate_mode=coordinate_mode,
        obstacle_mode=obstacle_mode,
        map_yaml=map_yaml,
        poses=poses,
        time_limit_sec=time_limit,
        target_color=target_color,
        color_order=color_order,
        fixed_place_yaw=fixed_place_yaw,
    )
