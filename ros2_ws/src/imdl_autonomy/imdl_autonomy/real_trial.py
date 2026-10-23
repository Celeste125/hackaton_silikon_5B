"""Create validated real-world mission files from measured arena poses."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import tempfile
from typing import Iterable, Mapping, Optional, Sequence

import yaml

from .mission_geometry import validate_mission_geometry
from .mission_spec import load_mission, Pose2D, SUPPORTED_TASKS
from .next_best_view import OccupancyGrid


ROBOT_CLEARANCE_M = 0.205
APPROACH_DISTANCE_M = 0.27


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_pose_values(
    values: Sequence[float],
    label: str,
    require_yaw: bool,
) -> Pose2D:
    """Parse ``x y [yaw]`` values and reject ambiguous input."""
    expected = 'x y yaw' if require_yaw else 'x y [yaw]'
    if len(values) not in ({3} if require_yaw else {2, 3}):
        raise ValueError(f'{label} requires {expected}')
    parsed = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f'{label} values must be finite')
    yaw = parsed[2] if len(parsed) == 3 else 0.0
    return Pose2D(parsed[0], parsed[1], normalize_angle(yaw))


def relative_pose_to_absolute(start: Pose2D, relative: Pose2D) -> Pose2D:
    """Transform a body-frame offset at A into the arena map frame."""
    cosine = math.cos(start.yaw)
    sine = math.sin(start.yaw)
    return Pose2D(
        start.x + cosine * relative.x - sine * relative.y,
        start.y + sine * relative.x + cosine * relative.y,
        normalize_angle(start.yaw + relative.yaw),
    )


def _resolve_goal(
    start: Pose2D,
    name: str,
    absolute: Optional[Sequence[float]],
    relative: Optional[Sequence[float]],
    required: bool,
    require_yaw: bool,
) -> Optional[Pose2D]:
    if absolute is not None and relative is not None:
        raise ValueError(
            f'Use either --{name.lower()} or '
            f'--{name.lower()}-relative-to-a, not both'
        )
    if absolute is not None:
        return parse_pose_values(absolute, name, require_yaw)
    if relative is not None:
        offset = parse_pose_values(relative, f'{name} relative to A', require_yaw)
        return relative_pose_to_absolute(start, offset)
    if required:
        raise ValueError(f'{name} is required for this task')
    return None


def build_trial_payload(
    *,
    mission_id: str,
    task: str,
    map_yaml: Path,
    a: Sequence[float],
    b: Optional[Sequence[float]] = None,
    b_relative_to_a: Optional[Sequence[float]] = None,
    c: Optional[Sequence[float]] = None,
    c_relative_to_a: Optional[Sequence[float]] = None,
    target_color: Optional[str] = None,
    color_order: Iterable[str] = (),
    fixed_place_yaw: Optional[float] = None,
    time_limit_sec: Optional[float] = None,
) -> Mapping[str, object]:
    """Build one absolute, unknown-obstacle mission dictionary."""
    normalized_task = str(task).strip().lower()
    if normalized_task not in SUPPORTED_TASKS:
        raise ValueError(f'Unsupported task {task!r}')
    normalized_id = str(mission_id).strip()
    if not normalized_id:
        raise ValueError('mission_id must not be empty')
    resolved_map = Path(map_yaml).expanduser().resolve()
    if not resolved_map.is_file():
        raise ValueError(f'Arena map does not exist: {resolved_map}')

    start = parse_pose_values(a, 'A', require_yaw=True)
    needs_b = normalized_task in {'task1', 'task2', 'task3'}
    needs_c = normalized_task in {'task2', 'task3', 'task4', 'task5'}
    pose_b = _resolve_goal(
        start,
        'B',
        b,
        b_relative_to_a,
        required=needs_b,
        require_yaw=normalized_task in {'task1', 'task2'},
    )
    pose_c = _resolve_goal(
        start,
        'C',
        c,
        c_relative_to_a,
        required=needs_c,
        require_yaw=normalized_task == 'task2',
    )

    poses = {'A': start}
    if pose_b is not None:
        poses['B'] = pose_b
    if pose_c is not None:
        poses['C'] = pose_c
    payload = {
        'mission_id': normalized_id,
        'task': normalized_task,
        'frame_id': 'map',
        'coordinate_mode': 'absolute',
        'obstacle_mode': 'unknown',
        'map_yaml': str(resolved_map),
        'time_limit_sec': float(
            time_limit_sec
            if time_limit_sec is not None
            else (600.0 if normalized_task == 'task5' else 180.0)
        ),
        'poses': {
            name: {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw}
            for name, pose in poses.items()
        },
    }
    if target_color is not None:
        payload['target_color'] = str(target_color).strip().lower()
    order = [str(color).strip().lower() for color in color_order]
    if order:
        payload['color_order'] = order
    if fixed_place_yaw is not None:
        payload['fixed_place_yaw'] = normalize_angle(float(fixed_place_yaw))
    return payload


def write_validated_trial(
    payload: Mapping[str, object],
    output_path: Path,
    *,
    force: bool = False,
) -> Path:
    """Validate syntax and walls-only geometry before atomically saving."""
    output = Path(output_path).expanduser().resolve()
    if output.exists() and not force:
        raise ValueError(f'Output already exists: {output}; use --force to replace it')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            prefix=f'.{output.stem}.',
            suffix='.yaml',
            dir=output.parent,
            delete=False,
        ) as stream:
            yaml.safe_dump(dict(payload), stream, sort_keys=False)
            temporary_path = Path(stream.name)
        mission = load_mission(temporary_path)
        grid = OccupancyGrid.from_yaml(mission.map_yaml)
        issues = validate_mission_geometry(
            mission,
            grid,
            robot_clearance_m=ROBOT_CLEARANCE_M,
            approach_distance_m=APPROACH_DISTANCE_M,
        )
        if issues:
            details = '; '.join(str(issue) for issue in issues)
            raise ValueError(f'Unsafe mission geometry: {details}')
        temporary_path.replace(output)
        return output
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Create an absolute IMDL mission for simulation or hardware. '
            'The map origin is the center of the 1.63 m arena; A is the '
            'measured robot pose and does not need to be at the origin. '
            'Static navigation knows only the arena walls; cylinders come '
            'from live lidar.'
        ),
    )
    parser.add_argument('--task', required=True, choices=sorted(SUPPORTED_TASKS))
    parser.add_argument('--mission-id', required=True)
    parser.add_argument(
        '--a', '--start',
        required=True,
        nargs='+',
        type=float,
        help=(
            'Absolute robot start pose in the arena-centered map frame: '
            'x y yaw_rad.'
        ),
    )
    b_group = parser.add_mutually_exclusive_group()
    b_group.add_argument(
        '--b', '--b-absolute',
        nargs='+',
        type=float,
        help='Absolute B pose: x y [yaw_rad].',
    )
    b_group.add_argument(
        '--b-relative-to-a',
        nargs='+',
        type=float,
        help='B offset in the robot frame at A: forward left [delta_yaw].',
    )
    c_group = parser.add_mutually_exclusive_group()
    c_group.add_argument(
        '--c', '--c-absolute',
        nargs='+',
        type=float,
        help='Absolute C pose: x y [yaw_rad].',
    )
    c_group.add_argument(
        '--c-relative-to-a',
        nargs='+',
        type=float,
        help='C offset in the robot frame at A: forward left [delta_yaw].',
    )
    parser.add_argument('--target-color')
    parser.add_argument('--color-order', nargs='+', default=[])
    parser.add_argument('--fixed-place-yaw', type=float)
    parser.add_argument('--time-limit-sec', type=float)
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--force', action='store_true')
    return parser


def main(argv=None) -> None:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.map_yaml is None:
        from ament_index_python.packages import get_package_share_directory

        args.map_yaml = (
            Path(get_package_share_directory('imdl_autonomy'))
            / 'maps'
            / 'arena_unknown.yaml'
        )
    try:
        payload = build_trial_payload(
            mission_id=args.mission_id,
            task=args.task,
            map_yaml=args.map_yaml,
            a=args.a,
            b=args.b,
            b_relative_to_a=args.b_relative_to_a,
            c=args.c,
            c_relative_to_a=args.c_relative_to_a,
            target_color=args.target_color,
            color_order=args.color_order,
            fixed_place_yaw=args.fixed_place_yaw,
            time_limit_sec=args.time_limit_sec,
        )
        output = write_validated_trial(payload, args.output, force=args.force)
    except ValueError as error:
        parser.error(str(error))
    print(f'Validated simulation/hardware mission written to {output}')
    print('map origin: arena center (0.000, 0.000)')
    for name, pose in payload['poses'].items():
        print(
            f'{name}: x={pose["x"]:.3f}, y={pose["y"]:.3f}, '
            f'yaw={pose["yaw"]:.3f} rad'
        )


if __name__ == '__main__':
    main()
