"""Portable task and mission-file selection shared by launch files."""

from glob import glob
import os
import re


_TASK_PATTERN = re.compile(r'^(?:task[_-]?)?([1-5])$', re.IGNORECASE)
_VARIANT_PATTERN = re.compile(r'^[ab]$', re.IGNORECASE)


def normalize_task_id(task: str) -> str:
    """Return the canonical task ID (for example ``task5``)."""
    value = str(task).strip()
    match = _TASK_PATTERN.fullmatch(value)
    if match is None:
        raise ValueError(
            f'Invalid task {value!r}. Use 1..5, task1..task5, or task_1..task_5.'
        )
    return f'task{match.group(1)}'


def normalize_variant(variant: str) -> str:
    """Return the canonical lowercase world variant."""
    value = str(variant).strip().lower()
    if _VARIANT_PATTERN.fullmatch(value) is None:
        raise ValueError(f'Invalid variant {variant!r}. Use a or b.')
    return value


def resolve_mission_file(
    package_share: str,
    task: str,
    variant: str,
    explicit_file: str = '',
) -> tuple[str, str, str]:
    """Resolve a mission YAML and return path, canonical task, and variant."""
    canonical_task = normalize_task_id(task)
    canonical_variant = normalize_variant(variant)

    if explicit_file:
        mission_file = os.path.abspath(os.path.expanduser(explicit_file))
    else:
        mission_file = os.path.join(
            package_share,
            'config',
            'missions',
            f'{canonical_task}_{canonical_variant}.yaml',
        )

    if not os.path.isfile(mission_file):
        mission_directory = os.path.join(
            package_share, 'config', 'missions'
        )
        available = ', '.join(
            os.path.basename(path)
            for path in sorted(glob(os.path.join(mission_directory, '*.yaml')))
        )
        raise RuntimeError(
            f'Autonomy mission file does not exist: {mission_file}. '
            f'Available missions: {available or "none"}'
        )

    return mission_file, canonical_task, canonical_variant
