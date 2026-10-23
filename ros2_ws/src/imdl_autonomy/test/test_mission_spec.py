from pathlib import Path

from imdl_autonomy.mission_spec import (
    load_mission,
    MissionValidationError,
)
import pytest


def test_all_repository_ai_missions_validate():
    mission_dir = Path(__file__).parents[1] / 'config' / 'missions'
    missions = [load_mission(path) for path in mission_dir.glob('*.yaml')]

    assert len(missions) == 10
    assert {mission.task for mission in missions} == {
        'task1', 'task2', 'task3', 'task4', 'task5'
    }
    assert all(mission.start == mission.pose('A') for mission in missions)
    assert all(mission.coordinate_mode == 'absolute' for mission in missions)
    assert all(mission.obstacle_mode == 'unknown' for mission in missions)
    assert all(mission.map_yaml.name == 'arena_unknown.yaml' for mission in missions)


def test_task5_requires_exactly_three_colors(tmp_path):
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.02\n', encoding='ascii')
    mission = tmp_path / 'bad.yaml'
    mission.write_text(
        '\n'.join([
            'mission_id: bad_task5',
            'task: task5',
            'map_yaml: map.yaml',
            'poses:',
            '  A: {x: 0, y: 0}',
            '  C: {x: 0.5, y: 0.5}',
            'color_order: [red, blue]',
        ]),
        encoding='ascii',
    )

    with pytest.raises(MissionValidationError, match='exactly three'):
        load_mission(mission)


def test_relative_coordinate_mode_is_rejected_before_navigation(tmp_path):
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.02\n', encoding='ascii')
    mission = tmp_path / 'relative.yaml'
    mission.write_text(
        '\n'.join([
            'mission_id: ambiguous',
            'task: task1',
            'coordinate_mode: relative',
            'map_yaml: map.yaml',
            'poses:',
            '  A: {x: 0, y: 0, yaw: 0}',
            '  B: {x: 0.5, y: 0, yaw: 0}',
        ]),
        encoding='ascii',
    )

    with pytest.raises(MissionValidationError, match='must be absolute'):
        load_mission(mission)
