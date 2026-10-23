from pathlib import Path

import pytest

from imdl_autonomy.mission_selection import (
    normalize_task_id,
    normalize_variant,
    resolve_mission_file,
)


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        ('1', 'task1'),
        ('task3', 'task3'),
        ('task_5', 'task5'),
        ('TASK-2', 'task2'),
    ],
)
def test_normalize_task_id_accepts_supported_spellings(value, expected):
    assert normalize_task_id(value) == expected


@pytest.mark.parametrize('value', ['', 'task', 'task_number', 'task6'])
def test_normalize_task_id_rejects_ambiguous_values(value):
    with pytest.raises(ValueError, match='Invalid task'):
        normalize_task_id(value)


def test_normalize_variant_is_case_insensitive():
    assert normalize_variant('B') == 'b'


def test_resolve_mission_file_uses_canonical_name(tmp_path):
    mission_directory = tmp_path / 'config' / 'missions'
    mission_directory.mkdir(parents=True)
    expected = mission_directory / 'task5_a.yaml'
    expected.write_text('task: task5\n', encoding='utf-8')

    path, task, variant = resolve_mission_file(
        str(tmp_path), '5', 'A'
    )

    assert Path(path) == expected
    assert task == 'task5'
    assert variant == 'a'


def test_missing_mission_lists_available_files(tmp_path):
    mission_directory = tmp_path / 'config' / 'missions'
    mission_directory.mkdir(parents=True)
    (mission_directory / 'task1_a.yaml').write_text(
        'task: task1\n', encoding='utf-8'
    )

    with pytest.raises(RuntimeError, match='task1_a.yaml'):
        resolve_mission_file(str(tmp_path), '5', 'b')
