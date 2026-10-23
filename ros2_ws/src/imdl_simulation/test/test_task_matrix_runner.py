import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'task_matrix_runner.py'
)
SPEC = importlib.util.spec_from_file_location('task_matrix_runner', SCRIPT)
MATRIX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MATRIX)


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        ('task1_a', ('task1', 'a')),
        ('TASK5_B', ('task5', 'b')),
    ],
)
def test_parse_case(value, expected):
    assert MATRIX.parse_case(value) == expected


def test_parse_case_rejects_non_official_names():
    with pytest.raises(ValueError):
        MATRIX.parse_case('task6_a')


def test_extract_task_result_uses_latest_result():
    first = {'success': False, 'reason': 'old'}
    second = {'success': True, 'reason': 'completed'}
    log = (
        f'[node] task_result: {json.dumps(first)}\n'
        f'[node] task_result: {json.dumps(second)}\n'
    )
    assert MATRIX.extract_task_result(log) == second
