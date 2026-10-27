import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from feed_mission import poses  # noqa: E402


def test_missing_file_is_empty(tmp_path):
    assert poses.load(tmp_path / 'none.yaml') == {}


def test_save_load_round_trip_and_yaw_wrap(tmp_path):
    f = tmp_path / 'p.yaml'
    poses.save_pose('silo2', 1.23456, -0.5, 3 * math.pi / 2, path=f)
    p = poses.load(f)['silo2']
    assert (p['x'], p['y']) == (1.235, -0.5)
    assert p['yaw'] == pytest.approx(-math.pi / 2, abs=1e-3)
    assert p['saved']


def test_save_keeps_other_poses_and_clear_removes_one(tmp_path):
    f = tmp_path / 'p.yaml'
    poses.save_pose('silo1', 0, 0, 0, path=f)
    poses.save_pose('start', 1, 1, 0, path=f)
    poses.clear_pose('silo1', path=f)
    assert set(poses.load(f)) == {'start'}


def test_unknown_name_is_refused(tmp_path):
    with pytest.raises(ValueError):
        poses.save_pose('silo4', 0, 0, 0, path=tmp_path / 'p.yaml')


def test_box_feed_settings_and_poses_live_side_by_side(tmp_path):
    f = tmp_path / 'p.yaml'
    poses.save_pose('silo1', 1, 2, 0, path=f)
    poses.save_box({'cx': 0.5, 'cy': -0.4, 'yaw': 1.2, 'length': 1.0, 'width': 0.5}, path=f)
    poses.save_feed({'gap': 0.1, 'door_open': 0.2}, path=f)
    poses.save_pose('start', 0, 0, 0, path=f)  # rewriting a pose keeps the rest
    assert set(poses.load(f)) == {'silo1', 'start'}
    box = poses.load_box(f)
    assert (box['cx'], box['length']) == (0.5, 1.0) and box['saved']
    assert poses.load_feed(f) == {'gap': 0.1, 'door_open': 0.2}


def test_no_box_yet(tmp_path):
    assert poses.load_box(tmp_path / 'p.yaml') is None
    assert poses.load_feed(tmp_path / 'p.yaml') == {}


def test_runs_are_appended_next_to_the_poses_file(tmp_path):
    f = tmp_path / 'p.yaml'
    poses.add_run({'door_open': 0.2, 'dispensed_kg': 0.31}, path=f)
    runs = poses.add_run({'door_open': 0.3, 'dispensed_kg': 0.52}, path=f)
    assert [r['door_open'] for r in runs] == [0.2, 0.3]
    assert poses.load_runs(f) == runs and runs[0]['time']
    assert (tmp_path / 'feed_runs.yaml').exists()


def test_predock_is_straight_back_from_the_docked_pose():
    p = poses.predock_pose({'x': 2.0, 'y': -1.5, 'yaw': 0.0})
    assert (round(p['x'], 3), round(p['y'], 3), p['yaw']) == (1.4, -1.5, 0.0)
    q = poses.predock_pose({'x': 0.0, 'y': 0.0, 'yaw': math.pi / 2}, distance=0.5)
    assert abs(q['x']) < 1e-9 and q['y'] == pytest.approx(-0.5)
