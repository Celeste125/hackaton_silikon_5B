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


def test_lap_corners_need_all_four_in_order(tmp_path):
    f = tmp_path / 'p.yaml'
    for i, c in enumerate(poses.LAP_CORNERS[:3]):
        poses.save_pose(c, i, 0, 0, path=f)
    assert poses.lap_corners(poses.load(f)) is None
    poses.save_pose('lap4', 3, 0, 0, path=f)
    assert poses.lap_corners(poses.load(f)) == [(0, 0), (1, 0), (2, 0), (3, 0)]
