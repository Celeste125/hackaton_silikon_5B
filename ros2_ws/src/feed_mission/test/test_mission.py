import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from feed_mission.mission import FillWatch, split_recipe  # noqa: E402


def test_recipe_split_by_share():
    assert split_recipe(2.0, [40, 40, 20]) == pytest.approx([0.8, 0.8, 0.4])
    assert split_recipe(1.0, [1, 1, 0]) == pytest.approx([0.5, 0.5, 0.0])
    assert split_recipe(1.0, [0, 0, 0]) == [0.0, 0.0, 0.0]
    assert split_recipe(0.0, [40, 40, 20]) == [0.0, 0.0, 0.0]


def test_fill_done_when_target_reached():
    w = FillWatch(0.5, stall_s=8, stall_kg=0.01, timeout_s=90, t0=0)
    assert w.update(1, 0.2) == 'filling'
    assert w.update(2, 0.5) == 'done'


def test_fill_stalls_without_progress_and_recovers_after_restart():
    w = FillWatch(0.5, stall_s=8, stall_kg=0.01, timeout_s=90, t0=0)
    assert w.update(1, 0.10) == 'filling'
    assert w.update(5, 0.105) == 'filling'    # under stall_kg: no new mark
    assert w.update(9.5, 0.106) == 'stalled'  # 8.5 s since the last real rise
    w.restart_stall(9.5, 0.106)
    assert w.update(12, 0.2) == 'filling'


def test_fill_times_out():
    w = FillWatch(5.0, stall_s=100, stall_kg=0.0, timeout_s=10, t0=0)
    assert w.update(11, 1.0) == 'timeout'
