import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest  # noqa: E402

from feed_mission.lap_driver import (  # noqa: E402
    blocked_ahead, door_open_sides, straight_command, turn_blocked, turn_command, wrap,
)

DT = 0.05


def drive_side(pose, start, target, speed=0.15, steps=2000):
    """Unicycle simulation of one straight side; returns the final pose and the worst offset."""
    x, y, yaw = pose
    worst = 0.0
    for _ in range(steps):
        v, w, _, done = straight_command((x, y, yaw), start, target, speed)
        if done:
            return (x, y, yaw), worst
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = wrap(yaw + w * DT)
        worst = max(worst, abs(y))  # the line in these tests is y = 0
    raise AssertionError('never reached the corner')


def test_straight_on_the_line_stays_on_it_and_stops_at_the_corner():
    (x, y, _), worst = drive_side((0.0, 0.0, 0.0), (0.0, 0.0), (2.0, 0.0))
    assert abs(x - 2.0) < 0.03 and worst < 1e-6


def test_straight_pulls_back_onto_the_line():
    # 8 cm off the line and 6 degrees off heading, as after odometry drift
    (x, y, _), _ = drive_side((0.0, 0.08, math.radians(6)), (0.0, 0.0), (2.0, 0.0))
    assert abs(x - 2.0) < 0.03
    assert abs(y) < 0.01


def test_turn_converges_both_ways():
    for target in (math.pi / 2, -math.pi / 2, math.pi):
        yaw = 0.0
        for _ in range(400):
            w, done = turn_command(yaw, target, max_w=0.6)
            if done:
                break
            yaw = wrap(yaw + w * DT)
        assert done and abs(wrap(yaw - target)) < 0.03


def test_blocked_ahead_only_in_front_within_width():
    hl, hw = 0.17, 0.26
    assert blocked_ahead(np.array([[0.25, 0.0]]), hl, hw, 0.15)
    assert not blocked_ahead(np.array([[0.40, 0.0]]), hl, hw, 0.15)   # far enough
    assert not blocked_ahead(np.array([[0.25, 0.40]]), hl, hw, 0.15)  # beside, not ahead
    assert not blocked_ahead(np.array([[-0.25, 0.0]]), hl, hw, 0.15)  # behind
    assert not blocked_ahead(np.zeros((0, 2)), hl, hw, 0.15)


def test_turn_blocked_by_anything_within_corner_reach():
    hl, hw = 0.17, 0.26
    assert turn_blocked(np.array([[-0.2, 0.2]]), hl, hw)       # 0.28 m: a corner would hit it
    assert not turn_blocked(np.array([[0.0, 0.40]]), hl, hw)   # 0.40 m: clear


def test_door_sides():
    # 2 m x 1 m box, starting on a short side: the long sides are 1 and 3
    corners = [(0, 0), (0, 1), (2, 1), (2, 0)]
    assert door_open_sides(corners, 'long') == {1, 3}
    assert door_open_sides(corners, 'all') == {0, 1, 2, 3}
    assert door_open_sides(corners, 'none') == set()
    with pytest.raises(ValueError):
        door_open_sides(corners, 'sometimes')
