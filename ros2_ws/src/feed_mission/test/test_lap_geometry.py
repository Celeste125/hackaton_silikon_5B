import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from feed_mission.lap_geometry import (  # noqa: E402
    CORNER_REACH, HALF_WIDTH, Box, WallDistance, check_plan, door_length, door_open_at,
    fit_box_from_map, fit_box_to_points, normalized, plan_lap, start_corner,
)

BOX = Box(1.0, 2.0, math.radians(30), 1.0, 0.5)


def test_box_is_on_the_right_of_every_side():
    plan = plan_lap(BOX)
    for s in plan['sides']:
        (x0, y0), (x1, y1) = plan['corners'][s['start']], plan['corners'][s['end']]
        cross = (x1 - x0) * (BOX.cy - y0) - (y1 - y0) * (BOX.cx - x0)
        assert cross < 0  # negative cross product: centre is to the right


def test_feeding_sides_keep_the_gap_and_others_the_end_gap():
    plan = plan_lap(BOX, gap=0.08, end_gap=0.2, feed='long')
    for s in plan['sides']:
        mid = np.mean([plan['corners'][s['start']], plan['corners'][s['end']]], axis=0)
        bx, by = BOX.to_box(*mid)
        to_face = abs(by) - BOX.width / 2 if s['face'] == 'long' else abs(bx) - BOX.length / 2
        expect = 0.08 if s['feed'] else 0.2
        assert to_face == pytest.approx(HALF_WIDTH + expect)
    assert [s['feed'] for s in plan['sides']] == [True, False, True, False]


def test_door_opens_only_beside_the_face_and_closes_before_the_turn():
    plan = plan_lap(BOX, gap=0.1, end_gap=0.15, feed='long', margin=0.05)
    s = plan['sides'][0]
    lo, hi = s['door']
    # the face starts end_gap + half width past the corner
    assert lo == pytest.approx(HALF_WIDTH + 0.15 + 0.05)
    assert hi - lo == pytest.approx(BOX.length - 0.10)
    assert hi < s['length'] - 0.3  # closed well before the corner
    assert not door_open_at(s, 0.1) and door_open_at(s, (lo + hi) / 2)
    assert plan['sides'][1]['door'] is None
    assert door_length(plan) == pytest.approx(2 * (BOX.length - 0.10))


def test_door_position_on_the_robot_shifts_the_stretch():
    a = plan_lap(BOX, door_x=0.0)['sides'][0]['door']
    b = plan_lap(BOX, door_x=0.1)['sides'][0]['door']
    assert b[0] == pytest.approx(a[0] - 0.1) and b[1] == pytest.approx(a[1] - 0.1)


def test_corners_leave_room_to_turn_and_tight_plans_warn():
    plan = plan_lap(BOX, gap=0.1, end_gap=0.15)
    for x, y in plan['corners']:
        nearest = min(math.dist((x, y), c) for c in BOX.corners())
        assert nearest > CORNER_REACH
    assert plan['warnings'] == []
    assert plan_lap(BOX, gap=0.01, end_gap=0.0)['warnings']


def test_start_corner_is_the_nearest():
    plan = plan_lap(BOX)
    x, y = plan['corners'][2]
    assert start_corner(plan, x + 0.05, y) == 2


def test_normalized_puts_length_first():
    b = normalized(Box(0, 0, 0.0, 0.5, 1.0))
    assert (b.length, b.width) == (1.0, 0.5)
    assert abs(b.yaw) == pytest.approx(math.pi / 2)


def arena_with_box(res=0.05):
    g = np.zeros((100, 80), dtype=np.int16)
    g[:, :2] = g[:, -2:] = g[:2, :] = g[-2:, :] = 100
    g[40:50, 30:50] = 100        # box: x 1.5..2.5, y 2.0..2.5
    return g


def test_fit_box_from_map_finds_the_free_standing_box():
    box = fit_box_from_map(arena_with_box(), 0.05, 0.0, 0.0, near=(2.0, 1.0))
    assert box.cx == pytest.approx(2.0, abs=0.03) and box.cy == pytest.approx(2.25, abs=0.03)
    assert box.length == pytest.approx(1.0, abs=0.06) and box.width == pytest.approx(0.5, abs=0.06)


def test_fit_box_to_points_snaps_onto_two_visible_faces():
    truth = Box(2.0, 2.25, math.radians(4), 1.0, 0.5)
    # the lidar sees the bottom face and the right end, a little noisy
    rng = np.random.default_rng(0)
    xs = np.linspace(-0.5, 0.5, 60)
    ys = np.linspace(-0.25, 0.25, 30)
    local = np.r_[np.c_[xs, np.full_like(xs, -0.25)], np.c_[np.full_like(ys, 0.5), ys]]
    pts = np.array([truth.to_map(x, y) for x, y in local]) + rng.normal(0, 0.005, (90, 2))
    guess = Box(2.05, 2.21, 0.0, 1.0, 0.5)
    fitted, used, err = fit_box_to_points(guess, pts)
    assert used == 90 and err < 0.01
    assert fitted.cx == pytest.approx(2.0, abs=0.01) and fitted.cy == pytest.approx(2.25, abs=0.01)
    assert math.degrees(fitted.yaw) == pytest.approx(4, abs=0.6)


def test_check_plan_flags_a_wall_in_the_way():
    g = arena_with_box()
    walls = WallDistance(g, 0.05, 0.0, 0.0)
    box = Box(2.0, 2.25, 0.0, 1.0, 0.5)
    assert check_plan(plan_lap(box), walls) == []
    g[60:64, 30:50] = 100        # a wall 0.75 m above the box: the top side runs into it
    assert any(k == 'side' for k, _, _ in check_plan(plan_lap(box), WallDistance(g, 0.05, 0, 0)))


def test_wall_check_ignores_the_box_itself():
    g = arena_with_box()
    g[38:52, 28:52] = 100  # the map draws the box 10 cm too thick on every side
    box = Box(2.0, 2.25, 0.0, 1.0, 0.5)
    tight = plan_lap(box, gap=0.02)  # 2 cm from the real face, inside the thick drawing
    assert check_plan(tight, WallDistance(g, 0.05, 0, 0))
    assert check_plan(tight, WallDistance(g, 0.05, 0, 0, ignore=box)) == []
