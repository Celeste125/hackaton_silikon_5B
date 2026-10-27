import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from feed_mission.lap_driver import straight_command, wrap  # noqa: E402
from feed_mission.silo_route import default_entry, drive_direction, last_silo  # noqa: E402

# the real silo 1: facing west along the row, the wall to its left (south)
SILO1 = {'x': 1.088, 'y': -3.923, 'yaw': math.radians(175)}


def test_default_entry_is_on_the_open_right_side_facing_silo_1():
    e = default_entry(SILO1, side='right', distance=0.6)
    assert e['y'] > SILO1['y'] + 0.55            # north of it, away from the wall
    assert math.dist((e['x'], e['y']), (SILO1['x'], SILO1['y'])) == pytest.approx(0.6)
    to_silo = math.atan2(SILO1['y'] - e['y'], SILO1['x'] - e['x'])
    assert abs(wrap(e['yaw'] - to_silo)) < 1e-9  # it faces silo 1


def test_along_the_row_forward_and_back_in_reverse():
    silo2 = {'x': 0.836, 'y': -3.921, 'yaw': SILO1['yaw']}
    at1 = (SILO1['x'], SILO1['y'], SILO1['yaw'])
    face, rev = drive_direction(at1, silo2)
    assert not rev and abs(wrap(face - SILO1['yaw'])) < math.radians(5)
    at3 = (0.581, -3.86, SILO1['yaw'])
    face, rev = drive_direction(at3, SILO1)            # silo 1 is behind it
    assert rev and abs(wrap(face - SILO1['yaw'])) < math.radians(15)  # no turning round


def test_entry_from_silo_1_is_a_quarter_turn_not_a_reverse():
    e = default_entry(SILO1)
    face, rev = drive_direction((SILO1['x'], SILO1['y'], SILO1['yaw']), e)
    assert not rev and abs(abs(wrap(face - SILO1['yaw'])) - math.pi / 2) < 0.05


def test_reverse_straight_move_converges_and_stops():
    # the mission's trick: steer with the back as the front, then negate v
    x, y, yaw = 0.581, -3.84, math.radians(170)   # a little off the row and its heading
    start, goal = (0.581, -3.86), (1.088, -3.923)
    dt = 0.05
    for _ in range(1000):
        v, w, _, done = straight_command((x, y, yaw + math.pi), start, goal, 0.08, max_w=0.4)
        if done:
            break
        x += -v * math.cos(yaw) * dt
        y += -v * math.sin(yaw) * dt
        yaw += w * dt
    assert done
    assert math.dist((x, y), goal) < 0.04


def test_last_silo():
    assert last_silo([0.4, 0.4, 0.2]) == 2
    assert last_silo([0.5, 0.0, 0.0]) == 0
    assert last_silo([0.0, 0.0, 0.0]) is None
