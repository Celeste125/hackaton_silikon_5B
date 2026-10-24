import math
from unittest.mock import MagicMock

import numpy as np

from imdl_statemachine.markers import marker_in_frame, marker_yaw, standoff_goal
from imdl_statemachine.state_machine import State, WaypointStateMachine


def test_standoff_goal_faces_marker():
    x, y, yaw = standoff_goal(2.0, 1.0, math.pi / 2, 0.8)  # marker faces +y
    assert (round(x, 3), round(y, 3)) == (2.0, 1.8)
    assert abs(abs(yaw) - math.pi / 2) < 1e-6 and yaw < 0   # robot looks toward -y


def test_marker_in_frame_identity_transform():
    rot = np.eye(3)
    center, normal = marker_in_frame(rot, [1, 2, 0], np.eye(3), [0, 0, 0])
    assert list(center) == [1, 2, 0] and list(normal) == [0, 0, 1]
    assert marker_yaw(normal) is None   # pointing up has no usable heading


def make(**marker):
    config = {'marker_route': {'enabled': True, 'ids': [0, 1], **marker}}
    machine = WaypointStateMachine(MagicMock(), config)
    machine.sleep = lambda seconds: None
    machine.pose = lambda p: p
    return machine


def test_visits_markers_in_order_then_checks_mix():
    machine = make()
    machine.marker_goals = {0: [0, 0, 0], 1: [1, 0, 0]}
    machine.run_task = lambda label, start: True
    assert machine.on_marker_route() == State.MARKER_ROUTE
    assert machine.on_marker_route() == State.MARKER_ROUTE
    assert machine.on_marker_route() == State.CHECK_MIX
    assert machine.marker_index == 0


def test_unseen_marker_not_found_fails():
    machine = make(search_step_rad=3.2)
    machine.run_task = lambda label, start: True
    assert machine.on_marker_route() == State.FAILED


def test_extra_lap_repeats_marker_route():
    assert make().on_extra_lap() == State.MARKER_ROUTE


def test_non_finite_marker_pose_is_ignored():
    from geometry_msgs.msg import PoseStamped
    machine = make()
    msg = PoseStamped()
    msg.pose.position.x = float('nan')
    machine._on_marker(0, msg)
    assert 0 not in machine.marker_goals
    msg.pose.position.x = 1.0
    msg.pose.orientation.w = 1.0
    machine._on_marker(0, msg)
    assert 0 in machine.marker_goals


def test_route_puts_refill_marker_first():
    assert make(ids=[0, 1, 2], refill_marker=1).route_ids() == [1, 0, 2]
    assert make(ids=[0, 1, 2]).route_ids() == [0, 1, 2]


def test_unknown_refill_marker_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        make(ids=[0, 1], refill_marker=7)


def test_loop_returns_to_refill_after_a_good_lap():
    machine = make()
    assert machine.on_check_mix() == State.REFILL          # no mix data: carry on
    machine.loop = False
    assert machine.on_check_mix() == State.TURN


def test_refill_waits_then_restarts_the_feeding_stops():
    machine = make(refill_wait_sec=0)
    machine.marker_goals = {0: [0, 0, 0], 1: [1, 0, 0]}
    machine.run_task = lambda label, start: True
    machine.extra_laps = 2
    assert machine.on_refill() == State.MARKER_ROUTE
    assert (machine.marker_index, machine.laps, machine.extra_laps) == (1, 1, 0)


def test_extra_lap_skips_the_refill_marker():
    machine = make()
    assert machine.on_extra_lap() == State.MARKER_ROUTE
    assert machine.marker_index == 1


def test_stop_message_ends_the_mission():
    from std_msgs.msg import Empty
    machine = make()
    machine._handlers[State.INIT] = lambda: State.REFILL
    machine._on_stop(Empty())
    assert machine.run() is True and machine.state == State.DONE
