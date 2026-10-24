import time
from unittest.mock import MagicMock

from imdl_statemachine.state_machine import State, WaypointStateMachine
from std_msgs.msg import Float32


def make(quality=None, age=0.0, **config):
    machine = WaypointStateMachine(MagicMock(), config)
    machine.sleep = lambda seconds: None
    if quality is not None:
        machine._on_mix_quality(Float32(data=quality))
        machine.mix_stamp -= age
    return machine


def test_good_mix_moves_on():
    assert make(0.9).on_check_mix() == State.TURN


def test_bad_mix_triggers_extra_lap():
    assert make(0.5).on_check_mix() == State.EXTRA_LAP


def test_gives_up_after_max_extra_laps():
    machine = make(0.5, max_extra_laps=2)
    machine.extra_laps = 2
    assert machine.on_check_mix() == State.TURN


def test_no_message_does_not_block():
    assert make().on_check_mix() == State.TURN


def test_stale_message_does_not_block():
    assert make(0.1, age=10.0).on_check_mix() == State.TURN


def test_disabled_check_is_skipped():
    assert make(0.1, mix_check_enabled=False).on_check_mix() == State.TURN


def test_extra_lap_goes_back_to_route_start():
    machine = make(0.5, route_start=[0, 0, 0])
    machine.pose = lambda xyz_yaw: None
    machine.run_task = lambda label, start: True
    assert machine.on_extra_lap() == State.SLOW_ROUTE
    assert machine.extra_laps == 1
