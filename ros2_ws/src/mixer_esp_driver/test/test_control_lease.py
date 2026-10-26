import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mixer_esp_driver.control_lease import ControlLease  # noqa: E402


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def make(timeout=6.0):
    clock = Clock()
    return ControlLease(timeout, clock), clock


def test_free_lease_can_be_claimed():
    lease, _ = make()
    assert lease.claim('a', 'A') == (True, True)
    assert lease.snapshot('a') == {'held': True, 'you': True, 'holder': 'A'}


def test_other_client_is_refused_without_force():
    lease, _ = make()
    lease.claim('a', 'A')
    assert lease.claim('b', 'B') == (False, False)
    assert lease.snapshot('b') == {'held': True, 'you': False, 'holder': 'A'}
    assert lease.touch('b') is False


def test_force_takes_over_and_reports_change():
    lease, _ = make()
    lease.claim('a', 'A')
    assert lease.claim('b', 'B', force=True) == (True, True)
    assert lease.touch('a') is False
    assert lease.touch('b') is True


def test_reclaiming_own_lease_is_not_a_change():
    lease, _ = make()
    lease.claim('a', 'A')
    assert lease.claim('a', 'A') == (True, False)


def test_lease_expires_when_holder_goes_silent():
    lease, clock = make(timeout=6.0)
    lease.claim('a', 'A')
    clock.now += 6.5
    assert lease.snapshot('a')['held'] is False
    assert lease.touch('a') is False
    assert lease.claim('b', 'B') == (True, True)


def test_touch_keeps_the_lease_alive():
    lease, clock = make(timeout=6.0)
    lease.claim('a', 'A')
    for _ in range(5):
        clock.now += 5.0
        assert lease.touch('a') is True
    assert lease.snapshot('b')['holder'] == 'A'


def test_release_only_by_the_holder():
    lease, _ = make()
    lease.claim('a', 'A')
    assert lease.release('b') is False
    assert lease.snapshot('a')['held'] is True
    assert lease.release('a') is True
    assert lease.snapshot('a')['held'] is False
