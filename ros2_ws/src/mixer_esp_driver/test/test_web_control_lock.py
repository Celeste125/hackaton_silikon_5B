"""The web panel's control lock, tested through the real FastAPI app.

ROS is replaced by stubs, so this runs on any machine with fastapi and httpx.
"""
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import rclpy  # noqa: F401
except ImportError:
    def _stub(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod

    _stub('rclpy')
    _stub('rclpy.executors', ExternalShutdownException=RuntimeError)
    _stub('rclpy.node', Node=object)
    _stub('geometry_msgs')
    _stub('geometry_msgs.msg', Twist=object)
    _stub('std_msgs')
    _stub('std_msgs.msg', Float32=object, Float32MultiArray=object)
    _stub('std_srvs')
    _stub('std_srvs.srv', Trigger=object)

pytest.importorskip('fastapi')
pytest.importorskip('httpx')

from fastapi.testclient import TestClient  # noqa: E402

from mixer_esp_driver.control_lease import ControlLease  # noqa: E402
from mixer_esp_driver.web_control import create_app  # noqa: E402


class FakeRos:
    def __init__(self):
        self.now = 100.0
        self.lease = ControlLease(6.0, lambda: self.now)
        self.calls = []

    def state(self):
        return {'num_motors': 2, 'weight': None, 'cells': [], 'door': None,
                'valves': {}, 'dose': None, 'weight_age_s': None, 'motors': [0.0, 0.0]}

    def _rec(self, name, *args):
        self.calls.append((name,) + args)

    def set_motors(self, speeds): self._rec('motors', speeds)
    def set_drive(self, lin, ang): self._rec('drive', lin, ang)
    def set_door(self, pos): self._rec('door', pos)
    def set_door_raw(self, us): self._rec('door_raw', us)
    def set_valve(self, valve, pct): self._rec('valve', valve, pct)
    def stop_outputs(self): self._rec('stop_outputs')
    def emergency_stop(self): self._rec('emergency_stop')
    def close_all_valves(self): self._rec('close_all_valves')
    def cancel_dose(self): self._rec('cancel_dose')

    def start_dose(self, valve, kg, pct):
        self._rec('dose', valve, kg)
        return True, 'dosing'

    def call(self, name):
        self._rec('call', name)
        return True, 'ok'

    def names(self):
        return [c[0] for c in self.calls]


@pytest.fixture
def env():
    ros = FakeRos()
    return ros, TestClient(create_app(ros))


def hdr(cid):
    return {'X-Client-Id': cid}


def claim(client, cid, force=False):
    return client.post('/api/control/claim', json={'force': force}, headers=hdr(cid)).json()


COMMANDS = [
    ('/api/motors', {'speeds': [0.5, 0.0]}),
    ('/api/drive', {'linear': 0.1, 'angular': 0.0}),
    ('/api/door', {'position': 0.5}),
    ('/api/door/raw_us', {'us': 1500}),
    ('/api/door/save_open', None),
    ('/api/door/save_closed', None),
    ('/api/valves/1', {'percent': 50}),
    ('/api/valves/1/dose', {'target_kg': 0.5}),
    ('/api/tare', None),
]


@pytest.mark.parametrize('path,body', COMMANDS)
def test_commands_need_control(env, path, body):
    ros, client = env
    r = client.post(path, json=body, headers=hdr('viewer'))
    assert r.status_code == 403
    assert r.json()['detail']['error'] == 'not_in_control'
    assert ros.calls == []


@pytest.mark.parametrize('path,body', COMMANDS)
def test_holder_can_command(env, path, body):
    ros, client = env
    assert claim(client, 'a')['ok']
    ros.calls.clear()
    assert client.post(path, json=body, headers=hdr('a')).status_code == 200
    assert ros.calls


def test_missing_client_id_is_rejected(env):
    ros, client = env
    assert client.post('/api/motors', json={'speeds': [1, 1]}).status_code == 400
    assert client.post('/api/stop').status_code == 400
    assert ros.calls == []


def test_second_device_cannot_override_the_first(env):
    ros, client = env
    assert claim(client, 'a')['ok']
    other = claim(client, 'b')
    assert other['ok'] is False
    assert other['control']['you'] is False
    ros.calls.clear()
    assert client.post('/api/motors', json={'speeds': [0, 0]}, headers=hdr('b')).status_code == 403
    assert ros.calls == []
    assert client.post('/api/motors', json={'speeds': [1, 1]}, headers=hdr('a')).status_code == 200


def test_takeover_stops_outputs_and_locks_out_the_old_holder(env):
    ros, client = env
    claim(client, 'a')
    ros.calls.clear()
    assert claim(client, 'b', force=True)['ok']
    assert ros.names() == ['stop_outputs']
    assert client.post('/api/motors', json={'speeds': [1, 1]}, headers=hdr('a')).status_code == 403
    assert client.post('/api/motors', json={'speeds': [1, 1]}, headers=hdr('b')).status_code == 200


def test_anyone_can_stop(env):
    ros, client = env
    claim(client, 'a')
    ros.calls.clear()
    assert client.post('/api/stop', headers=hdr('viewer')).status_code == 200
    assert client.post('/api/valves/close_all', headers=hdr('viewer')).status_code == 200
    assert client.post('/api/dose/cancel', headers=hdr('viewer')).status_code == 200
    assert ros.names() == ['emergency_stop', 'close_all_valves', 'cancel_dose']


def test_lease_expires_and_a_viewer_can_claim_without_force(env):
    ros, client = env
    claim(client, 'a')
    ros.now += 7.0
    assert client.post('/api/motors', json={'speeds': [1, 1]}, headers=hdr('a')).status_code == 403
    assert claim(client, 'b')['ok']


def test_holder_polling_keeps_control_alive_but_viewer_polling_does_not(env):
    ros, client = env
    claim(client, 'a')
    for _ in range(4):
        ros.now += 5.0
        client.get('/api/state', headers=hdr('viewer'))
        state = client.get('/api/state', headers=hdr('a')).json()
        assert state['control']['you'] is True
    assert client.get('/api/state', headers=hdr('viewer')).json()['control']['you'] is False


def test_state_works_for_clients_without_an_id(env):
    _, client = env
    r = client.get('/api/state')
    assert r.status_code == 200
    assert r.json()['control'] == {'held': False, 'you': False, 'holder': None}


def test_release_frees_control_and_stops_outputs(env):
    ros, client = env
    claim(client, 'a')
    ros.calls.clear()
    assert client.post('/api/control/release', headers=hdr('a')).status_code == 200
    assert ros.names() == ['stop_outputs']
    assert claim(client, 'b')['ok']


def test_release_by_a_non_holder_changes_nothing(env):
    ros, client = env
    claim(client, 'a')
    ros.calls.clear()
    client.post('/api/control/release', headers=hdr('b'))
    assert ros.names() == []
    assert client.get('/api/state', headers=hdr('a')).json()['control']['you'] is True
