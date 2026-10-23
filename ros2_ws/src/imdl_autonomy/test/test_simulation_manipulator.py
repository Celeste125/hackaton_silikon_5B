import math
from pathlib import Path
import threading
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from imdl_interfaces.srv import ManipulateObject
import pytest

from imdl_autonomy.runtime_context import ManipulatorClient, ManipulatorState
from imdl_autonomy.placed_objects import (
    parse_placement_event,
    placement_event_json,
)
from imdl_autonomy.simulation_manipulator import (
    pick_geometry,
    placement_height,
    SimulationManipulatorNode,
)


def test_pick_geometry_normalizes_bearing():
    distance, bearing = pick_geometry(
        (0.0, 0.0, math.pi - 0.1),
        -0.3,
        -0.03,
    )

    assert distance == pytest.approx(math.hypot(0.3, 0.03))
    assert abs(bearing) < 0.25


def test_three_blocks_are_stacked_flush_at_exact_center_heights():
    heights = [placement_height(0.035, index) for index in range(3)]

    assert heights == pytest.approx([0.0175, 0.0525, 0.0875])
    assert heights[1] - heights[0] == pytest.approx(0.035)
    assert heights[2] - heights[1] == pytest.approx(0.035)


def test_manipulator_placement_event_has_stable_hardware_adapter_contract():
    payload = placement_event_json(
        object_id='blue-004',
        color='blue',
        frame_id='map',
        x=0.1,
        y=-0.2,
        radius_m=0.075,
    )

    event = parse_placement_event(payload, default_radius_m=0.05)

    assert event.object_id == 'blue-004'
    assert event.color == 'blue'
    assert event.frame_id == 'map'
    assert event.x == pytest.approx(0.1)
    assert event.y == pytest.approx(-0.2)
    assert event.radius_m == pytest.approx(0.075)


def test_transport_world_blocks_are_movable_and_keep_collisions():
    src = Path(__file__).resolve().parents[2]
    worlds = src / 'imdl_simulation' / 'worlds'
    files = sorted(
        path
        for task in ('3', '4', '5')
        for path in worlds.glob(f'arena_task{task}*.world')
    )

    assert len(files) == 9
    for path in files:
        root = ET.parse(path).getroot()
        state_plugin = root.find(
            ".//world/plugin[@filename='libgazebo_ros_state.so']"
        )
        assert state_plugin is not None, path.name
        assert float(state_plugin.findtext('update_rate')) <= 1.0
        blocks = [
            model for model in root.findall('.//model')
            if model.get('name', '').endswith('_block')
        ]
        assert blocks, path.name
        for model in blocks:
            assert model.findtext('static') == 'false', path.name
            link = model.find('link')
            assert link is not None
            assert link.findtext('gravity') == 'false'
            assert link.find('inertial') is not None
            assert link.find('collision') is not None


class FutureMock:
    def add_done_callback(self, _callback):
        pass


class ClientMock:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return FutureMock()


class LoggerMock:
    def info(self, *_args):
        pass

    def error(self, *_args):
        pass


class NodeMock:
    def __init__(self):
        self.client = ClientMock()

    def create_client(self, *_args):
        return self.client

    def get_logger(self):
        return LoggerMock()


def test_cancel_queues_idempotent_release_request():
    node = NodeMock()
    client = ManipulatorClient(node, 'map')
    client.state = ManipulatorState.ACTIVE
    client._active_object_id = 'red-001'
    client._active_color = 'red'

    client.cancel()

    assert client.state is ManipulatorState.CANCELED
    assert len(node.client.requests) == 1
    request = node.client.requests[0]
    assert request.operation == ManipulateObject.Request.RELEASE
    assert request.object_id == 'red-001'
    assert request.color == 'red'


def test_release_restores_exact_pre_pick_entity_pose():
    node = object.__new__(SimulationManipulatorNode)
    original = (0.35, -0.22, 0.0175, 0.4)
    node.carried = ('red-001', 'red', 'red_block', original)
    node.carry_updates_paused = False
    node.carry_update_done = threading.Event()
    node.carry_update_done.set()
    restored = []
    node.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    node._set_entity_sync = (
        lambda entity, *pose: restored.append((entity, pose)) or True
    )
    node.get_logger = lambda: LoggerMock()
    response = ManipulateObject.Response()

    result = node._release(None, response)

    assert result.success
    assert restored == [('red_block', original)]
    assert node.carried is None
    assert not node.carry_updates_paused
