from imdl_autonomy.create3_adapter import (
    Create3SafetyAdapter,
    Create3SafetyState,
)


class _Client:
    @staticmethod
    def service_is_ready():
        return False


class _Node:
    def __init__(self):
        self.services = []

    def create_client(self, _service_type, service_name):
        self.services.append(service_name)
        return _Client()


def test_create3_adapter_uses_remote_parameter_services():
    node = _Node()
    adapter = Create3SafetyAdapter(node, '/Team2/motion_control')

    assert node.services == [
        '/Team2/motion_control/get_parameters',
        '/Team2/motion_control/set_parameters',
    ]
    assert not adapter.enable_backup_only()
    assert adapter.snapshot().state is Create3SafetyState.FAILED
    assert 'unavailable' in adapter.snapshot().message


def test_restore_is_noop_when_adapter_changed_nothing():
    adapter = Create3SafetyAdapter(_Node(), '/Team2/motion_control')

    assert adapter.restore()
    assert adapter.snapshot().state is Create3SafetyState.SUCCEEDED
