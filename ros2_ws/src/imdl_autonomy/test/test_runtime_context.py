import json

from std_msgs.msg import Bool, String

from imdl_autonomy.runtime_context import LocalizationStore, SafetyStore


class SubscriptionNode:
    def create_subscription(self, *_args, **_kwargs):
        return object()


def test_localization_store_decodes_runtime_json():
    store = LocalizationStore(SubscriptionNode())
    store._callback(String(data=json.dumps({
        'mission_id': 'task1_a',
        'ready': True,
        'scan_required': False,
        'reasons': [],
        'position_std_m': 0.02,
        'pose': {'x': 0.1, 'y': -0.2, 'yaw': 0.3},
    })))

    assert store.received
    assert store.ready
    assert store.mission_id == 'task1_a'
    assert store.pose.x == 0.1


def test_safety_store_decodes_runtime_json():
    store = SafetyStore(SubscriptionNode())
    store._stop_cb(Bool(data=False))
    store._state_cb(String(data=json.dumps({
        'safe': True,
        'reasons': [],
    })))

    assert store.received
    assert store.safe
    assert store.reasons == []


def test_runtime_stores_ignore_invalid_json():
    localization = LocalizationStore(SubscriptionNode())
    safety = SafetyStore(SubscriptionNode())

    localization._callback(String(data='{'))
    safety._state_cb(String(data='{'))

    assert not localization.received
    assert not safety.received
