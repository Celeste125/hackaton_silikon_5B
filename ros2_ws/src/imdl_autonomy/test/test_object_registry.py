from imdl_interfaces.msg import TrackedObject, TrackedObjectArray
import pytest

from imdl_autonomy.object_registry import square_obstacle_points
from imdl_autonomy.placed_objects import (
    placement_event_json,
    PlacedObjectZones,
)
from imdl_autonomy.runtime_context import ObjectStore


class NodeMock:
    def create_subscription(self, *_args, **_kwargs):
        return object()


def test_confirmed_block_cloud_covers_center_edges_and_corners():
    points = square_obstacle_points(
        x=0.4,
        y=-0.2,
        z=0.0175,
        half_extent_m=0.0175,
    )

    assert len(points) == 9
    assert (0.4, -0.2, 0.0175) in points
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    assert min(xs) == pytest.approx(0.3825)
    assert max(xs) == pytest.approx(0.4175)
    assert min(ys) == pytest.approx(-0.2175)
    assert max(ys) == pytest.approx(-0.1825)


def test_completed_placement_zone_suppresses_only_nearby_obstacles():
    zones = PlacedObjectZones('map', default_radius_m=0.075)
    payload = placement_event_json(
        object_id='red-001',
        color='red',
        frame_id='map',
        x=0.0,
        y=0.55,
        radius_m=0.075,
    )

    zone = zones.update(payload)

    assert zone is not None
    assert zones.contains(0.03, 0.55)
    assert not zones.contains(0.20, 0.55)
    assert zones.update('{"event": "placed", "frame_id": "odom"}') is None


def test_mission_cache_retains_confirmed_pose_but_lock_invalidates_it():
    store = ObjectStore(NodeMock())
    array = TrackedObjectArray()
    item = TrackedObject()
    item.object_id = 'blue-007'
    item.color = 'blue'
    item.pose.pose.position.x = 0.4
    item.pose.pose.position.y = -0.2
    item.pose.pose.orientation.w = 1.0
    item.confidence = 0.9
    item.confirmations = 4
    item.status = TrackedObject.STATUS_CONFIRMED
    array.objects.append(item)

    store._callback(array)
    store._callback(TrackedObjectArray())

    assert store.confirmed('blue') is None
    assert store.cached('blue').object_id == 'blue-007'
    store.lock('blue-007')
    assert store.cached('blue') is None


def test_forgetting_stale_cache_allows_future_reconfirmation():
    store = ObjectStore(NodeMock())
    array = TrackedObjectArray()
    item = TrackedObject()
    item.object_id = 'red-002'
    item.color = 'red'
    item.pose.pose.orientation.w = 1.0
    item.confidence = 0.8
    item.status = TrackedObject.STATUS_CONFIRMED
    array.objects.append(item)

    store._callback(array)
    store.forget_cached('red-002')
    assert store.cached('red') is None

    store._callback(array)
    assert store.cached('red').object_id == 'red-002'
