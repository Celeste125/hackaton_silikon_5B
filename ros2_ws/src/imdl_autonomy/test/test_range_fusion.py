import pytest

from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.range_fusion import (
    FrontRangeReading,
    evaluate_front_range,
    expected_front_range,
    target_geometry,
)
from imdl_autonomy.runtime_context import FrontRangeStore
from sensor_msgs.msg import Range


def _reading(value=0.063, samples=5):
    return FrontRangeReading(
        range_m=value,
        min_range_m=0.02,
        max_range_m=1.20,
        field_of_view_rad=0.2618,
        frame_id='ultrasonic_front_link',
        generation=10,
        sample_count=samples,
        received_wall=10.0,
        stamp_sec=4.0,
    )


def test_target_geometry_uses_robot_heading():
    geometry = target_geometry(
        Pose2D(0.0, 0.0, 1.57079632679),
        Pose2D(0.0, 0.27, 0.0),
    )

    assert geometry.distance_m == pytest.approx(0.27)
    assert geometry.forward_m == pytest.approx(0.27)
    assert geometry.lateral_m == pytest.approx(0.0, abs=1e-8)
    assert geometry.bearing_rad == pytest.approx(0.0, abs=1e-8)


def test_expected_range_is_from_sensor_face_to_block_surface():
    expected, _geometry = expected_front_range(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.27, 0.0, 0.0),
        sensor_offset_x_m=0.190,
        target_half_extent_m=0.0175,
    )

    assert expected == pytest.approx(0.0625)


def test_range_must_agree_with_camera_identified_target():
    evaluation = evaluate_front_range(
        _reading(0.064),
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.27, 0.0, 0.0),
        sensor_offset_x_m=0.190,
        target_half_extent_m=0.0175,
        tolerance_m=0.035,
        max_target_bearing_rad=0.10,
    )
    wrong_reflector = evaluate_front_range(
        _reading(0.020),
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.30, 0.0, 0.0),
        sensor_offset_x_m=0.190,
        target_half_extent_m=0.0175,
        tolerance_m=0.035,
        max_target_bearing_rad=0.10,
    )

    assert evaluation.consistent
    assert evaluation.error_m == pytest.approx(0.0015)
    assert not wrong_reflector.consistent


def test_range_rejects_target_outside_trusted_narrow_cone():
    evaluation = evaluate_front_range(
        _reading(0.064),
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.27, 0.06, 0.0),
        sensor_offset_x_m=0.190,
        target_half_extent_m=0.0175,
        tolerance_m=0.035,
        max_target_bearing_rad=0.10,
    )

    assert not evaluation.consistent
    assert 'outside the trusted range cone' in evaluation.reason


def test_range_store_requires_fresh_independent_samples(monkeypatch):
    callbacks = {}

    class Node:
        def create_subscription(self, _type, topic, callback, _qos):
            callbacks[topic] = callback

    now = {'value': 10.0}
    monkeypatch.setattr(
        'imdl_autonomy.runtime_context.time.monotonic',
        lambda: now['value'],
    )
    store = FrontRangeStore(Node())
    callback = callbacks['/ultrasonic/front/range']
    for value in (0.070, 0.060, 0.065):
        message = Range()
        message.header.frame_id = 'ultrasonic_front_link'
        message.min_range = 0.02
        message.max_range = 1.20
        message.field_of_view = 0.2618
        message.range = value
        callback(message)
        now['value'] += 0.02

    reading = store.measurement(
        max_age_sec=0.35,
        min_samples=3,
        not_before_wall=9.9,
    )

    assert reading is not None
    assert reading.range_m == pytest.approx(0.065)
    assert reading.sample_count == 3
    assert store.measurement(
        max_age_sec=0.35,
        min_samples=3,
        not_before_wall=10.03,
    ) is None
