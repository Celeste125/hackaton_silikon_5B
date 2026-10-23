import importlib.util
import math
from pathlib import Path

import pytest
from sensor_msgs.msg import LaserScan, Range


ADAPTER_PATH = (
    Path(__file__).parents[2]
    / 'imdl_simulation'
    / 'scripts'
    / 'ultrasonic_range_adapter.py'
)
SPEC = importlib.util.spec_from_file_location(
    'ultrasonic_range_adapter',
    ADAPTER_PATH,
)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


def test_representative_range_uses_median_and_rejects_invalid_echoes():
    value = ADAPTER.representative_range(
        [math.inf, 0.111, 0.115, 0.900, math.nan, 0.010, 1.300],
        0.020,
        1.200,
    )
    assert value == pytest.approx(0.115)


def test_representative_range_reports_clear_cone_as_max_range():
    assert ADAPTER.representative_range(
        [math.inf, math.nan, 0.010, 1.300],
        0.020,
        1.200,
    ) == pytest.approx(1.200)


def test_scan_to_range_preserves_geometry_and_publishes_ultrasound():
    scan = LaserScan()
    scan.header.frame_id = 'ultrasonic_front_link'
    scan.angle_min = -0.1309
    scan.angle_max = 0.1309
    scan.range_min = 0.020
    scan.range_max = 1.200
    scan.ranges = [0.112, 0.114, 0.116]

    message = ADAPTER.scan_to_range(scan)

    assert message.header.frame_id == 'ultrasonic_front_link'
    assert message.radiation_type == Range.ULTRASOUND
    assert message.field_of_view == pytest.approx(0.2618)
    assert message.min_range == pytest.approx(0.020)
    assert message.max_range == pytest.approx(1.200)
    assert message.range == pytest.approx(0.114)
