import importlib.util
import math
from pathlib import Path

import pytest
from sensor_msgs.msg import LaserScan


FILTER_PATH = (
    Path(__file__).parents[2]
    / 'imdl_simulation'
    / 'scripts'
    / 'scan_self_filter.py'
)
SPEC = importlib.util.spec_from_file_location(
    'scan_self_filter',
    FILTER_PATH,
)
FILTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FILTER)


def test_default_footprint_contains_camera_but_not_real_obstacle():
    polygon = FILTER.parse_footprint(FILTER.DEFAULT_SELF_FOOTPRINT)

    assert FILTER.point_in_polygon(0.155, 0.0, polygon)
    assert FILTER.point_in_polygon(0.19, 0.0, polygon)
    assert not FILTER.point_in_polygon(0.205, 0.0, polygon)
    assert not FILTER.point_in_polygon(0.0, 0.20, polygon)


def test_filter_removes_only_endpoints_inside_robot_polygon():
    polygon = FILTER.parse_footprint(FILTER.DEFAULT_SELF_FOOTPRINT)
    ranges, removed = FILTER.filter_self_returns(
        [0.15, 0.25, math.inf, 0.16, 0.30],
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        polygon=polygon,
    )

    assert removed == 2
    assert math.isinf(ranges[0])
    assert ranges[1] == pytest.approx(0.25)
    assert math.isinf(ranges[2])
    assert math.isinf(ranges[3])
    assert ranges[4] == pytest.approx(0.30)


def test_filtered_scan_preserves_laserscan_contract():
    scan = LaserScan()
    scan.header.frame_id = 'lidar_link'
    scan.angle_min = -0.1
    scan.angle_max = 0.1
    scan.angle_increment = 0.1
    scan.time_increment = 0.001
    scan.scan_time = 0.1
    scan.range_min = 0.12
    scan.range_max = 8.0
    scan.ranges = [0.13, 0.40, 0.13]
    scan.intensities = [1.0, 2.0, 3.0]

    message, removed = FILTER.filtered_scan(
        scan,
        FILTER.parse_footprint(FILTER.DEFAULT_SELF_FOOTPRINT),
    )

    assert removed == 2
    assert message.header.frame_id == 'lidar_link'
    assert message.range_min == pytest.approx(0.12)
    assert message.range_max == pytest.approx(8.0)
    assert message.ranges[1] == pytest.approx(0.40)
    assert list(message.intensities) == [1.0, 2.0, 3.0]
