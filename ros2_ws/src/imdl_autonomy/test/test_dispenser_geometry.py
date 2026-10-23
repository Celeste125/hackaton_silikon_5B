"""Geometry checks for dispenser_locator.

The depth sampling and the deprojection decide how accurate the estimate is,
and neither needs a robot to verify, so they are pinned here. A sign flip or a
millimetre/metre mix-up would otherwise only show up as an estimate quietly
landing in the wrong place.
"""

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image

from imdl_autonomy.dispenser_locator import DispenserLocator

WIDTH, HEIGHT = 640, 480


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    instance = DispenserLocator()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def _depth_image(grid: np.ndarray, encoding: str) -> Image:
    bytes_per_pixel = 2 if encoding == '16UC1' else 4
    message = Image(
        height=HEIGHT, width=WIDTH, encoding=encoding,
        step=WIDTH * bytes_per_pixel,
    )
    message.data = grid.tobytes()
    return message


def _full_box():
    return [280 / WIDTH, 200 / HEIGHT, 360 / WIDTH, 280 / HEIGHT]


def test_millimetre_depth_becomes_metres(node):
    grid = np.full((HEIGHT, WIDTH), 60000, dtype=np.uint16)
    grid[200:280, 280:360] = 2000
    depth, u, v = node._depth_in_box(_depth_image(grid, '16UC1'), _full_box())
    assert depth == pytest.approx(2.0)
    assert (u, v) == (320, 240)


def test_float_depth_is_already_metres(node):
    grid = np.full((HEIGHT, WIDTH), 50.0, dtype=np.float32)
    grid[200:280, 280:360] = 1.25
    depth, _, _ = node._depth_in_box(_depth_image(grid, '32FC1'), _full_box())
    assert depth == pytest.approx(1.25)


def test_depth_outside_the_window_is_rejected(node):
    """9 m is past max_depth_m: no estimate beats a confidently wrong one."""
    grid = np.full((HEIGHT, WIDTH), 9000, dtype=np.uint16)
    depth, _, _ = node._depth_in_box(
        _depth_image(grid, '16UC1'), [0.4, 0.4, 0.6, 0.6])
    assert depth is None


def test_only_the_middle_of_the_box_is_sampled(node):
    """The box edges straddle whatever is behind the dispenser."""
    grid = np.full((HEIGHT, WIDTH), 3000, dtype=np.uint16)
    grid[220:260, 300:340] = 1000
    depth, _, _ = node._depth_in_box(
        _depth_image(grid, '16UC1'),
        [240 / WIDTH, 180 / HEIGHT, 400 / WIDTH, 300 / HEIGHT])
    assert depth == pytest.approx(1.0)


def _camera_info():
    info = CameraInfo()
    info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    return info


def test_principal_point_deprojects_straight_ahead(node):
    point = node._deproject(_camera_info(), 320, 240, 2.0)
    assert (point.point.x, point.point.y) == pytest.approx((0.0, 0.0))
    assert point.point.z == pytest.approx(2.0)


def test_offset_pixel_deprojects_by_similar_triangles(node):
    """100 px right of centre at 2 m with fx=600 is 100*2/600 m across."""
    point = node._deproject(_camera_info(), 420, 240, 2.0)
    assert point.point.x == pytest.approx(100 * 2.0 / 600.0)
    assert point.point.y == pytest.approx(0.0)
