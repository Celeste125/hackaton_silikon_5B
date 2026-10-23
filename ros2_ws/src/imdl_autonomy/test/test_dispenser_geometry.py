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


# --------------------------------------------------------------------------
# Bearing-only geometry: the path a depthless camera such as a C920 takes.
# --------------------------------------------------------------------------

import math

from nav_msgs.msg import OccupancyGrid


def test_intrinsics_fall_back_to_field_of_view(node):
    """v4l2_camera publishes a zeroed K until calibrated."""
    info = CameraInfo()
    info.k = [0.0] * 9
    colour = Image(width=640, height=480)
    fx, fy, cx, cy = node._intrinsics(info, colour)
    hfov = math.radians(node.get_parameter('fallback_hfov_deg').value)
    assert fx == pytest.approx((640 / 2) / math.tan(hfov / 2))
    assert (cx, cy) == pytest.approx((320.0, 240.0))
    assert fy == pytest.approx(fx)


def test_calibrated_intrinsics_are_preferred(node):
    info = CameraInfo()
    info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    fx, _, cx, _ = node._intrinsics(info, Image(width=640, height=480))
    assert (fx, cx) == pytest.approx((600.0, 320.0))


def test_perpendicular_bearings_intersect(node):
    """A ray east from the origin and one north from (2, -2) meet at (2, 0)."""
    node._rays = [(np.array([0.0, 0.0]), np.array([1.0, 0.0]))]
    point = node._triangulate(np.array([2.0, -2.0]), np.array([0.0, 1.0]))
    assert point == pytest.approx((2.0, 0.0), abs=1e-9)


def test_near_parallel_bearings_are_refused(node):
    """Barely moving leaves the intersection unconstrained along the view."""
    node._rays = [(np.array([0.0, 0.0]), np.array([1.0, 0.0]))]
    assert node._triangulate(np.array([0.0, 0.1]), np.array([1.0, 0.0])) is None


def _grid_with_wall_at_x(wall_x: float, fill: int) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.info.resolution = 0.05
    grid.info.width = 200
    grid.info.height = 200
    grid.info.origin.position.x = -5.0
    grid.info.origin.position.y = -5.0
    cells = np.full((200, 200), fill, dtype=np.int8)
    column = int((wall_x - grid.info.origin.position.x) / grid.info.resolution)
    cells[:, column:column + 2] = 100
    grid.data = cells.flatten().tolist()
    return grid


def test_raycast_stops_at_the_first_wall(node):
    node._map = _grid_with_wall_at_x(2.0, fill=0)
    hit = node._raycast(np.array([0.0, 0.0]), np.array([1.0, 0.0]))
    assert hit is not None
    assert hit[0] == pytest.approx(2.0, abs=0.06)
    assert hit[1] == pytest.approx(0.0, abs=1e-9)


def test_raycast_passes_through_unexplored_cells(node):
    """-1 is unknown, not an obstacle; stopping there would be wrong."""
    node._map = _grid_with_wall_at_x(2.0, fill=-1)
    hit = node._raycast(np.array([0.0, 0.0]), np.array([1.0, 0.0]))
    assert hit is not None
    assert hit[0] == pytest.approx(2.0, abs=0.06)


def test_raycast_gives_up_past_its_range(node):
    node._map = _grid_with_wall_at_x(2.0, fill=0)
    # Pointing away from the wall: nothing to hit before the grid edge.
    assert node._raycast(np.array([0.0, 0.0]), np.array([-1.0, 0.0])) is None
