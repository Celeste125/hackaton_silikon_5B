import math

import numpy as np

from perception.object_geometry_utils import (
    masked_depth_statistics,
    masked_depth_median,
    pixel_to_camera_point,
    polygon_mask,
    polar_range_and_bearing,
    rectangle_corners_from_polygon,
)


SQUARE = [[1, 1], [4, 1], [4, 4], [1, 4]]


def test_masked_depth_median_ignores_background_and_outlier():
    depth = np.full((6, 6), 3000, dtype=np.uint16)
    depth[1:5, 1:5] = 500
    depth[2, 2] = 1800

    value = masked_depth_median(
        depth, SQUARE, 6, 6, 0.05, 3.0, 0.10, min_valid_pixels=5
    )

    assert value == 0.5


def test_masked_depth_median_returns_none_without_enough_valid_depth():
    depth = np.zeros((6, 6), dtype=np.uint16)
    assert masked_depth_median(
        depth, SQUARE, 6, 6, 0.05, 3.0, 0.10, min_valid_pixels=3
    ) is None


def test_polygon_mask_scales_to_depth_resolution():
    mask = polygon_mask([[0, 0], [8, 0], [8, 8], [0, 8]], 8, 8, 4, 4)
    assert mask.shape == (4, 4)
    assert mask.all()


def test_pixel_to_camera_and_polar_projection():
    point = pixel_to_camera_point(330, 250, 2.0, [100, 0, 320, 0, 100, 240, 0, 0, 1])
    assert point == {'x': 0.2, 'y': 0.2, 'z': 2.0}
    range_m, bearing = polar_range_and_bearing({'x': 3.0, 'y': 4.0, 'z': 0.0})
    assert range_m == 5.0
    assert bearing == math.atan2(4.0, 3.0)


def test_depth_statistics_reports_low_valid_depth_ratio():
    depth = np.zeros((6, 6), dtype=np.uint16)
    depth[1, 1] = 500
    quality = masked_depth_statistics(
        depth, SQUARE, 6, 6, 0.05, 3.0, 0.1, 4, 1, 0.5, 0.04
    )
    assert quality['status'] == 'low_valid_depth_ratio'
    assert quality['depth_m'] is None


def test_depth_statistics_reports_unstable_depth():
    depth = np.zeros((6, 6), dtype=np.uint16)
    depth[1:5, 1:3] = 500
    depth[1:5, 3:5] = 580
    quality = masked_depth_statistics(
        depth, SQUARE, 6, 6, 0.05, 3.0, 0.2, 4, 4, 0.2, 0.02
    )
    assert quality['status'] == 'depth_unstable'


def test_rectangle_corners_are_available_for_compact_mask():
    corners = rectangle_corners_from_polygon(SQUARE, 4, 0.8)
    assert corners['corner_status'] == 'ok'
    assert len(corners['corners_px']) == 4


def test_rectangle_corners_reject_slender_fragment():
    fragment = [[0, 0], [20, 0], [0, 1]]
    corners = rectangle_corners_from_polygon(fragment, 1, 0.8)
    assert corners['corner_status'] == 'low_rect_fill_ratio'
