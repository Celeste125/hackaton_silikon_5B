import numpy as np
import pytest
from sensor_msgs.msg import Image

from imdl_autonomy.depth_obstacle_projector import (
    depth_image_to_meters,
    project_low_obstacle_points,
    quaternion_rotation_matrix,
)
from imdl_autonomy.placed_objects import (
    exclude_camera_points_in_zones,
    PlacedObjectZone,
)


def make_depth_message(values, encoding):
    array = np.asarray(values)
    message = Image()
    message.height, message.width = array.shape
    message.encoding = encoding
    message.is_bigendian = False
    message.step = message.width * array.dtype.itemsize
    message.data = array.tobytes()
    return message


def test_depth_image_conversion_supports_simulation_and_realsense():
    millimetres = make_depth_message(
        np.array([[250, 800]], dtype=np.uint16),
        '16UC1',
    )
    metres = make_depth_message(
        np.array([[0.25, 0.80]], dtype=np.float32),
        '32FC1',
    )

    np.testing.assert_allclose(
        depth_image_to_meters(millimetres),
        [[0.25, 0.80]],
    )
    np.testing.assert_allclose(
        depth_image_to_meters(metres),
        [[0.25, 0.80]],
    )


def test_projection_keeps_block_height_and_rejects_floor():
    depth = np.zeros((9, 9), dtype=np.float32)
    depth[5, 4] = 0.40
    depth[7, 4] = 0.40
    camera_matrix = [
        8.0, 0.0, 4.0,
        0.0, 8.0, 4.0,
        0.0, 0.0, 1.0,
    ]
    # Standard optical axes at zero camera pitch:
    # base x <- optical z, base y <- -optical x, base z <- -optical y.
    optical_to_base = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    points = project_low_obstacle_points(
        depth,
        camera_matrix,
        optical_to_base,
        (0.18, 0.0, 0.08),
        pixel_stride=1,
        minimum_range_m=0.08,
        maximum_range_m=0.90,
        minimum_height_m=0.008,
        maximum_height_m=0.080,
        voxel_size_m=0.01,
        maximum_points=20,
    )

    assert points.shape == (1, 3)
    assert points[0] == pytest.approx([0.0, 0.05, 0.40])


def test_projection_caps_cloud_and_quaternion_is_normalized():
    depth = np.full((20, 20), 0.30, dtype=np.float32)
    rotation = quaternion_rotation_matrix((0.0, 0.0, 0.0, 2.0))
    points = project_low_obstacle_points(
        depth,
        [
            30.0, 0.0, 9.5,
            0.0, 30.0, 9.5,
            0.0, 0.0, 1.0,
        ],
        rotation,
        (0.0, 0.0, -0.25),
        pixel_stride=1,
        minimum_range_m=0.08,
        maximum_range_m=0.90,
        minimum_height_m=0.0,
        maximum_height_m=0.10,
        voxel_size_m=0.001,
        maximum_points=17,
    )

    assert rotation == pytest.approx(np.eye(3))
    assert len(points) == 17


def test_completed_drop_zone_is_removed_without_hiding_other_depth_geometry():
    camera_points = np.array([
        [0.00, 0.00, 0.30],
        [0.20, 0.00, 0.30],
    ], dtype=np.float32)
    zones = [PlacedObjectZone(
        object_id='red-001',
        color='red',
        frame_id='map',
        x=0.0,
        y=0.0,
        radius_m=0.075,
    )]

    filtered = exclude_camera_points_in_zones(
        camera_points,
        np.eye(3),
        (0.0, 0.0, 0.0),
        zones,
    )

    assert filtered.shape == (1, 3)
    assert filtered[0] == pytest.approx([0.20, 0.00, 0.30])
