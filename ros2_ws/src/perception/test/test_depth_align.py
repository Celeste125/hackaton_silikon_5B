import numpy as np
from perception.depth_align import (
    depth_image_to_meters,
    depth_meters_to_millimeters,
    project_depth_to_color,
    streams_are_pixel_aligned,
)
import pytest


CAMERA_K = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]


def test_32fc1_depth_is_already_in_metres():
    depth = np.array([[1.25]], dtype=np.float32)

    converted = depth_image_to_meters(depth, '32FC1')

    assert converted.dtype == np.float32
    assert converted[0, 0] == pytest.approx(1.25)


def test_16uc1_depth_is_converted_from_millimetres():
    depth = np.array([[1250]], dtype=np.uint16)

    converted = depth_image_to_meters(depth, '16UC1')

    assert converted[0, 0] == pytest.approx(1.25)


def test_identity_projection_preserves_pixels_and_millimetres():
    depth = np.array(
        [[1.0, 1.1, 1.2], [1.3, 1.4, 1.5], [1.6, 1.7, 1.8]],
        dtype=np.float32,
    )

    aligned = project_depth_to_color(
        depth,
        CAMERA_K,
        CAMERA_K,
        (3, 3),
        np.eye(3),
        np.zeros(3),
    )

    np.testing.assert_array_equal(
        aligned,
        np.rint(depth * 1000.0).astype(np.uint16),
    )


def test_pixel_aligned_fast_path_preserves_valid_depth_only():
    depth = np.array(
        [[1.25, np.nan], [0.001, 70.0]],
        dtype=np.float32,
    )

    aligned = depth_meters_to_millimeters(depth)

    np.testing.assert_array_equal(
        aligned,
        np.array([[1250, 0], [0, 65535]], dtype=np.uint16),
    )


def test_pixel_alignment_requires_same_frame_size_and_intrinsics():
    assert streams_are_pixel_aligned(
        'camera_optical',
        'camera_optical',
        (2, 3),
        CAMERA_K,
        CAMERA_K,
        (3, 2),
    )
    assert not streams_are_pixel_aligned(
        'depth_optical',
        'color_optical',
        (2, 3),
        CAMERA_K,
        CAMERA_K,
        (3, 2),
    )
    assert not streams_are_pixel_aligned(
        'camera_optical',
        'camera_optical',
        (2, 2),
        CAMERA_K,
        CAMERA_K,
        (3, 2),
    )


def test_invalid_depth_samples_are_ignored():
    depth = np.array(
        [[np.nan, np.inf, 0.0], [-1.0, 1.5, 0.001]],
        dtype=np.float32,
    )

    aligned = project_depth_to_color(
        depth,
        [100.0, 0.0, 1.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
        [100.0, 0.0, 1.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
        (3, 2),
        np.eye(3),
        np.zeros(3),
    )

    assert np.count_nonzero(aligned) == 1
    assert aligned[1, 1] == 1500


def test_z_buffer_keeps_nearest_surface():
    depth = np.array([[1.0, 2.0]], dtype=np.float32)
    depth_k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    color_k = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]

    aligned = project_depth_to_color(
        depth,
        depth_k,
        color_k,
        (1, 1),
        np.eye(3),
        np.zeros(3),
    )

    assert aligned[0, 0] == 1000
