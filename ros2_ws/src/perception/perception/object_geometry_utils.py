"""Pure helpers for mask-based RGB-D object geometry."""

import math

import cv2
import numpy as np


def stamp_to_nanoseconds(stamp):
    """Return a ROS-style stamp dictionary or message as nanoseconds."""
    if isinstance(stamp, dict):
        sec = stamp.get('sec', 0)
        nanosec = stamp.get('nanosec', 0)
    else:
        sec = stamp.sec
        nanosec = stamp.nanosec
    return int(sec) * 1_000_000_000 + int(nanosec)


def polygon_mask(polygon, source_width, source_height, output_width, output_height):
    """Rasterize a source-image polygon into an output-image binary mask."""
    points = np.asarray(polygon, dtype=np.float64)
    if points.ndim != 2 or points.shape != (len(points), 2) or len(points) < 3:
        raise ValueError('polygon must contain at least three 2D points')
    if not np.isfinite(points).all():
        raise ValueError('polygon coordinates must be finite')
    if source_width <= 0 or source_height <= 0:
        raise ValueError('source image dimensions must be positive')
    if output_width <= 0 or output_height <= 0:
        raise ValueError('output image dimensions must be positive')

    scaled = points.copy()
    scaled[:, 0] *= float(output_width) / float(source_width)
    scaled[:, 1] *= float(output_height) / float(source_height)
    scaled[:, 0] = np.clip(np.rint(scaled[:, 0]), 0, output_width - 1)
    scaled[:, 1] = np.clip(np.rint(scaled[:, 1]), 0, output_height - 1)

    mask = np.zeros((output_height, output_width), dtype=np.uint8)
    cv2.fillPoly(mask, [scaled.astype(np.int32)], 255)
    return mask.astype(bool)


def depth_image_to_meters(depth_image):
    """Convert RealSense Z16 depth or float depth images to float metres."""
    depth = np.asarray(depth_image)
    if depth.ndim != 2:
        raise ValueError('depth image must be single-channel')
    values = depth.astype(np.float32, copy=False)
    if depth.dtype == np.uint16:
        values = values / 1000.0
    return values


def masked_depth_median(
    depth_image,
    polygon,
    source_width,
    source_height,
    min_depth_m,
    max_depth_m,
    outlier_threshold_m,
    min_valid_pixels,
):
    """Return a robust depth median from valid pixels inside a segmentation mask."""
    if min_depth_m <= 0.0 or max_depth_m <= min_depth_m:
        raise ValueError('depth range must be positive and ordered')
    if outlier_threshold_m <= 0.0:
        raise ValueError('outlier_threshold_m must be positive')
    if min_valid_pixels < 1:
        raise ValueError('min_valid_pixels must be at least one')

    depth_m = depth_image_to_meters(depth_image)
    height, width = depth_m.shape
    mask = polygon_mask(polygon, source_width, source_height, width, height)
    candidates = depth_m[mask]
    candidates = candidates[
        np.isfinite(candidates)
        & (candidates >= min_depth_m)
        & (candidates <= max_depth_m)
    ]
    if candidates.size < min_valid_pixels:
        return None

    initial_median = float(np.median(candidates))
    inliers = candidates[np.abs(candidates - initial_median) <= outlier_threshold_m]
    if inliers.size < min_valid_pixels:
        return None
    return float(np.median(inliers))


def masked_depth_statistics(
    depth_image,
    polygon,
    source_width,
    source_height,
    min_depth_m,
    max_depth_m,
    outlier_threshold_m,
    min_mask_area_px,
    min_valid_pixels,
    min_valid_ratio,
    max_depth_mad_m,
):
    """Measure mask depth quality and return a robust depth estimate when valid."""
    if min_mask_area_px < 1 or min_valid_pixels < 1:
        raise ValueError('minimum mask and depth pixel counts must be positive')
    if not 0.0 <= min_valid_ratio <= 1.0:
        raise ValueError('min_valid_ratio must be within [0, 1]')
    if max_depth_mad_m <= 0.0:
        raise ValueError('max_depth_mad_m must be positive')

    depth_m = depth_image_to_meters(depth_image)
    height, width = depth_m.shape
    mask = polygon_mask(polygon, source_width, source_height, width, height)
    mask_area_px = int(np.count_nonzero(mask))
    result = {
        'mask_area_px': mask_area_px,
        'valid_depth_pixels': 0,
        'valid_depth_ratio': 0.0,
        'depth_mad_m': None,
        'depth_m': None,
        'status': 'mask_too_small',
    }
    if mask_area_px < min_mask_area_px:
        return result

    candidates = depth_m[mask]
    candidates = candidates[
        np.isfinite(candidates)
        & (candidates >= min_depth_m)
        & (candidates <= max_depth_m)
    ]
    result['valid_depth_pixels'] = int(candidates.size)
    result['valid_depth_ratio'] = float(candidates.size / mask_area_px)
    if candidates.size < min_valid_pixels:
        result['status'] = 'insufficient_valid_depth'
        return result
    if result['valid_depth_ratio'] < min_valid_ratio:
        result['status'] = 'low_valid_depth_ratio'
        return result

    initial_median = float(np.median(candidates))
    inliers = candidates[np.abs(candidates - initial_median) <= outlier_threshold_m]
    if inliers.size < min_valid_pixels:
        result['status'] = 'insufficient_depth_inliers'
        return result
    depth_median = float(np.median(inliers))
    depth_mad_m = float(np.median(np.abs(inliers - depth_median)))
    result['depth_mad_m'] = depth_mad_m
    if depth_mad_m > max_depth_mad_m:
        result['status'] = 'depth_unstable'
        return result

    result['depth_m'] = depth_median
    result['status'] = 'ok'
    return result


def rectangle_corners_from_polygon(
    polygon,
    min_mask_area_px,
    min_rect_fill_ratio,
):
    """Fit a minimum-area rectangle to a mask contour for 2D grasp guidance."""
    points = np.asarray(polygon, dtype=np.float32)
    result = {
        'mask_area_px': 0.0,
        'rect_fill_ratio': None,
        'orientation_rad': None,
        'rotated_bbox_px': None,
        'corners_px': None,
        'corner_status': 'invalid_polygon',
    }
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        return result
    if not np.isfinite(points).all():
        return result

    contour = points.reshape((-1, 1, 2))
    mask_area_px = abs(float(cv2.contourArea(contour)))
    result['mask_area_px'] = mask_area_px
    if mask_area_px < min_mask_area_px:
        result['corner_status'] = 'mask_too_small'
        return result

    rect = cv2.minAreaRect(contour)
    width, height = rect[1]
    rect_area = float(width * height)
    if rect_area <= 0.0:
        result['corner_status'] = 'degenerate_rectangle'
        return result
    fill_ratio = mask_area_px / rect_area
    result['rect_fill_ratio'] = fill_ratio
    if fill_ratio < min_rect_fill_ratio:
        result['corner_status'] = 'low_rect_fill_ratio'
        return result

    angle_deg = float(rect[2])
    if width < height:
        angle_deg += 90.0
    corners = cv2.boxPoints(rect).astype(np.float64).tolist()
    result.update({
        'orientation_rad': math.radians(angle_deg),
        'rotated_bbox_px': corners,
        'corners_px': corners,
        'corner_status': 'ok',
    })
    return result


def pixel_to_camera_point(u, v, depth_m, camera_matrix):
    """Back-project one RGB pixel into the aligned colour optical frame."""
    if not all(math.isfinite(float(value)) for value in (u, v, depth_m)):
        return None
    fx, fy, cx, cy = (
        float(camera_matrix[0]),
        float(camera_matrix[4]),
        float(camera_matrix[2]),
        float(camera_matrix[5]),
    )
    if fx <= 0.0 or fy <= 0.0 or depth_m <= 0.0:
        return None
    return {
        'x': (float(u) - cx) * float(depth_m) / fx,
        'y': (float(v) - cy) * float(depth_m) / fy,
        'z': float(depth_m),
    }


def polar_range_and_bearing(point):
    """Return the planar base-frame range and bearing for a point dictionary."""
    if point is None:
        return None, None
    x = float(point['x'])
    y = float(point['y'])
    if not math.isfinite(x) or not math.isfinite(y):
        return None, None
    return math.hypot(x, y), math.atan2(y, x)
