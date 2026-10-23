"""Conversion between the established JSON object dict and imdl_msgs."""

import math

from geometry_msgs.msg import Point, Point32
from imdl_msgs.msg import DetectedObject, DetectedObjectArray


def make_detected_object_array(payload, objects):
    """Convert one enriched-object frame into the shared typed interface."""
    message = DetectedObjectArray()
    _set_header(message.header, payload['timestamp'], payload['frame_id'])
    message.objects = [make_detected_object(obj) for obj in objects]
    return message


def make_detected_object(obj):
    """Convert a validated enriched-object dict without changing its meaning."""
    message = DetectedObject()
    _set_header(message.header, obj['timestamp'], obj['source_frame_id'])
    message.class_id = int(obj['class_id'])
    message.class_name = str(obj['class_name'])
    message.is_block = bool(obj['is_block'])
    message.confidence = float(obj['confidence'])

    center = obj['center_px']
    message.center_u_px = float(center['u'])
    message.center_v_px = float(center['v'])
    message.bbox_xyxy_px = [float(value) for value in obj['bbox_xyxy_px']]
    message.mask_polygon_px = [
        Point32(x=float(point[0]), y=float(point[1]), z=0.0)
        for point in obj['mask_polygon_px']
    ]

    depth_m = _finite_number(obj.get('depth_m'))
    message.depth_valid = depth_m is not None
    if depth_m is not None:
        message.depth_m = depth_m

    position_camera = _point_or_none(obj.get('position_camera'))
    position_base = _point_or_none(obj.get('position_base'))
    position_map = _point_or_none(obj.get('position_map'))
    if position_camera is not None:
        message.position_camera_m = position_camera
    if position_base is not None:
        message.position_base_m = position_base
    if position_map is not None:
        message.position_map_m = position_map
    message.base_pose_valid = position_base is not None
    message.map_pose_valid = position_map is not None

    polar = obj.get('polar_base') or {}
    range_m = _finite_number(polar.get('range_m'))
    bearing_rad = _finite_number(polar.get('bearing_rad'))
    if range_m is not None:
        message.range_m = range_m
    if bearing_rad is not None:
        message.bearing_rad = bearing_rad

    quality = obj.get('depth_quality') or {}
    message.mask_area_px = _nonnegative_int(obj.get('mask_area_px'))
    message.valid_depth_pixels = _nonnegative_int(quality.get('valid_depth_pixels'))
    valid_depth_ratio = _finite_number(quality.get('valid_depth_ratio'))
    depth_mad_m = _finite_number(quality.get('depth_mad_m'))
    if valid_depth_ratio is not None:
        message.valid_depth_ratio = valid_depth_ratio
    if depth_mad_m is not None:
        message.depth_mad_m = depth_mad_m
    message.is_stale = bool(obj.get('is_stale', False))
    message.transform_status = str(obj.get('transform_status', 'unknown'))
    return message


def _set_header(header, stamp, frame_id):
    header.stamp.sec = int(stamp['sec'])
    header.stamp.nanosec = int(stamp['nanosec'])
    header.frame_id = str(frame_id)


def _finite_number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _point_or_none(value):
    if not isinstance(value, dict):
        return None
    coordinates = [_finite_number(value.get(axis)) for axis in ('x', 'y', 'z')]
    if any(coordinate is None for coordinate in coordinates):
        return None
    return Point(x=coordinates[0], y=coordinates[1], z=coordinates[2])


def _nonnegative_int(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return 0
    return max(0, value)
