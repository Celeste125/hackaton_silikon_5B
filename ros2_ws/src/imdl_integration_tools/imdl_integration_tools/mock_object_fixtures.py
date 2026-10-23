"""Deterministic typed-object frames used by B/C software integration."""

import copy

from geometry_msgs.msg import Point, Point32
from imdl_msgs.msg import DetectedObject, DetectedObjectArray


SCENARIOS = ('mixed', 'empty')
SOURCE_FRAME = 'camera_color_optical_frame'


def make_scenario(scenario, stamp, frame_id=SOURCE_FRAME):
    """Build one deterministic typed-object array for a named mock scenario."""
    if scenario not in SCENARIOS:
        raise ValueError(f'Unsupported mock scenario: {scenario}')

    message = DetectedObjectArray()
    message.header.stamp = copy.deepcopy(stamp)
    message.header.frame_id = frame_id
    if scenario == 'empty':
        return message

    message.objects = [
        _object(stamp, frame_id, 0, 'red_block', 0, True, True, True, True),
        _object(stamp, frame_id, 4, 'blue_obstacle', 1, False, True, True, True),
        _object(stamp, frame_id, 1, 'yellow_block', 2, True, True, True, False),
        _object(stamp, frame_id, 2, 'green_block', 3, True, False, True, True),
        _object(stamp, frame_id, 3, 'blue_block', 4, True, True, False, True),
        _object(stamp, frame_id, 0, 'red_block', 5, True, True, True, True, stale=True),
    ]
    return message


def _object(
    stamp,
    frame_id,
    class_id,
    class_name,
    index,
    is_block,
    depth_valid,
    base_valid,
    map_valid,
    stale=False,
):
    object_message = DetectedObject()
    object_message.header.stamp = copy.deepcopy(stamp)
    object_message.header.frame_id = frame_id
    object_message.class_id = class_id
    object_message.class_name = class_name
    object_message.is_block = is_block
    object_message.confidence = 0.9
    object_message.center_u_px = 100.0 + index
    object_message.center_v_px = 80.0 + index
    object_message.bbox_xyxy_px = [90.0 + index, 70.0, 110.0 + index, 90.0]
    object_message.mask_polygon_px = [
        Point32(x=90.0 + index, y=70.0, z=0.0),
        Point32(x=110.0 + index, y=70.0, z=0.0),
        Point32(x=110.0 + index, y=90.0, z=0.0),
        Point32(x=90.0 + index, y=90.0, z=0.0),
    ]
    object_message.depth_valid = depth_valid
    object_message.base_pose_valid = base_valid
    object_message.map_pose_valid = map_valid
    object_message.is_stale = stale
    object_message.mask_area_px = 400
    object_message.valid_depth_pixels = 400 if depth_valid else 0
    object_message.valid_depth_ratio = 1.0 if depth_valid else 0.0
    object_message.depth_mad_m = 0.0

    if depth_valid:
        object_message.depth_m = 0.6 + 0.1 * index
        object_message.position_camera_m = Point(
            x=0.05 * index, y=0.0, z=object_message.depth_m
        )
    if base_valid:
        object_message.position_base_m = Point(
            x=0.6 + 0.1 * index, y=0.05 * index, z=0.0
        )
        object_message.range_m = 0.6 + 0.1 * index
        object_message.bearing_rad = 0.05 * index
    if map_valid:
        object_message.position_map_m = Point(
            x=1.0 + 0.1 * index, y=2.0 + 0.1 * index, z=0.0
        )

    if stale:
        object_message.transform_status = 'stale_mock'
    elif depth_valid and base_valid and map_valid:
        object_message.transform_status = 'ok'
    elif not depth_valid:
        object_message.transform_status = 'depth_unavailable'
    elif not base_valid and not map_valid:
        object_message.transform_status = 'missing_base_and_map_tf'
    elif not base_valid:
        object_message.transform_status = 'missing_base_tf'
    else:
        object_message.transform_status = 'missing_map_tf'
    return object_message
