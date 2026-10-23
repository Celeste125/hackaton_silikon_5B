"""Pure helpers for RGB-D perception visualisation."""

import math

import cv2
import numpy as np

from perception.object_geometry_utils import depth_image_to_meters


# BGR colours are used by OpenCV debug frames. Blue obstacle is cyan so it is
# visually distinct from the blue graspable block.
CLASS_COLORS_BGR = {
    'red_block': (50, 50, 230),
    'yellow_block': (0, 220, 245),
    'green_block': (60, 205, 60),
    'blue_block': (230, 120, 45),
    'blue_obstacle': (220, 220, 0),
}


def class_color_bgr(class_name):
    """Return a stable OpenCV colour for a known perception class."""
    return CLASS_COLORS_BGR.get(str(class_name), (180, 180, 180))


def colorize_depth(depth_image, min_depth_m, max_depth_m):
    """Render depth in metres as a fixed-range colour image.

    Valid near pixels are warm and far pixels are cool. Invalid pixels remain
    black so they cannot be mistaken for a measured distance.
    """
    if min_depth_m <= 0.0 or max_depth_m <= min_depth_m:
        raise ValueError('depth range must be positive and ordered')

    depth_m = depth_image_to_meters(depth_image)
    valid = (
        np.isfinite(depth_m)
        & (depth_m >= float(min_depth_m))
        & (depth_m <= float(max_depth_m))
    )
    scaled = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        normalised = (float(max_depth_m) - depth_m[valid]) / (
            float(max_depth_m) - float(min_depth_m)
        )
        scaled[valid] = np.clip(np.rint(normalised * 255.0), 0, 255).astype(
            np.uint8
        )
    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored


def object_has_map_position(obj):
    """Return whether an object has a valid map-frame position for RViz."""
    if not isinstance(obj, dict) or not obj.get('map_pose_valid', False):
        return False
    position = obj.get('position_map')
    if not isinstance(position, dict):
        return False
    try:
        values = (float(position['x']), float(position['y']), float(position['z']))
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def object_depth_label(obj):
    """Create a compact, honest distance label for a detected object."""
    class_name = str(obj.get('class_name', 'unknown'))
    confidence = obj.get('confidence')
    try:
        confidence_text = f'{float(confidence):.2f}'
    except (TypeError, ValueError):
        confidence_text = 'n/a'

    depth = obj.get('depth_m')
    polar = obj.get('polar_base') if isinstance(obj.get('polar_base'), dict) else {}
    range_m = polar.get('range_m')
    try:
        if depth is not None and range_m is not None:
            return f'{class_name} {confidence_text}  Z:{float(depth):.2f}m R:{float(range_m):.2f}m'
    except (TypeError, ValueError):
        pass
    return f'{class_name} {confidence_text}  {obj.get("transform_status", "unavailable")}'
