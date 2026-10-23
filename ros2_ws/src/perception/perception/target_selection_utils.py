"""Pure target-selection helpers for the IMDL object interface."""

import math


def parse_class_sequence(value):
    """Parse a comma-separated class sequence while preserving order."""
    if not value:
        return []
    return [item.strip() for item in value.split(',') if item.strip()]


def is_selectable_block(item, requested_class, min_confidence):
    """Return whether an enriched object is eligible to become a target."""
    if not isinstance(item, dict):
        return False
    if not item.get('is_block', False):
        return False
    if item.get('class_name') != requested_class:
        return False
    if item.get('is_stale', False):
        return False
    try:
        confidence = float(item.get('confidence', -1.0))
    except (TypeError, ValueError):
        return False
    if not math.isfinite(confidence) or confidence < min_confidence:
        return False
    if not item.get('map_pose_valid', False):
        return False
    position = item.get('position_map')
    if not isinstance(position, dict):
        return False
    try:
        return all(
            math.isfinite(float(position.get(axis, float('nan'))))
            for axis in ('x', 'y', 'z')
        )
    except (TypeError, ValueError):
        return False


def select_best_target(objects, requested_class, min_confidence, last_position=None):
    """Select one object by confidence, range, then map-position continuity."""
    candidates = [
        item for item in objects
        if is_selectable_block(item, requested_class, min_confidence)
    ]
    if not candidates:
        return None

    def sort_key(item):
        polar = item.get('polar_base', {})
        range_m = float(polar.get('range_m', float('inf')))
        if not math.isfinite(range_m):
            range_m = float('inf')
        continuity_m = 0.0
        position = item.get('position_map')
        if last_position is not None:
            continuity_m = math.hypot(
                float(position['x']) - float(last_position['x']),
                float(position['y']) - float(last_position['y']),
            )
        return (-float(item['confidence']), range_m, continuity_m)

    return min(candidates, key=sort_key)
