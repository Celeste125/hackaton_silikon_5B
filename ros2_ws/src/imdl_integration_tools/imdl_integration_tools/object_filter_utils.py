"""Shared, side-effect-free predicates for typed object consumers."""

import copy

from imdl_msgs.msg import DetectedObjectArray


def is_navigation_candidate(object_message):
    """Navigation needs a fresh object position in the map frame."""
    return object_message.map_pose_valid and not object_message.is_stale


def is_control_candidate(object_message):
    """Control needs a fresh, graspable block in the robot base frame."""
    return (
        object_message.is_block
        and object_message.depth_valid
        and object_message.base_pose_valid
        and not object_message.is_stale
    )


def filtered_array(message, predicate):
    """Return a typed array with its source header and matching objects intact."""
    result = DetectedObjectArray()
    result.header = copy.deepcopy(message.header)
    result.objects = [
        copy.deepcopy(object_message)
        for object_message in message.objects
        if predicate(object_message)
    ]
    return result
