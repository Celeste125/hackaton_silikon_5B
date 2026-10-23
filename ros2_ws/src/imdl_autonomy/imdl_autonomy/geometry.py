"""Small, deterministic geometry helpers shared by the autonomy components."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

from .mission_spec import Pose2D


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shortest_angular_distance(source: float, target: float) -> float:
    return normalize_angle(target - source)


def path_length(points: Iterable[Pose2D]) -> float:
    total = 0.0
    previous = None
    for point in points:
        if previous is not None:
            total += previous.distance_to(point)
        previous = point
    return total


def covariance_ellipse(
    covariance: Sequence[float],
    confidence_scale: float = 2.4477,
) -> Tuple[float, float, float]:
    """Return major diameter, minor diameter and yaw for a 2-D covariance."""
    if len(covariance) == 36:
        xx = float(covariance[0])
        xy = float(covariance[1])
        yy = float(covariance[7])
    elif len(covariance) == 4:
        xx, xy, _, yy = (float(value) for value in covariance)
    else:
        raise ValueError('covariance must contain 4 or 36 elements')

    trace = xx + yy
    determinant = max(0.0, xx * yy - xy * xy)
    root = math.sqrt(max(0.0, trace * trace * 0.25 - determinant))
    major_variance = max(0.0, trace * 0.5 + root)
    minor_variance = max(0.0, trace * 0.5 - root)
    yaw = 0.5 * math.atan2(2.0 * xy, xx - yy)
    return (
        2.0 * confidence_scale * math.sqrt(major_variance),
        2.0 * confidence_scale * math.sqrt(minor_variance),
        yaw,
    )
