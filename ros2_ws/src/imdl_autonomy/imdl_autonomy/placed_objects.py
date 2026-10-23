"""Shared contract for completed object-placement exclusion zones."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Iterable, Optional, Sequence

import numpy as np


PLACED_OBJECT_EVENT_TOPIC = '/autonomy/placed_object_event'


@dataclass(frozen=True)
class PlacedObjectZone:
    """Small map-frame area containing an intentionally placed task object."""

    object_id: str
    color: str
    frame_id: str
    x: float
    y: float
    radius_m: float


def placement_event_json(
    *,
    object_id: str,
    color: str,
    frame_id: str,
    x: float,
    y: float,
    radius_m: float,
) -> str:
    """Encode one successful placement as a stable ROS String payload."""
    return json.dumps({
        'event': 'placed',
        'object_id': str(object_id),
        'color': str(color).strip().lower(),
        'frame_id': str(frame_id),
        'x': float(x),
        'y': float(y),
        'radius_m': max(0.0, float(radius_m)),
    }, sort_keys=True)


def parse_placement_event(
    payload: str,
    *,
    default_radius_m: float,
) -> Optional[PlacedObjectZone]:
    """Parse and validate one placement event, returning ``None`` if invalid."""
    try:
        value = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(value, dict) or value.get('event') != 'placed':
        return None
    try:
        x = float(value['x'])
        y = float(value['y'])
        radius = float(value.get('radius_m', default_radius_m))
    except (KeyError, TypeError, ValueError):
        return None
    frame_id = str(value.get('frame_id', '')).strip()
    object_id = str(value.get('object_id', '')).strip()
    color = str(value.get('color', '')).strip().lower()
    if (
        not frame_id
        or not object_id
        or not color
        or not math.isfinite(x)
        or not math.isfinite(y)
        or not math.isfinite(radius)
        or radius <= 0.0
    ):
        return None
    return PlacedObjectZone(
        object_id=object_id,
        color=color,
        frame_id=frame_id,
        x=x,
        y=y,
        radius_m=radius,
    )


class PlacedObjectZones:
    """Maintain idempotent placement zones received from an actuator adapter."""

    def __init__(self, frame_id: str, default_radius_m: float):
        self.frame_id = str(frame_id)
        self.default_radius_m = max(0.001, float(default_radius_m))
        self._zones = {}

    @property
    def zones(self) -> tuple[PlacedObjectZone, ...]:
        return tuple(self._zones.values())

    def update(self, payload: str) -> Optional[PlacedObjectZone]:
        zone = parse_placement_event(
            payload,
            default_radius_m=self.default_radius_m,
        )
        if zone is None or zone.frame_id != self.frame_id:
            return None
        self._zones[zone.object_id] = zone
        return zone

    def contains(self, x: float, y: float) -> bool:
        return any(
            math.hypot(float(x) - zone.x, float(y) - zone.y)
            <= zone.radius_m
            for zone in self._zones.values()
        )


def exclude_camera_points_in_zones(
    camera_points: np.ndarray,
    zone_frame_from_camera_rotation: np.ndarray,
    zone_frame_from_camera_translation: Sequence[float],
    zones: Iterable[PlacedObjectZone],
) -> np.ndarray:
    """Remove camera-frame points lying inside completed placement zones."""
    points = np.asarray(camera_points, dtype=np.float32)
    if points.size == 0:
        return points.reshape((-1, 3))
    active_zones = tuple(zones)
    if not active_zones:
        return points

    rotation = np.asarray(
        zone_frame_from_camera_rotation,
        dtype=np.float64,
    ).reshape(3, 3)
    translation = np.asarray(
        zone_frame_from_camera_translation,
        dtype=np.float64,
    ).reshape(3)
    zone_points = points.astype(np.float64) @ rotation.T + translation
    keep = np.ones(points.shape[0], dtype=bool)
    for zone in active_zones:
        distance_squared = (
            (zone_points[:, 0] - zone.x) ** 2
            + (zone_points[:, 1] - zone.y) ** 2
        )
        keep &= distance_squared > zone.radius_m * zone.radius_m
    return points[keep]
