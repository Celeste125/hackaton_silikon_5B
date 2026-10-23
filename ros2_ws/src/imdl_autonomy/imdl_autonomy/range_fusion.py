"""Pure geometry and plausibility checks for the front range sensor."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from .mission_spec import Pose2D


@dataclass(frozen=True)
class FrontRangeReading:
    """Median-filtered reading from one fixed forward-facing sensor."""

    range_m: float
    min_range_m: float
    max_range_m: float
    field_of_view_rad: float
    frame_id: str
    generation: int
    sample_count: int
    received_wall: float
    stamp_sec: Optional[float] = None


@dataclass(frozen=True)
class TargetGeometry:
    """Target position expressed in the planar robot frame."""

    distance_m: float
    bearing_rad: float
    forward_m: float
    lateral_m: float


@dataclass(frozen=True)
class FrontRangeEvaluation:
    """Result of comparing one range return with one identified target."""

    consistent: bool
    reason: str
    measured_range_m: Optional[float]
    expected_range_m: float
    error_m: Optional[float]
    target_distance_m: float
    target_bearing_rad: float
    sample_count: int


def target_geometry(robot_pose: Pose2D, target_pose: Pose2D) -> TargetGeometry:
    """Return distance and signed bearing from robot x-axis to a target."""
    dx = float(target_pose.x) - float(robot_pose.x)
    dy = float(target_pose.y) - float(robot_pose.y)
    cosine = math.cos(float(robot_pose.yaw))
    sine = math.sin(float(robot_pose.yaw))
    forward = cosine * dx + sine * dy
    lateral = -sine * dx + cosine * dy
    return TargetGeometry(
        distance_m=math.hypot(dx, dy),
        bearing_rad=math.atan2(lateral, forward),
        forward_m=forward,
        lateral_m=lateral,
    )


def expected_front_range(
    robot_pose: Pose2D,
    target_pose: Pose2D,
    sensor_offset_x_m: float,
    target_half_extent_m: float,
) -> tuple[float, TargetGeometry]:
    """Estimate distance from the sensor face to the target's near surface."""
    geometry = target_geometry(robot_pose, target_pose)
    expected = (
        geometry.forward_m
        - max(0.0, float(sensor_offset_x_m))
        - max(0.0, float(target_half_extent_m))
    )
    return expected, geometry


def evaluate_front_range(
    reading: Optional[FrontRangeReading],
    robot_pose: Pose2D,
    target_pose: Pose2D,
    sensor_offset_x_m: float,
    target_half_extent_m: float,
    tolerance_m: float,
    max_target_bearing_rad: float,
) -> FrontRangeEvaluation:
    """Check whether a narrow-cone return plausibly belongs to the target."""
    expected, geometry = expected_front_range(
        robot_pose,
        target_pose,
        sensor_offset_x_m,
        target_half_extent_m,
    )
    if reading is None:
        return FrontRangeEvaluation(
            consistent=False,
            reason='no fresh ultrasonic samples',
            measured_range_m=None,
            expected_range_m=expected,
            error_m=None,
            target_distance_m=geometry.distance_m,
            target_bearing_rad=geometry.bearing_rad,
            sample_count=0,
        )
    if abs(geometry.bearing_rad) > max(
        0.0,
        float(max_target_bearing_rad),
    ):
        return FrontRangeEvaluation(
            consistent=False,
            reason='identified target is outside the trusted range cone',
            measured_range_m=reading.range_m,
            expected_range_m=expected,
            error_m=reading.range_m - expected,
            target_distance_m=geometry.distance_m,
            target_bearing_rad=geometry.bearing_rad,
            sample_count=reading.sample_count,
        )
    if expected < reading.min_range_m or expected > reading.max_range_m:
        return FrontRangeEvaluation(
            consistent=False,
            reason='expected target surface is outside sensor limits',
            measured_range_m=reading.range_m,
            expected_range_m=expected,
            error_m=reading.range_m - expected,
            target_distance_m=geometry.distance_m,
            target_bearing_rad=geometry.bearing_rad,
            sample_count=reading.sample_count,
        )
    error = reading.range_m - expected
    consistent = abs(error) <= max(0.0, float(tolerance_m))
    return FrontRangeEvaluation(
        consistent=consistent,
        reason=(
            'range agrees with camera target'
            if consistent
            else 'range return does not agree with camera target'
        ),
        measured_range_m=reading.range_m,
        expected_range_m=expected,
        error_m=error,
        target_distance_m=geometry.distance_m,
        target_bearing_rad=geometry.bearing_rad,
        sample_count=reading.sample_count,
    )
