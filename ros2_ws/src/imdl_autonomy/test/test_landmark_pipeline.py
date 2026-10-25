"""End-to-end check of landmark localisation, minus ROS.

A landmark is placed at a known spot in the map, the camera is driven to
several poses, and at each one the pixel it would project to is computed,
wrapped in the payload the detector publishes, parsed as the mapper parses it,
turned back into a map-frame sight line, and fused.

The point is the round trip. Every individual piece can be right while the
optical-frame convention is wrong by a sign, and that failure is invisible in
isolation: it produces landmarks in confidently wrong places, which on the
robot looks like a TF or a calibration problem and is neither.
"""

import json
import math

import numpy as np
import pytest

from imdl_autonomy.landmark_estimation import View, solve_landmark
from perception.landmark_detection import (
    LandmarkDetection,
    build_payload,
    parse_payload,
    resolve_intrinsics,
)

WIDTH, HEIGHT = 640, 480
FX = FY = 600.0
INTRINSICS = resolve_intrinsics(
    WIDTH, HEIGHT, [FX, 0, 320.0, 0, FY, 240.0, 0, 0, 1], 70.4)

LANDMARK = (2.5, 1.0)
BEARING_SIGMA = math.radians(3.0)


def _optical_from_base(point):
    """base_link (x fwd, y left, z up) -> optical (x right, y down, z fwd).

    The same convention webcam.launch.py publishes as a static transform,
    written out here so the test does not depend on TF.
    """
    x, y, z = point
    return (-y, -z, x)


def _base_from_optical(point):
    x, y, z = point
    return (z, -x, -y)


def _project(robot, landmark=LANDMARK):
    """The pixel a camera at `robot` would see `landmark` at, or None."""
    rx, ry, yaw = robot
    dx, dy = landmark[0] - rx, landmark[1] - ry
    cos, sin = math.cos(-yaw), math.sin(-yaw)
    in_base = (cos * dx - sin * dy, sin * dx + cos * dy, 0.0)
    ox, oy, oz = _optical_from_base(in_base)
    if oz <= 0.0:
        return None  # behind the camera
    u = INTRINSICS.cx + FX * ox / oz
    v = INTRINSICS.cy + FY * oy / oz
    if not (0.0 <= u < WIDTH and 0.0 <= v < HEIGHT):
        return None  # outside the frame
    return u, v


def _payload_at(robot, *, range_m=None, box_half=0.05, confidence=0.9):
    """What the detector on the Jetson would publish from this pose."""
    pixel = _project(robot)
    if pixel is None:
        return None
    u, v = pixel
    box = (
        max(u / WIDTH - box_half, 0.0), max(v / HEIGHT - box_half, 0.0),
        min(u / WIDTH + box_half, 1.0), min(v / HEIGHT + box_half, 1.0))
    detection = LandmarkDetection(
        landmark='food_dispenser', model_class='food_dispenser',
        confidence=confidence, box_norm=box,
        range_m=range_m,
        range_sigma_m=None if range_m is None else 0.05,
        range_source='none' if range_m is None else 'depth')
    return json.dumps(build_payload(
        stamp_sec=1, stamp_nanosec=0, frame_id='webcam_optical_frame',
        width=WIDTH, height=HEIGHT, intrinsics=INTRINSICS,
        detections=[detection], model={'backend': 'stub'}))


def _view_from_payload(text, robot):
    """What the mapper does: parse, deproject, transform, flatten.

    This mirrors LandmarkMapper._sight_line, including its two-point trick --
    transform a point on the ray rather than rotating the direction by hand --
    so the convention is exercised rather than assumed.
    """
    frame = parse_payload(text)
    detection = frame.detections[0]
    u, v = detection.centre_px(frame.width, frame.height)
    bearing = frame.intrinsics.bearing(u, v)

    rx, ry, yaw = robot
    cos, sin = math.cos(yaw), math.sin(yaw)
    points = []
    for scale in (0.0, 1.0):
        bx, by, _ = _base_from_optical(tuple(c * scale for c in bearing))
        points.append(np.array([rx + cos * bx - sin * by,
                                ry + sin * bx + cos * by]))
    origin, along = points
    flat = along - origin
    return View(
        origin=(float(origin[0]), float(origin[1])),
        direction=tuple(flat / np.linalg.norm(flat)),
        bearing_sigma_rad=BEARING_SIGMA,
        range_m=detection.range_m,
        range_sigma_m=detection.range_sigma_m,
        source='bearing' if detection.range_m is None else 'depth',
        confidence=detection.confidence,
    )


def _looking_at(x, y, landmark=LANDMARK):
    """A pose at (x, y) with the camera pointed at the landmark."""
    return (x, y, math.atan2(landmark[1] - y, landmark[0] - x))


# --------------------------------------------------------------------------


def test_the_projection_and_the_deprojection_agree():
    """A pose looking straight at the landmark must see it dead centre."""
    u, v = _project(_looking_at(0.0, 0.0))
    assert (u, v) == pytest.approx((INTRINSICS.cx, INTRINSICS.cy), abs=1e-6)


def test_a_landmark_to_the_left_appears_left_of_centre():
    """The sign that a mirrored optical convention would get wrong."""
    # Robot at the origin facing +x; the landmark is at +y, i.e. to its left.
    u, _ = _project((0.0, 0.0, 0.0))
    assert u < INTRINSICS.cx


def test_one_rgbd_view_recovers_the_landmark():
    robot = _looking_at(0.5, 0.0)
    distance = math.dist(robot[:2], LANDMARK)
    view = _view_from_payload(_payload_at(robot, range_m=distance), robot)
    estimate = solve_landmark('food_dispenser', [view])
    assert (estimate.x, estimate.y) == pytest.approx(LANDMARK, abs=0.01)
    assert estimate.converged


def test_two_webcam_views_from_different_angles_recover_the_landmark():
    """No depth anywhere: the baseline alone does it."""
    poses = [_looking_at(0.5, 1.0), _looking_at(2.5, -1.0)]
    views = [_view_from_payload(_payload_at(p), p) for p in poses]
    estimate = solve_landmark('food_dispenser', views)
    assert (estimate.x, estimate.y) == pytest.approx(LANDMARK, abs=0.02)
    assert estimate.converged


def test_the_estimate_improves_as_the_robot_drives_around_it():
    """The behaviour the whole feature is for, through the real wire format."""
    poses = [
        _looking_at(2.5 + 1.8 * math.cos(a), 1.0 + 1.8 * math.sin(a))
        for a in np.linspace(math.radians(200), math.radians(340), 8)
    ]
    views = [_view_from_payload(_payload_at(p), p) for p in poses]

    history = []
    for count in (2, 4, 8):
        estimate = solve_landmark('food_dispenser', views[:count])
        history.append((
            math.dist((estimate.x, estimate.y), LANDMARK),
            estimate.sigma_major_m))

    errors = [error for error, _ in history]
    sigmas = [sigma for _, sigma in history]
    assert sigmas[0] > sigmas[1] > sigmas[2], sigmas
    assert errors[-1] < 0.05
    # And the reported uncertainty is not a fiction: the true error sits
    # inside it at every stage.
    assert all(error <= sigma for error, sigma in history), history


def test_a_detection_the_robot_is_driving_past_still_contributes():
    """Off-centre boxes are the common case while exploring, not an edge case."""
    poses = [_looking_at(0.5, 1.0), _looking_at(2.5, -1.0)]
    views = []
    for pose in poses:
        # Turn 10 degrees away, so the landmark sits well off the optical axis.
        skewed = (pose[0], pose[1], pose[2] + math.radians(10.0))
        payload = _payload_at(skewed)
        assert payload is not None, 'should still be inside a 70 deg frame'
        views.append(_view_from_payload(payload, skewed))
    estimate = solve_landmark('food_dispenser', views)
    assert (estimate.x, estimate.y) == pytest.approx(LANDMARK, abs=0.03)


def test_bearing_noise_averages_out_but_an_extrinsic_bias_does_not():
    """Why the launch files say to measure the camera mounting.

    A constant yaw error in the camera extrinsics shifts every bearing the
    same way, so more views cannot cancel it -- and, worse, the fusion
    reports a small covariance while being wrong. Random box noise is the
    opposite. Worth a test so nobody expects averaging to fix a mounting.
    """
    poses = [
        _looking_at(2.5 + 1.8 * math.cos(a), 1.0 + 1.8 * math.sin(a))
        for a in np.linspace(math.radians(200), math.radians(340), 10)
    ]

    rng = np.random.default_rng(7)
    noisy = []
    biased = []
    for pose in poses:
        payload = _payload_at(pose)
        jitter = float(rng.normal(0.0, math.radians(1.5)))
        noisy.append(_view_from_payload(
            payload, (pose[0], pose[1], pose[2] + jitter)))
        biased.append(_view_from_payload(
            payload, (pose[0], pose[1], pose[2] + math.radians(3.0))))

    noisy_estimate = solve_landmark('food_dispenser', noisy)
    biased_estimate = solve_landmark('food_dispenser', biased)
    noisy_error = math.dist((noisy_estimate.x, noisy_estimate.y), LANDMARK)
    biased_error = math.dist((biased_estimate.x, biased_estimate.y), LANDMARK)

    assert noisy_error < 0.05
    assert biased_error > 0.05
    assert biased_error > noisy_error


def test_a_view_from_the_other_side_of_the_arena_is_not_fabricated():
    """A landmark behind the camera must produce no detection at all."""
    assert _payload_at((2.5, 1.0, math.pi)) is None
