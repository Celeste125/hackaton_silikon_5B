"""Pure geometry for the ArUco marker route (no ROS, so it is easy to test)."""

import math

import cv2
import numpy as np


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def marker_in_frame(rot_cam, center_cam, rot_frame_cam, t_frame_cam):
    """Marker centre and outward normal in another frame.

    rot_cam: 3x3 marker rotation in the camera frame (z axis = outward normal).
    rot_frame_cam, t_frame_cam: transform camera frame -> target frame.
    Returns (center, normal) as numpy 3-vectors.
    """
    center = rot_frame_cam @ np.asarray(center_cam, float) + np.asarray(t_frame_cam, float)
    normal = rot_frame_cam @ np.asarray(rot_cam, float)[:, 2]
    return center, normal


def marker_yaw(normal):
    """Heading of the (horizontal part of the) outward normal, or None if it points up/down."""
    if math.hypot(normal[0], normal[1]) < 0.3:
        return None
    return math.atan2(normal[1], normal[0])


def standoff_goal(x, y, normal_yaw, standoff_m):
    """Pose [x, y, yaw] in front of a marker, facing it."""
    gx = x + standoff_m * math.cos(normal_yaw)
    gy = y + standoff_m * math.sin(normal_yaw)
    yaw = math.atan2(-math.sin(normal_yaw), -math.cos(normal_yaw))
    return [gx, gy, yaw]


def _reprojection_rms(obj_points, image_points, rvec, tvec, k, dist):
    projected = cv2.projectPoints(obj_points, rvec, tvec, k, dist)[0].reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((projected - image_points.reshape(-1, 2)) ** 2, axis=1))))


def solve_marker_pose(obj_points, image_points, k, dist):
    """Marker pose (rvec, tvec) in the camera frame, or None.

    Candidates come from the planar solver (IPPE, which returns both flip
    solutions) and from an iterative fit, because OpenCV 4.5's IPPE returns a
    garbage pose for a square seen almost exactly head-on. Only solutions that
    face the camera are physically possible; of those the lowest reprojection
    error wins.
    """
    candidates = []
    for flags in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj_points, image_points, k, dist, flags=flags)
        candidates += list(zip(rvecs, tvecs))
    best = None
    for rvec, tvec in candidates:
        if not (np.isfinite(rvec).all() and np.isfinite(tvec).all()):
            continue
        if cv2.Rodrigues(rvec)[0][2, 2] >= 0.0 or tvec.flatten()[2] <= 0.0:
            continue                    # marker z toward the camera has z_cam < 0
        err = _reprojection_rms(obj_points, image_points, rvec, tvec, k, dist)
        if best is None or err < best[0]:
            best = (err, rvec, tvec)
    return None if best is None else (best[1], best[2])
