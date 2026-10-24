"""Render a marker at a known pose, run the detector maths, compare."""
import math

import cv2
import numpy as np

from imdl_statemachine.markers import marker_in_frame, marker_yaw

SIZE = 0.20
W, H, F = 640, 480, 500.0
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
OBJ = np.array([[-SIZE / 2, SIZE / 2, 0], [SIZE / 2, SIZE / 2, 0],
                [SIZE / 2, -SIZE / 2, 0], [-SIZE / 2, -SIZE / 2, 0]], np.float32)


def make_marker(dictionary, marker_id):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
    return cv2.aruco.drawMarker(dictionary, marker_id, 200)


def detect(gray, dictionary):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=cv2.aruco.DetectorParameters_create())


def test_marker_pose_recovered_in_robot_frame():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = make_marker(dictionary, 1)

    # Marker 1 m ahead of a forward-looking camera, rotated 30 deg about the vertical.
    yaw = math.radians(30)
    rot_true = cv2.Rodrigues(np.array([0.0, yaw, 0.0]))[0] @ cv2.Rodrigues(
        np.array([math.pi, 0.0, 0.0]))[0]          # face the camera (z toward it)
    rvec = cv2.Rodrigues(rot_true)[0]
    tvec = np.array([0.3, 0.0, 1.0])
    pts, _ = cv2.projectPoints(OBJ, rvec, tvec, K, None)
    pts = pts.reshape(4, 2).astype(np.float32)

    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    canvas = np.full((H, W), 255, np.uint8)
    cv2.warpPerspective(
        cv2.copyMakeBorder(marker, 0, 0, 0, 0, cv2.BORDER_CONSTANT), cv2.getPerspectiveTransform(src, pts),
        (W, H), canvas, borderMode=cv2.BORDER_TRANSPARENT)

    corners, ids, _ = detect(canvas, dictionary)
    assert ids is not None and int(np.ravel(ids)[0]) == 1
    ok, rv, tv = cv2.solvePnP(OBJ, corners[0].reshape(4, 2).astype(np.float32), K, np.zeros(5),
                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok
    assert np.allclose(tv.flatten(), tvec, atol=0.05)

    # Optical frame -> robot frame for a forward-facing camera (x fwd, y left, z up).
    optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    center, normal = marker_in_frame(cv2.Rodrigues(rv)[0], tv.flatten(), optical, [0, 0, 0])
    assert np.allclose(center, [1.0, -0.3, 0.0], atol=0.05)
    # Normal points back at the robot, tilted by the 30 deg rotation. Pose
    # orientation is noisy even at 1 m, so the tolerance is loose.
    heading = marker_yaw(normal)
    expected = math.pi - yaw
    diff = (heading - expected + math.pi) % (2 * math.pi) - math.pi
    assert abs(diff) < math.radians(20)


def test_head_on_marker_normal_faces_camera():
    from imdl_statemachine.markers import solve_marker_pose

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = make_marker(dictionary, 0)
    rvec = cv2.Rodrigues(np.diag([1.0, -1.0, -1.0]))[0]   # exactly frontal
    pts = cv2.projectPoints(OBJ, rvec, np.array([0.0, 0.0, 1.0]), K, None)[0].reshape(4, 2)
    canvas = np.full((H, W), 255, np.uint8)
    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    cv2.warpPerspective(marker, cv2.getPerspectiveTransform(src, pts.astype(np.float32)),
                        (W, H), canvas, borderMode=cv2.BORDER_TRANSPARENT)
    corners, ids, _ = detect(canvas, dictionary)
    assert ids is not None
    rv, tv = solve_marker_pose(OBJ, corners[0].reshape(4, 2).astype(np.float32), K, np.zeros(5))
    assert cv2.Rodrigues(rv)[0][2, 2] < 0   # normal points toward the camera


def test_axis_aligned_centred_square_is_solved():
    from imdl_statemachine.markers import solve_marker_pose

    # Exactly head-on and centred: OpenCV 4.5 IPPE alone returns a wrong pose here.
    pts = np.array([[239, 159], [401, 159], [401, 321], [239, 321]], np.float32)
    rv, tv = solve_marker_pose(OBJ, pts, K, np.zeros(5))
    assert cv2.Rodrigues(rv)[0][2, 2] < -0.9
    assert abs(tv.flatten()[2] - SIZE * F / 162) < 0.05
