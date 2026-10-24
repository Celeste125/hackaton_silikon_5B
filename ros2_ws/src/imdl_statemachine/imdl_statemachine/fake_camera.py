"""Draw what a forward camera would see on a saved map, with ArUco markers on the walls.

Pure numpy/OpenCV, no ROS. Walls come from the occupancy grid, markers from
``sim_markers`` ([x, y, yaw of the outward normal] in the map frame, centred
at ``marker_z`` on a wall). A marker is only drawn if nothing blocks the view.
"""

import math

import cv2
import numpy as np

MODULE_PX = 40          # pixels per marker module in the source image
MODULES = 6             # 4x4 dictionary: 4 bits + black border on both sides


def marker_image(marker_id, dictionary_name='DICT_4X4_50'):
    """Marker with a one-module white margin, as printed: (MODULES + 2) modules wide."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    side = MODULES * MODULE_PX
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, side)
    return cv2.copyMakeBorder(img, MODULE_PX, MODULE_PX, MODULE_PX, MODULE_PX,
                              cv2.BORDER_CONSTANT, value=255)


def intrinsics(width, height, hfov_rad):
    f = (width / 2.0) / math.tan(hfov_rad / 2.0)
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])


def render_view(grid, cast_scan, pose, markers, images, cam_offset=(0.1, 0.0, 0.25),
                marker_size_m=0.20, size=(640, 480), hfov_rad=math.radians(60.0),
                wall_height_m=0.6, max_range=10.0):
    """BGR image seen from robot pose (x, y, yaw) in the map frame."""
    width, height = size
    k = intrinsics(width, height, hfov_rad)
    f, cx, cy = k[0, 0], k[0, 2], k[1, 2]
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    cam = np.array([x + c * cam_offset[0] - s * cam_offset[1],
                    y + s * cam_offset[0] + c * cam_offset[1], cam_offset[2]])

    img = np.zeros((height, width, 3), np.uint8)
    img[:int(cy)] = (205, 200, 195)     # background above the horizon
    img[int(cy):] = (85, 90, 95)        # floor

    # Walls: one vertical strip per column, from the map ray in that direction.
    cols = np.arange(width)
    bearings = np.arctan2(cx - (cols + 0.5), f)               # left of heading is positive
    dists = cast_scan(grid, cam[0], cam[1], yaw, bearings, max_range, grid.res * 0.5)
    for u in cols[dists <= max_range]:
        z = dists[u] * math.cos(bearings[u])
        top = max(0, int(cy - f * (wall_height_m - cam[2]) / z))
        bottom = min(height, int(cy + f * cam[2] / z))
        shade = int(max(60, 190 - 25 * dists[u]))
        img[top:bottom, u] = (shade, shade + 5, shade + 10)

    # Markers, far to near.
    half = marker_size_m * (MODULES + 2) / MODULES / 2.0      # incl. white margin
    forward = np.array([c, s, 0.0])
    left = np.array([-s, c, 0.0])
    order = sorted(markers.items(), key=lambda kv: -math.hypot(kv[1][0] - cam[0], kv[1][1] - cam[1]))
    for marker_id, (mx, my, myaw, *rest) in order:
        mz = rest[0] if rest else cam_offset[2]
        centre = np.array([mx, my, mz])
        normal = np.array([math.cos(myaw), math.sin(myaw), 0.0])
        if np.dot(normal, cam - centre) <= 0.0:
            continue                                           # seen from behind
        to_marker = centre - cam
        planar = math.hypot(to_marker[0], to_marker[1])
        ray = cast_scan(grid, cam[0], cam[1], 0.0,
                        np.array([math.atan2(to_marker[1], to_marker[0])]), max_range, grid.res * 0.5)
        if ray[0] < planar - 0.1:
            continue                                           # a wall is in the way
        right = np.array([-math.sin(myaw), math.cos(myaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        world = [centre - right * half + up * half, centre + right * half + up * half,
                 centre + right * half - up * half, centre - right * half - up * half]
        pts = []
        for p in world:
            rel = p - cam
            z = float(np.dot(rel, forward))
            if z < 0.1:
                break
            pts.append((cx + f * float(np.dot(rel, -left)) / z, cy + f * float(-rel[2]) / z))
        if len(pts) < 4:
            continue
        src_img = images[marker_id]
        h, w = src_img.shape
        src = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], np.float32)
        warp = cv2.getPerspectiveTransform(src, np.array(pts, np.float32))
        layer = cv2.warpPerspective(src_img, warp, (width, height), flags=cv2.INTER_AREA)
        mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), warp, (width, height))
        inside = mask > 127
        img[inside] = layer[inside][:, None]
    return img
