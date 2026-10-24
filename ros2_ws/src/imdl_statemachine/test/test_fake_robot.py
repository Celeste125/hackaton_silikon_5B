import math
import os
import tempfile

import cv2
import numpy as np

from imdl_statemachine.fake_robot import Grid, cast_scan


def make_map(tmp):
    img = np.full((100, 100), 254, np.uint8)       # free, 1 m x 1 m at 0.01 m/px
    img[:, 95:] = 0                                 # wall at x >= 0.95
    cv2.imwrite(os.path.join(tmp, 'm.pgm'), img)
    with open(os.path.join(tmp, 'm.yaml'), 'w') as f:
        f.write('image: m.pgm\nresolution: 0.01\norigin: [0.0, 0.0, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n')
    return os.path.join(tmp, 'm.yaml')


def test_beam_hits_wall_at_expected_range():
    with tempfile.TemporaryDirectory() as tmp:
        grid = Grid(make_map(tmp))
        ranges = cast_scan(grid, 0.45, 0.5, 0.0, np.array([0.0, math.pi / 2]), 5.0, 0.005)
        assert abs(ranges[0] - 0.5) < 0.02       # wall at 0.95, robot at 0.45
        assert ranges[1] < 0.6                   # top edge: map ends at y = 1.0


def test_outside_map_is_occupied():
    with tempfile.TemporaryDirectory() as tmp:
        assert Grid(make_map(tmp)).hit(np.array([-1.0]), np.array([0.5]))[0]


def test_rendered_markers_are_detected_by_aruco():
    import cv2 as cv
    from imdl_statemachine.fake_camera import marker_image, render_view
    from imdl_statemachine.fake_robot import cast_scan as cast

    with tempfile.TemporaryDirectory() as tmp:
        grid = Grid(make_map(tmp))
        # Robot at (0.3, 0.5) looking +x at a wall at x = 0.95; marker 1 on that wall, facing -x.
        markers = {1: [0.95, 0.5, math.pi]}
        img = render_view(grid, cast, (0.3, 0.5, 0.0), markers, {1: marker_image(1)},
                          cam_offset=(0.0, 0.0, 0.25))
        d = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        if hasattr(cv.aruco, 'ArucoDetector'):
            _, ids, _ = cv.aruco.ArucoDetector(d, cv.aruco.DetectorParameters()).detectMarkers(gray)
        else:
            _, ids, _ = cv.aruco.detectMarkers(gray, d, parameters=cv.aruco.DetectorParameters_create())
        assert ids is not None and 1 in np.ravel(ids)
        # Looking away from the wall: nothing to see.
        away = render_view(grid, cast, (0.3, 0.5, math.pi), markers, {1: marker_image(1)},
                           cam_offset=(0.0, 0.0, 0.25))
        assert away.shape == img.shape
