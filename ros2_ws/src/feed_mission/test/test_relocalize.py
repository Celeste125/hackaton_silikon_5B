import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from feed_mission.relocalize import MapMatcher, decide  # noqa: E402

RES = 0.05


def arena():
    """4 x 5 m L-shaped room with a 1 x 0.5 m box, like the arena and cow box."""
    g = np.zeros((100, 80), dtype=np.int16)   # rows = y, cols = x
    g[:, :2] = g[:, -2:] = g[:2, :] = g[-2:, :] = 100
    g[:30, 55:] = 100                          # a corner cut out: not symmetric
    g[60:70, 20:40] = 100
    return g


def fake_scan(grid, pose, n=360, max_r=6.0):
    """Raycast a scan from pose; returns points in the robot frame."""
    x, y, yaw = pose
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        for r in np.arange(0.2, max_r, RES / 2):
            c, row = int((x + r * math.cos(yaw + a)) / RES), int((y + r * math.sin(yaw + a)) / RES)
            if not (0 <= row < grid.shape[0] and 0 <= c < grid.shape[1]) or grid[row, c] >= 65:
                pts.append((r * math.cos(a), r * math.sin(a)))
                break
    return np.array(pts)


def test_finds_the_true_pose_and_accepts_it():
    g = arena()
    m = MapMatcher(g, RES, 0.0, 0.0)
    truth = (2.6, 1.3, math.radians(-15))
    cands = m.search(fake_scan(g, truth))
    pose, why = decide(cands)
    assert pose is not None, why
    assert math.hypot(pose[0] - truth[0], pose[1] - truth[1]) < 0.06
    assert abs(math.remainder(pose[2] - truth[2], 2 * math.pi)) < math.radians(3)


def test_symmetric_room_is_refused_as_ambiguous():
    g = np.zeros((60, 60), dtype=np.int16)  # empty square: 4 equally good headings
    g[:, :2] = g[:, -2:] = g[:2, :] = g[-2:, :] = 100
    m = MapMatcher(g, RES, 0.0, 0.0)
    pose, why = decide(m.search(fake_scan(g, (1.5, 1.5, 0.3))))
    assert pose is None and 'ambiguous' in why


def test_bad_fit_is_refused():
    assert decide([(0.4, (0, 0, 0)), (0.1, (2, 2, 0))])[0] is None
    assert decide([])[0] is None
