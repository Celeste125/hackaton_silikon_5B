"""The feeding lap, worked out from the cow box instead of taught by hand.

The box is a rectangle on the map (centre, heading of its long axis, length,
width). The lap is a rectangle around it, driven clockwise so the box is on
the robot's right, where the door is:

            corner 0 -----side 0 (long face)-----> corner 1
               ^          +-----------------+         |
     side 3    |          |     cow box     |         |  side 1
    (short)    |          +-----------------+         v  (short)
            corner 3 <----side 2 (long face)------ corner 2

The robot drives each side `gap` from the box (its right side to the box
face) on the faces it feeds, `end_gap` on the others, and turns on the spot
at the corners. That puts each corner past the end of the box, so the robot's
own corners (0.31 m from its centre) clear the box when it turns.

The door opens only while the door itself is beside a fed face, `margin`
inside both ends of it, and closes before the robot drives past the box.
All lengths in metres, angles in radians, map frame.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt, label

HALF_LENGTH = 0.17   # robot footprint, as in nav2_params local_costmap
HALF_WIDTH = 0.26
CORNER_REACH = math.hypot(HALF_LENGTH, HALF_WIDTH)
WALL = 65


@dataclass
class Box:
    cx: float
    cy: float
    yaw: float      # direction of the long axis
    length: float
    width: float

    def axes(self):
        u = (math.cos(self.yaw), math.sin(self.yaw))
        return u, (-u[1], u[0])

    def to_map(self, x, y):
        """Box frame (along length, along width) -> map."""
        (ux, uy), (vx, vy) = self.axes()
        return self.cx + x * ux + y * vx, self.cy + x * uy + y * vy

    def to_box(self, x, y):
        (ux, uy), (vx, vy) = self.axes()
        dx, dy = x - self.cx, y - self.cy
        return dx * ux + dy * uy, dx * vx + dy * vy

    def corners(self):
        a, b = self.length / 2, self.width / 2
        return [self.to_map(x, y) for x, y in ((-a, b), (a, b), (a, -b), (-a, -b))]

    def as_dict(self):
        return {'cx': self.cx, 'cy': self.cy, 'yaw': self.yaw,
                'length': self.length, 'width': self.width}

    @classmethod
    def from_dict(cls, d):
        return cls(float(d['cx']), float(d['cy']), float(d['yaw']),
                   float(d['length']), float(d['width']))


def normalized(box):
    """The same rectangle with length >= width and yaw in (-pi/2, pi/2]."""
    yaw, length, width = box.yaw, box.length, box.width
    if width > length:
        yaw, length, width = yaw + math.pi / 2, width, length
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    if yaw <= -math.pi / 2:
        yaw += math.pi
    elif yaw > math.pi / 2:
        yaw -= math.pi
    return Box(box.cx, box.cy, yaw, length, width)


FEED_FACES = ('long', 'all', 'none')


def plan_lap(box, gap=0.10, end_gap=0.15, feed='long', margin=0.05, door_x=0.0):
    """Corners and sides of one lap. See the module docstring.

    Returns {'corners': [(x, y)] * 4, 'sides': [...], 'warnings': [...]}; each
    side is {'start', 'end' (corner indices), 'face': 'long'|'short', 'feed',
    'length', 'door': (from, to) metres along the side for the robot's
    centre, or None}.
    """
    if feed not in FEED_FACES:
        raise ValueError(f'feed must be one of {", ".join(FEED_FACES)}, not "{feed}"')
    a, b = box.length / 2, box.width / 2
    feed_long = feed in ('long', 'all')
    feed_short = feed == 'all'
    o_long = HALF_WIDTH + (gap if feed_long else end_gap)    # centre to a long face
    o_short = HALF_WIDTH + (gap if feed_short else end_gap)  # centre to a short face
    X, Y = a + o_short, b + o_long
    corners = [box.to_map(x, y) for x, y in ((-X, Y), (X, Y), (X, -Y), (-X, -Y))]
    sides = []
    for i in range(4):
        long_face = i % 2 == 0
        length = 2 * X if long_face else 2 * Y
        fed = feed_long if long_face else feed_short
        # The face runs from (offset of the other faces) to (that + face length)
        face_from = o_short if long_face else o_long
        face_len = 2 * a if long_face else 2 * b
        door = None
        if fed:
            lo = face_from + margin - door_x
            hi = face_from + face_len - margin - door_x
            if hi > lo:
                door = (lo, hi)
        sides.append({'start': i, 'end': (i + 1) % 4, 'face': 'long' if long_face else 'short',
                      'feed': fed, 'length': length, 'door': door})
    warnings = []
    if min(gap, end_gap) < 0.02:
        warnings.append('less than 2 cm between the robot and the box')
    clear = math.hypot(o_long, o_short) - CORNER_REACH
    if clear < 0.03:
        warnings.append(f'turns too close to the box corners ({clear * 100:.0f} cm to spare)')
    if feed != 'none' and not any(s['door'] for s in sides):
        warnings.append('the door never opens: the margin is longer than the face')
    return {'corners': corners, 'sides': sides, 'warnings': warnings}


def door_open_at(side, along):
    """True if the door should be open with the robot `along` metres into side."""
    return side['door'] is not None and side['door'][0] <= along <= side['door'][1]


def start_corner(plan, x, y):
    """Index of the corner nearest to (x, y): laps start there."""
    return min(range(4), key=lambda i: math.dist(plan['corners'][i], (x, y)))


def door_length(plan):
    return sum(s['door'][1] - s['door'][0] for s in plan['sides'] if s['door'])


def lap_length(plan):
    return sum(s['length'] for s in plan['sides'])


# ---- checking the lap against the map --------------------------------------------


class WallDistance:
    """Distance (m) from any map point to the nearest wall cell.

    ignore: a Box whose own cells (plus ignore_margin, for the map drawing
    it thick) do not count. The lap's distance to the cow box is set by the
    gap and watched by the lidar; the map check is for everything else.
    """

    def __init__(self, grid, resolution, origin_x, origin_y, ignore=None, ignore_margin=0.15):
        self.grid = np.asarray(grid)
        self.res, self.ox, self.oy = float(resolution), float(origin_x), float(origin_y)
        self.h, self.w = self.grid.shape
        walls = self.grid >= WALL
        if ignore is not None:
            rows, cols = np.mgrid[0:self.h, 0:self.w]
            bx, by = ignore.to_box(self.ox + (cols + 0.5) * self.res, self.oy + (rows + 0.5) * self.res)
            walls &= ~((np.abs(bx) <= ignore.length / 2 + ignore_margin)
                       & (np.abs(by) <= ignore.width / 2 + ignore_margin))
        self.dist = distance_transform_edt(~walls) * self.res

    def __call__(self, x, y):
        c, r = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0
        return float(self.dist[r, c])


def check_plan(plan, wall_distance, step=0.05):
    """Places where the map says the robot will not fit: [(kind, index, metres)].

    The map draws walls a few cm thick, so this errs on the safe side; the
    lidar check while driving decides in the end.
    """
    problems = []
    for i, s in enumerate(plan['sides']):
        (x0, y0), (x1, y1) = plan['corners'][s['start']], plan['corners'][s['end']]
        n = max(2, int(s['length'] / step))
        worst = min(wall_distance(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
                    for t in np.linspace(0.0, 1.0, n))
        if worst < HALF_WIDTH:
            problems.append(('side', i, worst))
    for i, (x, y) in enumerate(plan['corners']):
        d = wall_distance(x, y)
        if d < CORNER_REACH:
            problems.append(('corner', i, d))
    return problems


# ---- finding the box -------------------------------------------------------------


def _min_area_rect(points, cell=0.0):
    """Smallest rectangle around points (Nx2) -> Box; cell widens it by one cell."""
    best = None
    for yaw in np.radians(np.arange(0.0, 90.0, 0.5)):
        c, s = math.cos(yaw), math.sin(yaw)
        u = points @ np.array([c, s])
        v = points @ np.array([-s, c])
        area = (u.max() - u.min()) * (v.max() - v.min())
        if best is None or area < best[0]:
            best = (area, yaw, u.min(), u.max(), v.min(), v.max())
    _, yaw, u0, u1, v0, v1 = best
    c, s = math.cos(yaw), math.sin(yaw)
    uc, vc = (u0 + u1) / 2, (v0 + v1) / 2
    return normalized(Box(uc * c - vc * s, uc * s + vc * c, yaw,
                          u1 - u0 + cell, v1 - v0 + cell))


def fit_box_from_map(grid, resolution, origin_x, origin_y, near):
    """The free-standing obstacle nearest to `near` (x, y), as a rectangle.

    Free-standing: a blob of wall cells that does not touch the arena walls
    (the largest blob) and is no longer than 3 m. None if there is none.
    """
    grid = np.asarray(grid)
    walls = grid >= WALL
    labels, n = label(walls, structure=np.ones((3, 3)))
    if n == 0:
        return None
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    outer = int(np.argmax(sizes))
    best = None
    for i in range(1, n + 1):
        if i == outer or sizes[i] < 6:
            continue
        rows, cols = np.nonzero(labels == i)
        xs = origin_x + (cols + 0.5) * resolution
        ys = origin_y + (rows + 0.5) * resolution
        if max(xs.max() - xs.min(), ys.max() - ys.min()) > 3.0:
            continue
        d = math.hypot(xs.mean() - near[0], ys.mean() - near[1])
        if best is None or d < best[0]:
            best = (d, np.c_[xs, ys])
    if best is None:
        return None
    return _min_area_rect(best[1], cell=resolution)


def _perimeter_distance(box, x, y, a=None, b=None):
    """Distance from points (box frame) to the rectangle's outline."""
    a = box.length / 2 if a is None else a
    b = box.width / 2 if b is None else b
    qx, qy = np.abs(x) - a, np.abs(y) - b
    outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    inside = np.minimum(np.maximum(qx, qy), 0)
    return np.abs(outside + inside)


def fit_box_to_points(box, points, reach=0.20):
    """Move and turn the box (size kept) so its outline sits on lidar points.

    points: Nx2 in the map frame. Only points within `reach` of the current
    outline count, so the arena walls do not pull it. From one place the lidar
    sees two faces at most, which fixes position and heading but not the size.
    Returns (box, number of points used, mean distance in m) or (None, n, None).
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return None, 0, None
    bx, by = box.to_box(pts[:, 0], pts[:, 1])
    near = _perimeter_distance(box, bx, by) < reach
    pts = pts[near]
    if len(pts) < 15:
        return None, len(pts), None

    def cost(cx, cy, yaw):
        b2 = Box(cx, cy, yaw, box.length, box.width)
        x, y = b2.to_box(pts[:, 0], pts[:, 1])
        return float(np.minimum(_perimeter_distance(b2, x, y), 0.08).mean())

    best = (cost(box.cx, box.cy, box.yaw), box.cx, box.cy, box.yaw)
    for step, ang in ((0.02, math.radians(1.0)), (0.005, math.radians(0.25))):
        span = 5 if step == 0.02 else 4
        _, cx0, cy0, yaw0 = best
        for dyaw in np.arange(-span, span + 1) * ang:
            for dx in np.arange(-span, span + 1) * step:
                for dy in np.arange(-span, span + 1) * step:
                    c = cost(cx0 + dx, cy0 + dy, yaw0 + dyaw)
                    if c < best[0]:
                        best = (c, cx0 + dx, cy0 + dy, yaw0 + dyaw)
    c, cx, cy, yaw = best
    return normalized(Box(cx, cy, yaw, box.length, box.width)), len(pts), c
