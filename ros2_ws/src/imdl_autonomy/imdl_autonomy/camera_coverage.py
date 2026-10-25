"""Which floor the camera has actually looked at, and where to look next.

The lidar sees 360 degrees; the camera sees about 70 degrees out of the front
of the robot. So a frontier run that has finished mapping the arena has
usually looked at only a fraction of it, and the explorer declares itself done
while the dispenser may never have been in frame. This module is the
bookkeeping that fixes that.

What counts as "seen" is **known-free floor cells that have been inside the
camera frustum**, not walls. That is the right target for finding objects
rather than mapping geometry:

  * The lidar cannot see the dispenser, which is why the camera pipeline
    exists at all -- so the dispenser's own footprint is *free* in the SLAM
    map. Cover every free cell and you have covered wherever it is standing.
  * The enclosure is large and lidar-visible, so its cells are occupied; but
    covering the free cells around it puts it in frame anyway.
  * Unknown cells are not counted either way. They are the lidar explorer's
    job, and counting them would make coverage unreachable by construction.

Occlusion is honoured: a cell behind a wall is not seen just because it fell
inside the frustum's wedge. The ray marching is vectorised the same way
next_best_view.visible_mask does it, with a running maximum over the blocked
flags, but on the live nav_msgs grid convention -- row 0 at the origin -- so
it shares a frame convention with landmark_prior and the wall cast rather than
the row-flipped one that module uses for maps loaded from PGM files.

Nothing here imports ROS, so it runs under pytest.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np

# nav_msgs occupancy values.
UNKNOWN = -1
DEFAULT_OCCUPIED_THRESHOLD = 65
# Below this a cell counts as free floor rather than merely not-yet-a-wall.
DEFAULT_FREE_THRESHOLD = 50


@dataclass(frozen=True)
class CameraModel:
    """The camera's footprint on the floor, as the coverage check sees it.

    `max_range_m` is not the range at which the camera forms an image -- it is
    the range at which the model can still *recognise* something. Beyond a
    few metres a dispenser is a handful of pixels and a detection there is
    neither likely nor worth much, so counting that floor as covered would be
    a lie.
    """

    fov_rad: float = math.radians(70.4)
    min_range_m: float = 0.30
    max_range_m: float = 3.00
    # Where the camera points relative to base_link's x axis. Zero for a
    # front-facing mount, which is what this robot has.
    bearing_offset_rad: float = 0.0

    def __post_init__(self):
        if not 0.0 < self.fov_rad < 2.0 * math.pi:
            raise ValueError('fov_rad must be within (0, 2pi)')
        if self.min_range_m < 0.0 or self.max_range_m <= self.min_range_m:
            raise ValueError('need 0 <= min_range_m < max_range_m')


@dataclass(frozen=True)
class GridGeometry:
    """Enough of a nav_msgs/OccupancyGrid to place its cells in the world."""

    resolution_m: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def cell_centres(self) -> tuple[np.ndarray, np.ndarray]:
        rows = np.arange(self.height)
        columns = np.arange(self.width)
        x = self.origin_x + (columns + 0.5) * self.resolution_m
        y = self.origin_y + (rows + 0.5) * self.resolution_m
        return np.meshgrid(x, y)  # both (height, width)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        column = int(math.floor((x - self.origin_x) / self.resolution_m))
        row = int(math.floor((y - self.origin_y) / self.resolution_m))
        if 0 <= row < self.height and 0 <= column < self.width:
            return row, column
        return None

    def cell_to_world(self, row: int, column: int) -> tuple[float, float]:
        return (
            self.origin_x + (column + 0.5) * self.resolution_m,
            self.origin_y + (row + 0.5) * self.resolution_m,
        )


def classify(
    cells: np.ndarray,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
    free_threshold: int = DEFAULT_FREE_THRESHOLD,
) -> tuple[np.ndarray, np.ndarray]:
    """Split a grid into (occupied, free). Unknown is in neither."""
    occupied = cells >= occupied_threshold
    free = (cells >= 0) & (cells < free_threshold)
    return occupied, free


def visible_mask(
    occupied: np.ndarray,
    geometry: GridGeometry,
    pose: Sequence[float],
    camera: CameraModel,
) -> np.ndarray:
    """Cells the camera can see from `pose` = (x, y, yaw), with occlusion.

    One ray per half-cell of arc at the far edge, so no cell inside the wedge
    is missed between adjacent rays, and samples every half cell along each
    ray. The occlusion is the running maximum of the blocked flag along each
    ray: once a ray has passed through a wall, everything further out on that
    ray is hidden.
    """
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    heading = yaw + camera.bearing_offset_rad
    resolution = geometry.resolution_m

    arc_length = camera.fov_rad * camera.max_range_m
    ray_count = max(31, int(math.ceil(2.0 * arc_length / resolution)))
    angles = np.linspace(
        heading - camera.fov_rad * 0.5,
        heading + camera.fov_rad * 0.5,
        ray_count,
    )[:, None]
    ranges = np.arange(
        camera.min_range_m, camera.max_range_m + resolution * 0.5,
        resolution * 0.5,
    )[None, :]

    world_x = x + np.cos(angles) * ranges
    world_y = y + np.sin(angles) * ranges
    columns = np.floor((world_x - geometry.origin_x) / resolution).astype(int)
    rows = np.floor((world_y - geometry.origin_y) / resolution).astype(int)

    inside = (
        (rows >= 0) & (rows < geometry.height)
        & (columns >= 0) & (columns < geometry.width)
    )
    safe_rows = np.clip(rows, 0, geometry.height - 1)
    safe_columns = np.clip(columns, 0, geometry.width - 1)

    blocked = (~inside) | occupied[safe_rows, safe_columns]
    # A wall is itself visible -- it is what stops the ray -- so the running
    # maximum is taken from the *previous* sample along each ray.
    hidden = np.zeros_like(blocked)
    hidden[:, 1:] = np.maximum.accumulate(blocked[:, :-1], axis=1)
    usable = inside & ~hidden

    visible = np.zeros(occupied.shape, dtype=bool)
    visible[safe_rows[usable], safe_columns[usable]] = True
    return visible


def remap_seen(
    seen: np.ndarray,
    old: GridGeometry,
    new: GridGeometry,
) -> np.ndarray:
    """Carry coverage across a map that has grown or shifted under it.

    slam_toolbox enlarges /map as exploration proceeds, which moves the
    origin and changes the dimensions. Without this, every such update would
    silently reset the coverage to nothing and the robot would re-sweep floor
    it had already looked at, for ever.

    Cells are matched by world position, so only an exact resolution change
    forces a reset -- and that would mean a different map entirely.
    """
    fresh = np.zeros((new.height, new.width), dtype=bool)
    if abs(old.resolution_m - new.resolution_m) > 1e-9:
        return fresh
    if seen.shape != (old.height, old.width):
        return fresh

    # Where the old grid's cell (0, 0) lands in the new grid.
    row_shift = int(round((old.origin_y - new.origin_y) / new.resolution_m))
    column_shift = int(round((old.origin_x - new.origin_x) / new.resolution_m))

    src_row0 = max(0, -row_shift)
    src_col0 = max(0, -column_shift)
    dst_row0 = max(0, row_shift)
    dst_col0 = max(0, column_shift)
    rows = min(old.height - src_row0, new.height - dst_row0)
    columns = min(old.width - src_col0, new.width - dst_col0)
    if rows <= 0 or columns <= 0:
        return fresh

    fresh[dst_row0:dst_row0 + rows, dst_col0:dst_col0 + columns] = (
        seen[src_row0:src_row0 + rows, src_col0:src_col0 + columns])
    return fresh


def coverage_fraction(seen: np.ndarray, free: np.ndarray) -> float:
    """How much of the known-free floor has been in frame. 1.0 when none."""
    total = float(free.sum())
    if total <= 0.0:
        return 1.0
    return float((seen & free).sum()) / total


def unseen_clusters(
    seen: np.ndarray,
    free: np.ndarray,
    geometry: GridGeometry,
    *,
    min_cluster_cells: int = 8,
) -> list[tuple[float, float, int]]:
    """Group free-but-unseen cells into (x, y, size) blobs, largest first.

    Flood filled with an explicit stack rather than scipy.ndimage.label: the
    grids here are a couple of hundred cells square, the saving is irrelevant,
    and it keeps this module testable with nothing but numpy.
    """
    target = free & ~seen
    visited = np.zeros_like(target)
    clusters: list[tuple[float, float, int]] = []
    rows, columns = np.nonzero(target)

    for start_row, start_column in zip(rows, columns):
        if visited[start_row, start_column]:
            continue
        stack = [(int(start_row), int(start_column))]
        visited[start_row, start_column] = True
        members: list[tuple[int, int]] = []
        while stack:
            row, column = stack.pop()
            members.append((row, column))
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = row + dr, column + dc
                if (
                    0 <= nr < geometry.height and 0 <= nc < geometry.width
                    and target[nr, nc] and not visited[nr, nc]
                ):
                    visited[nr, nc] = True
                    stack.append((nr, nc))
        if len(members) < min_cluster_cells:
            continue
        centre_row = sum(m[0] for m in members) / len(members)
        centre_column = sum(m[1] for m in members) / len(members)
        x, y = geometry.cell_to_world(centre_row, centre_column)
        clusters.append((x, y, len(members)))

    clusters.sort(key=lambda item: item[2], reverse=True)
    return clusters


def has_clearance(
    occupied: np.ndarray,
    free: np.ndarray,
    geometry: GridGeometry,
    x: float,
    y: float,
    radius_m: float,
) -> bool:
    """True when a disc of this radius around (x, y) is known-free floor.

    A plain neighbourhood test rather than a distance transform: the radius is
    a few cells, and this keeps scipy out of a module that is otherwise pure
    numpy. Unknown cells fail the test, so the robot is never sent to stand in
    unexplored space.
    """
    cell = geometry.world_to_cell(x, y)
    if cell is None:
        return False
    radius = max(int(math.ceil(radius_m / geometry.resolution_m)), 0)
    row, column = cell
    row0, row1 = max(row - radius, 0), min(row + radius + 1, geometry.height)
    col0, col1 = (
        max(column - radius, 0), min(column + radius + 1, geometry.width))
    if row1 - row0 <= 2 * radius or col1 - col0 <= 2 * radius:
        return False  # the disc runs off the map
    patch_occupied = occupied[row0:row1, col0:col1]
    patch_free = free[row0:row1, col0:col1]

    offsets = np.arange(-radius, radius + 1)
    dr, dc = np.meshgrid(offsets, offsets, indexing='ij')
    disc = (dr ** 2 + dc ** 2) <= radius ** 2
    return bool(
        not patch_occupied[disc].any() and patch_free[disc].all())


@dataclass(frozen=True)
class ViewGoal:
    """Somewhere to stand, and what looking from there would buy."""

    x: float
    y: float
    yaw: float
    new_cells: int
    travel_m: float
    score: float


def plan_view(
    occupied: np.ndarray,
    free: np.ndarray,
    seen: np.ndarray,
    geometry: GridGeometry,
    robot: Sequence[float],
    camera: CameraModel,
    *,
    standoff_m: float = 1.2,
    clearance_m: float = 0.25,
    headings: int = 8,
    max_clusters: int = 4,
    min_new_cells: int = 8,
    min_goal_distance_m: float = 0.25,
    blacklist: Iterable[Sequence[float]] = (),
    blacklist_radius_m: float = 0.3,
) -> ViewGoal | None:
    """Pick the pose that uncovers the most unseen floor per metre driven.

    Candidates are rings of standing positions around the biggest unseen
    blobs, each facing that blob, plus the robot's current position turned to
    each heading -- the latter matters because the cheapest way to see new
    floor is very often to turn in place, which costs no travel at all and
    which a blob-centred ring alone would never propose.
    """
    clusters = unseen_clusters(seen, free, geometry)[:max_clusters]
    if not clusters:
        return None

    robot_x, robot_y, robot_yaw = (
        float(robot[0]), float(robot[1]), float(robot[2]))
    rejected = [np.asarray(point[:2], dtype=float) for point in blacklist]

    candidates: list[tuple[float, float, float]] = []
    # Turning in place: same spot, every heading.
    for index in range(headings):
        candidates.append(
            (robot_x, robot_y, robot_yaw + 2.0 * math.pi * index / headings))
    # A ring around each unseen blob, each candidate facing the blob.
    for target_x, target_y, _ in clusters:
        for index in range(headings):
            angle = 2.0 * math.pi * index / headings
            x = target_x + standoff_m * math.cos(angle)
            y = target_y + standoff_m * math.sin(angle)
            candidates.append(
                (x, y, math.atan2(target_y - y, target_x - x)))

    best: ViewGoal | None = None
    for x, y, yaw in candidates:
        moved = math.hypot(x - robot_x, y - robot_y)
        if moved > 1e-6 and moved < min_goal_distance_m:
            continue  # too close to be worth a Nav2 goal
        if any(
            float(np.linalg.norm(np.array([x, y]) - point))
            < blacklist_radius_m for point in rejected
        ):
            continue
        if moved > 1e-6 and not has_clearance(
            occupied, free, geometry, x, y, clearance_m
        ):
            continue

        gain = int((
            visible_mask(occupied, geometry, (x, y, yaw), camera)
            & free & ~seen
        ).sum())
        if gain < min_new_cells:
            continue
        # Per metre, with a floor so that turning in place is not infinitely
        # attractive and a 2 m drive for twice the gain can still win.
        score = gain / (1.0 + moved)
        if best is None or score > best.score:
            best = ViewGoal(
                x=x, y=y, yaw=yaw, new_cells=gain, travel_m=moved,
                score=score)
    return best
