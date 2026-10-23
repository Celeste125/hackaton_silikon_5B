"""Map-aware next-best-view and approach-pose planning.

The implementation keeps a discrete probability distribution over possible
object cells. Candidate camera poses are evaluated using line-of-sight,
distance-dependent detection probability, grid-path travel time, turn time,
and obstacle clearance.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image
import yaml

from .geometry import shortest_angular_distance
from .mission_spec import Pose2D

GridCell = Tuple[int, int]


@dataclass(frozen=True)
class ViewCandidate:
    pose: Pose2D
    score: float
    expected_detection: float
    new_coverage: float
    path_length_m: float
    travel_time_sec: float
    turn_time_sec: float
    risk: float


@dataclass(frozen=True)
class ApproachCandidate:
    pose: Pose2D
    score: float
    path_length_m: float
    clearance_m: float
    camera_error_rad: float
    accepted: bool
    reason: str = ''


class OccupancyGrid:
    """Occupancy map with conservative clearance and A* path estimates."""

    _NEIGHBORS = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )

    def __init__(
        self,
        occupied: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ):
        if occupied.ndim != 2:
            raise ValueError('occupied grid must be two-dimensional')
        self.occupied = occupied.astype(bool)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = self.occupied.shape
        self.clearance = self._distance_transform()

    @classmethod
    def from_yaml(cls, path: str | Path) -> 'OccupancyGrid':
        yaml_path = Path(path).expanduser().resolve()
        data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            raise ValueError(f'Invalid map YAML: {yaml_path}')
        image_path = Path(str(data['image'])).expanduser()
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        pixels = cls._load_grayscale(image_path)
        negate = bool(int(data.get('negate', 0)))
        occupancy_probability = pixels / 255.0 if negate else 1.0 - pixels / 255.0
        occupied_threshold = float(data.get('occupied_thresh', 0.65))
        free_threshold = float(data.get('free_thresh', 0.25))
        occupied = occupancy_probability >= occupied_threshold
        unknown = (
            (occupancy_probability > free_threshold)
            & (occupancy_probability < occupied_threshold)
        )
        occupied |= unknown
        origin = data.get('origin', [0.0, 0.0, 0.0])
        return cls(
            occupied=occupied,
            resolution=float(data['resolution']),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
        )

    @staticmethod
    def _load_grayscale(image_path: Path) -> np.ndarray:
        try:
            return np.asarray(Image.open(image_path).convert('L'), dtype=float)
        except OSError:
            if image_path.suffix.lower() != '.pgm':
                raise
            with image_path.open('r', encoding='ascii') as image_file:
                magic = image_file.readline().strip()
                dimensions = image_file.readline().split()
                maximum = image_file.readline().strip()
            if magic != 'P2' or len(dimensions) != 2 or maximum != '255':
                raise ValueError(f'Unsupported PGM format: {image_path}')
            width, height = (int(value) for value in dimensions)
            pixels = np.loadtxt(image_path, dtype=float, skiprows=3)
            if pixels.size != width * height:
                raise ValueError(
                    f'PGM pixel count mismatch in {image_path}: '
                    f'{pixels.size} != {width * height}'
                )
            return pixels.reshape((height, width))

    def in_bounds(self, cell: GridCell) -> bool:
        row, column = cell
        return 0 <= row < self.height and 0 <= column < self.width

    def world_to_cell(self, x: float, y: float) -> Optional[GridCell]:
        column = int(math.floor((x - self.origin_x) / self.resolution))
        row_from_bottom = int(math.floor((y - self.origin_y) / self.resolution))
        row = self.height - 1 - row_from_bottom
        cell = (row, column)
        return cell if self.in_bounds(cell) else None

    def cell_to_world(self, cell: GridCell) -> Tuple[float, float]:
        row, column = cell
        row_from_bottom = self.height - 1 - row
        return (
            self.origin_x + (column + 0.5) * self.resolution,
            self.origin_y + (row_from_bottom + 0.5) * self.resolution,
        )

    def clearance_at(self, x: float, y: float) -> float:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return 0.0
        return float(self.clearance[cell])

    def is_free(self, x: float, y: float, clearance_m: float = 0.0) -> bool:
        cell = self.world_to_cell(x, y)
        if cell is None or self.occupied[cell]:
            return False
        return self.clearance[cell] >= clearance_m

    def line_of_sight(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
        endpoint_margin_m: float = 0.025,
    ) -> bool:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance <= endpoint_margin_m:
            return True
        checked_distance = max(0.0, distance - endpoint_margin_m)
        samples = max(2, int(math.ceil(checked_distance / (self.resolution * 0.5))))
        for index in range(1, samples + 1):
            fraction = (index / samples) * (checked_distance / distance)
            x = start[0] + fraction * dx
            y = start[1] + fraction * dy
            cell = self.world_to_cell(x, y)
            if cell is None or self.occupied[cell]:
                return False
        return True

    def path_length(
        self,
        start: Pose2D,
        goal: Pose2D,
        clearance_m: float,
    ) -> float:
        start_cell = self.world_to_cell(start.x, start.y)
        goal_cell = self.world_to_cell(goal.x, goal.y)
        if start_cell is None or goal_cell is None:
            return math.inf
        if (
            self.clearance[start_cell] < clearance_m
            or self.clearance[goal_cell] < clearance_m
        ):
            return math.inf
        if start_cell == goal_cell:
            return start.distance_to(goal)

        frontier: List[Tuple[float, float, GridCell]] = []
        heapq.heappush(frontier, (0.0, 0.0, start_cell))
        best: Dict[GridCell, float] = {start_cell: 0.0}
        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if current == goal_cell:
                return cost * self.resolution
            if cost > best.get(current, math.inf):
                continue
            for dr, dc, step in self._NEIGHBORS:
                neighbor = (current[0] + dr, current[1] + dc)
                if not self.in_bounds(neighbor):
                    continue
                if self.clearance[neighbor] < clearance_m:
                    continue
                proximity = math.exp(
                    -self.clearance[neighbor] / max(clearance_m, 0.03)
                )
                next_cost = cost + step * (1.0 + 0.35 * proximity)
                if next_cost >= best.get(neighbor, math.inf):
                    continue
                best[neighbor] = next_cost
                heuristic = math.hypot(
                    neighbor[0] - goal_cell[0],
                    neighbor[1] - goal_cell[1],
                )
                heapq.heappush(
                    frontier,
                    (next_cost + heuristic, next_cost, neighbor),
                )
        return math.inf

    def _distance_transform(self) -> np.ndarray:
        distance_cells = np.full(self.occupied.shape, np.inf, dtype=float)
        frontier: List[Tuple[float, int, int]] = []
        occupied_rows, occupied_columns = np.nonzero(self.occupied)
        for row, column in zip(occupied_rows, occupied_columns):
            distance_cells[row, column] = 0.0
            heapq.heappush(frontier, (0.0, int(row), int(column)))
        while frontier:
            distance, row, column = heapq.heappop(frontier)
            if distance > distance_cells[row, column]:
                continue
            for dr, dc, step in self._NEIGHBORS:
                nr, nc = row + dr, column + dc
                if not (0 <= nr < self.height and 0 <= nc < self.width):
                    continue
                candidate = distance + step
                if candidate < distance_cells[nr, nc]:
                    distance_cells[nr, nc] = candidate
                    heapq.heappush(frontier, (candidate, nr, nc))
        return distance_cells * self.resolution


class NextBestViewPlanner:
    """Belief-space viewpoint selection for unknown colored blocks."""

    def __init__(
        self,
        grid: OccupancyGrid,
        camera_fov_rad: float = 1.211,
        camera_min_range_m: float = 0.12,
        camera_max_range_m: float = 0.90,
        robot_clearance_m: float = 0.205,
        view_clearance_margin_m: float = 0.0,
        max_linear_speed_mps: float = 0.14,
        max_angular_speed_radps: float = 0.90,
    ):
        self.grid = grid
        self.camera_fov_rad = float(camera_fov_rad)
        self.camera_min_range_m = float(camera_min_range_m)
        self.camera_max_range_m = float(camera_max_range_m)
        self.robot_clearance_m = float(robot_clearance_m)
        self.view_clearance_margin_m = max(
            0.0,
            float(view_clearance_margin_m),
        )
        self.max_linear_speed_mps = float(max_linear_speed_mps)
        self.max_angular_speed_radps = float(max_angular_speed_radps)
        possible = (~grid.occupied) & (grid.clearance >= 0.025)
        self.belief = possible.astype(float)
        total = float(self.belief.sum())
        if total <= 0.0:
            raise ValueError('Map contains no possible object cells')
        self.belief /= total
        self.searched = np.zeros_like(possible, dtype=bool)
        rows, columns = np.indices(grid.occupied.shape)
        row_from_bottom = grid.height - 1 - rows
        self._world_x = grid.origin_x + (columns + 0.5) * grid.resolution
        self._world_y = grid.origin_y + (row_from_bottom + 0.5) * grid.resolution

    def rank_candidates(
        self,
        current: Pose2D,
        limit: int = 12,
        future_color_bonus: float = 0.0,
    ) -> List[ViewCandidate]:
        spatial_candidates = self._sample_robot_positions(current)
        ranked: List[ViewCandidate] = []
        headings = [index * math.pi / 4.0 for index in range(8)]
        for x, y in spatial_candidates:
            path_length_m = self.grid.path_length(
                current,
                Pose2D(x, y, 0.0),
                self.robot_clearance_m,
            )
            if not math.isfinite(path_length_m):
                continue
            best_at_position = None
            for yaw in headings:
                pose = Pose2D(x, y, yaw)
                candidate = self._score_candidate(
                    current,
                    pose,
                    future_color_bonus,
                    path_length_m,
                )
                if candidate is not None and (
                    best_at_position is None
                    or candidate.score > best_at_position.score
                ):
                    best_at_position = candidate
            if best_at_position is not None:
                ranked.append(best_at_position)
        ranked.sort(key=lambda item: item.score, reverse=True)
        return ranked[:max(1, int(limit))]

    def observe_negative(
        self,
        pose: Pose2D,
        miss_probability: float = 0.22,
    ) -> float:
        visible = self.visible_mask(pose)
        probability_before = float(self.belief[visible].sum())
        self.belief[visible] *= max(0.01, min(0.99, miss_probability))
        self.searched |= visible
        total = float(self.belief.sum())
        if total > 0.0:
            self.belief /= total
        return probability_before

    def visible_mask(self, pose: Pose2D) -> np.ndarray:
        ray_count = max(
            41,
            int(math.ceil(
                1.8 * self.camera_fov_rad * self.camera_max_range_m
                / self.grid.resolution
            )),
        )
        ranges = np.arange(
            self.camera_min_range_m,
            self.camera_max_range_m + self.grid.resolution * 0.5,
            self.grid.resolution * 0.5,
            dtype=float,
        )[None, :]
        angles = np.linspace(
            pose.yaw - self.camera_fov_rad * 0.5,
            pose.yaw + self.camera_fov_rad * 0.5,
            ray_count,
            dtype=float,
        )[:, None]
        world_x = pose.x + np.cos(angles) * ranges
        world_y = pose.y + np.sin(angles) * ranges
        columns = np.floor(
            (world_x - self.grid.origin_x) / self.grid.resolution
        ).astype(int)
        rows_from_bottom = np.floor(
            (world_y - self.grid.origin_y) / self.grid.resolution
        ).astype(int)
        rows = self.grid.height - 1 - rows_from_bottom
        valid = (
            (rows >= 0) & (rows < self.grid.height)
            & (columns >= 0) & (columns < self.grid.width)
        )
        safe_rows = np.clip(rows, 0, self.grid.height - 1)
        safe_columns = np.clip(columns, 0, self.grid.width - 1)
        blocked = (~valid) | self.grid.occupied[safe_rows, safe_columns]
        blocked_before = np.maximum.accumulate(blocked, axis=1)
        visible_samples = valid & ~blocked_before
        visible = np.zeros_like(self.grid.occupied, dtype=bool)
        np.logical_or.at(
            visible,
            (safe_rows[visible_samples], safe_columns[visible_samples]),
            True,
        )
        expanded = visible.copy()
        expanded[1:, :] |= visible[:-1, :]
        expanded[:-1, :] |= visible[1:, :]
        expanded[:, 1:] |= visible[:, :-1]
        expanded[:, :-1] |= visible[:, 1:]
        visible = expanded & (self.belief > 0.0)
        return visible

    def searched_points(self, stride: int = 3) -> List[Tuple[float, float]]:
        points = []
        rows, columns = np.nonzero(self.searched)
        for index, (row, column) in enumerate(zip(rows, columns)):
            if index % max(1, stride) == 0:
                points.append(self.grid.cell_to_world((int(row), int(column))))
        return points

    def frustum(self, pose: Pose2D) -> Tuple[Tuple[float, float], ...]:
        half = self.camera_fov_rad * 0.5
        return (
            (pose.x, pose.y),
            (
                pose.x + self.camera_max_range_m * math.cos(pose.yaw - half),
                pose.y + self.camera_max_range_m * math.sin(pose.yaw - half),
            ),
            (
                pose.x + self.camera_max_range_m * math.cos(pose.yaw + half),
                pose.y + self.camera_max_range_m * math.sin(pose.yaw + half),
            ),
            (pose.x, pose.y),
        )

    def approach_ring(
        self,
        current: Pose2D,
        target: Pose2D,
        radius_m: float = 0.235,
        samples: int = 16,
    ) -> List[ApproachCandidate]:
        candidates: List[ApproachCandidate] = []
        for index in range(max(4, samples)):
            angle = 2.0 * math.pi * index / max(4, samples)
            x = target.x + radius_m * math.cos(angle)
            y = target.y + radius_m * math.sin(angle)
            yaw = math.atan2(target.y - y, target.x - x)
            pose = Pose2D(x, y, yaw)
            clearance = self.grid.clearance_at(x, y)
            reason = ''
            path = math.inf
            if clearance < self.robot_clearance_m:
                reason = 'insufficient footprint clearance'
            elif not self.grid.line_of_sight((x, y), (target.x, target.y)):
                reason = 'camera line of sight blocked'
            else:
                path = self.grid.path_length(
                    current,
                    pose,
                    self.robot_clearance_m,
                )
                if not math.isfinite(path):
                    reason = 'no collision-free static-map path'
            camera_error = abs(shortest_angular_distance(
                yaw,
                math.atan2(target.y - y, target.x - x),
            ))
            accepted = not reason
            risk = 1.0 / max(clearance, 0.01)
            score = -path - 0.025 * risk - 0.20 * camera_error
            candidates.append(ApproachCandidate(
                pose=pose,
                score=score if accepted else -math.inf,
                path_length_m=path,
                clearance_m=clearance,
                camera_error_rad=camera_error,
                accepted=accepted,
                reason=reason,
            ))
        candidates.sort(key=lambda item: item.score, reverse=True)
        return candidates

    def _sample_robot_positions(self, current: Pose2D) -> List[Tuple[float, float]]:
        step_cells = max(1, int(round(0.12 / self.grid.resolution)))
        view_clearance = (
            self.robot_clearance_m + self.view_clearance_margin_m
        )
        positions = []
        for row in range(0, self.grid.height, step_cells):
            for column in range(0, self.grid.width, step_cells):
                if self.grid.clearance[row, column] < view_clearance:
                    continue
                x, y = self.grid.cell_to_world((row, column))
                positions.append((x, y))
        positions.sort(key=lambda item: math.hypot(
            item[0] - current.x,
            item[1] - current.y,
        ))
        return positions[:60]

    def _score_candidate(
        self,
        current: Pose2D,
        pose: Pose2D,
        future_color_bonus: float,
        path_length_m: Optional[float] = None,
    ) -> Optional[ViewCandidate]:
        visible = self.visible_mask(pose)
        if not np.any(visible):
            return None
        if path_length_m is None:
            path_length_m = self.grid.path_length(
                current,
                pose,
                self.robot_clearance_m,
            )
        if not math.isfinite(path_length_m):
            return None

        new_visible = visible & ~self.searched
        distances = np.hypot(
            self._world_x[visible] - pose.x,
            self._world_y[visible] - pose.y,
        )
        range_fraction = np.clip(
            (distances - self.camera_min_range_m)
            / max(self.camera_max_range_m - self.camera_min_range_m, 1e-6),
            0.0,
            1.0,
        )
        detection_probability = 0.92 - 0.42 * range_fraction
        expected_detection = float(np.sum(
            self.belief[visible] * detection_probability
        ))
        new_coverage = float(self.belief[new_visible].sum())

        travel_time = path_length_m / max(self.max_linear_speed_mps, 1e-3)
        initial_heading = math.atan2(pose.y - current.y, pose.x - current.x)
        turn_angle = abs(shortest_angular_distance(current.yaw, initial_heading))
        turn_angle += abs(shortest_angular_distance(initial_heading, pose.yaw))
        turn_time = turn_angle / max(self.max_angular_speed_radps, 1e-3)
        clearance = self.grid.clearance_at(pose.x, pose.y)
        risk = math.exp(-clearance / 0.08)
        information_value = expected_detection + 0.65 * new_coverage
        information_value *= 1.0 + max(0.0, future_color_bonus)
        score = (
            36.0 * information_value
            - 0.28 * travel_time
            - 0.12 * turn_time
            - 2.0 * risk
        )
        return ViewCandidate(
            pose=pose,
            score=score,
            expected_detection=expected_detection,
            new_coverage=new_coverage,
            path_length_m=path_length_m,
            travel_time_sec=travel_time,
            turn_time_sec=turn_time,
            risk=risk,
        )
