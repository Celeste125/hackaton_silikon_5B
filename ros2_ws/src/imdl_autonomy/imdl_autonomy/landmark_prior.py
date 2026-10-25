"""The learned location prior: its input window, its output, and its geometry.

This module is the contract between three things that must agree exactly or
the whole idea silently fails:

  the recorder   (landmark_mapper, on the robot) writing training samples
  the trainer    (landmark_prior/scripts, on a GPU box) building labels
  the inference  (landmark_mapper again) turning a heatmap into a prior

So it imports nothing but numpy. The training scripts live outside the ROS
workspace and import this file directly by path, rather than keeping their own
copy of the geometry -- a half-cell disagreement between the labels and the
inference would show up as a prior that is quietly offset, which is close to
unfindable by inspection.

## The window

The network sees a fixed-size **robot-centric, heading-aligned** crop of the
SLAM occupancy grid, not the map in map coordinates. That choice is the
important one:

  * The map frame's origin is wherever the robot happened to start, so an
    absolute window would put the same arena at a different offset in every
    run and the network would have to learn each of them separately.
  * Egocentric, it learns something transferable instead: "given walls in this
    arrangement around me, the dispenser is over there, two metres ahead and
    to the left." That holds whatever the map origin was, and it is how the
    object-goal navigation literature frames the same problem.
  * It also means a run does not need AMCL on a frozen map. The alternative,
    a stored landmarks file, does -- see FilePriorProvider.

Index convention, used everywhere: a window array is `[i, j]` where **i runs
forward** along the robot's x axis and **j runs left** along its y axis, with
the robot at the centre cell. Displayed as an image that puts forward down the
page, which is unusual to look at but keeps the maths in the robot's own
frame; the inspection script flips it for viewing.

## The input channels

Two, both derived from the occupancy grid alone:

  0. `occupied` -- 1 where the grid says there is something
  1. `unknown`  -- 1 where the grid is unexplored, or outside it entirely

Free space is the absence of both, so a third channel would be redundant.

Deliberately **absent: anything derived from the camera detections.** It is
tempting to feed the accumulated bearings in, and it would help the network
sharpen its guess. It would also be double counting: the estimator already
uses those same bearings as a likelihood, and a prior that has secretly seen
them would make the fused result overconfident -- reporting a tight covariance
it has not earned. Keeping the prior purely structural, `p(location | map
shape)`, is what makes multiplying it by the bearing likelihood valid.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .landmark_estimation import Prior

# Changing either of these invalidates every recorded sample and every trained
# network, so they are constants rather than parameters. 128 cells at 5 cm is
# 6.4 m across, which covers the whole ~2 m arena from anywhere inside it with
# room to spare for a landmark seen from the far corner.
WINDOW_CELLS = 128
WINDOW_RESOLUTION_M = 0.05
CHANNEL_NAMES = ('occupied', 'unknown')
# What counts as a wall in the network's input. Part of the frozen contract
# rather than a tunable: the recorder and the inference must agree, and a
# threshold that drifted between recording a dataset and deploying the model
# trained on it would change the input distribution silently.
OCCUPIED_THRESHOLD = 65

# Keys in a recorded .npz sample.
SAMPLE_CHANNELS = 'channels'
SAMPLE_ROBOT_POSE = 'robot_pose'
SAMPLE_STAMP = 'stamp'

DATASET_VERSION = 1


@dataclass(frozen=True)
class Window:
    """Where a window sat in the map, so its cells can be placed and read."""

    robot_x: float
    robot_y: float
    robot_yaw: float
    cells: int = WINDOW_CELLS
    resolution_m: float = WINDOW_RESOLUTION_M

    @property
    def half(self) -> float:
        return (self.cells - 1) / 2.0

    @property
    def extent_m(self) -> float:
        return self.cells * self.resolution_m

    def index_to_local(self, i: float, j: float) -> tuple[float, float]:
        """Window index -> metres in the robot frame (x forward, y left)."""
        return (
            (float(i) - self.half) * self.resolution_m,
            (float(j) - self.half) * self.resolution_m,
        )

    def index_to_map(self, i: float, j: float) -> tuple[float, float]:
        x, y = self.index_to_local(i, j)
        cos, sin = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        return (
            self.robot_x + cos * x - sin * y,
            self.robot_y + sin * x + cos * y,
        )

    def map_to_index(self, x: float, y: float) -> tuple[float, float]:
        """Map metres -> window index, fractional. May fall outside."""
        dx, dy = float(x) - self.robot_x, float(y) - self.robot_y
        cos, sin = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
        local_x = cos * dx - sin * dy
        local_y = sin * dx + cos * dy
        return (
            local_x / self.resolution_m + self.half,
            local_y / self.resolution_m + self.half,
        )

    def contains_index(self, i: float, j: float, margin: int = 0) -> bool:
        return (
            margin <= i <= self.cells - 1 - margin
            and margin <= j <= self.cells - 1 - margin
        )

    def rotation(self) -> np.ndarray:
        cos, sin = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        return np.array([[cos, -sin], [sin, cos]])


def build_channels(
    grid: np.ndarray,
    grid_resolution_m: float,
    grid_origin: Sequence[float],
    window: Window,
    occupied_threshold: int = OCCUPIED_THRESHOLD,
) -> np.ndarray:
    """Resample an occupancy grid into the robot-centric network input.

    `grid` is the raw occupancy array, `[row, column]`, in the convention ROS
    uses: -1 unknown, 0-100 probability of being occupied.

    Anything beyond the grid's edge is reported as unknown rather than free.
    That is the honest reading during a frontier run -- the map simply does not
    extend there yet -- and it is what stops the network from learning that
    the arena is surrounded by open space.
    """
    if grid.ndim != 2:
        raise ValueError('occupancy grid must be 2-D')
    if grid_resolution_m <= 0.0:
        raise ValueError('grid resolution must be positive')

    rows, columns = grid.shape
    indices = np.arange(window.cells, dtype=np.float64)
    i_grid, j_grid = np.meshgrid(indices, indices, indexing='ij')

    local_x = (i_grid - window.half) * window.resolution_m
    local_y = (j_grid - window.half) * window.resolution_m
    cos, sin = math.cos(window.robot_yaw), math.sin(window.robot_yaw)
    map_x = window.robot_x + cos * local_x - sin * local_y
    map_y = window.robot_y + sin * local_x + cos * local_y

    column = np.floor(
        (map_x - float(grid_origin[0])) / grid_resolution_m).astype(np.int64)
    row = np.floor(
        (map_y - float(grid_origin[1])) / grid_resolution_m).astype(np.int64)
    inside = (
        (column >= 0) & (column < columns) & (row >= 0) & (row < rows))

    values = np.full((window.cells, window.cells), -1, dtype=np.int16)
    values[inside] = grid[row[inside], column[inside]]

    channels = np.zeros(
        (len(CHANNEL_NAMES), window.cells, window.cells), dtype=np.float32)
    channels[0] = (values >= int(occupied_threshold)).astype(np.float32)
    channels[1] = (values < 0).astype(np.float32)
    return channels


def pack_channels(channels: np.ndarray) -> np.ndarray:
    """Channels as uint8 for storage. They are indicator masks, not images.

    Keeps a recorded run to a few hundred kilobytes rather than tens of
    megabytes, which matters when the samples are copied off the robot over
    the same WiFi everything else is competing for.
    """
    return np.ascontiguousarray(np.asarray(channels) > 0.5).astype(np.uint8)


def unpack_channels(packed: np.ndarray) -> np.ndarray:
    """The inverse of pack_channels, back to what the network expects."""
    return np.asarray(packed, dtype=np.float32)


def heatmap_moments(heatmap: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Mean and covariance of a spatial distribution, in cell units.

    A confident single-peaked heatmap yields a small covariance; a flat or
    multi-modal one yields a large one, which is exactly what should happen --
    an uncertain network produces a weak prior without anyone having to
    threshold its confidence. The one case this reads oddly is a sharply
    bimodal heatmap, whose mean lands between the two peaks; the covariance
    then spans both, so the prior is weak and harmless rather than wrong in a
    way that matters.
    """
    if heatmap.ndim != 2:
        raise ValueError('heatmap must be 2-D')
    weights = np.asarray(heatmap, dtype=np.float64)
    if np.any(weights < 0.0):
        raise ValueError('heatmap must be non-negative')
    total = float(weights.sum())
    if total <= 0.0:
        raise ValueError('heatmap sums to zero; nothing to summarise')
    weights = weights / total

    rows, columns = weights.shape
    i_grid, j_grid = np.meshgrid(
        np.arange(rows, dtype=np.float64),
        np.arange(columns, dtype=np.float64), indexing='ij')

    mean_i = float((weights * i_grid).sum())
    mean_j = float((weights * j_grid).sum())
    di, dj = i_grid - mean_i, j_grid - mean_j
    covariance = np.array([
        [float((weights * di * di).sum()), float((weights * di * dj).sum())],
        [float((weights * di * dj).sum()), float((weights * dj * dj).sum())],
    ])
    return np.array([mean_i, mean_j]), covariance


def heatmap_to_prior(
    heatmap: np.ndarray,
    window: Window,
    *,
    inflate: float = 1.5,
    min_sigma_m: float = 0.15,
    source: str = 'network',
) -> Prior:
    """Turn one output channel into a prior in the map frame.

    The covariance is carried through the rotation properly rather than being
    reduced to a single sigma: a heatmap stretched along a wall should stay
    stretched along that wall once it reaches the estimator, because that
    shape is real information about which direction is still unknown.
    """
    mean_cells, covariance_cells = heatmap_moments(heatmap)
    mean_map = window.index_to_map(mean_cells[0], mean_cells[1])

    # Cells to metres, then the robot frame to the map frame.
    covariance_local = covariance_cells * (window.resolution_m ** 2)
    rotation = window.rotation()
    covariance_map = rotation @ covariance_local @ rotation.T

    return Prior.from_covariance(
        mean_map, covariance_map.reshape(-1),
        inflate=inflate, min_sigma_m=min_sigma_m, source=source)


def softmax_2d(logits: np.ndarray) -> np.ndarray:
    """Softmax over both spatial axes of one channel."""
    shifted = np.asarray(logits, dtype=np.float64) - float(np.max(logits))
    exponentiated = np.exp(shifted)
    return exponentiated / float(exponentiated.sum())


# --------------------------------------------------------------- providers


class PriorProvider:
    """Where a prior comes from. The mapper holds exactly one of these."""

    name = 'none'

    def priors(
        self,
        landmarks: Sequence[str],
        window: Window,
        channels: np.ndarray,
    ) -> dict[str, Prior]:
        return {}

    def describe(self) -> str:
        return self.name


class FilePriorProvider(PriorProvider):
    """Positions a previous run wrote out, as a round prior around each.

    This is the simple version and it has one hard requirement: the positions
    are in **map** coordinates, so they only mean anything if this run's map
    frame is the same as the recording run's. That means localising with AMCL
    against a saved map, not starting fresh SLAM, whose origin moves with
    wherever the robot happened to begin. The network provider has no such
    requirement, which is most of why it is worth the trouble.
    """

    name = 'file'

    def __init__(self, document: Mapping[str, Any], sigma_m: float = 0.5):
        if sigma_m <= 0.0:
            raise ValueError('prior sigma must be positive')
        self._sigma_m = float(sigma_m)
        self._positions: dict[str, tuple[float, float]] = {}
        for name, entry in (document.get('landmarks') or {}).items():
            position = (entry or {}).get('position') or {}
            if 'x' in position and 'y' in position:
                self._positions[str(name)] = (
                    float(position['x']), float(position['y']))

    @classmethod
    def from_yaml(cls, path: str | Path, sigma_m: float = 0.5):
        import yaml  # noqa: PLC0415 - only this provider needs it

        text = Path(path).expanduser().read_text(encoding='utf-8')
        return cls(yaml.safe_load(text) or {}, sigma_m=sigma_m)

    def priors(self, landmarks, window, channels):
        return {
            name: Prior.isotropic(position, self._sigma_m, source=self.name)
            for name, position in self._positions.items()
            if name in landmarks
        }

    def describe(self) -> str:
        return f'file({len(self._positions)} landmarks, {self._sigma_m} m)'


class NetworkPriorProvider(PriorProvider):
    """The trained prior network, run through onnxruntime.

    ONNX rather than torch on purpose: the network is tiny, inference is well
    under a millisecond on a CPU, and keeping torch off the robot avoids the
    JetPack version-matching problem entirely. Export with
    landmark_prior/scripts/train_landmark_prior.py --export-onnx.
    """

    name = 'network'

    def __init__(
        self,
        model_path: str | Path,
        class_order: Sequence[str],
        *,
        inflate: float = 1.5,
        min_sigma_m: float = 0.15,
        providers: Sequence[str] | None = None,
    ):
        self._path = Path(model_path).expanduser()
        self._class_order = tuple(str(name) for name in class_order)
        self._inflate = float(inflate)
        self._min_sigma_m = float(min_sigma_m)
        self._providers = list(providers or ())
        self._session = None
        self._input_name = ''

    def load(self) -> None:
        if not self._path.is_file():
            raise FileNotFoundError(f'no prior network at {self._path}')
        if not self._class_order:
            raise ValueError(
                'the prior network needs its class order: the output channels '
                'are positional, so a wrong order swaps the landmarks')

        import onnxruntime  # noqa: PLC0415 - optional dependency

        self._session = onnxruntime.InferenceSession(
            str(self._path),
            providers=self._providers or onnxruntime.get_available_providers())
        self._input_name = self._session.get_inputs()[0].name

        outputs = self._session.get_outputs()[0].shape
        if len(outputs) == 4 and isinstance(outputs[1], int):
            if outputs[1] != len(self._class_order):
                raise ValueError(
                    f'{self._path.name} has {outputs[1]} output channels but '
                    f'{len(self._class_order)} landmarks were configured')

    def priors(self, landmarks, window, channels):
        if self._session is None:
            return {}
        logits = self._session.run(
            None, {self._input_name: channels[None].astype(np.float32)})[0]
        logits = np.asarray(logits)[0]
        result: dict[str, Prior] = {}
        for index, name in enumerate(self._class_order):
            if name not in landmarks or index >= logits.shape[0]:
                continue
            result[name] = heatmap_to_prior(
                softmax_2d(logits[index]), window,
                inflate=self._inflate, min_sigma_m=self._min_sigma_m,
                source=self.name)
        return result

    def describe(self) -> str:
        return f'network({self._path.name}, {list(self._class_order)})'
