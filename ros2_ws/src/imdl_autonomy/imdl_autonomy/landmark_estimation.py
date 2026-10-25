"""Multi-view fusion of camera bearings into a map position, with an error bar.

This is the part that makes an estimate improve as the robot sees a landmark
from more directions, so it is worth being precise about why.

Each view gives a sight line: an origin (where the camera was, in the map) and
a unit direction. The landmark lies somewhere along it. Written as a
measurement, the informative part of one view is the *perpendicular* offset
from its sight line being zero:

    n . (x - o) = 0,     n = perp(d)

with a standard deviation of roughly ``range * bearing_sigma`` -- a box centre
that is off by one degree misses by more the further away it is.

Stacking those constraints and solving in information form,

    L  = sum  w_i n_i n_i^T          w_i = 1 / (r_i * sigma_i)^2
    e  = sum  w_i n_i (n_i . o_i)
    x  = L^-1 e,   covariance = L^-1

gives exactly the behaviour asked for, and gives it honestly:

  * Two views from nearly the same place have nearly parallel n, so L is close
    to rank one: the covariance stays enormous *along* the view direction.
    Averaging the two would have produced a confident-looking answer in the
    middle of nowhere; this reports a long thin ellipse instead.
  * Two views ninety degrees apart have orthogonal n, so both axes are pinned
    and the ellipse is small and round.
  * Each further view adds its information, so the ellipse keeps shrinking,
    roughly as 1/sqrt(views) for views at a new angle and barely at all for
    views that repeat an angle already held.

A range, when a view has one -- sampled from a depth image, or cast at a wall
behind the object -- is a second constraint on the same view, *along* the
sight line:

    d . (x - o) = r

so an RGB-D view fixes a landmark on its own, while a plain webcam needs the
robot to have moved. Both paths end in the same covariance, which is what lets
the two be mixed in one run and still reported meaningfully.

Nothing here imports ROS, so it runs under pytest on a laptop. The node that
feeds it is landmark_mapper.py.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np

# Below this, the solve is treated as having no constraint in that direction
# at all. 1 / 20 m squared: an axis we know to worse than 20 m is not known.
_MIN_INFORMATION = 1.0 / (20.0 ** 2)

# A range of zero would divide by zero in the bearing weight, and a landmark
# inside the camera is not a real measurement anyway.
_MIN_RANGE_M = 0.10


@dataclass(frozen=True)
class Prior:
    """A belief about where a landmark is, held before seeing it this run.

    Two things produce one: the landmarks file a previous run left behind, and
    the learned prior network, which looks at the partial SLAM map and guesses
    from the arena's shape. Either way it enters the solve as one more
    information contribution, which is what makes this safe:

      * Before any detection it is the whole estimate, so the robot has
        somewhere to drive rather than nothing.
      * Once real bearings arrive they outvote it in proportion to their own
        information. No arbitration logic, no mode switch.
      * If it is wrong -- the dispenser was moved, or the network guessed
        badly -- the observations simply overwhelm it, and the covariance says
        so honestly while that happens.

    `information` is the 2x2 inverse covariance, row-major. Working in
    information form rather than covariance matters: a prior that knows
    nothing has zero information, which adds correctly, whereas an infinite
    covariance does not.
    """

    mean: tuple[float, float]
    information: tuple[float, float, float, float]
    source: str = 'prior'

    @classmethod
    def isotropic(
        cls,
        mean: tuple[float, float],
        sigma_m: float,
        source: str = 'prior',
    ) -> 'Prior':
        """A round prior: 'probably here, to about this many metres'."""
        if sigma_m <= 0.0:
            raise ValueError('prior sigma must be positive')
        weight = 1.0 / (sigma_m ** 2)
        return cls(
            mean=(float(mean[0]), float(mean[1])),
            information=(weight, 0.0, 0.0, weight),
            source=source,
        )

    @classmethod
    def from_covariance(
        cls,
        mean: tuple[float, float],
        covariance: Sequence[float],
        *,
        inflate: float = 1.0,
        min_sigma_m: float = 0.10,
        source: str = 'prior',
    ) -> 'Prior':
        """Build from a covariance, with a floor on how certain it may claim.

        `inflate` multiplies the standard deviations, and `min_sigma_m` floors
        them. Both exist for the learned prior: a network trained on one arena
        will happily produce a very peaked heatmap, and a prior that confident
        would take several genuine views to argue down. Floored at 0.10 m it
        guides the search without ever beating the geometry.
        """
        if inflate <= 0.0:
            raise ValueError('inflate must be positive')
        matrix = np.asarray(covariance, dtype=float).reshape(2, 2)
        matrix = 0.5 * (matrix + matrix.T)  # enforce symmetry
        eigenvalues, eigenvectors = np.linalg.eigh(matrix)
        sigmas = np.sqrt(np.maximum(eigenvalues, 0.0)) * inflate
        sigmas = np.maximum(sigmas, float(min_sigma_m))
        information = eigenvectors @ np.diag(1.0 / sigmas ** 2) @ eigenvectors.T
        return cls(
            mean=(float(mean[0]), float(mean[1])),
            information=(
                float(information[0, 0]), float(information[0, 1]),
                float(information[1, 0]), float(information[1, 1])),
            source=source,
        )

    def as_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        return (
            np.asarray(self.information, dtype=float).reshape(2, 2),
            np.asarray(self.mean, dtype=float),
        )

    @property
    def sigma_major_m(self) -> float:
        matrix, _ = self.as_matrices()
        smallest = float(np.linalg.eigvalsh(matrix)[0])
        return math.inf if smallest <= 0.0 else math.sqrt(1.0 / smallest)


@dataclass(frozen=True)
class View:
    """One sight line in the map frame, flattened to the floor plane.

    The flattening is deliberate: the robot drives on a plane, the map is 2-D,
    and carrying a height through the fusion would add a dimension that only
    one of the three range sources can constrain.
    """

    origin: tuple[float, float]
    direction: tuple[float, float]
    bearing_sigma_rad: float
    range_m: float | None = None
    range_sigma_m: float | None = None
    source: str = 'bearing'
    confidence: float = 1.0
    stamp_sec: float = 0.0

    def line_angle_rad(self) -> float:
        """Heading of the sight *line*, folded into [0, pi).

        A ray and its reverse constrain the same line, so for judging whether
        two views are usefully different, 10 degrees and 190 degrees are the
        same look and must not be counted as a wide baseline.
        """
        return math.atan2(self.direction[1], self.direction[0]) % math.pi


@dataclass(frozen=True)
class LandmarkEstimate:
    """Where the landmark is, how well that is known, and what would help."""

    name: str
    x: float
    y: float
    covariance: tuple[float, float, float, float]
    sigma_major_m: float
    sigma_minor_m: float
    major_axis_rad: float
    view_count: int
    used_count: int
    rejected_count: int
    angular_spread_rad: float
    sources: tuple[str, ...]
    converged: bool
    reason: str
    best_next_view_rad: float
    has_prior: bool = False
    prior_source: str = ''

    @property
    def position(self) -> tuple[float, float]:
        return (self.x, self.y)

    @property
    def biased_by_wall_casts(self) -> bool:
        """Wall casts read the landmark as standing *at* the wall behind it.

        No amount of averaging removes that offset, so a run whose range came
        only from wall casts has a floor on its accuracy no spread figure
        reveals. Callers should say so out loud.
        """
        return 'wall' in self.sources and not (
            {'depth', 'triangulate'} & set(self.sources))

    def as_dict(self) -> dict:
        return {
            'landmark': self.name,
            'position': {'x': round(self.x, 4), 'y': round(self.y, 4)},
            'sigma_major_m': round(self.sigma_major_m, 4),
            'sigma_minor_m': round(self.sigma_minor_m, 4),
            'major_axis_deg': round(math.degrees(self.major_axis_rad), 2),
            'views': self.view_count,
            'views_used': self.used_count,
            'views_rejected': self.rejected_count,
            'angular_spread_deg': round(
                math.degrees(self.angular_spread_rad), 2),
            'sources': list(self.sources),
            'converged': self.converged,
            'reason': self.reason,
            'best_next_view_deg': round(
                math.degrees(self.best_next_view_rad), 2),
            'wall_cast_bias': self.biased_by_wall_casts,
            'prior': self.prior_source if self.has_prior else None,
        }


def solve_landmark(
    name: str,
    views: Sequence[View],
    *,
    nominal_range_m: float = 2.0,
    max_sigma_m: float = 0.25,
    gate_sigmas: float = 3.0,
    iterations: int = 3,
    min_views_to_gate: int = 4,
    confidence_weighting: bool = True,
    prior: Prior | None = None,
) -> LandmarkEstimate | None:
    """Fuse views and any prior into a position and covariance.

    Returns None only when there is nothing at all to go on -- no views and no
    prior. A prior on its own yields an estimate the robot can drive to, which
    is the point of having one, but it is never reported as converged: see
    `Prior` for why that separation matters.

    The solve is iterated because the weight on a bearing depends on the range
    to the landmark, which is what is being solved for. Three passes is ample:
    the weights move the answer by centimetres after the first, and a fixed
    small count keeps this bounded for a node that re-solves every detection.

    Outliers are gated rather than trusted. A single mislabelled box -- a
    bucket read as the dispenser -- would otherwise drag the estimate metres
    away and, worse, shrink the reported covariance while doing it. Gating
    only starts once there are enough views to tell which one is the odd one
    out; with three views, the majority is not yet a majority.
    """
    if not views and prior is None:
        return None

    guess = _initial_guess(views, nominal_range_m, prior)
    used: list[bool] = [True] * len(views)
    information, vector = _seed(prior)

    for iteration in range(max(1, iterations)):
        gating = iteration > 0 and len(views) >= min_views_to_gate
        information, vector = _seed(prior)
        used = []

        for view in views:
            origin = np.asarray(view.origin, dtype=float)
            direction = np.asarray(view.direction, dtype=float)
            normal = np.array([-direction[1], direction[0]])

            offset = guess - origin
            # Predicted range, used both to weight the bearing and to notice a
            # landmark that has ended up behind the camera (a sure sign the
            # view belongs to something else).
            along = float(direction @ offset)
            predicted_range = max(along, _MIN_RANGE_M)

            sigma_perp = view.bearing_sigma_rad * predicted_range
            if confidence_weighting:
                # A low-confidence box tends to be a loose box. Inflating its
                # sigma rather than dropping it keeps the weak evidence, and
                # the cap stops a 0.05-confidence detection from being treated
                # as twenty times worse than a perfect one.
                sigma_perp *= min(1.0 / max(view.confidence, 0.05), 4.0)
            sigma_perp = max(sigma_perp, 1e-4)

            residual = float(normal @ offset)
            keep = True
            if gating:
                behind = along <= 0.0
                keep = not behind and abs(residual) <= gate_sigmas * sigma_perp
            used.append(keep)
            if not keep:
                continue

            weight = 1.0 / (sigma_perp ** 2)
            information += weight * np.outer(normal, normal)
            vector += weight * normal * float(normal @ origin)

            if view.range_m is None:
                continue
            sigma_range = max(float(view.range_sigma_m or 0.10), 1e-3)
            range_residual = along - float(view.range_m)
            if gating and abs(range_residual) > gate_sigmas * sigma_range:
                # The bearing can still be good while the range is not: a
                # depth patch that caught the wall behind the object, say. Use
                # the bearing and drop only the range.
                continue
            range_weight = 1.0 / (sigma_range ** 2)
            information += range_weight * np.outer(direction, direction)
            vector += range_weight * direction * (
                float(direction @ origin) + float(view.range_m))

        solution = _solve(information, vector)
        if solution is None:
            break
        guess = solution

    return _finalise(
        name, views, used, information, guess, max_sigma_m, prior)


def _seed(prior: Prior | None) -> tuple[np.ndarray, np.ndarray]:
    """Start the normal equations from the prior, or from nothing."""
    if prior is None:
        return np.zeros((2, 2)), np.zeros(2)
    information, mean = prior.as_matrices()
    return information.copy(), information @ mean


def _initial_guess(
    views: Sequence[View],
    nominal_range_m: float,
    prior: Prior | None = None,
) -> np.ndarray:
    """Somewhere plausible to start the reweighting from.

    A ranged view says outright where the landmark is, so use those when there
    are any. Otherwise put it a nominal distance down the sight lines: wrong,
    but wrong in a way the first reweighted solve fixes, and it keeps the
    bearing weights finite on the first pass. With no views at all the prior's
    mean is the answer.
    """
    if not views:
        return np.asarray(prior.mean, dtype=float)  # type: ignore[union-attr]
    ranged = [v for v in views if v.range_m is not None]
    source = ranged or views
    points = [
        np.asarray(v.origin, dtype=float)
        + np.asarray(v.direction, dtype=float)
        * (v.range_m if v.range_m is not None else nominal_range_m)
        for v in source
    ]
    return np.mean(points, axis=0)


def _solve(information: np.ndarray, vector: np.ndarray) -> np.ndarray | None:
    """Solve the normal equations, refusing a direction that is unconstrained.

    A rank-deficient solve is the single-viewpoint case: pseudo-inverting it
    would hand back a confident-looking point sitting at the minimum-norm
    solution, which is nowhere in particular. Floor the eigenvalues instead,
    so the answer stays on the sight line and the covariance reports the
    unconstrained direction as unconstrained.
    """
    eigenvalues, eigenvectors = np.linalg.eigh(information)
    if not np.all(np.isfinite(eigenvalues)):
        return None
    floored = np.maximum(eigenvalues, _MIN_INFORMATION)
    inverse = eigenvectors @ np.diag(1.0 / floored) @ eigenvectors.T
    solution = inverse @ vector
    return solution if np.all(np.isfinite(solution)) else None


def _finalise(
    name: str,
    views: Sequence[View],
    used: Sequence[bool],
    information: np.ndarray,
    guess: np.ndarray,
    max_sigma_m: float,
    prior: Prior | None = None,
) -> LandmarkEstimate:
    eigenvalues, eigenvectors = np.linalg.eigh(information)
    floored = np.maximum(eigenvalues, _MIN_INFORMATION)
    covariance = eigenvectors @ np.diag(1.0 / floored) @ eigenvectors.T

    # eigh returns ascending eigenvalues of the *information*, so the smallest
    # information is the largest uncertainty: that is the major axis.
    sigma_major = float(math.sqrt(1.0 / floored[0]))
    sigma_minor = float(math.sqrt(1.0 / floored[-1]))
    major_axis = float(math.atan2(eigenvectors[1, 0], eigenvectors[0, 0]))

    kept = [view for view, flag in zip(views, used) if flag]
    spread = angular_spread([v.line_angle_rad() for v in kept])
    sources = tuple(sorted({v.source for v in kept}))
    has_range = any(v.range_m is not None for v in kept)

    if not kept and prior is not None:
        reason = (
            f'not seen yet this run; position is the {prior.source} prior '
            f'alone, good to about {prior.sigma_major_m:.2f} m')
    elif not kept:
        reason = 'every view was rejected as an outlier'
    elif len(kept) < 2 and not has_range:
        reason = (
            'one bearing and no range: the landmark is somewhere on this '
            'sight line. Take another view from elsewhere.')
    elif sigma_major > max_sigma_m:
        reason = (
            f'uncertain to {sigma_major:.2f} m along '
            f'{math.degrees(major_axis):.0f} deg; '
            f'baseline so far {math.degrees(spread):.0f} deg')
    else:
        reason = f'held to {sigma_major:.2f} m from {len(kept)} views'

    # A prior never makes a landmark "converged", however tight it looks. It
    # is a belief carried in from elsewhere, and the one thing the robot must
    # not do is treat last run's answer, or a network's guess, as a
    # measurement it made itself.
    converged = bool(
        kept and sigma_major <= max_sigma_m and (has_range or len(kept) >= 2))

    return LandmarkEstimate(
        name=name,
        x=float(guess[0]),
        y=float(guess[1]),
        covariance=(
            float(covariance[0, 0]), float(covariance[0, 1]),
            float(covariance[1, 0]), float(covariance[1, 1])),
        sigma_major_m=sigma_major,
        sigma_minor_m=sigma_minor,
        major_axis_rad=major_axis,
        view_count=len(views),
        used_count=len(kept),
        rejected_count=len(views) - len(kept),
        angular_spread_rad=spread,
        sources=sources,
        converged=converged,
        reason=reason,
        # A view shrinks the major axis fastest when its own perpendicular
        # lies along that axis, which means looking *across* it.
        best_next_view_rad=(major_axis + math.pi / 2.0) % math.pi,
        has_prior=prior is not None,
        prior_source='' if prior is None else prior.source,
    )


def angular_spread(angles: Iterable[float]) -> float:
    """Widest separation among sight lines, in radians, folded into [0, pi/2].

    Line angles live on a circle of period pi, so the distance between two of
    them is at most pi/2: perpendicular lines are as different as lines get.
    This is the number that says whether more views will help -- a spread near
    zero means every view so far was from the same side.
    """
    values = sorted(angle % math.pi for angle in angles)
    if len(values) < 2:
        return 0.0
    widest = 0.0
    for i, first in enumerate(values):
        for second in values[i + 1:]:
            delta = abs(second - first) % math.pi
            widest = max(widest, min(delta, math.pi - delta))
    return widest


class LandmarkTrack:
    """The views held for one landmark, and the estimate they produce.

    Views are kept and the whole set re-solved rather than folded into a
    running filter. With a few hundred views and a 2x2 solve that costs
    nothing, and it avoids the way a recursive filter that has once been
    confidently wrong -- after a mislabelled box, say -- can never recover: a
    batch solve simply gates the bad view out on the next pass.

    Retention is by viewing angle, not by age. The hundredth view from the
    doorway adds almost nothing; the first view from the far side halves the
    error. Capping each angular bucket keeps the diverse views and sheds the
    repetitive ones, so a robot that idles in front of a landmark cannot
    crowd out the geometry it collected earlier.
    """

    def __init__(
        self,
        name: str,
        *,
        views_per_bucket: int = 6,
        bucket_count: int = 18,
        **solver_options,
    ):
        if bucket_count < 2 or views_per_bucket < 1:
            raise ValueError('need at least 2 buckets of 1 view')
        self.name = name
        self._views_per_bucket = views_per_bucket
        self._bucket_count = bucket_count
        self._solver_options = solver_options
        self._buckets: dict[int, list[View]] = {}
        self._estimate: LandmarkEstimate | None = None
        self._prior: Prior | None = None
        self._dirty = True

    def set_prior(self, prior: Prior | None) -> None:
        """Replace the prior. The learned one changes as the map grows."""
        if prior != self._prior:
            self._prior = prior
            self._dirty = True

    @property
    def prior(self) -> Prior | None:
        return self._prior

    def add(self, view: View) -> None:
        # Line angles are periodic in pi, so the first and last buckets meet:
        # two views a fraction of a degree apart either side of zero land in
        # different buckets and both survive. That costs a handful of extra
        # retained views at one angle and nothing else -- the solve weighs
        # information correctly however many views carry it -- so it is not
        # worth an angle-aware eviction pass to avoid.
        index = int(view.line_angle_rad() / math.pi * self._bucket_count)
        bucket = self._buckets.setdefault(
            min(index, self._bucket_count - 1), [])
        bucket.append(view)
        if len(bucket) > self._views_per_bucket:
            # Within one angle the views are near-interchangeable, so keep the
            # ones the detector was most sure of.
            bucket.sort(key=lambda v: v.confidence, reverse=True)
            del bucket[self._views_per_bucket:]
        self._dirty = True

    @property
    def views(self) -> list[View]:
        return [view for bucket in self._buckets.values() for view in bucket]

    def estimate(self) -> LandmarkEstimate | None:
        if self._dirty:
            self._estimate = solve_landmark(
                self.name, self.views, prior=self._prior,
                **self._solver_options)
            self._dirty = False
        return self._estimate

    def __len__(self) -> int:
        return sum(len(bucket) for bucket in self._buckets.values())


def first_occupied_along_ray(
    cells: np.ndarray,
    resolution: float,
    grid_origin: tuple[float, float],
    ray_origin: Sequence[float],
    direction: Sequence[float],
    *,
    skip_m: float = 0.35,
    max_range_m: float = 6.0,
    occupied_threshold: int = 65,
) -> tuple[float, float] | None:
    """March a sight line over an occupancy grid to the first wall.

    This is the third range source, and the one that works from a single view
    with no depth hardware: the lidar never sees a dispenser standing against
    a wall, but it does see the wall, so the bearing can be stopped there.

    It reads the landmark as being *at* the wall, so it is wrong by however
    far the object stands off it. That is a bias, not noise, and no number of
    views removes it -- which is why callers give it a large sigma and stop
    using it as soon as there is a real baseline to triangulate on.

    Unknown cells (-1) are driven through rather than treated as walls:
    unexplored is not occupied, and stopping there during a frontier run would
    plant landmarks on the edge of the explored region.
    """
    rows, columns = cells.shape
    origin_x, origin_y = float(grid_origin[0]), float(grid_origin[1])
    step = max(resolution * 0.5, 1e-3)
    distance = float(skip_m)

    while distance <= float(max_range_m):
        x = float(ray_origin[0]) + float(direction[0]) * distance
        y = float(ray_origin[1]) + float(direction[1]) * distance
        column = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)
        if not (0 <= column < columns and 0 <= row < rows):
            return None
        if cells[row, column] >= occupied_threshold:
            return (x, y)
        distance += step
    return None
