"""Checks on the multi-view landmark fusion.

The claim this module makes is that an estimate gets better as the robot sees
a landmark from more directions, and honestly worse-reported when it does
not. That is the whole value of it over averaging, and none of it needs a
robot to verify, so it is pinned here.
"""

import math

import pytest

from imdl_autonomy.landmark_estimation import (
    LandmarkTrack,
    Prior,
    View,
    angular_spread,
    first_occupied_along_ray,
    solve_landmark,
)
import numpy as np

TRUTH = (2.0, 0.0)


def view(origin, target=TRUTH, *, sigma_deg=3.0, **kwargs):
    """A sight line from origin toward target, as the mapper would build it."""
    dx, dy = target[0] - origin[0], target[1] - origin[1]
    norm = math.hypot(dx, dy)
    return View(
        origin=tuple(origin),
        direction=(dx / norm, dy / norm),
        bearing_sigma_rad=math.radians(sigma_deg),
        **kwargs,
    )


def test_perpendicular_views_pin_both_axes():
    estimate = solve_landmark('d', [view((0.0, 0.0)), view((2.0, -2.0))])
    assert (estimate.x, estimate.y) == pytest.approx(TRUTH, abs=1e-6)
    # Orthogonal sight lines constrain both directions equally well.
    assert estimate.sigma_major_m == pytest.approx(
        estimate.sigma_minor_m, rel=0.01)
    assert estimate.converged


def test_a_short_baseline_is_reported_as_uncertain_not_averaged():
    """The failure a plain mean hides.

    Two views from 10 cm apart put the landmark in the right place along the
    bearing and nowhere in particular along the line of sight. Averaging the
    two deprojected points would look confident; this must not.
    """
    estimate = solve_landmark('d', [view((0.0, 0.0)), view((0.0, 0.1))])
    assert not estimate.converged
    assert estimate.sigma_major_m > 1.0
    assert estimate.sigma_minor_m < 0.2
    # The unconstrained direction is along the sight line, which is +x here.
    assert math.cos(estimate.major_axis_rad) ** 2 > 0.9


def test_one_bearing_alone_localises_nothing():
    estimate = solve_landmark('d', [view((0.0, 0.0))])
    assert not estimate.converged
    assert estimate.sigma_major_m >= 10.0
    assert 'sight line' in estimate.reason


def test_one_view_with_depth_is_enough():
    """An RGB-D view carries its own range, so it needs no second viewpoint."""
    estimate = solve_landmark('d', [
        view((0.0, 0.0), range_m=2.0, range_sigma_m=0.05, source='depth'),
    ])
    assert (estimate.x, estimate.y) == pytest.approx(TRUTH, abs=1e-6)
    assert estimate.converged
    assert estimate.sources == ('depth',)


def test_views_from_new_angles_shrink_the_estimate():
    """The property the whole design exists for."""
    def arc(count):
        return [
            view((TRUTH[0] + 2.0 * math.cos(a), TRUTH[1] + 2.0 * math.sin(a)))
            for a in np.linspace(-1.2, 1.2, count)
        ]

    sigmas = [solve_landmark('d', arc(n)).sigma_major_m for n in (3, 6, 12)]
    assert sigmas[0] > sigmas[1] > sigmas[2]
    # Information adds, so sigma falls roughly as 1/sqrt(views): four times
    # the views should be clearly better than half as good.
    assert sigmas[2] < sigmas[0] * 0.6


def test_repeated_views_from_one_angle_barely_help():
    """Staring at a landmark is not the same as walking around it."""
    from_one_spot = solve_landmark('d', [view((0.0, 0.02 * i)) for i in range(12)])
    from_two_angles = solve_landmark('d', [view((0.0, 0.0)), view((2.0, -2.0))])
    assert from_one_spot.sigma_major_m > from_two_angles.sigma_major_m
    assert not from_one_spot.converged


def test_opposite_views_along_one_line_do_not_triangulate():
    """Looking at a landmark from either side is still looking along one line.

    A naive baseline measure -- distance between camera positions -- would
    call this a 4 m baseline and be badly wrong.
    """
    estimate = solve_landmark(
        'd', [view((0.0, 0.0)), view((4.0, 0.0))])
    assert estimate.angular_spread_rad == pytest.approx(0.0, abs=1e-6)
    assert not estimate.converged


def test_a_mislabelled_box_is_gated_out():
    views = [
        view((TRUTH[0] + 2.0 * math.cos(a), TRUTH[1] + 2.0 * math.sin(a)))
        for a in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
    ]
    views.append(view((0.0, 0.0), target=(5.0, 4.0)))  # something else entirely
    estimate = solve_landmark('d', views)
    assert estimate.rejected_count == 1
    assert (estimate.x, estimate.y) == pytest.approx(TRUTH, abs=0.05)


def test_a_bad_range_does_not_cost_the_good_bearing():
    """Depth that caught the wall behind the object still gives a bearing."""
    views = [
        view((0.0, 0.0), range_m=2.0, range_sigma_m=0.05, source='depth'),
        view((2.0, -2.0), range_m=2.0, range_sigma_m=0.05, source='depth'),
        view((0.0, -2.0), range_m=2.0, range_sigma_m=0.05, source='depth'),
        # Same sight line as the first, but ranged a metre past the object.
        view((0.0, 0.0), range_m=3.0, range_sigma_m=0.05, source='depth'),
    ]
    estimate = solve_landmark('d', views)
    assert estimate.rejected_count == 0  # the bearing was kept
    assert (estimate.x, estimate.y) == pytest.approx(TRUTH, abs=0.1)


def test_wall_cast_only_runs_are_flagged_as_biased():
    only_walls = solve_landmark('d', [
        view((0.0, 0.0), range_m=2.0, range_sigma_m=0.35, source='wall'),
        view((2.0, -2.0), range_m=2.0, range_sigma_m=0.35, source='wall'),
    ])
    assert only_walls.biased_by_wall_casts

    mixed = solve_landmark('d', [
        view((0.0, 0.0), range_m=2.0, range_sigma_m=0.35, source='wall'),
        view((2.0, -2.0), range_m=2.0, range_sigma_m=0.05, source='depth'),
    ])
    assert not mixed.biased_by_wall_casts


def test_next_view_points_across_the_long_axis():
    estimate = solve_landmark('d', [view((0.0, 0.0)), view((0.0, 0.3))])
    difference = abs(estimate.best_next_view_rad - estimate.major_axis_rad)
    assert min(difference, math.pi - difference) == pytest.approx(
        math.pi / 2.0, abs=1e-6)


def test_angular_spread_folds_lines_not_rays():
    assert angular_spread([0.0, math.pi]) == pytest.approx(0.0)
    assert angular_spread([0.0, math.pi / 2.0]) == pytest.approx(math.pi / 2.0)
    assert angular_spread([0.1]) == 0.0


def test_no_views_has_no_estimate():
    assert solve_landmark('d', []) is None


# --------------------------------------------------------------------------
# Retention: a parked robot must not be able to crowd out earlier geometry.
# --------------------------------------------------------------------------


def test_the_track_keeps_angular_diversity_under_flooding():
    track = LandmarkTrack('d', views_per_bucket=3, bucket_count=18)
    track.add(view((2.0, -2.0)))                      # the one useful angle
    for i in range(300):
        track.add(view((0.0, 0.001 * i)))             # flood from one spot
    estimate = track.estimate()
    assert estimate.angular_spread_rad == pytest.approx(math.pi / 2.0, abs=0.05)
    assert estimate.converged
    assert len(track) <= 3 * 18


def test_the_track_re_solves_after_a_new_view():
    track = LandmarkTrack('d')
    track.add(view((0.0, 0.0)))
    assert not track.estimate().converged
    track.add(view((2.0, -2.0)))
    assert track.estimate().converged


def test_the_track_prefers_confident_views_within_one_angle():
    """Within one angle the views are interchangeable, so keep the best one."""
    track = LandmarkTrack('d', views_per_bucket=1, bucket_count=18)
    # Both sight lines are within a degree of +x, so they share a bucket.
    track.add(view((0.0, -0.01), confidence=0.3))
    track.add(view((0.0, -0.05), confidence=0.9))
    assert [v.confidence for v in track.views] == [0.9]


# --------------------------------------------------------------------------
# Wall cast: the single-view range source for a camera with no depth.
# --------------------------------------------------------------------------


def _grid_with_wall_at_x(wall_x: float, fill: int):
    cells = np.full((200, 200), fill, dtype=np.int16)
    column = int((wall_x - (-5.0)) / 0.05)
    cells[:, column:column + 2] = 100
    return cells


def test_wall_cast_stops_at_the_first_wall():
    hit = first_occupied_along_ray(
        _grid_with_wall_at_x(2.0, fill=0), 0.05, (-5.0, -5.0),
        (0.0, 0.0), (1.0, 0.0))
    assert hit is not None
    assert hit[0] == pytest.approx(2.0, abs=0.06)
    assert hit[1] == pytest.approx(0.0, abs=1e-9)


def test_wall_cast_drives_through_unexplored_cells():
    """-1 is unknown, not an obstacle. Stopping there during a frontier run
    would plant landmarks along the edge of the explored area."""
    hit = first_occupied_along_ray(
        _grid_with_wall_at_x(2.0, fill=-1), 0.05, (-5.0, -5.0),
        (0.0, 0.0), (1.0, 0.0))
    assert hit is not None
    assert hit[0] == pytest.approx(2.0, abs=0.06)


def test_wall_cast_gives_up_past_its_range():
    cells = _grid_with_wall_at_x(2.0, fill=0)
    assert first_occupied_along_ray(
        cells, 0.05, (-5.0, -5.0), (0.0, 0.0), (-1.0, 0.0)) is None
    assert first_occupied_along_ray(
        cells, 0.05, (-5.0, -5.0), (0.0, 0.0), (1.0, 0.0),
        max_range_m=1.0) is None


def test_wall_cast_skips_the_robots_own_footprint():
    cells = _grid_with_wall_at_x(0.2, fill=0)
    assert first_occupied_along_ray(
        cells, 0.05, (-5.0, -5.0), (0.0, 0.0), (1.0, 0.0),
        skip_m=0.35) is None


# --------------------------------------------------------------------------
# Priors: last run's landmarks file, or the learned prior network.
# --------------------------------------------------------------------------


def test_a_prior_alone_gives_a_position_but_never_converges():
    """Enough to drive to, never mistaken for something the robot measured."""
    estimate = solve_landmark(
        'd', [], prior=Prior.isotropic(TRUTH, 0.5, source='last_run'))
    assert (estimate.x, estimate.y) == pytest.approx(TRUTH)
    assert estimate.sigma_major_m == pytest.approx(0.5)
    assert not estimate.converged
    assert estimate.has_prior and estimate.prior_source == 'last_run'
    assert estimate.view_count == 0
    assert 'not seen yet' in estimate.reason


def test_nothing_at_all_still_has_no_estimate():
    assert solve_landmark('d', [], prior=None) is None


def test_a_prior_fills_in_the_direction_one_bearing_cannot():
    """One view plus a prior beats one view alone, which localises nothing."""
    alone = solve_landmark('d', [view((0.0, 0.0))])
    helped = solve_landmark(
        'd', [view((0.0, 0.0))], prior=Prior.isotropic(TRUTH, 0.5))
    assert alone.sigma_major_m > 10.0
    assert helped.sigma_major_m < 0.6
    assert (helped.x, helped.y) == pytest.approx(TRUTH, abs=0.05)


def test_observations_outvote_a_wrong_prior():
    """The dispenser was moved, or the network guessed badly."""
    wrong = Prior.isotropic((0.0, 3.0), 0.5, source='stale')
    views = [view((0.0, 0.0)), view((2.0, -2.0)), view((2.0, 2.0))]
    estimate = solve_landmark('d', views, prior=wrong)
    assert (estimate.x, estimate.y) == pytest.approx(TRUTH, abs=0.15)
    # Still reported as prior-informed, so nobody has to guess why it moved.
    assert estimate.has_prior


def test_a_prior_cannot_be_made_arbitrarily_confident():
    """A network trained on one arena produces very peaked heatmaps."""
    prior = Prior.from_covariance(
        TRUTH, [1e-8, 0.0, 0.0, 1e-8], min_sigma_m=0.10)
    assert prior.sigma_major_m == pytest.approx(0.10)


def test_inflating_a_prior_weakens_it_in_both_axes():
    base = Prior.from_covariance(TRUTH, [0.25, 0.0, 0.0, 0.25])
    loose = Prior.from_covariance(
        TRUTH, [0.25, 0.0, 0.0, 0.25], inflate=2.0)
    assert loose.sigma_major_m == pytest.approx(base.sigma_major_m * 2.0)


def test_an_anisotropic_prior_keeps_its_shape():
    """A heatmap stretched along a corridor must stay stretched."""
    prior = Prior.from_covariance(
        TRUTH, [1.0, 0.0, 0.0, 0.04], min_sigma_m=0.01)
    information, _ = prior.as_matrices()
    sigmas = sorted(1.0 / np.sqrt(np.linalg.eigvalsh(information)))
    assert sigmas == pytest.approx([0.2, 1.0], rel=1e-6)


def test_a_zero_sigma_prior_is_refused():
    with pytest.raises(ValueError):
        Prior.isotropic(TRUTH, 0.0)


def test_the_track_can_have_its_prior_replaced():
    """The learned prior changes every time the map grows."""
    track = LandmarkTrack('d')
    assert track.estimate() is None
    track.set_prior(Prior.isotropic((0.0, 0.0), 1.0))
    first = track.estimate()
    assert first is not None and (first.x, first.y) == pytest.approx((0.0, 0.0))
    track.set_prior(Prior.isotropic(TRUTH, 1.0))
    second = track.estimate()
    assert (second.x, second.y) == pytest.approx(TRUTH)


def test_clearing_the_tracks_prior_restores_the_views_only_answer():
    track = LandmarkTrack('d')
    track.add(view((0.0, 0.0)))
    track.set_prior(Prior.isotropic((0.0, 3.0), 0.3))
    assert track.estimate().sigma_major_m < 1.0
    track.set_prior(None)
    assert track.estimate().sigma_major_m > 10.0
