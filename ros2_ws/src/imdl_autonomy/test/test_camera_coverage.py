"""Checks on camera coverage: the wedge, occlusion, and the view planner.

The claim is that a finished lidar sweep does not mean the camera has looked
at the arena, and that this closes the gap. The numbers below are what make
that checkable without a robot: a single view covers a fraction of the floor,
turning in place covers most of it, a wall casts a real shadow, and the
planner drives coverage to the target and then stops.
"""

import math

import numpy as np
import pytest

from imdl_autonomy.camera_coverage import (
    CameraModel,
    GridGeometry,
    ViewGoal,
    classify,
    coverage_fraction,
    has_clearance,
    plan_view,
    remap_seen,
    unseen_clusters,
    visible_mask,
)

RESOLUTION = 0.05
# A 4 m square arena with walls, origin at (-2, -2), like the real one with
# room to spare.
GEOMETRY = GridGeometry(
    resolution_m=RESOLUTION, origin_x=-2.0, origin_y=-2.0,
    width=80, height=80)
CAMERA = CameraModel()


def _arena(extra_walls=()):
    cells = np.zeros((GEOMETRY.height, GEOMETRY.width), dtype=np.int16)
    cells[0:2, :] = 100
    cells[-2:, :] = 100
    cells[:, 0:2] = 100
    cells[:, -2:] = 100
    for rows, columns in extra_walls:
        cells[rows[0]:rows[1], columns[0]:columns[1]] = 100
    return cells


def _world_of(mask):
    rows, columns = np.nonzero(mask)
    return (
        GEOMETRY.origin_x + (columns + 0.5) * RESOLUTION,
        GEOMETRY.origin_y + (rows + 0.5) * RESOLUTION,
    )


# ---------------------------------------------------------------- the wedge


def test_a_camera_sees_a_wedge_not_a_circle():
    """The whole reason this module exists: the lidar sees 360, this does not."""
    occupied, free = classify(_arena())
    seen = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    fraction = coverage_fraction(seen, free)
    # One forward look from the middle of a 4 m arena: well under half.
    assert 0.1 < fraction < 0.4


def test_everything_seen_lies_within_the_field_of_view():
    """Allowing for a cell centre sitting outside the wedge by its own size.

    That slack is real and worst at close range: at the 0.30 m minimum a
    single 5 cm cell subtends nearly 7 degrees.
    """
    occupied, _ = classify(_arena())
    seen = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    x, y = _world_of(seen)
    ranges = np.hypot(x, y)
    bearings = np.abs(np.arctan2(y, x))
    slack = np.arctan2(
        RESOLUTION * math.sqrt(2) / 2.0, np.maximum(ranges, 1e-6))
    assert np.all(bearings <= CAMERA.fov_rad / 2.0 + slack)


def test_nothing_is_seen_beyond_the_useful_range():
    occupied, _ = classify(_arena())
    seen = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    x, y = _world_of(seen)
    assert np.hypot(x, y).max() <= CAMERA.max_range_m + RESOLUTION


def test_the_wedge_follows_the_robot_heading():
    occupied, _ = classify(_arena())
    forward = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    backward = visible_mask(occupied, GEOMETRY, (0.0, 0.0, math.pi), CAMERA)
    assert not (forward & backward).any()
    assert _world_of(forward)[0].mean() > 0.0
    assert _world_of(backward)[0].mean() < 0.0


def test_a_mounting_offset_rotates_the_wedge():
    """So a camera that is not front-facing is still modelled correctly."""
    occupied, _ = classify(_arena())
    straight = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    sideways = visible_mask(
        occupied, GEOMETRY, (0.0, 0.0, 0.0),
        CameraModel(bearing_offset_rad=math.pi / 2.0))
    assert _world_of(straight)[0].mean() > 0.5
    assert _world_of(sideways)[1].mean() > 0.5


def test_turning_in_place_covers_most_of_a_small_arena():
    occupied, free = classify(_arena())
    seen = np.zeros_like(free)
    for index in range(8):
        seen |= visible_mask(
            occupied, GEOMETRY, (0.0, 0.0, index * math.pi / 4.0), CAMERA)
    assert coverage_fraction(seen, free) > 0.9


# ----------------------------------------------------------------- occlusion


def test_a_wall_casts_a_shadow_the_camera_cannot_see_into():
    # A slab at x ~ 0, spanning y in [-0.5, 0.5].
    occupied, _ = classify(_arena([((30, 50), (40, 42))]))
    seen = visible_mask(occupied, GEOMETRY, (-1.0, 0.0, 0.0), CAMERA)
    x, y = _world_of(seen)
    in_shadow = (x > 0.10) & (np.abs(y) < 0.40)
    assert in_shadow.sum() == 0


def test_the_camera_still_sees_around_a_wall_narrower_than_its_view():
    """The field of view is wider than the slab, so the edges get past it."""
    occupied, _ = classify(_arena([((30, 50), (40, 42))]))
    seen = visible_mask(occupied, GEOMETRY, (-1.0, 0.0, 0.0), CAMERA)
    x, y = _world_of(seen)
    assert ((x > 0.10) & (np.abs(y) >= 0.55)).sum() > 0


def test_an_obstacle_reduces_what_is_visible():
    clear, _ = classify(_arena())
    blocked, _ = classify(_arena([((30, 50), (40, 42))]))
    pose = (-1.0, 0.0, 0.0)
    assert (
        visible_mask(blocked, GEOMETRY, pose, CAMERA).sum()
        < visible_mask(clear, GEOMETRY, pose, CAMERA).sum())


# ------------------------------------------------------------------ coverage


def test_unknown_floor_is_not_counted_either_way():
    """Unknown is the lidar explorer's problem; counting it would make the
    target unreachable by construction."""
    cells = _arena()
    cells[20:40, 20:40] = -1
    _, free = classify(cells)
    assert not free[30, 30]


def test_coverage_of_an_empty_map_is_complete_not_zero():
    """Nothing known-free means nothing left to look at, not 0% done."""
    assert coverage_fraction(
        np.zeros((4, 4), bool), np.zeros((4, 4), bool)) == 1.0


def test_unseen_cells_cluster_and_vanish_when_everything_is_seen():
    occupied, free = classify(_arena())
    seen = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    clusters = unseen_clusters(seen, free, GEOMETRY)
    assert clusters and clusters[0][2] > 100
    # Sorted largest first, so the planner looks at the biggest gap.
    assert clusters == sorted(clusters, key=lambda item: item[2], reverse=True)
    assert unseen_clusters(np.ones_like(free), free, GEOMETRY) == []


def test_specks_are_not_worth_a_navigation_goal():
    occupied, free = classify(_arena())
    seen = np.ones_like(free)
    seen[40, 40] = False  # one lonely cell
    assert unseen_clusters(seen, free, GEOMETRY, min_cluster_cells=8) == []


# ----------------------------------------------------------------- clearance


def test_clearance_accepts_open_floor_and_refuses_walls_and_the_unknown():
    occupied, free = classify(_arena())
    assert has_clearance(occupied, free, GEOMETRY, 0.0, 0.0, 0.25)
    assert not has_clearance(occupied, free, GEOMETRY, -1.93, 0.0, 0.25)
    assert not has_clearance(occupied, free, GEOMETRY, 9.0, 9.0, 0.25)

    cells = _arena()
    cells[38:42, 38:42] = -1
    occupied, free = classify(cells)
    # Unexplored is not somewhere to send the robot to stand.
    assert not has_clearance(occupied, free, GEOMETRY, 0.0, 0.0, 0.25)


# -------------------------------------------------------------- view planning


def test_the_planner_prefers_turning_in_place_when_that_is_enough():
    """It costs no travel, and a blob-centred ring alone would never find it."""
    occupied, free = classify(_arena())
    seen = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    goal = plan_view(occupied, free, seen, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    assert isinstance(goal, ViewGoal)
    assert goal.travel_m == pytest.approx(0.0)
    assert goal.new_cells > 0


def test_the_planner_reaches_the_target_and_then_gives_up():
    occupied, free = classify(_arena())
    seen = np.zeros_like(free)
    pose = (0.0, 0.0, 0.0)
    seen |= visible_mask(occupied, GEOMETRY, pose, CAMERA)

    for _ in range(30):
        goal = plan_view(
            occupied, free, seen, GEOMETRY, pose, CAMERA)
        if goal is None:
            break
        pose = (goal.x, goal.y, goal.yaw)
        seen |= visible_mask(occupied, GEOMETRY, pose, CAMERA)
    else:
        pytest.fail('the planner never ran out of views')

    assert coverage_fraction(seen, free) > 0.95


def test_the_planner_has_nothing_to_offer_once_everything_is_seen():
    occupied, free = classify(_arena())
    assert plan_view(
        occupied, free, np.ones_like(free), GEOMETRY, (0.0, 0.0, 0.0),
        CAMERA) is None


def test_a_blacklisted_spot_is_not_proposed_again():
    """Nav2 refusing a pose must not make the planner suggest it for ever."""
    occupied, free = classify(_arena())
    seen = visible_mask(occupied, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    first = plan_view(occupied, free, seen, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    again = plan_view(
        occupied, free, seen, GEOMETRY, (0.0, 0.0, 0.0), CAMERA,
        blacklist=[(first.x, first.y)], blacklist_radius_m=0.35)
    # Turning in place is at the robot's own position, so blacklisting it
    # rules out every heading there; the planner must then drive instead.
    assert again is None or again.travel_m > 0.0


def test_the_planner_will_not_stand_in_a_wall():
    occupied, free = classify(_arena())
    seen = np.ones_like(free)
    seen[2:20, 2:20] = False  # unseen floor tucked into a corner
    goal = plan_view(occupied, free, seen, GEOMETRY, (0.0, 0.0, 0.0), CAMERA)
    if goal is not None and goal.travel_m > 0.0:
        assert has_clearance(occupied, free, GEOMETRY, goal.x, goal.y, 0.25)


# ------------------------------------------------------------- map remapping


def test_coverage_survives_the_map_growing_underneath_it():
    """slam_toolbox enlarges /map as it explores; losing coverage on every
    growth would make the robot re-sweep the same floor for ever."""
    small = GridGeometry(RESOLUTION, -1.0, -1.0, 40, 40)
    seen = np.zeros((40, 40), bool)
    seen[10:20, 10:20] = True
    grown = GridGeometry(RESOLUTION, -2.0, -2.0, 80, 80)

    moved = remap_seen(seen, small, grown)
    assert moved.sum() == seen.sum()
    # The same world point is still marked.
    x, y = small.cell_to_world(15, 15)
    row, column = grown.world_to_cell(x, y)
    assert moved[row, column]


def test_a_resolution_change_resets_rather_than_mis_registering():
    small = GridGeometry(RESOLUTION, -1.0, -1.0, 40, 40)
    seen = np.ones((40, 40), bool)
    assert remap_seen(
        seen, small, GridGeometry(0.02, -1.0, -1.0, 100, 100)).sum() == 0


def test_a_map_that_no_longer_overlaps_resets_cleanly():
    small = GridGeometry(RESOLUTION, -1.0, -1.0, 40, 40)
    seen = np.ones((40, 40), bool)
    assert remap_seen(
        seen, small, GridGeometry(RESOLUTION, 50.0, 50.0, 10, 10)).sum() == 0


# ------------------------------------------------------------------ the model


def test_an_impossible_camera_is_refused():
    for bad in (
        {'fov_rad': 0.0},
        {'fov_rad': 7.0},
        {'min_range_m': 2.0, 'max_range_m': 1.0},
        {'min_range_m': -0.1},
    ):
        with pytest.raises(ValueError):
            CameraModel(**bad)
