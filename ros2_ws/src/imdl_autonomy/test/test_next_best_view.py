import numpy as np

from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.next_best_view import (
    NextBestViewPlanner,
    OccupancyGrid,
)


def _grid():
    occupied = np.zeros((50, 50), dtype=bool)
    occupied[[0, -1], :] = True
    occupied[:, [0, -1]] = True
    occupied[10:40, 25] = True
    return OccupancyGrid(occupied, 0.02, -0.5, -0.5)


def test_raycast_stops_at_known_wall():
    grid = _grid()

    assert grid.line_of_sight((-0.25, 0.0), (-0.05, 0.0))
    assert not grid.line_of_sight((-0.25, 0.0), (0.25, 0.0))


def test_negative_observation_reduces_visible_belief():
    planner = NextBestViewPlanner(
        _grid(),
        robot_clearance_m=0.08,
        camera_max_range_m=0.6,
    )
    view = Pose2D(-0.25, 0.0, 0.0)
    mask = planner.visible_mask(view)
    before = float(planner.belief[mask].sum())

    planner.observe_negative(view, miss_probability=0.2)

    after = float(planner.belief[mask].sum())
    assert after < before
    assert planner.searched[mask].all()


def test_approach_ring_rejects_wall_side_and_keeps_free_side():
    planner = NextBestViewPlanner(_grid(), robot_clearance_m=0.08)
    approaches = planner.approach_ring(
        Pose2D(-0.35, -0.3, 0.0),
        Pose2D(-0.10, 0.0, 0.0),
        radius_m=0.16,
    )

    assert any(candidate.accepted for candidate in approaches)
    assert any(not candidate.accepted for candidate in approaches)


def test_search_view_margin_keeps_ranked_poses_away_from_walls():
    planner = NextBestViewPlanner(
        _grid(),
        robot_clearance_m=0.08,
        view_clearance_margin_m=0.06,
        camera_max_range_m=0.6,
    )

    candidates = planner.rank_candidates(
        Pose2D(-0.30, -0.30, 0.0),
        limit=20,
    )

    assert candidates
    assert all(
        planner.grid.clearance_at(item.pose.x, item.pose.y) >= 0.14 - 1e-9
        for item in candidates
    )
