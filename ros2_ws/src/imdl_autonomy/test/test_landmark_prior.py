"""Checks on the prior window geometry and the heatmap-to-prior conversion.

Everything here is a contract between code that runs on the robot and code
that runs on a GPU box days later. A disagreement between them does not
crash: it trains a prior that is quietly offset, or silently swaps the two
landmarks, and the only symptom is a prior that never quite helps. So the
conventions are pinned rather than documented and hoped for.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from imdl_autonomy.landmark_estimation import solve_landmark
from imdl_autonomy.landmark_prior import (
    CHANNEL_NAMES,
    WINDOW_CELLS,
    WINDOW_RESOLUTION_M,
    FilePriorProvider,
    NetworkPriorProvider,
    PriorProvider,
    Window,
    build_channels,
    heatmap_moments,
    heatmap_to_prior,
    pack_channels,
    softmax_2d,
    unpack_channels,
)

GRID_RESOLUTION = 0.05
GRID_ORIGIN = (-5.0, -5.0)


def _grid(rows=200, columns=200, fill=0):
    return np.full((rows, columns), fill, dtype=np.int16)


# ------------------------------------------------------------ window geometry


def test_index_and_map_round_trip():
    window = Window(robot_x=1.3, robot_y=-0.7, robot_yaw=math.radians(37))
    for point in [(1.3, -0.7), (2.5, 1.0), (0.0, 0.0), (-1.2, 2.2)]:
        i, j = window.map_to_index(*point)
        assert window.index_to_map(i, j) == pytest.approx(point, abs=1e-9)


def test_the_robot_sits_at_the_centre_cell():
    window = Window(robot_x=4.0, robot_y=-2.0, robot_yaw=1.0)
    assert window.map_to_index(4.0, -2.0) == pytest.approx(
        (window.half, window.half))


def test_i_runs_forward_and_j_runs_left():
    """The convention the labels and the inference must share."""
    window = Window(robot_x=0.0, robot_y=0.0, robot_yaw=0.0)  # facing +x
    ahead_i, ahead_j = window.map_to_index(1.0, 0.0)
    assert ahead_i > window.half
    assert ahead_j == pytest.approx(window.half)

    left_i, left_j = window.map_to_index(0.0, 1.0)
    assert left_i == pytest.approx(window.half)
    assert left_j > window.half


def test_the_window_is_heading_aligned():
    """The same wall must appear ahead or to the side depending on the yaw.

    This is the whole reason for an egocentric window, so it is the one thing
    most worth pinning: if it failed, the network would be learning in map
    coordinates while believing it was learning relative geometry.
    """
    grid = _grid()
    grid[:, 150:152] = 100  # a wall at map x = +2.5

    facing_wall = build_channels(
        grid, GRID_RESOLUTION, GRID_ORIGIN, Window(0.0, 0.0, 0.0))
    turned_away = build_channels(
        grid, GRID_RESOLUTION, GRID_ORIGIN, Window(0.0, 0.0, math.pi / 2))

    ahead = np.argwhere(facing_wall[0])
    sideways = np.argwhere(turned_away[0])
    assert ahead[:, 0].mean() > WINDOW_CELLS * 0.75      # far forward
    assert sideways[:, 1].mean() < WINDOW_CELLS * 0.25   # off to the right


def test_outside_the_grid_reads_as_unknown_not_free():
    """Unexplored is not open space; learning otherwise invents an arena."""
    channels = build_channels(
        _grid(rows=10, columns=10), GRID_RESOLUTION, (0.0, 0.0),
        Window(0.25, 0.25, 0.0))
    assert channels[1].mean() > 0.9
    assert channels[0].sum() == 0


def test_unknown_cells_are_marked_unknown():
    channels = build_channels(
        _grid(fill=-1), GRID_RESOLUTION, GRID_ORIGIN, Window(0.0, 0.0, 0.0))
    assert channels[1].mean() == pytest.approx(1.0)


def test_free_space_is_neither_occupied_nor_unknown():
    channels = build_channels(
        _grid(fill=0), GRID_RESOLUTION, GRID_ORIGIN, Window(0.0, 0.0, 0.0))
    assert channels[0].sum() == 0
    assert channels[1].sum() == 0
    assert len(channels) == len(CHANNEL_NAMES)


def test_a_cell_below_the_threshold_is_not_occupied():
    channels = build_channels(
        _grid(fill=50), GRID_RESOLUTION, GRID_ORIGIN, Window(0.0, 0.0, 0.0),
        occupied_threshold=65)
    assert channels[0].sum() == 0


def test_a_malformed_grid_is_refused():
    with pytest.raises(ValueError):
        build_channels(
            np.zeros(10, dtype=np.int16), GRID_RESOLUTION, GRID_ORIGIN,
            Window(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        build_channels(_grid(), 0.0, GRID_ORIGIN, Window(0.0, 0.0, 0.0))


def test_channels_survive_being_packed_for_storage():
    grid = _grid()
    grid[:, 150:152] = 100
    grid[0:40, :] = -1
    channels = build_channels(
        grid, GRID_RESOLUTION, GRID_ORIGIN, Window(0.1, 0.2, 0.3))
    restored = unpack_channels(pack_channels(channels))
    assert restored.dtype == np.float32
    assert np.array_equal(restored, channels)


# ------------------------------------------------------- heatmap conversion


def _peak_at(window, point, cells=WINDOW_CELLS):
    i, j = window.map_to_index(*point)
    logits = np.full((cells, cells), -20.0)
    logits[int(round(i)), int(round(j))] = 20.0
    return softmax_2d(logits)


def test_a_peaked_heatmap_becomes_a_prior_in_the_right_place():
    window = Window(robot_x=0.5, robot_y=0.25, robot_yaw=math.radians(20))
    target = (2.5, 1.0)
    prior = heatmap_to_prior(
        _peak_at(window, target), window, inflate=1.0, min_sigma_m=0.05)
    assert prior.mean == pytest.approx(target, abs=0.05)
    assert prior.sigma_major_m == pytest.approx(0.05)
    assert prior.source == 'network'


def test_a_flat_heatmap_becomes_a_useless_prior():
    """An unsure network must produce a weak prior without being thresholded."""
    window = Window(0.0, 0.0, 0.0)
    prior = heatmap_to_prior(
        np.ones((WINDOW_CELLS, WINDOW_CELLS)), window,
        inflate=1.0, min_sigma_m=0.05)
    assert prior.sigma_major_m > 1.5


def test_a_weak_prior_cannot_move_a_measured_landmark():
    """The end-to-end safety property, stated as a test."""
    window = Window(0.0, 0.0, 0.0)
    flat = heatmap_to_prior(
        np.ones((WINDOW_CELLS, WINDOW_CELLS)), window, inflate=1.0)

    def view(origin, target=(2.0, 0.0)):
        from imdl_autonomy.landmark_estimation import View
        dx, dy = target[0] - origin[0], target[1] - origin[1]
        norm = math.hypot(dx, dy)
        return View(
            origin=origin, direction=(dx / norm, dy / norm),
            bearing_sigma_rad=math.radians(3.0))

    views = [view((0.0, 0.0)), view((2.0, -2.0))]
    with_prior = solve_landmark('d', views, prior=flat)
    without = solve_landmark('d', views)
    assert with_prior.x == pytest.approx(without.x, abs=0.02)
    assert with_prior.y == pytest.approx(without.y, abs=0.02)


def test_an_elongated_heatmap_keeps_its_direction_through_the_rotation():
    """'Somewhere along this wall' has to survive into the map frame."""
    yaw = math.radians(20)
    window = Window(robot_x=0.5, robot_y=0.25, robot_yaw=yaw)
    heatmap = np.zeros((WINDOW_CELLS, WINDOW_CELLS))
    heatmap[40:90, 63:65] = 1.0  # long along i, the robot's forward axis

    prior = heatmap_to_prior(
        heatmap, window, inflate=1.0, min_sigma_m=0.01)
    information = np.asarray(prior.information).reshape(2, 2)
    eigenvalues, eigenvectors = np.linalg.eigh(information)
    sigmas = sorted(1.0 / np.sqrt(eigenvalues))
    assert sigmas[1] > 5 * sigmas[0]

    major = eigenvectors[:, 0]  # least information = longest axis
    heading = math.degrees(math.atan2(major[1], major[0])) % 180.0
    assert heading == pytest.approx(math.degrees(yaw), abs=2.0)


def test_heatmap_moments_rejects_nonsense():
    with pytest.raises(ValueError):
        heatmap_moments(np.zeros((4, 4)))
    with pytest.raises(ValueError):
        heatmap_moments(np.full((4, 4), -1.0))
    with pytest.raises(ValueError):
        heatmap_moments(np.zeros(4))


def test_softmax_sums_to_one_over_both_axes():
    values = softmax_2d(np.random.default_rng(0).normal(size=(16, 16)) * 5)
    assert values.sum() == pytest.approx(1.0)
    assert (values >= 0).all()


# ------------------------------------------------------------------ providers


def test_the_default_provider_offers_nothing():
    provider = PriorProvider()
    assert provider.priors(['a'], Window(0, 0, 0), np.zeros((2, 4, 4))) == {}


def test_the_file_provider_reads_the_mappers_own_output():
    """It consumes exactly the document landmark_mapper writes."""
    document = {
        'frame_id': 'map',
        'landmarks': {
            'food_dispenser': {'position': {'x': 2.5, 'y': 1.0}},
            'cow_enclosure': {'position': {'x': -1.0, 'y': 0.5}},
        },
    }
    provider = FilePriorProvider(document, sigma_m=0.4)
    priors = provider.priors(
        ['food_dispenser'], Window(0, 0, 0), np.zeros((2, 4, 4)))
    assert set(priors) == {'food_dispenser'}   # the unrequested one is dropped
    assert priors['food_dispenser'].mean == pytest.approx((2.5, 1.0))
    assert priors['food_dispenser'].sigma_major_m == pytest.approx(0.4)
    assert priors['food_dispenser'].source == 'file'


def test_the_file_provider_tolerates_a_run_that_localised_nothing():
    assert FilePriorProvider({'landmarks': {}}).priors(
        ['a'], Window(0, 0, 0), np.zeros((2, 4, 4))) == {}
    assert FilePriorProvider({}).priors(
        ['a'], Window(0, 0, 0), np.zeros((2, 4, 4))) == {}


def test_the_file_provider_refuses_a_zero_sigma():
    with pytest.raises(ValueError):
        FilePriorProvider({'landmarks': {}}, sigma_m=0.0)


def test_a_missing_network_is_reported_as_such():
    provider = NetworkPriorProvider('/nonexistent/prior.onnx', ['a', 'b'])
    with pytest.raises(FileNotFoundError):
        provider.load()


def test_the_network_provider_insists_on_a_class_order():
    """Output channels are positional, so a wrong order swaps the landmarks."""
    provider = NetworkPriorProvider(__file__, [])
    with pytest.raises(ValueError, match='class order'):
        provider.load()


def test_an_unloaded_network_provider_yields_nothing_rather_than_crashing():
    provider = NetworkPriorProvider('/nonexistent/prior.onnx', ['a'])
    assert provider.priors(['a'], Window(0, 0, 0), np.zeros((2, 4, 4))) == {}


def test_the_window_constants_are_what_the_docs_promise():
    """Changing these invalidates every recorded sample and trained model."""
    assert WINDOW_CELLS == 128
    assert WINDOW_RESOLUTION_M == 0.05
    assert Window(0, 0, 0).extent_m == pytest.approx(6.4)


def test_the_occupied_threshold_is_part_of_the_frozen_contract():
    """Not a tunable: it defines the input every sample was recorded with."""
    from imdl_autonomy.landmark_prior import OCCUPIED_THRESHOLD
    assert OCCUPIED_THRESHOLD == 65
    mapper = (
        Path(__file__).parents[1] / 'imdl_autonomy' / 'landmark_mapper.py'
    ).read_text(encoding='utf-8')
    # The network's window must not borrow the wall cast's tunable threshold.
    start = mapper.index('def _channels')
    assert 'wall_cast_occupied_threshold' not in mapper[start:start + 900]
