"""Exercise physical geometry and motion exits without ROS or a model."""

from dataclasses import replace
import math

import pytest

from imdl_autonomy.object_approach import (
    ApproachController, Config, ExpansionEstimator, Observation, Pose,
)


def observation(stamp, travel=0.0, distance=1.4, bearing=0.0, **kwargs):
    size = 0.25 / (distance - travel)
    width = size * 0.8
    return replace(Observation(
        stamp, 'food_dispenser', 'food_dispenser',
        (0.5-width/2, 0.5-size/2, 0.5+width/2, 0.5+size/2),
        bearing, size, Pose(travel, 0.0, 0.0), 0.9), **kwargs)


def feed(core, stamp, **kwargs):
    o = observation(stamp, **kwargs)
    core.observe([o], stamp, stamp)
    return o


def aligned_controller():
    core = ApproachController()
    feed(core, 1.0)
    for stamp in (1.1, 1.3, 1.5, 1.7, 1.9, 2.1, 2.3, 2.5):
        feed(core, stamp)
        core.command(stamp, Pose(0, 0, 0), True, True)
    assert core.state == 'APPROACHING'
    return core


def test_range_is_metric_without_known_object_size():
    estimator = ExpansionEstimator(Config())
    for i in range(16):
        travel = i * 0.01
        estimator.add(travel, 0.37 / (2.0 - travel))
    estimate = estimator.estimate()
    assert estimate.initial_m == pytest.approx(2.0)
    assert estimate.current_m == pytest.approx(1.85)


@pytest.mark.parametrize('sizes', [
    [0.2] * 12,  # no growth
    [0.2 - i * 0.001 for i in range(12)],  # shrinking
    [0.2, 0.35, 0.19, 0.38, 0.2, 0.3, 0.22, 0.4, 0.3, 0.25, 0.4, 0.3],
])
def test_unobservable_or_inconsistent_scale_has_no_range(sizes):
    estimator = ExpansionEstimator(Config())
    for i, size in enumerate(sizes):
        estimator.add(i * 0.02, size)
    assert estimator.estimate() is None


def test_stationary_frames_do_not_create_a_baseline():
    estimator = ExpansionEstimator(Config())
    for i in range(100):
        estimator.add(0.0, 0.2 + i * 0.001)
    assert len(estimator.samples) == 1
    assert estimator.estimate() is None


# A 2.5 m target, 1% size noise, and only 14.3 cm of baseline: 4.4% growth.
# Small signal, so the scatter about the mean of 1/size is small too, and a
# residual threshold reads that as a clean fit. The answer is out by 38%.
_NOISY_WOBBLE = (1.004, 1.001, 0.991, 1.000, 1.007, 0.987,
                 0.995, 0.981, 0.987, 0.982, 0.998, 0.987)


def _noisy_leg(config):
    estimator = ExpansionEstimator(config)
    for i, jitter in enumerate(_NOISY_WOBBLE):
        travel = i * 0.013
        estimator.add(travel, jitter * 0.25 / (2.5 - travel))
    return estimator


def test_a_tidy_residual_does_not_mean_a_usable_range():
    """Why the gate is propagated sigma and not fit residual.

    These samples are what the old threshold accepted: residual 0.007 against
    a limit of 0.025, reported as a measurement, and wrong by nearly a metre.
    """
    loose = _noisy_leg(Config(max_range_sigma_ratio=0.999)).estimate()
    truth = 2.5 - 11 * 0.013
    assert loose.residual < 0.025  # the old gate passed this ...
    assert abs(loose.current_m - truth) / truth > 0.35  # ... at 38% error
    assert loose.sigma_m > 0.12 * loose.current_m  # sigma knew all along


def test_an_imprecise_leg_is_refused_and_says_so():
    estimator = _noisy_leg(Config())
    assert estimator.growth() > Config().min_growth  # it did grow ...
    assert estimator.estimate() is None  # ... but not enough to measure

    core = aligned_controller()
    core.estimator = estimator
    core.travel = 11 * 0.013
    core._finish_probe(9.0)
    assert core.state == 'STOPPED'
    assert core.retryable  # another viewpoint may do better
    assert 'imprecise' in core.reason


def test_the_probe_drives_until_the_growth_target_not_until_a_fit_appears():
    """Precision comes from baseline, so the leg length is set by geometry."""
    estimator = ExpansionEstimator(Config())
    for i in range(40):
        travel = i * 0.012
        estimator.add(travel, 0.25 / (1.4 - travel))
        if estimator.enough_growth():
            break
    assert estimator.growth() >= 0.20
    # 1.4 m target: 20% growth needs travel = 1.4 * 0.2 / 1.2 m of the probe.
    assert estimator.samples[-1][0] == pytest.approx(1.4 * 0.2 / 1.2, abs=0.012)
    assert estimator.estimate().current_m == pytest.approx(
        1.4 - estimator.samples[-1][0])


def test_first_detection_stops_before_confirmation_or_alignment():
    core = ApproachController()
    feed(core, 1.0, bearing=0.3)
    assert core.state == 'STOPPING'
    for stamp in (1.1, 1.3, 1.5):
        feed(core, stamp, bearing=0.3)
        assert core.command(stamp, Pose(0, 0, 0), True, False) == (0, 0)
    assert core.state == 'STOPPING'


def test_cancellation_ack_alone_does_not_allow_rotation():
    core = ApproachController()
    for stamp in (1.0, 1.2, 1.4, 1.6, 1.8):
        feed(core, stamp, bearing=0.3)
        assert core.command(stamp, Pose(0, 0, 0), False, True) == (0, 0)
    assert core.state == 'STOPPING'


@pytest.mark.parametrize('bearing', [-0.2, 0.2])
def test_alignment_only_turns_in_the_correct_direction(bearing):
    core = ApproachController()
    for stamp in (1.0, 1.2, 1.4, 1.6, 1.8):
        feed(core, stamp, bearing=bearing)
        linear, angular = core.command(stamp, Pose(0, 0, 0), True, True)
    assert linear == 0
    assert math.copysign(1, angular) == math.copysign(1, bearing)


def test_probe_runs_to_the_growth_target_then_reports_the_whole_leg():
    core = aligned_controller()
    for i in range(1, 100):
        travel = i * 0.012
        stamp = 2.5 + i * 0.2
        feed(core, stamp, travel=travel)
        speed, turn = core.command(stamp, Pose(travel, 0, 0), False, True)
        assert turn == 0
        if core.state == 'MEASURED':
            assert speed == 0
            # Not the first travel that could have produced a fit: the leg
            # runs until 20% growth, which at 1.4 m is most of the probe.
            assert travel >= 0.20
            assert core.range.current_m == pytest.approx(1.4 - travel)
            assert core.range.samples == len(core.estimator.samples)
            assert 0 < core.range.sigma_m <= 0.12 * core.range.current_m
            break
        assert speed > 0
    assert core.state == 'MEASURED'
    measured = core.target
    feed(core, stamp + 0.2)
    assert core.command(stamp + 1, Pose(0, 0, 0), True, True) == (0, 0)
    assert core.state == 'MEASURED'
    assert core.target == measured
    with pytest.raises(ValueError, match='save'):
        core.begin_resume(stamp + 1, Pose(travel, 0, 0))
    core.begin_resume(stamp + 1, Pose(travel, 0, 0), saved=True)
    assert core.command(stamp + 1, Pose(travel, 0, 0), True, True) == (0, 0)
    core.finish_resume()
    feed(core, stamp + 2)
    assert core.state == 'EXPLORING'  # same view is still on cooldown
    feed(core, stamp + 3, landmark='cow_enclosure', model_class='cow_stall')
    assert core.state == 'STOPPING'


def test_blocked_probe_defers_target_until_new_viewpoint():
    core = aligned_controller()
    for i in range(1, 25):
        now = 2.5 + i * 0.2
        feed(core, now)
        command = core.command(now, Pose(0, 0, 0), True, True)
        if core.state == 'STOPPED':
            break
    assert command == (0, 0)
    assert core.retryable and 'blocked' in core.reason
    assert core.range is None
    core.begin_resume(now, Pose(0, 0, 0))
    core.finish_resume()
    feed(core, now + 1, pose=Pose(1, 0, 0))
    assert core.state == 'EXPLORING'  # cooldown still active
    feed(core, now + 30)
    assert core.state == 'EXPLORING'  # time alone is not a new view
    feed(core, now + 31, pose=Pose(0.5, 0, 0))
    assert core.state == 'STOPPING'


def test_critical_failure_cannot_resume_automatically():
    core = aligned_controller()
    core.fail('disk write failed')
    with pytest.raises(ValueError, match='operator'):
        core.begin_resume(3.0, Pose(0, 0, 0))


def recorded_controller():
    core = ApproachController()
    core.target = observation(1.0)
    core.state = 'MEASURED'
    core.begin_resume(2.0, Pose(0, 0, 0), saved=True)
    core.finish_resume()
    return core


def test_saved_target_is_measured_again_after_cooldown_and_translation():
    core = recorded_controller()
    feed(core, 3.0, pose=Pose(1, 0, 0))
    assert core.state == 'EXPLORING'  # moved, but too soon
    feed(core, 40.0, pose=Pose(0, 0, math.pi / 2))
    assert core.state == 'EXPLORING'  # rotating in place is not a new viewpoint
    feed(core, 41.0, pose=Pose(0.2, 0, 0))
    assert core.state == 'EXPLORING'
    feed(core, 42.0, pose=Pose(0.6, 0, 0))
    assert core.state == 'STOPPING'
    assert core.target.landmark == 'food_dispenser'


def test_unmeasured_landmark_has_priority_over_remeasurement():
    core = recorded_controller()
    known = observation(40.0, pose=Pose(1, 0, 0), confidence=0.99)
    new = replace(known, landmark='cow_enclosure', model_class='cow_stall', confidence=0.7)
    core.observe([known, new], 40.0, 40.0)
    assert core.target.landmark == 'cow_enclosure'


def test_size_samples_during_rotation_are_not_used_for_range():
    core = aligned_controller()
    assert core.estimator.samples == [(0.0, observation(0).size)]


def test_probe_is_bounded_when_boxes_do_not_grow():
    core = aligned_controller()
    for i in range(1, 23):
        travel = i * 0.012
        stamp = 2.5 + i * 0.2
        feed(core, stamp, pose=Pose(travel, 0, 0))  # box stays fixed
        command = core.command(stamp, Pose(travel, 0, 0), False, True)
    assert core.state == 'STOPPED'
    assert command == (0, 0)
    assert 'size growth' in core.reason


@pytest.mark.parametrize('pose', [Pose(0.1, 0.1, 0), Pose(0.1, 0, 0.2), Pose(-0.1, 0, 0)])
def test_deviation_stops_instead_of_steering_a_curve(pose):
    core = aligned_controller()
    assert core.command(2.6, pose, False, True) == (0, 0)
    assert core.state == 'STOPPED'


@pytest.mark.parametrize('fault', ['empty', 'other_class', 'clipped', 'ambiguous', 'jump'])
def test_target_loss_or_switch_stops(fault):
    core = aligned_controller()
    o = observation(2.7)
    observations = {
        'empty': [], 'other_class': [replace(o, model_class='cow_head')],
        'clipped': [replace(o, box=(0, 0.4, 0.6, 0.8))],
        'ambiguous': [o, o], 'jump': [replace(o, size=o.size * 1.3)],
    }[fault]
    core.observe(observations, 2.7, 2.7)
    assert core.command(2.7, Pose(0, 0, 0), True, True) == (0, 0)
    assert core.state == 'STOPPED'


@pytest.mark.parametrize('ack,sensors,now', [(False, True, 2.6), (True, False, 2.6), (True, True, 4.0)])
def test_missing_heartbeat_or_sensor_stops(ack, sensors, now):
    core = aligned_controller()
    assert core.command(now, Pose(0, 0, 0), False, ack, sensors) == (0, 0)
    assert core.state == 'STOPPED'


def test_delayed_duplicate_and_future_frames_cannot_refresh_target():
    core = aligned_controller()
    for stamp, now in ((2.5, 2.6), (1.0, 2.6), (5.0, 2.6)):
        core.observe([observation(stamp)], stamp, now)
    assert core.last_seen == 2.5
    assert core.command(3.5, Pose(0, 0, 0), True, True) == (0, 0)


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        Config(max_probe_m=0.01)
    with pytest.raises(ValueError):
        Config(approach_speed=float('nan'))
