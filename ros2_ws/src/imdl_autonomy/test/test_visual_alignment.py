from types import SimpleNamespace

import pytest

from imdl_autonomy.runtime_context import CameraObservation
from imdl_autonomy.visual_alignment import (
    VisualAlignment,
    VisualAlignmentStatus,
)


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


class Navigation:
    def __init__(self):
        self.corrections = []
        self.state = 'IDLE'

    def snapshot(self):
        return SimpleNamespace(terminal=False)

    def spin(self, angle, timeout_sec):
        self.corrections.append((angle, timeout_sec))
        return True


class Logger:
    def info(self, *_args):
        pass


def _context(camera, **overrides):
    values = dict(
        camera=camera,
        navigation=Navigation(),
        logger=Logger(),
        monotonic=Clock(),
        camera_center_timeout_sec=15.0,
        camera_observation_max_age_sec=0.75,
        camera_center_tolerance_rad=0.05,
        camera_center_samples=3,
        camera_center_max_corrections=4,
        camera_center_max_step_rad=0.25,
        camera_center_gain=0.55,
        camera_center_settle_sec=0.55,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_pixel_error_has_ros_yaw_sign():
    left = CameraObservation(
        color='red',
        cx=250.0,
        cy=240.0,
        width=20.0,
        height=20.0,
        principal_x=320.0,
        focal_x=350.0,
        generation=1,
        received_wall=0.0,
        stamp_sec=1.0,
    )
    right = CameraObservation(
        color='red',
        cx=390.0,
        cy=240.0,
        width=20.0,
        height=20.0,
        principal_x=320.0,
        focal_x=350.0,
        generation=1,
        received_wall=0.0,
        stamp_sec=1.0,
    )

    assert left.horizontal_error_rad > 0.0
    assert right.horizontal_error_rad < 0.0
    assert left.horizontal_error_rad == pytest.approx(
        -right.horizontal_error_rad
    )


def test_off_center_frame_starts_bounded_nav2_spin():
    generation = {'value': 0}

    def observation(*_args, **_kwargs):
        generation['value'] += 1
        return SimpleNamespace(
            horizontal_error_rad=0.80,
            generation=generation['value'],
            cx=30.0,
            principal_x=320.0,
        )

    camera = SimpleNamespace(observation=observation)
    context = _context(camera, camera_center_samples=1)
    alignment = VisualAlignment(context)

    status = alignment.start('red')
    alignment.tick()

    assert status is VisualAlignmentStatus.RUNNING
    assert context.navigation.corrections[0][0] == pytest.approx(0.25)


def test_identical_frame_cannot_fake_multiple_center_confirmations():
    observation = SimpleNamespace(
        horizontal_error_rad=0.0,
        generation=7,
        cx=320.0,
        principal_x=320.0,
    )
    camera = SimpleNamespace(observation=lambda *_args, **_kwargs: observation)
    context = _context(camera)
    alignment = VisualAlignment(context)

    assert alignment.start('red') is VisualAlignmentStatus.RUNNING
    for _ in range(10):
        alignment.tick()

    assert alignment.status is VisualAlignmentStatus.RUNNING
    assert alignment.centered_samples == 0


def test_three_fresh_centered_frames_produce_explicit_evidence():
    generation = {'value': 0}

    def observation(*_args, **_kwargs):
        generation['value'] += 1
        return SimpleNamespace(
            horizontal_error_rad=0.01,
            generation=generation['value'],
            cx=316.5,
            principal_x=320.0,
        )

    context = _context(SimpleNamespace(observation=observation))
    alignment = VisualAlignment(context)

    assert alignment.start('red') is VisualAlignmentStatus.RUNNING
    assert alignment.tick() is VisualAlignmentStatus.RUNNING
    assert alignment.tick() is VisualAlignmentStatus.RUNNING
    assert alignment.tick() is VisualAlignmentStatus.SUCCEEDED
    assert alignment.state == 'BLOCK_CENTERED'
    assert alignment.message.startswith('BLOCK_CENTERED;')


def test_new_callback_with_older_sensor_stamp_is_not_centering_evidence():
    observations = iter([
        SimpleNamespace(
            horizontal_error_rad=0.0,
            generation=1,
            stamp_sec=10.0,
            cx=320.0,
            principal_x=320.0,
        ),
        SimpleNamespace(
            horizontal_error_rad=0.0,
            generation=2,
            stamp_sec=9.9,
            cx=320.0,
            principal_x=320.0,
        ),
    ])
    latest = {'value': None}

    def observation(*_args, **_kwargs):
        try:
            latest['value'] = next(observations)
        except StopIteration:
            pass
        return latest['value']

    context = _context(SimpleNamespace(observation=observation))
    alignment = VisualAlignment(context)

    assert alignment.start('red') is VisualAlignmentStatus.RUNNING
    assert alignment.tick() is VisualAlignmentStatus.RUNNING
    assert alignment.centered_samples == 0
    assert alignment.message == 'ignoring delayed or reordered camera frame'


def test_pre_spin_sensor_frame_cannot_drive_post_spin_correction():
    class RosNow:
        def __init__(self):
            self.nanoseconds = 10_000_000_000

    class RosClock:
        def now(self):
            return RosNow()

    node = SimpleNamespace(get_clock=lambda: RosClock())
    observations = iter([
        SimpleNamespace(
            horizontal_error_rad=0.30,
            generation=1,
            stamp_sec=9.8,
            cx=210.0,
            principal_x=320.0,
        ),
        SimpleNamespace(
            horizontal_error_rad=0.30,
            generation=2,
            stamp_sec=9.9,
            cx=210.0,
            principal_x=320.0,
        ),
    ])
    latest = {'value': None}

    def observation(*_args, **_kwargs):
        try:
            latest['value'] = next(observations)
        except StopIteration:
            pass
        return latest['value']

    context = _context(
        SimpleNamespace(observation=observation),
        node=node,
        camera_center_samples=1,
    )
    alignment = VisualAlignment(context)

    assert alignment.start('red') is VisualAlignmentStatus.RUNNING
    assert alignment.tick() is VisualAlignmentStatus.RUNNING
    assert alignment.message == 'waiting for a post-motion camera frame'
    assert context.navigation.corrections == []
