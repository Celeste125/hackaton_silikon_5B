"""Camera-verified object centering through bounded Nav2 Spin actions."""

from __future__ import annotations

from enum import Enum
from statistics import median

from .navigation_adapter import NavigationState


class VisualAlignmentStatus(str, Enum):
    """Lifecycle result of one image-centering operation."""

    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


class VisualAlignment:
    """Require fresh centered camera frames before manipulation.

    Solvers remain independent of raw velocity commands. Corrections are sent
    through Nav2's collision-checked Spin action and then verified in the next
    camera frames.
    """

    def __init__(self, context):
        """Create an alignment helper bound to one mission context."""
        self.context = context
        self.status = VisualAlignmentStatus.SUCCEEDED
        self.state = 'IDLE'
        self.message = ''
        self.color = ''
        self.started = 0.0
        self.verify_after = 0.0
        self.attempt = 0
        self.centered_samples = 0
        self.last_generation = -1
        self.last_sensor_stamp = None
        self.sensor_not_before_sec = None
        self.samples = []
        self.centered_observation = None
        self.error_rad = 0.0

    def start(self, color: str) -> VisualAlignmentStatus:
        """Start fresh-frame verification for one object color."""
        self.status = VisualAlignmentStatus.RUNNING
        self.state = 'VERIFYING_IMAGE_CENTER'
        self.message = 'waiting for a fresh camera observation'
        self.color = str(color).strip().lower()
        self.started = self.context.monotonic()
        self.verify_after = self.started
        self.attempt = 0
        self.centered_samples = 0
        self.samples = []
        self.centered_observation = None
        self.sensor_not_before_sec = self._ros_now_sec()
        # The image already present when alignment starts is a baseline, not
        # evidence. Every required sample must arrive after this operation was
        # requested; otherwise one recent pre-alignment frame could count as
        # the first of the three independent confirmations.
        self._refresh_camera_baseline()
        self.error_rad = 0.0
        return self.status

    def tick(self) -> VisualAlignmentStatus:
        """Advance the non-blocking verification and Spin state machine."""
        if self.status is not VisualAlignmentStatus.RUNNING:
            return self.status
        if self.context.monotonic() - self.started > float(
            self.context.camera_center_timeout_sec
        ):
            return self._fail(
                'camera centering timed out; last error='
                f'{self.error_rad:.3f}rad'
            )

        if self.state == 'SPINNING_TO_IMAGE_CENTER':
            result = self.context.navigation.tick()
            if result.state is NavigationState.SUCCEEDED:
                # Frames received while the robot was turning describe an
                # intermediate view. Exclude all of them before collecting the
                # three final centered samples.
                self._refresh_camera_baseline()
                self.sensor_not_before_sec = self._ros_now_sec()
                self.samples = []
                self.state = 'VERIFYING_IMAGE_CENTER'
                self.verify_after = (
                    self.context.monotonic()
                    + float(self.context.camera_center_settle_sec)
                )
            elif result.terminal:
                return self._fail(
                    'Nav2 camera-centering Spin failed: '
                    f'{result.message}'
                )
        elif (
            self.state == 'VERIFYING_IMAGE_CENTER'
            and self.context.monotonic() >= self.verify_after
        ):
            self._verify_or_correct()
        return self.status

    def cancel(self) -> None:
        """Cancel an active Nav2 correction and fail this alignment."""
        if self.status is VisualAlignmentStatus.RUNNING:
            self.context.navigation.cancel()
            self._fail('camera centering cancelled')

    def _verify_or_correct(self) -> VisualAlignmentStatus:
        observation = self.context.camera.observation(
            self.color,
            max_age_sec=float(self.context.camera_observation_max_age_sec),
        )
        if observation is None:
            self.message = f'waiting for fresh {self.color} pixels'
            return self.status
        if observation.generation == self.last_generation:
            return self.status
        sensor_stamp = getattr(observation, 'stamp_sec', None)
        if (
            sensor_stamp is not None
            and self.sensor_not_before_sec is not None
            and float(sensor_stamp) + 1e-6 < self.sensor_not_before_sec
        ):
            self.message = 'waiting for a post-motion camera frame'
            return self.status
        if (
            sensor_stamp is not None
            and self.last_sensor_stamp is not None
            and float(sensor_stamp) <= self.last_sensor_stamp
        ):
            self.message = 'ignoring delayed or reordered camera frame'
            return self.status
        self.last_generation = observation.generation
        if sensor_stamp is not None:
            self.last_sensor_stamp = float(sensor_stamp)
        self.samples.append(observation)
        required = max(1, int(self.context.camera_center_samples))
        if len(self.samples) < required:
            self.message = (
                f'collecting fresh image samples {len(self.samples)}/'
                f'{required}'
            )
            return self.status

        sample_batch = self.samples[-required:]
        errors = [
            float(sample.horizontal_error_rad)
            for sample in sample_batch
        ]
        self.error_rad = float(median(errors))
        tolerance = float(self.context.camera_center_tolerance_rad)

        if all(abs(error) <= tolerance for error in errors):
            self.centered_samples = required
            observation = sample_batch[-1]
            self.message = (
                f'{required}/{required} centered samples: '
                f'pixel={observation.cx:.1f}, optical_center='
                f'{observation.principal_x:.1f}, median_error='
                f'{self.error_rad:.3f}rad'
            )
            self.centered_observation = observation
            self.status = VisualAlignmentStatus.SUCCEEDED
            self.state = 'BLOCK_CENTERED'
            self.message = f'BLOCK_CENTERED; {self.message}'
            self.context.logger.info(
                f'visual_alignment[{self.color}]: {self.message}'
            )
            return self.status

        self.centered_samples = 0
        max_attempts = max(1, int(self.context.camera_center_max_corrections))
        if self.attempt >= max_attempts:
            return self._fail(
                f'block remains off-center by {self.error_rad:.3f}rad after '
                f'{self.attempt} correction(s)'
            )
        navigation = self.context.navigation
        if navigation.snapshot().terminal:
            navigation.reset()
        max_step = max(0.05, float(self.context.camera_center_max_step_rad))
        gain = max(0.10, min(1.0, float(
            self.context.camera_center_gain
        )))
        correction = max(
            -max_step,
            min(max_step, gain * self.error_rad),
        )
        timeout = max(6.0, abs(correction) / 0.30 + 3.0)
        if not navigation.spin(correction, timeout_sec=timeout):
            return self._fail(navigation.message)
        self.attempt += 1
        self.samples = []
        self.state = 'SPINNING_TO_IMAGE_CENTER'
        self.message = (
            f'median image error={self.error_rad:.3f}rad; correcting '
            f'{correction:.3f}rad ({self.attempt}/{max_attempts})'
        )
        self.context.logger.info(
            f'visual_alignment[{self.color}]: {self.message}'
        )
        return self.status

    def _refresh_camera_baseline(self) -> None:
        """Exclude every observation already delivered at this instant."""
        baseline = self.context.camera.observation(
            self.color,
            max_age_sec=float(self.context.camera_observation_max_age_sec),
        )
        if baseline is None:
            return
        self.last_generation = max(
            self.last_generation,
            int(baseline.generation),
        )
        sensor_stamp = getattr(baseline, 'stamp_sec', None)
        if sensor_stamp is not None:
            sensor_stamp = float(sensor_stamp)
            if (
                self.last_sensor_stamp is None
                or sensor_stamp > self.last_sensor_stamp
            ):
                self.last_sensor_stamp = sensor_stamp

    def _ros_now_sec(self):
        """Return ROS time for image-stamp gating when a node is available."""
        node = getattr(self.context, 'node', None)
        if node is None:
            return None
        try:
            return float(node.get_clock().now().nanoseconds) * 1e-9
        except (AttributeError, RuntimeError):
            return None

    def _fail(self, message: str) -> VisualAlignmentStatus:
        self.status = VisualAlignmentStatus.FAILED
        self.state = 'CENTERING_FAILED'
        self.message = message
        return self.status
