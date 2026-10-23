"""Non-blocking final-yaw verification using Nav2's collision-checked Spin action."""

from __future__ import annotations

from enum import Enum

from .geometry import shortest_angular_distance
from .navigation_adapter import NavigationState


class AlignmentStatus(str, Enum):
    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


class GoalAlignment:
    """Verify a mission yaw and correct it without publishing raw velocity."""

    def __init__(self, context, max_attempts: int = 2):
        self.context = context
        self.max_attempts = max(1, int(max_attempts))
        self.status = AlignmentStatus.SUCCEEDED
        self.state = 'IDLE'
        self.message = ''
        self.target = None
        self.attempt = 0
        self.error_rad = 0.0
        self.verify_after = 0.0

    def start(self, target) -> AlignmentStatus:
        self.target = target
        self.status = AlignmentStatus.RUNNING
        self.state = 'VERIFYING'
        self.message = ''
        self.attempt = 0
        self.verify_after = self.context.monotonic()
        return self._verify_or_correct()

    def tick(self) -> AlignmentStatus:
        if self.status is not AlignmentStatus.RUNNING:
            return self.status
        if self.state == 'SPINNING':
            result = self.context.navigation.tick()
            if result.state is NavigationState.SUCCEEDED:
                self.state = 'VERIFYING'
                self.verify_after = self.context.monotonic() + 0.20
            elif result.terminal:
                self._fail(f'Nav2 Spin failed: {result.message}')
        elif (
            self.state == 'VERIFYING'
            and self.context.monotonic() >= self.verify_after
        ):
            self._verify_or_correct()
        return self.status

    def cancel(self) -> None:
        if self.status is AlignmentStatus.RUNNING:
            self.context.navigation.cancel()
            self._fail('alignment cancelled')

    def _verify_or_correct(self) -> AlignmentStatus:
        current = self.context.current_pose()
        if current is None:
            return self._fail('current map pose is unavailable')
        self.error_rad = shortest_angular_distance(current.yaw, self.target.yaw)
        tolerance = float(self.context.final_yaw_tolerance_rad)
        if abs(self.error_rad) <= tolerance:
            self.status = AlignmentStatus.SUCCEEDED
            self.state = 'ALIGNED'
            self.message = (
                f'yaw error {self.error_rad:.3f} rad <= {tolerance:.3f} rad'
            )
            return self.status
        if self.attempt >= self.max_attempts:
            return self._fail(
                f'yaw error remains {self.error_rad:.3f} rad after '
                f'{self.attempt} correction(s)'
            )

        navigation = self.context.navigation
        if navigation.snapshot().terminal:
            navigation.reset()
        timeout_sec = max(8.0, min(20.0, abs(self.error_rad) / 0.25 + 4.0))
        if not navigation.spin(self.error_rad, timeout_sec=timeout_sec):
            return self._fail(navigation.message)
        self.attempt += 1
        self.state = 'SPINNING'
        self.message = (
            f'correcting yaw by {self.error_rad:.3f} rad '
            f'(attempt {self.attempt}/{self.max_attempts})'
        )
        self.context.logger.info(f'goal_alignment: {self.message}')
        return self.status

    def _fail(self, message: str) -> AlignmentStatus:
        self.status = AlignmentStatus.FAILED
        self.state = 'FAILED'
        self.message = message
        return self.status
