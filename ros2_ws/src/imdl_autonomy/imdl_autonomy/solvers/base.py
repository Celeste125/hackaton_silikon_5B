"""Non-blocking solver contract used by all autonomy task implementations."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
from typing import Any, Optional


class SolverStatus(str, Enum):
    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class SolverSnapshot:
    status: SolverStatus
    state: str
    message: str = ''
    attempt: int = 1


class BaseSolver:
    """Base class with lifecycle, timeout and transition bookkeeping."""

    def __init__(self, context: Any):
        self.context = context
        self.status = SolverStatus.RUNNING
        self.state = 'CREATED'
        self.message = ''
        self.attempt = 1
        self.started_at: Optional[float] = None
        self.state_started_at: Optional[float] = None
        self._cancelled = False

    def now(self) -> float:
        clock = getattr(self.context, 'monotonic', None)
        return float(clock()) if callable(clock) else time.monotonic()

    def start(self) -> SolverSnapshot:
        if self.started_at is not None:
            return self.snapshot()
        self.started_at = self.now()
        self._transition('STARTING')
        self.on_start()
        return self.snapshot()

    def tick(self) -> SolverSnapshot:
        if self.started_at is None:
            return self.start()
        if self.status is not SolverStatus.RUNNING:
            return self.snapshot()
        if self._cancelled:
            return self._fail('CANCELLED', 'Mission was cancelled')
        if self.elapsed() > self.context.mission.time_limit_sec:
            # Use the solver's complete cancellation path. Task 1 restores its
            # Create3 safety override here, while Tasks 3-5 also invalidate an
            # outstanding manipulation request.
            self.cancel()
            return self._fail('TIMEOUT', 'Mission time limit exceeded')
        self.on_tick()
        return self.snapshot()

    def cancel(self) -> SolverSnapshot:
        self._cancelled = True
        self.context.navigation.cancel()
        manipulator = getattr(self.context, 'manipulator', None)
        if manipulator is not None:
            manipulator.cancel()
        return self.snapshot()

    def on_start(self) -> None:
        raise NotImplementedError

    def on_tick(self) -> None:
        raise NotImplementedError

    def elapsed(self) -> float:
        if self.started_at is None:
            return 0.0
        return max(0.0, self.now() - self.started_at)

    def state_elapsed(self) -> float:
        if self.state_started_at is None:
            return 0.0
        return max(0.0, self.now() - self.state_started_at)

    def snapshot(self) -> SolverSnapshot:
        return SolverSnapshot(
            status=self.status,
            state=self.state,
            message=self.message,
            attempt=self.attempt,
        )

    def _transition(self, state: str, message: str = '') -> SolverSnapshot:
        self.state = state
        self.message = message
        self.state_started_at = self.now()
        visualizer = getattr(self.context, 'visualizer', None)
        if visualizer is not None:
            visualizer.set_solver_state(state, message, self.attempt)
        logger = getattr(self.context, 'logger', None)
        if logger is not None:
            logger.info(f'autonomy state -> {state}: {message}'.rstrip(': '))
        return self.snapshot()

    def _succeed(self, message: str) -> SolverSnapshot:
        self.status = SolverStatus.SUCCEEDED
        return self._transition('SUCCEEDED', message)

    def _fail(self, state: str, message: str) -> SolverSnapshot:
        self.status = SolverStatus.FAILED
        return self._transition(state, message)
