"""Reusable active visual search state machine for Tasks 4 and 5."""

from __future__ import annotations

from enum import Enum

from ..geometry import shortest_angular_distance
from ..goal_alignment import AlignmentStatus, GoalAlignment
from ..navigation_adapter import NavigationState


class SearchStatus(str, Enum):
    RUNNING = 'RUNNING'
    FOUND = 'FOUND'
    FAILED = 'FAILED'


class ActiveSearch:

    def __init__(
        self,
        context,
        planner,
        timeout_sec=None,
        max_viewpoints=None,
    ):
        self.context = context
        self.planner = planner
        self.status = SearchStatus.RUNNING
        self.state = 'IDLE'
        self.color = ''
        self.message = ''
        self.track = None
        self.candidates = []
        self.active_candidate = None
        self.visited = 0
        self.started = 0.0
        self.state_started = 0.0
        self.future_color_bonus = 0.0
        self.alignment = GoalAlignment(context)
        self.timeout_sec = (
            float(timeout_sec)
            if timeout_sec is not None
            else float(context.search_timeout_sec)
        )
        self.max_viewpoints = (
            int(max_viewpoints)
            if max_viewpoints is not None
            else int(context.max_search_viewpoints)
        )
        self.rejected_positions = []
        self.progress_pose = None
        self.best_distance_remaining = None
        self.last_progress_at = 0.0
        self.last_translation_progress_at = 0.0
        self.sweep_segment = 0
        self.sweep_direction = 1.0
        self.sweep_observations = []

    def start(self, color: str, future_color_bonus: float = 0.0) -> None:
        self.color = color
        self.future_color_bonus = future_color_bonus
        self.started = self.context.monotonic()
        self._transition('INITIAL_OBSERVE')

    def tick(self) -> SearchStatus:
        if self.status is not SearchStatus.RUNNING:
            return self.status
        track = self.context.objects.confirmed(self.color)
        if track is not None:
            # A detection can become confirmed while NavigateToPose or Spin is
            # still active. Cancel it before TransportFlow starts its approach.
            if self.context.navigation.snapshot().state in {
                NavigationState.SENDING,
                NavigationState.ACTIVE,
            }:
                self.context.navigation.cancel()
            self.track = track
            self.status = SearchStatus.FOUND
            self._transition('OBJECT_FOUND', track.object_id)
            return self.status
        if self.context.monotonic() - self.started > self.timeout_sec:
            self._fail('ACTIVE_SEARCH_TIMEOUT')
            return self.status

        if self.state == 'INITIAL_OBSERVE':
            if self._state_elapsed() >= 0.8:
                self._transition('START_INITIAL_SCAN')
        elif self.state == 'START_INITIAL_SCAN':
            self._reset_navigation()
            if self.context.navigation.spin(0.55, timeout_sec=12.0):
                self._transition('WAIT_INITIAL_SCAN')
            else:
                self._transition('PLAN_VIEW')
        elif self.state == 'WAIT_INITIAL_SCAN':
            result = self.context.navigation.tick()
            if result.terminal:
                self._transition('PLAN_VIEW')
        elif self.state == 'PLAN_VIEW':
            self._plan_view()
        elif self.state == 'START_VIEW_PLAN':
            self._start_view_plan()
        elif self.state == 'WAIT_VIEW_PLAN':
            self._wait_view_plan()
        elif self.state == 'START_VIEW_NAV':
            self._start_view_navigation()
        elif self.state == 'WAIT_VIEW_NAV':
            self._wait_view_navigation()
        elif self.state == 'WAIT_VIEW_TERMINAL_STOP':
            self._wait_view_terminal_stop()
        elif self.state == 'ALIGN_VIEW':
            self._align_view()
        elif self.state == 'START_VIEW_SWEEP':
            self._start_view_sweep()
        elif self.state == 'WAIT_VIEW_SWEEP':
            self._wait_view_sweep()
        elif self.state == 'OBSERVE_VIEW':
            self._observe_view()
        return self.status

    def _plan_view(self) -> None:
        current = self.context.current_pose()
        if current is None:
            if self._state_elapsed() > 2.0:
                self._fail('LOCALIZATION_POSE_UNAVAILABLE_DURING_SEARCH')
            return
        ranked = self.planner.rank_candidates(
            current,
            limit=int(getattr(self.context, 'search_candidate_pool', 60)),
            future_color_bonus=self.future_color_bonus,
        )
        rejection_radius = float(
            getattr(self.context, 'search_rejected_radius_m', 0.16)
        )
        self.candidates = [
            candidate for candidate in ranked
            if all(
                candidate.pose.distance_to(rejected) >= rejection_radius
                for rejected in self.rejected_positions
            )
        ]
        if not self.candidates:
            self._fail('NO_REACHABLE_NEXT_BEST_VIEW')
            return
        self.active_candidate = self.candidates[0]
        self.context.visualizer.set_search(
            self.candidates,
            self.active_candidate,
            self.planner.searched_points(),
        )
        self._transition('START_VIEW_PLAN')

    def _start_view_plan(self) -> None:
        self._reset_navigation()
        pose = self.active_candidate.pose
        self.context.visualizer.set_active_goal(pose, 'NBV path preflight')
        timeout = float(
            getattr(self.context, 'search_plan_timeout_sec', 5.0)
        )
        if self.context.navigation.compute_path(pose, timeout_sec=timeout):
            self._transition('WAIT_VIEW_PLAN')
        else:
            self._reject_active_view(self.context.navigation.message)

    def _wait_view_plan(self) -> None:
        result = self.context.navigation.tick()
        if result.state is NavigationState.SUCCEEDED:
            self._transition('START_VIEW_NAV', 'live-costmap path is feasible')
        elif result.terminal:
            self._reject_active_view(f'path preflight failed: {result.message}')

    def _start_view_navigation(self) -> None:
        if self.visited >= self.max_viewpoints:
            self._fail('MAX_SEARCH_VIEWPOINTS_REACHED')
            return
        self._reset_navigation()
        pose = self.active_candidate.pose
        self.context.visualizer.set_active_goal(pose, 'RobustMPPI')
        if self.context.navigation.navigate_to(
            pose,
            controller='RobustMPPI',
            timeout_sec=self.context.navigation_timeout_sec,
        ):
            self._reset_view_progress()
            self._transition('WAIT_VIEW_NAV')
        else:
            self._reject_active_view(self.context.navigation.message)

    def _wait_view_navigation(self) -> None:
        result = self.context.navigation.tick()
        if result.state is NavigationState.SUCCEEDED:
            self._start_view_alignment()
        elif result.state in {
            NavigationState.SENDING,
            NavigationState.ACTIVE,
        }:
            current = self.context.current_pose()
            if self._within_view_position_tolerance(current):
                self.context.navigation.cancel()
                error = current.distance_to(self.active_candidate.pose)
                self._transition(
                    'WAIT_VIEW_TERMINAL_STOP',
                    f'accepting search view at {error:.3f}m position error',
                )
            elif self._view_navigation_stalled(current, result):
                self._reject_active_view(
                    'no translational, rotational, or path progress for '
                    f'{self._search_stall_timeout():.1f}s'
                )
        elif result.terminal:
            self._reject_active_view(result.message)

    def _wait_view_terminal_stop(self) -> None:
        settle = float(
            getattr(self.context, 'terminal_stop_settle_sec', 0.30)
        )
        if self._state_elapsed() < settle:
            return
        stationary = getattr(
            self.context.navigation,
            'robot_stationary',
            None,
        )
        if callable(stationary) and not stationary():
            if self._state_elapsed() < settle + 1.0:
                return
            self._reject_active_view(
                'robot did not stop after accepting nearby search view'
            )
            return
        self._start_view_alignment()

    def _start_view_alignment(self) -> None:
        status = self.alignment.start(self.active_candidate.pose)
        if status is AlignmentStatus.SUCCEEDED:
            self._view_reached()
        elif status is AlignmentStatus.FAILED:
            self._reject_active_view(self.alignment.message)
        else:
            self._transition('ALIGN_VIEW', self.alignment.message)

    def _align_view(self) -> None:
        status = self.alignment.tick()
        if status is AlignmentStatus.SUCCEEDED:
            self._view_reached()
        elif status is AlignmentStatus.FAILED:
            self._reject_active_view(self.alignment.message)

    def _view_reached(self) -> None:
        self.visited += 1
        sweep_viewpoints = max(0, int(getattr(
            self.context,
            'search_sweep_viewpoints',
            0,
        )))
        sweep_segments = max(0, int(getattr(
            self.context,
            'search_sweep_segments',
            0,
        )))
        sweep_step = abs(float(getattr(
            self.context,
            'search_sweep_step_rad',
            0.0,
        )))
        if (
            self.visited <= sweep_viewpoints
            and sweep_segments > 0
            and sweep_step > 0.0
        ):
            self.sweep_segment = 0
            self.sweep_direction = 1.0 if self.visited % 2 else -1.0
            self.sweep_observations = [self.active_candidate.pose]
            self._transition(
                'START_VIEW_SWEEP',
                f'{sweep_segments} x '
                f'{self.sweep_direction * sweep_step:.2f}rad',
            )
            return
        self._transition('OBSERVE_VIEW')

    def _start_view_sweep(self) -> None:
        segments = max(0, int(getattr(
            self.context,
            'search_sweep_segments',
            0,
        )))
        if self.sweep_segment >= segments:
            self._transition('OBSERVE_VIEW', 'local camera sweep complete')
            return
        self._reset_navigation()
        step = self.sweep_direction * abs(float(getattr(
            self.context,
            'search_sweep_step_rad',
            1.0,
        )))
        timeout = max(1.0, float(getattr(
            self.context,
            'search_sweep_segment_timeout_sec',
            8.0,
        )))
        if self.context.navigation.spin(step, timeout_sec=timeout):
            self._transition(
                'WAIT_VIEW_SWEEP',
                f'segment {self.sweep_segment + 1}/{segments}: '
                f'{step:.2f}rad',
            )
        else:
            self._transition(
                'OBSERVE_VIEW',
                'local camera sweep could not start',
            )

    def _wait_view_sweep(self) -> None:
        result = self.context.navigation.tick()
        if not result.terminal:
            return
        current = self.context.current_pose()
        if result.state is NavigationState.SUCCEEDED and current is not None:
            self.sweep_observations.append(current)
            self.sweep_segment += 1
            self._transition('START_VIEW_SWEEP')
            return
        self._transition(
            'OBSERVE_VIEW',
            f'local camera sweep stopped early: {result.message}',
        )

    def _observe_view(self) -> None:
        if self._state_elapsed() < self.context.camera_dwell_sec:
            return
        observations = self.sweep_observations or [self.active_candidate.pose]
        for pose in observations:
            self.planner.observe_negative(pose)
        self.sweep_observations = []
        self.context.visualizer.set_search(
            self.candidates,
            self.active_candidate,
            self.planner.searched_points(),
        )
        self._transition('PLAN_VIEW')

    def _reject_active_view(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.rejected_positions.append(self.active_candidate.pose)
        self._transition('PLAN_VIEW', f'view rejected: {reason}')

    def _reset_navigation(self) -> None:
        if self.context.navigation.snapshot().terminal:
            self.context.navigation.reset()

    def _reset_view_progress(self) -> None:
        self.progress_pose = self.context.current_pose()
        self.best_distance_remaining = None
        self.last_progress_at = self.context.monotonic()
        self.last_translation_progress_at = self.last_progress_at

    def _within_view_position_tolerance(self, current) -> bool:
        if current is None or self.active_candidate is None:
            return False
        tolerance = float(
            getattr(
                self.context,
                'search_view_position_tolerance_m',
                0.12,
            )
        )
        return current.distance_to(self.active_candidate.pose) <= tolerance

    def _view_navigation_stalled(self, current, snapshot) -> bool:
        now = self.context.monotonic()
        distance_step = float(
            getattr(self.context, 'search_progress_distance_m', 0.03)
        )
        yaw_step = float(
            getattr(self.context, 'search_progress_yaw_rad', 0.12)
        )
        progressed = False
        translation_progressed = False

        remaining = snapshot.distance_remaining
        if remaining is not None:
            if (
                self.best_distance_remaining is None
                or remaining
                <= self.best_distance_remaining - distance_step
            ):
                self.best_distance_remaining = remaining
                progressed = True
                translation_progressed = True

        if current is not None:
            if self.progress_pose is None:
                progressed = True
                translation_progressed = True
            else:
                translation = current.distance_to(self.progress_pose)
                rotation = abs(shortest_angular_distance(
                    self.progress_pose.yaw,
                    current.yaw,
                ))
                progressed = (
                    progressed
                    or translation >= distance_step
                    or rotation >= yaw_step
                )
                translation_progressed = (
                    translation_progressed
                    or translation >= distance_step
                )

        if progressed:
            self.progress_pose = current
            self.last_progress_at = now
        if translation_progressed:
            self.last_translation_progress_at = now

        if (
            current is not None
            and not self._within_view_position_tolerance(current)
            and now - self.last_translation_progress_at
            >= self._translation_stall_timeout()
        ):
            return True
        if progressed:
            return False
        return now - self.last_progress_at >= self._search_stall_timeout()

    def _search_stall_timeout(self) -> float:
        return max(
            1.0,
            float(getattr(
                self.context,
                'search_stall_timeout_sec',
                8.0,
            )),
        )

    def _translation_stall_timeout(self) -> float:
        return max(
            1.0,
            float(getattr(
                self.context,
                'search_translation_stall_timeout_sec',
                7.0,
            )),
        )

    def _transition(self, state: str, message: str = '') -> None:
        self.state = state
        self.message = message
        self.state_started = self.context.monotonic()
        self.context.logger.info(
            f'search[{self.color}] -> {state}'
            + (f': {message}' if message else '')
        )

    def _state_elapsed(self) -> float:
        return max(0.0, self.context.monotonic() - self.state_started)

    def _fail(self, message: str) -> None:
        self.status = SearchStatus.FAILED
        self._transition('SEARCH_FAILED', message)
