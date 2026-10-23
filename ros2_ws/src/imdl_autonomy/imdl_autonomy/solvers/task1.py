"""Task 1 autonomy: A to B forward, then the same path backward to A."""

from .base import BaseSolver
from ..create3_adapter import Create3SafetyState
from ..goal_alignment import AlignmentStatus, GoalAlignment
from ..navigation_adapter import NavigationState, reverse_path


class Task1Solver(BaseSolver):

    def __init__(self, context):
        super().__init__(context)
        self.outbound_path = None
        self.pending_reverse_failure = None
        self.alignment = GoalAlignment(context)

    def on_start(self) -> None:
        self._transition('START_FORWARD', 'SmacPlanner2D + FastRPP')

    def on_tick(self) -> None:
        if self.state == 'START_FORWARD':
            self._start_forward()
        elif self.state == 'DRIVING_FORWARD':
            self._wait_forward()
        elif self.state == 'ALIGNING_AT_B':
            self._wait_b_alignment()
        elif self.state == 'ENABLE_REVERSE_SAFETY':
            self._enable_reverse_safety()
        elif self.state == 'WAIT_REVERSE_SAFETY':
            self._wait_reverse_safety()
        elif self.state == 'START_REVERSE':
            self._start_reverse()
        elif self.state == 'DRIVING_REVERSE':
            self._wait_reverse()
        elif self.state == 'RESTORE_REVERSE_SAFETY':
            self._restore_reverse_safety()
        elif self.state == 'WAIT_SAFETY_RESTORE':
            self._wait_safety_restore()
        elif self.state == 'START_FINAL_ORIENTATION':
            self._start_final_orientation()
        elif self.state == 'FINAL_ORIENTATION':
            self._wait_final_orientation()

    def _start_forward(self) -> None:
        self.context.navigation.reset()
        self.context.navigation.start_plan_capture()
        self.context.visualizer.set_active_goal(
            self.context.mission.pose('B'),
            'FastRPP',
        )
        if not self.context.navigation.navigate_to(
            self.context.mission.pose('B'),
            controller='FastRPP',
            timeout_sec=self.context.navigation_timeout_sec,
        ):
            self._fail('FORWARD_FAILED', self.context.navigation.message)
            return
        self._transition(
            'DRIVING_FORWARD',
            'A -> B with FastRPP and continuous replanning',
        )

    def _wait_forward(self) -> None:
        result = self.context.navigation.tick()
        if result.state is NavigationState.SUCCEEDED:
            path = self.context.navigation.stop_plan_capture()
            if path is None or len(path.poses) < 2:
                self._fail(
                    'FORWARD_PATH_MISSING',
                    'Nav2 reached B without publishing a reusable global path',
                )
                return
            self.outbound_path = path
            self.context.visualizer.set_path(path)
            status = self.alignment.start(self.context.mission.pose('B'))
            if status is AlignmentStatus.SUCCEEDED:
                self._transition('ENABLE_REVERSE_SAFETY', self.alignment.message)
            elif status is AlignmentStatus.FAILED:
                self._fail('B_ORIENTATION_FAILED', self.alignment.message)
            else:
                self._transition('ALIGNING_AT_B', self.alignment.message)
        elif result.terminal:
            self.context.navigation.stop_plan_capture()
            self._fail('FORWARD_FAILED', result.message)

    def _wait_b_alignment(self) -> None:
        status = self.alignment.tick()
        if status is AlignmentStatus.SUCCEEDED:
            self._transition('ENABLE_REVERSE_SAFETY', self.alignment.message)
        elif status is AlignmentStatus.FAILED:
            self._fail('B_ORIENTATION_FAILED', self.alignment.message)

    def _enable_reverse_safety(self) -> None:
        if not self.context.create3_safety.enable_backup_only():
            self._fail(
                'CREATE3_SAFETY_ADAPTER_UNAVAILABLE',
                self.context.create3_safety.snapshot().message,
            )
            return
        self._transition(
            'WAIT_REVERSE_SAFETY',
            'temporarily enabling Create3 backup_only',
        )

    def _wait_reverse_safety(self) -> None:
        snapshot = self.context.create3_safety.snapshot()
        if snapshot.state is Create3SafetyState.SUCCEEDED:
            self._transition('START_REVERSE', snapshot.message)
        elif snapshot.state is Create3SafetyState.FAILED:
            self._fail('CREATE3_SAFETY_OVERRIDE_FAILED', snapshot.message)

    def _start_reverse(self) -> None:
        self.context.navigation.reset()
        reversed_path = reverse_path(
            self.outbound_path,
            stamp=self.context.node.get_clock().now().to_msg(),
        )
        self.context.visualizer.set_path(reversed_path)
        self.context.visualizer.set_active_goal(
            self.context.mission.pose('A'),
            'ReverseMPPI',
        )
        if not self.context.navigation.follow_path(
            reversed_path,
            controller='ReverseMPPI',
            goal_checker='task1_goal_checker',
            timeout_sec=self.context.navigation_timeout_sec,
            validate_reverse=True,
        ):
            self._fail('REVERSE_FAILED', self.context.navigation.message)
            return
        self._transition(
            'DRIVING_REVERSE',
            'B -> A; positive translational commands are rejected',
        )

    def _wait_reverse(self) -> None:
        result = self.context.navigation.tick()
        if result.state is NavigationState.SUCCEEDED:
            if not self.context.navigation.reverse_motion_valid():
                self.pending_reverse_failure = (
                    'REVERSE_VALIDATION_FAILED',
                    'No exclusively negative translational velocity was observed',
                )
            self._transition('RESTORE_REVERSE_SAFETY')
        elif result.terminal:
            self.pending_reverse_failure = ('REVERSE_FAILED', result.message)
            self._transition('RESTORE_REVERSE_SAFETY')

    def _restore_reverse_safety(self) -> None:
        if not self.context.create3_safety.restore():
            self._fail(
                'CREATE3_SAFETY_RESTORE_FAILED',
                self.context.create3_safety.snapshot().message,
            )
            return
        self._transition('WAIT_SAFETY_RESTORE')

    def _wait_safety_restore(self) -> None:
        snapshot = self.context.create3_safety.snapshot()
        if snapshot.state is Create3SafetyState.FAILED:
            self._fail('CREATE3_SAFETY_RESTORE_FAILED', snapshot.message)
            return
        if snapshot.state is not Create3SafetyState.SUCCEEDED:
            return
        if self.pending_reverse_failure is not None:
            state, message = self.pending_reverse_failure
            self._fail(state, message)
            return
        self._transition('START_FINAL_ORIENTATION', snapshot.message)

    def cancel(self):
        self.context.create3_safety.restore()
        return super().cancel()

    def _start_final_orientation(self) -> None:
        status = self.alignment.start(self.context.mission.pose('A'))
        if status is AlignmentStatus.SUCCEEDED:
            self._succeed('A -> B forward and B -> A backward completed')
        elif status is AlignmentStatus.FAILED:
            self._fail('FINAL_ORIENTATION_FAILED', self.alignment.message)
        else:
            self._transition('FINAL_ORIENTATION', self.alignment.message)

    def _wait_final_orientation(self) -> None:
        status = self.alignment.tick()
        if status is AlignmentStatus.SUCCEEDED:
            self._succeed('A -> B forward and B -> A backward completed')
        elif status is AlignmentStatus.FAILED:
            self._fail('FINAL_ORIENTATION_FAILED', self.alignment.message)
