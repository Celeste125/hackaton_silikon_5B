"""Task 2 autonomy: exact A -> B -> C navigation with Nav2 replanning."""

from .base import BaseSolver
from ..geometry import shortest_angular_distance
from ..navigation_adapter import NavigationState


class Task2Solver(BaseSolver):

    def __init__(self, context):
        super().__init__(context)
        self.goals = []
        self.goal_index = 0
        self.goal_attempt = 1

    def on_start(self) -> None:
        self.goals = [
            self.context.mission.pose('B'),
            self.context.mission.pose('C'),
        ]
        self._transition('START_GOAL', 'A is the physical start; next target is B')

    def on_tick(self) -> None:
        if self.state == 'START_GOAL':
            self._start_goal()
        elif self.state == 'NAVIGATING_GOAL':
            self._wait_goal()

    def _start_goal(self) -> None:
        goal = self.goals[self.goal_index]
        name = ('B', 'C')[self.goal_index]
        if self.context.navigation.snapshot().terminal:
            self.context.navigation.reset()
        self.context.visualizer.set_active_goal(goal, 'RobustMPPI')
        if not self.context.navigation.navigate_to(
            goal,
            controller='RobustMPPI',
            timeout_sec=self.context.navigation_timeout_sec,
            mission_pose=True,
        ):
            self._goal_failed(name, self.context.navigation.message)
            return
        self._transition(
            'NAVIGATING_GOAL',
            f'target={name} ({goal.x:.3f}, {goal.y:.3f}, {goal.yaw:.3f}), '
            f'attempt={self.goal_attempt}',
        )

    def _wait_goal(self) -> None:
        result = self.context.navigation.tick()
        name = ('B', 'C')[self.goal_index]
        if result.state is NavigationState.SUCCEEDED:
            self._goal_reached(name, result.recoveries)
        elif result.terminal:
            self._goal_failed(name, result.message)

    def _goal_reached(self, name: str, recoveries: int) -> None:
        goal = self.goals[self.goal_index]
        current = self.context.current_pose()
        pose_text = 'pose unavailable'
        if current is not None:
            yaw_error = shortest_angular_distance(current.yaw, goal.yaw)
            pose_text = (
                f'position_error={current.distance_to(goal):.3f}m, '
                f'yaw_error={yaw_error:.3f}rad'
            )
        self.context.logger.info(
            f'Task 2 point {name} reached and aligned; {pose_text}; '
            f'recoveries={recoveries}'
        )
        self.goal_index += 1
        self.goal_attempt = 1
        if self.goal_index >= len(self.goals):
            self._succeed('A -> B -> C completed without raw cmd_vel fallback')
        else:
            self._transition('START_GOAL', 'next target is C')

    def _goal_failed(self, name: str, reason: str) -> None:
        self.context.navigation.cancel()
        if self.goal_attempt >= self.context.navigation_retries:
            self._fail(
                f'NAVIGATION_TO_{name}_FAILED',
                f'{reason}; attempts={self.goal_attempt}',
            )
            return
        self.goal_attempt += 1
        self.attempt = self.goal_attempt
        self.context.navigation.clear_costmaps()
        self._transition(
            'START_GOAL',
            f'Nav2 recovery for {name}: {reason}',
        )
