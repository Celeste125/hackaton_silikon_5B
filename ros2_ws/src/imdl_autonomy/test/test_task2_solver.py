from types import SimpleNamespace

from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.navigation_adapter import (
    NavigationSnapshot,
    NavigationState,
)
from imdl_autonomy.solvers.base import SolverStatus
from imdl_autonomy.solvers.task2 import Task2Solver


class NavigationMock:

    def __init__(self):
        self.goals = []
        self.spins = []
        self.state = NavigationState.IDLE
        self.message = ''
        self.recoveries = 0

    def snapshot(self):
        return NavigationSnapshot(self.state, '', self.message, None, 0)

    def navigate_to(
        self,
        goal,
        controller,
        timeout_sec,
        mission_pose=False,
        precise=False,
    ):
        self.goals.append((goal, controller, mission_pose, precise))
        self.state = NavigationState.ACTIVE
        return True

    def tick(self):
        self.state = NavigationState.SUCCEEDED
        return self.snapshot()

    def reset(self):
        self.state = NavigationState.IDLE

    def cancel(self):
        self.state = NavigationState.CANCELED

    def clear_costmaps(self):
        return True

    def spin(self, angle_rad, timeout_sec):
        self.spins.append((angle_rad, timeout_sec))
        self.state = NavigationState.ACTIVE
        return True


class VisualizerMock:

    def set_active_goal(self, *_args):
        pass

    def set_solver_state(self, *_args):
        pass


class LoggerMock:

    def info(self, *_args):
        pass


def test_task2_visits_only_b_then_c_without_variant_logic():
    mission = SimpleNamespace(
        time_limit_sec=180.0,
        start=Pose2D(0.5, -0.5, 0.0),
        pose=lambda name: {
            'B': Pose2D(-0.5, -0.5, 1.57),
            'C': Pose2D(-0.5, 0.5, 0.0),
        }[name],
    )
    navigation = NavigationMock()
    context = SimpleNamespace(
        mission=mission,
        navigation=navigation,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        monotonic=lambda: 0.0,
        navigation_timeout_sec=75.0,
        navigation_retries=2,
        final_yaw_tolerance_rad=0.10,
    )
    context.current_pose = lambda: (
        navigation.goals[-1][0] if navigation.goals else mission.start
    )
    solver = Task2Solver(context)
    solver.start()
    for _ in range(5):
        result = solver.tick()

    assert result.status is SolverStatus.SUCCEEDED
    assert [goal for goal, _, _, _ in navigation.goals] == [
        mission.pose('B'), mission.pose('C')
    ]
    assert all(
        controller == 'RobustMPPI' and mission_pose and not precise
        for _, controller, mission_pose, precise in navigation.goals
    )


class RetryNavigationMock(NavigationMock):

    def __init__(self):
        super().__init__()
        self.failed_once = False
        self.clear_count = 0

    def tick(self):
        if not self.failed_once:
            self.failed_once = True
            self.state = NavigationState.FAILED
            self.message = 'mock controller abort'
        else:
            self.state = NavigationState.SUCCEEDED
        return self.snapshot()

    def clear_costmaps(self):
        self.clear_count += 1
        return True


def test_task2_retries_aborted_goal_then_continues_to_c():
    mission = SimpleNamespace(
        time_limit_sec=180.0,
        start=Pose2D(0.5, -0.5, 0.0),
        pose=lambda name: {
            'B': Pose2D(-0.5, -0.5, 1.57),
            'C': Pose2D(-0.5, 0.5, 0.0),
        }[name],
    )
    navigation = RetryNavigationMock()
    context = SimpleNamespace(
        mission=mission,
        navigation=navigation,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        monotonic=lambda: 0.0,
        navigation_timeout_sec=75.0,
        navigation_retries=2,
        final_yaw_tolerance_rad=0.10,
    )
    context.current_pose = lambda: (
        navigation.goals[-1][0] if navigation.goals else mission.start
    )
    solver = Task2Solver(context)
    solver.start()

    for _ in range(8):
        result = solver.tick()

    assert result.status is SolverStatus.SUCCEEDED
    assert [goal for goal, _, _, _ in navigation.goals] == [
        mission.pose('B'), mission.pose('B'), mission.pose('C')
    ]
    assert navigation.clear_count == 1
