from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from imdl_autonomy.create3_adapter import (
    Create3SafetySnapshot,
    Create3SafetyState,
)
from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.navigation_adapter import (
    NavigationSnapshot,
    NavigationState,
)
from imdl_autonomy.solvers.base import SolverStatus
from imdl_autonomy.solvers.task1 import Task1Solver
from nav_msgs.msg import Path
from rclpy.time import Time


class NavigationMock:

    def __init__(self, path, start, goal):
        self.path = path
        self.result_path = None
        self.current = start
        self.goal = goal
        self.state = NavigationState.IDLE
        self.operation = ''
        self.message = ''
        self.navigate_calls = []
        self.follow_calls = []
        self.last_path = None
        self.captured_path = None
        self.spin_angles = []

    def snapshot(self):
        return NavigationSnapshot(
            self.state,
            self.operation,
            self.message,
            None,
            0,
        )

    def follow_path(self, path, controller, goal_checker, timeout_sec,
                    validate_reverse=False):
        self.operation = f'FOLLOW_{controller}'
        self.follow_calls.append((
            path,
            controller,
            goal_checker,
            validate_reverse,
        ))
        self.last_path = path
        self.state = NavigationState.ACTIVE
        return True

    def navigate_to(self, goal, controller, timeout_sec):
        self.operation = f'NAVIGATE_{controller}'
        self.navigate_calls.append((goal, controller, timeout_sec))
        self.state = NavigationState.ACTIVE
        return True

    def start_plan_capture(self):
        self.captured_path = self.path

    def stop_plan_capture(self):
        return self.captured_path

    def tick(self):
        if self.state is not NavigationState.ACTIVE:
            return self.snapshot()
        if self.operation == 'NAVIGATE_FastRPP':
            # Simulate a positional Nav2 success whose path tangent does not
            # necessarily match the requested beta orientation.
            self.current = Pose2D(self.goal.x, self.goal.y, 0.0)
        elif self.operation == 'FOLLOW_ReverseMPPI':
            self.current = Pose2D(0.0, 0.0, 0.0)
        elif self.operation == 'SPIN':
            self.current = Pose2D(
                self.current.x,
                self.current.y,
                self.current.yaw + self.spin_angles[-1],
            )
        self.state = NavigationState.SUCCEEDED
        return self.snapshot()

    def reset(self):
        self.state = NavigationState.IDLE
        self.operation = ''

    def reverse_motion_valid(self):
        return True

    def spin(self, angle_rad, **_kwargs):
        self.spin_angles.append(angle_rad)
        self.state = NavigationState.ACTIVE
        self.operation = 'SPIN'
        return True

    def cancel(self):
        self.state = NavigationState.CANCELED


class Create3SafetyMock:

    def __init__(self):
        self.state = Create3SafetyState.IDLE
        self.enabled = 0
        self.restored = 0

    def enable_backup_only(self):
        self.enabled += 1
        self.state = Create3SafetyState.SUCCEEDED
        return True

    def restore(self):
        self.restored += 1
        self.state = Create3SafetyState.SUCCEEDED
        return True

    def snapshot(self):
        return Create3SafetySnapshot(self.state, 'mock', 'ok')


class VisualizerMock:

    def set_active_goal(self, *_args):
        pass

    def set_solver_state(self, *_args):
        pass

    def set_path(self, *_args):
        pass


class LoggerMock:

    def info(self, *_args):
        pass


def _path():
    path = Path()
    path.header.frame_id = 'map'
    for x in (0.0, 0.5, 1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def test_task1_runs_forward_then_validated_reverse_and_restores_safety():
    point_a = Pose2D(0.0, 0.0, 0.0)
    point_b = Pose2D(1.0, 0.0, 0.0)
    mission = SimpleNamespace(
        time_limit_sec=180.0,
        pose=lambda name: {'A': point_a, 'B': point_b}[name],
    )
    navigation = NavigationMock(_path(), point_a, point_b)
    create3_safety = Create3SafetyMock()
    clock = {'value': 0.0}
    context = SimpleNamespace(
        mission=mission,
        navigation=navigation,
        create3_safety=create3_safety,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        monotonic=lambda: clock['value'],
        current_pose=lambda: navigation.current,
        navigation_timeout_sec=75.0,
        final_yaw_tolerance_rad=0.10,
        node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=2.0))
        ),
    )
    solver = Task1Solver(context)
    result = solver.start()

    for _ in range(14):
        if result.status is not SolverStatus.RUNNING:
            break
        result = solver.tick()

    assert result.status is SolverStatus.SUCCEEDED
    assert [call[1] for call in navigation.navigate_calls] == ['FastRPP']
    assert [call[1] for call in navigation.follow_calls] == ['ReverseMPPI']
    assert navigation.follow_calls[0][3] is True
    reverse_positions = [
        pose.pose.position.x for pose in navigation.follow_calls[0][0].poses
    ]
    assert reverse_positions == [1.0, 0.5, 0.0]
    assert create3_safety.enabled == 1
    assert create3_safety.restored == 1


def test_task1_verifies_beta_at_b_before_starting_reverse():
    point_a = Pose2D(0.0, 0.0, 0.0)
    point_b = Pose2D(1.0, 0.0, 1.5707963267948966)
    mission = SimpleNamespace(
        time_limit_sec=180.0,
        pose=lambda name: {'A': point_a, 'B': point_b}[name],
    )
    navigation = NavigationMock(_path(), point_a, point_b)
    create3_safety = Create3SafetyMock()
    clock = {'value': 0.0}
    context = SimpleNamespace(
        mission=mission,
        navigation=navigation,
        create3_safety=create3_safety,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        monotonic=lambda: clock['value'],
        current_pose=lambda: navigation.current,
        navigation_timeout_sec=75.0,
        final_yaw_tolerance_rad=0.10,
        node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=2.0))
        ),
    )
    solver = Task1Solver(context)
    result = solver.start()

    for _ in range(20):
        if result.status is not SolverStatus.RUNNING:
            break
        clock['value'] += 0.1
        result = solver.tick()

    assert result.status is SolverStatus.SUCCEEDED
    assert navigation.spin_angles[0] == point_b.yaw
    assert navigation.follow_calls
    assert create3_safety.enabled == 1
    assert create3_safety.restored == 1


def test_task1_reverifies_alpha_after_reverse_path():
    point_a = Pose2D(0.0, 0.0, 0.75)
    point_b = Pose2D(1.0, 0.0, 0.0)
    mission = SimpleNamespace(
        time_limit_sec=180.0,
        pose=lambda name: {'A': point_a, 'B': point_b}[name],
    )
    navigation = NavigationMock(_path(), point_a, point_b)
    create3_safety = Create3SafetyMock()
    clock = {'value': 0.0}
    context = SimpleNamespace(
        mission=mission,
        navigation=navigation,
        create3_safety=create3_safety,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        monotonic=lambda: clock['value'],
        current_pose=lambda: navigation.current,
        navigation_timeout_sec=75.0,
        final_yaw_tolerance_rad=0.10,
        node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=2.0))
        ),
    )
    solver = Task1Solver(context)
    result = solver.start()

    for _ in range(24):
        if result.status is not SolverStatus.RUNNING:
            break
        clock['value'] += 0.1
        result = solver.tick()

    assert result.status is SolverStatus.SUCCEEDED
    assert navigation.spin_angles[-1] == point_a.yaw
    assert navigation.current.yaw == point_a.yaw
