from types import SimpleNamespace

from imdl_autonomy.goal_alignment import AlignmentStatus, GoalAlignment
from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.navigation_adapter import (
    NavigationSnapshot,
    NavigationState,
)


class NavigationMock:
    def __init__(self):
        self.state = NavigationState.SUCCEEDED
        self.spins = []

    def snapshot(self):
        return NavigationSnapshot(self.state, 'SPIN', '', None, 0)

    def reset(self):
        self.state = NavigationState.IDLE

    def spin(self, angle_rad, timeout_sec):
        self.spins.append((angle_rad, timeout_sec))
        self.state = NavigationState.ACTIVE
        return True

    def tick(self):
        self.state = NavigationState.SUCCEEDED
        return self.snapshot()

    def cancel(self):
        self.state = NavigationState.CANCELED


class LoggerMock:
    def info(self, *_args):
        pass


def test_alignment_uses_shortest_collision_checked_spin():
    now = [0.0]
    pose = [Pose2D(0.0, 0.0, 3.0)]
    navigation = NavigationMock()
    context = SimpleNamespace(
        navigation=navigation,
        current_pose=lambda: pose[0],
        final_yaw_tolerance_rad=0.10,
        monotonic=lambda: now[0],
        logger=LoggerMock(),
    )
    alignment = GoalAlignment(context)

    status = alignment.start(Pose2D(0.0, 0.0, -3.0))

    assert status is AlignmentStatus.RUNNING
    assert len(navigation.spins) == 1
    assert 0.0 < navigation.spins[0][0] < 0.30

    alignment.tick()
    pose[0] = Pose2D(0.0, 0.0, -3.02)
    now[0] += 0.25

    assert alignment.tick() is AlignmentStatus.SUCCEEDED


def test_alignment_skips_spin_inside_tolerance():
    navigation = NavigationMock()
    context = SimpleNamespace(
        navigation=navigation,
        current_pose=lambda: Pose2D(0.0, 0.0, 0.04),
        final_yaw_tolerance_rad=0.10,
        monotonic=lambda: 0.0,
        logger=LoggerMock(),
    )

    status = GoalAlignment(context).start(Pose2D(0.0, 0.0, 0.0))

    assert status is AlignmentStatus.SUCCEEDED
    assert navigation.spins == []
