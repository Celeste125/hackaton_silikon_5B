from types import SimpleNamespace

from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.navigation_adapter import (
    NavigationSnapshot,
    NavigationState,
)
from imdl_autonomy.next_best_view import ViewCandidate
from imdl_autonomy.solvers.search import ActiveSearch, SearchStatus


class ClockMock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class NavigationMock:
    def __init__(self, start):
        self.current = start
        self.goal = None
        self.state = NavigationState.IDLE
        self.message = ''
        self.spin_calls = []
        self.plan_calls = []
        self.goals = []
        self.operation = ''

    def snapshot(self):
        return NavigationSnapshot(self.state, '', self.message, None, 0)

    def spin(self, angle, timeout_sec):
        self.spin_calls.append((angle, timeout_sec))
        self.goal = Pose2D(
            self.current.x,
            self.current.y,
            self.current.yaw + angle,
        )
        self.operation = 'spin'
        self.state = NavigationState.ACTIVE
        return True

    def compute_path(self, goal, timeout_sec):
        self.plan_calls.append((goal, timeout_sec))
        self.goal = goal
        self.operation = 'plan'
        self.state = NavigationState.ACTIVE
        return True

    def navigate_to(self, goal, controller, timeout_sec):
        self.goals.append((goal, controller, timeout_sec))
        self.goal = goal
        self.operation = 'navigate'
        self.state = NavigationState.ACTIVE
        return True

    def tick(self):
        if self.state is NavigationState.ACTIVE:
            if self.operation != 'plan':
                self.current = self.goal
            self.state = NavigationState.SUCCEEDED
        return self.snapshot()

    def reset(self):
        self.state = NavigationState.IDLE
        self.operation = ''

    def cancel(self):
        self.state = NavigationState.CANCELED

    def clear_costmaps(self):
        return True


class FailPlanningNavigationMock(NavigationMock):
    def tick(self):
        if self.state is NavigationState.ACTIVE and self.operation == 'plan':
            self.state = NavigationState.FAILED
            self.message = 'mock live-costmap rejection'
            return self.snapshot()
        return super().tick()


class StalledNavigationMock(NavigationMock):
    def snapshot(self):
        distance = 0.6 if self.operation == 'navigate' else None
        return NavigationSnapshot(
            self.state,
            self.operation,
            self.message,
            distance,
            0,
        )

    def tick(self):
        if self.state is NavigationState.ACTIVE:
            if self.operation in {'spin', 'plan'}:
                if self.operation == 'spin':
                    self.current = self.goal
                self.state = NavigationState.SUCCEEDED
        return self.snapshot()


class RotationOnlyNavigationMock(StalledNavigationMock):
    def tick(self):
        if (
            self.state is NavigationState.ACTIVE
            and self.operation == 'navigate'
        ):
            self.current = Pose2D(
                self.current.x,
                self.current.y,
                self.current.yaw + 0.20,
            )
        else:
            return super().tick()
        return self.snapshot()


class ObjectStoreMock:
    def __init__(self):
        self.track = None

    def confirmed(self, color):
        if self.track is not None and self.track.color == color:
            return self.track
        return None


class PlannerMock:
    def __init__(self, candidate):
        self.candidate = candidate
        self.negative_observations = []

    def rank_candidates(self, current, limit, future_color_bonus):
        assert current is not None
        assert limit == 60
        return [self.candidate]

    def observe_negative(self, pose, miss_probability=0.22):
        self.negative_observations.append((pose, miss_probability))

    def searched_points(self):
        return [(self.candidate.pose.x, self.candidate.pose.y)]


class VisualizerMock:
    def set_search(self, *_args):
        pass

    def set_active_goal(self, *_args):
        pass


class LoggerMock:
    def info(self, *_args):
        pass


def test_active_search_observes_view_and_stops_on_confirmed_track():
    clock = ClockMock()
    start = Pose2D(-0.4, -0.4, 0.0)
    view = ViewCandidate(
        pose=Pose2D(0.1, 0.1, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.7,
        travel_time_sec=5.0,
        turn_time_sec=1.0,
        risk=0.1,
    )
    navigation = NavigationMock(start)
    objects = ObjectStoreMock()
    planner = PlannerMock(view)
    context = SimpleNamespace(
        monotonic=clock,
        objects=objects,
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=10.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
    )
    search = ActiveSearch(context, planner)

    search.start('red')
    clock.advance(0.9)
    search.tick()  # START_INITIAL_SCAN
    search.tick()  # WAIT_INITIAL_SCAN
    search.tick()  # PLAN_VIEW
    search.tick()  # START_VIEW_PLAN
    search.tick()  # WAIT_VIEW_PLAN
    search.tick()  # START_VIEW_NAV after feasible path
    search.tick()  # WAIT_VIEW_NAV
    search.tick()  # OBSERVE_VIEW after successful alignment
    clock.advance(0.6)
    search.tick()  # Bayesian negative observation

    assert navigation.spin_calls == [(0.55, 12.0)]
    assert navigation.plan_calls[0][0] == view.pose
    assert navigation.goals[0][0] == view.pose
    assert planner.negative_observations == [(view.pose, 0.22)]
    assert search.visited == 1

    objects.track = SimpleNamespace(
        object_id='red-001',
        color='red',
        pose=Pose2D(0.2, 0.2, 0.0),
    )
    status = search.tick()

    assert status is SearchStatus.FOUND
    assert search.track.object_id == 'red-001'


def test_active_search_fails_when_map_has_no_reachable_view():
    clock = ClockMock()
    start = Pose2D(0.0, 0.0, 0.0)
    navigation = NavigationMock(start)
    objects = ObjectStoreMock()
    planner = PlannerMock(SimpleNamespace(pose=start))
    planner.rank_candidates = lambda *_args, **_kwargs: []
    context = SimpleNamespace(
        monotonic=clock,
        objects=objects,
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=10.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
    )
    search = ActiveSearch(context, planner)

    search.start('red')
    clock.advance(0.9)
    search.tick()
    search.tick()
    search.tick()
    status = search.tick()

    assert status is SearchStatus.FAILED
    assert search.message == 'NO_REACHABLE_NEXT_BEST_VIEW'


def test_detection_cancels_active_nav2_operation_before_transport():
    clock = ClockMock()
    start = Pose2D(-0.4, -0.4, 0.0)
    navigation = NavigationMock(start)
    objects = ObjectStoreMock()
    candidate = ViewCandidate(
        pose=Pose2D(0.1, 0.1, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.7,
        travel_time_sec=5.0,
        turn_time_sec=1.0,
        risk=0.1,
    )
    context = SimpleNamespace(
        monotonic=clock,
        objects=objects,
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=10.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
    )
    search = ActiveSearch(context, PlannerMock(candidate))
    search.start('red')
    clock.advance(0.9)
    search.tick()  # START_INITIAL_SCAN
    search.tick()  # active Spin
    assert navigation.state is NavigationState.ACTIVE
    objects.track = SimpleNamespace(
        object_id='red-001',
        color='red',
        pose=Pose2D(0.2, 0.2, 0.0),
    )

    status = search.tick()

    assert status is SearchStatus.FOUND
    assert navigation.state is NavigationState.CANCELED


def test_information_view_uses_overlapping_camera_sweep_and_updates_belief():
    clock = ClockMock()
    start = Pose2D(0.0, 0.0, 0.0)
    view = ViewCandidate(
        pose=Pose2D(0.1, 0.1, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.2,
        travel_time_sec=1.0,
        turn_time_sec=0.5,
        risk=0.1,
    )
    navigation = NavigationMock(view.pose)
    planner = PlannerMock(view)
    context = SimpleNamespace(
        monotonic=clock,
        objects=ObjectStoreMock(),
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=60.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=10.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
        search_sweep_viewpoints=3,
        search_sweep_segments=3,
        search_sweep_step_rad=1.0,
        search_sweep_segment_timeout_sec=8.0,
    )
    search = ActiveSearch(context, planner)
    search.start('red')
    search.active_candidate = view

    search._view_reached()
    for _ in range(3):
        search.tick()  # start one safe Nav2 Spin segment
        assert search.state == 'WAIT_VIEW_SWEEP'
        search.tick()  # complete segment
    search.tick()  # transition to observation after final segment
    assert search.state == 'OBSERVE_VIEW'
    clock.advance(0.6)
    search.tick()

    assert navigation.spin_calls == [(1.0, 8.0)] * 3
    assert len(planner.negative_observations) == 4
    yaws = [entry[0].yaw for entry in planner.negative_observations]
    assert yaws == [0.5, 1.5, 2.5, 3.5]


def test_detection_during_camera_sweep_cancels_spin_immediately():
    clock = ClockMock()
    view = ViewCandidate(
        pose=Pose2D(0.1, 0.1, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.2,
        travel_time_sec=1.0,
        turn_time_sec=0.5,
        risk=0.1,
    )
    navigation = NavigationMock(view.pose)
    objects = ObjectStoreMock()
    context = SimpleNamespace(
        monotonic=clock,
        objects=objects,
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=60.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=10.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
        search_sweep_viewpoints=1,
        search_sweep_segments=6,
        search_sweep_step_rad=1.0,
        search_sweep_segment_timeout_sec=8.0,
    )
    search = ActiveSearch(context, PlannerMock(view))
    search.start('red')
    search.active_candidate = view
    search._view_reached()
    search.tick()
    assert search.state == 'WAIT_VIEW_SWEEP'

    objects.track = SimpleNamespace(
        object_id='red-001',
        color='red',
        pose=Pose2D(-0.2, 0.2, 0.0),
    )
    status = search.tick()

    assert status is SearchStatus.FOUND
    assert navigation.state is NavigationState.CANCELED


def test_unreachable_view_is_rejected_without_fake_camera_observation():
    clock = ClockMock()
    start = Pose2D(-0.4, -0.4, 0.0)
    navigation = FailPlanningNavigationMock(start)
    objects = ObjectStoreMock()
    candidate = ViewCandidate(
        pose=Pose2D(0.1, 0.1, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.7,
        travel_time_sec=5.0,
        turn_time_sec=1.0,
        risk=0.1,
    )
    planner = PlannerMock(candidate)
    context = SimpleNamespace(
        monotonic=clock,
        objects=objects,
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=10.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
    )
    search = ActiveSearch(context, planner)
    search.start('red')
    clock.advance(0.9)
    for _ in range(6):
        search.tick()

    assert search.state == 'PLAN_VIEW'
    assert search.rejected_positions == [candidate.pose]
    assert planner.negative_observations == []


def test_stalled_search_navigation_rejects_view_before_action_timeout():
    clock = ClockMock()
    start = Pose2D(-0.4, -0.4, 0.0)
    candidate = ViewCandidate(
        pose=Pose2D(0.3, 0.3, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.9,
        travel_time_sec=6.0,
        turn_time_sec=1.0,
        risk=0.1,
    )
    navigation = StalledNavigationMock(start)
    context = SimpleNamespace(
        monotonic=clock,
        objects=ObjectStoreMock(),
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=75.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
        search_stall_timeout_sec=8.0,
        search_progress_distance_m=0.03,
        search_progress_yaw_rad=0.12,
    )
    search = ActiveSearch(context, PlannerMock(candidate))
    search.start('red')
    clock.advance(0.9)
    for _ in range(7):
        search.tick()
    assert search.state == 'WAIT_VIEW_NAV'

    search.tick()
    clock.advance(8.1)
    search.tick()

    assert search.state == 'PLAN_VIEW'
    assert search.rejected_positions == [candidate.pose]
    assert 'no translational' in search.message


def test_nearby_search_pose_is_accepted_without_waiting_for_nav2_timeout():
    clock = ClockMock()
    start = Pose2D(0.0, 0.0, 0.5)
    candidate = ViewCandidate(
        pose=Pose2D(0.10, 0.0, 1.05),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.1,
        travel_time_sec=1.0,
        turn_time_sec=0.0,
        risk=0.1,
    )
    navigation = StalledNavigationMock(start)
    context = SimpleNamespace(
        monotonic=clock,
        objects=ObjectStoreMock(),
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=75.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
        search_view_position_tolerance_m=0.12,
        terminal_stop_settle_sec=0.30,
    )
    search = ActiveSearch(context, PlannerMock(candidate))
    search.start('red')
    clock.advance(0.9)
    for _ in range(7):
        search.tick()
    assert search.state == 'WAIT_VIEW_NAV'

    search.tick()
    assert search.state == 'WAIT_VIEW_TERMINAL_STOP'
    clock.advance(0.31)
    search.tick()

    assert search.state == 'OBSERVE_VIEW'
    assert search.visited == 1


def test_rotation_only_cannot_mask_missing_translation_progress():
    clock = ClockMock()
    start = Pose2D(-0.35, -0.35, 0.0)
    candidate = ViewCandidate(
        pose=Pose2D(0.30, 0.30, 0.5),
        score=2.0,
        expected_detection=0.3,
        new_coverage=0.2,
        path_length_m=0.9,
        travel_time_sec=6.0,
        turn_time_sec=1.0,
        risk=0.1,
    )
    navigation = RotationOnlyNavigationMock(start)
    context = SimpleNamespace(
        monotonic=clock,
        objects=ObjectStoreMock(),
        navigation=navigation,
        current_pose=lambda: navigation.current,
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        search_timeout_sec=30.0,
        max_search_viewpoints=4,
        navigation_timeout_sec=75.0,
        camera_dwell_sec=0.5,
        final_yaw_tolerance_rad=0.1,
        search_stall_timeout_sec=8.0,
        search_translation_stall_timeout_sec=5.0,
        search_progress_distance_m=0.03,
        search_progress_yaw_rad=0.12,
    )
    search = ActiveSearch(context, PlannerMock(candidate))
    search.start('red')
    clock.advance(0.9)
    for _ in range(7):
        search.tick()
    assert search.state == 'WAIT_VIEW_NAV'

    for _ in range(6):
        clock.advance(1.0)
        search.tick()

    assert search.state == 'PLAN_VIEW'
    assert search.rejected_positions == [candidate.pose]
