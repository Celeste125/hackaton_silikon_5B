from types import SimpleNamespace
import math

import pytest

from imdl_autonomy.mission_spec import Pose2D
from imdl_autonomy.navigation_adapter import (
    NavigationSnapshot,
    NavigationState,
)
from imdl_autonomy.next_best_view import ApproachCandidate
from imdl_autonomy.range_fusion import FrontRangeReading
from imdl_autonomy.runtime_context import ManipulatorState
from imdl_autonomy.solvers.base import SolverStatus
import imdl_autonomy.solvers.task4 as task4_module
import imdl_autonomy.solvers.task5 as task5_module
from imdl_autonomy.solvers.task3 import Task3Solver
from imdl_autonomy.solvers.task4 import Task4Solver
from imdl_autonomy.solvers.task5 import Task5Solver
from imdl_autonomy.solvers.transport import TransportFlow, select_diverse_approaches


class ClockMock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds=0.1):
        self.value += seconds


class LoggerMock:
    def info(self, *_args):
        pass


class VisualizerMock:
    def set_active_goal(self, *_args):
        pass

    def set_solver_state(self, *_args):
        pass

    def set_approaches(self, *_args):
        pass

    def set_search(self, *_args):
        pass


class ViewPlannerMock:
    def approach_ring(self, _current, target, radius_m):
        pose = Pose2D(target.x - radius_m, target.y, 0.0)
        return [ApproachCandidate(
            pose=pose,
            score=1.0,
            path_length_m=radius_m,
            clearance_m=0.4,
            camera_error_rad=0.0,
            accepted=True,
        )]


class AlternateViewPlannerMock(ViewPlannerMock):
    def approach_ring(self, _current, target, radius_m):
        if radius_m != 0.36:
            return super().approach_ring(_current, target, radius_m)
        return [
            ApproachCandidate(
                pose=Pose2D(target.x - radius_m, target.y, 0.0),
                score=2.0,
                path_length_m=radius_m,
                clearance_m=0.4,
                camera_error_rad=0.0,
                accepted=True,
            ),
            ApproachCandidate(
                pose=Pose2D(target.x, target.y - radius_m, 1.570796),
                score=1.0,
                path_length_m=radius_m + 0.1,
                clearance_m=0.35,
                camera_error_rad=0.0,
                accepted=True,
            ),
        ]


class NavigationMock:
    def __init__(self, start):
        self.current = start
        self.goal = None
        self.state = NavigationState.IDLE
        self.message = ''
        self.operation = ''
        self.calls = []
        self.compute_calls = []
        self.spin_calls = []
        self.clear_count = 0

    def snapshot(self):
        return NavigationSnapshot(
            self.state,
            self.operation,
            self.message,
            None,
            0,
        )

    def compute_path(self, goal, timeout_sec):
        self.goal = goal
        self.compute_calls.append((goal, timeout_sec))
        self.operation = 'COMPUTE_PATH'
        self.state = NavigationState.ACTIVE
        return True

    def navigate_to(self, goal, controller, timeout_sec, precise=False):
        self.goal = goal
        self.calls.append((goal, controller, precise))
        self.operation = 'NAVIGATE'
        self.state = NavigationState.ACTIVE
        return True

    def tick(self):
        if self.state is NavigationState.ACTIVE:
            if self.operation in {'NAVIGATE', 'SPIN'}:
                self.current = self.goal
            self.state = NavigationState.SUCCEEDED
        return self.snapshot()

    def reset(self):
        self.state = NavigationState.IDLE
        self.message = ''
        self.operation = ''

    def cancel(self):
        if self.state is NavigationState.ACTIVE:
            self.state = NavigationState.CANCELED

    def clear_costmaps(self):
        self.clear_count += 1
        return True

    def spin(self, angle_rad, timeout_sec):
        self.spin_calls.append((angle_rad, timeout_sec))
        self.goal = Pose2D(
            self.current.x,
            self.current.y,
            self.current.yaw + angle_rad,
        )
        self.operation = 'SPIN'
        self.state = NavigationState.ACTIVE
        return True


class CameraMock:
    def __init__(self, errors=(0.0,)):
        self.errors = list(errors)
        self.generation = 0

    def observation(self, _color, max_age_sec):
        del max_age_sec
        self.generation += 1
        error = self.errors.pop(0) if len(self.errors) > 1 else self.errors[0]
        return SimpleNamespace(
            horizontal_error_rad=error,
            generation=self.generation,
            cx=320.0 - error * 350.0,
            principal_x=320.0,
        )


class FailFirstNavigationMock(NavigationMock):
    def __init__(self, start):
        super().__init__(start)
        self.failed_once = False
        self.clear_count = 0

    def tick(self):
        if (
            self.state is NavigationState.ACTIVE
            and self.operation == 'NAVIGATE'
            and not self.failed_once
        ):
            self.failed_once = True
            self.state = NavigationState.FAILED
            self.message = 'mock obstacle blocked first approach'
            return self.snapshot()
        return super().tick()

    def clear_costmaps(self):
        self.clear_count += 1
        return True


class FailFirstPlanNavigationMock(NavigationMock):
    def __init__(self, start):
        super().__init__(start)
        self.failed_plan = False
        self.clear_count = 0

    def tick(self):
        if (
            self.state is NavigationState.ACTIVE
            and self.operation == 'COMPUTE_PATH'
            and not self.failed_plan
        ):
            self.failed_plan = True
            self.state = NavigationState.FAILED
            self.message = 'goal lies in a live obstacle'
            return self.snapshot()
        return super().tick()

    def clear_costmaps(self):
        self.clear_count += 1
        return True


class ObjectStoreMock:
    def __init__(self, tracks):
        self.tracks = {track.color: track for track in tracks}
        self.locked = set()

    def confirmed(self, color, **_kwargs):
        track = self.tracks.get(color)
        if track is None or track.object_id in self.locked:
            return None
        return track

    def cached(self, color, **kwargs):
        return self.confirmed(color, **kwargs)

    def lock(self, object_id):
        self.locked.add(object_id)

    def forget_cached(self, _object_id):
        pass


class CachedOnlyObjectStoreMock(ObjectStoreMock):
    def __init__(self, tracks):
        super().__init__(tracks)
        self.forgotten = set()

    def confirmed(self, _color, **_kwargs):
        return None

    def cached(self, color, **_kwargs):
        track = self.tracks.get(color)
        if track is None or track.object_id in self.forgotten:
            return None
        return track

    def forget_cached(self, object_id):
        self.forgotten.add(object_id)


class ManipulatorMock:
    def __init__(self, available=True):
        self._available = available
        self.state = ManipulatorState.IDLE
        self.message = ''
        self.picks = []
        self.places = []

    def available(self):
        return self._available

    def pick(self, object_id, color, target):
        self.picks.append((object_id, color, target))
        self.state = ManipulatorState.ACTIVE
        return True

    def place(self, object_id, color, target, stack_index):
        self.places.append((object_id, color, target, stack_index))
        self.state = ManipulatorState.ACTIVE
        return True

    def tick(self):
        if self.state is ManipulatorState.ACTIVE:
            self.state = ManipulatorState.SUCCEEDED
        return self.state

    def reset(self):
        self.state = ManipulatorState.IDLE
        self.message = ''

    def cancel(self):
        self.state = ManipulatorState.CANCELED


class FailingManipulatorMock(ManipulatorMock):
    def tick(self):
        if self.state is ManipulatorState.ACTIVE:
            self.state = ManipulatorState.FAILED
            self.message = 'mock pick failure'
        return self.state


class SearchMock:
    def __init__(self, _context, _planner, **_kwargs):
        self.state = 'IDLE'
        self.message = ''
        self.color = ''
        self.track = None

    def start(self, color, **_kwargs):
        self.color = color
        self.state = 'SEARCHING'

    def tick(self):
        return task4_module.SearchStatus.RUNNING


def _track(color, x, y):
    return SimpleNamespace(
        object_id=f'{color}-001',
        color=color,
        pose=Pose2D(x, y, 0.0),
        confidence=0.9,
        age_sec=0.05,
    )


def _context(task, tracks, *, available=True, fixed_place_yaw=None,
             color_order=()):
    clock = ClockMock()
    start = Pose2D(-0.5, -0.5, 0.0)
    poses = {
        'A': start,
        'B': Pose2D(0.35, 0.30, 0.0),
        'C': Pose2D(-0.35, 0.40, 3.14159),
    }
    mission = SimpleNamespace(
        task=task,
        time_limit_sec=600.0 if task == 'task5' else 180.0,
        start=start,
        target_color='red',
        color_order=tuple(color_order),
        fixed_place_yaw=fixed_place_yaw,
        pose=lambda name: poses[name],
    )
    navigation = NavigationMock(start)
    context = SimpleNamespace(
        mission=mission,
        navigation=navigation,
        objects=ObjectStoreMock(tracks),
        camera=CameraMock(),
        manipulator=ManipulatorMock(available),
        view_planner=ViewPlannerMock(),
        visualizer=VisualizerMock(),
        logger=LoggerMock(),
        monotonic=clock,
        current_pose=lambda: navigation.current,
        navigation_timeout_sec=75.0,
        camera_confirmation_timeout_sec=4.0,
        camera_center_timeout_sec=15.0,
        camera_observation_max_age_sec=0.75,
        camera_center_tolerance_rad=0.05,
        camera_center_samples=3,
        camera_center_max_corrections=4,
        camera_center_max_step_rad=0.25,
        camera_center_gain=0.55,
        camera_center_settle_sec=0.55,
        front_range=None,
        ultrasonic_required_for_pick=False,
        ultrasonic_max_age_sec=0.35,
        ultrasonic_samples=3,
        ultrasonic_sensor_offset_x_m=0.190,
        ultrasonic_object_half_extent_m=0.0175,
        ultrasonic_match_tolerance_m=0.035,
        ultrasonic_max_target_bearing_rad=0.10,
        camera_view_distance_m=0.36,
        approach_distance_m=0.27,
        drop_approach_distance_m=0.31,
        pick_terminal_min_distance_m=0.24,
        pick_terminal_max_distance_m=0.335,
        pick_terminal_max_bearing_rad=0.55,
        pick_terminal_camera_error_rad=0.58,
        pick_terminal_track_age_sec=1.0,
        drop_terminal_min_distance_m=0.285,
        drop_terminal_max_distance_m=0.345,
        drop_terminal_max_bearing_rad=0.22,
        drop_alignment_timeout_sec=12.0,
        terminal_stop_settle_sec=0.30,
        placed_object_sync_sec=0.35,
        placed_costmap_settle_sec=0.45,
        camera_retry_separation_m=0.20,
        manipulation_retry_separation_m=0.14,
        final_yaw_tolerance_rad=0.10,
        robot_clearance_m=0.205,
        search_timeout_sec=85.0,
        max_search_viewpoints=12,
        task5_search_timeout_sec=180.0,
        task5_max_search_viewpoints=24,
        search_candidate_pool=60,
        search_rejected_radius_m=0.16,
        search_plan_timeout_sec=5.0,
        transport_plan_timeout_sec=3.0,
        camera_approach_candidate_limit=8,
        manipulation_approach_candidate_limit=12,
        camera_dwell_sec=0.8,
        visual_center_events=[],
        grid=SimpleNamespace(),
    )
    return context, clock


def _run(solver, clock, limit=120):
    snapshot = solver.start()
    for _ in range(limit):
        if snapshot.status is not SolverStatus.RUNNING:
            break
        clock.advance()
        snapshot = solver.tick()
    return snapshot


def test_task3_executes_confirmed_pick_and_exact_center_place():
    context, clock = _context('task3', [_track('red', 0.35, 0.30)])

    result = _run(Task3Solver(context), clock)

    assert result.status is SolverStatus.SUCCEEDED
    assert [call[2] for call in context.navigation.calls] == [True, True, True]
    assert [item[1] for item in context.manipulator.picks] == ['red']
    assert len(context.visual_center_events) == 1
    assert context.visual_center_events[0]['color'] == 'red'
    assert abs(
        context.visual_center_events[0]['horizontal_error_rad']
    ) <= context.camera_center_tolerance_rad
    assert context.manipulator.places[0][2] == context.mission.pose('C')
    assert context.manipulator.places[0][3] == 0
    assert context.navigation.clear_count == 1


def test_task3_fails_honestly_without_manipulator_adapter():
    context, clock = _context(
        'task3',
        [_track('red', 0.35, 0.30)],
        available=False,
    )

    result = _run(Task3Solver(context), clock)

    assert result.status is SolverStatus.FAILED
    assert result.state == 'MANIPULATOR_UNAVAILABLE'
    assert context.navigation.calls == []


def test_task4_uses_fixed_bonus_orientation_at_c():
    context, clock = _context(
        'task4',
        [_track('red', 0.20, 0.15)],
        fixed_place_yaw=0.25,
    )

    result = _run(Task4Solver(context), clock)

    assert result.status is SolverStatus.SUCCEEDED
    placed_pose = context.manipulator.places[0][2]
    assert placed_pose.x == context.mission.pose('C').x
    assert placed_pose.y == context.mission.pose('C').y
    assert placed_pose.yaw == 0.25


def test_task5_keeps_color_order_and_stack_indices_for_cached_tracks():
    order = ('yellow', 'red', 'blue')
    tracks = [
        _track('red', 0.20, 0.15),
        _track('blue', 0.10, -0.20),
        _track('yellow', -0.05, 0.20),
    ]
    context, clock = _context('task5', tracks, color_order=order)

    result = _run(Task5Solver(context), clock, limit=300)

    assert result.status is SolverStatus.SUCCEEDED
    assert [item[1] for item in context.manipulator.picks] == list(order)
    assert [item[1] for item in context.manipulator.places] == list(order)
    assert [item[3] for item in context.manipulator.places] == [0, 1, 2]
    assert [
        item['color'] for item in context.visual_center_events
    ] == list(order)
    assert all(
        item[2].x == context.mission.pose('C').x
        and item[2].y == context.mission.pose('C').y
        for item in context.manipulator.places
    )


def test_task3_uses_alternative_camera_approach_after_nav_abort():
    context, clock = _context('task3', [_track('red', 0.35, 0.30)])
    navigation = FailFirstNavigationMock(context.mission.start)
    context.navigation = navigation
    context.current_pose = lambda: navigation.current
    context.view_planner = AlternateViewPlannerMock()

    result = _run(Task3Solver(context), clock)

    assert result.status is SolverStatus.SUCCEEDED
    assert navigation.failed_once
    assert navigation.clear_count == 2
    assert navigation.calls[0][0] != navigation.calls[1][0]


def test_task3_rejects_blocked_live_costmap_approach_before_driving():
    context, clock = _context('task3', [_track('red', 0.35, 0.30)])
    navigation = FailFirstPlanNavigationMock(context.mission.start)
    context.navigation = navigation
    context.current_pose = lambda: navigation.current
    context.view_planner = AlternateViewPlannerMock()

    result = _run(Task3Solver(context), clock, limit=180)

    assert result.status is SolverStatus.SUCCEEDED
    assert navigation.failed_plan
    assert navigation.compute_calls[0][0] != navigation.calls[0][0]
    assert navigation.clear_count == 1


def test_task3_visually_centers_block_before_pick():
    context, clock = _context('task3', [_track('red', 0.35, 0.30)])
    context.camera = CameraMock(
        errors=(0.20, 0.20, 0.20, 0.20, 0.02, 0.01, 0.00)
    )

    result = _run(Task3Solver(context), clock, limit=180)

    assert result.status is SolverStatus.SUCCEEDED
    assert context.navigation.spin_calls
    assert 0.0 < context.navigation.spin_calls[0][0] <= 0.25
    assert len(context.manipulator.picks) == 1


def test_failed_pick_does_not_publish_successful_centering_evidence():
    context, clock = _context('task3', [_track('red', 0.35, 0.30)])
    context.manipulator = FailingManipulatorMock()

    result = _run(Task3Solver(context), clock, limit=180)

    assert result.status is SolverStatus.FAILED
    assert context.visual_center_events == []


def test_task4_rejects_stale_cache_and_resumes_active_search(monkeypatch):
    track = _track('red', 0.20, 0.15)
    context, clock = _context('task4', [track])
    context.objects = CachedOnlyObjectStoreMock([track])
    monkeypatch.setattr(
        task4_module,
        'NextBestViewPlanner',
        lambda *_args, **_kwargs: object(),
    )
    monkeypatch.setattr(task4_module, 'ActiveSearch', SearchMock)
    solver = Task4Solver(context)
    snapshot = solver.start()

    for _ in range(180):
        if track.object_id in context.objects.forgotten:
            break
        clock.advance()
        snapshot = solver.tick()

    assert snapshot.status is SolverStatus.RUNNING
    assert track.object_id in context.objects.forgotten
    assert solver.search is not None
    assert solver.search.color == 'red'
    assert solver.transport is None


def test_task5_rejects_stale_cache_without_changing_required_color(monkeypatch):
    tracks = [
        _track('red', 0.20, 0.15),
        _track('blue', 0.10, -0.20),
        _track('yellow', -0.05, 0.20),
    ]
    context, clock = _context(
        'task5',
        tracks,
        color_order=('red', 'blue', 'yellow'),
    )
    context.objects = CachedOnlyObjectStoreMock(tracks)
    monkeypatch.setattr(
        task5_module,
        'NextBestViewPlanner',
        lambda *_args, **_kwargs: object(),
    )
    monkeypatch.setattr(task5_module, 'ActiveSearch', SearchMock)
    solver = Task5Solver(context)
    snapshot = solver.start()

    for _ in range(180):
        if tracks[0].object_id in context.objects.forgotten:
            break
        clock.advance()
        snapshot = solver.tick()

    assert snapshot.status is SolverStatus.RUNNING
    assert tracks[0].object_id in context.objects.forgotten
    assert solver.index == 0
    assert solver.search is not None
    assert solver.search.color == 'red'


def test_retry_selection_prefers_spatially_distinct_approaches():
    def candidate(x, y, score):
        return ApproachCandidate(
            pose=Pose2D(x, y, 0.0),
            score=score,
            path_length_m=1.0,
            clearance_m=0.4,
            camera_error_rad=0.0,
            accepted=True,
        )

    candidates = [
        candidate(0.00, 0.00, 4.0),
        candidate(0.05, 0.00, 3.0),
        candidate(0.30, 0.00, 2.0),
    ]

    selected = select_diverse_approaches(candidates, 2, 0.20)

    assert [item.pose.x for item in selected] == [0.00, 0.30]


def test_pick_navigation_stops_when_live_camera_track_enters_valid_annulus():
    track = _track('red', 0.35, 0.30)
    context, clock = _context('task3', [track])
    context.navigation.current = Pose2D(0.08, 0.30, 0.0)
    context.navigation.state = NavigationState.ACTIVE
    context.navigation.operation = 'NAVIGATE'
    flow = TransportFlow(context)
    flow.color = 'red'
    flow.estimated_pose = track.pose
    flow.pick_approaches = [SimpleNamespace(pose=context.navigation.current)]
    flow.state = 'WAIT_PICK_APPROACH'
    flow.state_started = clock()

    flow.tick()

    assert flow.state == 'WAIT_PICK_TERMINAL_STOP'
    assert context.navigation.state is NavigationState.CANCELED
    clock.advance(0.31)
    flow.tick()
    assert flow.state == 'START_VISUAL_CENTER'


def test_pick_terminal_uses_reach_margin_without_exceeding_manipulator_limit():
    track = _track('red', 0.35, 0.30)
    context, _clock = _context('task3', [track])
    flow = TransportFlow(context)
    flow.color = 'red'
    flow.estimated_pose = track.pose

    context.navigation.current = Pose2D(0.02, 0.30, 0.0)
    assert flow._pick_terminal() is not None

    context.navigation.current = Pose2D(0.009, 0.30, 0.0)
    assert flow._pick_terminal() is None


def test_drop_transit_faces_travel_direction_then_aligns_to_center():
    context, clock = _context('task3', [_track('red', 0.35, 0.30)])
    flow = TransportFlow(context)
    flow.color = 'red'
    flow.drop_pose = Pose2D(0.0, 0.55, 0.0)

    context.navigation.current = Pose2D(-0.17, 0.50, 0.0)
    ring_pose = Pose2D(-0.31, 0.55, 0.0)
    transit = flow._forward_transit_pose(ring_pose)

    assert transit.x == ring_pose.x
    assert transit.y == ring_pose.y
    assert transit.yaw == pytest.approx(math.atan2(0.05, -0.14))
    assert abs(transit.yaw - ring_pose.yaw) > 1.0

    context.navigation.current = Pose2D(-0.31, 0.55, math.pi)
    flow.state = 'START_DROP_ALIGNMENT'
    flow.state_started = clock()

    flow.tick()

    assert flow.state == 'WAIT_DROP_ALIGNMENT'
    assert context.navigation.spin_calls[-1][0] == pytest.approx(-math.pi)

    clock.advance()
    flow.tick()

    assert flow.state == 'START_PLACE'


def test_centered_pick_requires_matching_ultrasonic_range_when_enabled():
    track = _track('red', 0.35, 0.30)
    context, clock = _context('task3', [track])
    context.navigation.current = Pose2D(0.08, 0.30, 0.0)
    context.ultrasonic_required_for_pick = True
    reading = FrontRangeReading(
        range_m=0.063,
        min_range_m=0.02,
        max_range_m=1.20,
        field_of_view_rad=0.2618,
        frame_id='ultrasonic_front_link',
        generation=7,
        sample_count=4,
        received_wall=clock(),
    )
    context.front_range = SimpleNamespace(
        measurement=lambda **_kwargs: reading
    )
    flow = TransportFlow(context)
    flow.color = 'red'
    flow.object_id = track.object_id
    flow.estimated_pose = track.pose
    flow.pick_approaches = [SimpleNamespace(pose=context.navigation.current)]
    flow.pick_index = 0
    flow.range_verify_after_wall = clock()
    flow.visual_alignment.centered_observation = SimpleNamespace(
        horizontal_error_rad=0.0,
        generation=5,
        stamp_sec=1.0,
        cx=320.0,
        principal_x=320.0,
    )
    flow.state = 'FINAL_VERIFY_OBJECT'
    flow.state_started = clock()

    flow.tick()

    assert flow.state == 'START_PICK'
    assert flow.pending_center_evidence['ultrasonic_used']
    assert flow.pending_center_evidence['ultrasonic_error_m'] == pytest.approx(
        0.0005
    )


def test_final_facing_yaw_uses_actual_robot_position():
    context, _clock = _context('task3', [_track('red', 0.35, 0.30)])
    context.navigation.current = Pose2D(0.10, 0.10, -1.0)

    facing = TransportFlow(context)._face_target(
        Pose2D(0.10, 0.40, 0.0),
        Pose2D(0.00, 0.00, 0.0),
    )

    assert facing.x == 0.10
    assert facing.y == 0.10
    assert facing.yaw == 1.5707963267948966
