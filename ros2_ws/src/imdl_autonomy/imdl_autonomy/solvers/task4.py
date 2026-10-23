"""Task 4 autonomy: active search for one unknown object, then transport."""

from .base import BaseSolver
from .search import ActiveSearch, SearchStatus
from .transport import FlowStatus, TransportFlow
from ..mission_spec import Pose2D
from ..next_best_view import NextBestViewPlanner


class Task4Solver(BaseSolver):

    def __init__(self, context):
        super().__init__(context)
        self.search = None
        self.transport = None
        self.transport_from_cache = False
        self.cache_fallback_used = False

    def on_start(self) -> None:
        if not self.context.manipulator.available():
            self._fail(
                'MANIPULATOR_UNAVAILABLE',
                'Tasks 3-5 require the Gazebo adapter or Member C hardware adapter',
            )
            return
        track = self.context.objects.confirmed(
            self.context.mission.target_color
        )
        from_cache = False
        if track is None:
            track = self.context.objects.cached(
                self.context.mission.target_color
            )
            from_cache = track is not None
        if track is not None:
            self._start_transport(track, from_cache=from_cache)
            self._transition(
                (
                    'TRANSPORT_CACHED_OBJECT'
                    if from_cache
                    else 'TRANSPORT_CONFIRMED_OBJECT'
                ),
                f'track={track.object_id}',
            )
            return
        self._start_search()

    def _start_search(self) -> None:
        planner = NextBestViewPlanner(
            self.context.grid,
            robot_clearance_m=self.context.robot_clearance_m,
            view_clearance_margin_m=(
                getattr(
                    self.context,
                    'search_view_clearance_margin_m',
                    0.10,
                )
            ),
        )
        self.search = ActiveSearch(self.context, planner)
        self.search.start(self.context.mission.target_color)
        self._transition('ACTIVE_SEARCH', 'object position is not read from Gazebo')

    def on_tick(self) -> None:
        if self.transport is None:
            status = self.search.tick()
            self.context.visualizer.set_solver_state(
                self.search.state,
                self.search.message,
                self.attempt,
            )
            if status is SearchStatus.FAILED:
                self._fail('TASK4_SEARCH_FAILED', self.search.message)
            elif status is SearchStatus.FOUND:
                self._start_transport(self.search.track, from_cache=False)
                self._transition('TRANSPORT_FOUND_OBJECT')
            return

        status = self.transport.tick()
        self.context.visualizer.set_solver_state(
            self.transport.state,
            self.transport.message,
            self.attempt,
        )
        if status is FlowStatus.SUCCEEDED:
            self._succeed('Unknown object found and placed at C')
        elif status is FlowStatus.FAILED:
            if (
                self.transport_from_cache
                and not self.transport.picked
                and not self.cache_fallback_used
            ):
                stale_id = self.transport.object_id
                self.context.objects.forget_cached(stale_id)
                self.cache_fallback_used = True
                self.transport = None
                self._start_search()
                self._transition(
                    'CACHE_HYPOTHESIS_REJECTED',
                    f'{stale_id}: fresh camera confirmation failed',
                )
                return
            self._fail('TASK4_TRANSPORT_FAILED', self.transport.message)

    def _start_transport(self, track, from_cache: bool) -> None:
        drop = self.context.mission.pose('C')
        fixed_yaw = self.context.mission.fixed_place_yaw
        if fixed_yaw is not None:
            drop = Pose2D(drop.x, drop.y, fixed_yaw)
        self.search = None
        self.transport_from_cache = bool(from_cache)
        self.transport = TransportFlow(self.context)
        self.transport.start(
            color=track.color,
            object_id=track.object_id,
            estimated_pose=track.pose,
            drop_pose=drop,
            require_confirmation=False,
        )
