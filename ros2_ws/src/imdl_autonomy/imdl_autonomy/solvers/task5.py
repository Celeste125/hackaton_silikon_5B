"""Task 5 autonomy: strict color-order search and three-object transport."""

from .base import BaseSolver
from .search import ActiveSearch, SearchStatus
from .transport import FlowStatus, TransportFlow
from ..next_best_view import NextBestViewPlanner


class Task5Solver(BaseSolver):

    def __init__(self, context):
        super().__init__(context)
        self.order = []
        self.index = 0
        self.search = None
        self.transport = None
        self.search_planner = None
        self.transport_from_cache = False
        self.cache_fallback_colors = set()

    def on_start(self) -> None:
        if not self.context.manipulator.available():
            self._fail(
                'MANIPULATOR_UNAVAILABLE',
                'Tasks 3-5 require the Gazebo adapter or Member C hardware adapter',
            )
            return
        self.order = list(self.context.mission.color_order)
        self._transition('SELECT_NEXT_COLOR', f'order={self.order}')

    def on_tick(self) -> None:
        if self.state == 'SELECT_NEXT_COLOR':
            self._select_next_color()
        elif self.search is not None:
            self._tick_search()
        elif self.transport is not None:
            self._tick_transport()

    def _select_next_color(self) -> None:
        if self.index >= len(self.order):
            self._succeed('All three objects placed at C in required order')
            return
        color = self.order[self.index]
        track = self.context.objects.confirmed(color)
        from_cache = False
        if track is None:
            track = self.context.objects.cached(color)
            from_cache = track is not None
        if track is not None:
            self._start_transport(track, from_cache=from_cache)
            return
        self._start_search(color)

    def _start_search(self, color) -> None:
        if self.search_planner is None:
            self.search_planner = NextBestViewPlanner(
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
        self.search = ActiveSearch(
            self.context,
            self.search_planner,
            timeout_sec=self.context.task5_search_timeout_sec,
            max_viewpoints=self.context.task5_max_search_viewpoints,
        )
        remaining_future = len(self.order) - self.index - 1
        self.search.start(
            color,
            future_color_bonus=0.12 * remaining_future,
        )
        self._transition(
            'SEARCH_REQUIRED_COLOR',
            f'{self.index + 1}/3: {color}',
        )

    def _tick_search(self) -> None:
        status = self.search.tick()
        self.context.visualizer.set_solver_state(
            self.search.state,
            self.search.message,
            self.attempt,
        )
        if status is SearchStatus.FAILED:
            self._fail(
                'TASK5_SEARCH_FAILED',
                f'{self.order[self.index]}: {self.search.message}',
            )
        elif status is SearchStatus.FOUND:
            track = self.search.track
            self.search = None
            self._start_transport(track, from_cache=False)

    def _start_transport(self, track, from_cache: bool) -> None:
        expected_color = self.order[self.index]
        if track.color != expected_color:
            self._fail(
                'COLOR_ORDER_VIOLATION',
                f'Expected {expected_color}, selected {track.color}',
            )
            return
        self.search = None
        self.transport_from_cache = bool(from_cache)
        self.transport = TransportFlow(self.context)
        self.transport.start(
            color=track.color,
            object_id=track.object_id,
            estimated_pose=track.pose,
            drop_pose=self.context.mission.pose('C'),
            stack_index=self.index,
            require_confirmation=False,
        )
        self._transition(
            'TRANSPORT_REQUIRED_COLOR',
            f'{self.index + 1}/3: {track.color}',
        )

    def _tick_transport(self) -> None:
        status = self.transport.tick()
        self.context.visualizer.set_solver_state(
            self.transport.state,
            self.transport.message,
            self.attempt,
        )
        if status is FlowStatus.FAILED:
            color = self.order[self.index]
            if (
                self.transport_from_cache
                and not self.transport.picked
                and color not in self.cache_fallback_colors
            ):
                stale_id = self.transport.object_id
                self.context.objects.forget_cached(stale_id)
                self.cache_fallback_colors.add(color)
                self.transport = None
                self._start_search(color)
                self._transition(
                    'CACHE_HYPOTHESIS_REJECTED',
                    f'{color}/{stale_id}: fresh confirmation failed',
                )
                return
            self._fail(
                'TASK5_TRANSPORT_FAILED',
                f'{color}: {self.transport.message}',
            )
        elif status is FlowStatus.SUCCEEDED:
            self.index += 1
            self.transport = None
            self.transport_from_cache = False
            self._transition('SELECT_NEXT_COLOR')
