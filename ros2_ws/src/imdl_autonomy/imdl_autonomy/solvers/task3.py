"""Task 3 autonomy: known object center with camera-confirmed pick and place."""

from .base import BaseSolver
from .transport import FlowStatus, TransportFlow


class Task3Solver(BaseSolver):

    def __init__(self, context):
        super().__init__(context)
        self.transport = TransportFlow(context)

    def on_start(self) -> None:
        if not self.context.manipulator.available():
            self._fail(
                'MANIPULATOR_UNAVAILABLE',
                'Tasks 3-5 require the Gazebo adapter or Member C hardware adapter',
            )
            return
        self.transport.start(
            color=self.context.mission.target_color,
            estimated_pose=self.context.mission.pose('B'),
            drop_pose=self.context.mission.pose('C'),
            require_confirmation=True,
        )
        self._transition('KNOWN_OBJECT_TRANSPORT', 'B is object center')

    def on_tick(self) -> None:
        status = self.transport.tick()
        self.context.visualizer.set_solver_state(
            self.transport.state,
            self.transport.message,
            self.attempt,
        )
        if status is FlowStatus.SUCCEEDED:
            self._succeed('Known object picked and placed at C')
        elif status is FlowStatus.FAILED:
            self._fail('TASK3_TRANSPORT_FAILED', self.transport.message)
