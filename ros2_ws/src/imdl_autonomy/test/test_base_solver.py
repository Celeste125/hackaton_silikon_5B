from types import SimpleNamespace

from imdl_autonomy.solvers.base import BaseSolver, SolverStatus


class SolverUnderTest(BaseSolver):

    def __init__(self, context):
        super().__init__(context)
        self.cancel_cleanup_calls = 0

    def on_start(self):
        self._transition('RUNNING')

    def on_tick(self):
        pass

    def cancel(self):
        self.cancel_cleanup_calls += 1
        return super().cancel()


class CancelCounter:

    def __init__(self):
        self.calls = 0

    def cancel(self):
        self.calls += 1


def test_timeout_uses_complete_solver_cancellation_path():
    clock = {'value': 0.0}
    navigation = CancelCounter()
    manipulator = CancelCounter()
    context = SimpleNamespace(
        mission=SimpleNamespace(time_limit_sec=1.0),
        navigation=navigation,
        manipulator=manipulator,
        monotonic=lambda: clock['value'],
    )
    solver = SolverUnderTest(context)
    solver.start()
    clock['value'] = 1.1

    result = solver.tick()

    assert result.status is SolverStatus.FAILED
    assert result.state == 'TIMEOUT'
    assert solver.cancel_cleanup_calls == 1
    assert navigation.calls == 1
    assert manipulator.calls == 1
