from types import SimpleNamespace

import pytest

import imdl_autonomy.ros_shutdown as ros_runtime


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class _Node:
    def __init__(self):
        self.context = object()
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


class _Executor:
    def __init__(self):
        self.calls = 0

    def spin_once(self, timeout_sec):
        assert timeout_sec == pytest.approx(0.2)
        self.calls += 1
        if self.calls == 1:
            raise RuntimeError(
                'Unable to convert call argument to Python object'
            )
        raise KeyboardInterrupt


class _RebuildableExecutor:
    instances = []

    def __init__(self, context=None):
        self.context = context
        self.calls = 0
        self.node = None
        self.removed = False
        self.stopped = False
        self.__class__.instances.append(self)

    def add_node(self, node):
        self.node = node

    def remove_node(self, node):
        assert node is self.node
        self.removed = True

    def shutdown(self, timeout_sec=None):
        self.stopped = True
        return True

    def spin_once(self, timeout_sec):
        self.calls += 1
        if len(self.__class__.instances) == 1:
            raise RuntimeError(
                'Unable to convert call argument to Python object'
            )
        raise KeyboardInterrupt


class _ShutdownDuringTakeExecutor(_RebuildableExecutor):
    context_ok = True

    def spin_once(self, timeout_sec):
        self.calls += 1
        self.__class__.context_ok = False
        raise RuntimeError(
            'Unable to convert call argument to Python object'
        )


def test_resilient_spin_survives_one_transient_take_error(monkeypatch):
    node = _Node()
    executor = _Executor()
    monkeypatch.setattr(
        ros_runtime.rclpy,
        'ok',
        lambda **_kwargs: True,
    )
    monkeypatch.setattr(ros_runtime.time, 'sleep', lambda _seconds: None)

    with pytest.raises(KeyboardInterrupt):
        ros_runtime.spin_node_resilient(node, executor)

    assert executor.calls == 2
    assert len(node.logger.warnings) == 1
    assert 'transient DDS conversion failure' in node.logger.warnings[0]


def test_resilient_spin_does_not_hide_unrelated_runtime_errors(monkeypatch):
    node = _Node()
    executor = SimpleNamespace(
        spin_once=lambda **_kwargs: (_ for _ in ()).throw(
            RuntimeError('unrelated failure')
        )
    )
    monkeypatch.setattr(
        ros_runtime.rclpy,
        'ok',
        lambda **_kwargs: True,
    )

    with pytest.raises(RuntimeError, match='unrelated failure'):
        ros_runtime.spin_node_resilient(node, executor)


def test_owned_executor_is_rebuilt_after_transient_take_error(monkeypatch):
    node = _Node()
    _RebuildableExecutor.instances = []
    monkeypatch.setattr(
        ros_runtime,
        'DiagnosticSingleThreadedExecutor',
        _RebuildableExecutor,
    )
    monkeypatch.setattr(
        ros_runtime.rclpy,
        'ok',
        lambda **_kwargs: True,
    )
    monkeypatch.setattr(ros_runtime.time, 'sleep', lambda _seconds: None)

    with pytest.raises(KeyboardInterrupt):
        ros_runtime.spin_node_resilient(node)

    assert len(_RebuildableExecutor.instances) == 2
    first, second = _RebuildableExecutor.instances
    assert first.removed
    assert first.stopped
    assert second.node is node
    assert second.stopped


def test_owned_executor_is_not_rebuilt_after_context_shutdown(monkeypatch):
    node = _Node()
    _ShutdownDuringTakeExecutor.instances = []
    _ShutdownDuringTakeExecutor.context_ok = True
    monkeypatch.setattr(
        ros_runtime,
        'DiagnosticSingleThreadedExecutor',
        _ShutdownDuringTakeExecutor,
    )
    monkeypatch.setattr(
        ros_runtime.rclpy,
        'ok',
        lambda **_kwargs: _ShutdownDuringTakeExecutor.context_ok,
    )

    ros_runtime.spin_node_resilient(node)

    assert len(_ShutdownDuringTakeExecutor.instances) == 1
    assert _ShutdownDuringTakeExecutor.instances[0].stopped
    assert node.logger.warnings == []
