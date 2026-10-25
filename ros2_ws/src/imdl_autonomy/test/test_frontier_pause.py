"""Run the explorer's actual asynchronous handoff methods with fake actions."""

import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def explorer():
    source = Path(__file__).parents[1] / 'imdl_autonomy' / 'frontier_explorer.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    names = {'_submit_goal', '_pause_cb', '_hold_cb', '_cancel_for_pause', '_publish_pause_state'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = {'Bool': lambda data: data}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
    harness = type('ExplorerHandoff', (), {name: namespace[name] for name in names})()
    harness._pause_requested = False
    harness._pending_goals = 0
    harness._active_handles = {}
    harness._cancel_requested = set()
    harness.requests, harness.events, harness.acks = [], [], []

    def send(goal):
        future = Future()
        harness.requests.append(future)
        return future

    harness._nav_client = SimpleNamespace(send_goal_async=send)
    harness._paused_pub = SimpleNamespace(publish=harness.acks.append)
    harness.get_logger = lambda: SimpleNamespace(error=lambda msg: None)
    harness._reset_nav_state = lambda cooldown: harness.events.append('reset')
    return harness


def handle(number=1, accepted=True):
    result = Future()
    cancellations = []
    return SimpleNamespace(
        accepted=accepted, goal_id=SimpleNamespace(uuid=[number] * 16),
        get_result_async=lambda: result,
        cancel_goal_async=lambda: cancellations.append('cancel'),
        result=result, cancellations=cancellations)


def submit(explorer):
    explorer._submit_goal(None, lambda f: explorer.events.append('accepted'),
                          lambda f: explorer.events.append('finished'))


def test_pause_catches_goal_accepted_after_request(explorer):
    submit(explorer)
    explorer._pause_cb(None)
    explorer._publish_pause_state()
    assert explorer.acks[-1] is False
    goal = handle()
    explorer.requests[0].set_result(goal)
    assert goal.cancellations == ['cancel']
    explorer._publish_pause_state()
    assert explorer.acks[-1] is False  # cancel request is NOT termination
    goal.result.set_result(SimpleNamespace(status=5))
    explorer._publish_pause_state()
    assert explorer.acks[-1] is True
    assert explorer.events == []  # a late callback cannot restart exploration


def test_pause_waits_for_every_goal_including_return_home(explorer):
    goals = [handle(1), handle(2)]
    for goal in goals:
        submit(explorer)
        explorer.requests[-1].set_result(goal)
    explorer._pause_cb(None)
    explorer._pause_cb(None)
    assert all(g.cancellations == ['cancel'] for g in goals)
    goals[0].result.set_result(None)
    explorer._publish_pause_state()
    assert not explorer.acks[-1]
    goals[1].result.set_result(None)
    explorer._publish_pause_state()
    assert explorer.acks[-1]
    submit(explorer)
    assert len(explorer.requests) == 2


def test_rejected_goal_and_idle_explorer_can_ack_pause(explorer):
    submit(explorer)
    explorer._pause_cb(None)
    explorer.requests[0].set_result(handle(accepted=False))
    explorer._publish_pause_state()
    assert explorer.acks[-1]


def test_transport_error_does_not_claim_the_robot_stopped(explorer):
    submit(explorer)
    explorer._pause_cb(None)
    explorer.requests[0].set_exception(RuntimeError('connection lost'))
    explorer._publish_pause_state()
    assert not explorer.acks[-1]


def test_normal_navigation_still_receives_response_and_result(explorer):
    submit(explorer)
    goal = handle()
    explorer.requests[0].set_result(goal)
    goal.result.set_result(None)
    assert explorer.events == ['accepted', 'finished']
    assert not explorer._active_handles


def test_resume_waits_for_pending_goal_and_terminal_result(explorer):
    submit(explorer)
    explorer._hold_cb(SimpleNamespace(data=True))
    explorer._hold_cb(SimpleNamespace(data=False))
    assert explorer._pause_requested
    goal = handle()
    explorer.requests[0].set_result(goal)
    explorer._hold_cb(SimpleNamespace(data=False))
    assert explorer._pause_requested
    goal.result.set_result(None)
    explorer._hold_cb(SimpleNamespace(data=False))
    assert not explorer._pause_requested
    assert not explorer._exploration_done
    assert explorer.events == ['reset']
    explorer._hold_cb(SimpleNamespace(data=False))
    assert explorer.events == ['reset']  # repeated release does not reset new goals
    submit(explorer)
    assert len(explorer.requests) == 2
