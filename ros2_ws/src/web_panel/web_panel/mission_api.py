"""Mission tab and dock back end.

The run itself is feed_mission's mission node; this only passes settings to
it, starts and stops it and reports /mission/progress. Dock and undock are
the Create 3's own actions (<ns>/dock, <ns>/undock), started from the panel
without waiting for them to finish; /<ns>/dock_status says whether the robot
is on the dock and sees it.
"""

import json
import math
import threading
import time

from pydantic import BaseModel, Field
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from feed_mission import poses as pose_store
from feed_mission.mission import STEPS

try:
    from irobot_create_msgs.action import Dock, Undock
    from irobot_create_msgs.msg import DockStatus
except ImportError:
    Dock = Undock = DockStatus = None

DOCK_STALE_S = 3.0


class MissionSettings(BaseModel):
    steps: list[str] = Field(default_factory=lambda: list(STEPS))
    dry_run: bool = True
    recipe_kg: float = Field(1.0, ge=0.0, le=20.0)
    recipe: list[float] = Field(default_factory=lambda: [40.0, 40.0, 20.0], min_length=3, max_length=3)
    valve_percent: float = Field(100.0, ge=5.0, le=100.0)
    mix_s: float = Field(60.0, ge=0.0, le=900.0)
    mix_speed: float = Field(0.6, ge=0.0, le=1.0)
    laps_max: int = Field(6, ge=1, le=50)
    empty_kg: float = Field(0.05, ge=0.0, le=5.0)


def _param(name, value):
    v = ParameterValue()
    if isinstance(value, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, int):
        v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
    elif isinstance(value, float):
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
    elif isinstance(value, list) and all(isinstance(x, str) for x in value):
        v.type, v.string_array_value = ParameterType.PARAMETER_STRING_ARRAY, value
    elif isinstance(value, list):
        v.type, v.double_array_value = ParameterType.PARAMETER_DOUBLE_ARRAY, [float(x) for x in value]
    else:
        v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
    return Parameter(name=name, value=v)


class MissionApi:
    def __init__(self, node, poses_file, namespace='/Team2'):
        self.node = node
        self.poses_file = poses_file
        ns = namespace.rstrip('/')
        self.lock = threading.Lock()
        self.progress = None
        self.dock_status = None      # (is_docked, visible, monotonic time)
        self.dock_busy = None        # 'docking' / 'undocking' while an action runs
        self.dock_result = None
        self.dock_goal = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, 'mission/progress', self._on_progress, latched)
        self.params = node.create_client(SetParameters, '/mission/set_parameters')
        self.start_client = node.create_client(Trigger, 'mission/start')
        self.stop_client = node.create_client(Trigger, 'mission/stop')
        self.initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        if DockStatus is not None:
            node.create_subscription(DockStatus, f'{ns}/dock_status', self._on_dock,
                                     qos_profile_sensor_data)
            self.dock_client = ActionClient(node, Dock, f'{ns}/dock')
            self.undock_client = ActionClient(node, Undock, f'{ns}/undock')

    def _on_progress(self, msg):
        try:
            p = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.progress = p

    def _on_dock(self, msg):
        with self.lock:
            self.dock_status = (bool(msg.is_docked), bool(msg.dock_visible), time.monotonic())

    def docked(self):
        with self.lock:
            s = self.dock_status
        return bool(s and s[0] and time.monotonic() - s[2] < DOCK_STALE_S)

    # ---- mission ------------------------------------------------------------

    def _call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None

    def start(self, s):
        req = SetParameters.Request()
        steps = [x for x in s.steps if x in STEPS]
        req.parameters = [
            _param('steps', steps), _param('dry_run', bool(s.dry_run)),
            _param('recipe_kg', float(s.recipe_kg)), _param('recipe', [float(x) for x in s.recipe]),
            _param('valve_percent', float(s.valve_percent)), _param('mix_s', float(s.mix_s)),
            _param('mix_speed', float(s.mix_speed)), _param('laps_max', int(s.laps_max)),
            _param('empty_kg', float(s.empty_kg)),
        ]
        res = self._call(self.params, req)
        if res is None:
            return False, 'the mission node is not running: ros2 launch feed_mission mission.launch.py'
        bad = [r.reason for r in res.results if not r.successful]
        if bad:
            return False, '; '.join(bad)
        pose_store.save_section(pose_store.MISSION_KEY, s.model_dump(), path=self.poses_file)
        res = self._call(self.start_client, Trigger.Request())
        if res is None:
            return False, 'the mission node did not answer'
        return res.success, res.message

    def stop(self, wait=True):
        """wait=False: fire and forget, for STOP ALL."""
        if self.dock_goal is not None:
            self.dock_goal.cancel_goal_async()
        if not wait:
            if self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
            return True, ''
        res = self._call(self.stop_client, Trigger.Request())
        return (True, res.message) if res else (False, 'the mission node is not running')

    # ---- dock ---------------------------------------------------------------

    def run_dock_action(self, undock):
        if DockStatus is None:
            return False, 'irobot_create_msgs is not installed'
        client = self.undock_client if undock else self.dock_client
        word = 'undocking' if undock else 'docking'
        with self.lock:
            if self.dock_busy:
                return False, f'already {self.dock_busy}'
        if not client.wait_for_server(timeout_sec=2.0):
            return False, f'the Create 3 {word} action is not available'
        with self.lock:
            self.dock_busy, self.dock_result = word, None
        goal = Undock.Goal() if undock else Dock.Goal()
        client.send_goal_async(goal).add_done_callback(lambda f: self._on_goal(f, word))
        return True, f'{word.capitalize()}…'

    def _on_goal(self, future, word):
        handle = future.result()
        if not handle.accepted:
            with self.lock:
                self.dock_busy, self.dock_result = None, f'{word} rejected'
            return
        self.dock_goal = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, word))

    def _on_result(self, future, word):
        self.dock_goal = None
        try:
            docked = future.result().result.is_docked
            text = 'docked' if docked else 'undocked'
            ok = docked == (word == 'docking')
        except Exception:
            text, ok = f'{word} failed', False
        with self.lock:
            self.dock_busy, self.dock_result = None, text if ok else f'{word} failed ({text})'

    def localize_from_dock(self):
        """While docked: tell AMCL the robot is exactly at the taught docked pose."""
        p = pose_store.load(self.poses_file).get('docked')
        if p is None:
            return False, 'no docked pose saved yet: dock, then Save here at "Docked"'
        if not self.docked():
            return False, 'the robot is not on the dock'
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = p['x'], p['y']
        msg.pose.pose.orientation.z = math.sin(p['yaw'] / 2)
        msg.pose.pose.orientation.w = math.cos(p['yaw'] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02 ** 2
        cov[35] = math.radians(2) ** 2
        msg.pose.covariance = cov
        for _ in range(3):
            self.initialpose_pub.publish(msg)
        return True, 'Position set from the docked pose'

    def state(self):
        with self.lock:
            s = self.dock_status
            dock = None
            if s is not None:
                fresh = time.monotonic() - s[2] < DOCK_STALE_S
                dock = {'docked': s[0] and fresh, 'visible': s[1] and fresh, 'fresh': fresh}
            busy, result = self.dock_busy, self.dock_result
            progress = dict(self.progress) if self.progress else None
        try:
            settings = pose_store.load_section(pose_store.MISSION_KEY, self.poses_file)
        except Exception:
            settings = {}
        return {'dock': dock, 'dock_busy': busy, 'dock_result': result,
                'progress': progress, 'settings': settings}


def add_routes(app, api, require_control, client_id, depends, Request):

    @app.post('/api/mission/start', dependencies=[depends(require_control)])
    def mission_start(body: MissionSettings):
        ok, msg = api.start(body)
        return {'ok': ok, 'message': msg}

    @app.post('/api/mission/stop')
    def mission_stop(request: Request):
        client_id(request)  # stopping is always safe, so any device may do it
        ok, msg = api.stop()
        return {'ok': ok, 'message': msg}

    @app.post('/api/dock', dependencies=[depends(require_control)])
    def dock():
        ok, msg = api.run_dock_action(undock=False)
        return {'ok': ok, 'message': msg}

    @app.post('/api/undock', dependencies=[depends(require_control)])
    def undock():
        ok, msg = api.run_dock_action(undock=True)
        return {'ok': ok, 'message': msg}

    @app.post('/api/localize/docked', dependencies=[depends(require_control)])
    def localize_docked():
        ok, msg = api.localize_from_dock()
        return {'ok': ok, 'message': msg}
