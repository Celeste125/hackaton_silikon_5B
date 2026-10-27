"""Map tools of the panel, like RViz's: set the robot's position, send a goal.

  POST /api/initialpose {x, y, yaw}   AMCL starts from there ("2D Pose Estimate")
  POST /api/goal {x, y, yaw}          Nav2 drives there ("Nav2 Goal"); on the
                                      dock it undocks first
  POST /api/goal/cancel               cancels every Nav2 goal (any device)

Also reports Nav2's current path (/plan) and goal for the map view.
"""

import math
import threading
import time

from pydantic import BaseModel
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal

try:
    from irobot_create_msgs.action import Undock
except ImportError:
    Undock = None

PLAN_POINTS_MAX = 200
PLAN_STALE_S = 3.0


class PoseBody(BaseModel):
    x: float
    y: float
    yaw: float


def _quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class NavApi:
    def __init__(self, node, docked=lambda: False, namespace='/Team2'):
        """docked: callable -> True while the robot is on its dock."""
        self.node = node
        self.docked = docked
        self.undock = ActionClient(node, Undock, f'{namespace.rstrip("/")}/undock') if Undock else None
        self.lock = threading.Lock()
        self.plan = None          # ([[x, y], ...], monotonic time)
        self.goal = None          # {'x', 'y', 'yaw', 'state'}
        self.initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        node.create_subscription(Path, '/plan', self._on_plan, 10)
        self.nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.cancel = node.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')

    def _on_plan(self, msg):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        step = max(1, len(pts) // PLAN_POINTS_MAX)
        with self.lock:
            self.plan = ([[round(x, 3), round(y, 3)] for x, y in pts[::step]] + pts[-1:], time.monotonic())

    def set_initial_pose(self, b):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = b.x, b.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = _quat(b.yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2       # as RViz's 2D Pose Estimate
        cov[35] = math.radians(15) ** 2
        msg.pose.covariance = cov
        for _ in range(3):
            self.initialpose_pub.publish(msg)
        return True, f'Position set to {b.x:.2f}, {b.y:.2f}, {math.degrees(b.yaw):.0f}°'

    def send_goal(self, b):
        if not self.nav.wait_for_server(timeout_sec=1.0):
            return False, 'Nav2 is not running (navigation.launch.py)'
        if self.undock is not None and self.docked():
            # Nav2 cannot drive the robot off its dock: undock, then go.
            if not self.undock.wait_for_server(timeout_sec=1.0):
                return False, 'the robot is docked and the Create 3 undock action is not available'
            with self.lock:
                self.goal = {'x': b.x, 'y': b.y, 'yaw': b.yaw, 'state': 'undocking'}
            self.undock.send_goal_async(Undock.Goal()).add_done_callback(
                lambda f: self._after_undock_accepted(f, b))
            return True, 'Undocking first, then driving to the goal'
        return self._send_nav(b)

    def _after_undock_accepted(self, future, b):
        handle = future.result()
        if not handle.accepted:
            self._set_goal_state('undock refused')
            return
        handle.get_result_async().add_done_callback(lambda f: self._after_undock(f, b))

    def _after_undock(self, future, b):
        with self.lock:
            wanted = self.goal is not None and self.goal['state'] == 'undocking'
        if not wanted:
            return  # cancelled meanwhile
        try:
            still_docked = future.result().result.is_docked
        except Exception:
            still_docked = True
        if still_docked:
            self._set_goal_state('undock failed')
        else:
            self._send_nav(b)

    def _send_nav(self, b):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = b.x, b.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = _quat(b.yaw)
        with self.lock:
            self.goal = {'x': b.x, 'y': b.y, 'yaw': b.yaw, 'state': 'sent'}
        self.nav.send_goal_async(goal).add_done_callback(self._on_accepted)
        return True, 'Goal sent'

    def _set_goal_state(self, state):
        with self.lock:
            if self.goal is not None:
                self.goal['state'] = state

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self._set_goal_state('rejected')
            return
        self._set_goal_state('driving')
        handle.get_result_async().add_done_callback(
            lambda f: self._set_goal_state({4: 'reached', 5: 'cancelled'}.get(f.result().status, 'failed')))

    def cancel_all(self):
        if not self.cancel.service_is_ready():
            return False, 'Nav2 is not running'
        self.cancel.call_async(CancelGoal.Request())  # zero id and stamp: every goal
        self._set_goal_state('cancelled')
        return True, 'Nav2 goal cancelled'

    def state(self):
        with self.lock:
            plan = self.plan
            goal = dict(self.goal) if self.goal else None
        driving = goal is not None and goal['state'] in ('undocking', 'sent', 'driving')
        # Nav2 republishes /plan while it drives; an old one is only kept for
        # a goal still being driven to.
        show = plan is not None and (driving or time.monotonic() - plan[1] < PLAN_STALE_S)
        return {'plan': plan[0] if show else None, 'goal': goal}


def add_routes(app, ros, api, require_control, client_id, depends, Request):

    @app.post('/api/initialpose', dependencies=[depends(require_control)])
    def initial_pose(body: PoseBody):
        ok, msg = api.set_initial_pose(body)
        return {'ok': ok, 'message': msg}

    @app.post('/api/goal', dependencies=[depends(require_control)])
    def goal(body: PoseBody):
        act = ros.activity()
        if act in ('mission', 'dock'):
            return {'ok': False, 'message': 'Not now: ' + ('a mission is running' if act == 'mission'
                                                           else 'the robot is docking or undocking')}
        if act == 'laps':
            ros.call('lap/stop')  # a goal from the map means: stop the laps, go there
        ok, msg = api.send_goal(body)
        return {'ok': ok, 'message': msg}

    @app.post('/api/goal/cancel')
    def goal_cancel(request: Request):
        client_id(request)  # cancelling is always safe, so any device may do it
        ok, msg = api.cancel_all()
        return {'ok': ok, 'message': msg}
