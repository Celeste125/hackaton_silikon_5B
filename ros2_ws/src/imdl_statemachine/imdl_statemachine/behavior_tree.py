"""Waypoint behavior tree for the Team2 Create3 on top of Nav2.

Tree (a Sequence, so the first failing child fails the whole mission):
    Mission
      ├── Nav2Ready
      ├── GoToActionPoint
      ├── DoAction
      ├── GoToRouteStart
      ├── SlowRoute
      └── Turn

Every leaf is a py_trees behaviour. Navigation leaves start a Nav2 task in
``initialise`` and poll it in ``update``; ``terminate`` cancels it, so a
timeout, a failing sibling or Ctrl-C never leaves a goal running. To extend
the mission, write another leaf and add it to ``build_tree``. Poses and
speeds come from a YAML file (parameter ``waypoints_file``).

Requires imdl_autonomy map_navigation.launch.py (AMCL + Nav2) and
real_rviz.launch.py.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import py_trees
from py_trees.common import Status
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
import yaml


TICK_PERIOD_SEC = 0.1


def load_config(path):
    with open(path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    for key in ('initial_pose', 'action_point', 'route_start', 'slow_route'):
        if key not in config:
            raise ValueError(f'{path}: missing key {key!r}')
    if not config['slow_route']:
        raise ValueError(f'{path}: slow_route must contain at least one pose')
    return config


def make_pose(nav: BasicNavigator, xyz_yaw) -> PoseStamped:
    x, y, yaw = (float(v) for v in xyz_yaw)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


# --------------------------------------------------------------------- leaves


class Nav2Ready(py_trees.behaviour.Behaviour):
    """Set the AMCL start pose and block until Nav2 is active (runs once)."""

    def __init__(self, nav: BasicNavigator, config: dict):
        super().__init__(name='Nav2Ready')
        self.nav = nav
        self.config = config

    def update(self) -> Status:
        self.nav.setInitialPose(make_pose(self.nav, self.config['initial_pose']))
        self.nav.info('Waiting for AMCL and Nav2 to become active ...')
        self.nav.waitUntilNav2Active()
        return Status.SUCCESS


class NavTask(py_trees.behaviour.Behaviour):
    """Base for leaves that run one Nav2 task with a timeout.

    Subclasses implement ``start`` (returns False when the goal is rejected).
    """

    def __init__(self, name: str, nav: BasicNavigator, timeout_sec: float):
        super().__init__(name=name)
        self.nav = nav
        self.timeout_sec = timeout_sec
        self._deadline = 0.0
        self._last_log = 0.0
        self._started = False
        self._finished = False

    def start(self):
        raise NotImplementedError

    def initialise(self):
        self.nav.info(f'Start: {self.name}')
        self._deadline = time.monotonic() + self.timeout_sec
        self._last_log = 0.0
        self._finished = False
        self._started = self.start() is not False
        if not self._started:
            self.nav.error(f'{self.name}: goal rejected')

    def update(self) -> Status:
        if not self._started:
            self._finished = True
            return Status.FAILURE

        now = time.monotonic()
        if not self.nav.isTaskComplete():
            if now > self._deadline:
                self.nav.error(f'{self.name}: timeout after {self.timeout_sec:.0f} s')
                return Status.FAILURE  # terminate() cancels the task
            if now - self._last_log > 2.0:
                self._last_log = now
                remaining = getattr(self.nav.getFeedback(), 'distance_remaining', None)
                if remaining is not None:
                    self.nav.info(f'{self.name}: {remaining:.2f} m remaining')
            return Status.RUNNING

        self._finished = True
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav.info(f'{self.name}: succeeded')
            return Status.SUCCESS
        self.nav.error(f'{self.name}: {result}')
        return Status.FAILURE

    def terminate(self, new_status: Status):
        # A task that did not finish on its own (timeout, interrupted by a
        # sibling or Ctrl-C) is still running in Nav2 and has to be cancelled.
        if self._started and not self._finished:
            self.nav.cancelTask()
        self._started = False


class GoToPose(NavTask):
    def __init__(self, name, nav, pose_xyz_yaw, timeout_sec):
        super().__init__(name, nav, timeout_sec)
        self.pose_xyz_yaw = pose_xyz_yaw

    def start(self):
        return self.nav.goToPose(make_pose(self.nav, self.pose_xyz_yaw))


class GoThroughPoses(NavTask):
    """Drive through poses, optionally under a speed limit (m/s)."""

    def __init__(self, name, nav, poses_xyz_yaw, timeout_sec, speed_limit_mps=0.0):
        super().__init__(name, nav, timeout_sec)
        self.poses_xyz_yaw = poses_xyz_yaw
        self.speed_limit_mps = speed_limit_mps
        # Read by controller_server; 0.0 removes the limit.
        self.speed_limit_pub = nav.create_publisher(SpeedLimit, 'speed_limit', 10)

    def start(self):
        self._publish_limit(self.speed_limit_mps)
        poses = [make_pose(self.nav, p) for p in self.poses_xyz_yaw]
        return self.nav.goThroughPoses(poses)

    def terminate(self, new_status: Status):
        super().terminate(new_status)
        if new_status != Status.RUNNING:
            self._publish_limit(0.0)

    def _publish_limit(self, speed_mps: float):
        msg = SpeedLimit()
        msg.header.frame_id = 'map'
        msg.percentage = False
        msg.speed_limit = speed_mps
        self.speed_limit_pub.publish(msg)


class Turn(NavTask):
    def __init__(self, name, nav, angle_rad, timeout_sec):
        super().__init__(name, nav, timeout_sec)
        self.angle_rad = angle_rad

    def start(self):
        return self.nav.spin(spin_dist=self.angle_rad, time_allowance=30)


class DoAction(py_trees.behaviour.Behaviour):
    """Placeholder action: waits. Replace with the gripper, a detection ..."""

    def __init__(self, nav: BasicNavigator, duration_sec: float):
        super().__init__(name='DoAction')
        self.nav = nav
        self.duration_sec = duration_sec
        self._end = 0.0

    def initialise(self):
        self.nav.info(f'Action: waiting {self.duration_sec:.1f} s')
        self._end = time.monotonic() + self.duration_sec

    def update(self) -> Status:
        return Status.SUCCESS if time.monotonic() >= self._end else Status.RUNNING


# ----------------------------------------------------------------------- tree


def build_tree(nav: BasicNavigator, config: dict) -> py_trees.behaviour.Behaviour:
    timeout = float(config.get('step_timeout_sec', 120.0))
    return py_trees.composites.Sequence(
        name='Mission',
        memory=True,
        children=[
            Nav2Ready(nav, config),
            GoToPose('GoToActionPoint', nav, config['action_point'], timeout),
            DoAction(nav, float(config.get('action_duration_sec', 3.0))),
            GoToPose('GoToRouteStart', nav, config['route_start'], timeout),
            GoThroughPoses(
                'SlowRoute', nav, config['slow_route'], timeout,
                speed_limit_mps=float(config.get('slow_speed_mps', 0.08)),
            ),
            Turn('Turn', nav, float(config.get('turn_angle_rad', math.pi)), timeout),
        ],
    )


def running_leaf(root: py_trees.behaviour.Behaviour) -> str:
    """Name of the deepest behaviour that is currently RUNNING."""
    name = root.name
    for child in root.children:
        if child.status == Status.RUNNING:
            return running_leaf(child)
    return name


def run(nav: BasicNavigator, config: dict) -> bool:
    state_pub = nav.create_publisher(String, 'behavior_tree/state', 10)
    tree = py_trees.trees.BehaviourTree(build_tree(nav, config))
    ok = False
    try:
        while rclpy.ok():
            tree.tick()
            state_pub.publish(String(data=running_leaf(tree.root)))
            if tree.root.status in (Status.SUCCESS, Status.FAILURE):
                ok = tree.root.status == Status.SUCCESS
                break
            rclpy.spin_once(nav, timeout_sec=TICK_PERIOD_SEC)
    finally:
        # Cancels whatever Nav2 task is still running (Ctrl-C, shutdown).
        tree.root.stop(Status.INVALID)
    return ok


def main(args=None):
    # Without rclpy's SIGINT handler, Ctrl-C raises KeyboardInterrupt while the
    # context is still valid, so the running Nav2 goal can still be cancelled.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    navigator = BasicNavigator()
    default_file = os.path.join(
        get_package_share_directory('imdl_statemachine'), 'config', 'waypoints.yaml'
    )
    navigator.declare_parameter('waypoints_file', default_file)
    path = os.path.expanduser(navigator.get_parameter('waypoints_file').value)
    navigator.info(f'Waypoints: {path}')

    try:
        ok = run(navigator, load_config(path))
        navigator.info('Finished' if ok else 'Mission failed')
    except KeyboardInterrupt:
        navigator.warn('Interrupted, navigation cancelled')
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
