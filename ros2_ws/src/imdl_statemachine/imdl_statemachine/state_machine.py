"""Waypoint state machine for the Team2 Create3 on top of Nav2.

Sequence:
    INIT -> GO_TO_ACTION_POINT -> DO_ACTION -> GO_TO_ROUTE_START
         -> SLOW_ROUTE -> TURN -> DONE
Any failed or timed-out step ends in FAILED.

Each state is one method that returns the next state, so new states are
added by extending ``State`` and ``_handlers``. Poses and speeds come from
a YAML file (parameter ``waypoints_file``).

Requires imdl_autonomy map_navigation.launch.py (AMCL + Nav2) and
real_rviz.launch.py.
"""

from enum import Enum, auto
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
import yaml


class State(Enum):
    INIT = auto()
    GO_TO_ACTION_POINT = auto()
    DO_ACTION = auto()
    GO_TO_ROUTE_START = auto()
    SLOW_ROUTE = auto()
    TURN = auto()
    DONE = auto()
    FAILED = auto()


TERMINAL_STATES = (State.DONE, State.FAILED)


def load_config(path):
    with open(path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    for key in ('initial_pose', 'action_point', 'route_start', 'slow_route'):
        if key not in config:
            raise ValueError(f'{path}: missing key {key!r}')
    if not config['slow_route']:
        raise ValueError(f'{path}: slow_route must contain at least one pose')
    return config


class WaypointStateMachine:
    def __init__(self, navigator: BasicNavigator, config: dict):
        self.nav = navigator
        self.config = config
        self.state = State.INIT
        self.step_timeout = float(config.get('step_timeout_sec', 120.0))

        self.state_pub = navigator.create_publisher(String, 'state_machine/state', 10)
        # Read by controller_server; 0.0 removes the limit.
        self.speed_limit_pub = navigator.create_publisher(SpeedLimit, 'speed_limit', 10)

        self._handlers = {
            State.INIT: self.on_init,
            State.GO_TO_ACTION_POINT: self.on_go_to_action_point,
            State.DO_ACTION: self.on_do_action,
            State.GO_TO_ROUTE_START: self.on_go_to_route_start,
            State.SLOW_ROUTE: self.on_slow_route,
            State.TURN: self.on_turn,
        }

    # ------------------------------------------------------------------ run

    def run(self) -> bool:
        while self.state not in TERMINAL_STATES:
            self._publish_state()
            next_state = self._handlers[self.state]()
            self.nav.info(f'{self.state.name} -> {next_state.name}')
            self.state = next_state
        self._publish_state()
        return self.state == State.DONE

    def abort(self):
        """Stop any running Nav2 task and lift the speed limit."""
        self.nav.cancelTask()
        self.set_speed_limit(0.0)

    # --------------------------------------------------------------- states

    def on_init(self) -> State:
        self.nav.setInitialPose(self.pose(self.config['initial_pose']))
        self.nav.info('Waiting for AMCL and Nav2 to become active ...')
        self.nav.waitUntilNav2Active()
        self.set_speed_limit(0.0)
        return State.GO_TO_ACTION_POINT

    def on_go_to_action_point(self) -> State:
        return self.go_to('action_point', State.DO_ACTION)

    def on_do_action(self) -> State:
        # Replace this with the real action (gripper, signal, measurement ...).
        duration = float(self.config.get('action_duration_sec', 3.0))
        self.nav.info(f'Action: waiting {duration:.1f} s')
        self.sleep(duration)
        return State.GO_TO_ROUTE_START

    def on_go_to_route_start(self) -> State:
        return self.go_to('route_start', State.SLOW_ROUTE)

    def on_slow_route(self) -> State:
        poses = [self.pose(p) for p in self.config['slow_route']]
        self.set_speed_limit(float(self.config.get('slow_speed_mps', 0.08)))
        try:
            ok = self.run_task('slow route', lambda: self.nav.goThroughPoses(poses))
        finally:
            self.set_speed_limit(0.0)
        return State.TURN if ok else State.FAILED

    def on_turn(self) -> State:
        angle = float(self.config.get('turn_angle_rad', math.pi))
        if self.run_task(
            'turn',
            lambda: self.nav.spin(spin_dist=angle, time_allowance=30),
        ):
            return State.DONE
        return State.FAILED

    # -------------------------------------------------------------- helpers

    def go_to(self, target, next_state: State, failed_state: State = State.FAILED) -> State:
        """Drive to ``target`` and return ``next_state`` on success.

        ``target`` is a key of the YAML file (e.g. 'action_point') or a pose
        given directly as [x, y, yaw].
        """
        if isinstance(target, str):
            label, xyz_yaw = f'go to {target}', self.config[target]
        else:
            label, xyz_yaw = f'go to {list(target)}', target
        pose = self.pose(xyz_yaw)
        if self.run_task(label, lambda: self.nav.goToPose(pose)):
            return next_state
        return failed_state

    def run_task(self, label: str, start) -> bool:
        """Start a Nav2 task and block until it finishes, fails or times out."""
        self.nav.info(f'Start: {label}')
        if start() is False:
            self.nav.error(f'{label}: goal rejected')
            return False

        deadline = time.monotonic() + self.step_timeout
        last_log = 0.0
        while not self.nav.isTaskComplete():
            now = time.monotonic()
            if now > deadline:
                self.nav.error(f'{label}: timeout after {self.step_timeout:.0f} s')
                self.nav.cancelTask()
                return False
            if now - last_log > 2.0:
                last_log = now
                feedback = self.nav.getFeedback()
                remaining = getattr(feedback, 'distance_remaining', None)
                if remaining is not None:
                    self.nav.info(f'{label}: {remaining:.2f} m remaining')

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav.info(f'{label}: succeeded')
            return True
        self.nav.error(f'{label}: {result}')
        return False

    def pose(self, xyz_yaw) -> PoseStamped:
        x, y, yaw = (float(v) for v in xyz_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def set_speed_limit(self, speed_mps: float):
        msg = SpeedLimit()
        msg.header.frame_id = 'map'
        msg.percentage = False
        msg.speed_limit = speed_mps
        self.speed_limit_pub.publish(msg)

    def sleep(self, seconds: float):
        """Wait while still processing ROS callbacks."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def _publish_state(self):
        self.state_pub.publish(String(data=self.state.name))


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

    machine = WaypointStateMachine(navigator, load_config(path))
    try:
        ok = machine.run()
        navigator.info('Finished' if ok else 'Stopped in FAILED')
    except KeyboardInterrupt:
        navigator.warn('Interrupted, cancelling navigation')
        machine.abort()
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
