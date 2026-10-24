"""Waypoint state machine for the Team2 Create3 on top of Nav2.

Sequence:
    INIT -> GO_TO_ACTION_POINT -> DO_ACTION -> GO_TO_ROUTE_START
         -> SLOW_ROUTE -> CHECK_MIX -> TURN -> DONE
CHECK_MIX reads the Float32 topic ``mix_quality`` (0..1). If the mix is below
``mix_quality_threshold`` the robot goes back to the route start and repeats
the slow route (EXTRA_LAP), at most ``max_extra_laps`` times, then moves on.
Any failed or timed-out step ends in FAILED.

Backup without a saved map position: with ``marker_route.enabled`` the
robot works from ArUco markers (poses come from the aruco_detector node, topic
``aruco/marker_<id>``), spinning in place to find a marker it has not seen yet.
``marker_route.refill_marker`` is where the food is loaded; the other ids are
the feeding stops, in the order of ``marker_route.ids``. With ``loop`` (default)
the mission repeats until stopped:
    INIT -> REFILL -> MARKER_ROUTE -> CHECK_MIX -> REFILL -> ...
REFILL drives to the refill marker and waits ``refill_wait_sec`` for loading.
Without ``loop`` it ends with CHECK_MIX -> TURN -> DONE. An extra lap repeats
the feeding stops. Publish std_msgs/Empty on ``state_machine/stop`` (or press
Ctrl-C) to cancel navigation and end the mission.

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
from std_msgs.msg import Empty, Float32, String
import yaml

from imdl_statemachine.markers import standoff_goal


class State(Enum):
    INIT = auto()
    MARKER_ROUTE = auto()
    REFILL = auto()
    GO_TO_ACTION_POINT = auto()
    DO_ACTION = auto()
    GO_TO_ROUTE_START = auto()
    SLOW_ROUTE = auto()
    CHECK_MIX = auto()
    EXTRA_LAP = auto()
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

        # Latest mix quality (0..1) and when it arrived; None until first message.
        self.mix_quality = None
        self.mix_stamp = 0.0
        self.extra_laps = 0
        navigator.create_subscription(Float32, 'mix_quality', self._on_mix_quality, 10)

        # Set by Empty on state_machine/stop; checked while driving and waiting.
        self.stop_requested = False
        navigator.create_subscription(Empty, 'state_machine/stop', self._on_stop, 10)
        self.laps = 0

        # Marker route: latest approach pose [x, y, yaw] per marker id.
        marker_cfg = config.get('marker_route') or {}
        self.marker_mode = bool(marker_cfg.get('enabled', False))
        self.marker_goals = {}
        self.marker_index = 0
        self.loop = bool(marker_cfg.get('loop', True))
        if self.marker_mode:
            self.route_ids()  # fail early on a bad refill_marker
            for marker_id in marker_cfg['ids']:
                navigator.create_subscription(
                    PoseStamped, f'aruco/marker_{marker_id}',
                    lambda msg, i=marker_id: self._on_marker(i, msg), 10)

        self._handlers = {
            State.INIT: self.on_init,
            State.MARKER_ROUTE: self.on_marker_route,
            State.REFILL: self.on_refill,
            State.GO_TO_ACTION_POINT: self.on_go_to_action_point,
            State.DO_ACTION: self.on_do_action,
            State.GO_TO_ROUTE_START: self.on_go_to_route_start,
            State.SLOW_ROUTE: self.on_slow_route,
            State.CHECK_MIX: self.on_check_mix,
            State.EXTRA_LAP: self.on_extra_lap,
            State.TURN: self.on_turn,
        }

    # ------------------------------------------------------------------ run

    def run(self) -> bool:
        while self.state not in TERMINAL_STATES:
            self._publish_state()
            next_state = self._handlers[self.state]()
            if self.stop_requested:
                self.abort()
                self.nav.info('Stop requested, ending the mission')
                next_state = State.DONE
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
        if self.marker_mode:
            return State.REFILL if self.loop else State.MARKER_ROUTE
        return State.GO_TO_ACTION_POINT

    def on_marker_route(self) -> State:
        ids = self.route_ids()
        if self.marker_index >= len(ids):
            self.marker_index = 0
            return State.CHECK_MIX
        # Drive the first leg normally, the rest (the feeding lap) slowly.
        if not self.drive_to_marker(ids[self.marker_index], slow=self.marker_index > 0):
            return State.FAILED
        self.marker_index += 1
        return State.MARKER_ROUTE

    def on_refill(self) -> State:
        refill = self.route_ids()[0]
        if not self.drive_to_marker(refill, slow=False):
            return State.FAILED
        wait = float(self.config['marker_route'].get('refill_wait_sec', 10.0))
        self.nav.info(f'REFILL at marker {refill}: loading for {wait:.0f} s (lap {self.laps + 1})')
        self.sleep(wait)
        self.laps += 1
        self.extra_laps = 0
        self.marker_index = 1  # already at the refill marker
        return State.MARKER_ROUTE

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
        return State.CHECK_MIX if ok else State.FAILED

    def on_check_mix(self) -> State:
        if not self.config.get('mix_check_enabled', True):
            return self.after_lap()
        threshold = float(self.config.get('mix_quality_threshold', 0.8))
        max_age = float(self.config.get('mix_max_age_sec', 2.0))
        self.sleep(1.0)  # let a fresh message arrive after stopping
        age = time.monotonic() - self.mix_stamp
        if self.mix_quality is None or age > max_age:
            # No sensor data must not block the mission.
            self.nav.warn('CHECK_MIX: no recent mix_quality, continuing')
            return self.after_lap()
        if self.mix_quality >= threshold:
            self.nav.info(f'CHECK_MIX: {self.mix_quality:.2f} >= {threshold:.2f}, mix is good')
            return self.after_lap()
        if self.extra_laps >= int(self.config.get('max_extra_laps', 2)):
            self.nav.warn(
                f'CHECK_MIX: {self.mix_quality:.2f} < {threshold:.2f} after '
                f'{self.extra_laps} extra laps, giving up')
            return self.after_lap()
        self.nav.info(f'CHECK_MIX: {self.mix_quality:.2f} < {threshold:.2f}, extra lap')
        return State.EXTRA_LAP

    def on_extra_lap(self) -> State:
        self.extra_laps += 1
        if self.marker_mode:
            self.marker_index = 1  # feeding stops again, not the refill marker
            return State.MARKER_ROUTE
        return self.go_to('route_start', State.SLOW_ROUTE)

    def on_turn(self) -> State:
        angle = float(self.config.get('turn_angle_rad', math.pi))
        if self.run_task(
            'turn',
            lambda: self.nav.spin(spin_dist=angle, time_allowance=30),
        ):
            return State.DONE
        return State.FAILED

    # -------------------------------------------------------------- helpers

    def route_ids(self):
        """Marker ids of one lap: the refill marker first, then the others in order."""
        cfg = self.config['marker_route']
        ids = list(cfg['ids'])
        refill = cfg.get('refill_marker', ids[0])
        if refill not in ids:
            raise ValueError(f'marker_route.refill_marker {refill!r} is not in ids {ids}')
        return [refill] + [i for i in ids if i != refill]

    def after_lap(self) -> State:
        """Where to go once a lap (and its mix check) is finished."""
        return State.REFILL if self.marker_mode and self.loop else State.TURN

    def drive_to_marker(self, marker_id, slow: bool) -> bool:
        if marker_id not in self.marker_goals and not self.search_marker(marker_id):
            self.nav.error(f'marker {marker_id} not found')
            return False
        goal = self.marker_goals[marker_id]
        if slow:
            self.set_speed_limit(float(self.config.get('slow_speed_mps', 0.08)))
        try:
            return self.run_task(
                f'go to marker {marker_id}', lambda: self.nav.goToPose(self.pose(goal)))
        finally:
            self.set_speed_limit(0.0)

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
        if self.stop_requested:
            return False
        self.nav.info(f'Start: {label}')
        if start() is False:
            self.nav.error(f'{label}: goal rejected')
            return False

        deadline = time.monotonic() + self.step_timeout
        last_log = 0.0
        while not self.nav.isTaskComplete():
            if self.stop_requested:
                self.nav.warn(f'{label}: stop requested')
                self.nav.cancelTask()
                return False
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

    def search_marker(self, marker_id) -> bool:
        """Spin in place, one step at a time, until the marker has been seen."""
        step = float(self.config['marker_route'].get('search_step_rad', 0.6))
        self.nav.info(f'Searching for marker {marker_id}')
        for _ in range(math.ceil(2.0 * math.pi / step)):
            self.sleep(1.0)  # let a detection arrive while standing still
            if marker_id in self.marker_goals:
                return True
            if not self.run_task(
                'search spin', lambda: self.nav.spin(spin_dist=step, time_allowance=20)
            ):
                return False
        self.sleep(1.0)
        return marker_id in self.marker_goals

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
        while time.monotonic() < end and not self.stop_requested:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def _on_mix_quality(self, msg: Float32):
        self.mix_quality = float(msg.data)
        self.mix_stamp = time.monotonic()

    def _on_stop(self, msg: Empty):
        self.nav.info('Stop request received')
        self.stop_requested = True

    def _on_marker(self, marker_id, msg: PoseStamped):
        q = msg.pose.orientation
        values = (msg.pose.position.x, msg.pose.position.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.nav.warn(f'marker {marker_id}: ignoring non-finite pose')
            return
        normal_yaw = 2.0 * math.atan2(q.z, q.w)
        standoff = float(self.config['marker_route'].get('standoff_m', 0.8))
        self.marker_goals[marker_id] = standoff_goal(
            msg.pose.position.x, msg.pose.position.y, normal_yaw, standoff)

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
