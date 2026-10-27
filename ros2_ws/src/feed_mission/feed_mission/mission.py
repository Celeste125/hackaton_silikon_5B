"""The feeding run, start to finish, as one sequence of steps.

    ros2 launch feed_mission mission.launch.py        (with navigation running)
    ros2 service call /mission/start std_srvs/srv/Trigger
    ros2 service call /mission/stop std_srvs/srv/Trigger

Steps (parameter `steps` picks which ones run, in this order):

  localize  On the dock with a taught "docked" pose: tell AMCL exactly where
            the robot is. Otherwise /relocalize (scan matching on the map).
  undock    Create 3 Undock action, if it is docked.
  fill      Nav2 to the silo entry, then straight to silo 1 and along the row
            (silo_route.py: the row is too tight for Nav2). At each silo up to
            the last one with a share: wait for the scale to settle, open the
            valve until the weight has risen by that silo's share, close it.
            If the weight stops rising it shakes the mixer once; if it still
            does not rise, it fails. Then back out the same way.
            fill_mix spreads the feed so it does not pile up under the chute:
            'pour' runs the mixer at fill_mix_speed while a valve is open (at
            half speed the scale stays within ~10 g; at full speed it jumps by
            several hundred g), 'between' mixes fill_mix_s after each silo with
            the valve closed, 'off' does neither.
  mix       Mixer motors for mix_s seconds.
  feed      Laps around the cow box (lap_driver, with the Feed tab settings),
            one at a time, until the weight is below empty_kg or laps_max.
            The mission's own door opening (feed_door_open) and mixer
            (feed_mix: 'always', 'below' feed_mix_below_kg, or 'off', at
            feed_mix_speed) replace the Feed tab's.
  dock      Nav2 to the spot in front of the dock (0.6 m out from the taught
            docked pose, facing it), then the Create 3's Dock. Without a docked
            pose: Nav2 to the start pose.

dry_run keeps the valves and the door closed and skips waiting for weight
(each silo waits a few seconds instead), so the whole run can be watched
without feed. Status on /mission/progress (latched JSON). /mission/stop, and
STOP ALL in the web panel, stop every output at once.
"""

import json
import math
import threading
import time
from statistics import median

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from std_srvs.srv import Trigger
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from feed_mission import poses as pose_store
from feed_mission import silo_route
from feed_mission.lap_driver import straight_command, turn_command

try:  # dock and undock need the Create 3 messages
    from irobot_create_msgs.action import Dock, Undock
    from irobot_create_msgs.msg import DockStatus
except ImportError:
    Dock = Undock = DockStatus = None

STEPS = ('localize', 'undock', 'fill', 'mix', 'feed', 'dock')
RUNNING = ('undock', 'approach', 'rejoin', 'paused', 'turn', 'straight', 'blocked')
NUM_SILOS = 3
MIX_ALWAYS_KG = 1000.0  # 'mix below' for the whole run: more than the container holds
MANUAL_MIX_S = 1.0  # s after the web panel's last 'mixer/manual' it has the mixer


class Stopped(Exception):
    """Raised inside a step when /mission/stop was called."""


class StepFailed(Exception):
    pass


def split_recipe(total_kg, shares):
    """kg per silo from a total and relative shares (percent or any scale)."""
    shares = [max(0.0, float(s)) for s in shares]
    whole = sum(shares)
    if total_kg <= 0 or whole <= 0:
        return [0.0] * len(shares)
    return [total_kg * s / whole for s in shares]


class FillWatch:
    """Decides when a silo is done: target reached, stalled, or out of time.

    update(t, added_kg) -> 'filling', 'done', 'stalled' or 'timeout'.
    Stalled: less than stall_kg added over the last stall_s seconds.
    """

    def __init__(self, target_kg, stall_s=8.0, stall_kg=0.01, timeout_s=90.0, t0=0.0):
        self.target, self.stall_s, self.stall_kg = target_kg, stall_s, stall_kg
        self.deadline = t0 + timeout_s
        self.mark = (t0, 0.0)  # last time the weight rose by stall_kg

    def update(self, t, added):
        if added >= self.target:
            return 'done'
        if t > self.deadline:
            return 'timeout'
        if added - self.mark[1] >= self.stall_kg:
            self.mark = (t, added)
        elif t - self.mark[0] > self.stall_s:
            return 'stalled'
        return 'filling'

    def restart_stall(self, t, added):
        """After a shake: give the flow a fresh stall_s to pick up."""
        self.mark = (t, added)


def _param(name, value):
    v = ParameterValue()
    if isinstance(value, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, int):
        v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
    elif isinstance(value, float):
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
    else:
        v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
    return Parameter(name=name, value=v)


class Mission(Node):
    def __init__(self, tf=None):
        """tf: a shared tf2 Buffer (all feed_mission nodes in one process), else its own."""
        super().__init__('mission')
        p = self.declare_parameter
        p('poses_file', pose_store.DEFAULT_FILE)
        p('namespace', '/Team2')
        p('steps', list(STEPS))
        p('dry_run', True)
        p('recipe_kg', 1.0)                 # total to load
        p('recipe', [40.0, 40.0, 20.0])     # share per silo
        p('valve_percent', 100.0)           # how far a valve opens while filling
        p('settle_s', 2.0)                  # scale settling time after stopping
        p('fill_timeout_s', 90.0)
        p('stall_s', 8.0)
        p('stall_kg', 0.01)
        p('mix_s', 60.0)
        p('mix_speed', 0.6)                 # -1..1 for both mixer motors
        p('fill_mix', 'pour')               # mixer while filling: pour, between or off
        p('fill_mix_speed', 0.5)
        p('fill_mix_s', 3.0)                # 'between': this long after each silo
        p('feed_door_open', 0.2)            # door opening on the laps, 0..1
        p('feed_mix', 'always')             # mixer on the laps: always, below or off
        p('feed_mix_below_kg', 0.5)         # 'below': on once the container is down to this
        p('feed_mix_speed', 0.6)
        p('laps_max', 6)
        p('empty_kg', 0.05)                 # feeding stops below this
        p('nav_timeout_s', 120.0)
        p('lap_timeout_s', 600.0)           # one lap taking longer than this: give up
        p('num_motors', 2)
        p('cmd_vel_topic', '/Team2/cmd_vel_lap')  # the silo moves, through twist_mux like the laps
        p('silo_speed', 0.08)               # m/s on the straight moves at the silos
        p('silo_turn_speed', 0.4)           # rad/s
        p('entry_side', 'right')            # untaught silo entry: this side of silo 1...
        p('entry_m', 0.6)                   # ...this far out
        self.g = lambda n: self.get_parameter(n).value
        ns = self.g('namespace').rstrip('/')
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.progress_pub = self.create_publisher(String, 'mission/progress', latched)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.valve_pubs = [self.create_publisher(Float32, f'valve{i + 1}/percent', 10)
                           for i in range(NUM_SILOS)]
        self.motor_pubs = [self.create_publisher(Float32, f'mixer/motor_{i}', 10)
                           for i in range(int(self.g('num_motors')))]
        self.shake_pub = self.create_publisher(Float32MultiArray, 'mixer/shake', 10)
        self.door_pub = self.create_publisher(Float32, 'door/position', 10)
        self.cmd_pub = self.create_publisher(Twist, self.g('cmd_vel_topic'), 10)
        self.tf = tf
        if tf is None:
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)

        self.weights = []  # (monotonic time, kg)
        self.create_subscription(Float32, 'loadcell/weight', self._on_weight, 10, callback_group=cb)
        self.manual_mix_t = 0.0  # last 'mixer/manual' from the web panel: hands off the mixer
        self.create_subscription(Bool, 'mixer/manual', self._on_manual_mix, 10, callback_group=cb)
        self.lap_progress = None
        self.create_subscription(String, 'lap/progress', self._on_lap, latched, callback_group=cb)
        self.dock_status = None
        if DockStatus is not None:
            self.create_subscription(DockStatus, f'{ns}/dock_status', self._on_dock,
                                     qos_profile_sensor_data, callback_group=cb)
            self.dock_client = ActionClient(self, Dock, f'{ns}/dock', callback_group=cb)
            self.undock_client = ActionClient(self, Undock, f'{ns}/undock', callback_group=cb)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.lap_params = self.create_client(SetParameters, '/lap_driver/set_parameters',
                                             callback_group=cb)
        self.lap_start = self.create_client(Trigger, 'lap/start', callback_group=cb)
        self.lap_stop = self.create_client(Trigger, 'lap/stop', callback_group=cb)
        self.relocalize = self.create_client(Trigger, 'relocalize', callback_group=cb)
        self.create_service(Trigger, 'mission/start', self._on_start, callback_group=cb)
        self.create_service(Trigger, 'mission/stop', self._on_stop, callback_group=cb)

        self.stop_event = threading.Event()
        self.worker = None
        self.active_goal = None  # goal handle to cancel on stop
        self.status = {'state': 'idle', 'steps': [], 'current': None, 'dry_run': None,
                       'detail': '', 'error': None}
        self._publish()
        self.create_timer(1.0, self._publish, callback_group=cb)

    # ---- inputs -------------------------------------------------------------

    def _on_weight(self, msg):
        now = time.monotonic()
        self.weights = [w for w in self.weights if now - w[0] < 5.0] + [(now, float(msg.data))]

    def _on_manual_mix(self, msg):
        self.manual_mix_t = time.monotonic() if msg.data else 0.0

    def _mixer(self, speed):
        """Both mixer motors; left alone while someone runs them from the web panel."""
        if time.monotonic() - self.manual_mix_t < MANUAL_MIX_S:
            return
        for pub in self.motor_pubs:
            pub.publish(Float32(data=float(speed)))

    def _on_lap(self, msg):
        try:
            self.lap_progress = json.loads(msg.data)
        except ValueError:
            pass

    def _on_dock(self, msg):
        self.dock_status = (msg.is_docked, msg.dock_visible, time.monotonic())

    def _weight(self, window=1.0):
        """Median kg over the last `window` s; waits up to 3 s for data."""
        end = time.monotonic() + 3.0
        while True:
            now = time.monotonic()
            recent = [w for t, w in self.weights if now - t <= window]
            if recent:
                return median(recent)
            if now > end:
                raise StepFailed('no load cell data on /loadcell/weight')
            self._sleep(0.1)

    def _docked(self):
        s = self.dock_status
        return bool(s and s[0] and time.monotonic() - s[2] < 5.0)

    def _dock_visible(self):
        s = self.dock_status
        return bool(s and s[1] and time.monotonic() - s[2] < 5.0)

    # ---- status -------------------------------------------------------------

    def _publish(self):
        self.progress_pub.publish(String(data=json.dumps(self.status)))

    def _step_status(self, name, state, detail=''):
        for s in self.status['steps']:
            if s['name'] == name:
                s['state'], s['detail'] = state, detail
        self.status['current'] = name if state == 'running' else self.status['current']
        self.status['detail'] = detail
        self._publish()
        if detail:
            self.get_logger().info(f'mission {name}: {detail}')

    # ---- services -----------------------------------------------------------

    def _on_start(self, request, response):
        if self.worker is not None and self.worker.is_alive():
            response.success, response.message = False, 'a mission is already running'
            return response
        steps = [s for s in STEPS if s in list(self.g('steps'))]
        if not steps:
            response.success, response.message = False, 'no steps selected'
            return response
        self.stop_event.clear()
        dry = bool(self.g('dry_run'))
        self.status = {
            'state': 'running', 'current': None, 'dry_run': dry, 'detail': '', 'error': None,
            'started': time.strftime('%H:%M:%S'),
            'steps': [{'name': s, 'state': 'pending', 'detail': ''} for s in steps],
            'fills': [],
        }
        self._publish()
        self.worker = threading.Thread(target=self._run, args=(steps,), daemon=True)
        self.worker.start()
        response.success = True
        response.message = f'mission started{" (dry run)" if dry else ""}: {", ".join(steps)}'
        return response

    def _on_stop(self, request, response):
        running = self.worker is not None and self.worker.is_alive()
        self.stop_event.set()
        self._safe_outputs()
        response.success = True
        response.message = 'mission stopped' if running else 'no mission running'
        return response

    # ---- the run ------------------------------------------------------------

    def _run(self, steps):
        for name in steps:
            if self.stop_event.is_set():
                break
            self._step_status(name, 'running', 'starting')
            try:
                detail = getattr(self, f'_step_{name}')()
                self._step_status(name, 'done', detail or 'done')
            except Stopped:
                self._step_status(name, 'stopped', 'stopped')
                break
            except StepFailed as e:
                self._step_status(name, 'failed', str(e))
                self.status.update(state='failed', error=f'{name}: {e}')
                self._safe_outputs()
                self._publish()
                return
            except Exception as e:  # a bug must not leave a valve open
                self.get_logger().error(f'mission {name} crashed: {e!r}')
                self._step_status(name, 'failed', f'error: {e}')
                self.status.update(state='failed', error=f'{name}: {e}')
                self._safe_outputs()
                self._publish()
                return
        stopped = self.stop_event.is_set()
        for s in self.status['steps']:
            if s['state'] == 'pending':
                s['state'] = 'skipped' if stopped else s['state']
        self.status.update(state='stopped' if stopped else 'done', current=None)
        self._safe_outputs()
        self._publish()

    def _sleep(self, seconds):
        if self.stop_event.wait(seconds):
            raise Stopped()

    def _safe_outputs(self):
        """Everything closed and still: valves, door, mixer, laps, Nav2 goal."""
        for pub in self.valve_pubs:
            pub.publish(Float32(data=0.0))
        self.door_pub.publish(Float32(data=1.0))
        for pub in self.motor_pubs:
            pub.publish(Float32(data=0.0))
        self.shake_pub.publish(Float32MultiArray(data=[]))
        self.cmd_pub.publish(Twist())
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
            self.active_goal = None
        if self.lap_stop.service_is_ready() and self.lap_progress \
                and self.lap_progress.get('state') in RUNNING:
            self.lap_stop.call_async(Trigger.Request())

    def _wait(self, future, timeout, what):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise StepFailed(f'{what} timed out')
            self._sleep(0.05)
        return future.result()

    def _action(self, client, goal, timeout, what):
        if not client.wait_for_server(timeout_sec=2.0):
            raise StepFailed(f'{what}: action server not available')
        handle = self._wait(client.send_goal_async(goal), 5.0, what)
        if not handle.accepted:
            raise StepFailed(f'{what} was rejected')
        self.active_goal = handle
        try:
            result = self._wait(handle.get_result_async(), timeout, what)
        finally:
            self.active_goal = None
        return result

    def _navigate(self, pose, what):
        if Undock is not None and self._docked():
            # Nav2 cannot drive it off the dock, whether or not Undock was ticked.
            self._step_status(self.status['current'] or 'undock', 'running', 'undocking first')
            self._step_undock()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose['x'], pose['y']
        goal.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
        result = self._action(self.nav, goal, self.g('nav_timeout_s'), f'driving to {what}')
        if result.status != 4:  # SUCCEEDED
            raise StepFailed(f'Nav2 could not reach {what}')

    def _poses(self):
        return pose_store.load(self.g('poses_file'))

    # ---- straight moves, for where Nav2 cannot go ------------------------------

    def _tf_pose(self, frame):
        try:
            t = self.tf.lookup_transform(frame, 'base_link', Time())
        except TransformException:
            raise StepFailed(f'no {frame} -> base_link transform')
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _drive(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def _turn_to(self, map_yaw, timeout=20.0):
        """Turn on the spot to a heading given in the map frame (tracked on odometry)."""
        offset = self._tf_pose('map')[2] - self._tf_pose('odom')[2]
        target = map_yaw - offset
        end = time.monotonic() + timeout
        try:
            while True:
                w, done = turn_command(self._tf_pose('odom')[2], target, float(self.g('silo_turn_speed')))
                if done:
                    return
                if time.monotonic() > end:
                    raise StepFailed('could not turn to the heading (stuck?)')
                self._drive(0.0, w)
                self._sleep(0.05)
        finally:
            self._drive(0.0, 0.0)

    def _drive_to(self, target, what, final_yaw=True):
        """Straight to a pose: face it (or its back, to reverse), drive, then turn to its heading."""
        here = self._tf_pose('map')
        dist = math.hypot(target['x'] - here[0], target['y'] - here[1])
        if dist > 0.03:
            face, reverse = silo_route.drive_direction(here, target)
            self._turn_to(face)
            # the line in the odom frame, taken from the map once: smooth to follow
            mx, my, myaw = self._tf_pose('map')
            ox, oy, oyaw = self._tf_pose('odom')
            dyaw = oyaw - myaw
            c, s = math.cos(dyaw), math.sin(dyaw)
            rel = (target['x'] - mx, target['y'] - my)
            goal = (ox + c * rel[0] - s * rel[1], oy + s * rel[0] + c * rel[1])
            start = (ox, oy)
            speed = float(self.g('silo_speed'))
            end = time.monotonic() + dist / speed * 3 + 10
            try:
                while True:
                    x, y, yaw = self._tf_pose('odom')
                    if reverse:  # drive the back first: steer as if the back were the front
                        yaw = yaw + math.pi
                    v, w, _, done = straight_command((x, y, yaw), start, goal, speed, max_w=0.4)
                    if done:
                        break
                    if time.monotonic() > end:
                        raise StepFailed(f'could not reach {what} (blocked?)')
                    self._drive(-v if reverse else v, w)
                    self._sleep(0.05)
            finally:
                self._drive(0.0, 0.0)
        if final_yaw:
            self._turn_to(target['yaw'])

    # ---- steps --------------------------------------------------------------

    def _step_localize(self):
        docked = self._poses().get('docked')
        if docked and self._docked():
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = docked['x'], docked['y']
            msg.pose.pose.orientation.z = math.sin(docked['yaw'] / 2)
            msg.pose.pose.orientation.w = math.cos(docked['yaw'] / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.02 ** 2
            cov[35] = math.radians(2) ** 2
            msg.pose.covariance = cov
            for _ in range(3):
                self.initialpose_pub.publish(msg)
                self._sleep(0.2)
            self._sleep(1.0)  # let AMCL take it
            return 'position set from the docked pose'
        if not self.relocalize.wait_for_service(timeout_sec=1.0):
            raise StepFailed('not docked (or no docked pose saved) and no /relocalize service')
        res = self._wait(self.relocalize.call_async(Trigger.Request()), 40.0, 'relocalize')
        if not res.success:
            raise StepFailed(res.message)
        return res.message

    def _step_undock(self):
        if Undock is None:
            raise StepFailed('irobot_create_msgs is not installed')
        if not self._docked():
            return 'not docked, nothing to do'
        result = self._action(self.undock_client, Undock.Goal(), 60.0, 'undocking')
        if result.result.is_docked:
            raise StepFailed('still docked after Undock')
        return 'undocked'

    def _step_fill(self):
        poses = self._poses()
        targets = split_recipe(float(self.g('recipe_kg')), list(self.g('recipe')))
        dry = bool(self.g('dry_run'))
        self.status['fills'] = [{'silo': i + 1, 'target': t, 'added': 0.0, 'state': 'pending'}
                                for i, t in enumerate(targets)]
        last = silo_route.last_silo(targets)
        if last is None:
            return 'nothing to fill'
        silos = [poses.get(f'silo{i + 1}') for i in range(NUM_SILOS)]
        missing = [i + 1 for i in range(last + 1) if silos[i] is None]
        if missing:
            raise StepFailed(f'no pose saved for silo {", ".join(map(str, missing))} (Setup tab)')
        entry = poses.get('silo_entry') or silo_route.default_entry(
            silos[0], self.g('entry_side'), float(self.g('entry_m')))
        self._step_status('fill', 'running', 'driving to the silo entry')
        self._navigate(entry, 'the silo entry')
        done = []
        for i in range(last + 1):
            fill = self.status['fills'][i]
            fill['state'] = 'driving'
            self._step_status('fill', 'running', f'driving to silo {i + 1}')
            self._drive_to(silos[i], f'silo {i + 1}')  # silo 1 from the entry, then along the row
            if targets[i] <= 0:
                fill['state'] = 'skipped'
                continue
            self._sleep(float(self.g('settle_s')))
            fill['state'] = 'filling'
            if dry:
                self._step_status('fill', 'running', f'silo {i + 1}: dry run, valve stays closed')
                self._sleep(3.0)
                fill['state'] = 'done'
                done.append(f'silo {i + 1}: dry')
                continue
            added = self._fill_one(i, targets[i], fill)
            done.append(f'silo {i + 1}: {added:.2f} kg')
        # out the way it came: reverse along the row to silo 1, straight out to the entry
        self._step_status('fill', 'running', 'leaving the silos')
        if last > 0:
            self._drive_to(silos[0], 'silo 1', final_yaw=False)
        self._drive_to(entry, 'the silo entry', final_yaw=False)
        return ', '.join(done) if done else 'nothing to fill'

    def _fill_one(self, i, target, fill):
        start = self._weight()
        watch = FillWatch(target, self.g('stall_s'), self.g('stall_kg'),
                          self.g('fill_timeout_s'), t0=time.monotonic())
        shaken = False
        mode, mix_speed = str(self.g('fill_mix')), float(self.g('fill_mix_speed'))
        pour_mix = mode == 'pour' and mix_speed != 0.0
        # the mixer shakes the scale: a longer median keeps a jolt from ending the fill early
        window = 1.0 if pour_mix else 0.6
        pub = self.valve_pubs[i]
        pub.publish(Float32(data=float(self.g('valve_percent'))))
        try:
            while True:
                if pour_mix:
                    self._mixer(mix_speed)  # every 0.2 s: the ESP stops it after 1 s
                self._sleep(0.2)
                added = self._weight(window=window) - start
                fill['added'] = added
                self._step_status('fill', 'running',
                                  f'silo {i + 1}: {added:.2f} of {target:.2f} kg')
                state = watch.update(time.monotonic(), added)
                if state == 'done':
                    break
                if state == 'timeout':
                    raise StepFailed(f'silo {i + 1}: only {added:.2f} of {target:.2f} kg in '
                                     f'{self.g("fill_timeout_s"):.0f} s')
                if state == 'stalled':
                    if shaken:
                        raise StepFailed(f'silo {i + 1}: feed stopped flowing at {added:.2f} kg')
                    shaken = True
                    self._step_status('fill', 'running', f'silo {i + 1}: flow stalled, shaking')
                    self._shake(2.0)
                    watch.restart_stall(time.monotonic(), added)
        finally:
            pub.publish(Float32(data=0.0))
            if pour_mix:
                self._mixer(0.0)
        if mode == 'between' and mix_speed != 0.0:
            self._step_status('fill', 'running', f'silo {i + 1}: mixing')
            self._mix_for(float(self.g('fill_mix_s')), mix_speed)
        self._sleep(float(self.g('settle_s')))
        added = self._weight() - start
        fill['added'], fill['state'] = added, 'done'
        return added

    def _shake(self, seconds):
        n = len(self.motor_pubs)
        msg = Float32MultiArray(data=[150.0, 150.0] + [1.0] * n)
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                self.shake_pub.publish(msg)
                self._sleep(0.2)
        finally:
            self.shake_pub.publish(Float32MultiArray(data=[]))

    def _mix_for(self, seconds, speed, step=None):
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                self._mixer(speed)
                if step:
                    self._step_status(step, 'running', f'{end - time.monotonic():.0f} s left')
                self._sleep(0.1)
        finally:
            self._mixer(0.0)

    def _step_mix(self):
        seconds, speed = float(self.g('mix_s')), float(self.g('mix_speed'))
        self._mix_for(seconds, speed, step='mix')
        return f'mixed {seconds:.0f} s at {speed * 100:.0f}%'

    def _set_lap_params(self, values):
        if not self.lap_params.wait_for_service(timeout_sec=2.0):
            raise StepFailed('lap_driver is not running')
        req = SetParameters.Request()
        req.parameters = [_param(k, v) for k, v in values.items()]
        res = self._wait(self.lap_params.call_async(req), 5.0, 'setting the lap parameters')
        bad = [r.reason for r in res.results if not r.successful]
        if bad:
            raise StepFailed('; '.join(bad))

    def _one_lap(self):
        if not self.lap_start.wait_for_service(timeout_sec=2.0):
            raise StepFailed('lap_driver is not running')
        res = self._wait(self.lap_start.call_async(Trigger.Request()), 5.0, 'starting a lap')
        if not res.success:
            raise StepFailed(res.message)
        end = time.monotonic() + 15.0  # until lap_driver reports it is moving
        while not (self.lap_progress and self.lap_progress.get('state') in RUNNING):
            if time.monotonic() > end:
                raise StepFailed('lap_driver did not start')
            self._sleep(0.1)
        end = time.monotonic() + float(self.g('lap_timeout_s'))
        while self.lap_progress.get('state') in RUNNING:
            if time.monotonic() > end:
                if self.lap_stop.service_is_ready():
                    self.lap_stop.call_async(Trigger.Request())
                raise StepFailed(f'a lap took over {self.g("lap_timeout_s"):.0f} s: '
                                 f'{self.lap_progress.get("text") or "stuck"}')
            self._sleep(0.2)
        state = self.lap_progress.get('state')
        if state != 'done':
            raise StepFailed(f'lap ended: {self.lap_progress.get("text") or state}')

    def _step_feed(self):
        dry = bool(self.g('dry_run'))
        settings = pose_store.load_feed(self.g('poses_file'))
        allowed = ('feed', 'gap', 'end_gap', 'door_open', 'door_margin', 'door_x', 'speed',
                   'mix_below_kg', 'mix_speed')
        values = {k: (float(v) if k != 'feed' else str(v)) for k, v in settings.items() if k in allowed}
        mix = str(self.g('feed_mix'))
        values.update(
            laps=1, dry_run=dry, door_open=float(self.g('feed_door_open')),
            mix_speed=float(self.g('feed_mix_speed')),
            # the lap driver mixes once the weight is at or below this
            mix_below_kg={'always': MIX_ALWAYS_KG, 'off': 0.0}.get(
                mix, float(self.g('feed_mix_below_kg'))))
        self._set_lap_params(values)
        laps_max, empty = int(self.g('laps_max')), float(self.g('empty_kg'))
        start_kg = None if dry else self._weight()
        for lap in range(1, laps_max + 1):
            if not dry:
                left = self._weight()
                if left <= empty:
                    return f'{lap - 1} lap(s), {start_kg - left:.2f} kg fed'
                self._step_status('feed', 'running', f'lap {lap} of up to {laps_max}, {left:.2f} kg left')
            else:
                self._step_status('feed', 'running', f'lap {lap} of {laps_max} (dry run)')
            self._one_lap()
            self._sleep(float(self.g('settle_s')))
        if dry:
            return f'{laps_max} lap(s), dry run'
        left = self._weight()
        return f'{laps_max} lap(s), {start_kg - left:.2f} kg fed, {left:.2f} kg left'

    def _step_dock(self):
        if self._docked():
            return 'already on the dock'
        poses = self._poses()
        if 'docked' in poses:
            target, what = pose_store.predock_pose(poses['docked']), 'the front of the dock'
        elif 'start' in poses:
            target, what = poses['start'], 'the start pose'
        else:
            raise StepFailed('save the Docked pose in Setup, while the robot is on the dock')
        if Dock is None or 'docked' not in poses:
            self._navigate(target, what)
            return f'at {what} (no dock to drive onto)'
        self._step_status('dock', 'running', f'driving to {what}')
        try:
            self._navigate(target, what)
        except StepFailed:
            if not self._dock_visible():
                raise
            # close enough that the Create 3 sees its dock: let it do the rest
            self._step_status('dock', 'running', 'Nav2 stopped short, but the dock is in sight')
        self._step_status('dock', 'running', 'driving onto the dock')
        result = self._action(self.dock_client, Dock.Goal(), 90.0, 'docking')
        if not result.result.is_docked:
            raise StepFailed('the Create 3 could not dock (is the dock in front of it?)')
        return 'docked'


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_event.set()
        if rclpy.ok():
            node._safe_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
