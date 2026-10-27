"""Drive feeding laps around the cow box: straight sides, 90-degree turns on the spot.

    ros2 launch feed_mission mission.launch.py     (with navigation running)
    ros2 service call /lap/start std_srvs/srv/Trigger
    ros2 service call /lap/stop std_srvs/srv/Trigger

The lap is worked out from the cow box saved in the poses file (web panel,
Feed tab) by lap_geometry.plan_lap: a rectangle around the box, clockwise so
the box is on the robot's right where the door is, `gap` from the faces it
feeds. Nav2 hesitates near walls by design, so the laps bypass it:

  1. Start at the lap corner nearest the robot; Nav2 takes it there if needed.
  2. Turn on the spot until it faces the next corner.
  3. Drive straight to it, steering back onto the corner-to-corner line.
     The line is taken from the map once per side and then followed on
     odometry: AMCL corrects the map pose in steps of a cm or more, and
     steering on those steps made the robot twitch. Drift is still corrected
     at every corner, when the next side is taken from the map.
  4. On a fed side the door opens by `door_open` (0..1) while the door is
     beside the box face, and closes before the end of the box.
  5. Repeat every side, `laps` times.

dry_run drives the same laps with the door kept closed (progress still shows
where it would open). Parameters are read at each start.

Safety: it stops while anything is in front of the robot (stop_distance) and
refuses a turn while anything is inside the circle its corners sweep; after
block_timeout_s it gives up. It drives through twist_mux (cmd_vel_lap, between
Nav2 and teleop), so teleop takes over at any time.

/lap/status (latched text) and /lap/progress (JSON: state, lap, laps, side,
door, along) report what it is doing.
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from feed_mission import poses as pose_store
from feed_mission.lap_geometry import Box, door_open_at, plan_lap, start_corner

LOOP_HZ = 20.0
PROGRESS_HZ = 5.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def turn_command(yaw, target_yaw, max_w, gain=2.0, min_w=0.15, tol=0.03):
    """(angular velocity, done) to turn on the spot to target_yaw."""
    err = wrap(target_yaw - yaw)
    if abs(err) < tol:
        return 0.0, True
    w = max(-max_w, min(max_w, gain * err))
    if abs(w) < min_w:  # the base does not move below this
        w = math.copysign(min_w, err)
    return w, False


def straight_command(pose, start, target, speed, max_w=0.6, gain=2.5,
                     lookahead=0.3, min_v=0.04, tol=0.02):
    """(v, w, along, done) to drive along the line start -> target.

    The robot steers toward a point `lookahead` ahead of itself on the line
    (pure pursuit on a straight line), so it converges back onto the line
    instead of only pointing at the corner. It slows down over the last
    `speed` metres and stops when it reaches the corner's perpendicular.
    `along` is how far along the line it is, for the door.
    """
    x, y, yaw = pose
    sx, sy = start
    tx, ty = target
    length = math.hypot(tx - sx, ty - sy)
    ux, uy = (tx - sx) / length, (ty - sy) / length
    along = (x - sx) * ux + (y - sy) * uy
    remaining = length - along
    if remaining <= tol:
        return 0.0, 0.0, along, True
    ax, ay = sx + ux * (along + lookahead), sy + uy * (along + lookahead)
    err = wrap(math.atan2(ay - y, ax - x) - yaw)
    w = max(-max_w, min(max_w, gain * err))
    v = max(min_v, min(speed, remaining))  # 1 m/s per m: slows to a stop
    if abs(err) > 0.5:  # far off the heading: turn before driving on
        v = 0.0
    return v, w, along, False


def blocked_ahead(points, half_len, half_w, stop_distance):
    """True if a point (base_link, Nx2) is in front of the robot within stop_distance."""
    if len(points) == 0:
        return False
    x, y = points[:, 0], points[:, 1]
    return bool(np.any((x > half_len) & (x < half_len + stop_distance) & (np.abs(y) < half_w + 0.02)))


def turn_blocked(points, half_len, half_w, margin=0.02):
    """True if a point is inside the circle the footprint's corners sweep."""
    if len(points) == 0:
        return False
    reach = math.hypot(half_len, half_w) + margin
    return bool(np.any(np.hypot(points[:, 0], points[:, 1]) < reach))


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LapDriver(Node):
    def __init__(self):
        super().__init__('lap_driver')
        p = self.declare_parameter
        p('poses_file', pose_store.DEFAULT_FILE)
        p('laps', 1)
        p('feed', 'long')              # faces to feed: long, all or none
        p('gap', 0.10)                 # m, robot's side to a fed face
        p('end_gap', 0.15)             # m, robot's side to the other faces
        p('door_open', 0.2)            # how far the door opens, 0..1
        p('door_margin', 0.05)         # m, door opens this far inside each end of a face
        p('door_x', 0.0)               # m, door position ahead of the robot's centre
        p('dry_run', False)            # drive the laps, keep the door closed
        p('speed', 0.15)               # m/s on the straights
        p('turn_speed', 0.6)           # rad/s max on the spot
        p('stop_distance', 0.15)       # m free space needed in front
        p('block_timeout_s', 20.0)     # blocked this long -> give up
        p('approach_tolerance', 0.25)  # closer than this to the first corner: no Nav2
        p('half_length', 0.17)         # footprint, as in nav2_params local_costmap
        p('half_width', 0.26)
        p('cmd_vel_topic', '/Team2/cmd_vel_lap')
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        self.g = lambda n: self.get_parameter(n).value

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Twist, self.g('cmd_vel_topic'), 10)
        self.door_pub = self.create_publisher(Float32, 'door/position', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'lap/status', latched)
        self.progress_pub = self.create_publisher(String, 'lap/progress', latched)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, 'lap/start', self._on_start)
        self.create_service(Trigger, 'lap/stop', self._on_stop)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.scan = None
        self.state = 'idle'
        self.status = None
        self.plan = None
        self.order = []          # plan side indices in driving order
        self.segment = 0         # sides driven so far
        self.total = 0
        self.along = 0.0
        self.line = None         # (start, target, frame) of the side being driven
        self.blocked_since = None
        self.nav_goal = None
        self.door_open = False   # commanded
        self.door_wanted = False  # by the plan, also in a dry run
        self._set_status('idle')
        self.create_timer(1.0 / LOOP_HZ, self._step)
        self.create_timer(1.0 / PROGRESS_HZ, self._publish_progress)

    # ---- inputs -------------------------------------------------------------

    def _on_scan(self, msg):
        self.scan = msg

    def _pose(self, frame=None):
        try:
            t = self.tf.lookup_transform(frame or self.g('map_frame'), self.g('base_frame'), Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    def _begin_side(self):
        """Take the next side from the map into the odom frame, to drive it on odometry."""
        side = self._side()
        a, b = self.plan['corners'][side['start']], self.plan['corners'][side['end']]
        try:
            t = self.tf.lookup_transform(self.g('odom_frame'), self.g('map_frame'), Time())
        except TransformException:
            self.line = (a, b, self.g('map_frame'))  # no odom: steer on the map pose
            return
        c, s = math.cos(_yaw(t.transform.rotation)), math.sin(_yaw(t.transform.rotation))
        tx, ty = t.transform.translation.x, t.transform.translation.y
        to_odom = lambda p: (tx + c * p[0] - s * p[1], ty + s * p[0] + c * p[1])  # noqa: E731
        self.line = (to_odom(a), to_odom(b), self.g('odom_frame'))

    def _scan_points(self):
        """Latest scan as Nx2 points in base_link (the lidar is mounted turned)."""
        s = self.scan
        if s is None:
            return np.zeros((0, 2))
        try:
            t = self.tf.lookup_transform(self.g('base_frame'), s.header.frame_id, Time())
        except TransformException:
            return np.zeros((0, 2))
        r = np.asarray(s.ranges, dtype=float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + _yaw(t.transform.rotation)
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        r, a = r[ok], a[ok]
        return np.c_[r * np.cos(a) + t.transform.translation.x,
                     r * np.sin(a) + t.transform.translation.y]

    # ---- services -----------------------------------------------------------

    def _on_start(self, request, response):
        response.success = False
        if self.state in ('approach', 'turn', 'straight'):
            response.message = f'already running ({self.status})'
            return response
        box = pose_store.load_box(self.g('poses_file'))
        if box is None:
            response.message = 'no cow box saved yet: measure it in the Feed tab'
            return response
        try:
            plan = plan_lap(Box.from_dict(box), gap=self.g('gap'), end_gap=self.g('end_gap'),
                            feed=self.g('feed'), margin=self.g('door_margin'),
                            door_x=self.g('door_x'))
        except ValueError as e:
            response.message = str(e)
            return response
        pose = self._pose()
        if pose is None:
            response.message = 'robot not localized: no map -> base_link (start navigation)'
            return response
        first = start_corner(plan, pose[0], pose[1])
        self.plan = plan
        self.order = [(first + i) % 4 for i in range(4)]
        self.segment = 0
        self.total = 4 * max(1, int(self.g('laps')))
        self.blocked_since = None
        start = plan['corners'][first]
        if math.dist(pose[:2], start) > self.g('approach_tolerance'):
            if not self._send_nav_goal(start, plan['corners'][(first + 1) % 4]):
                response.message = (
                    f'robot is {math.dist(pose[:2], start):.2f} m from the first lap corner '
                    'and Nav2 is not running: start navigation or drive it there')
                return response
            self._set_state('approach')
        else:
            self._set_state('turn')
        response.success = True
        dry = ', dry run' if self.g('dry_run') else ''
        response.message = f'{self.total // 4} lap(s) started{dry}'
        if plan['warnings']:
            response.message += '. Check: ' + '; '.join(plan['warnings'])
        return response

    def _on_stop(self, request, response):
        self._finish('idle')
        response.success, response.message = True, 'laps stopped'
        return response

    # ---- Nav2 to the first corner -------------------------------------------

    def _send_nav_goal(self, corner, next_corner):
        if not self.nav.wait_for_server(timeout_sec=1.0):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.g('map_frame')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = corner
        yaw = math.atan2(next_corner[1] - corner[1], next_corner[0] - corner[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav_goal = None
        self.nav.send_goal_async(goal).add_done_callback(self._on_nav_accepted)
        return True

    def _on_nav_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self._finish('failed: Nav2 rejected the goal to the first lap corner')
            return
        self.nav_goal = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self.nav_goal = None
        if self.state != 'approach':
            return  # stopped meanwhile
        if future.result().status == 4:  # SUCCEEDED
            self._set_state('turn')
        else:
            self._finish('failed: Nav2 could not reach the first lap corner')

    # ---- control loop -------------------------------------------------------

    def _side(self):
        return self.plan['sides'][self.order[self.segment % 4]]

    def _step(self):
        if self.state not in ('turn', 'straight'):
            return
        if self.line is None:
            self._begin_side()
        start, target, frame = self.line
        pose = self._pose(frame)
        if pose is None:
            self._finish(f'failed: lost the robot pose (no {frame} -> base_link)')
            return
        side = self._side()
        pts = self._scan_points()
        hl, hw = self.g('half_length'), self.g('half_width')

        if self.state == 'turn':
            self._door(False)
            if turn_blocked(pts, hl, hw):
                self._blocked('turning: something is within reach of the corners')
                return
            target_yaw = math.atan2(target[1] - pose[1], target[0] - pose[0])
            w, done = turn_command(pose[2], target_yaw, self.g('turn_speed'))
            if done:
                self._drive(0.0, 0.0)
                self._set_state('straight')
                return
            self._unblocked()
            self._drive(0.0, w)
            return

        if blocked_ahead(pts, hl, hw, self.g('stop_distance')):
            self._door(False)
            self._blocked('obstacle ahead')
            return
        self._unblocked()
        v, w, self.along, done = straight_command(pose, start, target, self.g('speed'))
        self._door(door_open_at(side, self.along))
        if not done:
            self._drive(v, w)
            return
        self._drive(0.0, 0.0)
        self._door(False)
        self.segment += 1
        self.along = 0.0
        if self.segment >= self.total:
            self._finish('done')
        else:
            self._set_state('turn')

    def _blocked(self, why):
        self._drive(0.0, 0.0)
        now = self.get_clock().now()
        if self.blocked_since is None:
            self.blocked_since = now
        if now - self.blocked_since > Duration(seconds=self.g('block_timeout_s')):
            self._finish(f'failed: blocked too long ({why})')
        else:
            self._set_status(f'blocked: {why}')

    def _unblocked(self):
        if self.blocked_since is not None:
            self.blocked_since = None
            self._set_state(self.state)  # restores the normal status text

    # ---- outputs ------------------------------------------------------------

    def _drive(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def _door(self, wanted):
        self.door_wanted = wanted
        open_ = wanted and not self.g('dry_run') and self.g('door_open') > 0.0
        if open_ != self.door_open:
            self.door_open = open_
            # door/position: 0 = fully open, 1 = closed
            pos = 1.0 - max(0.0, min(1.0, float(self.g('door_open')))) if open_ else 1.0
            self.door_pub.publish(Float32(data=pos))

    def _set_state(self, state):
        if state == 'turn' and self.state != 'turn':
            self.line = None  # a new side: taken from the map on the next step
        self.state = state
        if state == 'approach':
            self._set_status('approach: Nav2 to the first lap corner')
        elif state in ('turn', 'straight'):
            lap = self.segment // 4 + 1
            word = 'turning' if state == 'turn' else 'driving'
            what = 'feeding side' if self._side()['door'] else 'side'
            self._set_status(f'lap {lap}/{self.total // 4}: {word}, {what} {self.segment % 4 + 1}/4')
        else:
            self._set_status(state)

    def _set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(f'lap: {text}')
            self._publish_progress()

    def _publish_progress(self):
        running = self.state in ('turn', 'straight')
        state = 'blocked' if self.blocked_since is not None else self.state.split(':')[0]
        self.progress_pub.publish(String(data=json.dumps({
            'state': state,
            'text': self.status,
            'lap': self.segment // 4 + 1 if running else None,
            'laps': self.total // 4 if self.total else None,
            'side': self.order[self.segment % 4] if running else None,
            'along': round(self.along, 3) if running else None,
            'door': self.door_open,
            'door_planned': self.door_wanted and running,
            'dry_run': bool(self.g('dry_run')),
        })))

    def _finish(self, state):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None
        was_driving = self.state in ('turn', 'straight', 'approach')
        self.state = state
        self.blocked_since = None
        if was_driving:
            self._drive(0.0, 0.0)
        self._door(False)
        self.along = 0.0
        self._set_status(state)


def main(args=None):
    rclpy.init(args=args)
    node = LapDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._finish('idle')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
