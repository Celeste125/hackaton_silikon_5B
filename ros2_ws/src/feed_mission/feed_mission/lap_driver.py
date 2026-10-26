"""Drive laps around the cow box: straight lines and 90-degree turns on the spot.

    ros2 launch feed_mission mission.launch.py     (with navigation running)
    ros2 service call /lap/start std_srvs/srv/Trigger
    ros2 service call /lap/stop std_srvs/srv/Trigger

The four corners lap1..lap4 come from the poses file (web panel, Setup tab),
saved in driving order: clockwise around the box, because the door is on the
robot's right. Nav2 hesitates near walls by design (it slows down where cost
is high), so the laps bypass it:

  1. If the robot is not at lap1, Nav2 takes it there.
  2. Turn on the spot until it faces the next corner.
  3. Drive straight to it, steering back onto the corner-to-corner line in
     the map frame, so odometry drift is corrected on every side instead of
     adding up over the laps.
  4. Repeat for every side, `laps` times.

Safety: it stops while anything is in front of the robot (stop_distance), and
refuses a turn while anything is inside the circle its corners sweep. After
block_timeout_s it gives up. It drives through twist_mux (cmd_vel_lap,
priority between Nav2 and teleop), so teleop takes over at any time.
Parameters `laps`, `door_sides` and `door_open_position` are read at each start.
door_sides: none (door stays closed, e.g. dry runs), long (open on the two
longest sides only: feed along the long sides of the box) or all.
Status on /lap/status (latched): idle, approach, side 3/8 ..., done, failed: ...
"""

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

LOOP_HZ = 20.0


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
    """(v, w, remaining, done) to drive along the line start -> target.

    The robot steers toward a point `lookahead` ahead of itself on the line
    (pure pursuit on a straight line), so it converges back onto the line
    instead of only pointing at the corner. It slows down over the last
    `speed` metres and stops when it reaches the corner's perpendicular.
    """
    x, y, yaw = pose
    sx, sy = start
    tx, ty = target
    length = math.hypot(tx - sx, ty - sy)
    ux, uy = (tx - sx) / length, (ty - sy) / length
    remaining = (tx - x) * ux + (ty - y) * uy
    if remaining <= tol:
        return 0.0, 0.0, remaining, True
    along = (x - sx) * ux + (y - sy) * uy
    ax, ay = sx + ux * (along + lookahead), sy + uy * (along + lookahead)
    err = wrap(math.atan2(ay - y, ax - x) - yaw)
    w = max(-max_w, min(max_w, gain * err))
    v = max(min_v, min(speed, remaining))  # 1 m/s per m: slows to a stop
    if abs(err) > 0.5:  # far off the heading: turn before driving on
        v = 0.0
    return v, w, remaining, False


def door_open_sides(corners, mode):
    """Indices of the sides (side i runs corner i -> corner i+1) with the door open."""
    if mode == 'all':
        return {0, 1, 2, 3}
    if mode == 'long':
        lengths = [math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
        return set(sorted(range(4), key=lambda i: -lengths[i])[:2])
    if mode == 'none':
        return set()
    raise ValueError(f'door_sides must be none, long or all, not "{mode}"')


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


class LapDriver(Node):
    def __init__(self):
        super().__init__('lap_driver')
        p = self.declare_parameter
        p('poses_file', pose_store.DEFAULT_FILE)
        p('laps', 1)
        p('speed', 0.15)               # m/s on the straights
        p('turn_speed', 0.6)           # rad/s max on the spot
        p('stop_distance', 0.15)       # m free space needed in front
        p('block_timeout_s', 20.0)     # blocked this long -> give up
        p('approach_tolerance', 0.25)  # closer than this to lap1: no Nav2 needed
        p('half_length', 0.17)         # footprint, as in nav2_params local_costmap
        p('half_width', 0.26)
        p('door_sides', 'none')        # none, long (two longest sides) or all
        p('door_open_position', 0.0)   # 0 = fully open, 1 = closed
        p('cmd_vel_topic', '/Team2/cmd_vel_lap')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.door_pub = self.create_publisher(Float32, 'door/position', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'lap/status', latched)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, 'lap/start', self._on_start)
        self.create_service(Trigger, 'lap/stop', self._on_stop)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.scan = None
        self.state = 'idle'
        self.status = None
        self.corners = []
        self.segment = 0
        self.total = 0
        self.blocked_since = None
        self.nav_goal = None
        self.door_open = False
        self.door_sides = set()
        self._set_status('idle')
        self.create_timer(1.0 / LOOP_HZ, self._step)

    # ---- inputs -------------------------------------------------------------

    def _on_scan(self, msg):
        self.scan = msg

    def _pose(self):
        try:
            t = self.tf.lookup_transform(self.g('map_frame'), self.g('base_frame'), Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

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
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        r, a = r[ok], a[ok]
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, sn = math.cos(yaw), math.sin(yaw)
        lx, ly = r * np.cos(a), r * np.sin(a)
        return np.c_[c * lx - sn * ly + t.transform.translation.x,
                     sn * lx + c * ly + t.transform.translation.y]

    # ---- services -----------------------------------------------------------

    def _on_start(self, request, response):
        if self.state not in ('idle', 'done') and not self.state.startswith('failed'):
            response.success, response.message = False, f'already running ({self.status})'
            return response
        corners = pose_store.lap_corners(pose_store.load(self.g('poses_file')))
        if corners is None:
            response.success = False
            response.message = 'save all four lap corners first (web panel, Setup tab)'
            return response
        pose = self._pose()
        if pose is None:
            response.success = False
            response.message = 'robot not localized: no map -> base_link (start navigation)'
            return response
        try:
            self.door_sides = door_open_sides(corners, self.g('door_sides'))
        except ValueError as e:
            response.success, response.message = False, str(e)
            return response
        self.corners = corners
        self.segment = 0
        self.total = 4 * max(1, int(self.g('laps')))
        self.blocked_since = None
        if math.dist(pose[:2], corners[0]) > self.g('approach_tolerance'):
            if not self._send_nav_goal(corners[0], corners[1]):
                response.success = False
                response.message = (
                    f'robot is {math.dist(pose[:2], corners[0]):.2f} m from lap corner 1 and '
                    'Nav2 is not running: start navigation or drive it there')
                return response
            self._set_state('approach')
        else:
            self._set_state('turn')
        response.success, response.message = True, f'{self.total // 4} lap(s) started'
        return response

    def _on_stop(self, request, response):
        self._finish('idle')
        response.success, response.message = True, 'laps stopped'
        return response

    # ---- Nav2 to corner 1 ---------------------------------------------------

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
            self._finish('failed: Nav2 rejected the goal to lap corner 1')
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
            self._finish('failed: Nav2 could not reach lap corner 1')

    # ---- control loop -------------------------------------------------------

    def _step(self):
        if self.state not in ('turn', 'straight'):
            return
        pose = self._pose()
        if pose is None:
            self._finish('failed: lost localization (no map -> base_link)')
            return
        start = self.corners[self.segment % 4]
        target = self.corners[(self.segment + 1) % 4]
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
            self._blocked('obstacle ahead')
            return
        self._unblocked()
        self._door(self.segment % 4 in self.door_sides)
        v, w, _, done = straight_command(pose, start, target, self.g('speed'))
        if not done:
            self._drive(v, w)
            return
        self._drive(0.0, 0.0)
        self.segment += 1
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

    def _door(self, open_):
        if open_ != self.door_open:
            self.door_open = open_
            pos = self.g('door_open_position') if open_ else 1.0
            self.door_pub.publish(Float32(data=float(pos)))

    def _set_state(self, state):
        self.state = state
        if state == 'approach':
            self._set_status('approach: Nav2 to lap corner 1')
        elif state in ('turn', 'straight'):
            lap = self.segment // 4 + 1
            word = 'turning to' if state == 'turn' else 'driving to'
            self._set_status(
                f'lap {lap}/{self.total // 4}, side {self.segment % 4 + 1}/4: '
                f'{word} corner {(self.segment + 1) % 4 + 1}')
        else:
            self._set_status(state)

    def _set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(f'lap: {text}')

    def _finish(self, state):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None
        was_driving = self.state in ('turn', 'straight', 'approach')
        self.state = state
        if was_driving:
            self._drive(0.0, 0.0)
        self._door(False)
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
