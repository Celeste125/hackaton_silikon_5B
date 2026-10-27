"""Drive feeding laps around the cow box: straight sides, 90-degree turns on the spot.

    ros2 launch feed_mission mission.launch.py     (with navigation running)
    ros2 service call /lap/start std_srvs/srv/Trigger
    ros2 service call /lap/stop std_srvs/srv/Trigger

The lap is worked out from the cow box saved in the poses file (web panel,
Feed tab) by lap_geometry.plan_lap: a rectangle around the box, clockwise so
the box is on the robot's right where the door is, `gap` from the faces it
feeds. Nav2 hesitates near walls by design, so the laps bypass it:

  0. On the dock: Undock first (Create 3 action), then go on from there.
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

Disturbances: while anyone drives by hand (teleop) the laps pause, door
closed, sending nothing. Afterwards, after being blocked for a while, or if
the robot ends up more than off_line_m from the line, it rejoins: still close
to the line, it carries on; otherwise Nav2 takes it to the end of the side
(planning around the box) and the lap goes on from that corner. Steering
back onto a line from far away is what made it spin and zig-zag next to the
box.

Position watchdog: once a second it checks how much of the lidar lands on
the walls of the map (/map). Under min_fit for fit_pause_s it pauses
("position lost"): the lap lines follow the robot's believed position, so a
lost AMCL would steer it into the box. It rejoins once the match is back.

Safety: it stops while anything is in front of the robot (stop_distance) and
refuses a turn while anything is inside the circle its corners sweep; after
block_timeout_s it gives up. It drives through twist_mux (cmd_vel_lap, between
Nav2 and teleop), so teleop takes over at any time.

/lap/status (latched text) and /lap/progress (JSON: state, lap, laps, side,
door, along) report what it is doing.
"""

import json
import math
import time

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
from feed_mission.lap_geometry import Box, WallDistance, door_open_at, plan_lap, start_corner
from nav_msgs.msg import OccupancyGrid

try:  # undocking needs the Create 3 messages; without them laps start as they are
    from irobot_create_msgs.action import Undock
    from irobot_create_msgs.msg import DockStatus
except ImportError:
    Undock = DockStatus = None

DOCK_STALE_S = 3.0

LOOP_HZ = 20.0
PROGRESS_HZ = 5.0
ACTIVE = ('undock', 'approach', 'rejoin', 'turn', 'straight')  # a run is going on


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


def line_offset(pose, start, target):
    """(along, lateral, heading error, length) of pose relative to the line start -> target.

    lateral > 0: left of the line. heading error: line direction minus yaw.
    """
    x, y, yaw = pose
    length = math.hypot(target[0] - start[0], target[1] - start[1])
    ux, uy = (target[0] - start[0]) / length, (target[1] - start[1]) / length
    dx, dy = x - start[0], y - start[1]
    return dx * ux + dy * uy, ux * dy - uy * dx, wrap(math.atan2(uy, ux) - yaw), length


def rejoin_decision(pose, start, target, tol=0.12, end_tol=0.15):
    """After a disturbance: 'continue' on the line, 'next' (this side is as good
    as done), or 'navigate' (too far off: let Nav2 bring it to the side's end)."""
    along, lateral, _, length = line_offset(pose, start, target)
    if abs(length - along) < end_tol and abs(lateral) < 2 * tol:
        return 'next'  # at the corner (not pushed far past it)
    if abs(lateral) <= tol and -end_tol <= along <= length:
        return 'continue'
    return 'navigate'


def side_contact(points, half_len, half_w, margin=0.015):
    """True if a point is within margin of the robot's sides (it is brushing something)."""
    if len(points) == 0:
        return False
    x, y = np.abs(points[:, 0]), np.abs(points[:, 1])
    return bool(np.any((x <= half_len) & (y >= half_w - 0.02) & (y < half_w + margin)))


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
        p('rejoin_after_s', 1.5)       # blocked longer than this: check the lap is still right
        p('off_line_m', 0.20)          # further off the line than this: rejoin
        p('teleop_topic', '/Team2/cmd_vel_teleop')
        p('teleop_quiet_s', 1.0)       # no teleop for this long: driving by hand is over
        p('min_fit', 0.35)             # share of lidar on the map's walls below which it is lost
        p('fit_pause_s', 3.0)          # lost this long: pause until the match is back
        p('approach_tolerance', 0.25)  # closer than this to the first corner: no Nav2
        p('half_length', 0.17)         # footprint, as in nav2_params local_costmap
        p('half_width', 0.26)
        p('cmd_vel_topic', '/Team2/cmd_vel_lap')
        p('namespace', '/Team2')       # of the Create 3 (dock_status, undock)
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
        self.walls = None        # WallDistance of /map, for the position watchdog
        self.fit = None          # last share of lidar points on the map's walls
        self.lost_since = None   # monotonic time the fit dropped below min_fit
        self.lost = False        # paused because the position is lost
        latched_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched_map)
        self.create_timer(1.0, self._check_fit)
        self.teleop_t = 0.0      # last non-zero teleop command (monotonic)
        self.paused = False      # laps paused while someone drives by hand
        self.create_subscription(Twist, self.g('teleop_topic'), self._on_teleop, 10)
        self.dock_status = None  # (is_docked, monotonic time)
        self.undock_goal = None
        if DockStatus is not None:
            ns = self.g('namespace').rstrip('/')
            self.create_subscription(DockStatus, f'{ns}/dock_status', self._on_dock,
                                     qos_profile_sensor_data)
            self.undock_client = ActionClient(self, Undock, f'{ns}/undock')

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

    def _on_map(self, msg):
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self.walls = WallDistance(grid, info.resolution, info.origin.position.x, info.origin.position.y)

    def _check_fit(self):
        """Share of the lidar on the map's walls at the believed pose; tracks being lost."""
        if self.walls is None or self.scan is None or self.state not in ACTIVE:
            self.lost_since = None
            return
        pose = self._pose()
        pts = self._scan_points()[::3]
        if pose is None or len(pts) < 30:
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        near = sum(1 for x, y in pts
                   if self.walls(pose[0] + c * x - s * y, pose[1] + s * x + c * y) < 0.05)
        self.fit = near / len(pts)
        if self.fit < self.g('min_fit'):
            self.lost_since = self.lost_since or time.monotonic()
        else:
            self.lost_since = None

    def _position_lost(self):
        return self.lost_since is not None and time.monotonic() - self.lost_since > self.g('fit_pause_s')

    def _on_teleop(self, msg):
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.teleop_t = time.monotonic()

    def _teleop_active(self):
        return time.monotonic() - self.teleop_t < self.g('teleop_quiet_s')

    def _on_dock(self, msg):
        self.dock_status = (bool(msg.is_docked), time.monotonic())

    def _docked(self):
        s = self.dock_status
        return bool(s and s[0] and time.monotonic() - s[1] < DOCK_STALE_S)

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
        if self.state in ACTIVE:
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
        if self._pose() is None:
            response.message = 'robot not localized: no map -> base_link (start navigation)'
            return response
        self.plan = plan
        self.segment = 0
        self.total = 4 * max(1, int(self.g('laps')))
        self.blocked_since = None
        dry = ', dry run' if self.g('dry_run') else ''
        if self._docked():
            # Turning or driving on the dock goes nowhere: get off it first.
            if not self.undock_client.wait_for_server(timeout_sec=2.0):
                response.message = 'the robot is docked and the Create 3 undock action is not available'
                return response
            self._set_state('undock')
            self.undock_client.send_goal_async(Undock.Goal()).add_done_callback(self._on_undock_accepted)
            response.message = f'undocking first, then {self.total // 4} lap(s){dry}'
        else:
            ok, why = self._go()
            if not ok:
                response.message = why
                return response
            response.message = f'{self.total // 4} lap(s) started{dry}'
        response.success = True
        if plan['warnings']:
            response.message += '. Check: ' + '; '.join(plan['warnings'])
        return response

    def _go(self):
        """From where the robot is now: Nav2 to the nearest lap corner, or turn there."""
        pose = self._pose()
        if pose is None:
            return False, 'robot not localized: no map -> base_link (start navigation)'
        plan = self.plan
        first = start_corner(plan, pose[0], pose[1])
        self.order = [(first + i) % 4 for i in range(4)]
        start = plan['corners'][first]
        if math.dist(pose[:2], start) > self.g('approach_tolerance'):
            if not self._send_nav_goal(start, plan['corners'][(first + 1) % 4]):
                return False, (f'robot is {math.dist(pose[:2], start):.2f} m from the first lap '
                               'corner and Nav2 is not running: start navigation or drive it there')
            self._set_state('approach')
        else:
            self._set_state('turn')
        return True, ''

    def _on_undock_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self._finish('failed: the Create 3 refused to undock')
            return
        self.undock_goal = handle
        handle.get_result_async().add_done_callback(self._on_undock_result)

    def _on_undock_result(self, future):
        self.undock_goal = None
        if self.state != 'undock':
            return  # stopped meanwhile
        try:
            still_docked = future.result().result.is_docked
        except Exception:
            still_docked = True
        if still_docked:
            self._finish('failed: still on the dock after undocking')
            return
        ok, why = self._go()
        if not ok:
            self._finish(f'failed: {why}')

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
        if self.state not in ('approach', 'rejoin'):
            return  # stopped meanwhile
        if future.result().status != 4:  # not SUCCEEDED
            where = 'the first lap corner' if self.state == 'approach' else 'the lap'
            self._finish(f'failed: Nav2 could not bring the robot to {where}')
        elif self.state == 'rejoin':
            self._side_done()  # it is at the end of the side it left
        else:
            self._set_state('turn')

    # ---- getting back onto the lap ------------------------------------------

    def _rejoin(self, why):
        """After a disturbance: carry on, skip to the next corner, or Nav2 to it."""
        pose = self._pose()
        if pose is None:
            self._finish('failed: lost localization (no map -> base_link)')
            return
        side = self._side()
        start = self.plan['corners'][side['start']]
        target = self.plan['corners'][side['end']]
        decision = rejoin_decision(pose, start, target)
        self.get_logger().info(f'lap: rejoin {why}: {decision}')
        self.line = None  # re-taken from the map either way
        if decision == 'continue':
            self._set_state('straight' if self.state == 'straight' else 'turn')
            return
        if decision == 'next':
            self._side_done()
            return
        nxt = self.plan['sides'][self.order[(self.segment + 1) % 4]]
        if not self._send_nav_goal(target, self.plan['corners'][nxt['end']]):
            self._finish(f'failed: off the lap {why} and Nav2 is not running')
            return
        self._door(False)
        self._set_state('rejoin')
        self._set_status(f'rejoin: off the lap {why}, Nav2 to the next corner')

    # ---- control loop -------------------------------------------------------

    def _side(self):
        return self.plan['sides'][self.order[self.segment % 4]]

    def _step(self):
        if self.state not in ('turn', 'straight'):
            return
        # Someone drives by hand: pause, and rejoin once they have let go.
        if self._teleop_active():
            if not self.paused:
                self.paused = True
                self._door(False)
                self._set_status('paused: someone is driving by hand')
            return
        if self.paused:
            self.paused = False
            self._rejoin('after driving by hand')
            return
        # Lost: its idea of the lap no longer matches the room. Wait for AMCL.
        if self._position_lost():
            if not self.lost:
                self.lost = True
                self._drive(0.0, 0.0)
                self._door(False)
                self._set_status(f'paused: position lost ({self.fit * 100:.0f}% of the lidar on the map)'
                                 ', waiting for it to come back')
            return
        if self.lost:
            self.lost = False
            self._rejoin('after the position was lost')
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
            if self._unblocked():
                return
            target_yaw = math.atan2(target[1] - pose[1], target[0] - pose[0])
            w, done = turn_command(pose[2], target_yaw, self.g('turn_speed'))
            if done:
                self._drive(0.0, 0.0)
                self._set_state('straight')
                return
            self._drive(0.0, w)
            return

        if blocked_ahead(pts, hl, hw, self.g('stop_distance')):
            self._door(False)
            self._blocked('obstacle ahead')
            return
        if side_contact(pts, hl, hw):
            self._door(False)
            self._blocked('something is touching the side')
            return
        if self._unblocked():
            return
        if abs(line_offset(pose, start, target)[1]) > self.g('off_line_m'):
            self._drive(0.0, 0.0)
            self._rejoin('drifted off the line')
            return
        v, w, self.along, done = straight_command(pose, start, target, self.g('speed'))
        self._door(door_open_at(side, self.along))
        if not done:
            self._drive(v, w)
            return
        self._drive(0.0, 0.0)
        self._side_done()

    def _side_done(self):
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
        """Clear a block. True if that started a rejoin (the caller must stop here)."""
        if self.blocked_since is None:
            return False
        held = (self.get_clock().now() - self.blocked_since).nanoseconds * 1e-9
        self.blocked_since = None
        if held > self.g('rejoin_after_s'):
            # Blocked for a while: it may have been pushed, check before driving on.
            self._rejoin('after being blocked')
            return self.state not in ('turn', 'straight') or self.line is None
        self._set_state(self.state)  # restores the normal status text
        return False

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
        if state == 'rejoin':
            pass  # _rejoin sets the status with its reason
        elif state == 'undock':
            self._set_status('undock: getting off the dock first')
        elif state == 'approach':
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
        state = ('blocked' if self.blocked_since is not None else 'paused' if self.paused or self.lost
                 else self.state.split(':')[0])
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
            'fit': None if self.fit is None else round(self.fit, 2),
        })))

    def _finish(self, state):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None
        if self.undock_goal is not None:
            self.undock_goal.cancel_goal_async()
            self.undock_goal = None
        was_driving = self.state in ACTIVE
        self.paused = False
        self.lost = False
        self.lost_since = None
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
