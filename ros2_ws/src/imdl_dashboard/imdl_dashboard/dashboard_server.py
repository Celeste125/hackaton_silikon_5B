"""Web control panel for the Team2 Create3.

One process, no extra packages: rclpy for the robot side, the standard
library's ThreadingHTTPServer for the browser side.

    browser  <-- /events  (server-sent events, ~5 Hz state: pose, battery, ...)
    browser  <-- /stream/<camera>  (MJPEG, only decoded while someone watches)
    browser  <-- /map.png
    browser  --> POST /api/...  (drive, goto, speed, dock, places, ...)

Driving goes to <ns>/cmd_vel_teleop, so it outranks Nav2 when slam_navigation
runs twist_mux (see imdl_autonomy/config/twist_mux.yaml). Nav2 goals use the
navigate_to_pose action; the nav speed cap is Nav2's ``speed_limit`` topic.

Cameras are plain topic names (parameters ``camera_names`` / ``camera_topics``),
raw or compressed. A subscription exists only while a browser watches that
camera, so an idle dashboard costs the WiFi link nothing.
"""

import json
import math
import os
import re
import subprocess
import signal
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, CompressedImage, Image, LaserScan
from std_msgs.msg import Float32, String
from rosidl_runtime_py.utilities import get_message
import tf2_ros

try:
    from irobot_create_msgs.action import Dock, Undock
    from irobot_create_msgs.msg import DockStatus
except ImportError:  # dashboard still works for everything else
    Dock = Undock = DockStatus = None

DEFAULT_CAMERAS = {
    'realsense': '/camera/camera/color/image_raw',
    # Camera pointed at the mix. Topic is a guess: set camera_topics to the real one.
    'mezcla': '/mezcla/image_raw/compressed',
    'yolo': '/yolo/debug_image/compressed',
    'fusion': '/perception/fused_debug/compressed',
}
MAX_STREAM_WIDTH = 640
JPEG_QUALITY = 70
CAMERA_IDLE_S = 8.0
# Never let the browser ask for more than the arena's speed budget.
MAX_LINEAR_MPS = 0.30
MAX_ANGULAR_RPS = 1.2


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_msg(x, y, yaw, stamp):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def image_to_bgr(msg):
    """sensor_msgs/Image -> BGR uint8, without cv_bridge."""
    enc = msg.encoding
    wide = enc in ('16UC1', 'mono16')
    channels = {'mono8': 1, '8UC1': 1, 'mono16': 1, '16UC1': 1,
                'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}[enc]
    dtype = np.uint16 if wide else np.uint8
    row = np.frombuffer(msg.data, dtype).reshape(msg.height, -1)
    img = row[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if wide:  # depth: colour-map 0-4 m
        d8 = np.clip(img[:, :, 0].astype(np.float32) / 4000.0 * 255.0, 0, 255)
        return cv2.applyColorMap(d8.astype(np.uint8), cv2.COLORMAP_TURBO)
    if channels == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return np.ascontiguousarray(img)


def summarize(value, depth=0):
    """A ROS message as plain JSON, with big arrays (images, clouds) collapsed."""
    if hasattr(value, 'get_fields_and_field_types'):
        if depth > 6:
            return '...'
        return {f: summarize(getattr(value, f), depth + 1)
                for f in value.get_fields_and_field_types()}
    if isinstance(value, (bytes, bytearray)):
        return f'<{len(value)} bytes>'
    if isinstance(value, str):
        return value if len(value) <= 300 else value[:300] + '...'
    if isinstance(value, (list, tuple)) or hasattr(value, 'tolist'):
        n = len(value)
        if n > 24:
            return f'<{n} items>'
        return [summarize(v, depth + 1) for v in value]
    if isinstance(value, float):
        return None if math.isnan(value) or math.isinf(value) else round(value, 5)
    return value


def as_open(msg):
    """Open/closed from a door topic: Bool, Int/UInt (non-zero) or String."""
    data = getattr(msg, 'data', None)
    if isinstance(data, str):
        return data.strip().lower() in ('open', 'opened', 'true', '1', 'on')
    if data is None:
        return None
    return bool(data)


class Watch:
    """One topic being echoed for the topic browser; dropped when unwatched."""

    def __init__(self, name, type_name):
        self.name = name
        self.type_name = type_name
        self.sub = None
        self.last = None
        self.count = 0
        self.times = deque(maxlen=30)
        self.wanted = time.time()


class Seed:
    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self.sub = None
        self.type_name = None
        self.open = None  # None until the first message arrives
        self.stamp = 0.0
        self.since = 0.0


class Camera:
    """Latest JPEG of one image topic, subscribed only while watched."""

    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self.compressed = topic.endswith('/compressed')
        self.sub = None
        self.jpeg = None
        self.seq = 0
        self.stamp = 0.0
        self.last_wanted = 0.0
        self.watchers = 0
        self.cond = threading.Condition()
        self.last_bgr = None  # for the mix analyser


class MixAnalyzer:
    """Homogeneity of a region of a camera image, 0-100.

    This is a colour heuristic, not a trained model: the region is cut into a
    grid, and the score falls as the average Lab colour differs from cell to
    cell. A well-mixed batch looks the same everywhere. It needs a stable
    camera and lighting, and the thresholds below need calibrating on real
    good and bad batches.
    """

    GRID = 8

    def __init__(self):
        self.enabled = False
        self.camera = ''
        self.roi = [0.25, 0.25, 0.5, 0.5]  # x, y, w, h as image fractions
        self.good = 80.0  # = mix_quality_threshold 0.8 in waypoints.yaml
        self.fair = 45.0
        self.score = None
        self.label = 'no data'
        self.history = []
        self.updated = 0.0

    def config(self):
        return {'enabled': self.enabled, 'camera': self.camera,
                'roi': self.roi, 'good': self.good, 'fair': self.fair}

    def update(self, bgr):
        h, w = bgr.shape[:2]
        x, y, rw, rh = self.roi
        x0, y0 = int(x * w), int(y * h)
        x1, y1 = max(x0 + 16, int((x + rw) * w)), max(y0 + 16, int((y + rh) * h))
        patch = bgr[y0:y1, x0:x1]
        if patch.size == 0:
            return
        patch = cv2.resize(patch, (96, 96), interpolation=cv2.INTER_AREA)
        lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).astype(np.float32)
        n = self.GRID
        cell = 96 // n
        means = lab.reshape(n, cell, n, cell, 3).mean(axis=(1, 3)).reshape(-1, 3)
        # Spread of cell colours, in Lab units (about 1 unit = barely visible).
        spread = float(np.linalg.norm(means.std(axis=0)))
        score = 100.0 * math.exp(-spread / 14.0)
        self.score = round(score, 1)
        self.label = ('good' if score >= self.good
                      else 'fair' if score >= self.fair else 'poor')
        self.updated = time.time()
        self.history.append(self.score)
        del self.history[:-120]


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        self.declare_parameter('namespace', '/Team2')
        self.declare_parameter('port', 8080)
        self.declare_parameter('camera_names', list(DEFAULT_CAMERAS))
        self.declare_parameter('camera_topics', list(DEFAULT_CAMERAS.values()))
        self.declare_parameter(
            'places_file', os.path.expanduser('~/.imdl_dashboard/places.json'))
        self.declare_parameter('launch_prefix', 'ros2 launch')
        # Three seed doors. Each topic carries open/closed as Bool, an integer
        # (non-zero = open) or a String ("open"). These topic names are
        # placeholders: set seed_topics to what the door controller publishes.
        self.declare_parameter('seed_names', ['seed_1', 'seed_2', 'seed_3'])
        self.declare_parameter(
            'seed_topics', ['/seeds/door_1', '/seeds/door_2', '/seeds/door_3'])

        self.ns = self.get_parameter('namespace').value.rstrip('/')
        self.places_file = self.get_parameter('places_file').value
        self.lock = threading.RLock()

        names = self.get_parameter('camera_names').value
        topics = self.get_parameter('camera_topics').value
        self.cameras = {n: Camera(n, t) for n, t in zip(names, topics)}
        self.mix = MixAnalyzer()
        self.seeds = [Seed(n, t) for n, t in zip(
            self.get_parameter('seed_names').value,
            self.get_parameter('seed_topics').value)]
        self.watches = {}
        self.mix.camera = 'mezcla' if 'mezcla' in self.cameras else next(iter(self.cameras), '')
        self.mix_pub = self.create_publisher(Float32, '/mix_quality', 10)

        # Robot state shown in the UI.
        self.pose = None
        self.battery = None
        self.docked = None
        self.dock_visible = None
        self.scan_pts = []
        self.lidar = {'t': 0.0, 'points': 0, 'nearest': None}
        self.plan_pts = []
        self.map_png = None
        self.map_meta = None
        self.map_version = 0
        self._map_digest = None
        self.nav_state = 'idle'
        self.nav_text = ''
        self.nav_goal = None
        self.mission_state = ''
        self.objects = []
        self.beacon = None
        self.target_class = ''
        self.speed = {'nav': 0.18, 'teleop': 0.15}
        self.estop = False
        self.last_cmd = 0.0
        self.places = self._load_places()
        self.explorer = None
        self.log = []
        self._goal_handle = None
        self._after_nav = None  # callback after a successful goal

        qos_latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(
            Twist, f'{self.ns}/cmd_vel_teleop', 10)
        self.limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.target_pub = self.create_publisher(
            String, '/requested_target_class', 10)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos_latched)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(String, '/detected_objects', self._on_objects, 10)
        self.create_subscription(String, 'state_machine/state', self._on_mission, 10)
        self.create_subscription(Float32, '/beacon_cmd', self._on_beacon, 10)
        self.create_subscription(
            BatteryState, f'{self.ns}/battery_state', self._on_battery,
            qos_profile_sensor_data)
        if DockStatus is not None:
            self.create_subscription(
                DockStatus, f'{self.ns}/dock_status', self._on_dock_status,
                qos_profile_sensor_data)
            self.dock_client = ActionClient(self, Dock, f'{self.ns}/dock')
            self.undock_client = ActionClient(self, Undock, f'{self.ns}/undock')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.2, self._tick_pose)
        self.create_timer(0.25, self._tick_cameras)
        self.create_timer(0.5, self._tick_mix)
        self.create_timer(1.0, self._tick_seeds)
        self.create_timer(2.0, self._tick_watches)

    # ------------------------------------------------------------ utilities

    def say(self, text):
        self.get_logger().info(text)
        with self.lock:
            self.log.append({'t': time.strftime('%H:%M:%S'), 'msg': text})
            del self.log[:-40]

    def _load_places(self):
        try:
            with open(self.places_file, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _save_places(self):
        os.makedirs(os.path.dirname(self.places_file), exist_ok=True)
        tmp = self.places_file + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.places, f, indent=2)
        os.replace(tmp, self.places_file)

    # ----------------------------------------------------------- subscribers

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        digest = hash(data.tobytes())
        if digest == self._map_digest:  # slam_toolbox republishes unchanged maps
            return
        self._map_digest = digest
        # Dark palette (BGR) so robot, scan and labels stay readable on top.
        img = np.empty((h, w, 3), np.uint8)
        img[:] = (17, 13, 10)                         # unknown
        img[(data >= 0) & (data < 50)] = (53, 39, 27)  # free
        img[data >= 50] = (243, 237, 230)             # occupied
        img = np.flipud(img)                          # image row 0 = top
        ok, png = cv2.imencode('.png', img)
        if not ok:
            return
        with self.lock:
            self.map_png = png.tobytes()
            self.map_meta = {
                'w': w, 'h': h, 'res': msg.info.resolution,
                'ox': msg.info.origin.position.x,
                'oy': msg.info.origin.position.y,
            }
            self.map_version += 1

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        with self.lock:
            self.lidar = {'t': time.time(), 'points': len(valid),
                          'nearest': round(min(valid), 2) if valid else None}
        step = max(1, len(msg.ranges) // 120)
        pts = []
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                pts.append([round(r * math.cos(a), 2), round(r * math.sin(a), 2)])
        with self.lock:
            self.scan_pts = pts

    def _on_plan(self, msg):
        step = max(1, len(msg.poses) // 60)
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
               for p in msg.poses[::step]]
        with self.lock:
            self.plan_pts = pts

    def _on_objects(self, msg):
        try:
            raw = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(raw, dict):
            raw = raw.get('objects', [])
        objs = []
        for o in raw if isinstance(raw, list) else []:
            pm = o.get('position_map') or {}
            objs.append({
                'cls': o.get('class_name', '?'),
                'conf': round(float(o.get('confidence', 0.0)), 2),
                'depth': o.get('depth_m'),
                'x': pm.get('x'), 'y': pm.get('y'),
                'block': bool(o.get('is_block')),
            })
        with self.lock:
            self.objects = objs

    def _on_mission(self, msg):
        with self.lock:
            self.mission_state = msg.data

    def _on_beacon(self, msg):
        with self.lock:
            self.beacon = round(float(msg.data), 3)

    def _on_battery(self, msg):
        pct = msg.percentage
        if pct is not None and not math.isnan(pct):
            with self.lock:
                self.battery = round(pct * 100.0 if pct <= 1.0 else pct, 0)

    def _on_dock_status(self, msg):
        with self.lock:
            self.docked = bool(msg.is_docked)
            self.dock_visible = bool(msg.dock_visible)

    # ---------------------------------------------------------------- timers

    def _tick_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF not up yet is normal
            return
        tr = t.transform
        with self.lock:
            self.pose = {'x': round(tr.translation.x, 3),
                         'y': round(tr.translation.y, 3),
                         'yaw': round(yaw_of(tr.rotation), 3)}

    def _tick_cameras(self):
        now = time.time()
        wanted_mix = self.mix.enabled and self.mix.camera
        for cam in self.cameras.values():
            wanted = cam.watchers > 0 or cam.name == wanted_mix
            if wanted:
                cam.last_wanted = now
            if wanted and cam.sub is None:
                self._subscribe_camera(cam)
            elif cam.sub is not None and now - cam.last_wanted > CAMERA_IDLE_S:
                self.destroy_subscription(cam.sub)
                cam.sub = None

    def _subscribe_camera(self, cam):
        msg_type = CompressedImage if cam.compressed else Image
        cam.sub = self.create_subscription(
            msg_type, cam.topic, lambda m, c=cam: self._on_frame(c, m),
            qos_profile_sensor_data)

    def _on_frame(self, cam, msg):
        need_bgr = self.mix.enabled and self.mix.camera == cam.name
        try:
            if cam.compressed:
                buf = np.frombuffer(msg.data, np.uint8)
                bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            else:
                bgr = image_to_bgr(msg)
        except (KeyError, ValueError, cv2.error) as exc:
            self.get_logger().warn(f'{cam.name}: bad frame ({exc})',
                                   throttle_duration_sec=5.0)
            return
        if bgr is None:
            return
        if need_bgr:
            cam.last_bgr = bgr
        if bgr.shape[1] > MAX_STREAM_WIDTH:
            s = MAX_STREAM_WIDTH / bgr.shape[1]
            bgr = cv2.resize(bgr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        with cam.cond:
            cam.jpeg = jpg.tobytes()
            cam.seq += 1
            cam.stamp = time.time()
            cam.cond.notify_all()

    def _tick_mix(self):
        cam = self.cameras.get(self.mix.camera)
        if not self.mix.enabled or cam is None or cam.last_bgr is None:
            return
        self.mix.update(cam.last_bgr)
        if self.mix.score is not None:
            # 0..1, what imdl_statemachine's CHECK_MIX state reads.
            self.mix_pub.publish(Float32(data=float(self.mix.score) / 100.0))

    # --------------------------------------------------------------- commands

    def drive(self, linear, angular):
        if self.estop:
            return
        lin = max(-MAX_LINEAR_MPS, min(MAX_LINEAR_MPS, float(linear)))
        ang = max(-MAX_ANGULAR_RPS, min(MAX_ANGULAR_RPS, float(angular)))
        cap = self.speed['teleop']
        msg = Twist()
        msg.linear.x = max(-cap, min(cap, lin))
        msg.angular.z = ang
        self.cmd_pub.publish(msg)
        self.last_cmd = time.time()

    def stop_motion(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())

    def emergency_stop(self):
        self.estop = True
        self.cancel_nav()
        self.stop_explorer()
        self.stop_motion()
        self.say('EMERGENCY STOP')

    def release_estop(self):
        self.estop = False
        self.say('Emergency stop released')

    def set_speed(self, nav=None, teleop=None):
        if teleop is not None:
            self.speed['teleop'] = max(0.02, min(MAX_LINEAR_MPS, float(teleop)))
        if nav is not None:
            self.speed['nav'] = max(0.02, min(MAX_LINEAR_MPS, float(nav)))
            msg = SpeedLimit()
            msg.header.frame_id = 'map'
            msg.percentage = False
            msg.speed_limit = self.speed['nav']
            self.limit_pub.publish(msg)

    def goto(self, x, y, yaw, label='', after=None):
        if self.estop:
            self.say('Emergency stop active: not navigating')
            return False
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.say('Nav2 not responding (navigate_to_pose)')
            return False
        self.cancel_nav()
        goal = NavigateToPose.Goal()
        goal.pose = pose_msg(x, y, yaw, self.get_clock().now().to_msg())
        self._after_nav = after
        with self.lock:
            self.nav_state = 'sending'
            self.nav_text = label or f'({x:.2f}, {y:.2f})'
            self.nav_goal = {'x': x, 'y': y, 'yaw': yaw}
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)
        self.say(f'Heading to {self.nav_text}')
        return True

    def _on_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            with self.lock:
                self.nav_state = 'rejected'
            self.say('Nav2 rejected the goal')
            return
        self._goal_handle = handle
        with self.lock:
            self.nav_state = 'running'
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, fut):
        status = fut.result().status
        names = {4: 'done', 5: 'canceled', 6: 'failed'}
        state = names.get(status, 'failed')
        with self.lock:
            self.nav_state = state
            self._goal_handle = None
        self.say(f'Navigation: {state}')
        after, self._after_nav = self._after_nav, None
        if state == 'done' and after:
            after()

    def cancel_nav(self):
        handle = self._goal_handle
        self._after_nav = None
        if handle is not None:
            handle.cancel_goal_async()
        with self.lock:
            if self.nav_state in ('running', 'sending'):
                self.nav_state = 'canceled'

    def dock(self):
        if Dock is None or not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.say('Dock action not available')
            return
        self.say('Docking to the charger')
        self.dock_client.send_goal_async(Dock.Goal())

    def undock(self):
        if Undock is None or not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.say('Undock action not available')
            return
        self.say('Undocking')
        self.undock_client.send_goal_async(Undock.Goal())

    def go_place(self, name):
        p = self.places.get(name)
        if p is None:
            self.say(f'Unknown place: {name}')
            return False
        after = self.dock if p.get('dock') else None
        return self.goto(p['x'], p['y'], p['yaw'], label=name, after=after)

    def save_place(self, name, dock=False):
        with self.lock:
            pose = self.pose
        if pose is None:
            self.say('No robot pose: cannot save the place')
            return False
        self.places[name] = {**pose, 'dock': bool(dock)}
        self._save_places()
        self.say(f'Place saved: {name}')
        return True

    def set_place(self, name, x, y, yaw, dock=False):
        self.places[name] = {'x': x, 'y': y, 'yaw': yaw, 'dock': bool(dock)}
        self._save_places()

    def delete_place(self, name):
        if self.places.pop(name, None) is not None:
            self._save_places()

    def set_target(self, cls):
        self.target_class = cls
        self.target_pub.publish(String(data=cls))

    def start_explorer(self):
        if self.explorer and self.explorer.poll() is None:
            return
        self.explorer = subprocess.Popen(
            ['ros2', 'launch', 'imdl_autonomy', 'frontier_explore.launch.py'],
            start_new_session=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        self.say('Autonomous exploration started')

    def stop_explorer(self):
        if self.explorer and self.explorer.poll() is None:
            os.killpg(self.explorer.pid, signal.SIGINT)
            self.say('Exploration stopped')
        self.explorer = None


    # ----------------------------------------------------------- seed doors

    def _tick_seeds(self):
        graph = None
        for seed in self.seeds:
            if seed.sub is not None:
                continue
            if graph is None:
                graph = dict(self.get_topic_names_and_types())
            types = graph.get(seed.topic)
            if not types:
                continue
            try:
                msg_type = get_message(types[0])
            except (ValueError, AttributeError, ModuleNotFoundError):
                continue
            seed.type_name = types[0]
            seed.sub = self.create_subscription(
                msg_type, seed.topic, lambda m, s=seed: self._on_seed(s, m),
                qos_profile_sensor_data)

    def _on_seed(self, seed, msg):
        state = as_open(msg)
        if state is None:
            return
        now = time.time()
        if state != seed.open:
            seed.since = now
            self.say(f'{seed.name}: {"OPEN" if state else "closed"}')
        seed.open = state
        seed.stamp = now

    # --------------------------------------------------------- topic browser

    def list_topics(self):
        rows = []
        for name, types in sorted(self.get_topic_names_and_types()):
            rows.append({
                'name': name, 'type': types[0],
                'pubs': self.count_publishers(name),
                'subs': self.count_subscribers(name),
            })
        return rows

    def echo_topic(self, name):
        """Latest message and rate of one topic; starts watching on first call."""
        w = self.watches.get(name)
        if w is None:
            types = dict(self.get_topic_names_and_types()).get(name)
            if not types:
                raise KeyError(name)
            msg_type = get_message(types[0])
            w = Watch(name, types[0])
            self.watches[name] = w
            w.sub = self.create_subscription(
                msg_type, name, lambda m, w=w: self._on_watch(w, m),
                qos_profile_sensor_data)
        w.wanted = time.time()
        times = list(w.times)
        hz = 0.0
        if len(times) >= 2 and time.monotonic() - times[-1] < 5.0 and times[-1] > times[0]:
            hz = (len(times) - 1) / (times[-1] - times[0])
        last = w.last
        return {'name': name, 'type': w.type_name, 'hz': round(hz, 2),
                'count': w.count, 'age': None if not times else round(time.monotonic() - times[-1], 1),
                'msg': summarize(last) if last is not None else None}

    def _on_watch(self, w, msg):
        w.last = msg
        w.count += 1
        w.times.append(time.monotonic())

    def _tick_watches(self):
        now = time.time()
        for name in [n for n, w in self.watches.items() if now - w.wanted > 10.0]:
            self.destroy_subscription(self.watches.pop(name).sub)

    # ------------------------------------------------------------------ state

    def snapshot(self):
        with self.lock:
            return {
                'pose': self.pose, 'battery': self.battery,
                'docked': self.docked, 'dock_visible': self.dock_visible,
                'scan': self.scan_pts, 'plan': self.plan_pts,
                'lidar': {'live': time.time() - self.lidar['t'] < 2.0,
                          'points': self.lidar['points'],
                          'nearest': self.lidar['nearest']},
                'map': self.map_meta, 'map_version': self.map_version,
                'nav': {'state': self.nav_state, 'text': self.nav_text,
                        'goal': self.nav_goal},
                'mission': self.mission_state, 'objects': self.objects,
                'beacon': self.beacon, 'target': self.target_class,
                'speed': self.speed, 'estop': self.estop,
                'explore': bool(self.explorer and self.explorer.poll() is None),
                'places': self.places, 'log': self.log[-12:],
                'cameras': {n: {'topic': c.topic,
                                'live': time.time() - c.stamp < 3.0}
                            for n, c in self.cameras.items()},
                'mix': {**self.mix.config(), 'score': self.mix.score,
                        'label': self.mix.label,
                        'history': self.mix.history[-60:]},
                'seeds': [{'name': q.name, 'topic': q.topic, 'open': q.open,
                           'subscribed': q.sub is not None,
                           'secs': round(time.time() - q.since) if q.since else None}
                          for q in self.seeds],
                'limits': {'linear': MAX_LINEAR_MPS, 'angular': MAX_ANGULAR_RPS},
            }


def make_handler(node, web_dir):
    static = {'/': 'index.html', '/index.html': 'index.html'}
    mime = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript',
            '.css': 'text/css'}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):  # keep the ROS console readable
            pass

        def _send(self, code, body, ctype='application/json', extra=None):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            try:
                if path in static:
                    fp = os.path.join(web_dir, static[path])
                    with open(fp, 'rb') as f:
                        self._send(200, f.read(), mime['.html'])
                elif path == '/events':
                    self._events()
                elif path == '/map.png':
                    png = node.map_png
                    if png is None:
                        self._send(404, b'no map', 'text/plain')
                    else:
                        self._send(200, png, 'image/png')
                elif path.startswith('/stream/'):
                    self._stream(path[len('/stream/'):])
                elif path == '/api/topics':
                    self._send(200, json.dumps(node.list_topics()).encode())
                elif path == '/api/topic':
                    name = parse_qs(urlparse(self.path).query).get('name', [''])[0]
                    try:
                        body = json.dumps(node.echo_topic(name)).encode()
                    except (KeyError, ValueError, ModuleNotFoundError, AttributeError):
                        self._send(404, b'{"error":"unknown topic"}')
                    else:
                        self._send(200, body)
                elif path.startswith('/snapshot/'):
                    cam = node.cameras.get(path[len('/snapshot/'):])
                    if cam is None or cam.jpeg is None:
                        self._send(404, b'no frame', 'text/plain')
                    else:
                        self._send(200, cam.jpeg, 'image/jpeg')
                else:
                    self._send(404, b'not found', 'text/plain')
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    data = json.dumps(node.snapshot())
                    self.wfile.write(f'data: {data}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def _stream(self, name):
            cam = node.cameras.get(name)
            if cam is None:
                self._send(404, b'unknown camera', 'text/plain')
                return
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            with cam.cond:
                cam.watchers += 1
            last = -1
            try:
                while True:
                    with cam.cond:
                        cam.cond.wait_for(lambda: cam.seq != last, timeout=2.0)
                        jpg, last = cam.jpeg, cam.seq
                    if jpg is None:
                        continue
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n')
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with cam.cond:
                    cam.watchers -= 1

        def do_POST(self):
            path = urlparse(self.path).path
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                self._send(400, b'{"error":"bad json"}')
                return
            try:
                result = self._api(path, body)
            except (KeyError, ValueError, TypeError) as exc:
                self._send(400, json.dumps({'error': str(exc)}).encode())
                return
            self._send(200, json.dumps({'ok': result is not False}).encode())

        def _api(self, path, b):
            n = node
            if path == '/api/drive':
                n.drive(b['linear'], b['angular'])
            elif path == '/api/stop':
                n.stop_motion()
                n.cancel_nav()
            elif path == '/api/estop':
                n.emergency_stop()
            elif path == '/api/estop/release':
                n.release_estop()
            elif path == '/api/goto':
                return n.goto(float(b['x']), float(b['y']), float(b.get('yaw', 0.0)))
            elif path == '/api/cancel':
                n.cancel_nav()
            elif path == '/api/speed':
                n.set_speed(b.get('nav'), b.get('teleop'))
            elif path == '/api/dock':
                n.dock()
            elif path == '/api/undock':
                n.undock()
            elif path == '/api/place/go':
                return n.go_place(b['name'])
            elif path == '/api/place/save':
                name = re.sub(r'[^\w .-]', '', str(b['name']))[:40].strip()
                if not name:
                    raise ValueError('name')
                if 'x' in b:
                    n.set_place(name, float(b['x']), float(b['y']),
                                float(b.get('yaw', 0.0)), b.get('dock', False))
                else:
                    return n.save_place(name, b.get('dock', False))
            elif path == '/api/place/delete':
                n.delete_place(b['name'])
            elif path == '/api/explore':
                n.start_explorer() if b.get('on') else n.stop_explorer()
            elif path == '/api/target':
                n.set_target(str(b['class']))
            elif path == '/api/mix':
                m = n.mix
                if 'enabled' in b:
                    m.enabled = bool(b['enabled'])
                if 'camera' in b and b['camera'] in n.cameras:
                    m.camera = b['camera']
                if 'roi' in b:
                    x, y, w, h = (max(0.0, min(1.0, float(v))) for v in b['roi'])
                    m.roi = [x, y, max(0.05, w), max(0.05, h)]
                if 'good' in b:
                    m.good = float(b['good'])
                if 'fair' in b:
                    m.fair = float(b['fair'])
            else:
                raise KeyError(path)
            return True

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    web_dir = os.path.join(get_package_share_directory('imdl_dashboard'), 'web')
    port = int(node.get_parameter('port').value)
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node, web_dir))
    server.daemon_threads = True
    node.say(f'Dashboard at http://0.0.0.0:{port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.stop_explorer()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
