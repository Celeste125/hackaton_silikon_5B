"""Feed tab back end: map, live scan, cow box, lap plan, laps and the run log.

Needs feed_mission (lap_geometry, poses) and a running lap_driver for laps.
Everything is in the map frame. The page draws the map itself from /api/map
(the raw occupancy grid, sent again only when it changes) and polls /api/scan
for the lidar points while the map view is open.

Every lap run is logged with its settings and the weight before and after,
so the door opening can be tuned against kg of feed per metre of open door.
"""

import base64
import json
import math
import threading
import time

import numpy as np
from pydantic import BaseModel, Field
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException

from feed_mission import poses as pose_store
from feed_mission.lap_geometry import (
    Box, WallDistance, check_plan, door_length, fit_box_from_map, fit_box_to_points,
    lap_length, normalized, plan_lap,
)

SCAN_POINTS_MAX = 500
SNAP_MAX_ERROR_M = 0.02  # a worse fit means the size is wrong: keep the old box
MIN_FIT_TO_START = 0.40  # position check below this: the robot does not know where it is
FIT_EVERY_S = 1.0        # how often the position check runs
RUNS_SHOWN = 12
RUNNING = ('undock', 'approach', 'rejoin', 'paused', 'turn', 'straight', 'blocked')
BUSY = {'mission': 'a mission is running', 'laps': 'laps are running', 'goal': 'Nav2 is driving to a goal', 'dock': 'the robot is docking or undocking'}


class FeedSettings(BaseModel):
    laps: int = Field(1, ge=1, le=50)
    feed: str = Field('long', pattern='^(long|all|none)$')
    gap: float = Field(0.10, ge=0.0, le=0.6, description='m, robot side to a fed face')
    end_gap: float = Field(0.15, ge=0.0, le=0.8, description='m, to the other faces')
    door_open: float = Field(0.2, ge=0.0, le=1.0, description='door opening, 0..1')
    door_margin: float = Field(0.05, ge=0.0, le=0.5)
    door_x: float = Field(0.0, ge=-0.3, le=0.3, description='m, door ahead of the centre')
    speed: float = Field(0.15, ge=0.03, le=0.3)
    dry_run: bool = False
    mix_below_kg: float = Field(0.5, ge=0.0, le=10.0, description='mixer on below this; 0: never')
    mix_speed: float = Field(0.5, ge=0.0, le=1.0)


class BoxBody(BaseModel):
    cx: float
    cy: float
    yaw: float
    length: float = Field(..., gt=0.05, le=5.0)
    width: float = Field(..., gt=0.05, le=5.0)


class PlanBody(FeedSettings):
    box: BoxBody | None = Field(None, description='plan for this box instead of the saved one')


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


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Feed:
    def __init__(self, node, tf, poses_file, weight):
        """weight: callable -> current kg or None."""
        self.node = node
        self.tf = tf
        self.poses_file = poses_file
        self.weight = weight
        self.lock = threading.Lock()
        self.map = None            # OccupancyGrid
        self.map_version = 0
        self.walls = None          # WallDistance for the current map, built on demand
        self.scan = None
        self.progress = None       # dict from /lap/progress
        self.run = None            # the run being logged
        self.fit_cache = (0.0, None)  # (monotonic time, result) of the position check
        self.plain_walls = None    # (map version, WallDistance) without leaving anything out
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        node.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        node.create_subscription(String, 'lap/progress', self._on_progress, latched)
        self.lap_params = node.create_client(SetParameters, '/lap_driver/set_parameters')

    # ---- inputs -------------------------------------------------------------

    def _on_map(self, msg):
        with self.lock:
            self.map = msg
            self.map_version += 1
            self.walls = None

    def _on_scan(self, msg):
        self.scan = msg

    def _on_progress(self, msg):
        try:
            p = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.progress = p
            run = self.run
            ended = run is not None and run.get('started') and p.get('state') not in RUNNING
            if run is not None and p.get('state') in RUNNING:
                run['started'] = True
            if ended:
                self.run = None
        if ended:
            self._log_run(run, p.get('state', '?'))

    # ---- map and scan -------------------------------------------------------

    def map_payload(self):
        with self.lock:
            m = self.map
            version = self.map_version
        if m is None:
            return None
        data = np.array(m.data, dtype=np.int16)
        data[data < 0] = 255  # unknown
        return {
            'version': version,
            'width': m.info.width, 'height': m.info.height,
            'resolution': m.info.resolution,
            'origin': [m.info.origin.position.x, m.info.origin.position.y],
            'data': base64.b64encode(data.astype(np.uint8).tobytes()).decode(),
        }

    def _wall_distance(self, box):
        """WallDistance for the current map, leaving out the box; cached per map and box."""
        key = (self.map_version, tuple(round(v, 3) for v in box.as_dict().values()))
        with self.lock:
            if self.map is not None and (self.walls is None or self.walls[0] != key):
                info = self.map.info
                grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
                self.walls = (key, WallDistance(grid, info.resolution, info.origin.position.x,
                                                info.origin.position.y, ignore=box))
            return self.walls[1] if self.walls else None

    def scan_points(self):
        """Latest scan in the map frame, Nx2, and its age in s; (None, None) without TF."""
        s = self.scan
        if s is None:
            return None, None
        try:
            t = self.tf.lookup_transform('map', s.header.frame_id, Time())
        except TransformException:
            return None, None
        r = np.asarray(s.ranges, dtype=float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + _yaw(t.transform.rotation)
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 8.0))
        r, a = r[ok], a[ok]
        pts = np.c_[r * np.cos(a) + t.transform.translation.x,
                    r * np.sin(a) + t.transform.translation.y]
        age = (self.node.get_clock().now() - Time.from_msg(s.header.stamp)).nanoseconds * 1e-9
        return pts, max(0.0, age)

    def position_fit(self):
        """How well the robot knows where it is: the share of lidar points that land
        within 5 cm of a wall on the map. Over ~70% is good; a wrong AMCL pose
        drops it to 20-40%. None without a map, scan or TF."""
        t, cached = self.fit_cache
        if time.monotonic() - t < FIT_EVERY_S:
            return cached
        result = None
        with self.lock:
            m, version = self.map, self.map_version
        pts, age = self.scan_points()
        if m is not None and pts is not None and len(pts) >= 30 and age is not None and age < 2.0:
            if self.plain_walls is None or self.plain_walls[0] != version:
                info = m.info
                grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
                self.plain_walls = (version, WallDistance(grid, info.resolution,
                                                          info.origin.position.x,
                                                          info.origin.position.y))
            walls = self.plain_walls[1]
            near = sum(1 for x, y in pts[::3] if walls(x, y) < 0.05)
            result = round(near / len(pts[::3]), 2)
        self.fit_cache = (time.monotonic(), result)
        return result

    def scan_payload(self):
        pts, age = self.scan_points()
        if pts is None:
            return {'points': [], 'age_s': None}
        step = max(1, len(pts) // SCAN_POINTS_MAX)
        return {'points': np.round(pts[::step], 3).tolist(), 'age_s': age}

    # ---- the cow box --------------------------------------------------------

    def box(self):
        try:
            return pose_store.load_box(self.poses_file)
        except Exception:
            return None

    def detect_box(self, near):
        """Fit the box from the saved map, nearest to `near` (x, y)."""
        with self.lock:
            m = self.map
        if m is None:
            return False, 'no map yet: is navigation running?'
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        box = fit_box_from_map(grid, m.info.resolution, m.info.origin.position.x,
                               m.info.origin.position.y, near)
        if box is None:
            return False, 'no free-standing obstacle found on the map'
        pose_store.save_box(box, path=self.poses_file)
        return True, (f'Found a {box.length:.2f} x {box.width:.2f} m box. The map draws walls '
                      'thick: set the real size, then Snap to lidar.')

    def snap_box(self):
        """Move and turn the saved box onto the lidar points of its faces."""
        d = self.box()
        if d is None:
            return False, 'no box yet: Detect from map first'
        pts, age = self.scan_points()
        if pts is None:
            return False, 'no lidar scan in the map frame (is navigation localized?)'
        box, used, err = fit_box_to_points(Box.from_dict(d), pts)
        if box is None:
            return False, f'only {used} lidar points near the box: drive closer and retry'
        if err > SNAP_MAX_ERROR_M:
            return False, (f'the outline does not fit the lidar ({err * 100:.1f} cm average over '
                           f'{used} points), not changed: check the length and width first')
        moved = math.hypot(box.cx - d['cx'], box.cy - d['cy'])
        turned = math.degrees(abs(math.remainder(box.yaw - d['yaw'], math.pi)))
        pose_store.save_box(box, path=self.poses_file)
        return True, (f'Snapped onto {used} lidar points ({err * 100:.1f} cm average): '
                      f'moved {moved * 100:.0f} cm, turned {turned:.1f} deg')

    def set_box(self, body):
        box = normalized(Box(body.cx, body.cy, body.yaw, body.length, body.width))
        pose_store.save_box(box, path=self.poses_file)
        return True, 'Box saved'

    # ---- plan and laps ------------------------------------------------------

    def plan(self, s, box=None):
        """The lap for settings s, around `box` (a BoxBody, e.g. while dragging) or the saved box."""
        if box is None:
            d = self.box()
            if d is None:
                return None
            box = Box.from_dict(d)
        else:
            box = Box(box.cx, box.cy, box.yaw, box.length, box.width)
        plan = plan_lap(box, gap=s.gap, end_gap=s.end_gap, feed=s.feed,
                        margin=s.door_margin, door_x=s.door_x)
        corners = plan['corners']
        sides = []
        for side in plan['sides']:
            (x0, y0), (x1, y1) = corners[side['start']], corners[side['end']]
            ux, uy = (x1 - x0) / side['length'], (y1 - y0) / side['length']
            door = None
            if side['door']:
                lo, hi = side['door']
                door = [[x0 + ux * lo, y0 + uy * lo], [x0 + ux * hi, y0 + uy * hi]]
            sides.append({'from': [x0, y0], 'to': [x1, y1], 'face': side['face'],
                          'feed': side['feed'], 'length': side['length'], 'door': door})
        problems = []
        walls = self._wall_distance(box)
        if walls is not None:
            for kind, i, dist in check_plan(plan, walls):
                where = f'side {i + 1}' if kind == 'side' else f'corner {i + 1}'
                problems.append({'kind': kind, 'index': i, 'clearance': dist,
                                 'text': f'{where}: the map shows a wall {dist * 100:.0f} cm '
                                         'from the robot centre'})
        return {
            'corners': corners, 'sides': sides, 'warnings': plan['warnings'],
            'problems': problems, 'lap_length': lap_length(plan),
            'door_length': door_length(plan),
        }

    def set_lap_params(self, s):
        if not self.lap_params.wait_for_service(timeout_sec=1.0):
            return False, 'lap_driver is not running: ros2 launch feed_mission mission.launch.py'
        req = SetParameters.Request()
        req.parameters = [
            _param('laps', int(s.laps)), _param('feed', s.feed),
            _param('gap', float(s.gap)), _param('end_gap', float(s.end_gap)),
            _param('door_open', float(s.door_open)), _param('door_margin', float(s.door_margin)),
            _param('door_x', float(s.door_x)), _param('speed', float(s.speed)),
            _param('dry_run', bool(s.dry_run)),
            _param('mix_below_kg', float(s.mix_below_kg)), _param('mix_speed', float(s.mix_speed)),
        ]
        done = threading.Event()
        future = self.lap_params.call_async(req)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3.0):
            return False, 'lap_driver did not answer'
        bad = [r.reason for r in future.result().results if not r.successful]
        return (False, '; '.join(bad)) if bad else (True, '')

    def started(self, s):
        """Laps accepted by lap_driver: remember the settings and start the log entry."""
        pose_store.save_feed(s.model_dump(), path=self.poses_file)
        plan = self.plan(s)
        with self.lock:
            self.run = {
                'settings': s.model_dump(), 'start_kg': self.weight(),
                'door_m_per_lap': plan['door_length'] if plan else None,
                't0': time.monotonic(), 'started': False,
            }

    def _log_run(self, run, result):
        end = self.weight()
        start = run['start_kg']
        s = run['settings']
        dispensed = (start - end) if start is not None and end is not None else None
        door_m = (run['door_m_per_lap'] or 0.0) * s['laps']
        entry = {
            'result': result, 'laps': s['laps'], 'dry_run': s['dry_run'], 'feed': s['feed'],
            'door_open': s['door_open'], 'speed': s['speed'], 'gap': s['gap'],
            'start_kg': None if start is None else round(start, 3),
            'end_kg': None if end is None else round(end, 3),
            'dispensed_kg': None if dispensed is None else round(dispensed, 3),
            'door_m': round(door_m, 2),
            'kg_per_m': round(dispensed / door_m, 3) if dispensed is not None and door_m > 0 else None,
            'duration_s': round(time.monotonic() - run['t0'], 1),
        }
        try:
            pose_store.add_run(entry, path=self.poses_file)
        except Exception as e:
            self.node.get_logger().warn(f'cannot write the feed run log: {e}')

    def state(self):
        with self.lock:
            progress = dict(self.progress) if self.progress else None
            version = self.map_version
            running = self.run is not None
            start_kg = self.run['start_kg'] if self.run else None
        try:
            runs = pose_store.load_runs(self.poses_file)[-RUNS_SHOWN:][::-1]
            settings = pose_store.load_feed(self.poses_file)
        except Exception:
            runs, settings = [], {}
        return {
            'box': self.box(), 'settings': settings, 'progress': progress,
            'runs': runs, 'map_version': version, 'logging': running, 'run_start_kg': start_kg,
            'fit': self.position_fit(),
        }


def add_routes(app, ros, feed, require_control, client_id, depends, HTTPException, Request):
    """The /api/feed, /api/box, /api/map and /api/scan routes."""

    @app.get('/api/map')
    def get_map():
        m = feed.map_payload()
        if m is None:
            raise HTTPException(404, 'no map yet: is navigation running?')
        return m

    @app.get('/api/scan')
    def get_scan():
        return feed.scan_payload()

    @app.post('/api/box/detect', dependencies=[depends(require_control)])
    def box_detect():
        p = ros.robot_pose()
        b = feed.box()
        near = (b['cx'], b['cy']) if b else ((p['x'], p['y']) if p else (0.0, 0.0))
        ok, msg = feed.detect_box(near)
        return {'ok': ok, 'message': msg}

    @app.post('/api/box/snap', dependencies=[depends(require_control)])
    def box_snap():
        ok, msg = feed.snap_box()
        return {'ok': ok, 'message': msg}

    @app.put('/api/box', dependencies=[depends(require_control)])
    def box_set(body: BoxBody):
        ok, msg = feed.set_box(body)
        return {'ok': ok, 'message': msg}

    @app.post('/api/feed/plan')
    def feed_plan(body: PlanBody):
        """Preview: needs no control."""
        plan = feed.plan(body, body.box)
        if plan is None:
            raise HTTPException(404, 'no cow box yet')
        return plan

    @app.post('/api/lap/start', dependencies=[depends(require_control)])
    def lap_start(body: FeedSettings):
        if feed.box() is None:
            return {'ok': False, 'message': 'Measure the cow box first'}
        act = ros.activity()
        if act in ('mission', 'dock'):
            return {'ok': False, 'message': f'Not now: {BUSY[act]}'}
        if act == 'goal':
            ros.nav.cancel_all()  # laps replace a goal sent from the map
        fit = feed.position_fit()
        if fit is not None and fit < MIN_FIT_TO_START:
            return {'ok': False, 'message': (
                f'Position unsure ({fit * 100:.0f}% of the lidar on the map): '
                'use Find robot or Position from dock first')}
        ok, msg = feed.set_lap_params(body)
        if not ok:
            return {'ok': False, 'message': msg}
        ok, msg = ros.call('lap/start')
        if ok:
            feed.started(body)
        return {'ok': ok, 'message': msg}

    @app.post('/api/lap/stop')
    def lap_stop(request: Request):
        client_id(request)  # stopping is always safe, so any device may do it
        ok, msg = ros.call('lap/stop')
        return {'ok': ok, 'message': msg}
