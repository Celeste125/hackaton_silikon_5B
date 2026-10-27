"""Web control panel for the robot.

  ros2 run web_panel web_panel        then open http://<robot-ip>:8000
  (robot_bringup's bringup.launch.py starts it with everything else)

It is only a front end for topics of other nodes, mostly esp_driver_node:
  /door/position, /door/raw_us, /door/save_open, /door/save_closed
  /mixer/motor_0, /mixer/motor_1, /mixer/shake (unjam pattern, run on the ESP)
  /loadcell/weight, /loadcell/cells, /loadcell/tare
  /Team2/cmd_vel (geometry_msgs/Twist, Create 3 base, parameter cmd_vel_topic)
  /valve1..3/percent (published), /valve1..3/angle (feedback). The valves are
  driven by silo_controller/valve_controller.py on the Pi, which must be running.
  /camera0, /camera1 image_raw/compressed (JPEG, parameter camera_topics), from
  cameras.launch.py. Streamed to the page as MJPEG at /api/camera/<n>.mjpg.
  /Team2/battery_state (battery), /Team2/hazard_detection (bump, cliff, stall;
  needs irobot_create_msgs), /frontier_explorer/status (mapping progress).
  Setup tab: the robot's pose (TF map -> base_link) is saved under a name
  (silo1..3, start) in the feed_mission poses file; /relocalize.
  Feed tab and map view (feed.py): /map, /scan, the cow box, the lap plan,
  /lap/start, /lap/stop, /lap/status, /lap/progress, /lap_driver parameters.

Needs: pip install fastapi uvicorn

Motors and base driving are a dead man's switch: the page keeps sending its
setpoint, and the node only republishes it while those messages keep arriving.
Close the tab or lose Wi-Fi and the motors and the base stop. There is no login, use it on a trusted network.

Many browsers can open the panel, but only one holds control at a time (see
control_lease.py). Everyone else is view-only and can take control over
explicitly. STOP ALL, close all valves and cancel dose work from any device.
Every request carries an X-Client-Id header that the page generates per tab.
"""

import asyncio
import math
import threading
import time
from pathlib import Path

import rclpy
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel, Field
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from web_panel.control_lease import ControlLease

try:  # taught poses, the map view and laps need feed_mission; the rest works without it
    from feed_mission import poses as pose_store
    from web_panel import feed as feed_mod
except ImportError:
    pose_store = feed_mod = None

try:  # Create 3 hazards are optional: without the messages they are not shown
    from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
except ImportError:
    HazardDetection = HazardDetectionVector = None

INDEX_HTML = Path(__file__).parent / 'web' / 'index.html'
CONTROL_TIMEOUT_S = 6.0  # the holder is silent this long -> control is free
PUBLISH_HZ = 10.0
HEARTBEAT_TIMEOUT_S = 0.8  # page sends every ~0.3 s; silence longer than this stops the motors
SERVICE_TIMEOUT_S = 3.0
SERVO_MIN_US = 500   # same limits as the firmware (mixer_esp32/src/config.h)
SERVO_MAX_US = 2500
MAX_LINEAR = 0.3    # m/s, Create 3 limit is 0.306 with the safety features on
MAX_ANGULAR = 1.5   # rad/s
DRIVE_TIMEOUT_S = 0.5
VALVES = (1, 2, 3)
DOSE_TIMEOUT_S = 120.0  # a dose that has not reached its weight by then is aborted
WEIGHT_MAX_AGE_S = 1.0  # no fresh load cell reading -> abort the dose
CAMERA_TOPICS = ['/camera0/image_raw/compressed', '/camera1/image_raw/compressed']
CAMERA_STREAM_FPS = 10.0  # per browser; caps Wi-Fi use, the cameras run at 15
CAMERA_STALE_S = 2.0
HAZARD_HOLD_S = 0.6  # a hazard stays on the page this long after its last report
POSE_MAX_AGE_S = 2.0  # older map -> base_link is not saved as a pose


class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_panel')
        # Must match NUM_MOTORS in the firmware / mixer_esp_driver config.py.
        self.declare_parameter('num_motors', 2)
        self.num_motors = int(self.get_parameter('num_motors').value)
        self.door_pub = self.create_publisher(Float32, 'door/position', 10)
        self.raw_us_pub = self.create_publisher(Float32, 'door/raw_us', 10)
        self.motor_pubs = [
            self.create_publisher(Float32, f'mixer/motor_{i}', 10)
            for i in range(self.num_motors)
        ]
        self.declare_parameter('cmd_vel_topic', '/Team2/cmd_vel')
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.drive = (0.0, 0.0)
        self.last_drive_t = 0.0
        self._drive_active = False
        self.valve_pubs = {
            v: self.create_publisher(Float32, f'valve{v}/percent', 10) for v in VALVES
        }
        for v in VALVES:
            self.create_subscription(
                Float32, f'valve{v}/angle', lambda m, v=v: self._on_valve_angle(v, m), 10)
        self.valve_percent = {v: None for v in VALVES}
        self.valve_angle = {v: None for v in VALVES}
        self.dose = None  # {'valve', 'target', 'start', 'percent', 'deadline', 'status'}
        self.create_subscription(Float32, 'loadcell/weight', self._on_weight, 10)
        self.create_subscription(Float32MultiArray, 'loadcell/cells', self._on_cells, 10)
        # Latest JPEG per camera: (frame number, bytes, monotonic time). Written
        # by the ROS thread, read by the web streams; a tuple swap is atomic.
        self.declare_parameter('camera_topics', CAMERA_TOPICS)
        self.camera_topics = list(self.get_parameter('camera_topics').value)
        self.camera_frames = [None] * len(self.camera_topics)
        for i, topic in enumerate(self.camera_topics):
            self.create_subscription(
                CompressedImage, topic, lambda m, i=i: self._on_camera(i, m),
                qos_profile_sensor_data)
        self.battery = None  # {'percent', 'charging', 'voltage', 't'}
        self.declare_parameter('battery_topic', '/Team2/battery_state')
        self.create_subscription(
            BatteryState, self.get_parameter('battery_topic').value, self._on_battery,
            qos_profile_sensor_data)
        self.hazards = {}  # name -> last monotonic time it was reported
        if HazardDetectionVector is not None:
            self.declare_parameter('hazard_topic', '/Team2/hazard_detection')
            self.create_subscription(
                HazardDetectionVector, self.get_parameter('hazard_topic').value,
                self._on_hazards, qos_profile_sensor_data)
        self.explorer_status = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/frontier_explorer/status', self._on_explorer_status, latched)
        self.trigger_clients = {
            name: self.create_client(Trigger, name)
            for name in ('loadcell/tare', 'door/save_open', 'door/save_closed',
                         'lap/start', 'lap/stop', 'relocalize')
        }
        # Robot pose on the map, for the Setup tab.
        self.declare_parameter('poses_file', '~/maps/arena_poses.yaml')
        self.poses_file = self.get_parameter('poses_file').value
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.lap_status = None
        self.create_subscription(String, 'lap/status', self._on_lap_status, latched)

        self.lease = ControlLease(CONTROL_TIMEOUT_S)
        self.lock = threading.Lock()
        self.weight = None
        self.cells = []
        self.last_weight_t = 0.0
        self.door = None  # last commanded position or ('raw', us)
        self.motors = [0.0] * self.num_motors
        self.last_motor_t = 0.0
        self._motors_active = False
        self.shake_pub = self.create_publisher(Float32MultiArray, 'mixer/shake', 10)
        self.shake = None  # {'fwd_ms', 'rev_ms', 'speeds'} while shaking
        self.last_shake_t = 0.0
        self._shake_active = False
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_motors)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_shake)
        self.create_timer(0.1, self._check_dose)
        self.create_timer(0.1, self._publish_drive)
        self.feed = feed_mod.Feed(self, self.tf, self.poses_file, self.current_weight) \
            if feed_mod else None

    def current_weight(self):
        """kg, or None without a fresh reading."""
        with self.lock:
            fresh = time.monotonic() - self.last_weight_t < WEIGHT_MAX_AGE_S
            return self.weight if fresh else None

    def _on_weight(self, msg):
        with self.lock:
            self.weight = float(msg.data)
            self.last_weight_t = time.monotonic()

    def _on_battery(self, msg):
        pct = msg.percentage
        if pct != pct:  # NaN: unknown
            pct = None
        elif pct <= 1.0:  # sensor_msgs says 0..1; some drivers send 0..100
            pct *= 100.0
        charging = msg.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING, BatteryState.POWER_SUPPLY_STATUS_FULL)
        with self.lock:
            self.battery = {'percent': pct, 'charging': charging,
                            'voltage': float(msg.voltage), 't': time.monotonic()}

    def _on_hazards(self, msg):
        names = {
            HazardDetection.BUMP: 'bump',
            HazardDetection.CLIFF: 'cliff',
            HazardDetection.STALL: 'stall',
            HazardDetection.WHEEL_DROP: 'wheel drop',
            HazardDetection.BACKUP_LIMIT: 'backup limit',
        }  # OBJECT_PROXIMITY (IR) fires all the time near walls, not shown
        now = time.monotonic()
        with self.lock:
            for d in msg.detections:
                if d.type in names:
                    self.hazards[names[d.type]] = now

    def _on_lap_status(self, msg):
        with self.lock:
            self.lap_status = msg.data

    def robot_pose(self):
        """{'x', 'y', 'yaw', 'age_s'} from TF map -> base_link, or None."""
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        q = t.transform.rotation
        stamp = Time.from_msg(t.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return {
            'x': t.transform.translation.x, 'y': t.transform.translation.y,
            'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
            'age_s': max(0.0, age),
        }

    def saved_poses(self):
        if pose_store is None:
            return None
        try:
            return pose_store.load(self.poses_file)
        except Exception as e:  # a broken file must not take the panel down
            self.get_logger().warn(f'cannot read {self.poses_file}: {e}', throttle_duration_sec=30.0)
            return {}

    def _on_explorer_status(self, msg):
        with self.lock:
            self.explorer_status = msg.data

    def _on_camera(self, i, msg):
        if 'jpeg' not in msg.format and 'jpg' not in msg.format:
            self.get_logger().warn(
                f'{self.camera_topics[i]}: format "{msg.format}" is not JPEG, not streamed',
                throttle_duration_sec=30.0)
            return
        prev = self.camera_frames[i]
        self.camera_frames[i] = ((prev[0] + 1) if prev else 1, bytes(msg.data), time.monotonic())

    def _on_cells(self, msg):
        with self.lock:
            self.cells = [float(v) for v in msg.data]

    def _on_valve_angle(self, valve, msg):
        with self.lock:
            self.valve_angle[valve] = float(msg.data)

    def _send_valve(self, valve, percent):
        percent = max(0.0, min(100.0, float(percent)))
        self.valve_pubs[valve].publish(Float32(data=percent))
        self.valve_percent[valve] = percent

    def set_valve(self, valve, percent):
        with self.lock:
            # a manual command on the valve being dosed cancels that dose
            if self.dose and self.dose['status'] == 'running' and self.dose['valve'] == valve:
                self.dose['status'] = 'cancelled'
            self._send_valve(valve, percent)

    def close_all_valves(self):
        with self.lock:
            if self.dose and self.dose['status'] == 'running':
                self.dose['status'] = 'cancelled'
            for v in VALVES:
                self._send_valve(v, 0.0)

    def start_dose(self, valve, target_kg, percent):
        with self.lock:
            if self.dose and self.dose['status'] == 'running':
                return False, f"valve {self.dose['valve']} is already dosing"
            fresh = time.monotonic() - self.last_weight_t < WEIGHT_MAX_AGE_S
            if self.weight is None or not fresh:
                return False, 'no live load cell reading'
            self.dose = {
                'valve': valve, 'target': float(target_kg), 'start': self.weight,
                'percent': float(percent), 'status': 'running',
                'deadline': time.monotonic() + DOSE_TIMEOUT_S,
            }
            self._send_valve(valve, percent)
        return True, 'dosing'

    def cancel_dose(self):
        with self.lock:
            if self.dose and self.dose['status'] == 'running':
                self.dose['status'] = 'cancelled'
                self._send_valve(self.dose['valve'], 0.0)

    def _check_dose(self):
        with self.lock:
            d = self.dose
            if not d or d['status'] != 'running':
                return
            now = time.monotonic()
            if now - self.last_weight_t > WEIGHT_MAX_AGE_S:
                d['status'] = 'aborted: lost load cell data'
            elif self.weight - d['start'] >= d['target']:
                d['status'] = 'done'
            elif now > d['deadline']:
                d['status'] = 'aborted: timeout'
            else:
                return
            self._send_valve(d['valve'], 0.0)

    def _publish_motors(self):
        with self.lock:
            fresh = time.monotonic() - self.last_motor_t < HEARTBEAT_TIMEOUT_S
            speeds = list(self.motors) if fresh else [0.0] * len(self.motors)
            if not fresh:
                self.motors = list(speeds)
            publish = fresh or self._motors_active
            self._motors_active = fresh and any(speeds)
        if publish:
            for pub, s in zip(self.motor_pubs, speeds):
                pub.publish(Float32(data=s))

    def _publish_shake(self):
        # Same dead man's switch as the motors. When the pattern ends, one
        # empty message tells the driver to stop it right away.
        with self.lock:
            fresh = (self.shake is not None
                     and time.monotonic() - self.last_shake_t < HEARTBEAT_TIMEOUT_S)
            if not fresh:
                self.shake = None
            shake = self.shake
            publish = fresh or self._shake_active
            self._shake_active = fresh
        if publish:
            data = [shake['fwd_ms'], shake['rev_ms'], *shake['speeds']] if shake else []
            self.shake_pub.publish(Float32MultiArray(data=[float(v) for v in data]))

    def set_shake(self, fwd_ms, rev_ms, speeds):
        with self.lock:
            self.shake = {'fwd_ms': fwd_ms, 'rev_ms': rev_ms, 'speeds': list(speeds)}
            self.last_shake_t = time.monotonic()
            self.motors = [0.0] * len(self.motors)

    def _publish_drive(self):
        with self.lock:
            fresh = time.monotonic() - self.last_drive_t < DRIVE_TIMEOUT_S
            lin, ang = self.drive if fresh else (0.0, 0.0)
            publish = fresh or self._drive_active
            if not fresh:
                self.drive = (0.0, 0.0)
            self._drive_active = fresh and (lin != 0.0 or ang != 0.0)
        if publish:
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            self.cmd_vel_pub.publish(msg)

    def set_drive(self, linear, angular):
        with self.lock:
            self.drive = (max(-MAX_LINEAR, min(MAX_LINEAR, linear)),
                          max(-MAX_ANGULAR, min(MAX_ANGULAR, angular)))
            self.last_drive_t = time.monotonic()
            self._drive_active = self.drive != (0.0, 0.0)

    def set_motors(self, speeds):
        with self.lock:
            self.shake = None  # plain speeds end the shake pattern
            self.motors = list(speeds)
            self.last_motor_t = time.monotonic()
            self._motors_active = any(speeds)

    def stop_outputs(self):
        """Zero the motors and the base, e.g. when control changes hands."""
        self.set_motors([0.0] * self.num_motors)
        self.set_drive(0.0, 0.0)

    def emergency_stop(self):
        self.stop_outputs()
        self.close_all_valves()
        lap_stop = self.trigger_clients['lap/stop']
        if lap_stop.service_is_ready():  # fire and forget: STOP must never wait
            lap_stop.call_async(Trigger.Request())

    def set_door(self, position):
        self.door_pub.publish(Float32(data=float(position)))
        with self.lock:
            self.door = {'mode': 'position', 'value': float(position)}

    def set_door_raw(self, us):
        self.raw_us_pub.publish(Float32(data=float(us)))
        with self.lock:
            self.door = {'mode': 'raw_us', 'value': float(us)}

    def call(self, name, timeout=SERVICE_TIMEOUT_S):
        client = self.trigger_clients[name]
        if not client.wait_for_service(timeout_sec=1.0):
            owner = ('feed_mission (ros2 launch feed_mission mission.launch.py)'
                     if name.startswith('lap/') or name == 'relocalize' else 'esp_driver_node')
            return False, f'service /{name} not available, is {owner} running?'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return False, f'/{name} timed out'
        res = future.result()
        return res.success, res.message

    def state(self):
        with self.lock:
            now = time.monotonic()
            dose = None
            if self.dose:
                d = self.dose
                got = (self.weight - d['start']) if self.weight is not None else 0.0
                dose = {'valve': d['valve'], 'target': d['target'], 'status': d['status'],
                        'added': got}
            state = {
                'valves': {str(v): {'percent': self.valve_percent[v],
                                    'angle': self.valve_angle[v]} for v in VALVES},
                'dose': dose,
                'weight': self.weight,
                'weight_age_s': now - self.last_weight_t if self.weight is not None else None,
                'cells': list(self.cells),
                'door': self.door,
                'cameras': [
                    {'topic': t, 'age_s': (now - f[2]) if f else None}
                    for t, f in zip(self.camera_topics, self.camera_frames)
                ],
                'battery': None if self.battery is None else {
                    'percent': self.battery['percent'],
                    'charging': self.battery['charging'],
                    'voltage': self.battery['voltage'],
                    'age_s': now - self.battery['t'],
                },
                'hazards': sorted(n for n, t in self.hazards.items() if now - t < HAZARD_HOLD_S),
                'explorer': self.explorer_status,
                'lap': self.lap_status,
                'motors': list(self.motors),
                'shake': dict(self.shake) if self.shake else None,
                'num_motors': len(self.motors),
            }
        # Outside the lock: TF and the poses file have their own.
        state['pose'] = self.robot_pose()
        state['poses'] = self.saved_poses()
        state['feed'] = self.feed.state() if self.feed else None
        return state


class MotorsBody(BaseModel):
    speeds: list[float] = Field(..., description='-1.0 .. 1.0 per motor')


class ShakeBody(BaseModel):
    fwd_ms: float = Field(..., ge=0.0, le=10000.0, description='time at +speed')
    rev_ms: float = Field(..., ge=0.0, le=10000.0, description='time at -speed')
    speeds: list[float] = Field(..., description='-1.0 .. 1.0 per motor, 0 = motor off')


class DriveBody(BaseModel):
    linear: float = Field(0.0, description='m/s, forward positive')
    angular: float = Field(0.0, description='rad/s, left positive')


class DoorBody(BaseModel):
    position: float = Field(..., ge=0.0, le=1.0, description='0 = open, 1 = closed')


class RawUsBody(BaseModel):
    us: float = Field(..., ge=SERVO_MIN_US, le=SERVO_MAX_US)


class ValveBody(BaseModel):
    percent: float = Field(..., ge=0.0, le=100.0, description='0 = closed, 100 = open')


class DoseBody(BaseModel):
    target_kg: float = Field(..., gt=0.0, le=50.0, description='weight to add')
    percent: float = Field(100.0, gt=0.0, le=100.0)


class ClaimBody(BaseModel):
    force: bool = Field(False, description='take control even if another device holds it')


def create_app(ros):
    app = FastAPI(title='Mixer control')

    def client_id(request: Request):
        cid = request.headers.get('x-client-id', '').strip()
        if not cid or len(cid) > 64:
            raise HTTPException(400, 'missing X-Client-Id header')
        return cid

    def client_label(request: Request, cid):
        host = request.client.host if request.client else '?'
        return f'{host} ({cid[:4]})'

    def require_control(request: Request):
        """Dependency of every command: only the device holding control may send it."""
        if not ros.lease.touch(client_id(request)):
            raise HTTPException(403, {
                'error': 'not_in_control',
                'holder': ros.lease.snapshot()['holder'],
            })

    @app.get('/')
    def index():
        return FileResponse(INDEX_HTML)

    @app.get('/api/state')
    def state(request: Request):
        cid = request.headers.get('x-client-id', '').strip()
        if cid:
            ros.lease.touch(cid)  # the holder's poll keeps its control alive while idle
        return {**ros.state(), 'control': ros.lease.snapshot(cid or None)}

    def camera_or_404(n):
        if not 0 <= n < len(ros.camera_frames):
            raise HTTPException(404, 'no such camera')

    @app.get('/api/camera/{n}.jpg')
    def camera_snapshot(n: int):
        camera_or_404(n)
        frame = ros.camera_frames[n]
        if frame is None:
            raise HTTPException(503, f'no image yet on {ros.camera_topics[n]}')
        return Response(frame[1], media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.get('/api/camera/{n}.mjpg')
    async def camera_stream(n: int, request: Request):
        """multipart MJPEG: an <img> shows it as live video. Watching needs no control."""
        camera_or_404(n)

        async def frames():
            sent = None
            while not await request.is_disconnected():
                frame = ros.camera_frames[n]
                if frame is not None and frame[0] != sent:
                    sent = frame[0]
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                           + str(len(frame[1])).encode() + b'\r\n\r\n' + frame[1] + b'\r\n')
                await asyncio.sleep(1.0 / CAMERA_STREAM_FPS)

        return StreamingResponse(
            frames(), media_type='multipart/x-mixed-replace; boundary=frame',
            headers={'Cache-Control': 'no-store'})

    @app.post('/api/control/claim')
    def control_claim(body: ClaimBody, request: Request):
        cid = client_id(request)
        ok, changed = ros.lease.claim(cid, client_label(request, cid), force=body.force)
        if ok and changed:
            ros.stop_outputs()  # the new holder starts from standstill
        return {'ok': ok, 'control': ros.lease.snapshot(cid)}

    @app.post('/api/control/release')
    def control_release(request: Request):
        if ros.lease.release(client_id(request)):
            ros.stop_outputs()
        return {'ok': True}

    @app.post('/api/stop')
    def stop(request: Request):
        """Safe state for everyone: motors, base and valves. Needs no control."""
        client_id(request)
        ros.emergency_stop()
        return {'ok': True}

    @app.post('/api/motors', dependencies=[Depends(require_control)])
    def motors(body: MotorsBody):
        n = ros.state()['num_motors']
        if len(body.speeds) != n:
            raise HTTPException(422, f'expected {n} speeds')
        ros.set_motors([max(-1.0, min(1.0, s)) for s in body.speeds])
        return {'ok': True}

    @app.post('/api/shake', dependencies=[Depends(require_control)])
    def shake(body: ShakeBody):
        """Resend every ~0.3 s while shaking, like /api/motors."""
        n = ros.state()['num_motors']
        if len(body.speeds) != n:
            raise HTTPException(422, f'expected {n} speeds')
        if body.fwd_ms + body.rev_ms < 20:
            raise HTTPException(422, 'fwd_ms + rev_ms must be >= 20')
        ros.set_shake(body.fwd_ms, body.rev_ms, [max(-1.0, min(1.0, s)) for s in body.speeds])
        return {'ok': True}

    @app.post('/api/drive', dependencies=[Depends(require_control)])
    def drive(body: DriveBody):
        ros.set_drive(body.linear, body.angular)
        return {'ok': True}

    @app.post('/api/door', dependencies=[Depends(require_control)])
    def door(body: DoorBody):
        ros.set_door(body.position)
        return {'ok': True}

    @app.post('/api/door/raw_us', dependencies=[Depends(require_control)])
    def door_raw(body: RawUsBody):
        ros.set_door_raw(body.us)
        return {'ok': True}

    @app.post('/api/door/save_open', dependencies=[Depends(require_control)])
    def save_open():
        ok, msg = ros.call('door/save_open')
        return {'ok': ok, 'message': msg}

    @app.post('/api/door/save_closed', dependencies=[Depends(require_control)])
    def save_closed():
        ok, msg = ros.call('door/save_closed')
        return {'ok': ok, 'message': msg}

    @app.post('/api/valves/close_all')
    def valves_close_all(request: Request):
        client_id(request)  # closing is always safe, so any device may do it
        ros.close_all_valves()
        return {'ok': True}

    @app.post('/api/valves/{valve}', dependencies=[Depends(require_control)])
    def valve(valve: int, body: ValveBody):
        if valve not in VALVES:
            raise HTTPException(404, 'no such valve')
        ros.set_valve(valve, body.percent)
        return {'ok': True}

    @app.post('/api/valves/{valve}/dose', dependencies=[Depends(require_control)])
    def valve_dose(valve: int, body: DoseBody):
        if valve not in VALVES:
            raise HTTPException(404, 'no such valve')
        ok, msg = ros.start_dose(valve, body.target_kg, body.percent)
        return {'ok': ok, 'message': msg}

    @app.post('/api/dose/cancel')
    def dose_cancel(request: Request):
        client_id(request)  # cancelling is always safe, so any device may do it
        ros.cancel_dose()
        return {'ok': True}

    @app.post('/api/poses/{name}', dependencies=[Depends(require_control)])
    def save_pose(name: str):
        """Save where the robot is now, on the map, under this name."""
        if pose_store is None:
            raise HTTPException(503, 'feed_mission is not installed')
        if name not in pose_store.NAMES:
            raise HTTPException(404, f'unknown pose {name}')
        p = ros.robot_pose()
        if p is None or p['age_s'] > POSE_MAX_AGE_S:
            return {'ok': False, 'message': 'Robot not localized on the map: start navigation (or SLAM) first'}
        saved = pose_store.save_pose(name, p['x'], p['y'], p['yaw'], path=ros.poses_file)
        return {'ok': True, 'message': f"Saved {name} at {saved['x']:.2f}, {saved['y']:.2f}"}

    @app.post('/api/relocalize', dependencies=[Depends(require_control)])
    def relocalize():
        """Find the robot on the map from the lidar and correct AMCL (takes ~10 s)."""
        ok, msg = ros.call('relocalize', timeout=40.0)
        return {'ok': ok, 'message': msg}

    @app.delete('/api/poses/{name}', dependencies=[Depends(require_control)])
    def clear_pose(name: str):
        if pose_store is None:
            raise HTTPException(503, 'feed_mission is not installed')
        pose_store.clear_pose(name, path=ros.poses_file)
        return {'ok': True, 'message': f'Cleared {name}'}

    if ros.feed is not None:
        feed_mod.add_routes(app, ros, ros.feed, require_control, client_id, Depends,
                            HTTPException, Request)

    @app.post('/api/tare', dependencies=[Depends(require_control)])
    def tare():
        ok, msg = ros.call('loadcell/tare')
        return {'ok': ok, 'message': msg}

    return app


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8000)
    host = node.get_parameter('host').value
    port = int(node.get_parameter('port').value)

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    node.get_logger().info(f'Web control on http://{host}:{port}')
    try:
        uvicorn.run(create_app(node), host=host, port=port, log_level='warning')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.set_motors([0.0] * node.num_motors)
        node._publish_motors()
        node._publish_shake()
        node.set_drive(0.0, 0.0)
        node._publish_drive()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
