"""Closed-loop test of the real lap_driver against a simulated robot.

    ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 python3 test/sim_laps.py

Needs ROS 2 (rclpy, tf2_ros, nav2_msgs) but no robot; the domain keeps it off
the real one. About 100 s. Exit code 0: both laps done, the footprint never
touched the box, after every disturbance below.

A unicycle robot, TF (map -> odom -> base_link, base_link -> laser turned 180
degrees like the real lidar), a raycast lidar seeing the cow box (and an
optional obstacle), twist_mux-like priority for teleop, and a fake Nav2 that
drives straight to its goal. Disturbances: a burst of manual driving that
pushes the robot off the line, and an obstacle placed in front of it.
"""
import json, math, os, sys, tempfile, threading, time
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

PKG = str(__import__('pathlib').Path(__file__).resolve().parents[1])
sys.path.insert(0, PKG)
from feed_mission import poses as pose_store
from feed_mission.lap_geometry import Box, plan_lap

BOX = Box(1.0, 0.5, math.radians(80), 1.0, 0.5)
SETTINGS = dict(laps=2, feed='long', gap=0.10, end_gap=0.15, door_open=0.25, door_margin=0.05,
                door_x=0.0, speed=0.2)
DT = 0.02


def q(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class Sim(Node):
    def __init__(self, start_pose):
        super().__init__('sim')
        self.x, self.y, self.yaw = start_pose
        self.lap_cmd = (0.0, 0.0, -1.0)
        self.tel_cmd = (0.0, 0.0, -1.0)
        self.nav_target = None
        self.obstacle = None        # (x, y) in map, a 10 cm post
        self.map_offset = 0.0       # metres: AMCL believing the robot elsewhere
        self.door = []
        self.path = []
        self.progress = []
        self.tfb = TransformBroadcaster(self)
        st = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.rotation.z, t.transform.rotation.w = q(math.pi)
        t.header.stamp = self.get_clock().now().to_msg()
        st.sendTransform(t)
        self.create_subscription(Twist, '/sim/cmd_vel_lap', lambda m: setattr(self, 'lap_cmd', (m.linear.x, m.angular.z, time.monotonic())), 10)
        self.create_subscription(Twist, '/sim/teleop', lambda m: setattr(self, 'tel_cmd', (m.linear.x, m.angular.z, time.monotonic())), 10)
        self.create_subscription(Float32, 'door/position', lambda m: self.door.append((time.monotonic(), m.data)), 10)
        self.create_subscription(String, 'lap/progress', lambda m: self.progress.append((time.monotonic(), json.loads(m.data))), 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tel_pub = self.create_publisher(Twist, '/sim/teleop', 10)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self._nav)
        self.create_timer(1 / 30, self._tick)
        # the map: the box, drawn as wall cells, on a 6 x 6 m grid around it
        res, n = 0.05, 120
        g = OccupancyGrid(); g.header.frame_id = 'map'; g.info.resolution = res
        g.info.width = g.info.height = n
        g.info.origin.position.x, g.info.origin.position.y = BOX.cx - 3.0, BOX.cy - 3.0
        cells = np.zeros((n, n), dtype=np.int8)
        for r in range(n):
            for c in range(n):
                u, v = BOX.to_box(BOX.cx - 3.0 + (c + .5) * res, BOX.cy - 3.0 + (r + .5) * res)
                du, dv = abs(u) - BOX.length / 2, abs(v) - BOX.width / 2
                if max(du, dv) <= 0 and max(du, dv) > -0.06:
                    cells[r, c] = 100
        g.data = cells.ravel().tolist()
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_pub.publish(g)
        self.create_timer(0.1, self._scan)

    def _nav(self, handle):
        p = handle.request.pose.pose
        self.nav_target = (p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))
        while self.nav_target is not None:
            if handle.is_cancel_requested:
                self.nav_target = None
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)
        handle.succeed()
        return NavigateToPose.Result()

    def _tick(self):
        now = time.monotonic()
        dt = min(0.1, now - getattr(self, 'last_tick', now - DT))
        self.last_tick = now
        if self.nav_target is not None:  # fake Nav2: straight there, then turn
            tx, ty, tyaw = self.nav_target
            d = math.hypot(tx - self.x, ty - self.y)
            if d > 0.02:
                self.yaw = math.atan2(ty - self.y, tx - self.x)
                step = min(d, 0.2 * dt)
                self.x += step * math.cos(self.yaw); self.y += step * math.sin(self.yaw)
            else:
                err = math.atan2(math.sin(tyaw - self.yaw), math.cos(tyaw - self.yaw))
                if abs(err) < 0.02:
                    self.nav_target = None
                else:
                    self.yaw += max(-0.6 * dt, min(0.6 * dt, err))
        else:  # twist_mux: teleop wins for 0.5 s after its last message
            v, w, t = self.tel_cmd if now - self.tel_cmd[2] < 0.5 else self.lap_cmd
            if now - t > 0.5:
                v, w = 0.0, 0.0
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += w * dt
        self.path.append((now, self.x, self.y, self.yaw))
        stamp = self.get_clock().now().to_msg()
        a = TransformStamped(); a.header.stamp = stamp; a.header.frame_id = 'map'; a.child_frame_id = 'odom'
        a.transform.rotation.w = 1.0
        a.transform.translation.x = self.map_offset
        b = TransformStamped(); b.header.stamp = stamp; b.header.frame_id = 'odom'; b.child_frame_id = 'base_link'
        b.transform.translation.x, b.transform.translation.y = self.x, self.y
        b.transform.rotation.z, b.transform.rotation.w = q(self.yaw)
        self.tfb.sendTransform([a, b])

    def _scan(self):
        # raycast from the lidar against the box and the obstacle
        n = 720
        angles = -math.pi + np.arange(n) * (2 * math.pi / n)
        ranges = np.full(n, np.inf)
        world = self.yaw + math.pi + angles  # laser frame is turned 180 degrees
        segs = []
        cs = BOX.corners()
        for i in range(4):
            segs.append((cs[i], cs[(i + 1) % 4]))
        if self.obstacle:
            ox, oy = self.obstacle
            r = 0.05
            pts = [(ox - r, oy - r), (ox + r, oy - r), (ox + r, oy + r), (ox - r, oy + r)]
            segs += [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
        dx, dy = np.cos(world), np.sin(world)
        for (x1, y1), (x2, y2) in segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                t = ((x1 - self.x) * ey - (y1 - self.y) * ex) / den
                u = ((x1 - self.x) * dy - (y1 - self.y) * dx) / den
            hit = (np.abs(den) > 1e-9) & (t > 0.0) & (u >= 0) & (u <= 1)
            ranges = np.where(hit & (t < ranges), t, ranges)
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = 'laser'
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / n
        m.angle_max = m.angle_min + (n - 1) * m.angle_increment
        m.range_min, m.range_max = 0.16, 12.0
        m.ranges = [float(r) if r < 12.0 else float('inf') for r in ranges]
        self.scan_pub.publish(m)


def footprint_box_clearance(x, y, yaw):
    worst = 9.0
    for lx in np.linspace(-0.17, 0.17, 5):
        for ly in (-0.26, 0.26):
            px, py = x + lx * math.cos(yaw) - ly * math.sin(yaw), y + lx * math.sin(yaw) + ly * math.cos(yaw)
            u, v = BOX.to_box(px, py)
            du, dv = abs(u) - BOX.length / 2, abs(v) - BOX.width / 2
            worst = min(worst, math.hypot(max(du, 0), max(dv, 0)) if (du > 0 or dv > 0) else -min(-du, -dv))
    return worst


def main():
    tmp = tempfile.mkdtemp()
    pf = os.path.join(tmp, 'poses.yaml')
    pose_store.save_box(BOX, path=pf)
    plan = plan_lap(BOX, gap=SETTINGS['gap'], end_gap=SETTINGS['end_gap'], feed='long')
    c0, c1 = plan['corners'][0], plan['corners'][1]
    rclpy.init()
    sim = Sim((c0[0], c0[1], math.atan2(c1[1] - c0[1], c1[0] - c0[0])))
    import subprocess
    args = ['-p', f'poses_file:={pf}', '-p', 'cmd_vel_topic:=/sim/cmd_vel_lap', '-p', 'teleop_topic:=/sim/teleop']
    for k, v in SETTINGS.items():
        args += ['-p', f'{k}:={v}']
    proc = subprocess.Popen([sys.executable, '-c',
        f'import sys; sys.path.insert(0, "{PKG}"); '
        'from feed_mission.lap_driver import main; main()', '--ros-args', *args],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    threading.Thread(target=lambda: [print('   driver:', l.rstrip()) for l in proc.stdout if 'lap: ' in l and ('paused' in l or 'rejoin' in l or 'blocked' in l or 'failed' in l or 'done' in l or 'lost' in l)], daemon=True).start()
    class L:  # the driver's state, from its progress topic
        state = 'idle'
    lap = L()
    sim.create_subscription(String, 'lap/progress', lambda m: setattr(lap, 'state', json.loads(m.data)['state']), 10)
    ex = MultiThreadedExecutor(); ex.add_node(sim)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(4.0)
    cli = sim.create_client(Trigger, 'lap/start'); cli.wait_for_service(timeout_sec=5)
    f = cli.call_async(Trigger.Request())
    while not f.done(): time.sleep(0.05)
    print('start:', f.result().message)
    t0 = time.monotonic()
    pushed = blocked = False
    while time.monotonic() - t0 < 150:
        t = time.monotonic() - t0
        if not pushed and t > 7:  # someone drives it 1.5 s sideways and turned
            pushed = True
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                m = Twist(); m.linear.x = 0.2; m.angular.z = 1.0; sim.tel_pub.publish(m); time.sleep(0.05)
            print(f'{time.monotonic()-t0:5.1f}s pushed by teleop to ({sim.x:.2f}, {sim.y:.2f})')
        if not blocked and t > 24 and lap.state == 'straight':  # a post 25 cm in front for 4 s
            blocked = True
            sim.obstacle = (sim.x + 0.42 * math.cos(sim.yaw), sim.y + 0.42 * math.sin(sim.yaw))
            print(f'{t:5.1f}s obstacle placed ahead')
            threading.Timer(4.0, lambda: setattr(sim, 'obstacle', None)).start()
        if not getattr(sim, 'lost_done', False) and t > 45 and lap.state in ('turn', 'straight'):
            sim.lost_done = True
            sim.map_offset = 0.6
            print(f'{t:5.1f}s AMCL jumps 0.6 m off for 8 s')
            threading.Timer(8.0, lambda: setattr(sim, 'map_offset', 0.0)).start()
        if lap.state in ('done', 'idle') or lap.state.startswith('failed'):
            break
        if False:
            line = lap.line
            print(f'  t={t:5.1f} state={lap.state:8s} seg={lap.segment} pose=({sim.x:.2f},{sim.y:.2f},{math.degrees(sim.yaw):.0f}) '
                  f'lapcmd=({sim.lap_cmd[0]:.2f},{sim.lap_cmd[1]:.2f}, age {time.monotonic()-sim.lap_cmd[2]:.1f}s) '
                  f'blocked={lap.blocked_since is not None} paused={lap.paused} line={None if line is None else line[2]}')
        time.sleep(0.1)
    states = []
    for _, p in sim.progress:
        key = p['state']
        if not states or states[-1] != key:
            states.append(key)
    print('final:', lap.state, '| states:', ' > '.join(states))
    path = np.array(sim.path)
    clear = min(footprint_box_clearance(x, y, yw) for _, x, y, yw in path[::5])
    print(f'closest the footprint got to the box: {clear * 100:.1f} cm; door commands: {len(sim.door)}; '
          f'run took {time.monotonic() - t0:.0f} s')
    # did the robot ever spin in place for long (the old bug)?
    dts = np.maximum(np.diff(path[:, 0]), 1e-3)
    w = np.diff(np.unwrap(path[:, 3])) / dts
    v = np.hypot(np.diff(path[:, 1]), np.diff(path[:, 2])) / dts
    spin = float(np.sum(dts[(np.abs(w) > 0.3) & (v < 0.01)]))
    print(f'time spent turning on the spot: {spin:.1f} s (8 corners at ~2.6 s each would be ~21 s)')
    proc.terminate()
    rclpy.shutdown()
    return 0 if lap.state == 'done' and clear > 0.0 else 1


if __name__ == '__main__':
    sys.exit(main())
