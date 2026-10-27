"""Find the robot on the saved map from one lidar scan, and correct AMCL.

    ros2 service call /relocalize std_srvs/srv/Trigger     (robot standing still)

AMCL only tracks the robot from a starting estimate. Start it in the wrong
place (or let it lose track) and every saved pose and every goal is off by
that much; it does not recover on its own in an arena that looks the same
in many places. This searches the whole map instead: every free spot and
heading, scored by how close the scan's points land to the map's walls, then
refined around the best few. The result goes to AMCL on /initialpose, the
same as RViz's "2D Pose Estimate", but only if the match is good and clearly
better than the best match somewhere else; otherwise nothing changes.

The robot must stand still while the scans are taken (checked with odometry).
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

WALL = 65       # occupancy at or above this is a wall
CAP = 0.30      # m: a point further from any wall than this counts as this far
FIT_M = 0.05    # m: a point this close to a wall counts as fitting


class MapMatcher:
    """Scores robot poses on one occupancy grid. Build once per map."""

    def __init__(self, grid, resolution, origin_x, origin_y):
        self.grid = np.asarray(grid)
        self.res = float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)
        self.h, self.w = self.grid.shape
        self.dist = np.minimum(distance_transform_edt(self.grid < WALL) * self.res, CAP)
        # Where the robot can stand: known free, not hugging a wall.
        self.free = distance_transform_edt(self.grid == 0) * self.res >= 0.2

    def _wall_dist(self, X, Y):
        c = ((X - self.ox) / self.res).astype(int)
        r = ((Y - self.oy) / self.res).astype(int)
        inside = (c >= 0) & (c < self.w) & (r >= 0) & (r < self.h)
        d = np.full(X.shape, CAP)
        d[inside] = self.dist[r[inside], c[inside]]
        return d

    def scores(self, points, x, y, yaws):
        """Mean distance from points (base_link, Nx2) to walls, per yaw. Lower is better."""
        c, s = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
        X = x + c * points[:, 0] - s * points[:, 1]
        Y = y + s * points[:, 0] + c * points[:, 1]
        return self._wall_dist(X, Y).mean(axis=1)

    def fit(self, points, pose):
        """Fraction of points within FIT_M of a wall at pose (x, y, yaw)."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        X = x + c * points[:, 0] - s * points[:, 1]
        Y = y + s * points[:, 0] + c * points[:, 1]
        return float(np.mean(self._wall_dist(X, Y) < FIT_M))

    def _refine(self, points, pose, score):
        x, y, yaw = pose
        for step, ang in ((0.05, 2.0), (0.02, 1.0), (0.01, 0.5)):
            for _ in range(4):
                yaws = yaw + np.radians([-ang, 0.0, ang])
                best = (score, x, y, yaw)
                for dx in (-step, 0.0, step):
                    for dy in (-step, 0.0, step):
                        sc = self.scores(points, x + dx, y + dy, yaws)
                        i = int(np.argmin(sc))
                        if sc[i] < best[0]:
                            best = (float(sc[i]), x + dx, y + dy, float(yaws[i]))
                if best[0] >= score:
                    break
                score, x, y, yaw = best
        return score, (x, y, math.atan2(math.sin(yaw), math.cos(yaw)))

    def search(self, points, n_best=6, step_cells=2, yaw_step_deg=4.0):
        """Best poses at clearly different places: [(fit, (x, y, yaw))], best first."""
        yaws = np.radians(np.arange(0.0, 360.0, yaw_step_deg))
        coarse = []
        for r, c in np.argwhere(self.free)[::step_cells]:
            x, y = self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res
            sc = self.scores(points, x, y, yaws)
            i = int(np.argmin(sc))
            coarse.append((float(sc[i]), x, y, float(yaws[i])))
        coarse.sort()
        picked = []
        for cand in coarse:
            if all(_apart(cand[1:], p[1:]) for p in picked):
                picked.append(cand)
            if len(picked) == n_best:
                break
        refined = [self._refine(points, p[1:], p[0]) for p in picked]
        out = [(self.fit(points, pose), pose) for _, pose in refined]
        out.sort(key=lambda t: -t[0])
        # refining can bring two candidates together; keep one per place
        result = []
        for f, pose in out:
            if all(_apart(pose, q) for _, q in result):
                result.append((f, pose))
        return result


def _apart(a, b, dist=0.4, ang=0.5):
    return (math.hypot(a[0] - b[0], a[1] - b[1]) > dist
            or abs(math.remainder(a[2] - b[2], 2 * math.pi)) > ang)


def decide(candidates, min_fit=0.75, min_margin=0.15):
    """(pose or None, reason). Accept only a good fit that clearly beats the rest."""
    if not candidates:
        return None, 'no candidate poses (empty map?)'
    best_fit, best = candidates[0]
    if best_fit < min_fit:
        return None, f'best match fits only {best_fit:.0%} of the scan (need {min_fit:.0%})'
    if len(candidates) > 1 and candidates[1][0] > best_fit - min_margin:
        x, y, _ = candidates[1][1]
        return None, (f'ambiguous: {best_fit:.0%} here and {candidates[1][0]:.0%} at '
                      f'{x:.2f}, {y:.2f}; drive somewhere less symmetric and retry')
    why = f'fits {best_fit:.0%} of the scan'
    if len(candidates) > 1:
        why += f' (next best place {candidates[1][0]:.0%})'
    return best, why


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Relocalizer(Node):
    def __init__(self, tf=None):
        """tf: a shared tf2 Buffer (all feed_mission nodes in one process), else its own."""
        super().__init__('relocalizer')
        self.declare_parameter('scans', 3)              # scans merged for one search
        self.declare_parameter('max_motion', 0.02)      # m (and 0.02 rad) allowed meanwhile
        self.declare_parameter('min_fit', 0.75)
        self.declare_parameter('min_margin', 0.15)
        self.tf = tf
        if tf is None:
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
        cb = ReentrantCallbackGroup()  # the service waits for scans while others arrive
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.matcher = None
        self.scans = []
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched, callback_group=cb)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data,
                                 callback_group=cb)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_service(Trigger, 'relocalize', self._on_relocalize, callback_group=cb)

    def _on_map(self, msg):
        self.map = msg
        self.matcher = None  # rebuilt on the next request

    def _on_scan(self, msg):
        self.scans = (self.scans + [msg])[-10:]

    def _odom_pose(self):
        try:
            t = self.tf.lookup_transform('odom', 'base_link', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    def _points(self, scans):
        pts = []
        for s in scans:
            try:
                t = self.tf.lookup_transform('base_link', s.header.frame_id, Time())
            except TransformException:
                continue
            r = np.asarray(s.ranges, dtype=float)
            a = s.angle_min + np.arange(len(r)) * s.angle_increment + _yaw(t.transform.rotation)
            ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 6.0))
            pts.append(np.c_[r[ok] * np.cos(a[ok]) + t.transform.translation.x,
                             r[ok] * np.sin(a[ok]) + t.transform.translation.y])
        return np.vstack(pts) if pts else np.zeros((0, 2))

    def _on_relocalize(self, request, response):
        response.success = False
        if self.map is None:
            response.message = 'no /map yet: is navigation (map_server) running?'
            return response
        need = int(self.get_parameter('scans').value)
        start_pose = self._odom_pose()
        self.scans = []
        deadline = time.monotonic() + 5.0
        while len(self.scans) < need and time.monotonic() < deadline:
            time.sleep(0.05)
        end_pose = self._odom_pose()
        if len(self.scans) < need:
            response.message = 'no lidar scans on /scan'
            return response
        if start_pose is None or end_pose is None:
            response.message = 'no odom -> base_link: cannot check the robot is standing still'
            return response
        moved = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        turned = abs(math.remainder(end_pose[2] - start_pose[2], 2 * math.pi))
        lim = float(self.get_parameter('max_motion').value)
        if moved > lim or turned > lim:
            response.message = 'the robot moved during the scans: stop it and retry'
            return response
        points = self._points(self.scans[:need])[::2]
        if len(points) < 50:
            response.message = f'only {len(points)} usable lidar points'
            return response
        if self.matcher is None:
            info = self.map.info
            grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
            self.matcher = MapMatcher(grid, info.resolution,
                                      info.origin.position.x, info.origin.position.y)
        t0 = time.monotonic()
        candidates = self.matcher.search(points)
        pose, why = decide(candidates, float(self.get_parameter('min_fit').value),
                           float(self.get_parameter('min_margin').value))
        took = time.monotonic() - t0
        if pose is None:
            response.message = f'not changed: {why}'
            self.get_logger().warn(f'relocalize: {response.message} ({took:.1f} s)')
            return response
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(3) ** 2
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)
        response.success = True
        response.message = (f'robot is at {pose[0]:.2f}, {pose[1]:.2f}, '
                            f'{math.degrees(pose[2]):.0f} deg: {why}')
        self.get_logger().info(f'relocalize: {response.message} ({took:.1f} s)')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Relocalizer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
