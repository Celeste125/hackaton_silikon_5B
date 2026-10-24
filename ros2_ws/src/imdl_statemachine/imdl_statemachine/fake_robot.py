"""Kinematic stand-in for the Create3 + lidar, to test on a saved map without a robot.

Integrates ``<namespace>/cmd_vel`` into ``<namespace>/odom`` (odom starts at 0,
like the real robot) and ray-casts a LaserScan (frame ``base_link``) from the
map. No physics: the robot just stops in front of occupied cells.
Start it at the pose you give AMCL (``initial_x/y/yaw`` of map_navigation).

It also draws a forward camera (``/image_raw``, ``/camera_info``) of the map's
walls with the ArUco markers of ``marker_route.sim_markers`` from the waypoints
file, so aruco_detector can be tested on it and RViz can show the view.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import yaml

from imdl_statemachine.fake_camera import intrinsics, marker_image, render_view


class Grid:
    """Occupancy lookup from a nav2 map_server YAML + image."""

    def __init__(self, yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as stream:
            meta = yaml.safe_load(stream)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        pixels = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
        if pixels is None:
            raise FileNotFoundError(image)
        if not meta.get('negate', 0):
            pixels = 255 - pixels
        self.occupied = (pixels.astype(np.float32) / 255.0) > float(meta.get('occupied_thresh', 0.65))
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = self.occupied.shape

    def hit(self, x, y):
        """Occupied (or outside the map) for arrays of world coordinates."""
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        row = self.h - 1 - np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        out = np.ones(col.shape, bool)
        out[inside] = self.occupied[row[inside], col[inside]]
        return out


def cast_scan(grid, x, y, yaw, angles, max_range, step):
    """Range per beam; max_range+1 where nothing was hit."""
    dists = np.arange(step, max_range, step)
    ang = yaw + angles[:, None]
    hits = grid.hit(x + dists[None, :] * np.cos(ang), y + dists[None, :] * np.sin(ang))
    first = np.argmax(hits, axis=1)
    return np.where(hits.any(axis=1), dists[first], max_range + 1.0)


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('map', os.path.expanduser('~/maps/my_map.yaml'))
        self.declare_parameter('namespace', '/Team2')
        self.declare_parameter('start', [0.0, 0.0, 0.0])  # x, y, yaw in the map frame
        # Same pose as separate numbers (easy to pass from a launch file); they win over `start`.
        for name in ('start_x', 'start_y', 'start_yaw'):
            self.declare_parameter(name, float('nan'))
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('beams', 360)
        # Fake camera: markers come from marker_route.sim_markers of this file.
        self.declare_parameter('camera', True)
        self.declare_parameter('waypoints_file', os.path.join(
            get_package_share_directory('imdl_statemachine'), 'config', 'waypoints.yaml'))
        self.declare_parameter('camera_xyz', [0.1, 0.0, 0.25])  # on base_link
        self.declare_parameter('marker_size_m', 0.20)           # black square
        self.declare_parameter('hfov_deg', 60.0)

        ns = self.get_parameter('namespace').value
        self.grid = Grid(os.path.expanduser(self.get_parameter('map').value))
        self.max_range = float(self.get_parameter('max_range_m').value)
        n = int(self.get_parameter('beams').value)
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False)

        start = [float(v) for v in self.get_parameter('start').value]
        for index, name in enumerate(('start_x', 'start_y', 'start_yaw')):
            value = float(self.get_parameter(name).value)
            if not math.isnan(value):
                start[index] = value
        self.start = tuple(start)
        self.rel = [0.0, 0.0, 0.0]          # pose in the odom frame
        self.cmd = (0.0, 0.0)
        self.cmd_time = self.get_clock().now()

        self.create_subscription(Twist, f'{ns}/cmd_vel', self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, f'{ns}/odom', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.camera = bool(self.get_parameter('camera').value)
        if self.camera:
            self._setup_camera()
        self.dt = 0.05
        self.create_timer(self.dt, self._step)
        self.create_timer(0.1, self._scan)
        self.get_logger().info(f'Fake robot at {self.start} on {self.get_parameter("map").value}')

    def _setup_camera(self):
        path = os.path.expanduser(self.get_parameter('waypoints_file').value)
        with open(path, 'r', encoding='utf-8') as stream:
            route = yaml.safe_load(stream).get('marker_route') or {}
        self.markers = {int(i): [float(v) for v in p]
                        for i, p in (route.get('sim_markers') or {}).items()}
        self.marker_images = {i: marker_image(i) for i in self.markers}
        self.cam_xyz = [float(v) for v in self.get_parameter('camera_xyz').value]
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.hfov = math.radians(float(self.get_parameter('hfov_deg').value))
        self.image_pub = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.create_timer(0.2, self._image)
        self.get_logger().info(f'Fake camera shows markers {sorted(self.markers)}')

    def _image(self):
        img = render_view(
            self.grid, cast_scan, self._world(), self.markers, self.marker_images,
            cam_offset=self.cam_xyz, marker_size_m=self.marker_size, hfov_rad=self.hfov)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'camera'
        msg.height, msg.width, msg.encoding = img.shape[0], img.shape[1], 'bgr8'
        msg.step, msg.data = img.shape[1] * 3, img.tobytes()
        self.image_pub.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = msg.height, msg.width
        info.k = intrinsics(msg.width, msg.height, self.hfov).flatten().tolist()
        info.d, info.distortion_model = [0.0] * 5, 'plumb_bob'
        self.info_pub.publish(info)

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def _world(self):
        c, s = math.cos(self.start[2]), math.sin(self.start[2])
        x, y, yaw = self.rel
        return (self.start[0] + c * x - s * y, self.start[1] + s * x + c * y, self.start[2] + yaw)

    def _step(self):
        age = (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9
        v, w = self.cmd if age < 0.5 else (0.0, 0.0)
        x, y, yaw = self.rel
        nx, ny, nyaw = x + v * math.cos(yaw) * self.dt, y + v * math.sin(yaw) * self.dt, yaw + w * self.dt
        old = self.rel
        self.rel = [nx, ny, nyaw]
        wx, wy, _ = self._world()
        if v != 0.0 and self.grid.hit(np.array([wx]), np.array([wy]))[0]:
            self.rel = [old[0], old[1], nyaw]   # blocked: keep turning, stop moving
            v = 0.0

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.rel[0], self.rel[1]
        msg.pose.pose.orientation.z = math.sin(self.rel[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.rel[2] / 2.0)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = v, w
        self.odom_pub.publish(msg)

    def _scan(self):
        x, y, yaw = self._world()
        ranges = cast_scan(self.grid, x, y, yaw, self.angles, self.max_range, self.grid.res * 0.5)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = float(self.angles[0])
        msg.angle_increment = float(self.angles[1] - self.angles[0])
        msg.angle_max = float(self.angles[-1])
        msg.range_min, msg.range_max = 0.12, self.max_range
        msg.time_increment, msg.scan_time = 0.0, 0.1
        msg.ranges = [float(r) if r <= self.max_range else math.inf for r in ranges]
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
