"""Detect ArUco markers and publish each one's pose in the map frame.

Publishes ``aruco/marker_<id>`` (PoseStamped, frame ``target_frame``): position
is the marker centre, orientation is the yaw of its outward normal. The state
machine drives to a point in front of it.

Works with OpenCV 4.5 (ROS Humble image) and 4.7+.

Defaults fit a plain USB camera run with v4l2_camera (/image_raw). Without a
calibration (empty camera_info) the intrinsics come from ``hfov_deg``; without
a camera frame in TF, ``mount_xyz`` places a forward-facing camera on base_link.
"""

import math

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from imdl_statemachine.markers import (
    marker_in_frame, marker_yaw, quat_to_matrix, solve_marker_pose)


def image_to_gray(msg):
    channels = {'mono8': 1, '8UC1': 1, 'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}[msg.encoding]
    row = np.frombuffer(msg.data, np.uint8).reshape(msg.height, -1)
    img = row[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return np.ascontiguousarray(img[:, :, 0])
    code = {'bgr8': cv2.COLOR_BGR2GRAY, 'rgb8': cv2.COLOR_RGB2GRAY,
            'bgra8': cv2.COLOR_BGRA2GRAY, 'rgba8': cv2.COLOR_RGBA2GRAY}[msg.encoding]
    return cv2.cvtColor(img, code)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.20)  # printed black square, measured
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('max_range_m', 4.0)
        # Corners closer to the image border than this fraction of the width are
        # skipped: a marker cut off or seen at a sharp angle gives a bad pose.
        self.declare_parameter('border_margin', 0.08)
        self.declare_parameter('hfov_deg', 60.0)  # used when camera_info is uncalibrated
        # Camera position [x, y, z] on base_link, looking straight ahead. Leave
        # empty to use the camera frame of the image from TF instead.
        self.declare_parameter('mount_xyz', [0.0])
        self.declare_parameter('base_frame', 'base_link')

        self.size = float(self.get_parameter('marker_size_m').value)
        self.target = self.get_parameter('target_frame').value
        self.max_range = float(self.get_parameter('max_range_m').value)

        dict_id = getattr(cv2.aruco, self.get_parameter('dictionary').value)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
            self._detect = detector.detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda g: cv2.aruco.detectMarkers(g, self.dictionary, parameters=params)

        self.margin = float(self.get_parameter('border_margin').value)
        self.hfov = math.radians(float(self.get_parameter('hfov_deg').value))
        mount = list(self.get_parameter('mount_xyz').value)
        self.mount = mount if len(mount) == 3 else None
        self.base_frame = self.get_parameter('base_frame').value

        h = self.size / 2.0
        self.obj_points = np.array(
            [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

        self.k = None
        self.dist = None
        self.pubs = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._on_info, 1)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, qos_profile_sensor_data)
        self.get_logger().info('ArUco detector waiting for camera_info and images')

    def _on_info(self, msg):
        k = np.array(msg.k, float).reshape(3, 3)
        if k[0, 0] <= 0.0:
            return  # uncalibrated camera: _on_image falls back to hfov_deg
        self.k = k
        self.dist = np.array(msg.d, float) if len(msg.d) else np.zeros(5)

    def _on_image(self, msg):
        if self.k is None:
            f = (msg.width / 2.0) / math.tan(self.hfov / 2.0)
            self.k = np.array([[f, 0, msg.width / 2.0], [0, f, msg.height / 2.0], [0, 0, 1.0]])
            self.dist = np.zeros(5)
            self.get_logger().warn(
                f'no calibrated camera_info, assuming hfov {math.degrees(self.hfov):.0f} deg')
        try:
            gray = image_to_gray(msg)
        except KeyError:
            self.get_logger().warn(f'unsupported encoding {msg.encoding}', throttle_duration_sec=10.0)
            return
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return
        frame = self.base_frame if self.mount else msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(self.target, frame, Time())
        except tf2_ros.TransformException as err:
            self.get_logger().warn(f'no TF: {err}', throttle_duration_sec=5.0)
            return
        q, t = tf.transform.rotation, tf.transform.translation
        rot_t = quat_to_matrix(q.x, q.y, q.z, q.w)
        trans = [t.x, t.y, t.z]
        if self.mount:
            # Optical frame (x right, y down, z forward) on a forward-facing base_link.
            optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
            trans = list(rot_t @ np.array(self.mount) + np.array(trans))
            rot_t = rot_t @ optical

        edge = self.margin * msg.width
        for marker_id, corner in zip(ids.flatten(), corners):
            xy = corner.reshape(4, 2)
            if (xy[:, 0].min() < edge or xy[:, 0].max() > msg.width - edge
                    or xy[:, 1].min() < edge or xy[:, 1].max() > msg.height - edge):
                continue
            pose = solve_marker_pose(
                self.obj_points, xy.astype(np.float32), self.k, self.dist)
            if pose is None:
                continue
            rvec, tvec = pose
            if float(np.linalg.norm(tvec)) > self.max_range:
                continue  # far markers give a noisy pose
            rot, _ = cv2.Rodrigues(rvec)
            center, normal = marker_in_frame(rot, tvec.flatten(), rot_t, trans)
            yaw = marker_yaw(normal)
            if yaw is None:
                continue
            self._publish(int(marker_id), center, yaw, msg.header.stamp)

    def _publish(self, marker_id, center, yaw, stamp):
        if marker_id not in self.pubs:
            self.pubs[marker_id] = self.create_publisher(PoseStamped, f'aruco/marker_{marker_id}', 10)
            self.get_logger().info(f'first sighting of marker {marker_id}')
        pose = PoseStamped()
        pose.header.frame_id = self.target
        pose.header.stamp = stamp
        pose.pose.position.x = float(center[0])
        pose.pose.position.y = float(center[1])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pubs[marker_id].publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
