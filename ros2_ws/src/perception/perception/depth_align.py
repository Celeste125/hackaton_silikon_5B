"""Reproject the raw depth image into the RGB camera frame.

This bypasses the internal realsense2_camera align_depth path, which can fail
on the Jetson when HID/IMU support is unavailable.

Subscriptions:
  /camera/camera/depth/image_rect_raw   (sensor_msgs/Image, Z16)
  /camera/camera/color/camera_info      (sensor_msgs/CameraInfo)
  /camera/camera/depth/camera_info      (sensor_msgs/CameraInfo)
  /camera/camera/extrinsics/depth_to_color (realsense2_camera_msgs/Extrinsics)

Publications:
  /camera/camera/aligned_depth_to_color/image_raw  (sensor_msgs/Image, Z16)
  /camera/camera/aligned_depth_to_color/camera_info (sensor_msgs/CameraInfo)
"""

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def depth_image_to_meters(depth_image, encoding):
    """Return a floating-point depth image in metres.

    RealSense publishes ``16UC1`` millimetres while Gazebo depth cameras
    publish ``32FC1`` metres.  Using the ROS encoding, with the NumPy dtype as
    a conservative fallback, keeps both devices on the same internal unit.
    """
    image = np.asarray(depth_image)
    if image.ndim != 2:
        raise ValueError('depth image must be a single-channel 2D array')

    normalized_encoding = str(encoding).upper()
    if normalized_encoding in ('16UC1', 'MONO16') or np.issubdtype(
            image.dtype, np.integer):
        return image.astype(np.float32) * 0.001
    if normalized_encoding == '32FC1' or np.issubdtype(
            image.dtype, np.floating):
        return image.astype(np.float32, copy=False)
    raise ValueError(
        f'unsupported depth encoding {encoding!r} with dtype {image.dtype}')


def depth_meters_to_millimeters(depth_m, minimum_depth_m=0.01):
    """Convert metres to ``16UC1`` without changing pixel coordinates."""
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError('depth image must be a single-channel 2D array')

    valid = np.isfinite(depth) & (depth > float(minimum_depth_m))
    aligned = np.zeros(depth.shape, dtype=np.uint16)
    if np.any(valid):
        aligned[valid] = np.clip(
            np.rint(depth[valid] * 1000.0),
            1,
            np.iinfo(np.uint16).max,
        ).astype(np.uint16)
    return aligned


def streams_are_pixel_aligned(depth_frame_id, color_frame_id, depth_shape,
                              depth_k, color_k, color_size):
    """Return whether depth pixels already use the color-camera geometry."""
    color_width, color_height = int(color_size[0]), int(color_size[1])
    return (
        bool(depth_frame_id)
        and depth_frame_id == color_frame_id
        and tuple(depth_shape) == (color_height, color_width)
        and np.allclose(
            np.asarray(depth_k, dtype=np.float64),
            np.asarray(color_k, dtype=np.float64),
            rtol=0.0,
            atol=1e-6,
        )
    )


def project_depth_to_color(depth_m, depth_k, color_k, color_size, rotation,
                           translation, minimum_depth_m=0.01):
    """Project depth-camera points into the color image as ``16UC1``.

    Invalid samples are discarded before back-projection.  A small z-buffer
    ensures the nearest surface wins when several depth pixels map to the same
    color pixel.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError('depth image must be a single-channel 2D array')

    d_k = np.asarray(depth_k, dtype=np.float64).reshape(3, 3)
    c_k = np.asarray(color_k, dtype=np.float64).reshape(3, 3)
    d_fx, d_fy = d_k[0, 0], d_k[1, 1]
    c_fx, c_fy = c_k[0, 0], c_k[1, 1]
    if min(d_fx, d_fy, c_fx, c_fy) <= 0.0:
        raise ValueError('camera focal lengths must be positive')

    color_width, color_height = (int(color_size[0]), int(color_size[1]))
    if color_width <= 0 or color_height <= 0:
        raise ValueError('color image dimensions must be positive')

    r_depth_to_color = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    t_depth_to_color = np.asarray(translation, dtype=np.float64).reshape(3)

    valid_depth = np.isfinite(depth) & (depth > float(minimum_depth_m))
    rows, columns = np.nonzero(valid_depth)
    if rows.size == 0:
        return np.zeros((color_height, color_width), dtype=np.uint16)

    z = depth[rows, columns].astype(np.float64)
    x = (columns.astype(np.float64) - d_k[0, 2]) * z / d_fx
    y = (rows.astype(np.float64) - d_k[1, 2]) * z / d_fy
    # Explicit component operations avoid a multi-threaded BLAS dispatch for
    # this narrow 3-D transform. That dispatch is disproportionately expensive
    # for camera-rate callbacks on small computers.
    x_color = (
        r_depth_to_color[0, 0] * x
        + r_depth_to_color[0, 1] * y
        + r_depth_to_color[0, 2] * z
        + t_depth_to_color[0]
    )
    y_color = (
        r_depth_to_color[1, 0] * x
        + r_depth_to_color[1, 1] * y
        + r_depth_to_color[1, 2] * z
        + t_depth_to_color[1]
    )
    z_color = (
        r_depth_to_color[2, 0] * x
        + r_depth_to_color[2, 1] * y
        + r_depth_to_color[2, 2] * z
        + t_depth_to_color[2]
    )
    valid_color_depth = np.isfinite(z_color) & (
        z_color > float(minimum_depth_m))
    x_color = x_color[valid_color_depth]
    y_color = y_color[valid_color_depth]
    z_color = z_color[valid_color_depth]
    if z_color.size == 0:
        return np.zeros((color_height, color_width), dtype=np.uint16)

    u_color = np.rint(
        x_color * c_fx / z_color + c_k[0, 2]
    ).astype(np.int64)
    v_color = np.rint(
        y_color * c_fy / z_color + c_k[1, 2]
    ).astype(np.int64)
    in_image = (
        (u_color >= 0) & (u_color < color_width)
        & (v_color >= 0) & (v_color < color_height)
    )
    if not np.any(in_image):
        return np.zeros((color_height, color_width), dtype=np.uint16)

    u_color = u_color[in_image]
    v_color = v_color[in_image]
    depth_mm = np.clip(
        np.rint(z_color[in_image] * 1000.0), 1, np.iinfo(np.uint16).max
    ).astype(np.uint32)

    empty = np.iinfo(np.uint32).max
    z_buffer = np.full(color_width * color_height, empty, dtype=np.uint32)
    flat_indices = v_color * color_width + u_color
    np.minimum.at(z_buffer, flat_indices, depth_mm)
    z_buffer[z_buffer == empty] = 0
    return z_buffer.reshape(color_height, color_width).astype(np.uint16)


class DepthAlignNode(Node):

    def __init__(self):
        super().__init__('depth_align')

        self.bridge = CvBridge()
        self.color_info = None
        self.depth_info = None

        # D455F depth-to-RGB extrinsics, updated from /extrinsics/depth_to_color.
        # Default D455F calibration value, overwritten after the topic arrives.
        self.R = np.eye(3)
        self.T = np.array([-0.015, 0.0, 0.0])  # Approximate 15 mm offset.
        self._empty_output_count = 0
        self._empty_output_warning_sent = False
        self._fast_path_announced = False

        best_effort_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribe to camera intrinsics.
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._color_info_cb,
            best_effort_qos,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self._depth_info_cb,
            best_effort_qos,
        )

        # Subscribe to extrinsics.
        try:
            from realsense2_camera_msgs.msg import Extrinsics
            self.create_subscription(
                Extrinsics,
                '/camera/camera/extrinsics/depth_to_color',
                self._extrinsics_cb,
                reliable_qos,
            )
            self.get_logger().info('Subscribed to extrinsics topic')
        except ImportError:
            self.get_logger().warn(
                'realsense2_camera_msgs is unavailable; using default extrinsics')

        # There is only one image stream here, so a synchronizer would merely
        # retain stale high-bandwidth frames while the callback is busy.
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self._depth_cb,
            best_effort_qos,
        )

        # Publish aligned depth image.
        self.aligned_pub = self.create_publisher(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            best_effort_qos,
        )
        self.aligned_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            best_effort_qos,
        )

        self.get_logger().info('depth_align node started')

    def _color_info_cb(self, msg):
        self.color_info = msg

    def _depth_info_cb(self, msg):
        self.depth_info = msg

    def _extrinsics_cb(self, msg):
        self.R = np.array(msg.rotation).reshape(3, 3)
        self.T = np.array(msg.translation)

    def _depth_cb(self, depth_msg):
        if self.color_info is None or self.depth_info is None:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
            depth_m = depth_image_to_meters(depth_image, depth_msg.encoding)

            pixel_aligned = streams_are_pixel_aligned(
                depth_frame_id=depth_msg.header.frame_id,
                color_frame_id=self.color_info.header.frame_id,
                depth_shape=depth_m.shape,
                depth_k=self.depth_info.k,
                color_k=self.color_info.k,
                color_size=(self.color_info.width, self.color_info.height),
            )
            if pixel_aligned:
                aligned = depth_meters_to_millimeters(depth_m)
                if not self._fast_path_announced:
                    self.get_logger().info(
                        'Depth and color geometry already match; using '
                        'pixel-preserving fast alignment path')
                    self._fast_path_announced = True
            else:
                aligned = project_depth_to_color(
                    depth_m=depth_m,
                    depth_k=self.depth_info.k,
                    color_k=self.color_info.k,
                    color_size=(self.color_info.width, self.color_info.height),
                    rotation=self.R,
                    translation=self.T,
                )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f'Cannot align depth frame: {exc}', throttle_duration_sec=2.0)
            return

        if np.any(aligned):
            self._empty_output_count = 0
        else:
            self._empty_output_count += 1
            if (self._empty_output_count >= 10
                    and not self._empty_output_warning_sent):
                self.get_logger().warn(
                    'Aligned depth stayed empty for 10 frames; check camera '
                    'intrinsics, frame IDs, and depth encoding')
                self._empty_output_warning_sent = True

        # Publish.
        out_msg = self.bridge.cv2_to_imgmsg(aligned, encoding='16UC1')
        out_msg.header = depth_msg.header
        out_msg.header.frame_id = 'camera_color_optical_frame'
        self.aligned_pub.publish(out_msg)

        info_out = CameraInfo()
        info_out.header = out_msg.header
        info_out.width = self.color_info.width
        info_out.height = self.color_info.height
        info_out.k = self.color_info.k
        info_out.d = self.color_info.d
        info_out.r = self.color_info.r
        info_out.p = self.color_info.p
        info_out.distortion_model = self.color_info.distortion_model
        self.aligned_info_pub.publish(info_out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
