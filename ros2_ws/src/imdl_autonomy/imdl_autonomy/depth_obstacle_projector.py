"""Publish a sparse, floor-filtered RGB-D cloud for low-obstacle safety.

The main lidar is intentionally mounted above the 3.5 cm task blocks.  This
node complements it with geometry from the existing aligned depth stream.  It
does not inspect color or identify task objects: it only exposes nearby,
low-height surfaces to Nav2 and the Collision Monitor.
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .placed_objects import (
    exclude_camera_points_in_zones,
    PLACED_OBJECT_EVENT_TOPIC,
    PlacedObjectZones,
)
from .ros_shutdown import shutdown_node, spin_node_resilient


def depth_image_to_meters(message: Image) -> np.ndarray:
    """Decode a ROS depth image without copying through OpenCV."""
    encoding = str(message.encoding).upper()
    if encoding in {'16UC1', 'MONO16'}:
        dtype = np.dtype(np.uint16)
        scale = 0.001
    elif encoding == '32FC1':
        dtype = np.dtype(np.float32)
        scale = 1.0
    else:
        raise ValueError(f'unsupported depth encoding: {message.encoding!r}')

    dtype = dtype.newbyteorder('>' if message.is_bigendian else '<')
    item_size = dtype.itemsize
    if message.width <= 0 or message.height <= 0:
        raise ValueError('depth image dimensions must be positive')
    if message.step < message.width * item_size:
        raise ValueError('depth image row step is smaller than its width')
    row_items = message.step // item_size
    required_bytes = int(message.height) * int(message.step)
    data = memoryview(message.data)
    if len(data) < required_bytes:
        raise ValueError('depth image data is shorter than its declared size')

    image = np.frombuffer(
        data[:required_bytes],
        dtype=dtype,
        count=int(message.height) * row_items,
    ).reshape(int(message.height), row_items)
    depth = image[:, :int(message.width)].astype(np.float32, copy=False)
    if scale != 1.0:
        depth = depth * scale
    return depth


def quaternion_rotation_matrix(
    quaternion: Sequence[float],
) -> np.ndarray:
    """Return a 3x3 rotation matrix for an ``x, y, z, w`` quaternion."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12:
        raise ValueError('transform quaternion has zero length')
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float64)


def project_low_obstacle_points(
    depth_m: np.ndarray,
    camera_matrix: Sequence[float],
    target_from_camera_rotation: np.ndarray,
    target_from_camera_translation: Sequence[float],
    *,
    pixel_stride: int,
    minimum_range_m: float,
    maximum_range_m: float,
    minimum_height_m: float,
    maximum_height_m: float,
    voxel_size_m: float,
    maximum_points: int,
) -> np.ndarray:
    """Project sampled depth pixels and retain only a low-height obstacle band.

    Returned coordinates remain in the camera optical frame.  The supplied TF
    is used only for the height test, allowing downstream consumers to raytrace
    from the true camera origin.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError('depth image must be a two-dimensional array')
    stride = max(1, int(pixel_stride))
    maximum_points = max(1, int(maximum_points))
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera focal lengths must be positive')

    row_offset = min(stride // 2, max(depth.shape[0] - 1, 0))
    column_offset = min(stride // 2, max(depth.shape[1] - 1, 0))
    rows = np.arange(row_offset, depth.shape[0], stride, dtype=np.int64)
    columns = np.arange(
        column_offset,
        depth.shape[1],
        stride,
        dtype=np.int64,
    )
    if rows.size == 0 or columns.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    sampled = depth[np.ix_(rows, columns)]
    valid = (
        np.isfinite(sampled)
        & (sampled >= float(minimum_range_m))
        & (sampled <= float(maximum_range_m))
    )
    sample_rows, sample_columns = np.nonzero(valid)
    if sample_rows.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    z_camera = sampled[sample_rows, sample_columns].astype(
        np.float64,
        copy=False,
    )
    u = columns[sample_columns].astype(np.float64)
    v = rows[sample_rows].astype(np.float64)
    x_camera = (u - cx) * z_camera / fx
    y_camera = (v - cy) * z_camera / fy
    camera_points = np.column_stack((x_camera, y_camera, z_camera))

    rotation = np.asarray(
        target_from_camera_rotation,
        dtype=np.float64,
    ).reshape(3, 3)
    translation = np.asarray(
        target_from_camera_translation,
        dtype=np.float64,
    ).reshape(3)
    target_points = camera_points @ rotation.T + translation
    height = target_points[:, 2]
    low_obstacle = (
        np.isfinite(height)
        & (height >= float(minimum_height_m))
        & (height <= float(maximum_height_m))
    )
    if not np.any(low_obstacle):
        return np.empty((0, 3), dtype=np.float32)
    camera_points = camera_points[low_obstacle]
    target_points = target_points[low_obstacle]

    voxel = max(0.001, float(voxel_size_m))
    voxel_keys = np.floor(target_points / voxel).astype(np.int64)
    _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
    camera_points = camera_points[np.sort(unique_indices)]

    if camera_points.shape[0] > maximum_points:
        # Keep the nearest geometry when a large wall/floor edge fills the
        # height band; immediate collision safety has priority.
        nearest = np.argsort(camera_points[:, 2])[:maximum_points]
        camera_points = camera_points[np.sort(nearest)]
    return camera_points.astype(np.float32, copy=False)


class DepthObstacleProjector(Node):

    def __init__(self):
        super().__init__('depth_obstacle_projector')
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter(
            'output_topic',
            '/autonomy/depth_obstacle_cloud',
        )
        self.declare_parameter('height_reference_frame', 'base_link')
        self.declare_parameter('pixel_stride', 8)
        self.declare_parameter('minimum_range_m', 0.08)
        self.declare_parameter('maximum_range_m', 0.90)
        self.declare_parameter('minimum_height_m', 0.008)
        self.declare_parameter('maximum_height_m', 0.080)
        self.declare_parameter('voxel_size_m', 0.012)
        self.declare_parameter('maximum_points', 900)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('transform_timeout_sec', 0.08)
        self.declare_parameter('placed_object_frame', 'map')
        self.declare_parameter(
            'placed_object_event_topic',
            PLACED_OBJECT_EVENT_TOPIC,
        )
        self.declare_parameter('placed_obstacle_exclusion_radius_m', 0.075)

        self._camera_info = None
        self._last_publish_ns = 0
        self._announced = False
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._placed_zones = PlacedObjectZones(
            str(self.get_parameter('placed_object_frame').value),
            float(
                self.get_parameter(
                    'placed_obstacle_exclusion_radius_m'
                ).value
            ),
        )
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('placed_object_event_topic').value),
            self._placed_object_callback,
            10,
        )
        self.get_logger().info(
            'depth_obstacle_projector ready: sparse RGB-D geometry -> '
            f'{self.get_parameter("output_topic").value}'
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _placed_object_callback(self, message: String) -> None:
        zone = self._placed_zones.update(message.data)
        if zone is not None:
            self.get_logger().info(
                f'RGB-D fusion excludes completed placement zone at '
                f'({zone.x:.3f}, {zone.y:.3f})'
            )

    def _depth_callback(self, message: Image) -> None:
        if self._camera_info is None or not message.header.frame_id:
            return
        now_ns = self.get_clock().now().nanoseconds
        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        period_ns = int(1e9 / rate)
        if (
            self._last_publish_ns
            and now_ns >= self._last_publish_ns
            and now_ns - self._last_publish_ns < period_ns
        ):
            return

        target_frame = str(
            self.get_parameter('height_reference_frame').value
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=float(
                    self.get_parameter('transform_timeout_sec').value
                )),
            )
            depth = depth_image_to_meters(message)
            rotation = quaternion_rotation_matrix((
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ))
            translation = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            points = project_low_obstacle_points(
                depth,
                self._camera_info.k,
                rotation,
                translation,
                pixel_stride=int(
                    self.get_parameter('pixel_stride').value
                ),
                minimum_range_m=float(
                    self.get_parameter('minimum_range_m').value
                ),
                maximum_range_m=float(
                    self.get_parameter('maximum_range_m').value
                ),
                minimum_height_m=float(
                    self.get_parameter('minimum_height_m').value
                ),
                maximum_height_m=float(
                    self.get_parameter('maximum_height_m').value
                ),
                voxel_size_m=float(
                    self.get_parameter('voxel_size_m').value
                ),
                maximum_points=int(
                    self.get_parameter('maximum_points').value
                ),
            )
            if self._placed_zones.zones and len(points):
                placed_frame = str(
                    self.get_parameter('placed_object_frame').value
                )
                placed_transform = self._tf_buffer.lookup_transform(
                    placed_frame,
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=float(
                        self.get_parameter('transform_timeout_sec').value
                    )),
                )
                placed_rotation = quaternion_rotation_matrix((
                    placed_transform.transform.rotation.x,
                    placed_transform.transform.rotation.y,
                    placed_transform.transform.rotation.z,
                    placed_transform.transform.rotation.w,
                ))
                placed_translation = (
                    placed_transform.transform.translation.x,
                    placed_transform.transform.translation.y,
                    placed_transform.transform.translation.z,
                )
                points = exclude_camera_points_in_zones(
                    points,
                    placed_rotation,
                    placed_translation,
                    self._placed_zones.zones,
                )
        except (TransformException, TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Cannot project RGB-D obstacle cloud: {error}',
                throttle_duration_sec=2.0,
            )
            return

        cloud = point_cloud2.create_cloud_xyz32(
            message.header,
            points.tolist(),
        )
        self._publisher.publish(cloud)
        self._last_publish_ns = now_ns
        if not self._announced:
            self.get_logger().info(
                f'RGB-D low-obstacle cloud active: {len(points)} points, '
                f'frame={message.header.frame_id}'
            )
            self._announced = True


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleProjector()
    try:
        spin_node_resilient(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_node(node)
