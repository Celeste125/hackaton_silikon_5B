"""Publish a synchronised RGB-D debug stream and map-frame RViz markers."""

from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from imdl_msgs.msg import DetectedObjectArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker, MarkerArray

from perception.object_geometry_utils import stamp_to_nanoseconds
from perception.perception_visualization_utils import (
    class_color_bgr,
    colorize_depth,
    object_depth_label,
    object_has_map_position,
)


class PerceptionVisualizer(Node):
    """Keep demonstration visualisation separate from perception decisions."""

    def __init__(self):
        super().__init__('perception_visualizer')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter(
            'aligned_depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'typed_objects_topic', '/perception/detected_objects'
        )
        self.declare_parameter(
            'fused_debug_topic', '/perception/fused_debug/compressed'
        )
        self.declare_parameter('markers_topic', '/perception/object_markers')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('sync_tolerance_sec', 0.20)
        self.declare_parameter('buffer_size', 20)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('publish_fused_debug', True)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('show_map_coordinates', False)

        self.debug_image_topic = self._string_parameter('debug_image_topic')
        self.aligned_depth_topic = self._string_parameter('aligned_depth_topic')
        self.typed_objects_topic = self._string_parameter('typed_objects_topic')
        self.fused_debug_topic = self._string_parameter('fused_debug_topic')
        self.markers_topic = self._string_parameter('markers_topic')
        self.map_frame = self._string_parameter('map_frame')
        self.min_depth_m = self._float_parameter('min_depth_m')
        self.max_depth_m = self._float_parameter('max_depth_m')
        self.sync_tolerance_ns = int(
            self._float_parameter('sync_tolerance_sec') * 1_000_000_000
        )
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_fused_debug = bool(
            self.get_parameter('publish_fused_debug').value
        )
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.show_map_coordinates = bool(
            self.get_parameter('show_map_coordinates').value
        )
        self._validate_parameters()

        self.bridge = CvBridge()
        self.debug_frames = deque(maxlen=self.buffer_size)
        self.depth_frames = deque(maxlen=self.buffer_size)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        object_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.debug_image_topic, self._debug_cb, sensor_qos)
        self.create_subscription(
            Image, self.aligned_depth_topic, self._depth_cb, sensor_qos
        )
        self.create_subscription(
            DetectedObjectArray,
            self.typed_objects_topic,
            self._objects_cb,
            object_qos,
        )
        self.fused_pub = self.create_publisher(
            CompressedImage, self.fused_debug_topic, sensor_qos
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, object_qos)

        self.get_logger().info(
            'perception_visualizer ready: '
            f'{self.debug_image_topic} + depth + {self.typed_objects_topic} -> '
            f'{self.fused_debug_topic}, {self.markers_topic}'
        )

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _float_parameter(self, name):
        return float(self.get_parameter(name).value)

    def _validate_parameters(self):
        if self.min_depth_m <= 0.0 or self.max_depth_m <= self.min_depth_m:
            raise ValueError('min_depth_m and max_depth_m must define a valid range')
        if self.sync_tolerance_ns <= 0:
            raise ValueError('sync_tolerance_sec must be positive')
        if self.buffer_size < 1:
            raise ValueError('buffer_size must be positive')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be within [1, 100]')

    def _debug_cb(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot convert YOLO debug image: {exc}', throttle_duration_sec=2.0
            )
            return
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        self.debug_frames.append((stamp_ns, message.header, image))
        # Keep the remote demonstration live even when YOLO reports no object.
        # The matching detected_objects callback republishes this frame with
        # object-specific distance annotations shortly afterwards.
        if self.publish_fused_debug:
            self._publish_fused_debug([], stamp_ns)

    def _depth_cb(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot convert aligned depth image: {exc}', throttle_duration_sec=2.0
            )
            return
        self.depth_frames.append((stamp_to_nanoseconds(message.header.stamp), message.header, depth))

    def _objects_cb(self, message):
        objects = self._typed_objects_as_visualization_dicts(message)
        if self.publish_markers:
            self._publish_markers(objects)
        if self.publish_fused_debug:
            self._publish_fused_debug(objects)

    @staticmethod
    def _typed_objects_as_visualization_dicts(message):
        """Adapt the typed contract to the existing drawing helpers only."""
        objects = []
        for item in message.objects:
            objects.append({
                'class_name': item.class_name,
                'confidence': float(item.confidence),
                'center_px': {
                    'u': float(item.center_u_px),
                    'v': float(item.center_v_px),
                },
                'mask_polygon_px': [
                    [float(point.x), float(point.y)]
                    for point in item.mask_polygon_px
                ],
                'timestamp': {
                    'sec': int(item.header.stamp.sec),
                    'nanosec': int(item.header.stamp.nanosec),
                },
                'depth_m': float(item.depth_m) if item.depth_valid else None,
                'polar_base': {
                    'range_m': (
                        float(item.range_m) if item.base_pose_valid else None
                    ),
                },
                'position_map': {
                    'x': float(item.position_map_m.x),
                    'y': float(item.position_map_m.y),
                    'z': float(item.position_map_m.z),
                },
                'transform_status': item.transform_status,
                'map_pose_valid': bool(item.map_pose_valid),
            })
        return objects

    def _publish_fused_debug(self, objects, requested_stamp_ns=None):
        stamp_ns = requested_stamp_ns
        if stamp_ns is None and objects:
            stamp_ns = self._object_stamp_ns(objects[0])
        if stamp_ns is None:
            return
        debug_item = self._nearest_frame(self.debug_frames, stamp_ns)
        depth_item = self._nearest_frame(self.depth_frames, stamp_ns)
        if debug_item is None or depth_item is None:
            self.get_logger().debug('Waiting for synchronised debug RGB and depth')
            return

        _, header, debug_image = debug_item
        _, _, depth_image = depth_item
        try:
            depth_colored = colorize_depth(
                depth_image, self.min_depth_m, self.max_depth_m
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'Cannot colourise depth image: {exc}', throttle_duration_sec=2.0
            )
            return
        if depth_colored.shape[:2] != debug_image.shape[:2]:
            depth_colored = cv2.resize(
                depth_colored,
                (debug_image.shape[1], debug_image.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        left = debug_image.copy()
        right = depth_colored.copy()
        self._draw_header(left, 'YOLO segmentation + object distance')
        self._draw_header(
            right,
            f'Aligned depth: warm={self.min_depth_m:.2f}m, cool={self.max_depth_m:.2f}m',
        )
        for obj in objects:
            self._draw_object_annotation(left, obj)
            self._draw_object_outline(right, obj)

        fused = np.hstack((left, right))
        ok, encoded = cv2.imencode(
            '.jpg', fused, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            self.get_logger().warn('Failed to JPEG-encode fused debug frame')
            return
        output = CompressedImage()
        output.header = header
        output.format = 'jpeg'
        output.data = encoded.tobytes()
        self.fused_pub.publish(output)

    def _nearest_frame(self, frames, stamp_ns):
        if not frames:
            return None
        nearest = min(frames, key=lambda item: abs(item[0] - stamp_ns))
        if abs(nearest[0] - stamp_ns) > self.sync_tolerance_ns:
            return None
        return nearest

    @staticmethod
    def _object_stamp_ns(obj):
        try:
            return stamp_to_nanoseconds(obj['timestamp'])
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _draw_header(image, text):
        cv2.rectangle(image, (0, 0), (image.shape[1], 30), (25, 25, 25), -1)
        cv2.putText(
            image, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (245, 245, 245), 1, cv2.LINE_AA,
        )

    def _draw_object_annotation(self, image, obj):
        center = obj.get('center_px')
        if not isinstance(center, dict):
            return
        try:
            u = int(round(float(center['u'])))
            v = int(round(float(center['v'])))
        except (KeyError, TypeError, ValueError):
            return
        color = class_color_bgr(obj.get('class_name'))
        cv2.circle(image, (u, v), 4, color, -1)
        label = object_depth_label(obj)
        if self.show_map_coordinates and object_has_map_position(obj):
            position = obj['position_map']
            label = f"{label} M:({position['x']:.2f},{position['y']:.2f})"
        cv2.putText(
            image, label, (max(4, u - 90), min(image.shape[0] - 8, v + 18)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv2.LINE_AA,
        )

    @staticmethod
    def _draw_object_outline(image, obj):
        polygon = obj.get('mask_polygon_px')
        if not isinstance(polygon, list) or len(polygon) < 3:
            return
        try:
            points = np.asarray(polygon, dtype=np.int32).reshape((-1, 1, 2))
        except (TypeError, ValueError):
            return
        cv2.polylines(
            image, [points], True, class_color_bgr(obj.get('class_name')), 2, cv2.LINE_AA
        )

    def _publish_markers(self, objects):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, obj in enumerate(objects):
            if not object_has_map_position(obj):
                continue
            markers.markers.extend(self._object_markers(index, obj))
        self.marker_pub.publish(markers)

    def _object_markers(self, index, obj):
        position = obj['position_map']
        class_name = str(obj.get('class_name', 'unknown'))
        color = class_color_bgr(class_name)
        red, green, blue = color[2] / 255.0, color[1] / 255.0, color[0] / 255.0

        shape = Marker()
        shape.header.frame_id = self.map_frame
        shape.header.stamp = self.get_clock().now().to_msg()
        shape.ns = 'detected_objects'
        shape.id = index * 2
        shape.type = Marker.CYLINDER if class_name == 'blue_obstacle' else Marker.CUBE
        shape.action = Marker.ADD
        shape.pose.position.x = float(position['x'])
        shape.pose.position.y = float(position['y'])
        # Display height only: it keeps markers above the 2D occupancy grid.
        shape.pose.position.z = 0.05
        shape.pose.orientation.w = 1.0
        shape.scale.x = 0.10 if class_name == 'blue_obstacle' else 0.07
        shape.scale.y = 0.10 if class_name == 'blue_obstacle' else 0.07
        shape.scale.z = 0.15 if class_name == 'blue_obstacle' else 0.07
        shape.color.r = red
        shape.color.g = green
        shape.color.b = blue
        shape.color.a = 0.90

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = shape.header.stamp
        text.ns = 'detected_object_labels'
        text.id = index * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = shape.pose.position.x
        text.pose.position.y = shape.pose.position.y
        text.pose.position.z = 0.20
        text.pose.orientation.w = 1.0
        text.scale.z = 0.11
        text.color.r = red
        text.color.g = green
        text.color.b = blue
        text.color.a = 1.0
        text.text = self._marker_label(obj)
        return shape, text

    @staticmethod
    def _marker_label(obj):
        class_name = str(obj.get('class_name', 'unknown'))
        depth = obj.get('depth_m')
        polar = obj.get('polar_base') if isinstance(obj.get('polar_base'), dict) else {}
        range_m = polar.get('range_m')
        try:
            return f'{class_name}\nZ {float(depth):.2f}m | R {float(range_m):.2f}m'
        except (TypeError, ValueError):
            return class_name


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
