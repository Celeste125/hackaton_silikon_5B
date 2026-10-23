"""Fuse YOLO segmentation masks, aligned depth, camera intrinsics, and TF."""

import json
import math

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from imdl_msgs.msg import DetectedObjectArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from perception.object_geometry_utils import (
    masked_depth_statistics,
    pixel_to_camera_point,
    polar_range_and_bearing,
    rectangle_corners_from_polygon,
    stamp_to_nanoseconds,
)
from perception.detected_object_message_utils import make_detected_object_array
from perception.yolo_detection_utils import CLASS_NAMES, MODEL_VERSION

# Registers geometry_msgs stamped types for tf2_ros.Buffer.transform().
import tf2_geometry_msgs  # noqa: F401


class TargetPoseEstimator(Node):
    """Convert raw YOLO detections into all valid 3D object detections."""

    def __init__(self, *, parameter_overrides=None):
        """Create the estimator.

        ``parameter_overrides`` is used by ROS integration tests to isolate
        several complete pipelines without changing the production defaults.
        Launch files and normal ``ros2 run`` usage remain unchanged.
        """
        super().__init__(
            'target_pose_estimator', parameter_overrides=parameter_overrides
        )

        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter(
            'aligned_depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter(
            'typed_objects_topic', '/perception/detected_objects'
        )
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('depth_outlier_threshold_m', 0.10)
        self.declare_parameter('min_mask_area_px', 25)
        self.declare_parameter('min_mask_depth_pixels', 20)
        self.declare_parameter('min_valid_depth_ratio', 0.25)
        self.declare_parameter('max_depth_mad_m', 0.04)
        self.declare_parameter('depth_sync_tolerance_sec', 0.15)
        self.declare_parameter('min_corner_mask_area_px', 40)
        self.declare_parameter('min_rect_fill_ratio', 0.45)

        self.detections_topic = self._string_parameter('detections_topic')
        self.aligned_depth_topic = self._string_parameter('aligned_depth_topic')
        self.camera_info_topic = self._string_parameter('camera_info_topic')
        self.typed_objects_topic = self._string_parameter('typed_objects_topic')
        self.target_frame = self._string_parameter('target_frame')
        self.base_frame = self._string_parameter('base_frame')
        self.min_depth_m = self._float_parameter('min_depth_m')
        self.max_depth_m = self._float_parameter('max_depth_m')
        self.depth_outlier_threshold_m = self._float_parameter(
            'depth_outlier_threshold_m'
        )
        self.min_mask_area_px = int(self.get_parameter('min_mask_area_px').value)
        self.min_mask_depth_pixels = int(
            self.get_parameter('min_mask_depth_pixels').value
        )
        self.min_valid_depth_ratio = self._float_parameter('min_valid_depth_ratio')
        self.max_depth_mad_m = self._float_parameter('max_depth_mad_m')
        self.depth_sync_tolerance_sec = self._float_parameter(
            'depth_sync_tolerance_sec'
        )
        self.min_corner_mask_area_px = int(
            self.get_parameter('min_corner_mask_area_px').value
        )
        self.min_rect_fill_ratio = self._float_parameter('min_rect_fill_ratio')
        self._validate_parameters()

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_header = None
        self.camera_info = None
        self.latest_payload = None
        self.last_processed_key = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, self.detections_topic, self._detections_cb, 10
        )
        self.create_subscription(
            Image,
            self.aligned_depth_topic,
            self._depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_cb,
            qos_profile_sensor_data,
        )
        self.pub_typed_objects = self.create_publisher(
            DetectedObjectArray, self.typed_objects_topic, 10
        )

        self.get_logger().info(
            'target_pose_estimator ready: '
            f'{self.detections_topic} + depth + TF -> '
            f'{self.typed_objects_topic}'
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
        if self.depth_outlier_threshold_m <= 0.0:
            raise ValueError('depth_outlier_threshold_m must be positive')
        if self.min_mask_area_px < 1 or self.min_mask_depth_pixels < 1:
            raise ValueError('mask area and depth pixel thresholds must be positive')
        if not 0.0 <= self.min_valid_depth_ratio <= 1.0:
            raise ValueError('min_valid_depth_ratio must be within [0, 1]')
        if self.max_depth_mad_m <= 0.0:
            raise ValueError('max_depth_mad_m must be positive')
        if self.depth_sync_tolerance_sec <= 0.0:
            raise ValueError('depth_sync_tolerance_sec must be positive')
        if self.min_corner_mask_area_px < 1:
            raise ValueError('min_corner_mask_area_px must be positive')
        if not 0.0 <= self.min_rect_fill_ratio <= 1.0:
            raise ValueError('min_rect_fill_ratio must be within [0, 1]')

    def _detections_cb(self, msg):
        payload = self._parse_payload(msg.data)
        if payload is None:
            return
        self.latest_payload = payload
        self._process_latest_payload()

    def _depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as exc:  # CvBridge error types vary by encoding.
            self.get_logger().warn(
                f'Cannot convert aligned depth image: {exc}',
                throttle_duration_sec=2.0,
            )
            return
        self.latest_depth_header = msg.header
        self._process_latest_payload()

    def _camera_info_cb(self, msg):
        self.camera_info = msg
        self._process_latest_payload()

    def _parse_payload(self, raw):
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Received invalid JSON from YOLO detections', throttle_duration_sec=2.0
            )
            return None

        required = (
            'schema_version',
            'model_version',
            'timestamp',
            'frame_id',
            'image_size',
            'inference_ms',
            'detections',
        )
        if not isinstance(payload, dict) or not all(key in payload for key in required):
            self.get_logger().warn(
                'Received incomplete YOLO detection payload', throttle_duration_sec=2.0
            )
            return None
        if (
            payload['schema_version'] != 1
            or payload['model_version'] != MODEL_VERSION
            or not isinstance(payload['detections'], list)
        ):
            self.get_logger().warn(
                'Received unsupported YOLO detection schema', throttle_duration_sec=2.0
            )
            return None
        if not payload['frame_id']:
            self.get_logger().warn(
                'Received YOLO detections without a frame ID', throttle_duration_sec=2.0
            )
            return None
        try:
            width = int(payload['image_size']['width'])
            height = int(payload['image_size']['height'])
            stamp_to_nanoseconds(payload['timestamp'])
            inference_ms = float(payload['inference_ms'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                'Received YOLO detections with invalid image metadata',
                throttle_duration_sec=2.0,
            )
            return None
        if width < 1 or height < 1 or not math.isfinite(inference_ms) or inference_ms < 0.0:
            return None
        return payload

    def _process_latest_payload(self):
        payload = self.latest_payload
        if payload is None:
            return
        key = (stamp_to_nanoseconds(payload['timestamp']), payload['frame_id'])
        if key == self.last_processed_key:
            return

        if not payload['detections']:
            self._publish_objects(payload, [])
            self.last_processed_key = key
            return

        image_width = int(payload['image_size']['width'])
        image_height = int(payload['image_size']['height'])
        prerequisite_status = self._geometry_prerequisite_status(payload['timestamp'])
        objects = []
        for detection in payload['detections']:
            obj = self._enrich_detection(
                detection,
                image_width,
                image_height,
                payload,
                prerequisite_status,
            )
            if obj is not None:
                objects.append(obj)
        self._publish_objects(payload, objects)
        self.last_processed_key = key

    def _geometry_prerequisite_status(self, detection_stamp):
        if self.latest_depth is None or self.latest_depth_header is None:
            return 'depth_unavailable'
        if self.camera_info is None:
            return 'camera_info_unavailable'
        depth_stamp = self.latest_depth_header.stamp
        if stamp_to_nanoseconds(depth_stamp) == 0 or stamp_to_nanoseconds(detection_stamp) == 0:
            return None
        difference = abs(
            stamp_to_nanoseconds(depth_stamp) - stamp_to_nanoseconds(detection_stamp)
        ) / 1_000_000_000.0
        if difference <= self.depth_sync_tolerance_sec:
            return None
        return 'depth_unsynchronised'

    def _enrich_detection(
        self,
        detection,
        image_width,
        image_height,
        payload,
        prerequisite_status,
    ):
        if not self._is_valid_detection(detection, image_width, image_height):
            self.get_logger().warn(
                'Ignoring invalid YOLO detection item', throttle_duration_sec=2.0
            )
            return None

        obj = {
            'class_id': int(detection['class_id']),
            'class_name': detection['class_name'],
            'is_block': bool(detection['is_block']),
            'confidence': float(detection['confidence']),
            'bbox_xyxy_px': [float(value) for value in detection['bbox_xyxy_px']],
            'center_px': {
                'u': float(detection['center_px']['u']),
                'v': float(detection['center_px']['v']),
            },
            'mask_polygon_px': detection['mask_polygon_px'],
            'timestamp': dict(payload['timestamp']),
            'source_frame_id': payload['frame_id'],
            'depth_m': None,
            'position_camera': None,
            'position_base': None,
            'polar_base': {'range_m': None, 'bearing_rad': None},
            'position_map': None,
            'depth_quality': None,
            'transform_status': prerequisite_status or 'depth_invalid',
            'is_stale': False,
        }
        if obj['is_block']:
            obj.update(rectangle_corners_from_polygon(
                detection['mask_polygon_px'],
                self.min_corner_mask_area_px,
                self.min_rect_fill_ratio,
            ))
        else:
            obj.update({
                'mask_area_px': None,
                'rect_fill_ratio': None,
                'orientation_rad': None,
                'rotated_bbox_px': None,
                'corners_px': None,
                'corner_status': 'not_block',
            })
        if prerequisite_status is not None:
            return obj

        try:
            depth_quality = masked_depth_statistics(
                self.latest_depth,
                detection['mask_polygon_px'],
                image_width,
                image_height,
                self.min_depth_m,
                self.max_depth_m,
                self.depth_outlier_threshold_m,
                self.min_mask_area_px,
                self.min_mask_depth_pixels,
                self.min_valid_depth_ratio,
                self.max_depth_mad_m,
            )
        except (TypeError, ValueError):
            obj['transform_status'] = 'invalid_mask_geometry'
            return obj
        obj['depth_quality'] = depth_quality
        if depth_quality['status'] != 'ok':
            obj['transform_status'] = depth_quality['status']
            return obj
        depth_m = depth_quality['depth_m']

        point_values = pixel_to_camera_point(
            detection['center_px']['u'],
            detection['center_px']['v'],
            depth_m,
            self.camera_info.k,
        )
        if point_values is None:
            obj['transform_status'] = 'camera_info_invalid'
            return obj

        obj['depth_m'] = depth_m
        obj['position_camera'] = point_values
        point_camera = self._make_point(
            self.latest_depth_header.frame_id or payload['frame_id'], point_values
        )
        point_base = self._try_transform(point_camera, self.base_frame)
        point_map = self._try_transform(point_camera, self.target_frame)

        if point_base is not None:
            obj['position_base'] = self._point_dict(point_base)
            range_m, bearing_rad = polar_range_and_bearing(obj['position_base'])
            obj['polar_base'] = {
                'range_m': range_m,
                'bearing_rad': bearing_rad,
            }
        if point_map is not None:
            obj['position_map'] = self._point_dict(point_map)

        if point_base is not None and point_map is not None:
            obj['transform_status'] = 'ok'
        elif point_base is None and point_map is None:
            obj['transform_status'] = 'missing_base_and_map_tf'
        elif point_base is None:
            obj['transform_status'] = 'missing_base_tf'
        else:
            obj['transform_status'] = 'missing_map_tf'
        return obj

    @staticmethod
    def _is_valid_detection(detection, image_width, image_height):
        if not isinstance(detection, dict):
            return False
        try:
            class_id = int(detection['class_id'])
            class_name = str(detection['class_name'])
            confidence = float(detection['confidence'])
            center = detection['center_px']
            polygon = detection['mask_polygon_px']
            bbox = detection['bbox_xyxy_px']
        except (KeyError, TypeError, ValueError):
            return False
        try:
            values = [float(value) for value in bbox]
            values.extend((float(center['u']), float(center['v'])))
            values.extend(float(value) for point in polygon for value in point)
        except (KeyError, TypeError, ValueError):
            return False
        x1, y1, x2, y2 = values[:4]
        center_u, center_v = values[4:6]
        polygon_values = values[6:]
        return (
            CLASS_NAMES.get(class_id) == class_name
            and bool(detection.get('is_block')) == (class_id != 4)
            and 0.0 <= confidence <= 1.0
            and isinstance(center, dict)
            and len(polygon) >= 3
            and len(bbox) == 4
            and all(math.isfinite(value) for value in values)
            and 0.0 <= x1 < x2 <= image_width
            and 0.0 <= y1 < y2 <= image_height
            and 0.0 <= center_u < image_width
            and 0.0 <= center_v < image_height
            and all(
                0.0 <= value <= image_width if index % 2 == 0
                else 0.0 <= value <= image_height
                for index, value in enumerate(polygon_values)
            )
        )

    def _make_point(self, frame_id, values):
        point = PointStamped()
        point.header.frame_id = frame_id
        # The manual depth-align node can lag RGB by a few frames. Use the latest
        # TF while preserving the original RGB timestamp in the JSON output.
        point.header.stamp = Time().to_msg()
        point.point.x = values['x']
        point.point.y = values['y']
        point.point.z = values['z']
        return point

    def _try_transform(self, point, target_frame):
        try:
            return self.tf_buffer.transform(
                point, target_frame, timeout=Duration(seconds=0.2)
            )
        except TransformException:
            return None

    @staticmethod
    def _point_dict(point):
        return {
            'x': float(point.point.x),
            'y': float(point.point.y),
            'z': float(point.point.z),
        }

    def _publish_objects(self, payload, objects):
        self.pub_typed_objects.publish(make_detected_object_array(payload, objects))


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
