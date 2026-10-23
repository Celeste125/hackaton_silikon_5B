"""ROS 2 node for YOLO instance segmentation on RealSense RGB images."""

import hashlib
import json
import threading
import time
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from perception.yolo_detection_utils import (
    CLASS_NAMES,
    make_detection,
    make_payload,
    normalize_model_names,
)


DEFAULT_MODEL_NAME = 'yolo11n_seg_imdl.pt'


class YoloSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter(
            'debug_compressed_image_topic', '/yolo/debug_image/compressed'
        )
        self.declare_parameter('debug_jpeg_quality', 75)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.70)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', '0')
        self.declare_parameter('max_inference_rate_hz', 10.0)
        self.declare_parameter('publish_debug_image', True)

        self.model_path = self._resolve_model_path(str(self.get_parameter('model_path').value))
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.debug_compressed_image_topic = str(
            self.get_parameter('debug_compressed_image_topic').value
        )
        self.debug_jpeg_quality = int(self.get_parameter('debug_jpeg_quality').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.device = self._parse_device(str(self.get_parameter('device').value))
        self.max_inference_rate_hz = float(self.get_parameter('max_inference_rate_hz').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._validate_parameters()

        self._verify_frozen_model_hash()
        self.model = self._load_model()
        self.bridge = CvBridge()
        self._latest_image = None
        self._latest_lock = threading.Lock()
        self._last_processed_stamp = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.image_topic, self._image_cb, sensor_qos)
        self.detections_pub = self.create_publisher(String, self.detections_topic, detection_qos)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, sensor_qos)
        self.debug_compressed_pub = self.create_publisher(
            CompressedImage, self.debug_compressed_image_topic, sensor_qos
        )
        self.timer = self.create_timer(1.0 / self.max_inference_rate_hz, self._inference_cb)

        self.get_logger().info(
            f'YOLO segmentation ready: model={self.model_path}, '
            f'device={self.device}, max_rate={self.max_inference_rate_hz:.1f} Hz, '
            f'compressed_debug={self.debug_compressed_image_topic}'
        )

    def _resolve_model_path(self, configured_path):
        if configured_path:
            path = Path(configured_path).expanduser().resolve()
        else:
            share = Path(get_package_share_directory('perception'))
            path = share / 'models' / DEFAULT_MODEL_NAME
        if not path.is_file():
            raise FileNotFoundError(f'YOLO model does not exist: {path}')
        return path

    @staticmethod
    def _parse_device(device):
        stripped = device.strip()
        if not stripped:
            raise ValueError('device must not be empty')
        return int(stripped) if stripped.isdigit() else stripped

    def _validate_parameters(self):
        for name, value in (
            ('confidence_threshold', self.confidence_threshold),
            ('iou_threshold', self.iou_threshold),
        ):
            if value < 0.0 or value > 1.0:
                raise ValueError(f'{name} must be within [0, 1]')
        if self.image_size <= 0:
            raise ValueError('image_size must be positive')
        if self.max_inference_rate_hz <= 0.0:
            raise ValueError('max_inference_rate_hz must be positive')
        if self.debug_jpeg_quality < 1 or self.debug_jpeg_quality > 100:
            raise ValueError('debug_jpeg_quality must be within [1, 100]')
        for name, value in (
            ('image_topic', self.image_topic),
            ('detections_topic', self.detections_topic),
            ('debug_image_topic', self.debug_image_topic),
            ('debug_compressed_image_topic', self.debug_compressed_image_topic),
        ):
            if not value:
                raise ValueError(f'{name} must not be empty')

    def _verify_frozen_model_hash(self):
        checksum_path = self.model_path.with_suffix('.sha256')
        if not checksum_path.is_file():
            self.get_logger().warn(
                f'No SHA256 file for model {self.model_path}; skipping integrity check'
            )
            return
        expected = checksum_path.read_text(encoding='utf-8').split()[0]
        digest = hashlib.sha256()
        with self.model_path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        actual = digest.hexdigest()
        if actual != expected:
            raise RuntimeError(
                f'Model SHA256 mismatch: expected {expected}, received {actual}'
            )

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'Ultralytics is not installed in this ROS Python environment. '
                'Install the Jetson runtime dependencies before starting the node.'
            ) from exc

        model = YOLO(str(self.model_path), task='segment')
        if model.task != 'segment':
            raise RuntimeError(f'Expected a segmentation model, received task={model.task}')
        names = normalize_model_names(model.names)
        if names != CLASS_NAMES:
            raise RuntimeError(f'Model class mapping mismatch: {names}')
        return model

    def _image_cb(self, msg):
        with self._latest_lock:
            self._latest_image = msg

    def _take_latest_unprocessed_image(self):
        with self._latest_lock:
            msg = self._latest_image
        if msg is None:
            return None
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self._last_processed_stamp:
            return None
        self._last_processed_stamp = stamp
        return msg

    def _inference_cb(self):
        msg = self._take_latest_unprocessed_image()
        if msg is None:
            return
        if not msg.header.frame_id:
            self.get_logger().warn(
                'Dropping RGB frame with an empty frame_id', throttle_duration_sec=2.0
            )
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # CvBridge raises different exception types by encoding.
            self.get_logger().warn(
                f'Cannot convert RGB image: {exc}', throttle_duration_sec=2.0
            )
            return

        try:
            started = time.perf_counter()
            result = self.model.predict(
                source=image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                device=self.device,
                retina_masks=True,
                verbose=False,
            )[0]
            inference_ms = (time.perf_counter() - started) * 1000.0
        except Exception as exc:
            self.get_logger().error(
                f'YOLO inference failed: {exc}', throttle_duration_sec=2.0
            )
            return

        detections = self._convert_result(result, image.shape[1], image.shape[0])
        payload = make_payload(
            msg.header,
            image.shape[1],
            image.shape[0],
            inference_ms,
            detections,
        )
        self.detections_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=True, separators=(',', ':')))
        )

        if self.publish_debug_image:
            self._publish_debug_image(result, detections, msg.header)

    def _convert_result(self, result, image_width, image_height):
        if result.boxes is None or result.masks is None:
            return []
        boxes = result.boxes.xyxy.cpu().tolist()
        class_ids = result.boxes.cls.int().cpu().tolist()
        confidences = result.boxes.conf.cpu().tolist()
        polygons = result.masks.xy
        count = min(len(boxes), len(class_ids), len(confidences), len(polygons))
        if count != len(boxes) or count != len(polygons):
            self.get_logger().warn(
                'YOLO boxes and masks have different lengths; truncating result',
                throttle_duration_sec=2.0,
            )

        detections = []
        for index in range(count):
            try:
                detections.append(
                    make_detection(
                        class_ids[index],
                        confidences[index],
                        boxes[index],
                        polygons[index],
                        image_width,
                        image_height,
                    )
                )
            except (TypeError, ValueError) as exc:
                self.get_logger().warn(
                    f'Dropping invalid YOLO instance: {exc}',
                    throttle_duration_sec=2.0,
                )
        return sorted(detections, key=lambda item: item['confidence'], reverse=True)

    def _publish_debug_image(self, result, detections, header):
        try:
            debug_image = result.plot()
            for detection in detections:
                center = detection['center_px']
                cv2.circle(
                    debug_image,
                    (int(round(center['u'])), int(round(center['v']))),
                    4,
                    (0, 255, 255),
                    -1,
                )
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)

            encoded, jpeg = cv2.imencode(
                '.jpg', debug_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.debug_jpeg_quality],
            )
            if not encoded:
                self.get_logger().warn(
                    'Cannot JPEG-encode YOLO debug image', throttle_duration_sec=2.0
                )
                return
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg.tobytes()
            self.debug_compressed_pub.publish(compressed_msg)
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot publish YOLO debug image: {exc}', throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloSegmentationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
