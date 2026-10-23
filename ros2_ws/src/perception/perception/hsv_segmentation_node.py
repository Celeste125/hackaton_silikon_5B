"""Lightweight simulation detector implementing the YOLO JSON contract."""

import json
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perception.yolo_detection_utils import make_detection, make_payload


COLOR_PROFILES = {
    0: (
        (np.array([0, 75, 50]), np.array([12, 255, 255])),
        (np.array([168, 75, 50]), np.array([179, 255, 255])),
    ),
    1: ((np.array([18, 75, 65]), np.array([40, 255, 255])),),
    2: ((np.array([42, 55, 40]), np.array([92, 255, 255])),),
    3: ((np.array([94, 65, 40]), np.array([138, 255, 255])),),
}

DEBUG_COLORS = {
    0: (0, 0, 255),
    1: (0, 220, 255),
    2: (0, 190, 0),
    3: (255, 100, 0),
}


def detect_hsv_blocks(
    image,
    min_area_px=40.0,
    max_area_px=200000.0,
    morphology_kernel_px=5,
    min_fill_ratio=0.30,
    max_aspect_ratio=4.0,
):
    """Return schema-compatible block detections from one BGR image."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('image must be a BGR image with three channels')

    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel_size = max(1, int(morphology_kernel_px))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    detections = []

    for class_id, ranges in COLOR_PROFILES.items():
        mask = np.zeros((height, width), dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        if kernel_size > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < float(min_area_px) or area > float(max_area_px):
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width <= 0 or box_height <= 0:
                continue
            fill_ratio = area / float(box_width * box_height)
            aspect_ratio = max(
                box_width / float(box_height),
                box_height / float(box_width),
            )
            if fill_ratio < min_fill_ratio or aspect_ratio > max_aspect_ratio:
                continue

            perimeter = cv2.arcLength(contour, True)
            polygon = cv2.approxPolyDP(
                contour, max(1.0, 0.015 * perimeter), True
            ).reshape(-1, 2)
            if len(polygon) < 3:
                polygon = np.array([
                    [x, y],
                    [x + box_width, y],
                    [x + box_width, y + box_height],
                    [x, y + box_height],
                ])
            confidence = min(0.99, 0.65 + 0.30 * fill_ratio)
            detections.append(make_detection(
                class_id,
                confidence,
                [x, y, x + box_width, y + box_height],
                polygon,
                width,
                height,
            ))

    return sorted(
        detections,
        key=lambda item: item['confidence'],
        reverse=True,
    )


class HsvSegmentationNode(Node):
    """Publish simulation block segments without a PyTorch dependency."""

    def __init__(self):
        super().__init__('hsv_segmentation')
        self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw'
        )
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('min_area_px', 40.0)
        self.declare_parameter('max_area_px', 200000.0)
        self.declare_parameter('morphology_kernel_px', 5)
        self.declare_parameter('min_fill_ratio', 0.30)
        self.declare_parameter('max_aspect_ratio', 4.0)

        self.bridge = CvBridge()
        self.detections_pub = self.create_publisher(
            String,
            str(self.get_parameter('detections_topic').value),
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'HSV simulation segmentation ready; publishing YOLO-compatible JSON'
        )

    def _image_callback(self, message):
        started = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
            detections = detect_hsv_blocks(
                image,
                self.get_parameter('min_area_px').value,
                self.get_parameter('max_area_px').value,
                self.get_parameter('morphology_kernel_px').value,
                self.get_parameter('min_fill_ratio').value,
                self.get_parameter('max_aspect_ratio').value,
            )
        except Exception as exc:
            self.get_logger().warn(
                f'HSV segmentation dropped an image: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        payload = make_payload(
            message.header,
            image.shape[1],
            image.shape[0],
            (time.perf_counter() - started) * 1000.0,
            detections,
        )
        self.detections_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=True, separators=(',', ':')
        )))

        if bool(self.get_parameter('publish_debug_image').value):
            debug = image.copy()
            for detection in detections:
                points = np.asarray(
                    detection['mask_polygon_px'], dtype=np.int32
                )
                color = DEBUG_COLORS[detection['class_id']]
                cv2.polylines(debug, [points], True, color, 2)
                center = detection['center_px']
                cv2.circle(
                    debug,
                    (int(center['u']), int(center['v'])),
                    4,
                    color,
                    -1,
                )
            output = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            output.header = message.header
            self.debug_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = HsvSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
