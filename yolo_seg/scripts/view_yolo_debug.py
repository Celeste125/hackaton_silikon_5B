#!/usr/bin/env python3
"""Display the Jetson YOLO JPEG debug stream with minimal buffering."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class CompressedDebugViewer(Node):
    def __init__(self):
        super().__init__('yolo_debug_viewer')
        self.declare_parameter('topic', '/yolo/debug_image/compressed')
        self.declare_parameter('fullscreen', False)
        topic = str(self.get_parameter('topic').value)
        self.fullscreen = bool(self.get_parameter('fullscreen').value)

        self.window_name = 'IMDL YOLO Segmentation'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 720)
        if self.fullscreen:
            cv2.setWindowProperty(
                self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, topic, self._image_cb, qos)
        self.get_logger().info(f'Viewing JPEG debug stream: {topic}')

    def _image_cb(self, message):
        image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('Dropped invalid JPEG frame', throttle_duration_sec=2.0)
            return
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        return super().destroy_node()


def main():
    rclpy.init()
    viewer = CompressedDebugViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
