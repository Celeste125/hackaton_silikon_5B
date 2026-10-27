import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from .analyzer_node import bgr_to_image

# reliable: large frames are re-sent when a packet is lost (best-effort lost ~70%)
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


class VideoSource(Node):
    """Publishes a video file as sensor_msgs/Image on 'image_raw'."""

    def __init__(self):
        super().__init__('video_source')
        self.declare_parameter('video_path', 'videos/clip_hd.mp4')
        self.declare_parameter('rate', 12.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'camera')

        path = self.get_parameter('video_path').value
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            self.get_logger().error(f'cannot open video: {path}')
            raise SystemExit(1)

        self.loop = self.get_parameter('loop').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(Image, 'image_raw', IMAGE_QOS)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate').value, self._tick)
        self.get_logger().info(f'publishing {path}')

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            self.get_logger().info('end of video')
            self.timer.cancel()
            return
        header = Image().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg = bgr_to_image(frame, header)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VideoSource()
    except SystemExit:
        rclpy.shutdown()
        return
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
