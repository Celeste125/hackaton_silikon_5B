import array
import time
from dataclasses import fields, replace

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Header

from .pipeline import GRAIN_TYPES, GrainPipeline, PipelineParams, add_legend

# newest frame only; drop old ones if processing is slower than the camera
def image_qos(reliable):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST, depth=1)


def image_to_bgr(msg):
    """sensor_msgs/Image (bgr8 or rgb8) -> BGR numpy array (no cv_bridge needed)."""
    if msg.encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'unsupported encoding {msg.encoding!r}, use bgr8 or rgb8')
    buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def bgr_to_image(bgr, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    # array.array skips the per-byte validation of a plain list/bytes (365 ms vs 3 ms)
    data = array.array('B')
    data.frombytes(np.ascontiguousarray(bgr).tobytes())
    msg.data = data
    return msg


class GrainAnalyzer(Node):
    def __init__(self):
        super().__init__('grain_analyzer')

        # every PipelineParams field becomes a ROS parameter with the same name
        defaults = PipelineParams()
        values = {}
        for f in fields(defaults):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            value = self.get_parameter(f.name).value
            values[f.name] = list(value) if isinstance(value, (list, tuple)) or hasattr(value, 'tolist') else value
        self.params = PipelineParams(**values)

        # keep the CPU free for other tasks: analyse at most max_rate frames per
        # second (the rest are dropped), with at most cv_threads OpenCV threads
        self.declare_parameter('max_rate', 2.0)
        self.declare_parameter('cv_threads', 1)
        cv2.setNumThreads(int(self.get_parameter('cv_threads').value))
        self.pipeline = GrainPipeline(self.params)
        self.add_on_set_parameters_callback(self._on_params)

        # video_path: read a video file (or a camera index like "0") directly, so no
        # 6 MB frames travel between nodes. Empty: subscribe to the image_raw topic.
        self.declare_parameter('video_path', '')
        self.declare_parameter('loop', True)
        # reliable matches reliable and best-effort cameras only if the camera is
        # reliable too; set false for a best-effort camera driver (then expect lost frames)
        self.declare_parameter('reliable_image', True)
        self.cap = None
        video_path = self.get_parameter('video_path').value
        if video_path:
            self._open_source(video_path)
        else:
            self.sub = self.create_subscription(Image, 'image_raw', self._on_image,
                image_qos(self.get_parameter('reliable_image').value))
        self.pub_overlay = self.create_publisher(Image, '~/overlay', 1)
        self.pub_stacked = self.create_publisher(Image, '~/stacked', 1)
        self.pub_coverage = self.create_publisher(Float32, '~/coverage', 10)
        self.pub_homogeneity = self.create_publisher(Float32, '~/homogeneity', 10)
        self.pub_area = {g: self.create_publisher(Float32, f'~/area/{g}', 10) for g in GRAIN_TYPES}

        self.last_time = time.perf_counter()
        if self.cap is None:
            self.get_logger().info('grain_analyzer ready, waiting for frames on "image_raw"')

    def _open_source(self, video_path):
        is_camera = video_path.isdigit()
        self.cap = cv2.VideoCapture(int(video_path) if is_camera else video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'cannot open video source: {video_path}')
            raise SystemExit(1)
        rate = max(self.get_parameter('max_rate').value, 1e-3)
        # a file is read in real time: skip the frames between two analyses
        file_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.skip = 0 if is_camera or file_fps <= 0 else max(round(file_fps / rate) - 1, 0)
        if is_camera:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(f'grain_analyzer reading {video_path} at {rate:.1f} frames/s')

    def _on_timer(self):
        for _ in range(self.skip):
            self.cap.grab()
        ok, frame = self.cap.read()
        if not ok:
            if self.get_parameter('loop').value and self.cap.get(cv2.CAP_PROP_POS_FRAMES) > 0:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            else:
                self.get_logger().info('end of video')
                self.timer.cancel()
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera'
        self._analyse(frame, header)

    def _on_params(self, params):
        changes = {p.name: (list(p.value) if hasattr(p.value, 'tolist') or isinstance(p.value, (list, tuple)) else p.value)
                   for p in params if hasattr(self.params, p.name)}
        self.params = replace(self.params, **changes)
        self.pipeline = GrainPipeline(self.params)
        return SetParametersResult(successful=True)

    def _publish_float(self, pub, value):
        pub.publish(Float32(data=float(value)))

    def _on_image(self, msg):
        now = time.perf_counter()
        if now - self.last_time < 1.0 / max(self.get_parameter('max_rate').value, 1e-3):
            return
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        self._analyse(bgr, msg.header)

    def _analyse(self, bgr, header):
        try:
            result = self.pipeline.process(bgr)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        now = time.perf_counter()
        fps = 1 / max(now - self.last_time, 1e-6)
        self.last_time = now

        add_legend(result.overlay, fps, result.total_area, result.areas, result.homogeneity)

        self._publish_float(self.pub_coverage, result.total_area)
        self._publish_float(self.pub_homogeneity, result.homogeneity)
        for grain, area in result.areas.items():
            self._publish_float(self.pub_area[grain], area)

        # only encode images somebody is listening to
        if self.pub_overlay.get_subscription_count():
            self.pub_overlay.publish(bgr_to_image(result.overlay, header))
        if self.pub_stacked.get_subscription_count():
            stacked = np.vstack([result.raw, result.overlay])
            self.pub_stacked.publish(bgr_to_image(stacked, header))


def main(args=None):
    rclpy.init(args=args)
    node = GrainAnalyzer()
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
