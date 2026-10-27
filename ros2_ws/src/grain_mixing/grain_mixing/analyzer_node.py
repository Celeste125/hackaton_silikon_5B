import time
from dataclasses import fields, replace

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .pipeline import GRAIN_TYPES, GrainPipeline, PipelineParams, add_legend

# newest frame only; drop old ones if processing is slower than the camera
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
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
    msg.data = np.ascontiguousarray(bgr).tobytes()
    return msg


def plain(value):
    """ROS array parameters arrive as array.array; the pipeline wants plain lists."""
    if hasattr(value, '__iter__') and not isinstance(value, str):
        return list(value)
    return value


class GrainAnalyzer(Node):
    def __init__(self):
        super().__init__('grain_analyzer')

        # every PipelineParams field becomes a ROS parameter with the same name
        defaults = PipelineParams()
        values = {}
        for f in fields(defaults):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            values[f.name] = plain(self.get_parameter(f.name).value)
        self.params = PipelineParams(**values)
        self.pipeline = GrainPipeline(self.params)

        self.declare_parameter('process_every', 1)      # 2 = analyse every second frame
        self.process_every = max(1, self.get_parameter('process_every').value)
        self.add_on_set_parameters_callback(self._on_params)

        self.sub = self.create_subscription(Image, 'image_raw', self._on_image, SENSOR_QOS)
        self.pub_overlay = self.create_publisher(Image, '~/overlay', 1)
        self.pub_stacked = self.create_publisher(Image, '~/stacked', 1)
        self.pub_coverage = self.create_publisher(Float32, '~/coverage', 10)
        self.pub_homogeneity = self.create_publisher(Float32, '~/homogeneity', 10)
        self.pub_area = {g: self.create_publisher(Float32, f'~/area/{g}', 10) for g in GRAIN_TYPES}

        self.frame_count = 0
        self.last_time = time.perf_counter()
        self.get_logger().info('grain_analyzer ready, waiting for frames on "image_raw"')

    def _on_params(self, params):
        changes = {p.name: plain(p.value) for p in params}
        if 'process_every' in changes:
            self.process_every = max(1, int(changes.pop('process_every')))
        changes = {k: v for k, v in changes.items() if hasattr(self.params, k)}
        if not 0 < changes.get('scale', self.params.scale) <= 1:
            return SetParametersResult(successful=False, reason='scale must be in (0, 1]')
        self.params = replace(self.params, **changes)
        self.pipeline = GrainPipeline(self.params)      # rebuilds kernels, Hough settings and the roi
        return SetParametersResult(successful=True)

    @staticmethod
    def _publish_float(pub, value):
        pub.publish(Float32(data=float(value)))

    def _on_image(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every:
            return
        try:
            result = self.pipeline.process(image_to_bgr(msg))
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        now = time.perf_counter()
        fps = 1 / max(now - self.last_time, 1e-6)
        self.last_time = now

        # as in the standalone program, the legend is drawn on the raw frame
        add_legend(result.raw, fps, result.total_area, result.areas, result.homogeneity)

        self._publish_float(self.pub_coverage, result.total_area)
        self._publish_float(self.pub_homogeneity, result.homogeneity)
        for grain, area in result.areas.items():
            self._publish_float(self.pub_area[grain], area)

        # only encode images somebody is listening to
        if self.pub_overlay.get_subscription_count():
            self.pub_overlay.publish(bgr_to_image(result.overlay, msg.header))
        if self.pub_stacked.get_subscription_count():
            stacked = np.vstack([result.raw, result.overlay])
            self.pub_stacked.publish(bgr_to_image(stacked, msg.header))


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
