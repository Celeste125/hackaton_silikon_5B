"""Run a landmark model on the camera and publish boxes, not pictures.

This node is deliberately the only place a trained model is loaded, and it is
meant to run on whichever machine the camera is plugged into -- the Jetson. It
publishes a few hundred bytes of JSON per frame: the boxes, the resolved
intrinsics, and, when the camera has depth, the range already sampled out of
the depth image. Nothing downstream needs an image topic, so no frames cross
the arena WiFi and no frames leave the robot at all.

The model is a launch argument, not a code change:

    # Nothing trained yet -- drive the whole chain with a fixed box:
    ros2 launch perception landmark_detector.launch.py backend:=stub \\
        stub_detections:='[{"class":"food_dispenser","box":[0.4,0.4,0.6,0.7]}]'

    # The teammate's weights, once they exist:
    ros2 launch perception landmark_detector.launch.py \\
        backend:=ultralytics model_path:=~/models/imdl_landmarks.pt

    # The same weights exported to TensorRT on the Jetson itself:
    ros2 launch perception landmark_detector.launch.py \\
        backend:=ultralytics model_path:=~/models/imdl_landmarks.engine

Which of the model's class names counts as which landmark lives in
config/landmarks.yaml, so a model trained with the labels 'feeder' and 'pen'
needs one line of YAML rather than a rebuild.

The node starts and keeps reporting even when the model fails to load, so it
doubles as a camera check and so a missing model never silently stops the
exploration run it is attached to.
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
import yaml

from perception.landmark_backends import available_backends, create_backend
from perception.landmark_detection import (
    LandmarkDetection,
    build_payload,
    canonical_class,
    class_lookup,
    color_image_to_bgr,
    depth_image_to_metres,
    depth_in_box,
    load_landmark_specs,
    normalise_box,
    resolve_intrinsics,
)


class LandmarkDetectorNode(Node):

    def __init__(self):
        super().__init__('landmark_detector')

        self.declare_parameter('backend', 'stub')
        self.declare_parameter('model_path', '')
        self.declare_parameter('landmark_config', '')
        self.declare_parameter('image_topic', '/webcam/image_raw')
        self.declare_parameter('camera_info_topic', '/webcam/camera_info')
        # Only an aligned-to-colour depth image is usable: a raw depth frame
        # has a different projection, so a box from the colour image would
        # sample the wrong pixels. Empty means colour only, which is the
        # normal case for a plain webcam.
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('detections_topic', '/landmarks/detections')
        self.declare_parameter('status_topic', '/landmarks/detector_status')
        self.declare_parameter('debug_image_topic', '/landmarks/debug_image')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_jpeg_quality', 70)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.60)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', '0')
        self.declare_parameter('task', 'detect')
        self.declare_parameter('half', False)
        self.declare_parameter('max_inference_rate_hz', 5.0)
        self.declare_parameter('fallback_hfov_deg', 70.4)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('box_shrink', 0.5)
        self.declare_parameter('min_depth_pixels', 20)
        self.declare_parameter('class_names_path', '')
        self.declare_parameter('onnx_providers', [''])
        self.declare_parameter('stub_detections', '')
        self.declare_parameter('stub_landmark', '')
        self.declare_parameter('stub_box_fraction', 0.2)
        self.declare_parameter('claude_model', 'claude-opus-5-5')
        self.declare_parameter('claude_timeout_sec', 30.0)

        self._rate_hz = float(self.get_parameter('max_inference_rate_hz').value)
        if self._rate_hz <= 0.0:
            raise ValueError('max_inference_rate_hz must be positive')
        self._fallback_hfov = float(self.get_parameter('fallback_hfov_deg').value)

        self._specs = self._load_specs()
        self._by_class = class_lookup(self._specs)

        self._color: Image | None = None
        self._depth: Image | None = None
        self._info: CameraInfo | None = None
        self._lock = threading.Lock()
        self._last_stamp: tuple[int, int] | None = None
        self._frames = 0
        self._published = 0
        self._last_error = ''
        self._unmapped_classes: set[str] = set()

        self._backend = None
        self._backend_error = ''
        self._load_backend()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        # The detections must not be dropped: each one is a view the fusion
        # will never see again, and they are tiny.
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._on_color, sensor_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_info, sensor_qos)
        depth_topic = str(self.get_parameter('depth_topic').value)
        if depth_topic:
            self.create_subscription(
                Image, depth_topic, self._on_depth, sensor_qos)

        self._detections_pub = self.create_publisher(
            String, self.get_parameter('detections_topic').value, reliable_qos)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 1)
        self._debug_pub = self.create_publisher(
            CompressedImage,
            self.get_parameter('debug_image_topic').value + '/compressed',
            sensor_qos)

        self.create_timer(1.0 / self._rate_hz, self._tick)
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'landmark_detector ready: backend='
            f'{self.get_parameter("backend").value}, '
            f'landmarks={sorted(self._specs)}, rate={self._rate_hz:.1f} Hz, '
            f'depth={depth_topic or "none (bearing only)"}')

    # ----------------------------------------------------------- startup

    def _load_specs(self) -> dict:
        configured = str(self.get_parameter('landmark_config').value)
        if configured:
            path = Path(configured).expanduser()
        else:
            path = Path(get_package_share_directory('perception')) / \
                'config' / 'landmarks.yaml'
        if not path.is_file():
            raise FileNotFoundError(f'no landmark config at {path}')
        specs = load_landmark_specs(
            yaml.safe_load(path.read_text(encoding='utf-8')))
        self.get_logger().info(f'landmark config from {path}')
        return specs

    def _load_backend(self) -> None:
        """Build the backend, or record why not and carry on reporting.

        A model that will not load is a configuration problem the operator has
        to see, and taking the node down for it would also take down the
        camera diagnostics that tell them whether the camera is even alive.
        """
        name = str(self.get_parameter('backend').value)
        providers = [
            p for p in self.get_parameter('onnx_providers').value if p]
        options = {
            'model_path': str(self.get_parameter('model_path').value),
            'confidence_threshold': float(
                self.get_parameter('confidence_threshold').value),
            'iou_threshold': float(self.get_parameter('iou_threshold').value),
            'image_size': int(self.get_parameter('image_size').value),
            'device': self._device(),
            'task': str(self.get_parameter('task').value),
            'half': bool(self.get_parameter('half').value),
            'onnx_providers': providers,
            'class_names_path': str(
                self.get_parameter('class_names_path').value),
            'stub_detections': str(
                self.get_parameter('stub_detections').value),
            'stub_landmark': str(self.get_parameter('stub_landmark').value),
            'stub_box_fraction': float(
                self.get_parameter('stub_box_fraction').value),
            'claude_model': str(self.get_parameter('claude_model').value),
            'claude_timeout_sec': float(
                self.get_parameter('claude_timeout_sec').value),
            'landmark_names': sorted(self._specs),
            'landmark_descriptions': {
                spec.name: spec.note for spec in self._specs.values()},
        }
        try:
            backend = create_backend(name, options)
            backend.load()
        except Exception as exc:  # noqa: BLE001 - report, never crash
            self._backend_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(
                f'backend {name!r} did not load ({self._backend_error}). The '
                f'camera is still reported. Backends: '
                f'{", ".join(available_backends())}')
            return
        self._backend = backend
        self._warn_about_unmatched_classes(backend)

    def _warn_about_unmatched_classes(self, backend) -> None:
        """Say so at startup if the model's labels match no landmark.

        This is the failure that otherwise costs an afternoon: the model runs,
        the detector publishes nothing, and everything looks healthy.
        """
        names = {canonical_class(n) for n in backend.class_names}
        if not names:
            return
        matched = names & set(self._by_class)
        if not matched:
            self.get_logger().error(
                f'none of the model classes {sorted(names)} appear in the '
                f'landmark config, which maps {sorted(self._by_class)}. Every '
                'detection would be discarded -- fix model_classes in '
                'landmarks.yaml.')
        elif names - matched:
            self.get_logger().info(
                f'ignoring model classes {sorted(names - matched)}, which no '
                'landmark claims')

    def _device(self):
        value = str(self.get_parameter('device').value).strip()
        return int(value) if value.isdigit() else (value or 'cpu')

    # ------------------------------------------------------------ inputs

    def _on_color(self, msg: Image):
        with self._lock:
            self._color = msg

    def _on_depth(self, msg: Image):
        with self._lock:
            self._depth = msg

    def _on_info(self, msg: CameraInfo):
        self._info = msg

    # ---------------------------------------------------------- inference

    def _tick(self):
        with self._lock:
            color, depth = self._color, self._depth
        if color is None or self._backend is None:
            return
        # Skip a frame already run. A source that leaves the stamp at zero --
        # a replay, or a misconfigured driver -- would otherwise be processed
        # once and then look like a hung detector, so that case runs every
        # tick instead. The fusion needs a real stamp for its TF lookup and
        # will say so.
        stamp = (color.header.stamp.sec, color.header.stamp.nanosec)
        if stamp == self._last_stamp and stamp != (0, 0):
            return
        self._last_stamp = stamp

        try:
            self._process(color, depth)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(f'frame failed: {self._last_error}')

    def _process(self, color: Image, depth: Image | None) -> None:
        image = color_image_to_bgr(
            color.data, color.height, color.width, color.step, color.encoding)

        started = time.perf_counter()
        raw = self._backend.detect(image)
        inference_ms = (time.perf_counter() - started) * 1000.0
        self._frames += 1

        depth_metres = self._depth_metres(depth, color)
        detections = [
            detection for detection in (
                self._attribute(entry, color, depth_metres) for entry in raw)
            if detection is not None
        ]
        # Empty frames are also observations: control must distinguish a lost
        # target from a stalled camera, model, or network connection.

        intrinsics = resolve_intrinsics(
            color.width, color.height,
            self._info.k if self._info is not None else None,
            self._fallback_hfov)
        payload = build_payload(
            stamp_sec=color.header.stamp.sec,
            stamp_nanosec=color.header.stamp.nanosec,
            frame_id=color.header.frame_id,
            width=color.width,
            height=color.height,
            intrinsics=intrinsics,
            detections=detections,
            inference_ms=inference_ms,
            model=self._backend.describe(),
        )
        self._detections_pub.publish(String(data=json.dumps(payload)))
        self._published += 1
        if self.get_parameter('publish_debug_image').value:
            self._publish_debug(image, detections)

    def _attribute(self, raw, color: Image, depth_metres):
        """Turn one raw box into a LandmarkDetection, or drop it."""
        spec = self._by_class.get(canonical_class(raw.model_class))
        if spec is None:
            if raw.model_class not in self._unmapped_classes:
                self._unmapped_classes.add(raw.model_class)
                self.get_logger().info(
                    f'class {raw.model_class!r} maps to no landmark; ignoring')
            return None
        if not spec.accepts(raw.confidence):
            return None
        try:
            box = normalise_box(raw.box_xyxy_px, color.width, color.height)
        except ValueError as exc:
            self.get_logger().warn(f'{spec.name}: unusable box ({exc})')
            return None

        range_m, range_sigma, source = None, None, 'none'
        if depth_metres is not None:
            range_m, range_sigma, pixels = depth_in_box(
                depth_metres, box,
                shrink=float(self.get_parameter('box_shrink').value),
                min_depth_m=float(self.get_parameter('min_depth_m').value),
                max_depth_m=float(self.get_parameter('max_depth_m').value),
                min_pixels=int(self.get_parameter('min_depth_pixels').value))
            if range_m is None:
                self.get_logger().debug(
                    f'{spec.name}: only {pixels} valid depth pixels in the '
                    'box; publishing the bearing alone')
            else:
                source = 'depth'
                # The depth sigma from the patch spread understates the real
                # error, which is dominated by the box centroid sitting on the
                # object's near face rather than its middle. Add the
                # landmark's own range sigma in quadrature.
                range_sigma = float(
                    (range_sigma ** 2 + spec.range_sigma_m ** 2) ** 0.5)

        return LandmarkDetection(
            landmark=spec.name,
            model_class=raw.model_class,
            confidence=float(raw.confidence),
            box_norm=box,
            range_m=range_m,
            range_sigma_m=range_sigma,
            range_source=source,
        )

    def _depth_metres(self, depth: Image | None, color: Image):
        if depth is None:
            return None
        if (depth.width, depth.height) != (color.width, color.height):
            self.get_logger().warn(
                f'depth is {depth.width}x{depth.height} but colour is '
                f'{color.width}x{color.height}: the depth topic must be the '
                'aligned-to-colour one, so the boxes index the same pixels. '
                'Ignoring depth.', once=True)
            return None
        try:
            return depth_image_to_metres(
                depth.data, depth.height, depth.width, depth.step,
                depth.encoding)
        except ValueError as exc:
            self.get_logger().warn(str(exc), once=True)
            return None

    # ----------------------------------------------------------- outputs

    def _publish_debug(self, image, detections) -> None:
        """A JPEG of the boxes, for checking what the model actually saw.

        Off by default: it is the one output that costs real bandwidth, which
        is the thing this node exists to avoid spending.
        """
        try:
            import cv2
        except ImportError:
            self.get_logger().warn('no cv2; cannot draw a debug image', once=True)
            return
        canvas = image.copy()
        height, width = canvas.shape[:2]
        for detection in detections:
            x0, y0, x1, y1 = detection.box_norm
            spec = self._specs[detection.landmark]
            color = tuple(int(c * 255) for c in reversed(spec.color))
            cv2.rectangle(
                canvas, (int(x0 * width), int(y0 * height)),
                (int(x1 * width), int(y1 * height)), color, 2)
            label = f'{detection.landmark} {detection.confidence:.2f}'
            if detection.range_m is not None:
                label += f' {detection.range_m:.2f} m'
            cv2.putText(
                canvas, label, (int(x0 * width), max(int(y0 * height) - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        ok, buffer = cv2.imencode(
            '.jpg', canvas,
            [int(cv2.IMWRITE_JPEG_QUALITY),
             int(self.get_parameter('debug_jpeg_quality').value)])
        if not ok:
            return
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.format = 'jpeg'
        message.data = buffer.tobytes()
        self._debug_pub.publish(message)

    def _report(self):
        calibrated = self._info is not None and self._info.k[0] > 0.0
        state = (
            f"backend={'ready' if self._backend else 'FAILED'} "
            f"color={'yes' if self._color else 'NO'} "
            f"info={'calibrated' if calibrated else 'FOV fallback'} "
            f"depth={'yes' if self._depth else 'no'} "
            f"frames={self._frames} published={self._published}"
        )
        if self._backend_error:
            state += f' error={self._backend_error}'
        elif self._last_error:
            state += f' last_error={self._last_error}'
        self._status_pub.publish(String(data=state))
        if self._color is None:
            self.get_logger().warn(
                f'no camera frames on '
                f'{self.get_parameter("image_topic").value}: {state}')


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDetectorNode()
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
