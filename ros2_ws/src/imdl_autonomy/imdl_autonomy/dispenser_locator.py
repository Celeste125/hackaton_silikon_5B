"""Estimate the food dispenser's position in the map from camera views.

The lidar never sees the dispenser, so its position has to come from the
camera. The split of work matters:

  Claude  decides *which pixels* are the dispenser -- a bounding box, i.e. a
          bearing. It is asked for nothing metric, because a VLM cannot
          measure range and will invent plausible numbers if asked.
  Depth   supplies the range, from the RealSense frame aligned to colour.
  TF      turns that into a map-frame point.

One view lands within roughly 10-20 cm, dominated by the camera extrinsics and
by the mask centroid sitting on the dispenser's front face rather than its
centre. Observing from a few different angles cancels most of the second error,
which is why estimates are fused rather than taken from a single frame.

Trigger a view and watch the result:

    ros2 topic pub --once /dispenser/locate std_msgs/msg/Empty "{}"
    ros2 topic echo /dispenser/pose

Needs ANTHROPIC_API_KEY in the environment. Without it the node still starts
and still reports what the camera is doing, so it doubles as a camera check.
"""

from __future__ import annotations

import base64
import io
import json
import math
import os
import threading

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker

# What Claude must return. additionalProperties false plus required keeps the
# reply parseable without defensive guessing.
_BOX_SCHEMA = {
    'type': 'object',
    'properties': {
        'visible': {
            'type': 'boolean',
            'description': 'True only if the food dispenser is in this image.',
        },
        'box': {
            'type': 'array',
            'description': (
                'Normalised [x0, y0, x1, y1] in 0-1, origin top-left, '
                'tight around the dispenser. All zeros when not visible.'
            ),
            'items': {'type': 'number'},
            'minItems': 4,
            'maxItems': 4,
        },
        'confidence': {'type': 'number'},
        'note': {'type': 'string'},
    },
    'required': ['visible', 'box', 'confidence', 'note'],
    'additionalProperties': False,
}

_PROMPT = (
    'This is the forward camera of a small floor robot exploring an arena.\n'
    'Find the food dispenser: a container or machine that dispenses food, '
    'often a hopper, bin or bottle on a stand.\n\n'
    'Return a tight normalised bounding box around it. Report visible=false '
    'if it is not clearly in frame -- most frames will not contain it, and a '
    'guessed box is worse than none, because the box is used to sample a '
    'depth measurement. Prefer a box on the object body, not its shadow or '
    'its reflection.'
)


class DispenserLocator(Node):
    """Fuse Claude-identified bounding boxes with depth into a map pose."""

    def __init__(self):
        super().__init__('dispenser_locator')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('model', 'claude-opus-5-5')
        # The RealSense is configured for 0.05-3.0 m; beyond ~2.5 m its noise
        # grows faster than the box error, so a far view adds little.
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('min_confidence', 0.4)
        # Sample depth from the middle of the box only: the edges straddle
        # whatever is behind the dispenser.
        self.declare_parameter('box_shrink', 0.5)
        self.declare_parameter('min_depth_pixels', 20)
        self.declare_parameter('auto_interval_sec', 0.0)
        self.declare_parameter('jpeg_quality', 80)

        self._map_frame = self.get_parameter('map_frame').value
        self._model = self.get_parameter('model').value

        self._color: Image | None = None
        self._depth: Image | None = None
        self._info: CameraInfo | None = None
        self._busy = False
        self._observations: list[tuple[float, float, float]] = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            Image, self.get_parameter('color_topic').value,
            self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self._on_info, qos_profile_sensor_data)
        self.create_subscription(Empty, '/dispenser/locate', self._on_trigger, 1)

        self._pose_pub = self.create_publisher(PoseStamped, '/dispenser/pose', 1)
        self._marker_pub = self.create_publisher(Marker, '/dispenser/marker', 1)
        self._status_pub = self.create_publisher(String, '/dispenser/status', 1)

        self._client = None
        self._client_error = None
        try:
            import anthropic
            if not os.environ.get('ANTHROPIC_API_KEY'):
                raise RuntimeError('ANTHROPIC_API_KEY is not set')
            # The arena WiFi is congested; fail fast rather than stall a view.
            self._client = anthropic.Anthropic(timeout=30.0, max_retries=1)
        except Exception as exc:  # noqa: BLE001 - report, never crash the node
            self._client_error = str(exc)

        interval = float(self.get_parameter('auto_interval_sec').value)
        if interval > 0.0:
            self.create_timer(interval, lambda: self._on_trigger(Empty()))
        self.create_timer(5.0, self._report_inputs)

        self.get_logger().info(
            f'dispenser_locator ready, model={self._model}, '
            f'map_frame={self._map_frame}'
        )
        if self._client is None:
            self.get_logger().warn(
                f'No Claude client ({self._client_error}). Camera reporting '
                'still works; set ANTHROPIC_API_KEY to locate the dispenser.'
            )

    # ---------------------------------------------------------------- inputs

    def _on_color(self, msg: Image):
        self._color = msg

    def _on_depth(self, msg: Image):
        self._depth = msg

    def _on_info(self, msg: CameraInfo):
        self._info = msg

    def _report_inputs(self):
        """Say what is and is not arriving, so a dead topic is obvious."""
        state = (
            f"color={'yes' if self._color else 'NO'} "
            f"depth={'yes' if self._depth else 'NO'} "
            f"info={'yes' if self._info else 'NO'} "
            f"views={len(self._observations)}"
        )
        self._status_pub.publish(String(data=state))
        if not (self._color and self._depth and self._info):
            self.get_logger().warn(
                f'waiting on camera: {state} '
                '(run: ros2 launch perception camera.launch.py)'
            )

    # --------------------------------------------------------------- trigger

    def _on_trigger(self, _msg: Empty):
        if self._busy:
            self.get_logger().info('a view is already in flight, skipping')
            return
        if not (self._color and self._depth and self._info):
            self.get_logger().warn('no camera data yet; cannot take a view')
            return
        if self._client is None:
            self.get_logger().error(f'no Claude client: {self._client_error}')
            return
        # The API call takes seconds. Off the executor thread, or callbacks
        # (including TF) stall for its duration.
        self._busy = True
        threading.Thread(target=self._locate_once, daemon=True).start()

    def _locate_once(self):
        try:
            self._do_locate(self._color, self._depth, self._info)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'view failed: {exc}')
        finally:
            self._busy = False

    # ---------------------------------------------------------------- vision

    def _do_locate(self, color: Image, depth: Image, info: CameraInfo):
        answer = self._ask_claude(color)
        if answer is None:
            return
        if not answer.get('visible'):
            self.get_logger().info(
                f"dispenser not in view ({answer.get('note', '')[:80]})")
            return
        confidence = float(answer.get('confidence', 0.0))
        if confidence < float(self.get_parameter('min_confidence').value):
            self.get_logger().info(f'box rejected, confidence {confidence:.2f}')
            return

        depth_m, u, v = self._depth_in_box(depth, answer['box'])
        if depth_m is None:
            self.get_logger().warn(
                'box found but no usable depth inside it -- too far, too '
                'shiny, or outside the 0.15-3.0 m window')
            return

        point = self._deproject(info, u, v, depth_m)
        mapped = self._to_map(point, color.header)
        if mapped is None:
            return

        self._observations.append(mapped)
        self._publish(mapped, depth_m, confidence)

    def _ask_claude(self, color: Image) -> dict | None:
        jpeg = self._to_jpeg(color)
        if jpeg is None:
            return None
        response = self._client.messages.create(
            model=self._model,
            max_tokens=1024,
            # Picking a box is not a reasoning-heavy task, and this is in the
            # robot's loop, so buy latency rather than depth here.
            output_config={
                'effort': 'low',
                'format': {'type': 'json_schema', 'schema': _BOX_SCHEMA},
            },
            messages=[{
                'role': 'user',
                'content': [
                    {
                        'type': 'image',
                        'source': {
                            'type': 'base64',
                            'media_type': 'image/jpeg',
                            'data': base64.b64encode(jpeg).decode('ascii'),
                        },
                    },
                    {'type': 'text', 'text': _PROMPT},
                ],
            }],
        )
        if response.stop_reason == 'refusal':
            self.get_logger().error('request declined by the safety classifier')
            return None
        text = next((b.text for b in response.content if b.type == 'text'), None)
        if not text:
            self.get_logger().error('empty reply')
            return None
        return json.loads(text)

    def _to_jpeg(self, msg: Image) -> bytes | None:
        """Encode without cv_bridge, which the container does not carry."""
        try:
            from PIL import Image as PILImage
        except ImportError:
            self.get_logger().error('python3-pil missing; cannot encode frames')
            return None
        channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
        if channels is None:
            self.get_logger().error(f'unsupported colour encoding {msg.encoding}')
            return None
        array = np.frombuffer(msg.data, dtype=np.uint8)
        array = array.reshape(msg.height, msg.step // channels, channels)
        array = array[:, :msg.width, :]
        if msg.encoding == 'bgr8':
            array = array[:, :, ::-1]
        mode = 'L' if channels == 1 else 'RGB'
        buffer = io.BytesIO()
        PILImage.fromarray(array.squeeze(), mode=mode).save(
            buffer, format='JPEG',
            quality=int(self.get_parameter('jpeg_quality').value))
        return buffer.getvalue()

    # -------------------------------------------------------------- geometry

    def _depth_in_box(self, depth: Image, box) -> tuple[float | None, int, int]:
        """Median depth over the middle of the box, in metres."""
        if depth.encoding == '16UC1':
            raw = np.frombuffer(depth.data, dtype=np.uint16).astype(np.float32)
            raw *= 0.001  # RealSense 16UC1 is millimetres
        elif depth.encoding == '32FC1':
            raw = np.frombuffer(depth.data, dtype=np.float32).copy()
        else:
            self.get_logger().error(f'unsupported depth encoding {depth.encoding}')
            return None, 0, 0
        grid = raw.reshape(depth.height, -1)[:, :depth.width]

        x0, y0, x1, y1 = (float(v) for v in box)
        x0, x1 = sorted((min(max(x0, 0.0), 1.0), min(max(x1, 0.0), 1.0)))
        y0, y1 = sorted((min(max(y0, 0.0), 1.0), min(max(y1, 0.0), 1.0)))
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        shrink = float(self.get_parameter('box_shrink').value)
        half_w = (x1 - x0) * shrink / 2.0
        half_h = (y1 - y0) * shrink / 2.0

        u0 = int((cx - half_w) * depth.width)
        u1 = max(int((cx + half_w) * depth.width), u0 + 1)
        v0 = int((cy - half_h) * depth.height)
        v1 = max(int((cy + half_h) * depth.height), v0 + 1)

        patch = grid[v0:v1, u0:u1]
        lo = float(self.get_parameter('min_depth_m').value)
        hi = float(self.get_parameter('max_depth_m').value)
        valid = patch[(patch > lo) & (patch < hi) & np.isfinite(patch)]
        if valid.size < int(self.get_parameter('min_depth_pixels').value):
            return None, 0, 0
        return float(np.median(valid)), int(cx * depth.width), int(cy * depth.height)

    def _deproject(self, info: CameraInfo, u: int, v: int, z: float) -> PointStamped:
        """Pixel plus depth to a point in the camera's optical frame."""
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        point = PointStamped()
        point.header = info.header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point

    def _to_map(self, point: PointStamped, stamp_header):
        # Use the image's own stamp: the robot may have moved since.
        point.header.stamp = stamp_header.stamp
        try:
            in_map = self._tf_buffer.transform(
                point, self._map_frame, timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as exc:
            self.get_logger().error(
                f'no transform {point.header.frame_id} -> {self._map_frame}: {exc}')
            return None
        return (in_map.point.x, in_map.point.y, in_map.point.z)

    # --------------------------------------------------------------- outputs

    def _publish(self, latest, depth_m: float, confidence: float):
        array = np.array(self._observations)
        mean = array.mean(axis=0)
        spread = (
            float(np.linalg.norm(array[:, :2].std(axis=0)))
            if len(array) > 1 else float('nan')
        )

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = mean
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = 'dispenser'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.1, 0.9, 0.2, 0.85)
        self._marker_pub.publish(marker)

        self.get_logger().info(
            f'dispenser view {len(self._observations)}: '
            f'this one ({latest[0]:.2f}, {latest[1]:.2f}) at {depth_m:.2f} m, '
            f'confidence {confidence:.2f} | fused ({mean[0]:.2f}, {mean[1]:.2f})'
            + (f', spread {spread:.3f} m' if not math.isnan(spread) else
               ' -- take another view from a different angle')
        )


def main(args=None):
    rclpy.init(args=args)
    node = DispenserLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
