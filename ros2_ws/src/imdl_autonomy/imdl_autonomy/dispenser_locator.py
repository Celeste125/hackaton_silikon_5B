"""Estimate the food dispenser's position in the map from camera views.

The lidar never sees the dispenser, so its position has to come from the
camera. The split of work matters:

  Claude  decides *which pixels* are the dispenser -- a bounding box, i.e. a
          bearing. It is asked for nothing metric, because a VLM cannot
          measure range and will invent plausible numbers if asked.
  Geometry turns that bearing into a range, by whichever of three means is
          available this view.
  TF      turns the result into a map-frame point.

The three range sources, best first:

  depth         An RGB-D camera aligned to colour: sample the depth inside the
                box. ~10-20 cm, and works from one view.
  triangulate   Two or more bearings from different robot poses intersect at
                the object. No depth hardware at all, so this is the path for
                a plain webcam like a C920. Needs the robot to have moved
                between views -- near-parallel bearings are rejected rather
                than intersected badly.
  wall ray cast March the bearing across the SLAM occupancy grid until it hits
                an occupied cell. The lidar never sees the dispenser, but it
                does see the wall behind it, so for anything standing against
                a wall this is good from a single view. It reads the dispenser
                as being *at* the wall, so it is wrong by the standoff.

Whichever is used is named in the log and on /dispenser/status, because the
accuracy differs a lot between them.

Error is dominated by the camera extrinsics and, with depth, by the box
centroid sitting on the dispenser's front face. Viewing from several angles
cancels most of the second, which is why estimates are fused.

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
from nav_msgs.msg import OccupancyGrid
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
        self.declare_parameter('map_topic', '/map')
        # v4l2_camera publishes a zeroed K until calibrated. The C920s is
        # ~70.4 deg horizontal at 4:3; good to a couple of degrees, which is
        # several centimetres of cross-range error at 2 m. Calibrate for better.
        self.declare_parameter('fallback_hfov_deg', 70.4)
        # Do not intersect bearings the robot barely moved between: the
        # intersection is then almost unconstrained along the view direction.
        self.declare_parameter('min_triangulation_angle_deg', 12.0)
        self.declare_parameter('raycast_max_range_m', 6.0)
        # Skip the robot's own footprint, and treat unknown cells as empty:
        # unexplored is not a wall.
        self.declare_parameter('raycast_skip_m', 0.35)
        self.declare_parameter('occupied_threshold', 65)

        self._map_frame = self.get_parameter('map_frame').value
        self._model = self.get_parameter('model').value

        self._color: Image | None = None
        self._depth: Image | None = None
        self._info: CameraInfo | None = None
        self._map: OccupancyGrid | None = None
        self._busy = False
        self._observations: list[tuple[float, float, float]] = []
        self._methods: list[str] = []
        # (origin_xy, unit_direction_xy) in the map frame, one per view that
        # produced a usable bearing, kept for triangulation.
        self._rays: list[tuple[np.ndarray, np.ndarray]] = []

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
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, 1)
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

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _report_inputs(self):
        """Say what is and is not arriving, so a dead topic is obvious."""
        state = (
            f"color={'yes' if self._color else 'NO'} "
            f"depth={'yes' if self._depth else 'no (bearing-only)'} "
            f"info={'yes' if self._info else 'NO'} "
            f"map={'yes' if self._map else 'no'} "
            f"range_by={self._available_method()} "
            f"views={len(self._observations)}"
        )
        self._status_pub.publish(String(data=state))
        # Depth is optional now; colour is not, and neither is a map or a
        # second viewpoint if there is no depth.
        if not self._color:
            self.get_logger().warn(
                f'waiting on camera: {state} (run: ros2 launch perception '
                'webcam.launch.py, or camera.launch.py for a RealSense)'
            )
        elif self._available_method() == 'nothing':
            self.get_logger().warn(
                f'colour only, and no way to get range yet: {state} -- needs '
                'depth, or /map for a wall cast, or a second view from '
                'elsewhere to triangulate'
            )

    def _available_method(self) -> str:
        """Which range source could serve the next view."""
        if self._depth is not None:
            return 'depth'
        if len(self._rays) >= 1:
            return 'triangulate-or-wall'
        if self._map is not None:
            return 'wall'
        return 'nothing'

    # --------------------------------------------------------------- trigger

    def _on_trigger(self, _msg: Empty):
        if self._busy:
            self.get_logger().info('a view is already in flight, skipping')
            return
        if self._color is None:
            self.get_logger().warn('no camera image yet; cannot take a view')
            return
        if self._depth is None and self._map is None and not self._rays:
            self.get_logger().warn(
                'image only: no depth, no map to cast at, and no earlier '
                'bearing to triangulate against -- taking the view anyway so '
                'the bearing is banked for the next one')
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

        fx, fy, cx, cy = self._intrinsics(info, color)
        u, v = self._box_centre(answer['box'], color.width, color.height)

        # 1. Depth, when the camera has it: one view, best accuracy.
        if depth is not None:
            depth_m, du, dv = self._depth_in_box(depth, answer['box'])
            if depth_m is not None:
                point = self._deproject_xy(fx, fy, cx, cy, du, dv, depth_m,
                                           info, color)
                mapped = self._to_map(point, color.header)
                if mapped is not None:
                    self._record(mapped, 'depth', confidence, depth_m)
                    return
            self.get_logger().warn(
                'depth present but unusable in the box; falling back to '
                'bearing-only geometry')

        # Everything below needs the bearing as a ray in the map.
        ray = self._bearing_ray(fx, fy, cx, cy, u, v, color, info)
        if ray is None:
            return
        origin, direction = ray

        # 2. Triangulate against an earlier bearing from a different pose.
        triangulated = self._triangulate(origin, direction)
        self._rays.append((origin, direction))
        if triangulated is not None:
            self._record((*triangulated, 0.0), 'triangulate', confidence, None)
            return

        # 3. Otherwise cast the bearing at the wall behind it.
        hit = self._raycast(origin, direction)
        if hit is not None:
            self._record((*hit, 0.0), 'wall', confidence, None)
            return

        self.get_logger().warn(
            'bearing taken but no range: the ray hit no mapped wall within '
            'range, and there is no second viewpoint yet. Drive somewhere '
            'else and take another view.')

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

    def _intrinsics(self, info: CameraInfo | None, color: Image):
        """fx, fy, cx, cy, from camera_info when it is calibrated.

        v4l2_camera publishes a zeroed K until a calibration file is given, and
        a zero focal length would send every bearing straight down the optical
        axis. Fall back to the declared horizontal field of view, which is
        good to a couple of degrees.
        """
        if info is not None and info.k[0] > 0.0:
            return info.k[0], info.k[4], info.k[2], info.k[5]
        hfov = math.radians(float(self.get_parameter('fallback_hfov_deg').value))
        focal = (color.width / 2.0) / math.tan(hfov / 2.0)
        self.get_logger().warn(
            f'camera_info is uncalibrated; assuming {math.degrees(hfov):.1f} '
            'deg HFOV. Run camera_calibration for better bearings.',
            once=True)
        return focal, focal, color.width / 2.0, color.height / 2.0

    def _box_centre(self, box, width: int, height: int) -> tuple[int, int]:
        x0, y0, x1, y1 = (float(value) for value in box)
        return (int((x0 + x1) / 2.0 * width), int((y0 + y1) / 2.0 * height))

    def _deproject_xy(self, fx, fy, cx, cy, u, v, z, info, color) -> PointStamped:
        """Same geometry as _deproject, but with intrinsics already resolved."""
        point = PointStamped()
        point.header = info.header if info is not None else color.header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point

    def _bearing_ray(self, fx, fy, cx, cy, u, v, color, info):
        """The sight line as (origin, unit direction) in the map, flattened.

        Two points are transformed rather than the direction rotated by hand:
        TF then owns the whole optical-frame convention, and a wrong sign in
        the extrinsics shows up as a wrong answer rather than a silent one.
        """
        frame = info.header.frame_id if info is not None else color.header.frame_id
        if not frame:
            self.get_logger().error('no camera frame_id; cannot place a bearing')
            return None
        direction = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        direction /= np.linalg.norm(direction)

        points = []
        for scale in (0.0, 1.0):
            point = PointStamped()
            point.header.frame_id = frame
            point.header.stamp = color.header.stamp
            point.point.x = float(direction[0] * scale)
            point.point.y = float(direction[1] * scale)
            point.point.z = float(direction[2] * scale)
            mapped = self._to_map(point, color.header)
            if mapped is None:
                return None
            points.append(np.array(mapped[:2]))

        origin, along = points
        flat = along - origin
        norm = float(np.linalg.norm(flat))
        if norm < 1e-6:
            self.get_logger().error(
                'the sight line is vertical in the map frame; check the '
                'camera pitch in the extrinsics')
            return None
        return origin, flat / norm

    def _triangulate(self, origin: np.ndarray, direction: np.ndarray):
        """Least-squares intersection with the bearings taken so far.

        Each ray contributes the projection that measures perpendicular
        distance from it; summing them and solving puts the point closest to
        every sight line at once.
        """
        min_angle = math.radians(
            float(self.get_parameter('min_triangulation_angle_deg').value))
        usable = [
            (o, n) for o, n in self._rays
            if abs(math.atan2(float(np.cross(n, direction)),
                              float(np.dot(n, direction)))) >= min_angle
        ]
        if not usable:
            return None
        rays = usable + [(origin, direction)]
        matrix = np.zeros((2, 2))
        vector = np.zeros(2)
        for o, n in rays:
            projector = np.eye(2) - np.outer(n, n)
            matrix += projector
            vector += projector @ o
        if abs(np.linalg.det(matrix)) < 1e-9:
            return None
        point = np.linalg.solve(matrix, vector)
        return float(point[0]), float(point[1])

    def _raycast(self, origin: np.ndarray, direction: np.ndarray):
        """March the bearing over the occupancy grid to the first wall."""
        grid = self._map
        if grid is None:
            return None
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width)
        occupied = int(self.get_parameter('occupied_threshold').value)
        distance = float(self.get_parameter('raycast_skip_m').value)
        limit = float(self.get_parameter('raycast_max_range_m').value)
        step = max(resolution * 0.5, 1e-3)

        while distance <= limit:
            x = origin[0] + direction[0] * distance
            y = origin[1] + direction[1] * distance
            column = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
                return None
            value = data[row, column]
            # -1 is unexplored, not a wall: casting through it is correct.
            if value >= occupied:
                return float(x), float(y)
            distance += step
        return None

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

    def _record(self, latest, method: str, confidence: float,
                depth_m: float | None):
        """Keep the view and republish the fused estimate.

        Wall casts are kept separate in the log because they are biased by the
        dispenser's standoff from the wall, so a run that mixes them with
        depth or triangulated views has a floor on how good the mean can be.
        """
        self._observations.append(latest)
        self._methods.append(method)
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

        range_note = f'at {depth_m:.2f} m ' if depth_m is not None else ''
        self.get_logger().info(
            f'dispenser view {len(self._observations)} by {method}: '
            f'this one ({latest[0]:.2f}, {latest[1]:.2f}) {range_note}'
            f'confidence {confidence:.2f} | fused ({mean[0]:.2f}, {mean[1]:.2f})'
            + (f', spread {spread:.3f} m' if not math.isnan(spread) else
               ' -- take another view from somewhere else')
            + (' | NOTE: wall casts read the dispenser as being at the wall'
               if 'wall' in self._methods else '')
        )
        self._status_pub.publish(String(data=(
            f'fused=({mean[0]:.3f},{mean[1]:.3f}) views={len(self._observations)} '
            f'methods={",".join(sorted(set(self._methods)))} '
            f'spread={spread:.3f}' if not math.isnan(spread) else
            f'fused=({mean[0]:.3f},{mean[1]:.3f}) views=1 methods={method}'
        )))


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
