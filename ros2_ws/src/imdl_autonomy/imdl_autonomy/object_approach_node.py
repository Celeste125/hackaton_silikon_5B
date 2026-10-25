"""Network detection consumer and command gate upstream of Collision Monitor."""

from dataclasses import fields
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from perception.landmark_detection import parse_payload
from imdl_autonomy.object_approach import ApproachController, Config, Observation, Pose
from imdl_autonomy.object_positions import FusionConfig, ObjectPositionStore, project_box


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid orientation')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def pose_from(position, orientation):
    matrix = rotation(orientation)
    if not math.isfinite(position.x) or not math.isfinite(position.y):
        raise ValueError('invalid odometry')
    return Pose(position.x, position.y, math.atan2(matrix[1, 0], matrix[0, 0]))


class ObjectApproachNode(Node):
    def __init__(self):
        super().__init__('object_approach')
        for field in fields(Config):
            self.declare_parameter(field.name, field.default)
        self.core = ApproachController(Config(**{
            f.name: self.get_parameter(f.name).value for f in fields(Config)}))
        for field in fields(FusionConfig):
            self.declare_parameter('fusion_' + field.name, field.default)
        fusion_config = FusionConfig(**{
            f.name: self.get_parameter('fusion_' + f.name).value for f in fields(FusionConfig)})
        defaults = {
            'detections_topic': '/landmarks/detections',
            'odom_topic': '/Team2/odom', 'scan_topic': '/scan',
            'nav_velocity_topic': '/cmd_vel_smoothed',
            'output_velocity_topic': '/approach/cmd_vel',
            'base_frame': 'base_link', 'odom_frame': 'odom', 'map_frame': 'map',
            'target_landmarks': ['food_dispenser', 'cow_enclosure'],
            'sensor_timeout': 0.5,
            'positions_file': '', 'map_id': '',
            # A stale detector normally stops the rover: with no frames there
            # is nothing to notice a target with, so driving on is driving
            # blind. Set false to bring up SLAM and Nav2 and drive to a goal
            # without the camera machine in the loop -- no measurement can
            # happen then, which is why it is not the default.
            'require_detector_for_navigation': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        self.positions = ObjectPositionStore(
            self.options['positions_file'], self.options['map_frame'],
            self.options['odom_frame'], self.options['map_id'], fusion_config)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._pose = None
        self._stationary = False
        self._odom_at = self._scan_at = self._ack_at = self._nav_at = -math.inf
        self._frame_at = -math.inf
        self._paused = False
        self._nav = Twist()
        self._pending = None
        self._camera_signature = None
        self._last_state = None
        self._measurement_frame = None
        self._measurement_saved = False
        self._settled_since = None
        self._resume_since = math.inf
        self._output = self.create_publisher(
            Twist, self.options['output_velocity_topic'], 1)
        self._hold = self.create_publisher(
            Bool, '/frontier_explorer/hold',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status = self.create_publisher(String, '/approach/status', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._positions_pub = self.create_publisher(String, '/approach/object_positions', latched)
        self._markers = self.create_publisher(MarkerArray, '/approach/object_markers', latched)
        self.create_subscription(String, self.options['detections_topic'],
                                 self._detection, 1)
        self.create_subscription(Odometry, self.options['odom_topic'],
                                 self._odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.options['scan_topic'],
                                 self._scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/frontier_explorer/paused', self._ack, 1)
        self.create_subscription(Twist, self.options['nav_velocity_topic'],
                                 self._navigation, 1)
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._refresh_positions)
        self.get_logger().info('Approach command gate ready; waiting for detector, odometry and scan')
        self.get_logger().info(f'Object coordinates will be saved on this laptop to {self.positions.path}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _odom(self, msg):
        try:
            self._pose = pose_from(msg.pose.pose.position, msg.pose.pose.orientation)
            twist = msg.twist.twist
            values = (twist.linear.x, twist.linear.y, twist.angular.z)
            if not all(math.isfinite(v) for v in values):
                raise ValueError('invalid odometry velocity')
            self._stationary = all(abs(v) < 0.01 for v in values)
            self._odom_at = time.monotonic()
        except ValueError:
            self._odom_at = -math.inf

    def _scan(self, msg):
        # +inf is a valid clear ray. An empty/all-invalid scan is not clear.
        if any(r == math.inf or (math.isfinite(r) and
               msg.range_min <= r <= msg.range_max) for r in msg.ranges):
            self._scan_at = time.monotonic()

    def _ack(self, msg):
        self._paused, self._ack_at = msg.data, time.monotonic()

    def _navigation(self, msg):
        self._nav, self._nav_at = msg, time.monotonic()

    def _detection(self, msg):
        try:
            frame = parse_payload(msg.data)
            stamp = frame.stamp_sec + frame.stamp_nanosec / 1e9
            if stamp <= self.core.last_stamp or not 0 <= self._now() - stamp <= self.core.config.detection_timeout:
                return
            self._pending = frame
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
            self.get_logger().warning(f'Invalid detection: {exc}', throttle_duration_sec=2.0)

    def _consume_frame(self):
        frame = self._pending
        if frame is None:
            return
        stamp = frame.stamp_sec + frame.stamp_nanosec / 1e9
        now = self._now()
        if not 0 <= now - stamp <= self.core.config.detection_timeout:
            self._pending = None
            return
        if self.core.state in ('MEASURED', 'STOPPED', 'RESUMING'):
            # Retain the exact frame that produced the range while still
            # checking camera liveness for the subsequent resume handshake.
            self.core.observe([], stamp, now)
            self._frame_at = time.monotonic()
            self._pending = None
            return
        backend = frame.model.get('backend')
        if backend in ('stub', 'disabled'):
            # Fixed boxes cannot measure expansion and must never move a rover.
            self.core.observe([], stamp, now)
            self._frame_at = time.monotonic()
            self._pending = None
            return
        candidates = [d for d in frame.detections
                      if d.landmark in self.options['target_landmarks']
                      and d.confidence >= self.core.config.min_confidence
                      and (self.core.target is not None
                           or self.core.eligible(d.landmark, now, self._pose))]
        if not candidates:
            self.core.observe([], stamp, now)
            self._frame_at = time.monotonic()
            self._pending = None
            return
        captured = Time(seconds=frame.stamp_sec, nanoseconds=frame.stamp_nanosec)
        try:
            camera = self._tf.lookup_transform(
                self.options['base_frame'], frame.frame_id, captured).transform
            odom = self._tf.lookup_transform(
                self.options['odom_frame'], self.options['base_frame'], captured).transform
        except TransformException:
            # A target has been detected: stop even if its bearing/pose cannot
            # yet be reconstructed. Never continue a frontier while waiting
            # for geometry, and never substitute a latest-pose TF for capture.
            self.core.fail('target detected but capture-time camera/odometry TF unavailable')
            self._pending = None
            return
        try:
            matrix = rotation(camera.rotation)
            # Size expansion assumes optical z aligned with rover forward x.
            # Reject a materially pitched/sideways camera instead of fitting a
            # different projection to forward odometry.
            if matrix[0, 2] < math.cos(0.05) or abs(camera.translation.y) > 0.03:
                raise ValueError('approach requires a level forward camera on the rover centreline')
            signature = (frame.frame_id, frame.width, frame.height,
                         frame.intrinsics.fx, frame.intrinsics.fy,
                         frame.intrinsics.cx, frame.intrinsics.cy)
            if self.core.holding and signature != self._camera_signature:
                raise ValueError('camera calibration or resolution changed')
            self._camera_signature = signature
            pose = pose_from(odom.translation, odom.rotation)
            observations = []
            for detection in candidates:
                u, v = detection.centre_px(frame.width, frame.height)
                ray = matrix @ np.asarray(frame.intrinsics.bearing(u, v))
                observations.append(Observation(
                    stamp, detection.landmark, detection.model_class,
                    detection.box_norm, math.atan2(ray[1], ray[0]),
                    (detection.box_norm[3] - detection.box_norm[1]) * frame.height
                    / frame.intrinsics.fy, pose, detection.confidence))
            self.core.observe(observations, stamp, now)
            if self.core.state == 'MEASURED':
                self._measurement_frame = frame
                self._measurement_saved = False
            self._frame_at = time.monotonic()
        except (ValueError, ZeroDivisionError) as exc:
            self.core.fail(str(exc))
        self._pending = None

    def _tick(self):
        self._consume_frame()
        now, steady = self._now(), time.monotonic()
        timeout = self.options['sensor_timeout']
        sensors = steady - self._odom_at < timeout and steady - self._scan_at < timeout
        ack_fresh = steady - self._ack_at < timeout
        detector_fresh = steady - self._frame_at < self.core.config.detection_timeout
        if self.core.state == 'EXPLORING' and self._paused:
            self.core.fail('explorer already paused; restart both nodes for a new run')
        command = Twist()
        if self.core.state == 'RESUMING':
            self._hold.publish(Bool(data=False))
            if ack_fresh and not self._paused and self._ack_at > self._resume_since:
                self.core.finish_resume()
                # Discard all velocity commands from before release. Wait for
                # a fresh command from the explorer's newly planned goal.
                self._nav_at = -math.inf
                self._measurement_frame = None
                self._measurement_saved = False
                self._settled_since = None
                self._camera_signature = None
            elif now - self.core.entered > self.core.config.stop_timeout:
                self.core.fail('explorer resume acknowledgement timed out')
        elif self.core.holding:
            self._hold.publish(Bool(data=True))
            command.linear.x, command.angular.z = self.core.command(
                now, self._pose, self._stationary, self._paused and ack_fresh, sensors)
        elif (sensors and ack_fresh and steady - self._nav_at < timeout
              and (detector_fresh or not self.options['require_detector_for_navigation'])):
            command = self._nav
        self._output.publish(command)
        # Zero is published before any TF lookup or disk operation. A measured
        # target is persisted before its hold is released.
        if self.core.state == 'MEASURED' and not self._measurement_saved:
            self._save_measurement()
        can_resume = ((self.core.state == 'MEASURED' and self._measurement_saved)
                      or (self.core.state == 'STOPPED' and self.core.retryable))
        if (can_resume and sensors and ack_fresh and self._paused and self._stationary
                and (detector_fresh
                     or not self.options['require_detector_for_navigation'])):
            if self._settled_since is None:
                self._settled_since = steady
            elif steady - self._settled_since >= self.core.config.settle_time:
                self.core.begin_resume(now, self._pose, saved=self._measurement_saved)
                self._resume_since = steady
        else:
            self._settled_since = None
        if self.core.state != self._last_state:
            self.get_logger().info(f'{self.core.state}: {self.core.reason}')
            self._last_state = self.core.state
        estimate = self.core.range
        self._status.publish(String(data=json.dumps({
            'state': self.core.state, 'reason': self.core.reason,
            'target': self.core.target.landmark if self.core.target else None,
            'travel_m': self.core.travel,
            'range_m': max(0.0, estimate.initial_m - self.core.travel) if estimate else None,
            'range_valid': estimate is not None and self.core.state == 'MEASURED',
            'range_reference': 'camera',
            'fit_residual': estimate.residual if estimate else None,
            'range_sigma_m': estimate.sigma_m if estimate else None,
            'growth_ratio': self.core.estimator.growth(),
            'samples': len(self.core.estimator.samples),
            'sensors_fresh': sensors, 'explorer_paused': self._paused and ack_fresh,
            'detector_fresh': detector_fresh,
            'detector_required': self.options['require_detector_for_navigation'],
            'saved_objects': sorted(self.positions.objects),
            'object_measurements': {name: {
                'total': entry['measurement_count'],
                'used': entry['used_measurement_count'],
                'rejected': entry['rejected_measurement_count'],
                'sigma_major_m': entry['sigma_major_m'],
                'view_angle_spread_deg': entry['view_angle_spread_deg'],
                'fusion_status': entry['fusion_status'],
            } for name, entry in self.positions.objects.items()},
            'remaining_targets': sorted(set(self.options['target_landmarks']) - set(self.positions.objects)),
            'positions_file': str(self.positions.path),
            'map_id': self.positions.document['map_id'],
            'retryable': self.core.retryable,
        }, allow_nan=False)))

    def _save_measurement(self):
        frame, target, estimate = self._measurement_frame, self.core.target, self.core.range
        captured = Time(seconds=frame.stamp_sec, nanoseconds=frame.stamp_nanosec)
        try:
            camera_in_odom = self._tf.lookup_transform(
                self.options['odom_frame'], frame.frame_id, captured).transform
            odom_in_map = self._tf.lookup_transform(
                self.options['map_frame'], self.options['odom_frame'], Time()).transform
        except TransformException:
            if self._now() - self.core.entered > self.core.config.stop_timeout:
                self.core.fail('range measured but map transform unavailable', retryable=True)
            return
        try:
            translation = camera_in_odom.translation
            anchor = project_box(
                target.box, frame.width, frame.height, frame.intrinsics,
                estimate.current_m, rotation(camera_in_odom.rotation),
                (translation.x, translation.y, translation.z))
            camera_origin = {'x': translation.x, 'y': translation.y, 'z': translation.z}
            translation = odom_in_map.translation
            fused = self.positions.save(target.landmark, {
                'model_class': target.model_class,
                'position_odom_m': anchor,
                'camera_origin_odom_m': camera_origin,
                'source_image_stamp': {'sec': frame.stamp_sec, 'nanosec': frame.stamp_nanosec},
                'camera_frame': frame.frame_id, 'confidence': target.confidence,
                'optical_depth_m': estimate.current_m,
                'method': 'image_expansion_and_odometry',
                'fit_residual': estimate.residual, 'samples': estimate.samples,
                'range_sigma_m': estimate.sigma_m,
                'baseline_m': self.core.travel,
                'growth_ratio': self.core.estimator.growth(),
            }, map_rotation=rotation(odom_in_map.rotation),
                map_translation=(translation.x, translation.y, translation.z))
            self._measurement_saved = True
            self._publish_positions()
            mapped = fused['position_map_m']
            self.get_logger().info(
                f'Saved {target.landmark}: map x={mapped["x"]:.3f}, y={mapped["y"]:.3f} '
                f'from {fused["used_measurement_count"]} retained views; '
                f'latest used={fused["last_measurement_used"]}; file={self.positions.path}')
        except (OSError, ValueError, TypeError) as exc:
            self.core.fail(f'cannot save measured object: {exc}')

    def _refresh_positions(self):
        if not self.positions.objects:
            return
        try:
            transform = self._tf.lookup_transform(
                self.options['map_frame'], self.options['odom_frame'], Time()).transform
            t = transform.translation
            self.positions.reproject(rotation(transform.rotation), (t.x, t.y, t.z))
        except TransformException:
            return
        except (OSError, ValueError) as exc:
            self.core.fail(f'cannot refresh saved map positions: {exc}')
        self._publish_positions()

    def _publish_positions(self):
        self._positions_pub.publish(String(data=json.dumps(self.positions.document, allow_nan=False)))
        markers = MarkerArray()
        for index, (name, entry) in enumerate(sorted(self.positions.objects.items())):
            p = entry['position_map_m']
            for label in (False, True):
                marker = Marker()
                marker.header.frame_id = self.options['map_frame']
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = 'measured_objects', 2 * index + int(label)
                marker.type = Marker.TEXT_VIEW_FACING if label else Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.pose.position.x, marker.pose.position.y = p['x'], p['y']
                marker.pose.position.z = p['z'] + (0.20 if label else 0.0)
                marker.scale.x = marker.scale.y = marker.scale.z = 0.15
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    (1.0, 1.0, 1.0, 1.0) if label else (0.2, 0.9, 0.3, 1.0))
                marker.text = (
                    f'{name}: ({p["x"]:.2f}, {p["y"]:.2f}) m; '
                    f'{entry["used_measurement_count"]} views; '
                    f'est. sigma {entry["sigma_major_m"]:.2f} m')
                markers.markers.append(marker)
        self._markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._output.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
