"""ROS adapter for statistically gated, TF-safe object tracking."""

from __future__ import annotations

import math
from typing import List, Tuple

from imdl_interfaces.msg import TrackedObject, TrackedObjectArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

from .placed_objects import PLACED_OBJECT_EVENT_TOPIC, PlacedObjectZones
from .ros_shutdown import shutdown_node, spin_node_resilient
from .tracking import Detection, MultiObjectTracker

_TF2_GEOMETRY_REGISTRATION = tf2_geometry_msgs


def square_obstacle_points(
    x: float,
    y: float,
    z: float,
    half_extent_m: float,
) -> List[Tuple[float, float, float]]:
    """Represent one small square block with enough points for safety zones."""
    extent = max(0.0, float(half_extent_m))
    offsets = (
        (0.0, 0.0),
        (-extent, -extent),
        (-extent, 0.0),
        (-extent, extent),
        (0.0, -extent),
        (0.0, extent),
        (extent, -extent),
        (extent, 0.0),
        (extent, extent),
    )
    return [
        (float(x) + dx, float(y) + dy, float(z))
        for dx, dy in offsets
    ]


class ObjectRegistryNode(Node):

    def __init__(self):
        super().__init__('object_registry')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter(
            'typed_input_topic',
            '/perception/detected_objects',
        )
        self.declare_parameter('minimum_detection_confidence', 0.35)
        self.declare_parameter('output_topic', '/tracked_objects')
        self.declare_parameter('confirmations_required', 3)
        self.declare_parameter('gate_chi2', 9.21)
        self.declare_parameter('stale_after_sec', 1.5)
        self.declare_parameter('track_lifetime_sec', 5.0)
        self.declare_parameter('measurement_variance_m2', 0.0016)
        self.declare_parameter('max_detection_age_sec', 2.0)
        self.declare_parameter(
            'obstacle_output_topic',
            '/autonomy/tracked_obstacle_cloud',
        )
        self.declare_parameter('obstacle_min_confidence', 0.42)
        self.declare_parameter('obstacle_max_age_sec', 1.5)
        self.declare_parameter('obstacle_half_extent_m', 0.0175)
        self.declare_parameter(
            'placed_object_event_topic',
            PLACED_OBJECT_EVENT_TOPIC,
        )
        self.declare_parameter('placed_obstacle_exclusion_radius_m', 0.075)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.minimum_detection_confidence = float(
            self.get_parameter('minimum_detection_confidence').value
        )
        self.measurement_variance = float(
            self.get_parameter('measurement_variance_m2').value
        )
        self.tracker = MultiObjectTracker(
            gate_chi2=float(self.get_parameter('gate_chi2').value),
            confirmations_required=int(
                self.get_parameter('confirmations_required').value
            ),
            stale_after_sec=float(
                self.get_parameter('stale_after_sec').value
            ),
            lifetime_sec=float(
                self.get_parameter('track_lifetime_sec').value
            ),
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.placed_zones = PlacedObjectZones(
            self.target_frame,
            float(
                self.get_parameter(
                    'placed_obstacle_exclusion_radius_m'
                ).value
            ),
        )
        self.publisher = self.create_publisher(
            TrackedObjectArray,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.obstacle_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('obstacle_output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectedObjectArray,
            str(self.get_parameter('typed_input_topic').value),
            self._typed_detections_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('placed_object_event_topic').value),
            self._placed_object_callback,
            10,
        )
        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            'object_registry: typed='
            f'{self.get_parameter("typed_input_topic").value}, '
            f'frame={self.target_frame}'
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _placed_object_callback(self, message: String) -> None:
        zone = self.placed_zones.update(message.data)
        if zone is not None:
            self.get_logger().info(
                f'Excluding placed {zone.color}/{zone.object_id} from '
                f'low-obstacle fusion within {zone.radius_m:.3f}m'
            )

    def _measurement_variance(
        self,
        depth_m: float,
        area_px: float,
        confidence: float = 1.0,
        depth_mad_m: float = 0.0,
        valid_depth_ratio: float = 1.0,
    ) -> float:
        variance = self.measurement_variance
        if math.isfinite(depth_m) and depth_m > 0.0:
            variance += min(0.003, depth_m * depth_m * 0.0008)
        if area_px > 1.0:
            variance += min(0.002, 0.02 / math.sqrt(area_px))
        if math.isfinite(depth_mad_m) and depth_mad_m > 0.0:
            variance += min(0.004, depth_mad_m * depth_mad_m)
        if math.isfinite(valid_depth_ratio):
            variance += self.measurement_variance * max(
                0.0,
                0.5 - min(1.0, valid_depth_ratio),
            )
        variance += self.measurement_variance * max(
            0.0,
            1.0 - min(1.0, confidence),
        )
        return variance

    def _typed_detections_callback(
        self,
        message: DetectedObjectArray,
    ) -> None:
        # An empty typed array is authoritative too: it means the current
        # frame contained no accepted objects and must suppress duplicate JSON.
        now_sec = self._now_seconds()
        max_detection_age = float(
            self.get_parameter('max_detection_age_sec').value
        )
        detections: List[Detection] = []
        for item in message.objects:
            if not is_navigation_candidate(
                item,
                self.minimum_detection_confidence,
            ):
                continue
            color = block_color(item.class_name, item.is_block)
            measurement_sec = stamp_seconds(item.header, message.header)
            if measurement_sec is None:
                measurement_sec = now_sec
            detection_age = now_sec - measurement_sec
            if detection_age > max_detection_age or detection_age < -0.5:
                self.get_logger().warning(
                    f'Dropping typed {color} detection with implausible age '
                    f'{detection_age:.3f}s',
                    throttle_duration_sec=2.0,
                )
                continue
            position = item.position_map_m
            coordinates = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
            if not all(math.isfinite(value) for value in coordinates):
                continue
            confidence = float(item.confidence)
            depth = float(item.depth_m) if item.depth_valid else 0.0
            detections.append(Detection(
                color=color,
                x=coordinates[0],
                y=coordinates[1],
                z=coordinates[2],
                stamp_sec=measurement_sec,
                variance=self._measurement_variance(
                    depth,
                    float(item.mask_area_px),
                    confidence,
                    float(item.depth_mad_m),
                    float(item.valid_depth_ratio),
                ),
            ))
        if detections:
            self.tracker.update(detections, now_sec)

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        self.tracker.prune(now_sec)
        array = TrackedObjectArray()
        array.header.frame_id = self.target_frame
        array.header.stamp = now.to_msg()
        status_values = {
            'TENTATIVE': TrackedObject.STATUS_TENTATIVE,
            'CONFIRMED': TrackedObject.STATUS_CONFIRMED,
            'STALE': TrackedObject.STATUS_STALE,
            'CARRIED': TrackedObject.STATUS_CARRIED,
        }
        for track in self.tracker.tracks:
            message = TrackedObject()
            message.header = array.header
            message.object_id = track.object_id
            message.color = track.color
            message.pose.pose.position.x = float(track.position[0])
            message.pose.pose.position.y = float(track.position[1])
            message.pose.pose.position.z = float(track.z)
            message.pose.pose.orientation.w = 1.0
            message.pose.covariance[0] = float(track.covariance[0, 0])
            message.pose.covariance[1] = float(track.covariance[0, 1])
            message.pose.covariance[6] = float(track.covariance[1, 0])
            message.pose.covariance[7] = float(track.covariance[1, 1])
            message.pose.covariance[35] = 0.25
            message.confidence = track.confidence(
                now_sec,
                self.tracker.stale_after_sec,
            )
            message.confirmations = track.confirmations
            message.age_sec = track.age(now_sec)
            message.status = status_values[self.tracker.status(track, now_sec)]
            array.objects.append(message)
        self.publisher.publish(array)
        self._publish_obstacle_cloud(array)

    def _publish_obstacle_cloud(self, array: TrackedObjectArray) -> None:
        """Expose confirmed low objects to Nav2 without raw depth-cloud cost."""
        min_confidence = float(
            self.get_parameter('obstacle_min_confidence').value
        )
        max_age_sec = float(
            self.get_parameter('obstacle_max_age_sec').value
        )
        half_extent = float(
            self.get_parameter('obstacle_half_extent_m').value
        )
        points = []
        for item in array.objects:
            if (
                item.status != TrackedObject.STATUS_CONFIRMED
                or item.confidence < min_confidence
                or item.age_sec > max_age_sec
            ):
                continue
            position = item.pose.pose.position
            if self.placed_zones.contains(position.x, position.y):
                continue
            points.extend(square_obstacle_points(
                position.x,
                position.y,
                position.z,
                half_extent,
            ))
        self.obstacle_publisher.publish(
            point_cloud2.create_cloud_xyz32(array.header, points)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRegistryNode()
    try:
        spin_node_resilient(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_node(node)
