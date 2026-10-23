"""Select a stable, task-requested block from enriched 3D detections."""

import copy
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from imdl_msgs.msg import DetectedObjectArray
from rclpy.node import Node
from std_msgs.msg import String

from perception.target_selection_utils import parse_class_sequence, select_best_target
from perception.yolo_detection_utils import CLASS_NAMES


class TargetSelector(Node):
    """Publish the requested block while never selecting a blue obstacle."""

    def __init__(self):
        super().__init__('target_selector')

        self.declare_parameter(
            'typed_objects_topic', '/perception/detected_objects'
        )
        self.declare_parameter('current_target_topic', '/current_target_object')
        self.declare_parameter('target_pose_topic', '/target_block_pose')
        self.declare_parameter('requested_class_topic', '/requested_target_class')
        self.declare_parameter('target_class', '')
        self.declare_parameter(
            'target_color_sequence',
            'red_block,yellow_block,green_block,blue_block',
        )
        self.declare_parameter('min_target_confidence', 0.35)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('selection_rate_hz', 10.0)
        self.declare_parameter('target_frame', 'map')

        self.typed_objects_topic = self._string_parameter('typed_objects_topic')
        self.current_target_topic = self._string_parameter('current_target_topic')
        self.target_pose_topic = self._string_parameter('target_pose_topic')
        self.requested_class_topic = self._string_parameter('requested_class_topic')
        self.target_class = str(self.get_parameter('target_class').value).strip()
        self.target_sequence = parse_class_sequence(
            str(self.get_parameter('target_color_sequence').value)
        )
        self.min_target_confidence = float(
            self.get_parameter('min_target_confidence').value
        )
        self.stale_timeout_sec = float(
            self.get_parameter('stale_timeout_sec').value
        )
        self.selection_rate_hz = float(
            self.get_parameter('selection_rate_hz').value
        )
        self.target_frame = self._string_parameter('target_frame')
        self._validate_parameters()

        self.requested_class = self.target_class or self.target_sequence[0]
        self.latest_objects = []
        self.last_target = None

        self.create_subscription(
            DetectedObjectArray,
            self.typed_objects_topic,
            self._objects_cb,
            10,
        )
        self.create_subscription(
            String, self.requested_class_topic, self._requested_class_cb, 10
        )
        self.current_target_pub = self.create_publisher(
            String, self.current_target_topic, 10
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / self.selection_rate_hz, self._select_and_publish
        )
        self.get_logger().info(
            'target_selector ready: '
            f'requested_class={self.requested_class}, '
            f'objects={self.typed_objects_topic} -> {self.target_pose_topic}'
        )

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _validate_parameters(self):
        selectable = set(CLASS_NAMES.values()) - {'blue_obstacle'}
        if not self.target_sequence or not set(self.target_sequence) <= selectable:
            raise ValueError('target_color_sequence must contain block class names')
        if self.target_class and self.target_class not in selectable:
            raise ValueError('target_class must be empty or a block class name')
        if not 0.0 <= self.min_target_confidence <= 1.0:
            raise ValueError('min_target_confidence must be within [0, 1]')
        if self.stale_timeout_sec <= 0.0 or self.selection_rate_hz <= 0.0:
            raise ValueError('stale_timeout_sec and selection_rate_hz must be positive')

    def _objects_cb(self, msg):
        self.latest_objects = [
            self._typed_object_as_selection_dict(item)
            for item in msg.objects
        ]
        self._select_and_publish()

    @staticmethod
    def _typed_object_as_selection_dict(item):
        return {
            'class_name': item.class_name,
            'is_block': bool(item.is_block),
            'confidence': float(item.confidence),
            'is_stale': bool(item.is_stale),
            'transform_status': item.transform_status,
            'timestamp': {
                'sec': int(item.header.stamp.sec),
                'nanosec': int(item.header.stamp.nanosec),
            },
            'position_map': {
                'x': float(item.position_map_m.x),
                'y': float(item.position_map_m.y),
                'z': float(item.position_map_m.z),
            },
            'polar_base': {
                'range_m': float(item.range_m) if item.base_pose_valid else None,
            },
            'map_pose_valid': bool(item.map_pose_valid),
        }

    def _requested_class_cb(self, msg):
        requested = msg.data.strip()
        selectable = set(CLASS_NAMES.values()) - {'blue_obstacle'}
        if requested not in selectable:
            self.get_logger().warn(
                f'Ignoring invalid requested target class: {requested!r}',
                throttle_duration_sec=2.0,
            )
            return
        if requested != self.requested_class:
            self.requested_class = requested
            self.last_target = None
            self.get_logger().info(f'Requested target class changed to {requested}')

    def _select_and_publish(self):
        last_position = None
        if self.last_target is not None:
            last_position = self.last_target['position_map']
        selected = select_best_target(
            self.latest_objects,
            self.requested_class,
            self.min_target_confidence,
            last_position=last_position,
        )
        if selected is not None:
            self.last_target = copy.deepcopy(selected)
            self._publish_target(selected, is_stale=False, reason='fresh_match')
            return

        if self.last_target is not None and self._last_target_is_fresh():
            self._publish_target(
                self.last_target, is_stale=True, reason='stale_last_known'
            )
            return

        self.last_target = None
        self.current_target_pub.publish(
            String(data=json.dumps({
                'selected': False,
                'requested_class': self.requested_class,
                'is_stale': False,
                'selection_reason': 'no_eligible_target',
                'object': None,
            }, ensure_ascii=True, separators=(',', ':')))
        )

    def _last_target_is_fresh(self):
        stamp = self.last_target.get('timestamp', {})
        try:
            stamp_ns = int(stamp['sec']) * 1_000_000_000 + int(stamp['nanosec'])
        except (KeyError, TypeError, ValueError):
            return False
        if stamp_ns == 0:
            return False
        age_sec = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0
        return 0.0 <= age_sec <= self.stale_timeout_sec

    def _publish_target(self, target, is_stale, reason):
        current = copy.deepcopy(target)
        current['is_stale'] = is_stale
        self.current_target_pub.publish(
            String(data=json.dumps({
                'selected': True,
                'requested_class': self.requested_class,
                'is_stale': is_stale,
                'selection_reason': reason,
                'object': current,
            }, ensure_ascii=True, separators=(',', ':')))
        )

        position = current['position_map']
        stamp = current['timestamp']
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp.sec = int(stamp['sec'])
        pose.header.stamp.nanosec = int(stamp['nanosec'])
        pose.pose.position.x = float(position['x'])
        pose.pose.position.y = float(position['y'])
        pose.pose.position.z = float(position['z'])
        pose.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
