import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

# Republishes Create3 odometry into the frame pair selected by the launch file.
# Some Create3 deployments expose /Team2/tf but not /Team2/odom, so the node
# can fall back to extracting a matching transform from the namespaced topic.


def normalized_frame_id(value):
    """Return a TF frame id without ROS topic-style leading slashes."""
    return str(value).strip().lstrip('/')


class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_republisher')
        self.declare_parameter('odom_topic', '/Team2/odom')
        self.declare_parameter('namespaced_tf_topic', '/Team2/tf')
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('use_message_stamp', False)
        self.declare_parameter('stamp_backdate_sec', 0.0)
        self.declare_parameter('odom_fallback_timeout_sec', 0.5)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.namespaced_tf_topic = str(
            self.get_parameter('namespaced_tf_topic').value
        )
        self.parent_frame_id = normalized_frame_id(
            self.get_parameter('parent_frame_id').value
        )
        self.child_frame_id = normalized_frame_id(
            self.get_parameter('child_frame_id').value
        )
        if not self.parent_frame_id or not self.child_frame_id:
            raise ValueError(
                'parent_frame_id and child_frame_id must not be empty'
            )
        if self.parent_frame_id == self.child_frame_id:
            raise ValueError('parent_frame_id and child_frame_id must differ')
        self.use_message_stamp = bool(
            self.get_parameter('use_message_stamp').value
        )
        self.stamp_backdate_sec = max(
            0.0,
            float(self.get_parameter('stamp_backdate_sec').value),
        )
        fallback_timeout = float(
            self.get_parameter('odom_fallback_timeout_sec').value
        )
        if fallback_timeout <= 0.0:
            raise ValueError('odom_fallback_timeout_sec must be positive')
        self.odom_fallback_timeout = Duration(seconds=fallback_timeout)

        self.broadcaster = TransformBroadcaster(self)
        self.latest_odom = None
        self.latest_odom_received_at = None
        self.latest_odom_stamp = None
        self.latest_namespaced_tf = None
        self.latest_namespaced_tf_stamp = None
        self.active_source = None

        # Create3 publishes odometry with BEST_EFFORT, so the subscriber must match it.
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, sub_qos)
        self.create_subscription(
            TFMessage,
            self.namespaced_tf_topic,
            self.namespaced_tf_callback,
            sub_qos,
        )
        # Publish TF continuously at 50 Hz to keep the buffer fresh.
        self.create_timer(0.02, self.timer_callback)
        stamp_source = (
            'source message' if self.use_message_stamp else 'local clock'
        )
        self.get_logger().info(
            'odom_tf_republisher started: '
            f'odom={self.odom_topic}, tf fallback={self.namespaced_tf_topic}, '
            f'transform={self.parent_frame_id} -> {self.child_frame_id}, '
            f'stamp={stamp_source}'
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation,
        )
        self.latest_odom_stamp = msg.header.stamp
        self.latest_odom_received_at = self.get_clock().now()

    def namespaced_tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            if (
                parent == self.parent_frame_id
                and child == self.child_frame_id
            ):
                self.latest_namespaced_tf = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation,
                )
                self.latest_namespaced_tf_stamp = transform.header.stamp
                return

    def _select_latest_transform(self):
        now = self.get_clock().now()
        if (
            self.latest_odom is not None
            and self.latest_odom_received_at is not None
            and now - self.latest_odom_received_at <= self.odom_fallback_timeout
        ):
            return 'odom', self.latest_odom, self.latest_odom_stamp
        if self.latest_namespaced_tf is not None:
            return (
                'namespaced_tf',
                self.latest_namespaced_tf,
                self.latest_namespaced_tf_stamp,
            )
        return None, None, None

    def _set_stamp(self, transform, source_stamp):
        if not self.use_message_stamp or source_stamp is None:
            transform.header.stamp = self.get_clock().now().to_msg()
            return
        stamp_ns = (
            int(source_stamp.sec) * 1_000_000_000
            + int(source_stamp.nanosec)
            - int(self.stamp_backdate_sec * 1_000_000_000)
        )
        stamp_ns = max(0, stamp_ns)
        transform.header.stamp.sec = stamp_ns // 1_000_000_000
        transform.header.stamp.nanosec = stamp_ns % 1_000_000_000

    def timer_callback(self):
        source, transform, source_stamp = self._select_latest_transform()
        if transform is None:
            return
        if source != self.active_source:
            self.active_source = source
            self.get_logger().info(f'Publishing odom TF from {source}')

        x, y, z, rotation = transform
        t = TransformStamped()
        self._set_stamp(t, source_stamp)
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = rotation
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
