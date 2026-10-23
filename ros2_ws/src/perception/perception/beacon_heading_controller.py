"""
Beacon heading controller: computes the angle from the robot heading to the
current target block and publishes it as a Float32 command.

Subscriptions:
  /target_block_pose  (geometry_msgs/PoseStamped)

TF lookup:
  map -> base_link

Publications:
  /beacon_cmd         (std_msgs/Float32) angle error in radians

Usage:
  ros2 run perception beacon_heading_controller
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener


class BeaconHeadingController(Node):
    def __init__(self):
        super().__init__('beacon_heading_controller')

        self.declare_parameter('target_pose_topic', '/target_block_pose')
        self.declare_parameter('command_topic', '/beacon_cmd')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('stale_timeout_sec', 1.0)

        self.target_pose_topic = self.get_parameter(
            'target_pose_topic').get_parameter_value().string_value
        self.command_topic = self.get_parameter(
            'command_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        publish_rate_hz = self.get_parameter(
            'publish_rate_hz').get_parameter_value().double_value
        self.stale_timeout = Duration(
            seconds=self.get_parameter(
                'stale_timeout_sec').get_parameter_value().double_value
        )

        if publish_rate_hz <= 0.0:
            publish_rate_hz = 10.0

        self.latest_target_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_cb,
            10,
        )
        self.pub_cmd = self.create_publisher(Float32, self.command_topic, 10)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_cmd)
        self.get_logger().info(
            f'beacon_heading_controller started: '
            f'TF {self.target_pose_topic} + {self.base_frame} -> '
            f'{self.command_topic}')

    def _target_pose_cb(self, msg: PoseStamped):
        self.latest_target_pose = msg

    def _publish_cmd(self):
        if self.latest_target_pose is None:
            return

        now = self.get_clock().now()
        if self._is_stale(self.latest_target_pose.header.stamp, now):
            self.get_logger().warn(
                'Target pose is stale; skipping beacon command',
                throttle_duration_sec=2.0,
            )
            return

        target_frame = self.latest_target_pose.header.frame_id
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {target_frame} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        robot_pose = robot_transform.transform
        target_pose = self.latest_target_pose.pose

        dx = target_pose.position.x - robot_pose.translation.x
        dy = target_pose.position.y - robot_pose.translation.y
        if dx == 0.0 and dy == 0.0:
            return

        target_yaw = math.atan2(dy, dx)
        robot_yaw = self._yaw_from_quaternion(robot_pose.rotation)
        angle_error = self._normalize_angle(target_yaw - robot_yaw)

        self.pub_cmd.publish(Float32(data=float(angle_error)))

    def _is_stale(self, stamp, now):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False
        return now - Time.from_msg(stamp) > self.stale_timeout

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = BeaconHeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
