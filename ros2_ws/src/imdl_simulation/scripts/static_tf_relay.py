#!/usr/bin/env python3
"""Relay namespaced Create3 static transforms into the global TF tree."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


class StaticTfRelay(Node):
    """Cache and republish static transforms with transient-local QoS."""

    def __init__(self) -> None:
        super().__init__('static_tf_relay')
        self.declare_parameter('input_topic', '/Team2/tf_static')
        self.declare_parameter('output_topic', '/tf_static')
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.transforms = {}
        self.received_batches = 0
        self.publisher = self.create_publisher(
            TFMessage,
            str(self.get_parameter('output_topic').value),
            output_qos,
        )
        self.subscription = self.create_subscription(
            TFMessage,
            str(self.get_parameter('input_topic').value),
            self._callback,
            input_qos,
        )
        self.create_timer(1.0, self._publish_cache)
        self.get_logger().info(
            'static_tf_relay: '
            f'{self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}'
        )

    def _callback(self, message: TFMessage) -> None:
        self.received_batches += 1
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self.transforms[key] = transform
        self._publish_cache()
        if self.received_batches == 1:
            self.get_logger().info(
                f'static TF cache initialized with '
                f'{len(self.transforms)} transforms'
            )

    def _publish_cache(self) -> None:
        if self.transforms:
            self.publisher.publish(TFMessage(
                transforms=list(self.transforms.values())
            ))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticTfRelay()
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
