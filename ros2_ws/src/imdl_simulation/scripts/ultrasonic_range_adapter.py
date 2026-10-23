#!/usr/bin/env python3
"""Convert Gazebo's narrow ultrasonic ray scan to a canonical Range topic."""

from __future__ import annotations

import math
import statistics
from typing import Iterable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


def representative_range(
    ranges: Iterable[float],
    minimum: float,
    maximum: float,
) -> float:
    """Return a robust central echo, or max range when the cone is clear."""
    valid = sorted(
        float(value)
        for value in ranges
        if math.isfinite(value) and minimum <= float(value) <= maximum
    )
    if not valid:
        # A variable-distance Range message must stay within its declared
        # limits. Using max_range for a clear cone also lets Nav2 clear an old
        # sonar obstacle; an infinite value is discarded by RangeSensorLayer.
        return float(maximum)
    return float(statistics.median(valid))


def scan_to_range(scan: LaserScan) -> Range:
    """Build a standards-compliant ultrasound Range message."""
    message = Range()
    message.header = scan.header
    message.radiation_type = Range.ULTRASOUND
    message.field_of_view = max(
        0.0,
        min(math.pi, abs(float(scan.angle_max) - float(scan.angle_min))),
    )
    message.min_range = float(scan.range_min)
    message.max_range = float(scan.range_max)
    message.range = representative_range(
        scan.ranges,
        message.min_range,
        message.max_range,
    )
    return message


class UltrasonicRangeAdapter(Node):
    """Expose the simulated sonar through the same topic as future hardware."""

    def __init__(self) -> None:
        super().__init__('ultrasonic_range_adapter')
        self.declare_parameter(
            'input_topic',
            '/ultrasonic/front/scan_raw',
        )
        self.declare_parameter(
            'output_topic',
            '/ultrasonic/front/range',
        )
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(
            Range,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.received = 0
        self.get_logger().info(
            f'ultrasonic adapter: {input_topic} -> {output_topic}'
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        message = scan_to_range(scan)
        self.publisher.publish(message)
        self.received += 1
        if self.received == 1:
            reading = (
                'clear cone'
                if message.range >= message.max_range
                else f'{message.range:.3f} m'
            )
            self.get_logger().info(
                f'first ultrasonic cone received: '
                f'{len(scan.ranges)} rays, {reading}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicRangeAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
