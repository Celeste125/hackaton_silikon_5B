#!/usr/bin/env python3
"""Remove simulated lidar returns caused by the robot's own geometry."""

from __future__ import annotations

import ast
import math
from typing import Iterable, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


Point = Tuple[float, float]

DEFAULT_SELF_FOOTPRINT = (
    '[[0.19, 0.06], [0.15, 0.13], [0.07, 0.17], '
    '[-0.12, 0.16], [-0.17, 0.08], [-0.17, -0.08], '
    '[-0.12, -0.16], [0.07, -0.17], [0.15, -0.13], '
    '[0.19, -0.06]]'
)


def parse_footprint(value: str) -> Tuple[Point, ...]:
    """Parse and validate a ROS-style polygon parameter."""
    parsed = ast.literal_eval(value)
    if not isinstance(parsed, (list, tuple)) or len(parsed) < 3:
        raise ValueError('self footprint requires at least three points')
    points = []
    for item in parsed:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError('each self-footprint point must contain x and y')
        x, y = float(item[0]), float(item[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('self-footprint coordinates must be finite')
        points.append((x, y))
    return tuple(points)


def _point_on_segment(
    x: float,
    y: float,
    first: Point,
    second: Point,
    epsilon: float = 1e-9,
) -> bool:
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    cross = (x - first[0]) * dy - (y - first[1]) * dx
    if abs(cross) > epsilon:
        return False
    dot = (x - first[0]) * dx + (y - first[1]) * dy
    if dot < -epsilon:
        return False
    squared_length = dx * dx + dy * dy
    return dot <= squared_length + epsilon


def point_in_polygon(x: float, y: float, polygon: Sequence[Point]) -> bool:
    """Return true for points inside or on the boundary of a polygon."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _point_on_segment(x, y, previous, current):
            return True
        if (current[1] > y) != (previous[1] > y):
            intersection_x = (
                (previous[0] - current[0])
                * (y - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if x < intersection_x:
                inside = not inside
        previous = current
    return inside


def filter_self_returns(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    polygon: Sequence[Point],
    sensor_origin_x: float = 0.0,
    sensor_origin_y: float = 0.0,
) -> Tuple[list[float], int]:
    """Replace finite scan endpoints inside the robot polygon with infinity."""
    filtered = []
    removed = 0
    for index, value in enumerate(ranges):
        distance = float(value)
        if math.isfinite(distance):
            angle = angle_min + index * angle_increment
            x = sensor_origin_x + distance * math.cos(angle)
            y = sensor_origin_y + distance * math.sin(angle)
            if point_in_polygon(x, y, polygon):
                distance = math.inf
                removed += 1
        filtered.append(distance)
    return filtered, removed


def filtered_scan(
    scan: LaserScan,
    polygon: Sequence[Point],
    sensor_origin_x: float = 0.0,
    sensor_origin_y: float = 0.0,
) -> Tuple[LaserScan, int]:
    """Copy a LaserScan while removing only endpoints inside the robot."""
    message = LaserScan()
    message.header = scan.header
    message.angle_min = scan.angle_min
    message.angle_max = scan.angle_max
    message.angle_increment = scan.angle_increment
    message.time_increment = scan.time_increment
    message.scan_time = scan.scan_time
    message.range_min = scan.range_min
    message.range_max = scan.range_max
    message.ranges, removed = filter_self_returns(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
        polygon,
        sensor_origin_x,
        sensor_origin_y,
    )
    message.intensities = list(scan.intensities)
    return message, removed


class ScanSelfFilter(Node):
    """Publish a canonical scan without self-reflections from mounted sensors."""

    def __init__(self) -> None:
        super().__init__('scan_self_filter')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('self_footprint', DEFAULT_SELF_FOOTPRINT)
        self.declare_parameter('sensor_origin_x', 0.0)
        self.declare_parameter('sensor_origin_y', 0.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.polygon = parse_footprint(
            str(self.get_parameter('self_footprint').value)
        )
        self.sensor_origin_x = float(
            self.get_parameter('sensor_origin_x').value
        )
        self.sensor_origin_y = float(
            self.get_parameter('sensor_origin_y').value
        )
        self.publisher = self.create_publisher(
            LaserScan,
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
            f'lidar self-filter: {input_topic} -> {output_topic}, '
            f'footprint_points={len(self.polygon)}'
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        message, removed = filtered_scan(
            scan,
            self.polygon,
            self.sensor_origin_x,
            self.sensor_origin_y,
        )
        self.publisher.publish(message)
        self.received += 1
        if self.received == 1:
            self.get_logger().info(
                f'first lidar scan filtered: '
                f'{removed}/{len(scan.ranges)} self returns removed'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSelfFilter()
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
