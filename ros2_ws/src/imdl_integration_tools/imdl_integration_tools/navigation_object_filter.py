"""Minimal navigation-side typed-object subscription example."""

import rclpy
from imdl_msgs.msg import DetectedObjectArray
from rclpy.node import Node

from imdl_integration_tools.object_filter_utils import (
    filtered_array,
    is_navigation_candidate,
)


class NavigationObjectFilter(Node):
    """Keep fresh objects that have a valid map-frame position for navigation."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('navigation_object_filter', parameter_overrides=parameter_overrides)
        self.declare_parameter('objects_topic', 'perception/detected_objects')
        self.declare_parameter(
            'output_topic', 'integration/navigation/map_valid_objects'
        )
        self.objects_topic = str(self.get_parameter('objects_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        if not self.objects_topic or not self.output_topic:
            raise ValueError('objects_topic and output_topic must not be empty')

        self.publisher = self.create_publisher(DetectedObjectArray, self.output_topic, 10)
        self.create_subscription(
            DetectedObjectArray, self.objects_topic, self._objects_cb, 10
        )
        self.get_logger().info(
            f'Navigation example: {self.objects_topic} -> {self.output_topic}'
        )

    def _objects_cb(self, message):
        result = filtered_array(message, is_navigation_candidate)
        self.publisher.publish(result)
        for object_message in result.objects:
            point = object_message.position_map_m
            self.get_logger().info(
                f'nav candidate {object_message.class_name}: '
                f'map=({point.x:.3f}, {point.y:.3f}, {point.z:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationObjectFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
