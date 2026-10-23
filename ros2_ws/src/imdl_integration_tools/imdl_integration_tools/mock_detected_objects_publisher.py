"""Publish deterministic typed object frames without camera or YOLO hardware."""

import rclpy
from imdl_msgs.msg import DetectedObjectArray
from rclpy.node import Node

from imdl_integration_tools.mock_object_fixtures import SCENARIOS, make_scenario


class MockDetectedObjectsPublisher(Node):
    """Publish one named test scenario to the shared typed perception topic."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            'mock_detected_objects_publisher', parameter_overrides=parameter_overrides
        )
        self.declare_parameter('objects_topic', 'perception/detected_objects')
        self.declare_parameter('scenario', 'mixed')
        self.declare_parameter('publish_rate_hz', 2.0)

        self.objects_topic = str(self.get_parameter('objects_topic').value)
        self.scenario = str(self.get_parameter('scenario').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not self.objects_topic:
            raise ValueError('objects_topic must not be empty')
        if self.scenario not in SCENARIOS:
            raise ValueError(f'scenario must be one of: {", ".join(SCENARIOS)}')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self.publisher = self.create_publisher(
            DetectedObjectArray, self.objects_topic, 10
        )
        self.last_message = None
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish)
        self.get_logger().info(
            f'Publishing mock scenario={self.scenario} to {self.objects_topic}'
        )

    def _publish(self):
        message = make_scenario(self.scenario, self.get_clock().now().to_msg())
        self.last_message = message
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectedObjectsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
