"""End-to-end ROS test for the mock publisher and B/C filter examples."""

import time
import uuid

import pytest
import rclpy
from imdl_msgs.msg import DetectedObjectArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from imdl_integration_tools.control_object_filter import ControlObjectFilter
from imdl_integration_tools.mock_detected_objects_publisher import (
    MockDetectedObjectsPublisher,
)
from imdl_integration_tools.navigation_object_filter import NavigationObjectFilter


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mock_pipeline_filters_navigation_and_control_candidates():
    identifier = uuid.uuid4().hex
    namespace = f'/bc_mock_test_{identifier}'
    input_topic = f'{namespace}/perception/detected_objects'
    navigation_topic = f'{namespace}/integration/navigation/map_valid_objects'
    control_topic = f'{namespace}/integration/control/grasp_candidates'

    mock = MockDetectedObjectsPublisher(parameter_overrides=[
        Parameter('objects_topic', value=input_topic),
        Parameter('scenario', value='mixed'),
        Parameter('publish_rate_hz', value=20.0),
    ])
    navigation = NavigationObjectFilter(parameter_overrides=[
        Parameter('objects_topic', value=input_topic),
        Parameter('output_topic', value=navigation_topic),
    ])
    control = ControlObjectFilter(parameter_overrides=[
        Parameter('objects_topic', value=input_topic),
        Parameter('output_topic', value=control_topic),
    ])
    capture = Node(f'bc_capture_{identifier}')
    navigation_messages = []
    control_messages = []
    capture.create_subscription(
        DetectedObjectArray, navigation_topic, navigation_messages.append, 10
    )
    capture.create_subscription(
        DetectedObjectArray, control_topic, control_messages.append, 10
    )
    executor = SingleThreadedExecutor()
    for node in (mock, navigation, control, capture):
        executor.add_node(node)

    try:
        assert spin_until(
            executor,
            lambda: navigation_messages and control_messages and mock.last_message,
        ), 'mock B/C pipeline did not publish both filtered topics'

        navigation_message = navigation_messages[-1]
        control_message = control_messages[-1]
        assert [object_message.class_name for object_message in navigation_message.objects] == [
            'red_block',
            'blue_obstacle',
            'green_block',
            'blue_block',
        ]
        assert [object_message.class_name for object_message in control_message.objects] == [
            'red_block',
            'yellow_block',
        ]
        assert all(object_message.is_block for object_message in control_message.objects)
        assert navigation_message.header.frame_id == mock.last_message.header.frame_id
        assert navigation_message.header.stamp == mock.last_message.header.stamp
        assert control_message.header.stamp == mock.last_message.header.stamp
    finally:
        executor.shutdown()
        for node in (capture, control, navigation, mock):
            node.destroy_node()


def spin_until(executor, predicate, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return predicate()
