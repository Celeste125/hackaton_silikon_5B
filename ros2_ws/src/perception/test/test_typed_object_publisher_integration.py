"""ROS integration tests for the typed detected-objects publisher."""

import json
import time
import uuid

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from imdl_msgs.msg import DetectedObjectArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from perception.target_pose_estimator import TargetPoseEstimator


IMAGE_WIDTH = 20
IMAGE_HEIGHT = 20
SOURCE_FRAME = 'camera_color_optical_frame'


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class TypedPublisherHarness:
    """Run the estimator with deterministic mock ROS inputs."""

    def __init__(self, publish_transforms=True):
        identifier = uuid.uuid4().hex
        self.namespace = f'/typed_object_integration_{identifier}'
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.input_node = Node(f'typed_input_{identifier}')
        self.output_node = Node(f'typed_output_{identifier}')
        self.typed_messages = []

        self.detections_topic = f'{self.namespace}/yolo/detections'
        self.depth_topic = f'{self.namespace}/aligned_depth'
        self.camera_info_topic = f'{self.namespace}/camera_info'
        self.typed_topic = f'{self.namespace}/perception/detected_objects'

        parameters = [
            Parameter('detections_topic', value=self.detections_topic),
            Parameter('aligned_depth_topic', value=self.depth_topic),
            Parameter('camera_info_topic', value=self.camera_info_topic),
            Parameter('typed_objects_topic', value=self.typed_topic),
            Parameter('target_frame', value='map'),
            Parameter('base_frame', value='base_link'),
        ]
        self.estimator = TargetPoseEstimator(parameter_overrides=parameters)

        self.detections_publisher = self.input_node.create_publisher(
            String, self.detections_topic, 10
        )
        self.depth_publisher = self.input_node.create_publisher(
            Image, self.depth_topic, 10
        )
        self.camera_info_publisher = self.input_node.create_publisher(
            CameraInfo, self.camera_info_topic, 10
        )
        self.output_node.create_subscription(
            DetectedObjectArray, self.typed_topic, self.typed_messages.append, 10
        )

        self.static_broadcaster = None
        if publish_transforms:
            self.static_broadcaster = StaticTransformBroadcaster(self.input_node)
            self.static_broadcaster.sendTransform([
                static_transform('map', 'base_link', 1.0, 2.0, 0.0),
                static_transform('base_link', SOURCE_FRAME, 0.0, 0.0, 0.0),
            ])

        for node in (self.input_node, self.output_node, self.estimator):
            self.executor.add_node(node)
        assert self.spin_until(self._connections_ready), 'ROS test graph did not connect'
        if publish_transforms:
            assert self.spin_until(
                lambda: self.estimator.tf_buffer.can_transform(
                    'map', SOURCE_FRAME, rclpy.time.Time()
                )
            ), 'Static map/base/camera TF was not received'

    def close(self):
        self.executor.shutdown()
        for node in (self.estimator, self.output_node, self.input_node):
            node.destroy_node()

    def spin_until(self, predicate, timeout_sec=3.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def _connections_ready(self):
        return all(
            publisher.get_subscription_count() >= 1
            for publisher in (
                self.detections_publisher,
                self.depth_publisher,
                self.camera_info_publisher,
            )
        )

    def publish_camera_info(self, stamp):
        message = CameraInfo()
        set_header(message.header, stamp, SOURCE_FRAME)
        message.width = IMAGE_WIDTH
        message.height = IMAGE_HEIGHT
        message.k = [100.0, 0.0, 10.0, 0.0, 100.0, 10.0, 0.0, 0.0, 1.0]
        self.camera_info_publisher.publish(message)
        assert self.spin_until(lambda: self.estimator.camera_info is not None)

    def publish_depth(self, stamp, depth_mm=1000):
        image = np.full((IMAGE_HEIGHT, IMAGE_WIDTH), depth_mm, dtype=np.uint16)
        message = Image()
        set_header(message.header, stamp, SOURCE_FRAME)
        message.height = IMAGE_HEIGHT
        message.width = IMAGE_WIDTH
        message.encoding = '16UC1'
        message.is_bigendian = False
        message.step = IMAGE_WIDTH * image.dtype.itemsize
        message.data = image.tobytes()
        self.depth_publisher.publish(message)
        assert self.spin_until(lambda: self.estimator.latest_depth_header is not None)

    def publish_detection(self, payload):
        typed_start = len(self.typed_messages)
        self.detections_publisher.publish(String(data=json.dumps(payload)))
        assert self.spin_until(lambda: len(self.typed_messages) > typed_start)
        return self.typed_messages[-1]


@pytest.fixture
def harness_factory():
    harnesses = []

    def create(**kwargs):
        harness = TypedPublisherHarness(**kwargs)
        harnesses.append(harness)
        return harness

    yield create

    for harness in reversed(harnesses):
        harness.close()


def static_transform(parent, child, x, y, z):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation.w = 1.0
    return transform


def set_header(header, stamp, frame_id):
    header.stamp.sec = stamp['sec']
    header.stamp.nanosec = stamp.get('nanosec', 0)
    header.frame_id = frame_id


def payload(stamp, class_id=0, class_name='red_block', is_block=True):
    return {
        'schema_version': 1,
        'model_version': 'imdl_yolo_seg_v1',
        'timestamp': stamp,
        'frame_id': SOURCE_FRAME,
        'image_size': {'width': IMAGE_WIDTH, 'height': IMAGE_HEIGHT},
        'inference_ms': 12.5,
        'detections': [{
            'class_id': class_id,
            'class_name': class_name,
            'is_block': is_block,
            'confidence': 0.9,
            'bbox_xyxy_px': [4.0, 4.0, 12.0, 12.0],
            'center_px': {'u': 8.0, 'v': 8.0},
            'mask_polygon_px': [[4.0, 4.0], [12.0, 4.0], [12.0, 12.0], [4.0, 12.0]],
        }],
    }


def test_normal_block_publishes_metric_typed_object(harness_factory):
    harness = harness_factory()
    stamp = {'sec': 10, 'nanosec': 20}
    harness.publish_camera_info(stamp)
    harness.publish_depth(stamp)
    typed = harness.publish_detection(payload(stamp))

    assert typed.header.frame_id == SOURCE_FRAME
    assert typed.header.stamp.sec == 10
    assert typed.header.stamp.nanosec == 20
    assert len(typed.objects) == 1
    object_message = typed.objects[0]
    assert object_message.header.frame_id == SOURCE_FRAME
    assert object_message.class_name == 'red_block'
    assert object_message.is_block
    assert object_message.depth_valid
    assert object_message.base_pose_valid
    assert object_message.map_pose_valid
    assert object_message.transform_status == 'ok'
    assert object_message.depth_m == pytest.approx(1.0)
    assert object_message.center_u_px == pytest.approx(8.0)
    assert list(object_message.bbox_xyxy_px) == pytest.approx([4.0, 4.0, 12.0, 12.0])
    assert object_message.position_base_m.z == pytest.approx(1.0)
    assert object_message.position_map_m.x == pytest.approx(0.98)
    assert object_message.position_map_m.y == pytest.approx(1.98)


def test_missing_depth_keeps_object_but_marks_metric_fields_invalid(harness_factory):
    harness = harness_factory()
    stamp = {'sec': 11, 'nanosec': 0}
    harness.publish_camera_info(stamp)
    typed = harness.publish_detection(payload(stamp))

    object_message = typed.objects[0]
    assert not object_message.depth_valid
    assert not object_message.base_pose_valid
    assert not object_message.map_pose_valid
    assert object_message.transform_status == 'depth_unavailable'


def test_missing_tf_preserves_depth_but_marks_base_and_map_invalid(harness_factory):
    harness = harness_factory(publish_transforms=False)
    stamp = {'sec': 12, 'nanosec': 0}
    harness.publish_camera_info(stamp)
    harness.publish_depth(stamp)
    typed = harness.publish_detection(payload(stamp))

    object_message = typed.objects[0]
    assert object_message.depth_valid
    assert not object_message.base_pose_valid
    assert not object_message.map_pose_valid
    assert object_message.transform_status == 'missing_base_and_map_tf'


def test_unsynchronised_depth_is_rejected_before_tf_projection(harness_factory):
    harness = harness_factory()
    detection_stamp = {'sec': 13, 'nanosec': 0}
    harness.publish_camera_info(detection_stamp)
    harness.publish_depth({'sec': 12, 'nanosec': 700_000_000})
    typed = harness.publish_detection(payload(detection_stamp))

    object_message = typed.objects[0]
    assert not object_message.depth_valid
    assert not object_message.base_pose_valid
    assert not object_message.map_pose_valid
    assert object_message.transform_status == 'depth_unsynchronised'


def test_blue_obstacle_is_not_a_block_but_retains_valid_navigation_geometry(
    harness_factory,
):
    harness = harness_factory()
    stamp = {'sec': 14, 'nanosec': 0}
    harness.publish_camera_info(stamp)
    harness.publish_depth(stamp)
    typed = harness.publish_detection(
        payload(stamp, class_id=4, class_name='blue_obstacle', is_block=False)
    )

    object_message = typed.objects[0]
    assert object_message.class_name == 'blue_obstacle'
    assert not object_message.is_block
    assert object_message.depth_valid
    assert object_message.base_pose_valid
    assert object_message.map_pose_valid
    assert object_message.transform_status == 'ok'
