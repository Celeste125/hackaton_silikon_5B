"""Validated Gazebo-only pick, carry, and place service."""

from __future__ import annotations

import math
import threading

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from imdl_interfaces.msg import TrackedObject, TrackedObjectArray
from imdl_interfaces.srv import ManipulateObject
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .placed_objects import PLACED_OBJECT_EVENT_TOPIC, placement_event_json
from .ros_shutdown import shutdown_node, spin_node_resilient


def pick_geometry(robot_pose, object_x, object_y):
    """Return distance and normalized bearing from robot to object."""
    distance = math.hypot(object_x - robot_pose[0], object_y - robot_pose[1])
    bearing = math.atan2(
        object_y - robot_pose[1],
        object_x - robot_pose[0],
    ) - robot_pose[2]
    return distance, math.atan2(math.sin(bearing), math.cos(bearing))


def placement_height(block_size_m, stack_index):
    """Return a block-center height for a flush vertical stack."""
    size = max(0.001, float(block_size_m))
    return size * 0.5 + max(0, int(stack_index)) * size


class SimulationManipulatorNode(Node):

    def __init__(self):
        super().__init__('simulation_manipulator')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pick_max_distance_m', 0.34)
        self.declare_parameter('pick_max_bearing_rad', 0.10)
        self.declare_parameter('pick_target_tolerance_m', 0.08)
        self.declare_parameter('entity_track_tolerance_m', 0.06)
        self.declare_parameter('place_max_distance_m', 0.36)
        self.declare_parameter('place_max_bearing_rad', 0.25)
        self.declare_parameter('carry_offset_x_m', -0.08)
        self.declare_parameter('carry_height_m', 0.16)
        self.declare_parameter('block_size_m', 0.035)
        self.declare_parameter('service_timeout_sec', 2.0)
        self.declare_parameter('entity_set_retries', 3)
        self.declare_parameter('entity_pose_tolerance_m', 0.012)
        self.declare_parameter('entity_yaw_tolerance_rad', 0.05)
        self.declare_parameter(
            'placed_object_event_topic',
            PLACED_OBJECT_EVENT_TOPIC,
        )
        self.declare_parameter('placed_obstacle_exclusion_radius_m', 0.075)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.io_callback_group = ReentrantCallbackGroup()
        self.manipulation_callback_group = MutuallyExclusiveCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_entity_client = self.create_client(
            SetEntityState,
            '/set_entity_state',
            callback_group=self.io_callback_group,
        )
        self.get_entity_client = self.create_client(
            GetEntityState,
            '/get_entity_state',
            callback_group=self.io_callback_group,
        )
        self.tracks = {}
        self.carried = None
        self.pending_carry_update = None
        self.carry_updates_paused = False
        self.carry_update_done = threading.Event()
        self.carry_update_done.set()
        self.create_subscription(
            TrackedObjectArray,
            '/tracked_objects',
            self._tracks_callback,
            10,
            callback_group=self.io_callback_group,
        )
        self.placed_event_publisher = self.create_publisher(
            String,
            str(self.get_parameter('placed_object_event_topic').value),
            10,
        )
        self.service = self.create_service(
            ManipulateObject,
            '/autonomy/manipulate_object',
            self._manipulate,
            callback_group=self.manipulation_callback_group,
        )
        self.create_timer(
            0.1,
            self._carry_tick,
            callback_group=self.io_callback_group,
        )
        self.get_logger().info('simulation_manipulator ready (Gazebo only)')

    def _tracks_callback(self, message: TrackedObjectArray) -> None:
        self.tracks = {item.object_id: item for item in message.objects}

    def _manipulate(self, request, response):
        if request.operation == ManipulateObject.Request.PICK:
            return self._pick(request, response)
        if request.operation == ManipulateObject.Request.PLACE:
            return self._place(request, response)
        if request.operation == ManipulateObject.Request.RELEASE:
            return self._release(request, response)
        return self._error(
            response,
            ManipulateObject.Response.INVALID_REQUEST,
            f'Unknown operation {request.operation}',
        )

    def _pick(self, request, response):
        if self.carried is not None:
            return self._error(
                response,
                ManipulateObject.Response.INVALID_REQUEST,
                f'Already carrying {self.carried[0]}',
            )
        track = self.tracks.get(request.object_id)
        if (
            track is None
            or track.color != request.color
            or track.status != TrackedObject.STATUS_CONFIRMED
        ):
            return self._error(
                response,
                ManipulateObject.Response.OBJECT_NOT_CONFIRMED,
                f'Track {request.object_id!r} is not confirmed',
            )
        robot = self._robot_pose()
        if robot is None:
            return self._error(
                response,
                ManipulateObject.Response.INTERNAL_ERROR,
                f'TF {self.map_frame}->{self.base_frame} unavailable',
            )
        x = track.pose.pose.position.x
        y = track.pose.pose.position.y
        requested = request.target_pose.pose.position
        target_error = math.hypot(requested.x - x, requested.y - y)
        if target_error > float(
            self.get_parameter('pick_target_tolerance_m').value
        ):
            return self._error(
                response,
                ManipulateObject.Response.OBJECT_NOT_CONFIRMED,
                f'Pick target differs from confirmed track by '
                f'{target_error:.3f}m',
            )
        distance, bearing = pick_geometry(robot, x, y)
        if (
            distance > float(self.get_parameter('pick_max_distance_m').value)
            or abs(bearing) > float(
                self.get_parameter('pick_max_bearing_rad').value
            )
        ):
            return self._error(
                response,
                ManipulateObject.Response.OUT_OF_REACH,
                f'Object out of reach: distance={distance:.3f}m, '
                f'bearing={bearing:.3f}rad',
            )

        entity = f'{request.color}_block'
        actual = self._get_entity_sync(entity)
        if actual is None:
            return self._error(
                response,
                ManipulateObject.Response.GAZEBO_UNAVAILABLE,
                f'Gazebo state for {entity} is unavailable',
            )
        entity_track_error = math.hypot(actual.position.x - x, actual.position.y - y)
        if entity_track_error > float(
            self.get_parameter('entity_track_tolerance_m').value
        ):
            return self._error(
                response,
                ManipulateObject.Response.OBJECT_NOT_CONFIRMED,
                f'{entity} differs from camera track by '
                f'{entity_track_error:.3f}m',
            )
        carry_pose = self._carry_pose(robot)
        if not self._set_entity_sync(entity, *carry_pose):
            return self._error(
                response,
                ManipulateObject.Response.GAZEBO_UNAVAILABLE,
                f'Gazebo could not attach {entity}',
            )
        original_pose = (
            actual.position.x,
            actual.position.y,
            actual.position.z,
            self._yaw(actual.orientation),
        )
        self.carried = (
            request.object_id,
            request.color,
            entity,
            original_pose,
        )
        response.success = True
        response.error_code = ManipulateObject.Response.OK
        response.message = (
            f'PICK confirmed: {request.object_id}, '
            f'distance={distance:.3f}m, bearing={bearing:.3f}rad'
        )
        self.get_logger().info(response.message)
        return response

    def _place(self, request, response):
        if self.carried is None:
            return self._error(
                response,
                ManipulateObject.Response.NOT_CARRYING,
                'PLACE requested while no object is carried',
            )
        object_id, color, entity, _original_pose = self.carried
        if request.object_id != object_id or request.color != color:
            return self._error(
                response,
                ManipulateObject.Response.INVALID_REQUEST,
                f'Carrying {object_id}/{color}, not '
                f'{request.object_id}/{request.color}',
            )
        pose = request.target_pose.pose
        robot = self._robot_pose()
        if robot is None:
            return self._error(
                response,
                ManipulateObject.Response.INTERNAL_ERROR,
                f'TF {self.map_frame}->{self.base_frame} unavailable',
            )
        distance, bearing = pick_geometry(
            robot,
            pose.position.x,
            pose.position.y,
        )
        if (
            distance > float(self.get_parameter('place_max_distance_m').value)
            or abs(bearing) > float(
                self.get_parameter('place_max_bearing_rad').value
            )
        ):
            return self._error(
                response,
                ManipulateObject.Response.OUT_OF_REACH,
                f'Place center out of reach: distance={distance:.3f}m, '
                f'bearing={bearing:.3f}rad',
            )
        self.carry_updates_paused = True
        if not self.carry_update_done.wait(
            float(self.get_parameter('service_timeout_sec').value)
        ):
            self.carry_updates_paused = False
            return self._error(
                response,
                ManipulateObject.Response.GAZEBO_UNAVAILABLE,
                'Timed out waiting for the final carried-object update',
            )
        block_size = float(self.get_parameter('block_size_m').value)
        z = placement_height(block_size, request.stack_index)
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        if not self._set_entity_sync(
            entity,
            pose.position.x,
            pose.position.y,
            z,
            yaw,
        ):
            self.carry_updates_paused = False
            return self._error(
                response,
                ManipulateObject.Response.GAZEBO_UNAVAILABLE,
                f'Gazebo could not place {entity}',
            )
        self.carried = None
        self.carry_updates_paused = False
        self.placed_event_publisher.publish(String(data=placement_event_json(
            object_id=object_id,
            color=color,
            frame_id=self.map_frame,
            x=pose.position.x,
            y=pose.position.y,
            radius_m=float(
                self.get_parameter(
                    'placed_obstacle_exclusion_radius_m'
                ).value
            ),
        )))
        response.success = True
        response.error_code = ManipulateObject.Response.OK
        response.message = (
            f'PLACE confirmed: {object_id} at '
            f'({pose.position.x:.3f}, {pose.position.y:.3f}, {z:.3f})'
        )
        self.get_logger().info(response.message)
        return response

    def _release(self, _request, response):
        """Rollback an unfinished simulated carry so another trial can start."""
        if self.carried is None:
            response.success = True
            response.error_code = ManipulateObject.Response.OK
            response.message = 'RELEASE confirmed: no object was carried'
            return response
        object_id, color, entity, original_pose = self.carried
        self.carry_updates_paused = True
        if not self.carry_update_done.wait(
            float(self.get_parameter('service_timeout_sec').value)
        ):
            self.carry_updates_paused = False
            return self._error(
                response,
                ManipulateObject.Response.GAZEBO_UNAVAILABLE,
                'Timed out waiting to release carried object',
            )
        if not self._set_entity_sync(entity, *original_pose):
            self.carry_updates_paused = False
            return self._error(
                response,
                ManipulateObject.Response.GAZEBO_UNAVAILABLE,
                f'Gazebo could not restore {entity}',
            )
        self.carried = None
        self.carry_updates_paused = False
        response.success = True
        response.error_code = ManipulateObject.Response.OK
        response.message = (
            f'RELEASE confirmed: restored {object_id}/{color} to its '
            'pre-pick pose'
        )
        self.get_logger().info(response.message)
        return response

    def _carry_tick(self) -> None:
        if (
            self.carried is None
            or self.carry_updates_paused
            or self.pending_carry_update is not None
        ):
            return
        robot = self._robot_pose()
        if robot is None or not self.set_entity_client.service_is_ready():
            return
        _, _, entity, _original_pose = self.carried
        x, y, z, yaw = self._carry_pose(robot)
        request = self._set_request(entity, x, y, z, yaw)
        self.carry_update_done.clear()
        self.pending_carry_update = self.set_entity_client.call_async(request)
        self.pending_carry_update.add_done_callback(self._carry_update_done)

    def _carry_update_done(self, future) -> None:
        try:
            response = future.result()
            if response is None or not response.success:
                self.get_logger().error(
                    'Gazebo rejected a carried-object pose update',
                    throttle_duration_sec=2.0,
                )
        except Exception as error:
            self.get_logger().error(
                f'Carried-object update failed: {error}',
                throttle_duration_sec=2.0,
            )
        self.pending_carry_update = None
        self.carry_update_done.set()

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (
                rotation.w * rotation.z + rotation.x * rotation.y
            ),
            1.0 - 2.0 * (
                rotation.y * rotation.y + rotation.z * rotation.z
            ),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def _carry_pose(self, robot):
        offset = float(self.get_parameter('carry_offset_x_m').value)
        return (
            robot[0] + offset * math.cos(robot[2]),
            robot[1] + offset * math.sin(robot[2]),
            float(self.get_parameter('carry_height_m').value),
            robot[2],
        )

    @staticmethod
    def _yaw(orientation):
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _set_request(self, entity, x, y, z, yaw):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = entity
        request.state.reference_frame = 'world'
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.z = math.sin(yaw * 0.5)
        request.state.pose.orientation.w = math.cos(yaw * 0.5)
        return request

    def _set_entity_sync(self, entity, x, y, z, yaw=0.0):
        if not self.set_entity_client.service_is_ready():
            return False
        retries = max(1, int(self.get_parameter('entity_set_retries').value))
        tolerance = max(
            0.001,
            float(self.get_parameter('entity_pose_tolerance_m').value),
        )
        yaw_tolerance = max(
            0.001,
            float(self.get_parameter('entity_yaw_tolerance_rad').value),
        )
        for attempt in range(1, retries + 1):
            response = self._call_sync(
                self.set_entity_client,
                self._set_request(entity, x, y, z, yaw),
            )
            if response is None or not response.success:
                self.get_logger().warn(
                    f'Gazebo rejected pose for {entity} '
                    f'(attempt {attempt}/{retries})'
                )
                continue
            actual = self._get_entity_sync(entity)
            if actual is None:
                # SetEntityState itself is authoritative if an older Gazebo
                # installation does not expose GetEntityState.
                if not self.get_entity_client.service_is_ready():
                    return True
                continue
            error = math.sqrt(
                (actual.position.x - float(x)) ** 2
                + (actual.position.y - float(y)) ** 2
                + (actual.position.z - float(z)) ** 2
            )
            actual_yaw = self._yaw(actual.orientation)
            yaw_error = abs(math.atan2(
                math.sin(actual_yaw - float(yaw)),
                math.cos(actual_yaw - float(yaw)),
            ))
            if error <= tolerance and yaw_error <= yaw_tolerance:
                return True
            self.get_logger().warn(
                f'Gazebo pose verification for {entity} differs by '
                f'{error:.4f}m/{yaw_error:.4f}rad '
                f'(attempt {attempt}/{retries})'
            )
        return False

    def _get_entity_sync(self, entity):
        if not self.get_entity_client.service_is_ready():
            return None
        request = GetEntityState.Request()
        request.name = entity
        request.reference_frame = 'world'
        response = self._call_sync(self.get_entity_client, request)
        if response is None or not response.success:
            return None
        return response.state.pose

    def _call_sync(self, client, request):
        event = threading.Event()
        result = {'response': None}
        future = client.call_async(request)

        def complete(completed):
            try:
                result['response'] = completed.result()
            except Exception:
                result['response'] = None
            finally:
                event.set()

        future.add_done_callback(complete)
        completed = event.wait(
            float(self.get_parameter('service_timeout_sec').value)
        )
        if not completed:
            future.cancel()
            return None
        return result['response']

    @staticmethod
    def _error(response, code, message):
        response.success = False
        response.error_code = code
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimulationManipulatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        spin_node_resilient(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_node(node, executor)
