"""Small runtime stores and service clients used by the mission manager."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import json
import math
from statistics import median
import time
from typing import Dict, Iterable, Optional

from imdl_interfaces.msg import TrackedObject, TrackedObjectArray
from imdl_interfaces.srv import ManipulateObject
from imdl_msgs.msg import DetectedObjectArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Range
from std_msgs.msg import Bool, String

from .mission_spec import Pose2D
from .navigation_adapter import pose_stamped
from .range_fusion import FrontRangeReading


@dataclass(frozen=True)
class ObjectTrackView:
    object_id: str
    color: str
    pose: Pose2D
    z: float
    covariance: tuple
    confidence: float
    age_sec: float
    confirmations: int
    status: int


@dataclass(frozen=True)
class CameraObservation:
    """One fresh color detection expressed relative to the optical center."""

    color: str
    cx: float
    cy: float
    width: float
    height: float
    principal_x: float
    focal_x: float
    generation: int
    received_wall: float
    stamp_sec: Optional[float] = None
    source: str = 'typed'
    confidence: float = 1.0
    depth_m: Optional[float] = None
    range_m: Optional[float] = None
    bearing_rad: Optional[float] = None
    base_x_m: Optional[float] = None
    base_y_m: Optional[float] = None

    @property
    def horizontal_error_rad(self) -> float:
        # Positive ROS yaw turns left; pixels left of the principal point
        # therefore require a positive correction.
        return math.atan2(self.principal_x - self.cx, self.focal_x)


class CameraObservationStore:
    """Store grasp-valid detections from the shared typed perception contract."""

    def __init__(
        self,
        node,
        typed_topic: str = '/perception/detected_objects',
        minimum_confidence: float = 0.35,
    ):
        self._camera_info = None
        self._generation = 0
        self._observations: Dict[str, CameraObservation] = {}
        self._minimum_confidence = float(minimum_confidence)
        node.create_subscription(
            DetectedObjectArray,
            str(typed_topic),
            self._typed_detections_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _typed_detections_callback(
        self,
        message: DetectedObjectArray,
    ) -> None:
        self._generation += 1
        if self._camera_info is None:
            return
        focal_x = float(self._camera_info.k[0])
        principal_x = float(self._camera_info.k[2])
        if focal_x <= 0.0:
            return

        received_wall = time.monotonic()
        best = {}
        for item in message.objects:
            if not is_control_candidate(item, self._minimum_confidence):
                continue
            color = block_color(item.class_name, item.is_block)
            bbox = item.bbox_xyxy_px
            width = float(bbox[2]) - float(bbox[0])
            height = float(bbox[3]) - float(bbox[1])
            values = (
                float(item.center_u_px),
                float(item.center_v_px),
                width,
                height,
                float(item.depth_m),
                float(item.range_m),
                float(item.bearing_rad),
                float(item.position_base_m.x),
                float(item.position_base_m.y),
            )
            if (
                not all(math.isfinite(value) for value in values)
                or width <= 0.0
                or height <= 0.0
                or item.depth_m <= 0.0
                or item.range_m <= 0.0
            ):
                continue
            candidate = CameraObservation(
                color=color,
                cx=float(item.center_u_px),
                cy=float(item.center_v_px),
                width=width,
                height=height,
                principal_x=principal_x,
                focal_x=focal_x,
                generation=self._generation,
                received_wall=received_wall,
                stamp_sec=stamp_seconds(item.header, message.header),
                source='typed',
                confidence=float(item.confidence),
                depth_m=float(item.depth_m),
                range_m=float(item.range_m),
                bearing_rad=float(item.bearing_rad),
                base_x_m=float(item.position_base_m.x),
                base_y_m=float(item.position_base_m.y),
            )
            current = best.get(color)
            if (
                current is None
                or candidate.confidence > current.confidence
                or (
                    candidate.confidence == current.confidence
                    and width * height > current.width * current.height
                )
            ):
                best[color] = candidate
        self._observations.update(best)

    def observation(
        self,
        color: str,
        max_age_sec: float = 0.75,
    ) -> Optional[CameraObservation]:
        observation = self._observations.get(str(color).strip().lower())
        if observation is None:
            return None
        if time.monotonic() - observation.received_wall > max(0.0, max_age_sec):
            return None
        return observation


class FrontRangeStore:
    """Keep a short, median-filtered history of front ultrasound samples."""

    def __init__(self, node, topic: str = '/ultrasonic/front/range'):
        self.received = False
        self._generation = 0
        self._samples = deque(maxlen=80)
        node.create_subscription(
            Range,
            str(topic),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message: Range) -> None:
        self.received = True
        self._generation += 1
        value = float(message.range)
        minimum = float(message.min_range)
        maximum = float(message.max_range)
        if (
            not math.isfinite(value)
            or not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum < 0.0
            or maximum <= minimum
            or value < minimum
            or value > maximum
        ):
            return
        stamp_sec = None
        stamp = getattr(message.header, 'stamp', None)
        if stamp is not None:
            seconds = int(getattr(stamp, 'sec', 0))
            nanoseconds = int(getattr(stamp, 'nanosec', 0))
            if seconds >= 0 and 0 <= nanoseconds < 1_000_000_000:
                stamp_sec = seconds + nanoseconds * 1e-9
        self._samples.append(FrontRangeReading(
            range_m=value,
            min_range_m=minimum,
            max_range_m=maximum,
            field_of_view_rad=float(message.field_of_view),
            frame_id=str(message.header.frame_id),
            generation=self._generation,
            sample_count=1,
            received_wall=time.monotonic(),
            stamp_sec=stamp_sec,
        ))

    def measurement(
        self,
        max_age_sec: float = 0.35,
        min_samples: int = 3,
        not_before_wall: Optional[float] = None,
    ) -> Optional[FrontRangeReading]:
        """Return a fresh median only after the requested independent samples."""
        now = time.monotonic()
        oldest = now - max(0.0, float(max_age_sec))
        if not_before_wall is not None:
            oldest = max(oldest, float(not_before_wall))
        samples = [
            sample for sample in self._samples
            if sample.received_wall >= oldest
        ]
        required = max(1, int(min_samples))
        if len(samples) < required:
            return None
        latest = samples[-1]
        values = [sample.range_m for sample in samples]
        return FrontRangeReading(
            range_m=float(median(values)),
            min_range_m=latest.min_range_m,
            max_range_m=latest.max_range_m,
            field_of_view_rad=latest.field_of_view_rad,
            frame_id=latest.frame_id,
            generation=latest.generation,
            sample_count=len(samples),
            received_wall=latest.received_wall,
            stamp_sec=latest.stamp_sec,
        )


class ObjectStore:

    def __init__(self, node):
        self.tracks: Dict[str, ObjectTrackView] = {}
        self.confirmed_cache = {}
        self.locked_ids = set()
        node.create_subscription(
            TrackedObjectArray,
            '/tracked_objects',
            self._callback,
            10,
        )

    def _callback(self, message: TrackedObjectArray) -> None:
        current = {}
        for item in message.objects:
            orientation = item.pose.pose.orientation
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
            view = ObjectTrackView(
                object_id=item.object_id,
                color=item.color,
                pose=Pose2D(
                    item.pose.pose.position.x,
                    item.pose.pose.position.y,
                    yaw,
                ),
                z=item.pose.pose.position.z,
                covariance=tuple(item.pose.covariance),
                confidence=float(item.confidence),
                age_sec=float(item.age_sec),
                confirmations=int(item.confirmations),
                status=int(item.status),
            )
            current[item.object_id] = view
            if (
                view.status == TrackedObject.STATUS_CONFIRMED
                and view.object_id not in self.locked_ids
            ):
                self.confirmed_cache[view.object_id] = (view, time.monotonic())
        self.tracks = current

    def confirmed(
        self,
        color: str,
        reference: Optional[Pose2D] = None,
        max_age_sec: float = 2.0,
        min_confidence: float = 0.42,
    ) -> Optional[ObjectTrackView]:
        candidates = [
            track for track in self.tracks.values()
            if track.object_id not in self.locked_ids
            and track.color == color
            and track.status == TrackedObject.STATUS_CONFIRMED
            and track.age_sec <= max_age_sec
            and track.confidence >= min_confidence
        ]
        if not candidates:
            return None
        if reference is None:
            return max(candidates, key=lambda item: item.confidence)
        return min(candidates, key=lambda item: item.pose.distance_to(reference))

    def lock(self, object_id: str) -> None:
        self.locked_ids.add(object_id)
        self.confirmed_cache.pop(object_id, None)

    def forget_cached(self, object_id: str) -> None:
        """Discard one stale hypothesis without permanently locking its ID."""
        self.confirmed_cache.pop(object_id, None)

    def cached(
        self,
        color: str,
        reference: Optional[Pose2D] = None,
        max_cache_age_sec: float = 300.0,
    ) -> Optional[ObjectTrackView]:
        """Return a previously confirmed pose as a navigation hypothesis.

        A stale cache entry is never sufficient for PICK. TransportFlow always
        obtains a fresh confirmed track and centered image at the object.
        """
        now = time.monotonic()
        candidates = [
            view for object_id, (view, stored_wall) in self.confirmed_cache.items()
            if object_id not in self.locked_ids
            and view.color == color
            and now - stored_wall <= max(0.0, max_cache_age_sec)
        ]
        if not candidates:
            return None
        if reference is None:
            return max(candidates, key=lambda item: item.confidence)
        return min(candidates, key=lambda item: item.pose.distance_to(reference))

    def all_visible(self) -> Iterable[ObjectTrackView]:
        return tuple(self.tracks.values())


class LocalizationStore:

    def __init__(self, node):
        self.received = False
        self.mission_id = ''
        self.ready = False
        self.scan_required = False
        self.reasons = []
        self.pose: Optional[Pose2D] = None
        self.position_std_m = math.inf
        qos = node.create_subscription(
            String,
            '/autonomy/localization_state',
            self._callback,
            10,
        )
        self.subscription = qos

    def _callback(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.received = True
        self.mission_id = str(state.get('mission_id', ''))
        self.ready = bool(state.get('ready', False))
        self.scan_required = bool(state.get('scan_required', False))
        self.reasons = list(state.get('reasons', []))
        self.position_std_m = float(state.get('position_std_m', math.inf))
        pose = state.get('pose')
        if isinstance(pose, dict):
            try:
                self.pose = Pose2D(
                    float(pose['x']),
                    float(pose['y']),
                    float(pose['yaw']),
                )
            except (KeyError, TypeError, ValueError):
                pass


class SafetyStore:

    def __init__(self, node):
        self.received = False
        self.safe = False
        self.reasons = ['SAFETY_STATE_MISSING']
        node.create_subscription(Bool, '/autonomy/safety_stop', self._stop_cb, 10)
        node.create_subscription(String, '/autonomy/safety_state', self._state_cb, 10)

    def _stop_cb(self, message: Bool) -> None:
        self.received = True
        self.safe = not message.data

    def _state_cb(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.received = True
        self.safe = bool(state.get('safe', False))
        self.reasons = list(state.get('reasons', []))


class ManipulatorState(str, Enum):
    IDLE = 'IDLE'
    ACTIVE = 'ACTIVE'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    TIMED_OUT = 'TIMED_OUT'
    CANCELED = 'CANCELED'


class ManipulatorClient:

    def __init__(self, node, frame_id: str):
        self.node = node
        self.frame_id = frame_id
        self.client = node.create_client(
            ManipulateObject,
            '/autonomy/manipulate_object',
        )
        self.state = ManipulatorState.IDLE
        self.message = ''
        self.error_code = 0
        self._deadline = math.inf
        self._generation = 0
        self._active_operation = 0
        self._active_object_id = ''
        self._active_color = ''
        self._carried_object_id = ''
        self._carried_color = ''

    def available(self) -> bool:
        return self.client.service_is_ready()

    def reset(self) -> None:
        if self.state is ManipulatorState.ACTIVE:
            raise RuntimeError('Cannot reset active manipulation')
        self.state = ManipulatorState.IDLE
        self.message = ''
        self.error_code = 0

    def pick(
        self,
        object_id: str,
        color: str,
        target: Pose2D,
        timeout_sec: float = 5.0,
    ) -> bool:
        return self._request(
            ManipulateObject.Request.PICK,
            object_id,
            color,
            target,
            0,
            timeout_sec,
        )

    def place(
        self,
        object_id: str,
        color: str,
        target: Pose2D,
        stack_index: int,
        timeout_sec: float = 5.0,
    ) -> bool:
        return self._request(
            ManipulateObject.Request.PLACE,
            object_id,
            color,
            target,
            stack_index,
            timeout_sec,
        )

    def tick(self) -> ManipulatorState:
        if self.state is ManipulatorState.ACTIVE and time.monotonic() > self._deadline:
            self._generation += 1
            self.state = ManipulatorState.TIMED_OUT
            self.message = 'Manipulation service timed out'
            self._request_release_cleanup()
        return self.state

    def cancel(self) -> None:
        self._generation += 1
        if self.state is ManipulatorState.ACTIVE:
            self.state = ManipulatorState.CANCELED
            self.message = 'Manipulation request cancelled locally'
        self._request_release_cleanup()

    def _request(
        self,
        operation: int,
        object_id: str,
        color: str,
        target: Pose2D,
        stack_index: int,
        timeout_sec: float,
    ) -> bool:
        if self.state is ManipulatorState.ACTIVE:
            return False
        if not self.client.service_is_ready():
            self.state = ManipulatorState.FAILED
            self.message = 'MANIPULATOR_UNAVAILABLE'
            return False
        request = ManipulateObject.Request()
        request.operation = operation
        request.object_id = object_id
        request.color = color
        request.target_pose = pose_stamped(self.node, target, self.frame_id)
        request.stack_index = max(0, int(stack_index))
        self._generation += 1
        generation = self._generation
        self.state = ManipulatorState.ACTIVE
        self.message = 'request sent'
        self.error_code = 0
        self._active_operation = int(operation)
        self._active_object_id = str(object_id)
        self._active_color = str(color)
        self._deadline = time.monotonic() + max(0.1, timeout_sec)
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda completed: self._result(completed, generation, operation)
        )
        return True

    def _result(self, future, generation: int, operation: int) -> None:
        if generation != self._generation:
            return
        try:
            response = future.result()
        except Exception as error:
            self.state = ManipulatorState.FAILED
            self.message = f'Manipulation service error: {error}'
            return
        if response is not None and response.success:
            self.state = ManipulatorState.SUCCEEDED
            if operation == ManipulateObject.Request.PICK:
                self._carried_object_id = self._active_object_id
                self._carried_color = self._active_color
            elif operation in {
                ManipulateObject.Request.PLACE,
                ManipulateObject.Request.RELEASE,
            }:
                self._carried_object_id = ''
                self._carried_color = ''
        else:
            self.state = ManipulatorState.FAILED
        if response is not None:
            self.message = response.message
            self.error_code = int(response.error_code)
        self._active_operation = 0

    def _request_release_cleanup(self) -> None:
        """Best-effort rollback queued after any canceled manipulation."""
        try:
            service_ready = self.client.service_is_ready()
        except Exception:
            # During SIGINT the context can be invalid before destroy_node()
            # requests its best-effort cleanup.
            return
        if not service_ready:
            return
        request = ManipulateObject.Request()
        request.operation = ManipulateObject.Request.RELEASE
        request.object_id = (
            self._carried_object_id or self._active_object_id
        )
        request.color = self._carried_color or self._active_color
        try:
            future = self.client.call_async(request)
        except Exception:
            return
        future.add_done_callback(self._release_result)

    def _release_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.node.get_logger().error(
                f'Manipulator cleanup service error: {error}'
            )
            return
        if response is None or not response.success:
            detail = 'no response' if response is None else response.message
            self.node.get_logger().error(
                f'Manipulator cleanup failed: {detail}'
            )
            return
        self._carried_object_id = ''
        self._carried_color = ''
        self.node.get_logger().info(response.message)
