"""Non-blocking Nav2 action adapter for the autonomy mission state machines."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import (ComputePathToPose, FollowPath, NavigateToPose,
                              Spin)
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from .mission_spec import Pose2D


NAV2_LIFECYCLE_NODES = (
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
)


class NavigationState(str, Enum):
    IDLE = 'IDLE'
    SENDING = 'SENDING'
    ACTIVE = 'ACTIVE'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'
    TIMED_OUT = 'TIMED_OUT'


@dataclass(frozen=True)
class NavigationSnapshot:
    state: NavigationState
    operation: str
    message: str
    distance_remaining: Optional[float]
    recoveries: int

    @property
    def terminal(self) -> bool:
        return self.state in {
            NavigationState.SUCCEEDED,
            NavigationState.FAILED,
            NavigationState.CANCELED,
            NavigationState.TIMED_OUT,
        }


@dataclass
class VelocityStats:
    samples: int = 0
    minimum: float = math.inf
    maximum: float = -math.inf
    total: float = 0.0
    nonzero_samples: int = 0

    def observe(self, value: float) -> None:
        self.samples += 1
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)
        self.total += value
        if abs(value) > 0.005:
            self.nonzero_samples += 1

    @property
    def mean(self) -> float:
        return self.total / self.samples if self.samples else 0.0

    def summary(self) -> str:
        if not self.samples:
            return 'n=0'
        return (
            f'n={self.samples},min={self.minimum:.3f},'
            f'max={self.maximum:.3f},mean={self.mean:.3f},'
            f'active={self.nonzero_samples}'
        )


@dataclass
class MotionStats:
    """Signed linear and angular velocity statistics for one command stage."""

    linear: VelocityStats = field(default_factory=VelocityStats)
    angular: VelocityStats = field(default_factory=VelocityStats)

    def observe(self, linear_x: float, angular_z: float) -> None:
        self.linear.observe(linear_x)
        self.angular.observe(angular_z)


def pose_stamped(node, pose: Pose2D, frame_id: str) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = float(pose.x)
    message.pose.position.y = float(pose.y)
    message.pose.orientation.z = math.sin(pose.yaw * 0.5)
    message.pose.orientation.w = math.cos(pose.yaw * 0.5)
    return message


def reverse_path(path: Path, stamp=None) -> Path:
    """Reverse path geometry while retaining forward-facing orientations."""
    result = Path()
    result.header = path.header
    if stamp is not None:
        result.header.stamp = stamp
    for original in reversed(path.poses):
        pose = PoseStamped()
        pose.header = original.header
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose = original.pose
        result.poses.append(pose)
    return result


def path_length(path: Path) -> float:
    """Return the planar arc length of a ROS path."""
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(path.poses, path.poses[1:])
    )


class NavigationAdapter:
    def __init__(
        self,
        node,
        frame_id: str,
        rpp_behavior_tree: str,
        mppi_behavior_tree: str,
        mppi_mission_behavior_tree: str,
        mppi_precise_behavior_tree: str,
    ):
        self.node = node
        self.frame_id = frame_id
        self.rpp_behavior_tree = rpp_behavior_tree
        self.mppi_behavior_tree = mppi_behavior_tree
        self.mppi_mission_behavior_tree = mppi_mission_behavior_tree
        self.mppi_precise_behavior_tree = mppi_precise_behavior_tree
        self.navigate_client = ActionClient(
            node,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self.follow_client = ActionClient(node, FollowPath, '/follow_path')
        self.compute_client = ActionClient(
            node,
            ComputePathToPose,
            '/compute_path_to_pose',
        )
        self.spin_client = ActionClient(node, Spin, '/spin')
        self.clear_local_client = node.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        self.clear_global_client = node.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self._lifecycle_clients = {
            name: node.create_client(GetState, f'/{name}/get_state')
            for name in NAV2_LIFECYCLE_NODES
        }
        self._lifecycle_futures = {
            name: None for name in NAV2_LIFECYCLE_NODES
        }
        self._active_lifecycle_nodes = set()
        self._last_lifecycle_request = {
            name: -math.inf for name in NAV2_LIFECYCLE_NODES
        }
        self.path_publisher = node.create_publisher(Path, '/autonomy/global_path', 10)
        node.create_subscription(Path, '/plan', self._global_plan_callback, 10)
        node.create_subscription(
            Twist,
            'cmd_vel_feedback',
            lambda message: self._velocity_callback(
                'collision_monitor', message
            ),
            20,
        )
        node.create_subscription(
            Odometry,
            'odom_feedback',
            lambda message: self._velocity_callback(
                'odometry', message.twist.twist
            ),
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Twist,
            '/cmd_vel_nav',
            lambda message: self._velocity_callback('controller', message),
            20,
        )
        node.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            lambda message: self._velocity_callback('smoother', message),
            20,
        )

        self.state = NavigationState.IDLE
        self.operation = ''
        self.message = ''
        self.distance_remaining: Optional[float] = None
        self.recoveries = 0
        self._action_recoveries = 0
        self.result_path: Optional[Path] = None
        self.last_path: Optional[Path] = None
        self.captured_path: Optional[Path] = None
        self._captured_path_length = 0.0
        self._capture_global_plan = False
        self._deadline_wall = math.inf
        self._goal_handle = None
        self._generation = 0
        self._last_cmd = Twist()
        self._reverse_validation = False
        self._reverse_positive_samples = 0
        self._reverse_negative_samples = 0
        self._reverse_velocity_stats = self._new_velocity_stats()
        self._operation_velocity_stats = self._new_velocity_stats()
        self._mission_velocity_stats = self._new_velocity_stats()
        self._latest_velocity = {
            name: (0.0, 0.0) for name in self._operation_velocity_stats
        }

    def servers_ready(self, require_spin: bool = True) -> bool:
        clients = [self.navigate_client, self.follow_client, self.compute_client]
        if require_spin:
            clients.append(self.spin_client)
        return all(client.server_is_ready() for client in clients)

    def lifecycle_ready(self) -> bool:
        """Poll Nav2 lifecycle services without blocking the ROS executor."""
        now = time.monotonic()
        for name, client in self._lifecycle_clients.items():
            if name in self._active_lifecycle_nodes:
                continue

            future = self._lifecycle_futures[name]
            if future is not None:
                if not future.done():
                    continue
                self._lifecycle_futures[name] = None
                try:
                    response = future.result()
                except Exception:
                    response = None
                if (
                    response is not None
                    and response.current_state.id
                    == State.PRIMARY_STATE_ACTIVE
                ):
                    self._active_lifecycle_nodes.add(name)
                    continue

            if (
                client.service_is_ready()
                and now - self._last_lifecycle_request[name] >= 0.25
            ):
                self._last_lifecycle_request[name] = now
                self._lifecycle_futures[name] = client.call_async(
                    GetState.Request()
                )

        return len(self._active_lifecycle_nodes) == len(
            self._lifecycle_clients
        )

    def inactive_lifecycle_nodes(self) -> tuple[str, ...]:
        return tuple(
            name
            for name in NAV2_LIFECYCLE_NODES
            if name not in self._active_lifecycle_nodes
        )

    def stack_ready(self, require_spin: bool = True) -> bool:
        """Require active lifecycle nodes as well as discoverable actions."""
        return (
            self.lifecycle_ready()
            and self.servers_ready(require_spin=require_spin)
        )

    def snapshot(self) -> NavigationSnapshot:
        return NavigationSnapshot(
            state=self.state,
            operation=self.operation,
            message=self.message,
            distance_remaining=self.distance_remaining,
            recoveries=self.recoveries,
        )

    def reset(self) -> None:
        if self.state in {NavigationState.SENDING, NavigationState.ACTIVE}:
            raise RuntimeError('Cannot reset active navigation operation')
        self.state = NavigationState.IDLE
        self.operation = ''
        self.message = ''
        self.distance_remaining = None
        self.result_path = None

    def start_plan_capture(self) -> None:
        """Capture the longest Nav2 plan published by the next navigation."""
        self.captured_path = None
        self._captured_path_length = 0.0
        self._capture_global_plan = True

    def stop_plan_capture(self) -> Optional[Path]:
        self._capture_global_plan = False
        return self.captured_path

    def tick(self) -> NavigationSnapshot:
        if (
            self.state in {NavigationState.SENDING, NavigationState.ACTIVE}
            and time.monotonic() > self._deadline_wall
        ):
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self.state = NavigationState.TIMED_OUT
            self.message = f'{self.operation} exceeded its timeout'
            self._reverse_validation = False
        return self.snapshot()

    def compute_path(
        self,
        goal: Pose2D,
        start: Optional[Pose2D] = None,
        planner_id: str = 'GridBased',
        timeout_sec: float = 8.0,
    ) -> bool:
        request = ComputePathToPose.Goal()
        request.goal = pose_stamped(self.node, goal, self.frame_id)
        request.planner_id = planner_id
        request.use_start = start is not None
        if start is not None:
            request.start = pose_stamped(self.node, start, self.frame_id)
        return self._send(
            self.compute_client,
            request,
            'COMPUTE_PATH',
            timeout_sec,
        )

    def navigate_to(
        self,
        goal: Pose2D,
        controller: str = 'RobustMPPI',
        timeout_sec: float = 75.0,
        mission_pose: bool = False,
        precise: bool = False,
    ) -> bool:
        request = NavigateToPose.Goal()
        request.pose = pose_stamped(self.node, goal, self.frame_id)
        if controller == 'FastRPP':
            request.behavior_tree = self.rpp_behavior_tree
        elif mission_pose:
            request.behavior_tree = self.mppi_mission_behavior_tree
        elif precise:
            request.behavior_tree = self.mppi_precise_behavior_tree
        else:
            request.behavior_tree = self.mppi_behavior_tree
        return self._send(
            self.navigate_client,
            request,
            f'NAVIGATE_{controller}',
            timeout_sec,
        )

    def follow_path(
        self,
        path: Path,
        controller: str,
        goal_checker: str = 'general_goal_checker',
        timeout_sec: float = 75.0,
        validate_reverse: bool = False,
    ) -> bool:
        if len(path.poses) < 2:
            self.state = NavigationState.FAILED
            self.operation = f'FOLLOW_{controller}'
            self.message = 'Path has fewer than two poses'
            return False
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = controller
        request.goal_checker_id = goal_checker
        self.last_path = path
        self.path_publisher.publish(path)
        self._reverse_validation = validate_reverse
        self._reverse_positive_samples = 0
        self._reverse_negative_samples = 0
        self._reverse_velocity_stats = self._new_velocity_stats()
        return self._send(
            self.follow_client,
            request,
            f'FOLLOW_{controller}',
            timeout_sec,
        )

    def spin(self, angle_rad: float, timeout_sec: float = 15.0) -> bool:
        request = Spin.Goal()
        request.target_yaw = float(angle_rad)
        seconds = max(1, int(math.ceil(timeout_sec)))
        request.time_allowance.sec = seconds
        return self._send(self.spin_client, request, 'SPIN', timeout_sec)

    def cancel(self) -> None:
        self._generation += 1
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                # SIGINT may invalidate the ROS context before node teardown
                # reaches this best-effort action cancellation.
                pass
        if self.state in {NavigationState.SENDING, NavigationState.ACTIVE}:
            self.state = NavigationState.CANCELED
            self.message = f'{self.operation} cancelled'
        self._goal_handle = None
        self._reverse_validation = False
        self._capture_global_plan = False

    def clear_costmaps(self) -> bool:
        sent = False
        request = ClearEntireCostmap.Request()
        for client in (self.clear_local_client, self.clear_global_client):
            if client.service_is_ready():
                client.call_async(request)
                sent = True
        return sent

    def reverse_motion_valid(self) -> bool:
        return (
            self._reverse_positive_samples == 0
            and self._reverse_negative_samples > 0
        )

    def reverse_velocity_summary(self) -> str:
        return '; '.join(
            f'{name}[linear:{stats.linear.summary()},'
            f'angular:{stats.angular.summary()}]'
            for name, stats in self._reverse_velocity_stats.items()
        )

    def velocity_metrics(self) -> dict:
        return {
            name: {
                'linear_x': self._latest_velocity[name][0],
                'angular_z': self._latest_velocity[name][1],
                'samples': stats.linear.samples,
                'linear_x_min': (
                    stats.linear.minimum if stats.linear.samples else 0.0
                ),
                'linear_x_max': (
                    stats.linear.maximum if stats.linear.samples else 0.0
                ),
                'linear_x_mean': stats.linear.mean,
                'angular_z_min': (
                    stats.angular.minimum if stats.angular.samples else 0.0
                ),
                'angular_z_max': (
                    stats.angular.maximum if stats.angular.samples else 0.0
                ),
                'angular_z_mean': stats.angular.mean,
                'active_linear_samples': stats.linear.nonzero_samples,
                'active_angular_samples': stats.angular.nonzero_samples,
            }
            for name, stats in self._mission_velocity_stats.items()
        }

    def velocity_summary(self) -> str:
        return '; '.join(
            f'{name}[x={self._latest_velocity[name][0]:.3f},'
            f'z={self._latest_velocity[name][1]:.3f},'
            f'linear:{stats.linear.summary()},'
            f'angular:{stats.angular.summary()}]'
            for name, stats in self._operation_velocity_stats.items()
        )

    def robot_stationary(
        self,
        linear_tolerance_mps: float = 0.015,
        angular_tolerance_radps: float = 0.06,
    ) -> bool:
        """Return whether measured odometry is inside manipulation stop limits."""
        linear_x, angular_z = self._latest_velocity['odometry']
        return (
            abs(linear_x) <= max(0.0, float(linear_tolerance_mps))
            and abs(angular_z) <= max(
                0.0,
                float(angular_tolerance_radps),
            )
        )

    @staticmethod
    def _new_velocity_stats():
        return {
            'controller': MotionStats(),
            'smoother': MotionStats(),
            'collision_monitor': MotionStats(),
            'odometry': MotionStats(),
        }

    def _send(self, client, request, operation: str, timeout_sec: float) -> bool:
        if self.state in {NavigationState.SENDING, NavigationState.ACTIVE}:
            self.message = f'Cannot start {operation}; {self.operation} is active'
            return False
        if not client.server_is_ready():
            self.state = NavigationState.FAILED
            self.operation = operation
            self.message = f'Action server for {operation} is unavailable'
            return False
        self._generation += 1
        generation = self._generation
        self.state = NavigationState.SENDING
        self.operation = operation
        self.message = 'goal request sent'
        self.distance_remaining = None
        self.result_path = None
        self._operation_velocity_stats = self._new_velocity_stats()
        self._action_recoveries = 0
        self._deadline_wall = time.monotonic() + max(0.1, timeout_sec)
        self._goal_handle = None
        future = client.send_goal_async(request, feedback_callback=self._feedback)
        future.add_done_callback(
            lambda completed: self._goal_response(completed, generation)
        )
        return True

    def _goal_response(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy action transport exception
            self.state = NavigationState.FAILED
            self.message = f'Goal transport failed: {error}'
            return
        if goal_handle is None or not goal_handle.accepted:
            self.state = NavigationState.FAILED
            self.message = f'{self.operation} goal was rejected'
            return
        self._goal_handle = goal_handle
        self.state = NavigationState.ACTIVE
        self.message = 'goal accepted'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._result(completed, generation)
        )

    def _feedback(self, message) -> None:
        feedback = getattr(message, 'feedback', None)
        if feedback is None:
            return
        if hasattr(feedback, 'distance_remaining'):
            self.distance_remaining = float(feedback.distance_remaining)
        elif hasattr(feedback, 'distance_to_goal'):
            self.distance_remaining = float(feedback.distance_to_goal)
        if hasattr(feedback, 'number_of_recoveries'):
            action_recoveries = int(feedback.number_of_recoveries)
            if action_recoveries > self._action_recoveries:
                self.recoveries += action_recoveries - self._action_recoveries
            self._action_recoveries = action_recoveries

    def _global_plan_callback(self, message: Path) -> None:
        if len(message.poses) < 2:
            return
        self.last_path = message
        self.path_publisher.publish(message)
        if not self._capture_global_plan:
            return
        length = path_length(message)
        if length > self._captured_path_length:
            self.captured_path = message
            self._captured_path_length = length

    def _result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            wrapped = future.result()
        except Exception as error:  # rclpy action transport exception
            self.state = NavigationState.FAILED
            self.message = f'Action result failed: {error}'
            self._reverse_validation = False
            return
        status = wrapped.status
        reverse_summary = (
            self.reverse_velocity_summary() if self._reverse_validation else ''
        )
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = wrapped.result
            if hasattr(result, 'path'):
                self.result_path = result.path
                self.last_path = result.path
                self.path_publisher.publish(result.path)
            if self._reverse_validation and not self.reverse_motion_valid():
                self.state = NavigationState.FAILED
                self.message = (
                    'Reverse path completed without validated negative '
                    f'translational velocity; {reverse_summary}'
                )
            else:
                self.state = NavigationState.SUCCEEDED
                self.message = f'{self.operation} succeeded'
                if reverse_summary:
                    self.message += f'; {reverse_summary}'
        elif status == GoalStatus.STATUS_CANCELED:
            self.state = NavigationState.CANCELED
            self.message = f'{self.operation} was canceled'
        else:
            self.state = NavigationState.FAILED
            self.message = f'{self.operation} ended with action status {status}'
            if reverse_summary:
                self.message += f'; {reverse_summary}'
        self._reverse_validation = False
        self._goal_handle = None

    def _velocity_callback(self, source: str, message: Twist) -> None:
        self._last_cmd = message
        self._latest_velocity[source] = (
            float(message.linear.x),
            float(message.angular.z),
        )
        if self.state is NavigationState.ACTIVE:
            self._operation_velocity_stats[source].observe(
                float(message.linear.x), float(message.angular.z)
            )
            self._mission_velocity_stats[source].observe(
                float(message.linear.x), float(message.angular.z)
            )
        if not self._reverse_validation or self.state is not NavigationState.ACTIVE:
            return
        self._reverse_velocity_stats[source].observe(
            float(message.linear.x), float(message.angular.z)
        )
        if source != 'odometry':
            return
        if message.linear.x > 0.005:
            self._reverse_positive_samples += 1
        elif message.linear.x < -0.005:
            self._reverse_negative_samples += 1
