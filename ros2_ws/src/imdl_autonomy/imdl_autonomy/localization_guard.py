"""AMCL initialization and localization-health guard for autonomy missions."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data,
                       QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .mission_spec import load_mission, Pose2D
from .ros_shutdown import shutdown_node, spin_node_resilient


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def amcl_stale_motion(
    current_odom,
    odom_at_last_amcl,
) -> tuple[float, float]:
    """Return odometric distance/yaw accumulated since the last AMCL pose."""
    if current_odom is None or odom_at_last_amcl is None:
        return math.inf, math.inf
    dx = current_odom[0] - odom_at_last_amcl[0]
    dy = current_odom[1] - odom_at_last_amcl[1]
    dyaw = abs(_normalize_angle(current_odom[2] - odom_at_last_amcl[2]))
    return math.hypot(dx, dy), dyaw


def amcl_update_overdue(
    age_sec: float,
    timeout_sec: float,
    current_odom,
    odom_at_last_amcl,
    distance_limit_m: float,
    yaw_limit_rad: float,
) -> tuple[bool, float, float]:
    """AMCL poses are event-driven; stale is unsafe only after robot motion."""
    distance, yaw = amcl_stale_motion(current_odom, odom_at_last_amcl)
    overdue = (
        age_sec > timeout_sec
        and (distance > distance_limit_m or yaw > yaw_limit_rad)
    )
    return overdue, distance, yaw


def pose_from_transform(transform) -> Pose2D:
    translation = transform.transform.translation
    orientation = transform.transform.rotation
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
    return Pose2D(translation.x, translation.y, yaw)


class LocalizationGuardNode(Node):

    def __init__(self):
        super().__init__('localization_guard')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_timeout_sec', 1.5)
        self.declare_parameter('odom_timeout_sec', 0.8)
        self.declare_parameter('amcl_timeout_sec', 1.5)
        self.declare_parameter('amcl_motion_limit_m', 0.02)
        self.declare_parameter('amcl_motion_limit_rad', 0.05)
        self.declare_parameter('position_std_limit_m', 0.20)
        self.declare_parameter('yaw_std_limit_rad', 0.60)
        self.declare_parameter('active_scan_after_sec', 4.0)

        mission_file = str(self.get_parameter('mission_file').value)
        self.mission = load_mission(mission_file)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.started_wall = time.monotonic()
        self.last_scan_wall = -math.inf
        self.last_odom_wall = -math.inf
        self.last_amcl_wall = -math.inf
        self.latest_amcl = None
        self.latest_odom_pose = None
        self.odom_pose_at_last_amcl = None
        self.initial_pose_publications = 0
        self.last_initial_pose_wall = -math.inf

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.create_publisher(
            String,
            '/autonomy/localization_state',
            state_qos,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._odom_cb, odom_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            10,
        )
        self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f'localization_guard: mission={self.mission.mission_id}, '
            f'A=({self.mission.start.x:.3f}, {self.mission.start.y:.3f}, '
            f'{self.mission.start.yaw:.3f})'
        )

    def _scan_cb(self, _message: LaserScan) -> None:
        self.last_scan_wall = time.monotonic()

    def _odom_cb(self, message: Odometry) -> None:
        self.last_odom_wall = time.monotonic()
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
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
        self.latest_odom_pose = (position.x, position.y, yaw)
        if (
            self.latest_amcl is not None
            and self.odom_pose_at_last_amcl is None
        ):
            self.odom_pose_at_last_amcl = self.latest_odom_pose

    def _amcl_cb(self, message: PoseWithCovarianceStamped) -> None:
        self.latest_amcl = message
        self.last_amcl_wall = time.monotonic()
        self.odom_pose_at_last_amcl = self.latest_odom_pose

    def _publish_initial_pose(self) -> None:
        start = self.mission.start
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = start.x
        message.pose.pose.position.y = start.y
        message.pose.pose.orientation.z = math.sin(start.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(start.yaw * 0.5)
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.04
        self.initial_pose_publisher.publish(message)
        self.initial_pose_publications += 1
        self.last_initial_pose_wall = time.monotonic()

    def _tick(self) -> None:
        now_wall = time.monotonic()
        if (
            self.latest_amcl is None
            and now_wall - self.last_initial_pose_wall >= 1.0
        ):
            self._publish_initial_pose()

        scan_age = now_wall - self.last_scan_wall
        odom_age = now_wall - self.last_odom_wall
        amcl_age = now_wall - self.last_amcl_wall
        amcl_motion_distance = 0.0
        amcl_motion_yaw = 0.0
        reasons = []
        if scan_age > float(self.get_parameter('scan_timeout_sec').value):
            reasons.append('SCAN_STALE')
        if odom_age > float(self.get_parameter('odom_timeout_sec').value):
            reasons.append('ODOM_STALE')
        if self.latest_amcl is None:
            reasons.append('AMCL_POSE_MISSING')
            position_std = math.inf
            yaw_std = math.inf
            pose = self.mission.start
        else:
            covariance = self.latest_amcl.pose.covariance
            position_std = math.sqrt(max(0.0, covariance[0] + covariance[7]))
            yaw_std = math.sqrt(max(0.0, covariance[35]))
            position = self.latest_amcl.pose.pose.position
            orientation = self.latest_amcl.pose.pose.orientation
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
            pose = Pose2D(position.x, position.y, yaw)
            amcl_overdue, amcl_motion_distance, amcl_motion_yaw = (
                amcl_update_overdue(
                    amcl_age,
                    float(self.get_parameter('amcl_timeout_sec').value),
                    self.latest_odom_pose,
                    self.odom_pose_at_last_amcl,
                    float(
                        self.get_parameter('amcl_motion_limit_m').value
                    ),
                    float(
                        self.get_parameter('amcl_motion_limit_rad').value
                    ),
                )
            )
            if amcl_overdue:
                reasons.append('AMCL_POSE_STALE')
            if position_std > float(
                self.get_parameter('position_std_limit_m').value
            ):
                reasons.append('POSITION_UNCERTAIN')
            if yaw_std > float(
                self.get_parameter('yaw_std_limit_rad').value
            ):
                reasons.append('YAW_UNCERTAIN')

        tf_ok = False
        tf_pose = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            tf_ok = True
            tf_pose = pose_from_transform(transform)
        except TransformException:
            reasons.append('MAP_BASE_TF_MISSING')
        if tf_pose is not None:
            # AMCL messages are event-driven. The composed map->base transform
            # includes fresh odometry and is therefore the current robot pose.
            pose = tf_pose

        scan_required = (
            now_wall - self.started_wall
            >= float(self.get_parameter('active_scan_after_sec').value)
            and any(reason in reasons for reason in (
                'AMCL_POSE_MISSING',
                'POSITION_UNCERTAIN',
                'YAW_UNCERTAIN',
            ))
            and scan_age <= float(self.get_parameter('scan_timeout_sec').value)
        )
        state = {
            'mission_id': self.mission.mission_id,
            'ready': not reasons,
            'scan_required': scan_required,
            'tf_ok': tf_ok,
            'reasons': reasons,
            'position_std_m': position_std,
            'yaw_std_rad': yaw_std,
            'scan_age_sec': scan_age if math.isfinite(scan_age) else None,
            'odom_age_sec': odom_age if math.isfinite(odom_age) else None,
            'amcl_age_sec': amcl_age if math.isfinite(amcl_age) else None,
            'amcl_motion_distance_m': (
                amcl_motion_distance
                if math.isfinite(amcl_motion_distance)
                else None
            ),
            'amcl_motion_yaw_rad': (
                amcl_motion_yaw if math.isfinite(amcl_motion_yaw) else None
            ),
            'initial_pose_publications': self.initial_pose_publications,
            'pose_source': 'tf' if tf_pose is not None else 'amcl',
            'pose': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw},
        }
        self.state_publisher.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGuardNode()
    try:
        spin_node_resilient(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_node(node)
