"""Independent health and Create3 hazard watchdog for autonomy missions."""

from __future__ import annotations

import json
import math
import time

from irobot_create_msgs.msg import (HazardDetection, HazardDetectionVector,
                                    KidnapStatus)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data,
                       QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .ros_shutdown import shutdown_node, spin_node_resilient

HARD_HAZARDS = {
    HazardDetection.BUMP: 'BUMP',
    HazardDetection.CLIFF: 'CLIFF',
    HazardDetection.STALL: 'STALL',
    HazardDetection.WHEEL_DROP: 'WHEEL_DROP',
}


def evaluate_safety(
    now: float,
    started: float,
    last_scan: float,
    last_odom: float,
    scan_timeout: float,
    odom_timeout: float,
    startup_grace: float,
    hazard_until: float,
    active_hazards,
    kidnapped: bool,
):
    """Pure watchdog decision used by the node and unit tests."""
    reasons = []
    grace_complete = now - started >= startup_grace
    if grace_complete and now - last_scan > scan_timeout:
        reasons.append('SCAN_STALE')
    if grace_complete and now - last_odom > odom_timeout:
        reasons.append('ODOM_STALE')
    if now <= hazard_until:
        reasons.extend(active_hazards)
    if kidnapped:
        reasons.append('KIDNAPPED')
    return reasons


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('scan_timeout_sec', 1.5)
        self.declare_parameter('odom_timeout_sec', 0.8)
        self.declare_parameter('startup_grace_sec', 5.0)
        self.declare_parameter('hazard_hold_sec', 2.0)

        self.started_wall = time.monotonic()
        self.last_scan_wall = -math.inf
        self.last_odom_wall = -math.inf
        self.hazard_until_wall = -math.inf
        self.active_hazards = []
        self.kidnapped = False
        self.last_stop = None

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stop_publisher = self.create_publisher(
            Bool,
            '/autonomy/safety_stop',
            state_qos,
        )
        self.state_publisher = self.create_publisher(
            String,
            '/autonomy/safety_state',
            state_qos,
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
            HazardDetectionVector,
            'hazard_detection',
            self._hazard_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            KidnapStatus,
            'kidnap_status',
            self._kidnap_cb,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info('safety_monitor armed')

    def _scan_cb(self, _message: LaserScan) -> None:
        self.last_scan_wall = time.monotonic()

    def _odom_cb(self, _message: Odometry) -> None:
        self.last_odom_wall = time.monotonic()

    def _hazard_cb(self, message: HazardDetectionVector) -> None:
        hazards = sorted({
            HARD_HAZARDS[detection.type]
            for detection in message.detections
            if detection.type in HARD_HAZARDS
        })
        if hazards:
            self.active_hazards = hazards
            self.hazard_until_wall = time.monotonic() + float(
                self.get_parameter('hazard_hold_sec').value
            )

    def _kidnap_cb(self, message: KidnapStatus) -> None:
        self.kidnapped = bool(message.is_kidnapped)

    def _tick(self) -> None:
        now = time.monotonic()
        scan_age = now - self.last_scan_wall
        odom_age = now - self.last_odom_wall
        reasons = evaluate_safety(
            now=now,
            started=self.started_wall,
            last_scan=self.last_scan_wall,
            last_odom=self.last_odom_wall,
            scan_timeout=float(self.get_parameter('scan_timeout_sec').value),
            odom_timeout=float(self.get_parameter('odom_timeout_sec').value),
            startup_grace=float(self.get_parameter('startup_grace_sec').value),
            hazard_until=self.hazard_until_wall,
            active_hazards=self.active_hazards,
            kidnapped=self.kidnapped,
        )
        if now > self.hazard_until_wall:
            self.active_hazards = []

        stop = bool(reasons)
        self.stop_publisher.publish(Bool(data=stop))
        state = {
            'safe': not stop,
            'reasons': reasons,
            'scan_age_sec': scan_age if math.isfinite(scan_age) else None,
            'odom_age_sec': odom_age if math.isfinite(odom_age) else None,
        }
        self.state_publisher.publish(String(data=json.dumps(state)))
        if stop != self.last_stop:
            detail = ', '.join(reasons) if reasons else 'all monitored inputs healthy'
            if stop:
                self.get_logger().error(f'safety stop=True: {detail}')
            else:
                self.get_logger().info(f'safety stop=False: {detail}')
            self.last_stop = stop


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        spin_node_resilient(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_node(node)
