#!/usr/bin/env python3
"""Block autonomy bringup until simulated sensors and TF are usable."""

from __future__ import annotations

import time

from gazebo_msgs.srv import GetEntityState, SetEntityState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, Range
from tf2_ros import Buffer, TransformListener


class SimulationReadiness(Node):
    """Exit successfully only after the Gazebo robot contract is live."""

    def __init__(self) -> None:
        super().__init__('simulation_readiness')
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('require_camera', True)
        self.declare_parameter('require_ultrasonic', False)
        self.declare_parameter('require_manipulation', False)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/Team2/odom')
        self.declare_parameter(
            'rgb_topic',
            '/camera/camera/color/image_raw',
        )
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/depth/image_rect_raw',
        )
        self.declare_parameter(
            'ultrasonic_topic',
            '/ultrasonic/front/range',
        )

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.require_camera = bool(
            self.get_parameter('require_camera').value
        )
        self.require_ultrasonic = bool(
            self.get_parameter('require_ultrasonic').value
        )
        self.require_manipulation = bool(
            self.get_parameter('require_manipulation').value
        )
        self.started = time.monotonic()
        self.last_log = 0.0
        self.ready = False
        self.timed_out = False
        self.transient_take_errors = 0
        self.received = {
            'scan': False,
            'odom': False,
            'rgb': False,
            'depth': False,
            'ultrasonic': False,
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_entity_client = self.create_client(
            GetEntityState,
            '/get_entity_state',
        )
        self.set_entity_client = self.create_client(
            SetEntityState,
            '/set_entity_state',
        )

        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            lambda _message: self._mark_received('scan'),
            qos_profile_sensor_data,
            raw=True,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            lambda _message: self._mark_received('odom'),
            qos_profile_sensor_data,
            raw=True,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('rgb_topic').value),
            lambda _message: self._mark_received('rgb'),
            qos_profile_sensor_data,
            raw=True,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            lambda _message: self._mark_received('depth'),
            qos_profile_sensor_data,
            raw=True,
        )
        self.create_subscription(
            Range,
            str(self.get_parameter('ultrasonic_topic').value),
            lambda _message: self._mark_received('ultrasonic'),
            qos_profile_sensor_data,
            raw=True,
        )
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'waiting for Gazebo sensor/TF readiness before starting Nav2'
        )

    def _mark_received(self, name: str) -> None:
        self.received[name] = True

    def _tick(self) -> None:
        tf_status = {
            'odom_base': self.tf_buffer.can_transform(
                'odom', 'base_link', Time()
            ),
            'base_lidar': self.tf_buffer.can_transform(
                'base_link', 'lidar_link', Time()
            ),
            'base_camera': self.tf_buffer.can_transform(
                'base_link', 'camera_color_optical_frame', Time()
            ),
            'base_ultrasonic': self.tf_buffer.can_transform(
                'base_link', 'ultrasonic_front_link', Time()
            ),
        }
        sensor_ready = (
            self.received['scan']
            and self.received['odom']
            and (
                not self.require_ultrasonic
                or self.received['ultrasonic']
            )
        )
        camera_ready = (
            not self.require_camera
            or (self.received['rgb'] and self.received['depth'])
        )
        transforms_ready = (
            tf_status['odom_base']
            and tf_status['base_lidar']
            and (not self.require_camera or tf_status['base_camera'])
            and (
                not self.require_ultrasonic
                or tf_status['base_ultrasonic']
            )
        )
        manipulation_status = {
            'get_entity_state': self.get_entity_client.service_is_ready(),
            'set_entity_state': self.set_entity_client.service_is_ready(),
        }
        manipulation_ready = (
            not self.require_manipulation
            or all(manipulation_status.values())
        )
        if (
            sensor_ready
            and camera_ready
            and transforms_ready
            and manipulation_ready
        ):
            self.ready = True
            elapsed = time.monotonic() - self.started
            self.get_logger().info(
                f'Gazebo contract ready after {elapsed:.2f}s: '
                f'sensors={self.received}, tf={tf_status}, '
                f'manipulation={manipulation_status}'
            )
            self.timer.cancel()
            return

        elapsed = time.monotonic() - self.started
        if elapsed >= self.timeout_sec:
            self.timed_out = True
            self.get_logger().error(
                f'Gazebo readiness timeout after {elapsed:.1f}s: '
                f'sensors={self.received}, tf={tf_status}, '
                f'manipulation={manipulation_status}'
            )
            self.timer.cancel()
            return
        if elapsed - self.last_log >= 5.0:
            self.last_log = elapsed
            self.get_logger().info(
                f'waiting for Gazebo contract: sensors={self.received}, '
                f'tf={tf_status}, manipulation={manipulation_status}'
            )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = SimulationReadiness()
    try:
        while rclpy.ok() and not node.ready and not node.timed_out:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except RuntimeError as error:
                # Humble can occasionally surface a transient take_message()
                # conversion failure while Gazebo publishers are being
                # created. Keep the readiness contract strict, but let the
                # following DDS sample satisfy it.
                if 'Unable to convert call argument' not in str(error):
                    raise
                node.transient_take_errors += 1
                if (
                    node.transient_take_errors == 1
                    or node.transient_take_errors % 10 == 0
                ):
                    node.get_logger().warning(
                        'discarding transient DDS conversion failure '
                        f'during startup (count={node.transient_take_errors})'
                    )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.ready else 2


if __name__ == '__main__':
    raise SystemExit(main())
