"""Stand-in for aruco_detector when testing on a map in RViz, without a camera.

Publishes ``aruco/marker_<id>`` (PoseStamped in ``map``, yaw = outward normal)
for every entry of ``marker_route.sim_markers`` in the waypoints file, and an
``aruco/markers_viz`` MarkerArray (arrows plus ids) to show in RViz.
With ``visible_range_m`` > 0 a marker is only published while the robot is
within that distance, like a real camera that has to be close enough.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def quat_z(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FakeMarkers(Node):
    def __init__(self):
        super().__init__('fake_markers')
        default_file = os.path.join(
            get_package_share_directory('imdl_statemachine'), 'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter('visible_range_m', 3.0)  # 0 = always visible
        self.declare_parameter('base_frame', 'base_link')

        path = os.path.expanduser(self.get_parameter('waypoints_file').value)
        with open(path, 'r', encoding='utf-8') as stream:
            cfg = yaml.safe_load(stream)['marker_route']
        self.markers = {int(k): [float(v) for v in p] for k, p in cfg['sim_markers'].items()}
        self.range = float(self.get_parameter('visible_range_m').value)
        self.base = self.get_parameter('base_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pubs = {i: self.create_publisher(PoseStamped, f'aruco/marker_{i}', 10)
                     for i in self.markers}
        self.viz = self.create_publisher(MarkerArray, 'aruco/markers_viz', 1)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f'Fake markers {sorted(self.markers)} from {path}')

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', self.base, Time()).transform.translation
        except tf2_ros.TransformException:
            return None
        return t.x, t.y

    def _tick(self):
        now = self.get_clock().now().to_msg()
        robot = self._robot_xy()
        viz = MarkerArray()
        for marker_id, (x, y, yaw) in self.markers.items():
            qz, qw = quat_z(yaw)
            arrow = Marker()
            arrow.header.frame_id, arrow.header.stamp = 'map', now
            arrow.ns, arrow.id, arrow.type = 'aruco', marker_id, Marker.ARROW
            arrow.pose.position.x, arrow.pose.position.y = x, y
            arrow.pose.orientation.z, arrow.pose.orientation.w = qz, qw
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.4, 0.06, 0.06
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.5, 0.0, 1.0
            text = Marker()
            text.header, text.ns, text.id = arrow.header, 'aruco_id', marker_id
            text.type, text.pose.position.x, text.pose.position.y = Marker.TEXT_VIEW_FACING, x, y
            text.pose.position.z, text.scale.z = 0.3, 0.2
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'ID {marker_id}'
            viz.markers += [arrow, text]

            if self.range > 0.0:
                if robot is None or math.hypot(x - robot[0], y - robot[1]) > self.range:
                    continue
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'map', now
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            self.pubs[marker_id].publish(pose)
        self.viz.publish(viz)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
