#!/usr/bin/env python3
"""Publish RViz markers for the simulated arena objects."""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ArenaMarkerPublisher(Node):

    def __init__(self):
        super().__init__('arena_marker_publisher')
        self.declare_parameter('world_path', '')
        self.declare_parameter('frame_id', 'odom')
        self.world_path = self.get_parameter('world_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.objects = self._load_world_markers(self.world_path)
        self.pub = self.create_publisher(MarkerArray, '/arena_markers', 1)
        self.timer = self.create_timer(0.5, self._publish)
        self.get_logger().info(
            f'publishing {len(self.objects)} arena markers from {self.world_path}')

    def _parse_pose(self, text):
        if not text:
            return (0.0, 0.0, 0.0)
        values = [float(value) for value in text.split()]
        return tuple((values + [0.0, 0.0, 0.0])[:3])

    def _parse_color(self, visual):
        ambient = visual.findtext('./material/ambient')
        if ambient:
            values = [float(value) for value in ambient.split()]
            return tuple((values + [1.0])[:4])
        return (0.6, 0.6, 0.6, 0.75)

    def _load_world_markers(self, world_path):
        if not world_path:
            return []
        try:
            root = ET.parse(world_path).getroot()
        except (ET.ParseError, OSError) as exc:
            self.get_logger().warn(f'Cannot load arena markers from {world_path}: {exc}')
            return []

        objects = []
        for model in root.findall('.//world/model'):
            model_name = model.get('name', 'model')
            if model_name == 'ground_plane':
                continue
            model_pose = self._parse_pose(model.findtext('pose'))
            for visual in model.findall('.//visual'):
                geometry = visual.find('geometry')
                if geometry is None:
                    continue
                local_pose = self._parse_pose(visual.findtext('pose'))
                position = tuple(model_pose[i] + local_pose[i] for i in range(3))
                name = f'{model_name}/{visual.get("name", "visual")}'
                color = self._parse_color(visual)

                box = geometry.find('box')
                if box is not None:
                    size_text = box.findtext('size')
                    if not size_text:
                        continue
                    scale = tuple(float(value) for value in size_text.split()[:3])
                    objects.append((name, Marker.CUBE, position, scale, color))
                    continue

                cylinder = geometry.find('cylinder')
                if cylinder is not None:
                    radius = float(cylinder.findtext('radius', default='0.0'))
                    length = float(cylinder.findtext('length', default='0.0'))
                    scale = (2.0 * radius, 2.0 * radius, length)
                    objects.append((name, Marker.CYLINDER, position, scale, color))

        return objects

    def _base_marker(self, marker_id, name, marker_type, position, scale, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'imdl_arena'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.text = name
        return marker

    def _publish(self):
        markers = MarkerArray()
        for marker_id, (name, marker_type, position, scale, color) in enumerate(self.objects):
            marker = self._base_marker(marker_id, name, marker_type, position, scale, color)
            markers.markers.append(marker)

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaMarkerPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can race a subscription conversion with SIGINT teardown.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
