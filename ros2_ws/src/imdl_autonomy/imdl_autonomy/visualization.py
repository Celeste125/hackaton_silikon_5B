"""Stable RViz MarkerArray visualization for autonomy mission internals."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import covariance_ellipse

POINT_COLORS = {
    'A': (0.00, 0.75, 0.95),
    'B': (1.00, 0.38, 0.02),
    'C': (0.10, 0.78, 0.30),
}
OBJECT_COLORS = {
    'red': (0.95, 0.08, 0.05),
    'green': (0.05, 0.80, 0.18),
    'blue': (0.05, 0.30, 0.95),
    'yellow': (0.95, 0.82, 0.05),
}
ROBOT_FOOTPRINT = [
    (0.19, 0.06), (0.15, 0.13), (0.07, 0.17), (-0.12, 0.16),
    (-0.17, 0.08), (-0.17, -0.08), (-0.12, -0.16), (0.07, -0.17),
    (0.15, -0.13), (0.19, -0.06),
]
# The velocity-aware Collision Monitor approach model uses the live footprint.
COLLISION_STOP_ZONE = list(ROBOT_FOOTPRINT)
COLLISION_SLOW_ZONE = [
    (0.29, 0.09), (0.23, 0.20), (0.10, 0.27), (-0.19, 0.25),
    (-0.26, 0.12), (-0.26, -0.12), (-0.19, -0.25), (0.10, -0.27),
    (0.23, -0.20), (0.29, -0.09),
]


class MissionVisualization:

    def __init__(self, node, mission):
        self.node = node
        self.mission = mission
        self.publisher = node.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10,
        )
        self.state = 'WAITING'
        self.message = ''
        self.attempt = 1
        self.controller = 'none'
        self.active_goal = None
        self.path = None
        self.tracks = []
        self.approaches = []
        self.view_candidates = []
        self.active_view = None
        self.searched_points = []

    def set_solver_state(self, state: str, message: str, attempt: int) -> None:
        self.state = state
        self.message = message
        self.attempt = attempt

    def set_active_goal(self, goal, controller: str) -> None:
        self.active_goal = goal
        self.controller = controller

    def set_path(self, path) -> None:
        self.path = path

    def set_tracks(self, tracks) -> None:
        self.tracks = list(tracks)

    def set_approaches(self, approaches) -> None:
        self.approaches = list(approaches)

    def set_search(self, candidates, active, searched_points) -> None:
        self.view_candidates = list(candidates)
        self.active_view = active
        self.searched_points = list(searched_points)

    def publish(self, elapsed_sec: float = 0.0) -> None:
        stamp = self.node.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_id = 0

        route = self._marker(marker_id, 'mission_route', Marker.LINE_STRIP, stamp)
        marker_id += 1
        route.scale.x = 0.012
        route.color.r, route.color.g, route.color.b, route.color.a = (0.9, 0.9, 0.9, 0.6)
        for name in ('A', 'B', 'C'):
            if name in self.mission.poses:
                pose = self.mission.poses[name]
                route.points.append(Point(x=pose.x, y=pose.y, z=0.025))
        markers.markers.append(route)

        for name in ('A', 'B', 'C'):
            if name not in self.mission.poses:
                continue
            pose = self.mission.poses[name]
            color = POINT_COLORS[name]
            sphere = self._marker(marker_id, 'mission_points', Marker.SPHERE, stamp)
            marker_id += 1
            sphere.pose.position.x = pose.x
            sphere.pose.position.y = pose.y
            sphere.pose.position.z = 0.055
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.085
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (*color, 0.95)
            markers.markers.append(sphere)
            label = self._marker(marker_id, 'mission_point_labels', Marker.TEXT_VIEW_FACING, stamp)
            marker_id += 1
            label.pose.position.x = pose.x
            label.pose.position.y = pose.y
            label.pose.position.z = 0.18
            label.scale.z = 0.085
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = name
            markers.markers.append(label)

        if self.active_goal is not None:
            marker = self._marker(marker_id, 'mission_active_goal', Marker.ARROW, stamp)
            marker_id += 1
            marker.pose.position.x = self.active_goal.x
            marker.pose.position.y = self.active_goal.y
            marker.pose.position.z = 0.08
            marker.pose.orientation.z = math.sin(self.active_goal.yaw * 0.5)
            marker.pose.orientation.w = math.cos(self.active_goal.yaw * 0.5)
            marker.scale.x, marker.scale.y, marker.scale.z = (0.16, 0.025, 0.025)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 1.0)
            markers.markers.append(marker)

        if self.path is not None and self.path.poses:
            marker = self._marker(marker_id, 'mission_global_path', Marker.LINE_STRIP, stamp)
            marker_id += 1
            marker.scale.x = 0.018
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.05, 0.85, 1.0, 0.9)
            for pose in self.path.poses:
                marker.points.append(Point(
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    z=0.045,
                ))
            markers.markers.append(marker)

        for track in self.tracks:
            color = OBJECT_COLORS.get(track.color, (0.8, 0.8, 0.8))
            cube = self._marker(marker_id, 'mission_tracks', Marker.CUBE, stamp)
            marker_id += 1
            cube.pose.position.x = track.pose.x
            cube.pose.position.y = track.pose.y
            cube.pose.position.z = max(0.025, track.z)
            cube.scale.x = cube.scale.y = cube.scale.z = 0.035
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = (*color, 0.95)
            markers.markers.append(cube)
            major, minor, yaw = covariance_ellipse(track.covariance)
            ellipse = self._marker(marker_id, 'mission_track_covariance', Marker.CYLINDER, stamp)
            marker_id += 1
            ellipse.pose.position.x = track.pose.x
            ellipse.pose.position.y = track.pose.y
            ellipse.pose.position.z = 0.008
            ellipse.pose.orientation.z = math.sin(yaw * 0.5)
            ellipse.pose.orientation.w = math.cos(yaw * 0.5)
            ellipse.scale.x = max(0.01, major)
            ellipse.scale.y = max(0.01, minor)
            ellipse.scale.z = 0.006
            (
                ellipse.color.r,
                ellipse.color.g,
                ellipse.color.b,
                ellipse.color.a,
            ) = (*color, 0.25)
            markers.markers.append(ellipse)

        for candidate in self.approaches:
            pose = candidate.pose
            marker = self._marker(
                marker_id,
                'mission_approaches',
                Marker.SPHERE,
                stamp,
            )
            marker_id += 1
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = 0.035
            marker.scale.x = marker.scale.y = marker.scale.z = 0.035
            if candidate.accepted:
                color = (0.1, 0.9, 0.3, 0.75)
            else:
                color = (0.95, 0.1, 0.1, 0.55)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            markers.markers.append(marker)

        for index, candidate in enumerate(self.view_candidates[:12]):
            marker = self._marker(
                marker_id,
                'mission_nbv_candidates',
                Marker.SPHERE,
                stamp,
            )
            marker_id += 1
            marker.pose.position.x = candidate.pose.x
            marker.pose.position.y = candidate.pose.y
            marker.pose.position.z = 0.03
            marker.scale.x = marker.scale.y = marker.scale.z = max(
                0.025,
                0.06 - index * 0.002,
            )
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.65, 0.2, 1.0, 0.75
            )
            markers.markers.append(marker)

        if self.active_view is not None:
            half = 1.211 * 0.5
            frustum = self._marker(
                marker_id,
                'mission_camera_frustum',
                Marker.LINE_STRIP,
                stamp,
            )
            marker_id += 1
            frustum.scale.x = 0.01
            frustum.color.r, frustum.color.g, frustum.color.b, frustum.color.a = (
                0.8, 0.25, 1.0, 0.9
            )
            pose = self.active_view.pose
            frustum.points.extend([
                Point(x=pose.x, y=pose.y, z=0.06),
                Point(
                    x=pose.x + 0.9 * math.cos(pose.yaw - half),
                    y=pose.y + 0.9 * math.sin(pose.yaw - half),
                    z=0.06,
                ),
                Point(
                    x=pose.x + 0.9 * math.cos(pose.yaw + half),
                    y=pose.y + 0.9 * math.sin(pose.yaw + half),
                    z=0.06,
                ),
                Point(x=pose.x, y=pose.y, z=0.06),
            ])
            markers.markers.append(frustum)

        if self.searched_points:
            searched = self._marker(
                marker_id,
                'mission_searched_area',
                Marker.POINTS,
                stamp,
            )
            marker_id += 1
            searched.scale.x = searched.scale.y = 0.018
            (
                searched.color.r,
                searched.color.g,
                searched.color.b,
                searched.color.a,
            ) = (0.25, 0.55, 1.0, 0.25)
            searched.points.extend(
                Point(x=x, y=y, z=0.012)
                for x, y in self.searched_points
            )
            markers.markers.append(searched)

        self._append_robot_zones(markers, marker_id, stamp)
        marker_id += 3
        status = self._marker(marker_id, 'mission_status', Marker.TEXT_VIEW_FACING, stamp)
        status.pose.position.x = -0.68
        status.pose.position.y = -0.92
        status.pose.position.z = 0.24
        status.scale.z = 0.045
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0
        status.text = (
            f'{self.mission.mission_id} | {self.state} | {self.controller} | '
            f't={elapsed_sec:.1f}s | try={self.attempt}'
        )
        markers.markers.append(status)
        self.publisher.publish(markers)

    def _marker(self, marker_id, namespace, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.mission.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.nanosec = 600000000
        return marker

    def _append_robot_zones(self, markers, marker_id, stamp):
        shapes = (
            ('mission_footprint', ROBOT_FOOTPRINT, (0.1, 0.9, 0.9, 0.9)),
            ('mission_collision_stop', COLLISION_STOP_ZONE, (1.0, 0.05, 0.05, 0.8)),
            ('mission_collision_slow', COLLISION_SLOW_ZONE, (1.0, 0.75, 0.0, 0.55)),
        )
        for offset, (namespace, points, color) in enumerate(shapes):
            marker = self._marker(marker_id + offset, namespace, Marker.LINE_STRIP, stamp)
            marker.header.frame_id = 'base_link'
            marker.scale.x = 0.008
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.points.extend(Point(x=x, y=y, z=0.025) for x, y in points + [points[0]])
            markers.markers.append(marker)
