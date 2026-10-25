"""Make sure the camera has looked at the arena, not just the lidar.

The frontier explorer finishes when the *lidar* has mapped everything. The
lidar sees 360 degrees; the camera sees about 70 out of the front. So a
completed frontier run has typically had the camera over a third of the floor,
and the dispenser may never once have been in frame -- the map looks finished
and the landmark was never findable.

This node closes that gap. It does two separable things:

  Always, from the moment it starts, it accumulates which free floor cells
  have been inside the camera frustum, occlusion included, and publishes that
  as a grid for RViz. Running it during the frontier sweep costs nothing and
  means most of the coverage is already banked by the time the sweep ends.

  Then, once the frontier explorer reports itself done, it drives: it picks
  the pose that uncovers the most unlooked-at floor per metre travelled and
  sends it to Nav2, until coverage passes its target or nothing is left worth
  looking at.

It drives only through navigate_to_pose, like the frontier explorer, so every
motion still passes the collision monitor and teleop still outranks it.

The two never drive at once. This node waits for /frontier_explorer/done,
because two nodes sending goals to one Nav2 action server would preempt each
other turn by turn and the robot would twitch between two plans. Pass
start_mode:=immediate only when the frontier explorer is not running.

    ros2 launch imdl_autonomy camera_coverage.launch.py
    ros2 topic echo /camera_coverage/status
"""

from __future__ import annotations

import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
import tf2_ros
from tf2_ros import TransformException

from .camera_coverage import (
    CameraModel,
    GridGeometry,
    classify,
    coverage_fraction,
    plan_view,
    remap_seen,
    unseen_clusters,
    visible_mask,
)

# What the published coverage grid means, chosen so RViz's Map display tells
# the story at a glance: unlooked-at floor lights up, looked-at floor is
# clear, and everything else stays unknown.
_UNSEEN = 100
_SEEN = 0
_IRRELEVANT = -1


class CameraCoverageExplorer(Node):

    def __init__(self):
        super().__init__('camera_coverage_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('coverage_topic', '/camera_coverage')
        self.declare_parameter('status_topic', '/camera_coverage/status')

        # The camera's footprint on the floor. max_range_m is the range at
        # which the model can still recognise something, not the range at
        # which the lens forms an image -- see camera_coverage.CameraModel.
        self.declare_parameter('camera_fov_deg', 70.4)
        self.declare_parameter('camera_min_range_m', 0.30)
        self.declare_parameter('camera_max_range_m', 3.00)
        self.declare_parameter('camera_bearing_offset_deg', 0.0)

        self.declare_parameter('coverage_target', 0.92)
        self.declare_parameter('start_mode', 'wait_for_frontier')
        self.declare_parameter('update_interval_sec', 0.5)
        self.declare_parameter('plan_interval_sec', 2.0)
        self.declare_parameter('standoff_m', 1.2)
        self.declare_parameter('clearance_m', 0.25)
        self.declare_parameter('min_new_cells', 8)
        self.declare_parameter('nav_timeout_sec', 40.0)
        self.declare_parameter('blacklist_radius_m', 0.35)
        self.declare_parameter('done_ticks_threshold', 3)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._camera = CameraModel(
            fov_rad=math.radians(
                float(self.get_parameter('camera_fov_deg').value)),
            min_range_m=float(
                self.get_parameter('camera_min_range_m').value),
            max_range_m=float(
                self.get_parameter('camera_max_range_m').value),
            bearing_offset_rad=math.radians(
                float(self.get_parameter('camera_bearing_offset_deg').value)),
        )

        mode = str(self.get_parameter('start_mode').value)
        if mode not in ('wait_for_frontier', 'immediate', 'observe_only'):
            raise ValueError(f'unknown start_mode {mode!r}')
        self._driving_allowed = mode == 'immediate'
        self._observe_only = mode == 'observe_only'

        self._map: OccupancyGrid | None = None
        self._geometry: GridGeometry | None = None
        self._seen: np.ndarray | None = None
        self._navigating = False
        self._goal_handle = None
        self._goal_sent_at: float | None = None
        self._pending: tuple[float, float] = (0.0, 0.0)
        self._blacklist: list[tuple[float, float]] = []
        self._barren_ticks = 0
        self._finished = False
        self._views = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._on_map, 1)
        # Latched, so starting after the frontier run has already finished
        # still picks the signal up rather than waiting for ever.
        self.create_subscription(
            Empty, '/frontier_explorer/done', self._on_frontier_done,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._coverage_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('coverage_topic').value),
            latched)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 1)
        self._done_pub = self.create_publisher(
            Empty, '/camera_coverage/done', latched)

        self.create_timer(
            float(self.get_parameter('update_interval_sec').value),
            self._accumulate)
        self.create_timer(
            float(self.get_parameter('plan_interval_sec').value), self._tick)

        self.get_logger().info(
            f'camera_coverage_explorer ready: fov '
            f'{math.degrees(self._camera.fov_rad):.0f} deg out to '
            f'{self._camera.max_range_m:.1f} m, target '
            f'{float(self.get_parameter("coverage_target").value) * 100:.0f}%'
            f', mode={mode}')

    # -------------------------------------------------------------- inputs

    def _on_map(self, msg: OccupancyGrid):
        geometry = GridGeometry(
            resolution_m=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            width=msg.info.width,
            height=msg.info.height,
        )
        if self._seen is None:
            self._seen = np.zeros((geometry.height, geometry.width), bool)
        elif self._geometry != geometry:
            # slam_toolbox grows the map as it explores. Without carrying the
            # coverage across, every growth would reset it and the robot
            # would re-sweep the same floor for ever.
            self._seen = remap_seen(self._seen, self._geometry, geometry)
        self._map = msg
        self._geometry = geometry

    def _on_frontier_done(self, _msg: Empty):
        if self._observe_only or self._driving_allowed:
            return
        self._driving_allowed = True
        self.get_logger().info(
            'frontier explorer finished; taking over to cover what the '
            'camera has not looked at')

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(
                f'no {self._map_frame} -> {self._base_frame}: {exc}',
                throttle_duration_sec=10.0)
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    # ---------------------------------------------------------- accumulate

    def _grids(self):
        if self._map is None or self._geometry is None or self._seen is None:
            return None
        cells = np.asarray(self._map.data, dtype=np.int16).reshape(
            self._geometry.height, self._geometry.width)
        occupied, free = classify(cells)
        return occupied, free

    def _accumulate(self):
        """Mark what the camera can see from wherever the robot is now.

        Runs whoever is driving -- the frontier explorer, teleop, or this
        node. Coverage is a property of where the camera has pointed, not of
        who pointed it.
        """
        grids = self._grids()
        pose = self._robot_pose()
        if grids is None or pose is None:
            return
        occupied, free = grids
        self._seen |= visible_mask(
            occupied, self._geometry, pose, self._camera)
        self._views += 1
        self._publish_coverage(free)

    def _publish_coverage(self, free: np.ndarray):
        grid = OccupancyGrid()
        grid.header.frame_id = self._map_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = self._map.info
        values = np.full(free.shape, _IRRELEVANT, dtype=np.int8)
        values[free & self._seen] = _SEEN
        values[free & ~self._seen] = _UNSEEN
        grid.data = values.reshape(-1).tolist()
        self._coverage_pub.publish(grid)

    # ---------------------------------------------------------------- drive

    def _tick(self):
        grids = self._grids()
        if grids is None:
            return
        occupied, free = grids
        fraction = coverage_fraction(self._seen, free)
        remaining = unseen_clusters(self._seen, free, self._geometry)

        self._status_pub.publish(String(data=json.dumps({
            'coverage': round(fraction, 4),
            'target': float(self.get_parameter('coverage_target').value),
            'free_cells': int(free.sum()),
            'seen_cells': int((free & self._seen).sum()),
            'unseen_clusters': len(remaining),
            'largest_unseen_cells': remaining[0][2] if remaining else 0,
            'views': self._views,
            'driving': self._driving_allowed and not self._finished,
            'finished': self._finished,
        })))

        if self._finished or not self._driving_allowed:
            return
        if fraction >= float(self.get_parameter('coverage_target').value):
            self._finish(f'coverage {fraction * 100:.0f}% reached the target')
            return
        if self._navigating:
            self._check_timeout()
            return

        pose = self._robot_pose()
        if pose is None:
            return
        goal = plan_view(
            occupied, free, self._seen, self._geometry, pose, self._camera,
            standoff_m=float(self.get_parameter('standoff_m').value),
            clearance_m=float(self.get_parameter('clearance_m').value),
            min_new_cells=int(self.get_parameter('min_new_cells').value),
            blacklist=self._blacklist,
            blacklist_radius_m=float(
                self.get_parameter('blacklist_radius_m').value),
        )
        if goal is None:
            # Not necessarily finished: a cluster may be unreachable this tick
            # and reachable once the map fills in a little more.
            self._barren_ticks += 1
            if self._barren_ticks >= int(
                self.get_parameter('done_ticks_threshold').value
            ):
                self._finish(
                    f'nothing reachable left to look at; stopped at '
                    f'{fraction * 100:.0f}%')
            return
        self._barren_ticks = 0
        self._send(goal, fraction)

    def _send(self, goal, fraction: float):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'Nav2 navigate_to_pose is not up yet',
                throttle_duration_sec=10.0)
            return
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
        pose.pose.orientation.w = math.cos(goal.yaw / 2.0)

        message = NavigateToPose.Goal()
        message.pose = pose
        self._navigating = True
        self._goal_sent_at = self.get_clock().now().nanoseconds * 1e-9
        self._pending = (goal.x, goal.y)
        self.get_logger().info(
            f'coverage {fraction * 100:.0f}%: '
            + ('turning in place' if goal.travel_m < 0.05 else
               f'driving {goal.travel_m:.2f} m to '
               f'({goal.x:.2f}, {goal.y:.2f})')
            + f' to see {goal.new_cells} more cells')
        self._nav_client.send_goal_async(message).add_done_callback(
            self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the coverage goal')
            self._blacklist.append(self._pending)
            self._reset()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            # Somewhere Nav2 will not take the robot. Remember it, so the
            # planner stops proposing the same unreachable spot every tick.
            self._blacklist.append(self._pending)
            self.get_logger().info(
                f'coverage goal ended with status {status}; blacklisted '
                f'({self._pending[0]:.2f}, {self._pending[1]:.2f})')
        self._reset()

    def _check_timeout(self):
        if self._goal_sent_at is None:
            return
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._goal_sent_at
        if elapsed <= float(self.get_parameter('nav_timeout_sec').value):
            return
        self.get_logger().warn(
            f'coverage goal took more than {elapsed:.0f} s; cancelling')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._blacklist.append(self._pending)
        self._reset()

    def _reset(self):
        self._navigating = False
        self._goal_handle = None
        self._goal_sent_at = None

    def _finish(self, reason: str):
        self._finished = True
        self.get_logger().info(f'camera coverage done: {reason}')
        self._done_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = CameraCoverageExplorer()
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
