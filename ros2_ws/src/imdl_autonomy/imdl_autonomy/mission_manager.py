"""Top-level, non-blocking mission state machine for the autonomy solver stack."""

from __future__ import annotations

import json
from pathlib import Path
import time
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from .create3_adapter import Create3SafetyAdapter
from .mission_geometry import validate_mission_geometry
from .mission_spec import load_mission
from .navigation_adapter import NavigationAdapter
from .next_best_view import NextBestViewPlanner, OccupancyGrid
from .ros_shutdown import shutdown_node, spin_node_resilient
from .runtime_context import (
    CameraObservationStore,
    FrontRangeStore,
    LocalizationStore,
    ManipulatorClient,
    ObjectStore,
    SafetyStore,
)
from .solvers.base import SolverStatus
from .solvers.registry import create_solver
from .visualization import MissionVisualization


MANIPULATION_TASKS = frozenset({'task3', 'task4', 'task5'})


def task_requires_manipulator(task: str) -> bool:
    """Return whether a task needs the pick/place adapter before it can start."""
    return str(task).strip().lower() in MANIPULATION_TASKS


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('rpp_behavior_tree', '')
        self.declare_parameter('mppi_behavior_tree', '')
        self.declare_parameter('mppi_mission_behavior_tree', '')
        self.declare_parameter('mppi_precise_behavior_tree', '')
        self.declare_parameter('startup_timeout_sec', 90.0)
        self.declare_parameter('manipulator_startup_timeout_sec', 12.0)
        self.declare_parameter('localization_loss_grace_sec', 1.5)
        self.declare_parameter('navigation_timeout_sec', 75.0)
        self.declare_parameter('navigation_retries', 2)
        self.declare_parameter('search_timeout_sec', 85.0)
        self.declare_parameter('max_search_viewpoints', 12)
        self.declare_parameter('task5_search_timeout_sec', 180.0)
        self.declare_parameter('task5_max_search_viewpoints', 24)
        self.declare_parameter('search_candidate_pool', 60)
        self.declare_parameter('search_rejected_radius_m', 0.16)
        self.declare_parameter('search_plan_timeout_sec', 5.0)
        self.declare_parameter('search_view_position_tolerance_m', 0.08)
        self.declare_parameter('search_view_clearance_margin_m', 0.10)
        self.declare_parameter('search_stall_timeout_sec', 8.0)
        self.declare_parameter('search_translation_stall_timeout_sec', 7.0)
        self.declare_parameter('search_progress_distance_m', 0.03)
        self.declare_parameter('search_progress_yaw_rad', 0.12)
        self.declare_parameter('search_sweep_viewpoints', 3)
        self.declare_parameter('search_sweep_segments', 6)
        self.declare_parameter('search_sweep_step_rad', 1.0)
        self.declare_parameter('search_sweep_segment_timeout_sec', 8.0)
        self.declare_parameter('transport_plan_timeout_sec', 3.0)
        self.declare_parameter('camera_approach_candidate_limit', 8)
        self.declare_parameter('manipulation_approach_candidate_limit', 12)
        self.declare_parameter('camera_dwell_sec', 0.8)
        self.declare_parameter('camera_confirmation_timeout_sec', 4.0)
        self.declare_parameter('camera_center_timeout_sec', 15.0)
        self.declare_parameter('camera_observation_max_age_sec', 0.75)
        self.declare_parameter('camera_center_tolerance_rad', 0.05)
        self.declare_parameter('camera_center_samples', 3)
        self.declare_parameter('camera_center_max_corrections', 4)
        self.declare_parameter('camera_center_max_step_rad', 0.25)
        self.declare_parameter('camera_center_gain', 0.55)
        self.declare_parameter('camera_center_settle_sec', 0.55)
        self.declare_parameter(
            'typed_perception_topic',
            '/perception/detected_objects',
        )
        self.declare_parameter('perception_minimum_confidence', 0.35)
        self.declare_parameter('realsense_range_match_tolerance_m', 0.06)
        self.declare_parameter(
            'ultrasonic_topic',
            '/ultrasonic/front/range',
        )
        self.declare_parameter('ultrasonic_required_for_pick', False)
        self.declare_parameter('ultrasonic_max_age_sec', 0.35)
        self.declare_parameter('ultrasonic_samples', 3)
        self.declare_parameter('ultrasonic_sensor_offset_x_m', 0.190)
        self.declare_parameter('ultrasonic_object_half_extent_m', 0.0175)
        self.declare_parameter('ultrasonic_match_tolerance_m', 0.035)
        self.declare_parameter('ultrasonic_max_target_bearing_rad', 0.10)
        self.declare_parameter('camera_view_distance_m', 0.36)
        self.declare_parameter('approach_distance_m', 0.27)
        self.declare_parameter('drop_approach_distance_m', 0.31)
        self.declare_parameter('pick_terminal_min_distance_m', 0.24)
        self.declare_parameter('pick_terminal_max_distance_m', 0.335)
        self.declare_parameter('pick_terminal_max_bearing_rad', 0.55)
        self.declare_parameter('pick_terminal_camera_error_rad', 0.58)
        self.declare_parameter('pick_terminal_track_age_sec', 1.0)
        self.declare_parameter('drop_terminal_min_distance_m', 0.285)
        self.declare_parameter('drop_terminal_max_distance_m', 0.345)
        self.declare_parameter('drop_terminal_max_bearing_rad', 0.22)
        self.declare_parameter('drop_alignment_timeout_sec', 12.0)
        self.declare_parameter('terminal_stop_settle_sec', 0.30)
        self.declare_parameter('placed_object_sync_sec', 0.35)
        self.declare_parameter('placed_costmap_settle_sec', 0.45)
        self.declare_parameter('camera_retry_separation_m', 0.20)
        self.declare_parameter('manipulation_retry_separation_m', 0.14)
        self.declare_parameter('robot_clearance_m', 0.17)
        self.declare_parameter('final_yaw_tolerance_rad', 0.10)
        self.declare_parameter('diagnostic_log_period_sec', 2.0)
        self.declare_parameter(
            'create3_motion_control_node',
            '/Team2/motion_control',
        )

        self.mission = load_mission(
            str(self.get_parameter('mission_file').value)
        )
        share = Path(get_package_share_directory('imdl_autonomy'))
        rpp_tree = str(self.get_parameter('rpp_behavior_tree').value) or str(
            share / 'behavior_trees' / 'navigate_rpp.xml'
        )
        mppi_tree = str(self.get_parameter('mppi_behavior_tree').value) or str(
            share / 'behavior_trees' / 'navigate_mppi.xml'
        )
        mppi_mission_tree = str(
            self.get_parameter('mppi_mission_behavior_tree').value
        ) or str(
            share / 'behavior_trees' / 'navigate_mppi_mission.xml'
        )
        mppi_precise_tree = str(
            self.get_parameter('mppi_precise_behavior_tree').value
        ) or str(
            share / 'behavior_trees' / 'navigate_mppi_precise.xml'
        )
        for tree in (
            rpp_tree,
            mppi_tree,
            mppi_mission_tree,
            mppi_precise_tree,
        ):
            if not Path(tree).is_file():
                raise RuntimeError(f'Behavior tree does not exist: {tree}')

        self.grid = OccupancyGrid.from_yaml(self.mission.map_yaml)
        self.localization = LocalizationStore(self)
        self.safety = SafetyStore(self)
        self.objects = ObjectStore(self)
        self.camera = CameraObservationStore(
            self,
            typed_topic=str(
                self.get_parameter('typed_perception_topic').value
            ),
            minimum_confidence=float(
                self.get_parameter('perception_minimum_confidence').value
            ),
        )
        self.front_range = FrontRangeStore(
            self,
            str(self.get_parameter('ultrasonic_topic').value),
        )
        self.navigation = NavigationAdapter(
            self,
            self.mission.frame_id,
            rpp_tree,
            mppi_tree,
            mppi_mission_tree,
            mppi_precise_tree,
        )
        self.manipulator = ManipulatorClient(self, self.mission.frame_id)
        self.create3_safety = Create3SafetyAdapter(
            self,
            str(self.get_parameter('create3_motion_control_node').value),
        )
        self.visualizer = MissionVisualization(self, self.mission)
        self.view_planner = NextBestViewPlanner(
            self.grid,
            robot_clearance_m=float(
                self.get_parameter('robot_clearance_m').value
            ),
        )
        self.geometry_issues = validate_mission_geometry(
            self.mission,
            self.grid,
            robot_clearance_m=float(
                self.get_parameter('robot_clearance_m').value
            ),
            approach_distance_m=float(
                self.get_parameter('approach_distance_m').value
            ),
        )

        self.mission_context = SimpleNamespace(
            node=self,
            mission=self.mission,
            navigation=self.navigation,
            localization=self.localization,
            safety=self.safety,
            objects=self.objects,
            camera=self.camera,
            front_range=self.front_range,
            manipulator=self.manipulator,
            create3_safety=self.create3_safety,
            visualizer=self.visualizer,
            view_planner=self.view_planner,
            grid=self.grid,
            logger=self.get_logger(),
            monotonic=time.monotonic,
            current_pose=lambda: self.localization.pose,
            navigation_timeout_sec=float(
                self.get_parameter('navigation_timeout_sec').value
            ),
            navigation_retries=int(
                self.get_parameter('navigation_retries').value
            ),
            search_timeout_sec=float(
                self.get_parameter('search_timeout_sec').value
            ),
            max_search_viewpoints=int(
                self.get_parameter('max_search_viewpoints').value
            ),
            task5_search_timeout_sec=float(
                self.get_parameter('task5_search_timeout_sec').value
            ),
            task5_max_search_viewpoints=int(
                self.get_parameter('task5_max_search_viewpoints').value
            ),
            search_candidate_pool=int(
                self.get_parameter('search_candidate_pool').value
            ),
            search_rejected_radius_m=float(
                self.get_parameter('search_rejected_radius_m').value
            ),
            search_plan_timeout_sec=float(
                self.get_parameter('search_plan_timeout_sec').value
            ),
            search_view_position_tolerance_m=float(
                self.get_parameter(
                    'search_view_position_tolerance_m'
                ).value
            ),
            search_view_clearance_margin_m=float(
                self.get_parameter(
                    'search_view_clearance_margin_m'
                ).value
            ),
            search_stall_timeout_sec=float(
                self.get_parameter('search_stall_timeout_sec').value
            ),
            search_translation_stall_timeout_sec=float(
                self.get_parameter(
                    'search_translation_stall_timeout_sec'
                ).value
            ),
            search_progress_distance_m=float(
                self.get_parameter('search_progress_distance_m').value
            ),
            search_progress_yaw_rad=float(
                self.get_parameter('search_progress_yaw_rad').value
            ),
            search_sweep_viewpoints=int(
                self.get_parameter('search_sweep_viewpoints').value
            ),
            search_sweep_segments=int(
                self.get_parameter('search_sweep_segments').value
            ),
            search_sweep_step_rad=float(
                self.get_parameter('search_sweep_step_rad').value
            ),
            search_sweep_segment_timeout_sec=float(
                self.get_parameter(
                    'search_sweep_segment_timeout_sec'
                ).value
            ),
            transport_plan_timeout_sec=float(
                self.get_parameter('transport_plan_timeout_sec').value
            ),
            camera_approach_candidate_limit=int(
                self.get_parameter('camera_approach_candidate_limit').value
            ),
            manipulation_approach_candidate_limit=int(
                self.get_parameter(
                    'manipulation_approach_candidate_limit'
                ).value
            ),
            camera_dwell_sec=float(
                self.get_parameter('camera_dwell_sec').value
            ),
            camera_confirmation_timeout_sec=float(
                self.get_parameter('camera_confirmation_timeout_sec').value
            ),
            camera_center_timeout_sec=float(
                self.get_parameter('camera_center_timeout_sec').value
            ),
            camera_observation_max_age_sec=float(
                self.get_parameter('camera_observation_max_age_sec').value
            ),
            camera_center_tolerance_rad=float(
                self.get_parameter('camera_center_tolerance_rad').value
            ),
            camera_center_samples=int(
                self.get_parameter('camera_center_samples').value
            ),
            camera_center_max_corrections=int(
                self.get_parameter('camera_center_max_corrections').value
            ),
            camera_center_max_step_rad=float(
                self.get_parameter('camera_center_max_step_rad').value
            ),
            camera_center_gain=float(
                self.get_parameter('camera_center_gain').value
            ),
            camera_center_settle_sec=float(
                self.get_parameter('camera_center_settle_sec').value
            ),
            ultrasonic_required_for_pick=bool(
                self.get_parameter('ultrasonic_required_for_pick').value
            ),
            ultrasonic_max_age_sec=float(
                self.get_parameter('ultrasonic_max_age_sec').value
            ),
            ultrasonic_samples=int(
                self.get_parameter('ultrasonic_samples').value
            ),
            ultrasonic_sensor_offset_x_m=float(
                self.get_parameter('ultrasonic_sensor_offset_x_m').value
            ),
            ultrasonic_object_half_extent_m=float(
                self.get_parameter(
                    'ultrasonic_object_half_extent_m'
                ).value
            ),
            ultrasonic_match_tolerance_m=float(
                self.get_parameter('ultrasonic_match_tolerance_m').value
            ),
            ultrasonic_max_target_bearing_rad=float(
                self.get_parameter(
                    'ultrasonic_max_target_bearing_rad'
                ).value
            ),
            camera_view_distance_m=float(
                self.get_parameter('camera_view_distance_m').value
            ),
            approach_distance_m=float(
                self.get_parameter('approach_distance_m').value
            ),
            drop_approach_distance_m=float(
                self.get_parameter('drop_approach_distance_m').value
            ),
            pick_terminal_min_distance_m=float(
                self.get_parameter('pick_terminal_min_distance_m').value
            ),
            pick_terminal_max_distance_m=float(
                self.get_parameter('pick_terminal_max_distance_m').value
            ),
            pick_terminal_max_bearing_rad=float(
                self.get_parameter('pick_terminal_max_bearing_rad').value
            ),
            pick_terminal_camera_error_rad=float(
                self.get_parameter('pick_terminal_camera_error_rad').value
            ),
            pick_terminal_track_age_sec=float(
                self.get_parameter('pick_terminal_track_age_sec').value
            ),
            drop_terminal_min_distance_m=float(
                self.get_parameter('drop_terminal_min_distance_m').value
            ),
            drop_terminal_max_distance_m=float(
                self.get_parameter('drop_terminal_max_distance_m').value
            ),
            drop_terminal_max_bearing_rad=float(
                self.get_parameter('drop_terminal_max_bearing_rad').value
            ),
            drop_alignment_timeout_sec=float(
                self.get_parameter('drop_alignment_timeout_sec').value
            ),
            terminal_stop_settle_sec=float(
                self.get_parameter('terminal_stop_settle_sec').value
            ),
            placed_object_sync_sec=float(
                self.get_parameter('placed_object_sync_sec').value
            ),
            placed_costmap_settle_sec=float(
                self.get_parameter('placed_costmap_settle_sec').value
            ),
            camera_retry_separation_m=float(
                self.get_parameter('camera_retry_separation_m').value
            ),
            manipulation_retry_separation_m=float(
                self.get_parameter('manipulation_retry_separation_m').value
            ),
            robot_clearance_m=float(
                self.get_parameter('robot_clearance_m').value
            ),
            final_yaw_tolerance_rad=float(
                self.get_parameter('final_yaw_tolerance_rad').value
            ),
            visual_center_events=[],
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.create_publisher(
            String,
            '/mission_state',
            state_qos,
        )
        self.result_publisher = self.create_publisher(
            String,
            '/task_result',
            state_qos,
        )
        self.metrics_publisher = self.create_publisher(
            String,
            '/performance_metrics',
            10,
        )
        self.solver = None
        self.manager_state = 'WAITING_FOR_STACK'
        self.manager_message = ''
        self.started_wall = time.monotonic()
        self.mission_started_wall = None
        self.localization_lost_since = None
        self.localization_scan_attempts = 0
        self.manipulator_wait_started = None
        self.result_published = False
        self.create_timer(0.1, self._tick)
        self.create_timer(0.2, self._publish_visualization)
        diagnostic_period = float(
            self.get_parameter('diagnostic_log_period_sec').value
        )
        if diagnostic_period > 0.0:
            self.create_timer(diagnostic_period, self._publish_diagnostics)
        self.get_logger().info(
            f'mission_manager ready: mission={self.mission.mission_id}, '
            f'task={self.mission.task}, '
            f'obstacles={self.mission.obstacle_mode}, '
            f'coordinates={self.mission.coordinate_mode}, '
            f'map={self.mission.map_yaml}'
        )

    def _tick(self) -> None:
        if self.result_published:
            self._publish_state()
            return
        if self.solver is None:
            self._startup_tick()
        else:
            self._mission_tick()
        self._publish_state()

    def _startup_tick(self) -> None:
        if self.geometry_issues:
            self._finish(
                False,
                'MISSION_GEOMETRY_INVALID',
                '; '.join(str(issue) for issue in self.geometry_issues),
            )
            return
        elapsed = time.monotonic() - self.started_wall
        if elapsed > float(self.get_parameter('startup_timeout_sec').value):
            if (
                task_requires_manipulator(self.mission.task)
                and not self.manipulator.available()
            ):
                self._finish(
                    False,
                    'MANIPULATOR_UNAVAILABLE',
                    'Pick/place adapter did not become available before the '
                    'autonomy stack startup timeout',
                )
                return
            details = {
                'localization_received': self.localization.received,
                'localization_reasons': self.localization.reasons,
                'safety_received': self.safety.received,
                'safety_reasons': self.safety.reasons,
                'nav2_ready': self.navigation.stack_ready(),
                'nav2_inactive_nodes':
                    self.navigation.inactive_lifecycle_nodes(),
                'manipulator_required': task_requires_manipulator(
                    self.mission.task
                ),
                'manipulator_ready': self.manipulator.available(),
            }
            self._finish(False, 'STARTUP_TIMEOUT', json.dumps(details))
            return
        if (
            self.localization.received
            and self.localization.mission_id != self.mission.mission_id
        ):
            self._finish(
                False,
                'MISSION_MISMATCH',
                f'Localization guard uses {self.localization.mission_id}, '
                f'manager uses {self.mission.mission_id}',
            )
            return
        if not self.navigation.stack_ready():
            inactive_nodes = self.navigation.inactive_lifecycle_nodes()
            self._set_manager_state(
                'WAITING_FOR_NAV2',
                (
                    'inactive lifecycle nodes=' + ','.join(inactive_nodes)
                    if inactive_nodes
                    else 'navigate/follow/compute/spin action servers'
                ),
            )
            return

        if self.manager_state == 'LOCALIZATION_ACTIVE_SCAN':
            scan_result = self.navigation.tick()
            if not scan_result.terminal:
                return
            scan_state = scan_result.state.value
            scan_message = scan_result.message
            self.navigation.reset()
            self._set_manager_state(
                'WAITING_FOR_LOCALIZATION',
                f'active scan {scan_state}: {scan_message}',
            )
            return

        if not self.safety.received or not self.safety.safe:
            self._set_manager_state(
                'WAITING_FOR_SAFETY',
                ','.join(self.safety.reasons),
            )
            return
        if not self.localization.received:
            self._set_manager_state('WAITING_FOR_LOCALIZATION_GUARD')
            return
        if not self.localization.ready:
            if (
                self.localization.scan_required
                and self.localization_scan_attempts < 2
            ):
                angle = 0.65 if self.localization_scan_attempts == 0 else -1.30
                if self.navigation.spin(angle, timeout_sec=15.0):
                    self.localization_scan_attempts += 1
                    self._set_manager_state(
                        'LOCALIZATION_ACTIVE_SCAN',
                        f'attempt={self.localization_scan_attempts}',
                    )
                    return
            self._set_manager_state(
                'WAITING_FOR_LOCALIZATION',
                ','.join(self.localization.reasons),
            )
            return

        if (
            task_requires_manipulator(self.mission.task)
            and not self.manipulator.available()
        ):
            now = time.monotonic()
            if self.manipulator_wait_started is None:
                self.manipulator_wait_started = now
            wait_elapsed = now - self.manipulator_wait_started
            wait_limit = float(
                self.get_parameter('manipulator_startup_timeout_sec').value
            )
            if wait_elapsed > wait_limit:
                self._finish(
                    False,
                    'MANIPULATOR_UNAVAILABLE',
                    f'Pick/place adapter unavailable after '
                    f'{wait_elapsed:.1f}s of otherwise-ready stack time',
                )
                return
            self._set_manager_state(
                'WAITING_FOR_MANIPULATOR',
                f'{wait_elapsed:.1f}/{wait_limit:.1f}s',
            )
            return
        self.manipulator_wait_started = None

        if self.navigation.snapshot().terminal:
            self.navigation.reset()
        self.solver = create_solver(self.mission.task, self.mission_context)
        self.mission_started_wall = time.monotonic()
        self.solver.start()
        self._set_manager_state('RUNNING', self.solver.state)

    def _mission_tick(self) -> None:
        if not self.safety.safe:
            self.solver.cancel()
            self._finish(
                False,
                'SAFETY_STOP',
                ','.join(self.safety.reasons),
            )
            return
        if not self.localization.ready:
            if self.localization_lost_since is None:
                self.localization_lost_since = time.monotonic()
                self._set_manager_state(
                    'LOCALIZATION_DEGRADED',
                    ','.join(self.localization.reasons),
                )
                return
            if time.monotonic() - self.localization_lost_since > float(
                self.get_parameter('localization_loss_grace_sec').value
            ):
                self.solver.cancel()
                self._finish(
                    False,
                    'LOCALIZATION_LOST',
                    ','.join(self.localization.reasons),
                )
            return
        self.localization_lost_since = None

        snapshot = self.solver.tick()
        self._set_manager_state('RUNNING', snapshot.state)
        if snapshot.status is SolverStatus.SUCCEEDED:
            self._finish(True, snapshot.state, snapshot.message)
        elif snapshot.status is SolverStatus.FAILED:
            # A transport can fail after PICK. Let the canonical cancellation
            # path release the simulated object before publishing the result.
            self.solver.cancel()
            self._finish(False, snapshot.state, snapshot.message)

    def _publish_visualization(self) -> None:
        self.visualizer.set_tracks(self.objects.all_visible())
        if self.navigation.last_path is not None:
            self.visualizer.set_path(self.navigation.last_path)
        elapsed = 0.0
        if self.mission_started_wall is not None:
            elapsed = time.monotonic() - self.mission_started_wall
        self.visualizer.publish(elapsed)

    def _publish_diagnostics(self) -> None:
        navigation = self.navigation.snapshot()
        pose = self.localization.pose
        metrics = {
            'mission_id': self.mission.mission_id,
            'operation': navigation.operation,
            'navigation_state': navigation.state.value,
            'distance_remaining_m': navigation.distance_remaining,
            'recoveries': navigation.recoveries,
            'pose': None if pose is None else {
                'x': pose.x,
                'y': pose.y,
                'yaw': pose.yaw,
            },
            'velocity': self.navigation.velocity_metrics(),
        }
        self.metrics_publisher.publish(String(data=json.dumps(metrics)))
        if navigation.state.value in {'SENDING', 'ACTIVE'}:
            pose_text = 'unknown'
            if pose is not None:
                pose_text = f'({pose.x:.3f},{pose.y:.3f},{pose.yaw:.3f})'
            remaining = navigation.distance_remaining
            remaining_text = 'unknown' if remaining is None else f'{remaining:.3f}m'
            self.get_logger().info(
                f'nav_progress operation={navigation.operation}, '
                f'pose={pose_text}, remaining={remaining_text}, '
                f'cmd={self.navigation.velocity_summary()}'
            )

    def _set_manager_state(self, state: str, message: str = '') -> None:
        if state != self.manager_state or message != self.manager_message:
            self.get_logger().info(
                f'manager -> {state}' + (f': {message}' if message else '')
            )
        self.manager_state = state
        self.manager_message = message

    def _publish_state(self) -> None:
        elapsed = 0.0
        if self.mission_started_wall is not None:
            elapsed = time.monotonic() - self.mission_started_wall
        state = {
            'mission_id': self.mission.mission_id,
            'task': self.mission.task,
            'obstacle_mode': self.mission.obstacle_mode,
            'coordinate_mode': self.mission.coordinate_mode,
            'state': self.manager_state,
            'detail': self.manager_message,
            'elapsed_sec': elapsed,
            'localization_ready': self.localization.ready,
            'safety_ok': self.safety.safe,
            'navigation': self.navigation.snapshot().state.value,
            'controller': self.visualizer.controller,
        }
        self.state_publisher.publish(String(data=json.dumps(state)))

    def _finish(self, success: bool, reason: str, message: str) -> None:
        if self.result_published:
            return
        self.navigation.cancel()
        elapsed = 0.0
        if self.mission_started_wall is not None:
            elapsed = time.monotonic() - self.mission_started_wall
        result = {
            'mission_id': self.mission.mission_id,
            'task': self.mission.task,
            'success': success,
            'reason': reason,
            'message': message,
            'elapsed_sec': elapsed,
            'recoveries': self.navigation.recoveries,
            'visual_centered_picks': list(
                self.mission_context.visual_center_events
            ),
            'velocity': self.navigation.velocity_metrics(),
            'hardware_validated': False,
        }
        self.result_publisher.publish(String(data=json.dumps(result)))
        self.result_published = True
        self._set_manager_state('SUCCEEDED' if success else 'FAILED', reason)
        log = self.get_logger().info if success else self.get_logger().error
        log(f'task_result: {json.dumps(result)}')

    def destroy_node(self):
        if self.solver is not None and not self.result_published:
            self.solver.cancel()
        self.navigation.cancel()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        spin_node_resilient(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_node(node)
