"""Reusable non-blocking approach, pick, transport, and place flow."""

from __future__ import annotations

from enum import Enum
import math
from typing import Iterable, List

from ..mission_spec import Pose2D
from ..navigation_adapter import NavigationState
from ..next_best_view import ApproachCandidate
from ..range_fusion import evaluate_front_range, target_geometry
from ..runtime_context import ManipulatorState
from ..visual_alignment import VisualAlignment, VisualAlignmentStatus


class FlowStatus(str, Enum):
    """Lifecycle result of a complete object transport flow."""

    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


def select_diverse_approaches(
    candidates: Iterable[ApproachCandidate],
    limit: int,
    minimum_separation_m: float,
) -> List[ApproachCandidate]:
    """Keep high-scoring alternatives that lead to meaningfully new views."""
    accepted = [candidate for candidate in candidates if candidate.accepted]
    selected = []
    for candidate in accepted:
        if all(
            candidate.pose.distance_to(chosen.pose) >= minimum_separation_m
            for chosen in selected
        ):
            selected.append(candidate)
            if len(selected) >= limit:
                return selected

    # Sparse maps may offer fewer separated poses. Preserve remaining valid
    # alternatives instead of turning a recoverable mission into a hard fail.
    for candidate in accepted:
        if candidate not in selected:
            selected.append(candidate)
            if len(selected) >= limit:
                break
    return selected


class TransportFlow:
    """Coordinate safe approach, confirmation, pick, transport, and place."""

    def __init__(self, context):
        """Create an idle transport state machine for one mission context."""
        self.context = context
        self.status = FlowStatus.RUNNING
        self.state = 'IDLE'
        self.message = ''
        self.color = ''
        self.object_id = ''
        self.estimated_pose = None
        self.drop_pose = None
        self.stack_index = 0
        self.approaches = []
        self.pick_approaches = []
        self.drop_approaches = []
        self.drop_navigation_pose = None
        self.approach_index = 0
        self.pick_index = 0
        self.drop_index = 0
        self.state_started = 0.0
        self.require_confirmation = True
        self.pending_center_evidence = None
        self.range_verify_after_wall = None
        self.picked = False
        self.visual_alignment = VisualAlignment(context)

    def start(
        self,
        color: str,
        estimated_pose: Pose2D,
        drop_pose: Pose2D,
        object_id: str = '',
        stack_index: int = 0,
        require_confirmation: bool = True,
    ) -> None:
        """Plan camera approaches and begin transporting one tracked object."""
        self.color = color
        self.object_id = object_id
        self.estimated_pose = estimated_pose
        self.drop_pose = drop_pose
        self.stack_index = stack_index
        self.require_confirmation = require_confirmation
        self.pending_center_evidence = None
        self.range_verify_after_wall = None
        self.picked = False
        current = self.context.current_pose() or self.context.mission.start
        self.approaches = self.context.view_planner.approach_ring(
            current,
            estimated_pose,
            radius_m=self.context.camera_view_distance_m,
        )
        self.context.visualizer.set_approaches(self.approaches)
        accepted = select_diverse_approaches(
            self.approaches,
            limit=int(getattr(
                self.context,
                'camera_approach_candidate_limit',
                8,
            )),
            minimum_separation_m=self.context.camera_retry_separation_m,
        )
        if not accepted:
            self._fail('NO_APPROACH_POSE')
            return
        self.approaches = accepted
        self.approach_index = 0
        self._transition('START_APPROACH_PLAN')

    def tick(self) -> FlowStatus:
        """Advance the non-blocking transport state machine once."""
        if self.status is not FlowStatus.RUNNING:
            return self.status
        if self.state == 'START_APPROACH_PLAN':
            self._start_approach_plan()
        elif self.state == 'WAIT_APPROACH_PLAN':
            self._wait_approach_plan()
        elif self.state == 'START_APPROACH':
            self._start_approach()
        elif self.state == 'WAIT_APPROACH':
            self._wait_approach()
        elif self.state == 'WAIT_APPROACH_TERMINAL_STOP':
            self._wait_terminal_stop(
                'VERIFY_OBJECT',
                self._retry_approach,
            )
        elif self.state == 'VERIFY_OBJECT':
            self._verify_object()
        elif self.state == 'PLAN_PICK_APPROACH':
            self._plan_pick_approach()
        elif self.state == 'START_PICK_APPROACH_PLAN':
            self._start_pick_approach_plan()
        elif self.state == 'WAIT_PICK_APPROACH_PLAN':
            self._wait_pick_approach_plan()
        elif self.state == 'START_PICK_APPROACH':
            self._start_pick_approach()
        elif self.state == 'WAIT_PICK_APPROACH':
            self._wait_pick_approach()
        elif self.state == 'WAIT_PICK_TERMINAL_STOP':
            self._wait_terminal_stop(
                'START_VISUAL_CENTER',
                self._retry_pick_approach,
            )
        elif self.state == 'START_VISUAL_CENTER':
            self._start_visual_center()
        elif self.state == 'ALIGN_VISUAL_CENTER':
            self._align_visual_center()
        elif self.state == 'FINAL_VERIFY_OBJECT':
            self._final_verify_object()
        elif self.state == 'START_PICK':
            self._start_pick()
        elif self.state == 'WAIT_PICK':
            self._wait_pick()
        elif self.state == 'PLAN_DROP_APPROACH':
            self._plan_drop_approach()
        elif self.state == 'START_DROP_APPROACH_PLAN':
            self._start_drop_approach_plan()
        elif self.state == 'WAIT_DROP_APPROACH_PLAN':
            self._wait_drop_approach_plan()
        elif self.state == 'START_DROP_APPROACH':
            self._start_drop_approach()
        elif self.state == 'WAIT_DROP_APPROACH':
            self._wait_drop_approach()
        elif self.state == 'START_DROP_ALIGNMENT':
            self._start_drop_alignment()
        elif self.state == 'WAIT_DROP_ALIGNMENT':
            self._wait_drop_alignment()
        elif self.state == 'WAIT_DROP_TERMINAL_STOP':
            self._wait_terminal_stop(
                'START_PLACE',
                self._retry_drop,
            )
        elif self.state == 'START_PLACE':
            self._start_place()
        elif self.state == 'WAIT_PLACE':
            self._wait_place()
        elif self.state == 'WAIT_PLACED_OBJECT_SYNC':
            self._wait_placed_object_sync()
        elif self.state == 'WAIT_PLACED_COSTMAP_SETTLE':
            self._wait_placed_costmap_settle()
        return self.status

    def _start_approach_plan(self) -> None:
        self._start_path_preflight(
            self.approaches[self.approach_index],
            label='camera approach',
            wait_state='WAIT_APPROACH_PLAN',
            reject=self._reject_approach_plan,
        )

    def _wait_approach_plan(self) -> None:
        self._wait_path_preflight(
            success_state='START_APPROACH',
            reject=self._reject_approach_plan,
        )

    def _start_approach(self) -> None:
        candidate = self.approaches[self.approach_index]
        self.context.logger.info(
            f'camera view {self.approach_index + 1}/{len(self.approaches)}: '
            f'({candidate.pose.x:.3f}, {candidate.pose.y:.3f}, '
            f'{candidate.pose.yaw:.3f}), '
            f'static_path={candidate.path_length_m:.3f}m, '
            f'clearance={candidate.clearance_m:.3f}m'
        )
        self._reset_navigation()
        self.context.visualizer.set_active_goal(candidate.pose, 'RobustMPPI')
        if self.context.navigation.navigate_to(
            candidate.pose,
            controller='RobustMPPI',
            timeout_sec=self.context.navigation_timeout_sec,
            precise=True,
        ):
            self._transition('WAIT_APPROACH')
        else:
            self._retry_approach(self.context.navigation.message)

    def _wait_approach(self) -> None:
        terminal = self._camera_view_terminal()
        if terminal is not None:
            self.context.navigation.cancel()
            self._transition('WAIT_APPROACH_TERMINAL_STOP', terminal)
            return
        snapshot = self.context.navigation.tick()
        if snapshot.state is NavigationState.SUCCEEDED:
            # The Nav2 goal checker requires the camera-facing ring yaw.
            # Object confirmation is the independent proof that the selected
            # view is usable, so a second Spin would add no information.
            self._transition('VERIFY_OBJECT')
        elif snapshot.terminal:
            self._retry_approach(snapshot.message)

    def _verify_object(self) -> None:
        track = self.context.objects.confirmed(
            self.color,
            reference=self.estimated_pose,
            max_age_sec=2.5,
            min_confidence=0.38,
        )
        if track is not None:
            max_refinement = 0.20 if self.require_confirmation else 0.35
            if track.pose.distance_to(self.estimated_pose) <= max_refinement:
                self.object_id = track.object_id
                self.estimated_pose = track.pose
                self._transition(
                    'PLAN_PICK_APPROACH',
                    f'track={track.object_id}, '
                    f'confidence={track.confidence:.2f}, '
                    f'age={track.age_sec:.2f}s',
                )
                return
        if (
            self._state_elapsed()
            > self.context.camera_confirmation_timeout_sec
        ):
            self._retry_approach('camera confirmation timed out')

    def _plan_pick_approach(self) -> None:
        current = self.context.current_pose() or self.approaches[
            self.approach_index
        ].pose
        candidates = self.context.view_planner.approach_ring(
            current,
            self.estimated_pose,
            radius_m=self.context.approach_distance_m,
        )
        self.context.visualizer.set_approaches(candidates)
        self.pick_approaches = select_diverse_approaches(
            candidates,
            limit=int(getattr(
                self.context,
                'manipulation_approach_candidate_limit',
                8,
            )),
            minimum_separation_m=self.context.manipulation_retry_separation_m,
        )
        if not self.pick_approaches:
            self._retry_approach('no collision-free pick approach')
            return
        self.pick_index = 0
        self._transition('START_PICK_APPROACH_PLAN')

    def _start_pick_approach_plan(self) -> None:
        self._start_path_preflight(
            self.pick_approaches[self.pick_index],
            label='pick approach',
            wait_state='WAIT_PICK_APPROACH_PLAN',
            reject=self._reject_pick_approach_plan,
        )

    def _wait_pick_approach_plan(self) -> None:
        self._wait_path_preflight(
            success_state='START_PICK_APPROACH',
            reject=self._reject_pick_approach_plan,
        )

    def _start_pick_approach(self) -> None:
        candidate = self.pick_approaches[self.pick_index]
        self.context.logger.info(
            f'pick approach {self.pick_index + 1}/'
            f'{len(self.pick_approaches)}: '
            f'({candidate.pose.x:.3f}, {candidate.pose.y:.3f}, '
            f'{candidate.pose.yaw:.3f}), '
            f'static_path={candidate.path_length_m:.3f}m, '
            f'clearance={candidate.clearance_m:.3f}m'
        )
        self._reset_navigation()
        self.context.visualizer.set_active_goal(
            candidate.pose,
            'RobustMPPI + camera',
        )
        if self.context.navigation.navigate_to(
            candidate.pose,
            controller='RobustMPPI',
            timeout_sec=min(35.0, self.context.navigation_timeout_sec),
            precise=True,
        ):
            self._transition('WAIT_PICK_APPROACH')
        else:
            self._retry_pick_approach(self.context.navigation.message)

    def _wait_pick_approach(self) -> None:
        terminal = self._pick_terminal()
        if terminal is not None:
            self.context.navigation.cancel()
            self._transition('WAIT_PICK_TERMINAL_STOP', terminal)
            return
        snapshot = self.context.navigation.tick()
        if snapshot.state is NavigationState.SUCCEEDED:
            # The ring pose already faces the object. A second geometric Spin
            # here can be rejected because the object is intentionally close
            # to the footprint. The camera closes the remaining yaw error and
            # provides the evidence required by PICK.
            self._transition('START_VISUAL_CENTER')
        elif snapshot.terminal:
            self._retry_pick_approach(snapshot.message)

    def _start_visual_center(self) -> None:
        status = self.visual_alignment.start(self.color)
        if status is VisualAlignmentStatus.SUCCEEDED:
            self._visual_center_succeeded()
        elif status is VisualAlignmentStatus.FAILED:
            self._retry_pick_approach(self.visual_alignment.message)
        else:
            self._transition(
                'ALIGN_VISUAL_CENTER',
                self.visual_alignment.message,
            )

    def _align_visual_center(self) -> None:
        status = self.visual_alignment.tick()
        self.message = self.visual_alignment.message
        if status is VisualAlignmentStatus.SUCCEEDED:
            self._visual_center_succeeded()
        elif status is VisualAlignmentStatus.FAILED:
            self._retry_pick_approach(self.visual_alignment.message)

    def _final_verify_object(self) -> None:
        track = self.context.objects.confirmed(
            self.color,
            reference=self.estimated_pose,
            max_age_sec=1.5,
            min_confidence=0.38,
        )
        observation = self.visual_alignment.centered_observation
        centered = (
            observation is not None
            and abs(observation.horizontal_error_rad)
            <= self.context.camera_center_tolerance_rad
        )
        realsense_expected_m = None
        realsense_error_m = None
        realsense_ok = True
        if (
            track is not None
            and centered
            and getattr(observation, 'source', 'unknown') == 'typed'
        ):
            robot = self.context.current_pose()
            if robot is not None:
                realsense_expected_m = robot.distance_to(track.pose)
            measured_range = getattr(observation, 'range_m', None)
            if measured_range is None or realsense_expected_m is None:
                realsense_ok = False
            else:
                realsense_error_m = abs(
                    float(measured_range) - realsense_expected_m
                )
                realsense_ok = realsense_error_m <= float(getattr(
                    self.context,
                    'realsense_range_match_tolerance_m',
                    0.06,
                ))
        range_evaluation = None
        range_ok = not bool(getattr(
            self.context,
            'ultrasonic_required_for_pick',
            False,
        ))
        if track is not None and centered:
            range_evaluation = self._front_range_evaluation(track.pose)
            if range_evaluation.measured_range_m is not None:
                # A contradictory return is safety information even when an
                # optional hardware sensor was discovered automatically.
                range_ok = range_evaluation.consistent
        if (
            track is not None
            and track.pose.distance_to(self.estimated_pose) <= 0.12
            and centered
            and range_ok
        ):
            self.object_id = track.object_id
            self.estimated_pose = track.pose
            self.pending_center_evidence = {
                'color': self.color,
                'object_id': self.object_id,
                'generation': int(observation.generation),
                'sensor_stamp_sec': getattr(observation, 'stamp_sec', None),
                'pixel_x': float(observation.cx),
                'principal_x': float(observation.principal_x),
                'horizontal_error_rad': float(
                    observation.horizontal_error_rad
                ),
                'perception_source': getattr(
                    observation,
                    'source',
                    'unknown',
                ),
                'perception_confidence': float(getattr(
                    observation,
                    'confidence',
                    1.0,
                )),
                'realsense_depth_m': getattr(
                    observation,
                    'depth_m',
                    None,
                ),
                'realsense_range_m': getattr(
                    observation,
                    'range_m',
                    None,
                ),
                'realsense_expected_range_m': realsense_expected_m,
                'realsense_range_error_m': realsense_error_m,
                'realsense_bearing_rad': getattr(
                    observation,
                    'bearing_rad',
                    None,
                ),
                'ultrasonic_used': (
                    range_evaluation is not None
                    and range_evaluation.measured_range_m is not None
                ),
                'ultrasonic_range_m': (
                    None if range_evaluation is None
                    else range_evaluation.measured_range_m
                ),
                'ultrasonic_expected_m': (
                    None if range_evaluation is None
                    else range_evaluation.expected_range_m
                ),
                'ultrasonic_error_m': (
                    None if range_evaluation is None
                    else range_evaluation.error_m
                ),
                'ultrasonic_samples': (
                    0 if range_evaluation is None
                    else range_evaluation.sample_count
                ),
            }
            range_text = 'ultrasonic unavailable (optional)'
            if range_evaluation is not None:
                if range_evaluation.measured_range_m is None:
                    range_text = range_evaluation.reason
                else:
                    range_text = (
                        f'ultrasonic={range_evaluation.measured_range_m:.3f}m, '
                        f'expected={range_evaluation.expected_range_m:.3f}m, '
                        f'error={range_evaluation.error_m:.3f}m'
                    )
            realsense_text = 'RealSense metric evidence unavailable'
            if getattr(observation, 'source', 'unknown') == 'typed':
                realsense_text = (
                    f'RealSense range={observation.range_m:.3f}m, '
                    f'map expected={realsense_expected_m:.3f}m, '
                    f'error={realsense_error_m:.3f}m'
                )
            self._transition(
                'START_PICK',
                f'fresh centered frame generation={observation.generation}, '
                f'error={observation.horizontal_error_rad:.3f}rad; '
                f'{range_text}',
            )
            return
        if (
            self._state_elapsed()
            > self.context.camera_confirmation_timeout_sec
        ):
            reason = 'final camera/track confirmation timed out'
            if track is not None and not centered:
                reason += '; block is not centered in image'
            if (
                track is not None
                and centered
                and range_evaluation is not None
                and not range_evaluation.consistent
            ):
                reason += (
                    f'; {range_evaluation.reason}: measured='
                    f'{range_evaluation.measured_range_m}, expected='
                    f'{range_evaluation.expected_range_m:.3f}m'
                )
            self._retry_pick_approach(reason)

    def _start_pick(self) -> None:
        self._reset_manipulator()
        if not self.context.manipulator.pick(
            self.object_id,
            self.color,
            self.estimated_pose,
        ):
            self._retry_pick_approach(self.context.manipulator.message)
            return
        self._transition('WAIT_PICK')

    def _wait_pick(self) -> None:
        state = self.context.manipulator.tick()
        if state is ManipulatorState.SUCCEEDED:
            self.picked = True
            evidence = getattr(
                self.context,
                'visual_center_events',
                None,
            )
            if evidence is not None and self.pending_center_evidence is not None:
                evidence.append(self.pending_center_evidence)
            self.pending_center_evidence = None
            self.context.objects.lock(self.object_id)
            self._transition('PLAN_DROP_APPROACH')
        elif state in {
            ManipulatorState.FAILED,
            ManipulatorState.TIMED_OUT,
            ManipulatorState.CANCELED,
        }:
            self._retry_pick_approach(self.context.manipulator.message)

    def _plan_drop_approach(self) -> None:
        current = self.context.current_pose() or self.approaches[
            self.approach_index
        ].pose
        candidates = self.context.view_planner.approach_ring(
            current,
            self.drop_pose,
            radius_m=self.context.drop_approach_distance_m,
        )
        self.context.visualizer.set_approaches(candidates)
        self.drop_approaches = select_diverse_approaches(
            candidates,
            limit=int(getattr(
                self.context,
                'manipulation_approach_candidate_limit',
                8,
            )),
            minimum_separation_m=self.context.manipulation_retry_separation_m,
        )
        if not self.drop_approaches:
            self._fail('NO_DROP_APPROACH_POSE')
            return
        self.drop_index = 0
        self.drop_navigation_pose = None
        self._transition('START_DROP_APPROACH_PLAN')

    def _start_drop_approach_plan(self) -> None:
        candidate = self.drop_approaches[self.drop_index]
        self.drop_navigation_pose = self._forward_transit_pose(candidate.pose)
        transit_candidate = ApproachCandidate(
            pose=self.drop_navigation_pose,
            score=candidate.score,
            path_length_m=candidate.path_length_m,
            clearance_m=candidate.clearance_m,
            camera_error_rad=candidate.camera_error_rad,
            accepted=candidate.accepted,
            reason=candidate.reason,
        )
        self._start_path_preflight(
            transit_candidate,
            label='forward drop transit',
            wait_state='WAIT_DROP_APPROACH_PLAN',
            reject=self._reject_drop_approach_plan,
        )

    def _wait_drop_approach_plan(self) -> None:
        self._wait_path_preflight(
            success_state='START_DROP_APPROACH',
            reject=self._reject_drop_approach_plan,
        )

    def _start_drop_approach(self) -> None:
        candidate = self.drop_approaches[self.drop_index]
        navigation_pose = (
            self.drop_navigation_pose
            or self._forward_transit_pose(candidate.pose)
        )
        self.context.logger.info(
            f'drop approach {self.drop_index + 1}/'
            f'{len(self.drop_approaches)}: '
            f'({candidate.pose.x:.3f}, {candidate.pose.y:.3f}, '
            f'{candidate.pose.yaw:.3f}), '
            f'forward_transit_yaw={navigation_pose.yaw:.3f}, '
            f'static_path={candidate.path_length_m:.3f}m, '
            f'clearance={candidate.clearance_m:.3f}m'
        )
        self._reset_navigation()
        self.context.visualizer.set_active_goal(
            navigation_pose,
            'RobustMPPI forward drop transit',
        )
        if self.context.navigation.navigate_to(
            navigation_pose,
            controller='RobustMPPI',
            timeout_sec=self.context.navigation_timeout_sec,
            precise=True,
        ):
            self._transition('WAIT_DROP_APPROACH')
        else:
            self._retry_drop(self.context.navigation.message)

    def _wait_drop_approach(self) -> None:
        terminal = self._drop_terminal()
        if terminal is not None:
            self.context.navigation.cancel()
            self._transition('WAIT_DROP_TERMINAL_STOP', terminal)
            return
        snapshot = self.context.navigation.tick()
        if snapshot.state is NavigationState.SUCCEEDED:
            self._transition(
                'START_DROP_ALIGNMENT',
                'forward transit reached; aligning to drop center',
            )
        elif snapshot.terminal:
            self._retry_drop(snapshot.message)

    def _start_drop_alignment(self) -> None:
        robot = self.context.current_pose()
        if robot is None:
            self._retry_drop('robot pose unavailable before drop alignment')
            return
        geometry = target_geometry(robot, self.drop_pose)
        minimum = float(getattr(
            self.context,
            'drop_terminal_min_distance_m',
            0.23,
        ))
        maximum = float(getattr(
            self.context,
            'drop_terminal_max_distance_m',
            0.33,
        ))
        if not minimum <= geometry.distance_m <= maximum:
            self._retry_drop(
                'forward transit ended outside drop annulus: '
                f'distance={geometry.distance_m:.3f}m'
            )
            return
        maximum_bearing = float(getattr(
            self.context,
            'drop_terminal_max_bearing_rad',
            0.22,
        ))
        if abs(geometry.bearing_rad) <= maximum_bearing:
            self._transition(
                'START_PLACE',
                f'drop center aligned: bearing={geometry.bearing_rad:.3f}rad',
            )
            return
        self._reset_navigation()
        facing = Pose2D(
            robot.x,
            robot.y,
            robot.yaw + geometry.bearing_rad,
        )
        self.context.visualizer.set_active_goal(
            facing,
            'Nav2 drop-center alignment',
        )
        timeout = float(getattr(
            self.context,
            'drop_alignment_timeout_sec',
            12.0,
        ))
        if self.context.navigation.spin(
            geometry.bearing_rad,
            timeout_sec=timeout,
        ):
            self._transition(
                'WAIT_DROP_ALIGNMENT',
                f'aligning {geometry.bearing_rad:.3f}rad to C',
            )
        else:
            self._retry_drop(self.context.navigation.message)

    def _wait_drop_alignment(self) -> None:
        snapshot = self.context.navigation.tick()
        if snapshot.state is NavigationState.SUCCEEDED:
            terminal = self._drop_terminal()
            if terminal is None:
                self._retry_drop(
                    'drop alignment completed outside validated reach/bearing'
                )
                return
            self._transition('START_PLACE', terminal)
        elif snapshot.terminal:
            self._retry_drop(snapshot.message)

    def _camera_view_terminal(self):
        """Accept a useful live view without forcing one exact ring sample."""
        track = self.context.objects.confirmed(
            self.color,
            reference=self.estimated_pose,
            max_age_sec=1.0,
            min_confidence=0.38,
        )
        observation = self.context.camera.observation(
            self.color,
            max_age_sec=float(self.context.camera_observation_max_age_sec),
        )
        if track is None or observation is None:
            return None
        robot = self.context.current_pose()
        if robot is None:
            return None
        if robot.distance_to(track.pose) > (
            float(self.context.camera_view_distance_m) + 0.15
        ):
            return None
        max_refinement = 0.20 if self.require_confirmation else 0.35
        if track.pose.distance_to(self.estimated_pose) > max_refinement:
            return None
        self.object_id = track.object_id
        self.estimated_pose = track.pose
        return (
            f'camera already sees {track.object_id}; accepting live view '
            f'at image error={observation.horizontal_error_rad:.3f}rad'
        )

    def _pick_terminal(self):
        """Recognize a safe annular manipulation pose while Nav2 is active."""
        robot = self.context.current_pose()
        if robot is None:
            return None
        track = self.context.objects.confirmed(
            self.color,
            reference=self.estimated_pose,
            max_age_sec=float(getattr(
                self.context,
                'pick_terminal_track_age_sec',
                1.0,
            )),
            min_confidence=0.38,
        )
        observation = self.context.camera.observation(
            self.color,
            max_age_sec=float(self.context.camera_observation_max_age_sec),
        )
        if track is None or observation is None:
            return None
        if track.pose.distance_to(self.estimated_pose) > 0.12:
            return None
        geometry = target_geometry(robot, track.pose)
        if not (
            float(getattr(
                self.context,
                'pick_terminal_min_distance_m',
                0.24,
            ))
            <= geometry.distance_m
            <= float(getattr(
                self.context,
                'pick_terminal_max_distance_m',
                0.335,
            ))
        ):
            return None
        if abs(geometry.bearing_rad) > float(
            getattr(
                self.context,
                'pick_terminal_max_bearing_rad',
                0.55,
            )
        ):
            return None
        if abs(observation.horizontal_error_rad) > float(
            getattr(
                self.context,
                'pick_terminal_camera_error_rad',
                0.58,
            )
        ):
            return None
        self.object_id = track.object_id
        self.estimated_pose = track.pose
        self.context.visualizer.set_active_goal(
            track.pose,
            'camera-range manipulation terminal',
        )
        return (
            f'valid pick annulus: distance={geometry.distance_m:.3f}m, '
            f'bearing={geometry.bearing_rad:.3f}rad, '
            f'image_error={observation.horizontal_error_rad:.3f}rad'
        )

    def _drop_terminal(self):
        """Accept any reachable, correctly facing pose around the drop center."""
        robot = self.context.current_pose()
        if robot is None:
            return None
        geometry = target_geometry(robot, self.drop_pose)
        if not (
            float(getattr(
                self.context,
                'drop_terminal_min_distance_m',
                0.23,
            ))
            <= geometry.distance_m
            <= float(getattr(
                self.context,
                'drop_terminal_max_distance_m',
                0.33,
            ))
        ):
            return None
        if abs(geometry.bearing_rad) > float(
            getattr(
                self.context,
                'drop_terminal_max_bearing_rad',
                0.22,
            )
        ):
            return None
        return (
            f'valid drop annulus: distance={geometry.distance_m:.3f}m, '
            f'bearing={geometry.bearing_rad:.3f}rad'
        )

    def _wait_terminal_stop(self, next_state, retry) -> None:
        """Wait for canceled Nav2 motion to settle before sensing or acting."""
        settle = max(0.0, float(getattr(
            self.context,
            'terminal_stop_settle_sec',
            0.30,
        )))
        elapsed = self._state_elapsed()
        if elapsed < settle:
            return
        stationary = getattr(
            self.context.navigation,
            'robot_stationary',
            lambda: True,
        )()
        if stationary:
            self._transition(next_state, 'robot stationary after Nav2 cancel')
            return
        if elapsed > settle + 1.5:
            retry('robot did not stop after manipulation-terminal cancel')

    def _visual_center_succeeded(self) -> None:
        """Start a post-alignment range sample window."""
        self.range_verify_after_wall = self.context.monotonic()
        self._transition(
            'FINAL_VERIFY_OBJECT',
            self.visual_alignment.message,
        )

    def _front_range_evaluation(self, target: Pose2D):
        store = getattr(self.context, 'front_range', None)
        reading = None
        if store is not None:
            reading = store.measurement(
                max_age_sec=float(getattr(
                    self.context,
                    'ultrasonic_max_age_sec',
                    0.35,
                )),
                min_samples=int(getattr(
                    self.context,
                    'ultrasonic_samples',
                    3,
                )),
                not_before_wall=self.range_verify_after_wall,
            )
        robot = self.context.current_pose()
        if robot is None:
            robot = self.pick_approaches[self.pick_index].pose
        return evaluate_front_range(
            reading,
            robot,
            target,
            sensor_offset_x_m=float(
                getattr(
                    self.context,
                    'ultrasonic_sensor_offset_x_m',
                    0.190,
                )
            ),
            target_half_extent_m=float(
                getattr(
                    self.context,
                    'ultrasonic_object_half_extent_m',
                    0.0175,
                )
            ),
            tolerance_m=float(
                getattr(
                    self.context,
                    'ultrasonic_match_tolerance_m',
                    0.035,
                )
            ),
            max_target_bearing_rad=float(
                getattr(
                    self.context,
                    'ultrasonic_max_target_bearing_rad',
                    0.10,
                )
            ),
        )

    def _start_place(self) -> None:
        self._reset_manipulator()
        if not self.context.manipulator.place(
            self.object_id,
            self.color,
            self.drop_pose,
            self.stack_index,
        ):
            self._fail(self.context.manipulator.message)
            return
        self._transition('WAIT_PLACE')

    def _wait_place(self) -> None:
        state = self.context.manipulator.tick()
        if state is ManipulatorState.SUCCEEDED:
            self._transition(
                'WAIT_PLACED_OBJECT_SYNC',
                'placement confirmed; waiting for obstacle-fusion update',
            )
        elif state in {
            ManipulatorState.FAILED,
            ManipulatorState.TIMED_OUT,
            ManipulatorState.CANCELED,
        }:
            self._fail(self.context.manipulator.message)

    def _wait_placed_object_sync(self) -> None:
        """Let placement events reach safety nodes, then clear stale marks."""
        if self._state_elapsed() < float(getattr(
            self.context,
            'placed_object_sync_sec',
            0.35,
        )):
            return
        cleared = self.context.navigation.clear_costmaps()
        detail = (
            'stale local/global obstacle marks cleared'
            if cleared
            else 'costmap clear services unavailable; relying on live clearing'
        )
        self._transition('WAIT_PLACED_COSTMAP_SETTLE', detail)

    def _wait_placed_costmap_settle(self) -> None:
        if self._state_elapsed() < float(getattr(
            self.context,
            'placed_costmap_settle_sec',
            0.45,
        )):
            return
        self.status = FlowStatus.SUCCEEDED
        self._transition('TRANSPORT_SUCCEEDED')

    def _retry_approach(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.approach_index += 1
        if self.approach_index >= len(self.approaches):
            self._fail(f'APPROACH_FAILED: {reason}')
            return
        self.context.navigation.clear_costmaps()
        self._transition(
            'START_APPROACH_PLAN',
            f'alternative {self.approach_index + 1}: {reason}',
        )

    def _retry_drop(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.drop_navigation_pose = None
        self.drop_index += 1
        if self.drop_index >= len(self.drop_approaches):
            self._fail(f'DROP_APPROACH_FAILED: {reason}')
            return
        self.context.navigation.clear_costmaps()
        self._transition(
            'START_DROP_APPROACH_PLAN',
            f'alternative {self.drop_index + 1}: {reason}',
        )

    def _retry_pick_approach(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.pending_center_evidence = None
        self.pick_index += 1
        if self.pick_index >= len(self.pick_approaches):
            self._retry_approach(f'pick approaches exhausted: {reason}')
            return
        self.context.navigation.clear_costmaps()
        self._transition(
            'START_PICK_APPROACH_PLAN',
            f'alternative {self.pick_index + 1}: {reason}',
        )

    def _reject_approach_plan(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.approach_index += 1
        if self.approach_index >= len(self.approaches):
            self._fail(f'NO_LIVE_PATH_TO_CAMERA_APPROACH: {reason}')
            return
        self._transition(
            'START_APPROACH_PLAN',
            f'candidate {self.approach_index + 1}: {reason}',
        )

    def _reject_pick_approach_plan(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.pick_index += 1
        if self.pick_index >= len(self.pick_approaches):
            self._retry_approach(
                f'no live-costmap path to pick ring: {reason}'
            )
            return
        self._transition(
            'START_PICK_APPROACH_PLAN',
            f'candidate {self.pick_index + 1}: {reason}',
        )

    def _reject_drop_approach_plan(self, reason: str) -> None:
        self.context.navigation.cancel()
        self.drop_navigation_pose = None
        self.drop_index += 1
        if self.drop_index >= len(self.drop_approaches):
            self._fail(f'NO_LIVE_PATH_TO_DROP_APPROACH: {reason}')
            return
        self._transition(
            'START_DROP_APPROACH_PLAN',
            f'candidate {self.drop_index + 1}: {reason}',
        )

    def _start_path_preflight(
        self,
        candidate: ApproachCandidate,
        label: str,
        wait_state: str,
        reject,
    ) -> None:
        self._reset_navigation()
        self.context.visualizer.set_active_goal(
            candidate.pose,
            f'{label} live-costmap preflight',
        )
        timeout = float(getattr(
            self.context,
            'transport_plan_timeout_sec',
            5.0,
        ))
        if self.context.navigation.compute_path(
            candidate.pose,
            timeout_sec=timeout,
        ):
            self._transition(wait_state)
        else:
            reject(
                f'{label} preflight could not start: '
                f'{self.context.navigation.message}'
            )

    def _wait_path_preflight(self, success_state: str, reject) -> None:
        snapshot = self.context.navigation.tick()
        if snapshot.state is NavigationState.SUCCEEDED:
            self._transition(success_state, 'live-costmap path is feasible')
        elif snapshot.terminal:
            reject(f'path preflight failed: {snapshot.message}')

    def _reset_navigation(self) -> None:
        if self.context.navigation.snapshot().terminal:
            self.context.navigation.reset()

    def _reset_manipulator(self) -> None:
        if self.context.manipulator.state is not ManipulatorState.IDLE:
            self.context.manipulator.reset()

    def _face_target(self, target: Pose2D, fallback: Pose2D) -> Pose2D:
        current = self.context.current_pose()
        if current is None or current.distance_to(target) < 1e-4:
            return fallback
        return Pose2D(
            current.x,
            current.y,
            math.atan2(target.y - current.y, target.x - current.x),
        )

    def _forward_transit_pose(self, target: Pose2D) -> Pose2D:
        """Face the travel direction so a non-reversing controller can leave."""
        current = self.context.current_pose()
        if current is None or current.distance_to(target) < 0.02:
            return target
        return Pose2D(
            target.x,
            target.y,
            math.atan2(target.y - current.y, target.x - current.x),
        )

    def _transition(self, state: str, message: str = '') -> None:
        self.state = state
        self.message = message
        self.state_started = self.context.monotonic()
        self.context.logger.info(
            f'transport[{self.color}] -> {state}'
            + (f': {message}' if message else '')
        )

    def _state_elapsed(self) -> float:
        return max(0.0, self.context.monotonic() - self.state_started)

    def _fail(self, message: str) -> None:
        self.status = FlowStatus.FAILED
        self._transition('TRANSPORT_FAILED', message)
