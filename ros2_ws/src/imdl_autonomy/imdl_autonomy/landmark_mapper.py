"""Put the landmarks a local model detects onto the map, and keep improving them.

This is the half of landmark localisation that has to live where the TF tree
lives -- on the laptop running SLAM and Nav2, per the project's rule that the
TF chain must not be split across machines. It never touches an image. Its
input is the JSON the detector publishes from the Jetson: boxes, intrinsics,
and a range where the camera had depth.

For each detection it does three things:

  1. Turns the box centre into a sight line in the map frame, using TF at the
     image's own timestamp -- not now. During a frontier run the robot is
     usually turning, and a 200 ms error in the lookup is several degrees of
     bearing, which is tens of centimetres at arena ranges.
  2. Attaches a range if one is available: the detector's depth sample, or
     failing that a cast of the bearing at the wall behind the object.
  3. Hands the result to that landmark's track, which re-solves the fused
     position and its covariance over every view held so far.

The estimate therefore tightens as the robot sees a landmark from more
directions, and says so: the covariance ellipse published to RViz shrinks, and
/landmarks/state carries the numbers. Views from one angle barely help and the
ellipse stays long and thin, which is the honest answer -- see
landmark_estimation.py for why that is the right behaviour rather than a
limitation.

Run it alongside the explorer. Nothing needs triggering:

    ros2 launch imdl_autonomy slam_navigation.launch.py run_slam:=true
    ros2 launch imdl_autonomy landmark_mapping.launch.py
    ros2 launch imdl_autonomy frontier_explore.launch.py

    ros2 topic echo /landmarks/state
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from perception.landmark_detection import (
    load_landmark_specs,
    parse_payload,
    touches_image_edge,
)

from .landmark_estimation import (
    LandmarkTrack,
    View,
    first_occupied_along_ray,
)
from .landmark_prior import (
    DATASET_VERSION,
    OCCUPIED_THRESHOLD,
    SAMPLE_CHANNELS,
    SAMPLE_ROBOT_POSE,
    SAMPLE_STAMP,
    WINDOW_CELLS,
    WINDOW_RESOLUTION_M,
    FilePriorProvider,
    NetworkPriorProvider,
    PriorProvider,
    Window,
    build_channels,
    pack_channels,
)

_TF2_GEOMETRY_REGISTRATION = tf2_geometry_msgs


class LandmarkMapper(Node):

    def __init__(self):
        super().__init__('landmark_mapper')

        self.declare_parameter('detections_topic', '/landmarks/detections')
        self.declare_parameter('landmark_config', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('state_topic', '/landmarks/state')
        self.declare_parameter('marker_topic', '/landmarks/markers')
        # Nothing consumes this yet; it keeps the earlier Claude-based
        # dispenser pipeline's topic working for anyone echoing it by hand.
        self.declare_parameter('legacy_dispenser_topic', '/dispenser/pose')
        self.declare_parameter('legacy_dispenser_landmark', 'food_dispenser')

        # Fusion
        self.declare_parameter('max_sigma_m', 0.25)
        self.declare_parameter('nominal_range_m', 2.0)
        self.declare_parameter('gate_sigmas', 3.0)
        self.declare_parameter('views_per_angle_bucket', 6)
        self.declare_parameter('angle_bucket_count', 18)
        # A clipped box's centroid is pulled toward the image centre by
        # however much of the object is missing, so its bearing is biased.
        self.declare_parameter('edge_box_sigma_scale', 2.5)
        # Parked in front of a landmark the detector produces the same view
        # several times a second; one of them is evidence, fifty are not.
        self.declare_parameter('min_view_motion_m', 0.05)
        self.declare_parameter('min_view_angle_deg', 1.0)

        # Wall cast, the single-view fallback when there is no depth
        self.declare_parameter('use_wall_cast', True)
        self.declare_parameter('wall_cast_max_range_m', 6.0)
        self.declare_parameter('wall_cast_skip_m', 0.35)
        self.declare_parameter('wall_cast_occupied_threshold', 65)
        # Deliberately loose: a wall cast is biased by the object's standoff
        # from the wall, so it should bootstrap an estimate and then be
        # outvoted by anything better.
        self.declare_parameter('wall_cast_sigma_m', 0.35)

        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('publish_interval_sec', 1.0)
        self.declare_parameter('next_view_standoff_m', 1.2)
        self.declare_parameter('output_path', '')
        self.declare_parameter('save_interval_sec', 10.0)

        # Where the robot is, needed to centre the prior network's window.
        self.declare_parameter('base_frame', 'base_link')

        # The learned location prior. 'none' is the default because a prior is
        # only as good as its provenance: a stale file or a network trained on
        # a different arena should be opted into deliberately.
        self.declare_parameter('prior_source', 'none')
        self.declare_parameter('prior_file', '')
        self.declare_parameter('prior_file_sigma_m', 0.5)
        self.declare_parameter('prior_network_path', '')
        # A network trained on one arena produces very peaked heatmaps. These
        # two keep it a hint rather than an answer; see landmark_prior.Prior.
        self.declare_parameter('prior_inflate', 1.5)
        self.declare_parameter('prior_min_sigma_m', 0.15)
        self.declare_parameter('prior_update_interval_sec', 2.0)

        # Recording training data for the prior network. Off unless a
        # directory is given.
        self.declare_parameter('dataset_dir', '')
        self.declare_parameter('dataset_interval_sec', 2.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._specs = self._load_specs()
        self._tracks: dict[str, LandmarkTrack] = {
            name: LandmarkTrack(
                name,
                views_per_bucket=int(
                    self.get_parameter('views_per_angle_bucket').value),
                bucket_count=int(
                    self.get_parameter('angle_bucket_count').value),
                nominal_range_m=float(
                    self.get_parameter('nominal_range_m').value),
                max_sigma_m=float(self.get_parameter('max_sigma_m').value),
                gate_sigmas=float(self.get_parameter('gate_sigmas').value),
            )
            for name in self._specs
        }
        self._last_view: dict[str, tuple[np.ndarray, float]] = {}
        self._announced: set[str] = set()
        self._map: OccupancyGrid | None = None
        self._frames = 0
        self._views = 0
        self._dropped_tf = 0
        self._last_save = 0.0
        self._samples = 0
        self._prior_updates = 0
        self._run_dir: Path | None = None
        self._provider: PriorProvider = PriorProvider()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            String, str(self.get_parameter('detections_topic').value),
            self._on_detections,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._on_map, 1)

        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 1)
        self._state_pub = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), 1)
        self._pose_pubs = {
            name: self.create_publisher(
                PoseWithCovarianceStamped, f'/landmarks/{name}/pose', 1)
            for name in self._specs
        }
        self._next_view_pubs = {
            name: self.create_publisher(
                PoseStamped, f'/landmarks/{name}/next_view', 1)
            for name in self._specs
        }
        legacy = str(self.get_parameter('legacy_dispenser_topic').value)
        self._legacy_pub = (
            self.create_publisher(PoseStamped, legacy, 1) if legacy else None)

        self.create_timer(
            float(self.get_parameter('publish_interval_sec').value),
            self._publish)

        self._setup_prior()
        self._setup_dataset()

        self.get_logger().info(
            f'landmark_mapper ready: landmarks={sorted(self._specs)}, '
            f'map_frame={self._map_frame}, '
            f'converged below {self.get_parameter("max_sigma_m").value} m')

    def _load_specs(self) -> dict:
        configured = str(self.get_parameter('landmark_config').value)
        path = (
            Path(configured).expanduser() if configured
            else Path(get_package_share_directory('perception'))
            / 'config' / 'landmarks.yaml')
        if not path.is_file():
            raise FileNotFoundError(f'no landmark config at {path}')
        return load_landmark_specs(
            yaml.safe_load(path.read_text(encoding='utf-8')))

    # -------------------------------------------------------- prior setup

    def _setup_prior(self) -> None:
        """Build the prior provider, or carry on without one.

        A failure here is logged and otherwise ignored. The prior is an
        optimisation -- it gives the robot somewhere to look before it has
        seen anything -- and losing it must never cost the run the
        measurements it would have made anyway.
        """
        source = str(self.get_parameter('prior_source').value)
        if source == 'none':
            return
        try:
            if source == 'file':
                path = str(self.get_parameter('prior_file').value)
                if not path:
                    raise ValueError("prior_source 'file' needs prior_file")
                self._provider = FilePriorProvider.from_yaml(
                    path,
                    sigma_m=float(
                        self.get_parameter('prior_file_sigma_m').value))
            elif source == 'network':
                provider = NetworkPriorProvider(
                    str(self.get_parameter('prior_network_path').value),
                    # Positional output channels, so the order has to be the
                    # order the network was trained with. Sorted names is that
                    # order by construction -- see the training script.
                    sorted(self._specs),
                    inflate=float(self.get_parameter('prior_inflate').value),
                    min_sigma_m=float(
                        self.get_parameter('prior_min_sigma_m').value))
                provider.load()
                self._provider = provider
            else:
                raise ValueError(f'unknown prior_source {source!r}')
        except Exception as exc:  # noqa: BLE001 - never lose the run over this
            self.get_logger().error(
                f'no location prior ({type(exc).__name__}: {exc}). '
                'Localisation still works from the camera alone.')
            return

        self.get_logger().info(f'location prior: {self._provider.describe()}')
        self.create_timer(
            float(self.get_parameter('prior_update_interval_sec').value),
            self._update_priors)

    def _update_priors(self) -> None:
        """Re-run the prior as the map grows and the robot moves.

        The file provider's answer never changes, but the network's does: a
        fuller map is a different input, and its guess should improve over the
        run just as the geometry does.
        """
        window = self._window()
        if window is None:
            return
        channels = self._channels(window)
        if channels is None:
            return
        try:
            priors = self._provider.priors(
                list(self._specs), window, channels)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'prior failed: {exc}', throttle_duration_sec=30.0)
            return
        for name, prior in priors.items():
            self._tracks[name].set_prior(prior)
        if priors:
            self._prior_updates += 1

    def _window(self) -> Window | None:
        """The robot-centric window the network reads, placed by TF.

        Uses the latest transform rather than one at a past stamp: this asks
        'where am I now', unlike a bearing, which must be placed at the
        instant its image was taken.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'no {self._map_frame} -> {self._base_frame} transform, so no '
                f'prior window ({exc})', throttle_duration_sec=15.0)
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
        return Window(
            robot_x=transform.transform.translation.x,
            robot_y=transform.transform.translation.y,
            robot_yaw=yaw,
            cells=WINDOW_CELLS,
            resolution_m=WINDOW_RESOLUTION_M)

    def _channels(self, window: Window) -> np.ndarray | None:
        grid = self._map
        if grid is None:
            return None
        cells = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width)
        # Deliberately not the wall-cast threshold: the network's input is a
        # frozen contract shared with every recorded sample, so it must not
        # move when someone tunes the wall cast.
        return build_channels(
            cells, grid.info.resolution,
            (grid.info.origin.position.x, grid.info.origin.position.y),
            window)

    # ------------------------------------------------------ dataset recording

    def _setup_dataset(self) -> None:
        """Start a run directory for prior-network training samples.

        Each sample is the network's input at that moment. The label -- where
        the landmark actually was -- is only known once the run has driven
        around enough to converge, so it is written separately, and rewritten
        on every publish rather than at shutdown: a run that ends in Ctrl-C or
        a flat battery should still be usable training data.
        """
        directory = str(self.get_parameter('dataset_dir').value)
        if not directory:
            return
        stamp = time.strftime('%Y%m%d_%H%M%S')
        run_dir = Path(directory).expanduser() / f'run_{stamp}'
        try:
            (run_dir / 'samples').mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f'cannot record to {run_dir}: {exc}')
            return
        self._run_dir = run_dir
        self.create_timer(
            float(self.get_parameter('dataset_interval_sec').value),
            self._record_sample)
        self.get_logger().info(f'recording prior training samples to {run_dir}')

    def _record_sample(self) -> None:
        if self._run_dir is None:
            return
        window = self._window()
        if window is None:
            return
        channels = self._channels(window)
        if channels is None:
            return
        # Nothing to learn from a window that has seen nothing yet, and an
        # all-unknown input would teach the network to predict the prior mean
        # for every blank map it ever sees.
        if float(channels[0].sum()) < 10.0:
            return

        path = self._run_dir / 'samples' / f'sample_{self._samples:06d}.npz'
        try:
            np.savez_compressed(path, **{
                SAMPLE_CHANNELS: pack_channels(channels),
                SAMPLE_ROBOT_POSE: np.array(
                    [window.robot_x, window.robot_y, window.robot_yaw],
                    dtype=np.float64),
                SAMPLE_STAMP: np.array(
                    [self.get_clock().now().nanoseconds * 1e-9],
                    dtype=np.float64),
            })
        except OSError as exc:
            self.get_logger().warn(
                f'could not write {path}: {exc}', throttle_duration_sec=30.0)
            return
        self._samples += 1

    def _write_run_labels(self, state) -> None:
        """The labels for this run's samples: where each landmark ended up.

        Only converged landmarks are written as labels. An unconverged
        estimate is a guess, and training a prior on guesses would teach the
        network this pipeline's own mistakes.
        """
        if self._run_dir is None:
            return
        labels = {
            name: {
                'x': entry['position']['x'],
                'y': entry['position']['y'],
                'sigma_major_m': entry['sigma_major_m'],
                'views': entry['views_used'],
                'sources': entry['sources'],
            }
            for name, entry in state['landmarks'].items()
            if entry['converged']
        }
        document = {
            'dataset_version': DATASET_VERSION,
            'window': {
                'cells': WINDOW_CELLS,
                'resolution_m': WINDOW_RESOLUTION_M,
                'occupied_threshold': OCCUPIED_THRESHOLD,
            },
            'map_frame': self._map_frame,
            'samples': self._samples,
            'labels': labels,
        }
        try:
            target = self._run_dir / 'run.yaml'
            temporary = target.with_suffix('.yaml.tmp')
            temporary.write_text(
                '# Written by landmark_mapper. Labels are the fused positions\n'
                '# this run converged on, in map_frame.\n'
                + yaml.safe_dump(document, sort_keys=True), encoding='utf-8')
            temporary.replace(target)
        except OSError as exc:
            self.get_logger().warn(
                f'could not write run labels: {exc}',
                throttle_duration_sec=30.0)

    # ------------------------------------------------------------- inputs

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_detections(self, msg: String):
        try:
            frame = parse_payload(msg.data)
        except (ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'unusable detection payload: {exc}')
            return
        self._frames += 1
        for detection in frame.detections:
            spec = self._specs.get(detection.landmark)
            if spec is None:
                self.get_logger().warn(
                    f'detector reported {detection.landmark!r}, which this '
                    'node has no config for; detector and mapper are reading '
                    'different landmarks.yaml files', once=True)
                continue
            self._add_view(frame, detection, spec)

    def _add_view(self, frame, detection, spec) -> None:
        u, v = detection.centre_px(frame.width, frame.height)
        ray = self._sight_line(frame, u, v)
        if ray is None:
            return
        origin, direction = ray

        if not self._is_new_viewpoint(detection.landmark, origin, direction):
            return

        bearing_sigma = spec.bearing_sigma_rad
        if touches_image_edge(detection.box_norm):
            bearing_sigma *= float(
                self.get_parameter('edge_box_sigma_scale').value)

        range_m = detection.range_m
        range_sigma = detection.range_sigma_m
        source = detection.range_source if range_m is not None else 'bearing'

        if range_m is None and self._should_wall_cast(detection.landmark):
            hit = self._wall_cast(origin, direction)
            if hit is not None:
                range_m = float(np.linalg.norm(np.asarray(hit) - origin))
                range_sigma = float(
                    self.get_parameter('wall_cast_sigma_m').value)
                source = 'wall'

        self._tracks[detection.landmark].add(View(
            origin=(float(origin[0]), float(origin[1])),
            direction=(float(direction[0]), float(direction[1])),
            bearing_sigma_rad=bearing_sigma,
            range_m=range_m,
            range_sigma_m=range_sigma,
            source=source,
            confidence=detection.confidence,
            stamp_sec=frame.stamp_sec + frame.stamp_nanosec * 1e-9,
        ))
        self._last_view[detection.landmark] = (
            origin, math.atan2(direction[1], direction[0]) % math.pi)
        self._views += 1

    def _is_new_viewpoint(self, name: str, origin, direction) -> bool:
        """Reject a view that repeats the last one from the same spot."""
        previous = self._last_view.get(name)
        if previous is None:
            return True
        last_origin, last_angle = previous
        moved = float(np.linalg.norm(origin - last_origin))
        angle = math.atan2(direction[1], direction[0]) % math.pi
        delta = abs(angle - last_angle) % math.pi
        turned = min(delta, math.pi - delta)
        return (
            moved >= float(self.get_parameter('min_view_motion_m').value)
            or turned >= math.radians(
                float(self.get_parameter('min_view_angle_deg').value))
        )

    def _should_wall_cast(self, name: str) -> bool:
        """Cast at walls only until the geometry can stand on its own.

        The cast's standoff error is a bias. It is worth accepting to get a
        first position out of a single webcam view, and worth dropping the
        moment triangulation has a real baseline, because then it would only
        pull a good estimate toward the wall.
        """
        if not self.get_parameter('use_wall_cast').value or self._map is None:
            return False
        estimate = self._tracks[name].estimate()
        return estimate is None or not estimate.converged

    def _wall_cast(self, origin, direction):
        grid = self._map
        if grid is None:
            return None
        cells = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width)
        return first_occupied_along_ray(
            cells,
            grid.info.resolution,
            (grid.info.origin.position.x, grid.info.origin.position.y),
            origin,
            direction,
            skip_m=float(self.get_parameter('wall_cast_skip_m').value),
            max_range_m=float(
                self.get_parameter('wall_cast_max_range_m').value),
            occupied_threshold=int(
                self.get_parameter('wall_cast_occupied_threshold').value),
        )

    # ------------------------------------------------------------ geometry

    def _sight_line(self, frame, u: float, v: float):
        """The bearing as (origin, unit direction) in the map, flattened.

        Two points on the ray are transformed rather than the direction being
        rotated by hand: TF then owns the whole optical-frame convention, so a
        sign error in the camera extrinsics shows up as a landmark in the
        wrong place rather than as a silently plausible one.
        """
        bearing = frame.intrinsics.bearing(u, v)
        points = []
        for scale in (0.0, 1.0):
            point = PointStamped()
            point.header.frame_id = frame.frame_id
            point.header.stamp.sec = frame.stamp_sec
            point.header.stamp.nanosec = frame.stamp_nanosec
            point.point.x = bearing[0] * scale
            point.point.y = bearing[1] * scale
            point.point.z = bearing[2] * scale
            mapped = self._to_map(point)
            if mapped is None:
                return None
            points.append(np.array(mapped[:2]))

        origin, along = points
        flat = along - origin
        norm = float(np.linalg.norm(flat))
        if norm < 1e-6:
            self.get_logger().error(
                'the sight line is vertical in the map frame; check the '
                'camera pitch in the extrinsics', once=True)
            return None
        return origin, flat / norm

    def _to_map(self, point: PointStamped):
        timeout = rclpy.duration.Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))
        try:
            in_map = self._tf_buffer.transform(
                point, self._map_frame, timeout=timeout)
        except TransformException as exc:
            self._dropped_tf += 1
            # Throttled: while the robot is turning fast or the detector's
            # payload arrives late over WiFi, this can fire on every view, and
            # the useful information is that it keeps happening.
            self.get_logger().warn(
                f'no transform {point.header.frame_id} -> {self._map_frame} '
                f'at the image stamp ({exc}); {self._dropped_tf} views lost '
                'to TF so far', throttle_duration_sec=5.0)
            return None
        return (in_map.point.x, in_map.point.y, in_map.point.z)

    # ------------------------------------------------------------- outputs

    def _publish(self):
        markers = MarkerArray()
        state: dict[str, object] = {
            'frames': self._frames,
            'views': self._views,
            'views_lost_to_tf': self._dropped_tf,
            'landmarks': {},
        }
        stamp = self.get_clock().now().to_msg()

        for index, (name, track) in enumerate(sorted(self._tracks.items())):
            estimate = track.estimate()
            if estimate is None:
                continue
            spec = self._specs[name]
            state['landmarks'][name] = estimate.as_dict()  # type: ignore[index]

            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = self._map_frame
            pose.header.stamp = stamp
            pose.pose.pose.position.x = estimate.x
            pose.pose.pose.position.y = estimate.y
            pose.pose.pose.orientation.w = 1.0
            # The 6x6 is row-major (x, y, z, roll, pitch, yaw); only the xy
            # block is measured, and the rest is left at zero to say so.
            covariance = [0.0] * 36
            covariance[0], covariance[1] = estimate.covariance[0], estimate.covariance[1]
            covariance[6], covariance[7] = estimate.covariance[2], estimate.covariance[3]
            pose.pose.covariance = covariance
            self._pose_pubs[name].publish(pose)

            if (self._legacy_pub is not None and name == str(
                    self.get_parameter('legacy_dispenser_landmark').value)):
                legacy = PoseStamped()
                legacy.header = pose.header
                legacy.pose = pose.pose.pose
                self._legacy_pub.publish(legacy)

            markers.markers.extend(
                self._markers_for(estimate, spec, index, stamp))
            self._publish_next_view(name, estimate, stamp)
            self._announce(estimate)

        state['samples_recorded'] = self._samples
        state['prior'] = self._provider.describe()

        self._marker_pub.publish(markers)
        self._state_pub.publish(String(data=json.dumps(state)))
        self._save(state)
        self._write_run_labels(state)

    def _markers_for(self, estimate, spec, index: int, stamp):
        """A sphere at the estimate, the 2-sigma ellipse, and a label.

        The ellipse is the point of the display: it is how an operator sees
        that a landmark is pinned down rather than merely detected, and which
        way to drive to improve it.
        """
        base_id = index * 10
        red, green, blue = spec.color

        sphere = Marker()
        sphere.header.frame_id = self._map_frame
        sphere.header.stamp = stamp
        sphere.ns = 'landmarks'
        sphere.id = base_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = estimate.x
        sphere.pose.position.y = estimate.y
        sphere.pose.position.z = 0.10
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = spec.marker_scale_m
        sphere.color.r, sphere.color.g, sphere.color.b = red, green, blue
        sphere.color.a = 0.9 if estimate.converged else 0.4

        ellipse = Marker()
        ellipse.header = sphere.header
        ellipse.ns = 'landmark_uncertainty'
        ellipse.id = base_id + 1
        ellipse.type = Marker.CYLINDER
        ellipse.action = Marker.ADD
        ellipse.pose.position.x = estimate.x
        ellipse.pose.position.y = estimate.y
        ellipse.pose.position.z = 0.01
        half = estimate.major_axis_rad / 2.0
        ellipse.pose.orientation.z = math.sin(half)
        ellipse.pose.orientation.w = math.cos(half)
        # Clamped, because an unconstrained axis is tens of metres and an RViz
        # disc that size hides the whole map.
        ellipse.scale.x = 2.0 * min(estimate.sigma_major_m, 5.0) * 2.0
        ellipse.scale.y = 2.0 * min(estimate.sigma_minor_m, 5.0) * 2.0
        ellipse.scale.z = 0.01
        ellipse.color.r, ellipse.color.g, ellipse.color.b = red, green, blue
        ellipse.color.a = 0.25

        label = Marker()
        label.header = sphere.header
        label.ns = 'landmark_labels'
        label.id = base_id + 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = estimate.x
        label.pose.position.y = estimate.y
        label.pose.position.z = 0.40
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = (
            f'{estimate.name}\n'
            f'+/-{estimate.sigma_major_m:.2f} m  {estimate.used_count} views  '
            f'{math.degrees(estimate.angular_spread_rad):.0f} deg baseline')
        return [sphere, ellipse, label]

    def _publish_next_view(self, name: str, estimate, stamp) -> None:
        """Where to stand to shrink this landmark fastest.

        The long axis of the ellipse is the direction nothing has measured
        yet, and a sight line across it is what measures it. Published as a
        pose so it can be handed straight to Nav2 if anyone wants the
        exploration to care about landmark quality.
        """
        if estimate.converged:
            return
        standoff = float(self.get_parameter('next_view_standoff_m').value)
        heading = estimate.best_next_view_rad
        candidates = [
            np.array([estimate.x - math.cos(heading) * standoff,
                      estimate.y - math.sin(heading) * standoff]),
            np.array([estimate.x + math.cos(heading) * standoff,
                      estimate.y + math.sin(heading) * standoff]),
        ]
        chosen = next(
            (point for point in candidates if self._is_free(point)), None)
        if chosen is None:
            return

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(chosen[0])
        pose.pose.position.y = float(chosen[1])
        # Face the landmark, so the camera sees it on arrival.
        yaw = math.atan2(estimate.y - chosen[1], estimate.x - chosen[0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._next_view_pubs[name].publish(pose)

    def _is_free(self, point) -> bool:
        """True when the map says this cell is known and clear.

        Unknown is not free here, unlike in the wall cast: suggesting the
        robot drive into unexplored space is how a hint becomes a hazard.
        """
        grid = self._map
        if grid is None:
            return False
        column = int(
            (point[0] - grid.info.origin.position.x) / grid.info.resolution)
        row = int(
            (point[1] - grid.info.origin.position.y) / grid.info.resolution)
        if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
            return False
        value = grid.data[row * grid.info.width + column]
        return 0 <= value < 50

    def _announce(self, estimate) -> None:
        """One log line per landmark when it first becomes usable."""
        if not estimate.converged or estimate.name in self._announced:
            return
        self._announced.add(estimate.name)
        note = (
            ' NOTE: ranged only by wall casts, so it is biased by however far '
            'the object stands off the wall'
            if estimate.biased_by_wall_casts else '')
        self.get_logger().info(
            f'{estimate.name} localised at ({estimate.x:.2f}, '
            f'{estimate.y:.2f}) to +/-{estimate.sigma_major_m:.2f} m from '
            f'{estimate.used_count} views over a '
            f'{math.degrees(estimate.angular_spread_rad):.0f} deg baseline '
            f'({"/".join(estimate.sources)}).{note}')

    def _save(self, state) -> None:
        """Write the landmarks beside the map, so a restart keeps them."""
        path = str(self.get_parameter('output_path').value)
        if not path:
            return
        interval = float(self.get_parameter('save_interval_sec').value)
        now = time.monotonic()
        if now - self._last_save < interval:
            return
        self._last_save = now
        document = {
            'frame_id': self._map_frame,
            'landmarks': state['landmarks'],
        }
        target = Path(path).expanduser()
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            # Written through a temporary file: the mission code may read this
            # while exploration is still running, and a half-written YAML is
            # worse than a slightly stale one.
            temporary = target.with_suffix(target.suffix + '.tmp')
            temporary.write_text(
                '# Written by landmark_mapper; positions are in frame_id.\n'
                + yaml.safe_dump(document, sort_keys=True),
                encoding='utf-8')
            temporary.replace(target)
        except OSError as exc:
            self.get_logger().warn(
                f'could not write {target}: {exc}', throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkMapper()
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
