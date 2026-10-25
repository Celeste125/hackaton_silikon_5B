"""Metric object projection and atomic, per-map-session JSON persistence."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import uuid

import numpy as np

from imdl_autonomy.landmark_estimation import View, solve_landmark


@dataclass(frozen=True)
class FusionConfig:
    bearing_sigma_rad: float = 0.10
    min_range_sigma_m: float = 0.15
    relative_range_sigma: float = 0.15
    position_sigma_floor_m: float = 0.08
    outlier_sigmas: float = 3.0

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('fusion parameters must be positive and finite')


def _view(record, config):
    origin = record['camera_origin_odom_m']
    point = record['position_odom_m']
    dx, dy = point['x'] - origin['x'], point['y'] - origin['y']
    distance = math.hypot(dx, dy)
    confidence = float(record['confidence'])
    residual = float(record['fit_residual'])
    growth = float(record['growth_ratio'])
    fitted = record.get('range_sigma_m')
    fitted = None if fitted is None else float(fitted)
    if (not all(math.isfinite(v) for v in (*origin.values(), *point.values(),
                                          confidence, residual, growth))
            or distance <= 1e-6 or not 0 < confidence <= 1
            or residual < 0 or growth <= 0
            or (fitted is not None and (not math.isfinite(fitted) or fitted <= 0))):
        raise ValueError('invalid multi-view measurement')
    # The probe's own propagated sigma when it recorded one; residual/growth
    # is the pre-schema-3 stand-in for it. Either way the floors stay: they
    # cover what the fit cannot see -- odometry scale and camera extrinsics --
    # and those are systematic, so no number of frames averages them away.
    range_sigma = max(config.min_range_sigma_m,
                      distance * config.relative_range_sigma,
                      fitted if fitted is not None
                      else distance * residual / max(growth, 0.01)) / confidence
    bearing_sigma = max(config.bearing_sigma_rad,
                        config.position_sigma_floor_m / distance)
    stamp = record.get('source_image_stamp', {})
    return View(
        origin=(origin['x'], origin['y']), direction=(dx / distance, dy / distance),
        bearing_sigma_rad=bearing_sigma, range_m=distance, range_sigma_m=range_sigma,
        confidence=confidence, source='image_expansion_and_odometry',
        stamp_sec=float(stamp.get('sec', 0)) + float(stamp.get('nanosec', 0)) / 1e9)


def _point_covariance(view):
    direction = np.asarray(view.direction)
    normal = np.array([-direction[1], direction[0]])
    cross_sigma = view.range_m * view.bearing_sigma_rad / view.confidence
    return (view.range_sigma_m ** 2 * np.outer(direction, direction)
            + cross_sigma ** 2 * np.outer(normal, normal))


def _consensus(records, views, previous_used, config):
    """Choose the largest compatible group, not a gate around the first view.

    Conflicting views stay in the bounded history. Two later agreeing views
    can replace a bad first view; a single later outlier cannot overwrite an
    established group. Ties favour the previously used group.
    """
    points = [np.array([r['position_odom_m'][a] for a in ('x', 'y')]) for r in records]
    covariances = [_point_covariance(v) for v in views]
    groups = []
    for i, point in enumerate(points):
        group = []
        for j, other in enumerate(points):
            delta = point - other
            covariance = covariances[i] + covariances[j]
            if float(delta @ np.linalg.solve(covariance, delta)) <= config.outlier_sigmas ** 2:
                group.append(j)
        groups.append(group)
    return max(groups, key=lambda group: (
        len(group), sum(records[i]['measurement_index'] in previous_used for i in group)))


def _fuse(landmark, records, previous_used, config):
    views = [_view(record, config) for record in records]
    selected = _consensus(records, views, previous_used, config)
    # Outliers were selected as a group above. The shared solver now combines
    # the retained bearing and range constraints with directional weights.
    estimate = solve_landmark(
        landmark, [views[i] for i in selected], min_views_to_gate=len(selected) + 1)
    if estimate is None:
        raise ValueError('no usable views for object fusion')
    covariance = np.array(estimate.covariance).reshape(2, 2)
    covariance += np.eye(2) * config.position_sigma_floor_m ** 2
    z = float(np.median([records[i]['position_odom_m']['z'] for i in selected]))
    return {
        'position_odom_m': {'x': estimate.x, 'y': estimate.y, 'z': z},
        'covariance_xy_odom_m2': covariance.reshape(-1).tolist(),
        'sigma_major_m': float(math.sqrt(np.linalg.eigvalsh(covariance)[-1])),
        'view_angle_spread_deg': math.degrees(estimate.angular_spread_rad),
        'used_measurement_count': len(selected),
        'rejected_measurement_count': len(records) - len(selected),
        'retained_measurements': [{**r, 'used': i in selected} for i, r in enumerate(records)],
        'uncertainty_model': 'heuristic_bearing_range_with_systematic_floor',
    }


def project_box(box, width, height, intrinsics, depth_m, matrix, translation):
    """Transform the box centre at optical Z=depth_m into a fixed frame.

    Expansion estimates optical depth, not the length of the viewing ray.
    Using a normalized ray times depth would put off-centre targets too close.
    """
    u = (box[0] + box[2]) * width / 2
    v = (box[1] + box[3]) * height / 2
    point = np.array([(u - intrinsics.cx) / intrinsics.fx * depth_m,
                      (v - intrinsics.cy) / intrinsics.fy * depth_m, depth_m])
    if not math.isfinite(depth_m) or depth_m <= 0:
        raise ValueError('object depth must be positive and finite')
    return transform_point(point, matrix, translation)


def transform_point(point, matrix, translation):
    result = np.asarray(matrix) @ np.asarray(point) + np.asarray(translation)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('object position must be finite and three-dimensional')
    return {axis: float(value) for axis, value in zip(('x', 'y', 'z'), result)}


class ObjectPositionStore:
    """One dispenser and one cow stall, identified by canonical landmark name.

    Do not reuse an old run implicitly: a fresh SLAM map can have a completely
    different origin. A caller-specified existing file is refused, not erased.
    """

    def __init__(self, path='', map_frame='map', odom_frame='odom', map_id='', fusion_config=None):
        self.fusion_config = fusion_config or FusionConfig()
        now = datetime.now(timezone.utc)
        self.session_id = now.strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
        self.path = (Path(path).expanduser() if path else
                     Path.home() / 'maps' / 'detected_objects' / f'{self.session_id}.json')
        if self.path.exists():
            raise FileExistsError(f'choose a new positions_file; {self.path} already exists')
        self.document = {
            'schema_version': 3, 'session_id': self.session_id,
            'map_id': map_id or self.session_id,
            'frame_id': map_frame, 'odom_frame_id': odom_frame,
            'units': 'metres', 'objects': {},
        }

    @property
    def objects(self):
        return self.document['objects']

    def save(self, landmark, entry, map_rotation=None, map_translation=None):
        previous = self.objects.get(landmark, {})
        history = list(previous.get('retained_measurements', []))
        stamp = entry.get('source_image_stamp')
        if stamp is not None and any(
                r.get('source_image_stamp') == stamp
                and r.get('camera_frame') == entry.get('camera_frame') for r in history):
            return previous  # the same image is not independent evidence
        count = previous.get('measurement_count', 0) + 1
        history.append({**entry, 'measurement_index': count})
        # Keep three recent views per 15-degree sight-line bucket (max 36).
        # Revisiting one doorway cannot erase all observations from other angles.
        buckets = {}
        for record in history:
            view = _view(record, self.fusion_config)
            bucket = min(11, int(view.line_angle_rad() / math.pi * 12))
            buckets.setdefault(bucket, []).append(record)
        history = sorted(
            [record for bucket in buckets.values() for record in bucket[-3:]],
            key=lambda r: r['measurement_index'])
        previous_used = {r['measurement_index'] for r in previous.get('retained_measurements', [])
                         if r['used']}
        fused = _fuse(landmark, history, previous_used, self.fusion_config)
        matrix = np.eye(3) if map_rotation is None else np.asarray(map_rotation)
        translation = (0, 0, 0) if map_translation is None else map_translation
        anchor = fused['position_odom_m']
        fused['position_map_m'] = transform_point(
            [anchor[a] for a in ('x', 'y', 'z')], matrix, translation)
        covariance = np.array(fused['covariance_xy_odom_m2']).reshape(2, 2)
        fused['covariance_xy_map_m2'] = (
            matrix[:2, :2] @ covariance @ matrix[:2, :2].T).reshape(-1).tolist()
        last_used = any(r['measurement_index'] == count and r['used']
                        for r in fused['retained_measurements'])
        fused.update({
            'method': 'multi_view_expansion_fusion',
            'measurement_count': count,
            'retained_measurement_count': len(history),
            'last_measurement_used': last_used,
            'fusion_status': ('single_view' if count == 1 else
                              'disagreement' if fused['rejected_measurement_count'] else 'fused'),
        })
        objects = {**self.objects, landmark: fused}
        self._commit(objects)
        return fused

    def reproject(self, matrix, translation):
        """Refresh global map corrections without taking a new size measurement."""
        objects = {}
        changed = False
        for name, entry in self.objects.items():
            anchor = entry['position_odom_m']
            mapped = transform_point([anchor[a] for a in ('x', 'y', 'z')], matrix, translation)
            old = entry['position_map_m']
            changed |= math.dist([mapped[a] for a in ('x', 'y', 'z')],
                                 [old[a] for a in ('x', 'y', 'z')]) >= 0.01
            rotation_xy = np.asarray(matrix)[:2, :2]
            covariance = np.array(entry['covariance_xy_odom_m2']).reshape(2, 2)
            mapped_covariance = (rotation_xy @ covariance @ rotation_xy.T).reshape(-1).tolist()
            changed |= not np.allclose(mapped_covariance, entry['covariance_xy_map_m2'], atol=1e-6)
            objects[name] = {**entry, 'position_map_m': mapped,
                             'covariance_xy_map_m2': mapped_covariance}
        if changed:
            self._commit(objects)
        return changed

    def _commit(self, objects):
        document = {**self.document, 'objects': objects,
                    'updated_at_utc': datetime.now(timezone.utc).isoformat()}
        content = json.dumps(document, indent=2, allow_nan=False) + '\n'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode='w', encoding='utf-8', dir=self.path.parent,
                    prefix='.' + self.path.name, suffix='.tmp', delete=False) as stream:
                temporary = stream.name
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary is not None and os.path.exists(temporary):
                os.unlink(temporary)
        # A failed write never marks an object as successfully recorded.
        self.document = document
