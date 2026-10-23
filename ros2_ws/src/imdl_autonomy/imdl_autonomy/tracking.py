"""Constant-position Kalman tracking with gating and finite track lifetime."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Detection:
    color: str
    x: float
    y: float
    z: float
    stamp_sec: float
    variance: float = 0.0025


@dataclass
class Track:
    object_id: str
    color: str
    position: np.ndarray
    covariance: np.ndarray
    z: float
    created_sec: float
    updated_sec: float
    confirmations: int = 1
    rejected_updates: int = 0
    carried: bool = False

    def age(self, now_sec: float) -> float:
        return max(0.0, now_sec - self.updated_sec)

    def confidence(self, now_sec: float, stale_after_sec: float) -> float:
        confirmation_term = 1.0 - math.exp(-self.confirmations / 2.0)
        uncertainty_term = math.exp(-12.0 * float(np.trace(self.covariance)))
        freshness_term = math.exp(-self.age(now_sec) / max(stale_after_sec, 1e-3))
        return float(max(
            0.0,
            min(
                1.0,
                confirmation_term * uncertainty_term * freshness_term,
            ),
        ))


class MultiObjectTracker:
    """Tracks each color independently and rejects statistically implausible jumps."""

    def __init__(
        self,
        gate_chi2: float = 9.21,
        process_variance_per_sec: float = 0.0004,
        stale_after_sec: float = 1.5,
        lifetime_sec: float = 5.0,
        confirmations_required: int = 3,
    ):
        self.gate_chi2 = float(gate_chi2)
        self.process_variance_per_sec = float(process_variance_per_sec)
        self.stale_after_sec = float(stale_after_sec)
        self.lifetime_sec = float(lifetime_sec)
        self.confirmations_required = int(confirmations_required)
        self._tracks: Dict[str, Track] = {}
        self._next_id = 1

    @property
    def tracks(self) -> Tuple[Track, ...]:
        return tuple(self._tracks.values())

    def update(self, detections: Iterable[Detection], now_sec: float) -> None:
        self.prune(now_sec)
        for detection in sorted(detections, key=lambda item: item.color):
            candidates = [
                track for track in self._tracks.values()
                if track.color == detection.color and not track.carried
            ]
            matched = self._best_match(candidates, detection, now_sec)
            if matched is None:
                self._create_track(detection)
            else:
                self._kalman_update(matched, detection)

    def prune(self, now_sec: float) -> None:
        expired = [
            object_id for object_id, track in self._tracks.items()
            if not track.carried and track.age(now_sec) > self.lifetime_sec
        ]
        for object_id in expired:
            self._tracks.pop(object_id, None)

    def mark_carried(self, object_id: str) -> bool:
        track = self._tracks.get(object_id)
        if track is None:
            return False
        track.carried = True
        return True

    def remove(self, object_id: str) -> bool:
        return self._tracks.pop(object_id, None) is not None

    def status(self, track: Track, now_sec: float) -> str:
        if track.carried:
            return 'CARRIED'
        if track.age(now_sec) > self.stale_after_sec:
            return 'STALE'
        if track.confirmations >= self.confirmations_required:
            return 'CONFIRMED'
        return 'TENTATIVE'

    def _best_match(
        self,
        tracks: Sequence[Track],
        detection: Detection,
        now_sec: float,
    ) -> Optional[Track]:
        best = None
        best_distance = self.gate_chi2
        measurement = np.array([detection.x, detection.y], dtype=float)
        for track in tracks:
            dt = max(0.0, now_sec - track.updated_sec)
            predicted_covariance = track.covariance + np.eye(2) * (
                self.process_variance_per_sec * dt
            )
            innovation_covariance = predicted_covariance + np.eye(2) * max(
                detection.variance, 1e-6
            )
            difference = measurement - track.position
            try:
                distance = float(
                    difference.T @ np.linalg.inv(innovation_covariance) @ difference
                )
            except np.linalg.LinAlgError:
                distance = math.inf
            if distance <= best_distance:
                best = track
                best_distance = distance
        return best

    def _create_track(self, detection: Detection) -> Track:
        object_id = f'{detection.color}-{self._next_id:03d}'
        self._next_id += 1
        track = Track(
            object_id=object_id,
            color=detection.color,
            position=np.array([detection.x, detection.y], dtype=float),
            covariance=np.eye(2) * max(detection.variance, 1e-6),
            z=float(detection.z),
            created_sec=detection.stamp_sec,
            updated_sec=detection.stamp_sec,
        )
        self._tracks[object_id] = track
        return track

    def _kalman_update(self, track: Track, detection: Detection) -> None:
        if detection.stamp_sec <= track.updated_sec + 1e-6:
            track.rejected_updates += 1
            return
        dt = max(0.0, detection.stamp_sec - track.updated_sec)
        predicted_covariance = track.covariance + np.eye(2) * (
            self.process_variance_per_sec * dt
        )
        measurement_covariance = np.eye(2) * max(detection.variance, 1e-6)
        innovation_covariance = predicted_covariance + measurement_covariance
        kalman_gain = predicted_covariance @ np.linalg.inv(innovation_covariance)
        measurement = np.array([detection.x, detection.y], dtype=float)
        track.position = track.position + kalman_gain @ (
            measurement - track.position
        )
        track.covariance = (
            np.eye(2) - kalman_gain
        ) @ predicted_covariance
        track.updated_sec = max(track.updated_sec, detection.stamp_sec)
        track.z = 0.75 * track.z + 0.25 * float(detection.z)
        track.confirmations += 1
