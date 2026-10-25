"""ROS-independent pause, align, measure, and resume controller.

For a static target, apparent height s = k / (Z0 - travel). Fitting 1/s
against measured travel estimates Z0 without knowing k or the object size.
Samples must belong to one rigid feature and one straight, fixed-heading leg.
"""

from dataclasses import dataclass
import math

import numpy as np


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Observation:
    stamp: float
    landmark: str
    model_class: str
    box: tuple[float, float, float, float]
    bearing: float  # positive left, relative to base_link
    size: float  # bounding box height / fy
    pose: Pose  # odometry interpolated at the image capture time
    confidence: float


@dataclass(frozen=True)
class Config:
    min_confidence: float = 0.65
    detection_timeout: float = 0.8
    stop_timeout: float = 8.0
    settle_time: float = 0.5
    alignment_timeout: float = 20.0
    alignment_tolerance: float = 0.035
    alignment_frames: int = 3
    angular_gain: float = 1.0
    max_angular_speed: float = 0.25
    approach_speed: float = 0.06
    max_probe_m: float = 0.25
    approach_timeout: float = 12.0
    progress_timeout: float = 3.0
    retry_cooldown: float = 20.0
    retry_viewpoint_m: float = 0.40
    remeasure_cooldown: float = 30.0
    remeasure_viewpoint_m: float = 0.50
    max_heading_error: float = 0.05
    max_lateral_m: float = 0.04
    max_bearing_error: float = 0.08
    min_baseline_m: float = 0.08
    min_growth: float = 0.04
    target_growth: float = 0.20
    min_samples: int = 5
    max_range_sigma_ratio: float = 0.12
    max_range_m: float = 10.0
    association_iou: float = 0.20

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('approach parameters must be positive and finite')
        if not 0 < self.min_confidence <= 1 or not 0 < self.association_iou < 1:
            raise ValueError('confidence and association thresholds must be in [0, 1]')
        if not self.min_baseline_m < self.max_probe_m:
            raise ValueError('require min_baseline_m < max_probe_m')
        if self.min_samples < 4 or self.alignment_frames < 2:
            raise ValueError('require at least 4 range samples and 2 aligned frames')
        if not self.min_growth < self.target_growth:
            raise ValueError('require min_growth < target_growth')
        if not 0 < self.max_range_sigma_ratio < 1:
            raise ValueError('max_range_sigma_ratio must be a fraction of the range')


@dataclass(frozen=True)
class RangeEstimate:
    initial_m: float
    current_m: float
    residual: float
    sigma_m: float
    samples: int


class ExpansionEstimator:
    """Fit one straight probe leg, once, over every sample it collected.

    `estimate` is deliberately not a per-frame stopping rule. Stopping at the
    first fit that clears a threshold picks the shortest baseline available
    and, among noise realisations, the one that happens to look cleanest --
    the least precise answer on offer, reported with the most confidence. How
    far to drive is decided by `enough_growth` instead: apparent size against
    measured travel is geometry, and noise cannot flatter it into stopping
    early. The fit then runs on the whole leg.

    The gate is the fit's own propagated sigma on the range, not the residual
    about the mean of 1/size. Those differ by a large factor: with 4% growth
    the signal spans ~4% of mean(1/size), so a residual of 2% about that mean
    is half the signal and still looks small. Sigma knows the difference.
    """

    def __init__(self, config):
        self.config = config
        self.samples = []

    def add(self, travel, size):
        """Record one sample. Returns whether it was taken, not an estimate."""
        if not math.isfinite(travel) or not math.isfinite(size) or size <= 0:
            return False
        # Stationary frames add no geometric baseline and can bias a fit.
        if self.samples and travel - self.samples[-1][0] < 0.002:
            return False
        self.samples.append((travel, size))
        return True

    def growth(self):
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1][1] / self.samples[0][1] - 1.0

    def enough_growth(self):
        """Is the leg long enough to stop driving? Geometry only, never fit quality."""
        return (len(self.samples) >= self.config.min_samples
                and self.samples[-1][0] - self.samples[0][0] >= self.config.min_baseline_m
                and self.growth() >= self.config.target_growth)

    def estimate(self):
        c = self.config
        if len(self.samples) < c.min_samples:
            return None
        x, sizes = np.asarray(self.samples).T
        if x[-1] - x[0] < c.min_baseline_m or self.growth() < c.min_growth:
            return None
        y = 1.0 / sizes
        design = np.column_stack((x, np.ones_like(x)))
        slope, intercept = np.linalg.lstsq(design, y, rcond=None)[0]
        if slope >= -1e-9:
            return None
        deviations = y - (intercept + slope * x)
        residual = float(np.sqrt(np.mean(deviations ** 2)) / np.mean(y))
        initial = float(-intercept / slope)
        remaining = initial - x[-1]
        if not 0 < remaining <= c.max_range_m:
            return None
        # Propagate the fit's scatter into the answer. Z0 = -intercept/slope,
        # so d/d(intercept) = -1/slope and d/d(slope) = intercept/slope**2;
        # the two are strongly correlated and the cross term is not optional.
        variance = float(deviations @ deviations) / max(len(x) - 2, 1)
        try:
            covariance = variance * np.linalg.inv(design.T @ design)
        except np.linalg.LinAlgError:
            return None
        gradient = np.array([intercept / slope ** 2, -1.0 / slope])
        sigma = float(np.sqrt(max(gradient @ covariance @ gradient, 0.0)))
        if not math.isfinite(sigma) or sigma > c.max_range_sigma_ratio * remaining:
            return None
        return RangeEstimate(initial, float(remaining), residual, sigma, len(x))


def overlap(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1]))
    area = lambda box: (box[2] - box[0]) * (box[3] - box[1])
    return intersection / (area(a) + area(b) - intersection)


class ApproachController:
    """Briefly interrupt exploration to obtain one metric object location.

    The node owns command arbitration. STOPPING waits for the explorer's
    terminal action result AND measured stationarity before allowing a turn.
    APPROACHING emits zero angular velocity; drift aborts instead of curving.
    MEASURED freezes the successful observation until the node saves it.
    RESUMING remains stopped until the explorer acknowledges release.
    """

    def __init__(self, config=None):
        self.config = config or Config()
        self.state = 'EXPLORING'
        self.reason = 'waiting for a target'
        self.target = None
        self.last_stamp = -1.0
        self.last_seen = None
        self.entered = 0.0
        self.still_since = None
        self.aligned = 0
        self.origin = None
        self.estimator = ExpansionEstimator(self.config)
        self.range = None
        self.travel = 0.0
        self.retryable = False
        self.completed = set()
        self.measured_from = {}
        self.deferred = {}
        self.progress_at = 0.0
        self.progress_travel = 0.0

    @property
    def holding(self):
        return self.state != 'EXPLORING'

    def fail(self, reason, retryable=False):
        self.state, self.reason = 'STOPPED', reason
        self.retryable = retryable

    def eligible(self, landmark, now, pose):
        previous = self.deferred.get(landmark)
        retry_ready = previous is None or (
            now >= previous[0] and pose is not None
            and math.hypot(pose.x - previous[1].x, pose.y - previous[1].y)
            >= self.config.retry_viewpoint_m)
        measured = self.measured_from.get(landmark)
        revisit_ready = measured is None or (
            now >= measured[0] and pose is not None
            and math.hypot(pose.x - measured[1].x, pose.y - measured[1].y)
            >= self.config.remeasure_viewpoint_m)
        return retry_ready and revisit_ready

    def begin_resume(self, now, pose, saved=False):
        """The caller first stops the rover and, on success, persists the point."""
        if self.state not in ('MEASURED', 'STOPPED'):
            raise ValueError('can only resume after measurement or an aborted probe')
        if self.state == 'MEASURED' and not saved:
            raise ValueError('save the measured target before resuming')
        if self.state == 'STOPPED' and not self.retryable:
            raise ValueError('this fault requires operator intervention')
        if self.target is not None:
            if saved:
                self.completed.add(self.target.landmark)
                self.measured_from[self.target.landmark] = (
                    now + self.config.remeasure_cooldown, self.target.pose)
                self.deferred.pop(self.target.landmark, None)
            else:
                self.deferred[self.target.landmark] = (
                    now + self.config.retry_cooldown, pose or self.target.pose)
        reason = 'target saved; resuming exploration' if saved else (
            'measurement deferred; resuming exploration: ' + self.reason)
        self._transition('RESUMING', now, reason)

    def finish_resume(self):
        if self.state != 'RESUMING':
            raise ValueError('no resume handshake in progress')
        self.target = self.origin = self.range = None
        self.last_seen = self.still_since = None
        self.aligned = 0
        self.travel = 0.0
        self.retryable = False
        self.estimator = ExpansionEstimator(self.config)
        self.state, self.reason = 'EXPLORING', 'searching for targets and new measurement viewpoints'

    def _transition(self, state, now, reason):
        self.state, self.entered, self.reason = state, now, reason

    def observe(self, observations, stamp, now):
        c = self.config
        if stamp <= self.last_stamp or not 0 <= now - stamp <= c.detection_timeout:
            return
        self.last_stamp = stamp
        if self.state in ('STOPPED', 'MEASURED', 'RESUMING'):
            return
        usable = [o for o in observations if (
            o.confidence >= c.min_confidence
            and all(math.isfinite(v) for v in (*o.box, o.bearing, o.size,
                                               o.pose.x, o.pose.y, o.pose.yaw))
            and 0.01 < o.box[0] < o.box[2] < 0.99
            and 0.01 < o.box[1] < o.box[3] < 0.99 and o.size > 0)]
        if self.target is None:
            usable = [o for o in usable if self.eligible(o.landmark, now, o.pose)]
            if not usable:
                return
            # Find the second landmark before spending another probe on one
            # already recorded, when both are visible at the same time.
            self.target = max(usable, key=lambda o: (
                o.landmark not in self.completed, o.confidence))
            self.last_seen = stamp
            self._transition('STOPPING', now, 'target acquired; cancelling exploration')
            return
        matched = [o for o in usable if (
            o.landmark == self.target.landmark
            and o.model_class == self.target.model_class
            and overlap(o.box, self.target.box) >= c.association_iou)]
        if len(matched) != 1:
            self.fail('target lost, clipped, or ambiguous', retryable=True)
            return
        previous_target = self.target
        self.target = matched[0]
        self.last_seen = stamp
        if self.state == 'ALIGNING':
            if abs(self.target.bearing) <= c.alignment_tolerance:
                self.aligned += 1
            else:
                self.aligned = 0
        elif self.state == 'APPROACHING':
            def aspect(box):
                return (box[2] - box[0]) / (box[3] - box[1])
            if not 0.9 <= aspect(self.target.box) / aspect(previous_target.box) <= 1.1:
                self.fail('target shape changed', retryable=True)
                return
            travel, lateral, heading = self._displacement(self.target.pose)
            if (abs(lateral) > c.max_lateral_m or abs(heading) > c.max_heading_error
                    or abs(self.target.bearing) > c.max_bearing_error or travel < -0.01):
                self.fail('straight approach geometry changed', retryable=True)
                return
            previous = self.estimator.samples[-1][1]
            # Reject abrupt scale/shape changes (occlusion or another instance).
            if not 0.85 <= self.target.size / previous <= 1.20:
                self.fail('target size jumped', retryable=True)
                return
            self.estimator.add(travel, self.target.size)
            self.travel = travel
            if self.estimator.enough_growth() or travel >= c.max_probe_m:
                self._finish_probe(now)

    def _finish_probe(self, now):
        """End of the leg: fit once and keep the answer only if it is precise.

        Called from the frame path when the growth target is reached and from
        the timer when the probe runs out of travel or time. Either way the
        fit sees the whole leg, so a probe that ended early still reports the
        precision it actually earned rather than being failed outright.
        """
        self.range = self.estimator.estimate()
        if self.range is not None:
            self._transition('MEASURED', now, 'distance obtained; saving map position')
        elif self.estimator.growth() < self.config.min_growth:
            # Worth separating: nothing grew (too far, or the probe was
            # blocked) reads differently from grew but too noisily to trust.
            self.fail('probe limit: insufficient size growth for a reliable range',
                      retryable=True)
        else:
            self.fail('probe range too imprecise at this baseline', retryable=True)

    def _displacement(self, pose):
        dx, dy = pose.x - self.origin.x, pose.y - self.origin.y
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        return (c * dx + s * dy, -s * dx + c * dy,
                angle_delta(pose.yaw, self.origin.yaw))

    def command(self, now, pose, stationary, explorer_paused, sensors_fresh=True):
        c = self.config
        if not self.holding or self.state in ('STOPPED', 'MEASURED', 'RESUMING'):
            return 0.0, 0.0
        if not sensors_fresh or pose is None:
            self.fail('odometry or obstacle scan unavailable', retryable=True)
            return 0.0, 0.0
        if self.last_seen is None or now - self.last_seen > c.detection_timeout:
            self.fail('camera/detector/network timeout', retryable=True)
            return 0.0, 0.0
        if self.state == 'STOPPING':
            if now - self.entered > c.stop_timeout:
                self.fail('explorer cancellation or stopping timed out')
            elif explorer_paused and stationary:
                if self.still_since is None:
                    self.still_since = now
                elif now - self.still_since >= c.settle_time:
                    self._transition('ALIGNING', now, 'centering target')
            else:
                self.still_since = None
            return 0.0, 0.0
        if not explorer_paused:
            self.fail('explorer pause acknowledgement lost')
            return 0.0, 0.0
        if self.state == 'ALIGNING':
            if now - self.entered > c.alignment_timeout:
                self.fail('alignment timed out', retryable=True)
                return 0.0, 0.0
            error = self.target.bearing
            if abs(error) <= c.alignment_tolerance:
                if self.aligned >= c.alignment_frames and stationary:
                    # Start the baseline AFTER rotation has stopped. Never fit
                    # scale changes caused by the initial turn.
                    self.origin = self.target.pose
                    self.estimator.add(0.0, self.target.size)
                    self.progress_at, self.progress_travel = now, 0.0
                    self._transition('APPROACHING', now, 'measuring expansion during straight travel')
                return 0.0, 0.0
            return 0.0, max(-c.max_angular_speed, min(c.max_angular_speed,
                                                   c.angular_gain * error))
        self.travel, lateral, heading = self._displacement(pose)
        if self.travel - self.progress_travel >= 0.005:
            self.progress_at, self.progress_travel = now, self.travel
        if (abs(lateral) > c.max_lateral_m or abs(heading) > c.max_heading_error
                or self.travel < -0.01):
            self.fail('rover deviated from the straight approach', retryable=True)
        elif now - self.progress_at >= c.progress_timeout:
            self.fail('probe blocked: no odometry progress', retryable=True)
        elif self.travel >= c.max_probe_m or now - self.entered > c.approach_timeout:
            self._finish_probe(now)
        if self.state != 'APPROACHING':
            return 0.0, 0.0
        return c.approach_speed, 0.0
