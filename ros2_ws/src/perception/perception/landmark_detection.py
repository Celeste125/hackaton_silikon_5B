"""Framework-free pieces of the landmark detection interface.

Nothing in this module imports ROS, torch or onnxruntime, so the wire format,
the class mapping and the pixel geometry can be tested on a laptop -- and on
the Jetson before any model exists.

The split this file defines is what makes the detector swappable:

  a backend   turns an image into boxes. It knows nothing about ROS, TF or the
              map, and lives in landmark_backends.py.
  a spec      says which of the model's class names count as which landmark,
              and how much to trust a box of that landmark (an enclosure's
              centroid wanders between views far more than a dispenser's).
  a payload   is the only thing that leaves the machine holding the camera:
              boxes, intrinsics and an optional range, as JSON. A few hundred
              bytes a frame instead of a VGA image, which matters because the
              arena WiFi is congested.

The payload carries resolved intrinsics and, where the camera has depth, the
range already sampled from the depth image. That is deliberate: the consumer
then needs no camera topics at all, so the images never cross the network and
the fusion node can sit on whichever machine owns the TF tree.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

SCHEMA_VERSION = 1

# A box centre is only as good as the model's idea of the object's middle.
# 3 deg is about right for a compact object with a tight box; large or
# partially-framed things deserve more, set per landmark in landmarks.yaml.
DEFAULT_BEARING_SIGMA_DEG = 3.0
DEFAULT_RANGE_SIGMA_M = 0.10


def canonical_class(name: str) -> str:
    """Fold the spellings a training set might use into one key.

    A teammate labelling in CVAT may write 'Food Dispenser', 'food-dispenser'
    or 'food_dispenser'; none of them should silently fail to match.
    """
    return '_'.join(str(name).strip().lower().replace('-', ' ').split())


@dataclass(frozen=True)
class LandmarkSpec:
    """One thing worth putting on the map, and how much to trust seeing it."""

    name: str
    model_classes: tuple[str, ...]
    min_confidence: float = 0.40
    bearing_sigma_rad: float = math.radians(DEFAULT_BEARING_SIGMA_DEG)
    range_sigma_m: float = DEFAULT_RANGE_SIGMA_M
    color: tuple[float, float, float] = (0.1, 0.9, 0.2)
    marker_scale_m: float = 0.20
    note: str = ''

    def accepts(self, confidence: float) -> bool:
        return float(confidence) >= self.min_confidence


def load_landmark_specs(document: Mapping[str, Any]) -> dict[str, LandmarkSpec]:
    """Build specs from a parsed landmarks.yaml.

    Raises rather than defaulting on a malformed entry: a typo'd class name
    means the model's detections are silently dropped for the whole run, and
    that is the kind of failure nobody notices until the arena.
    """
    landmarks = document.get('landmarks') if document else None
    if not isinstance(landmarks, Mapping) or not landmarks:
        raise ValueError("landmarks.yaml needs a non-empty 'landmarks' mapping")

    specs: dict[str, LandmarkSpec] = {}
    for name, entry in landmarks.items():
        if not isinstance(entry, Mapping):
            raise ValueError(f'landmark {name!r} must be a mapping')
        classes = entry.get('model_classes') or [name]
        if isinstance(classes, str):
            classes = [classes]
        canonical = tuple(dict.fromkeys(canonical_class(c) for c in classes))
        if not canonical or not all(canonical):
            raise ValueError(f'landmark {name!r} has an empty model class')

        confidence = float(entry.get('min_confidence', 0.40))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f'landmark {name!r}: min_confidence out of [0, 1]')

        bearing_deg = float(
            entry.get('bearing_sigma_deg', DEFAULT_BEARING_SIGMA_DEG))
        if bearing_deg <= 0.0:
            raise ValueError(f'landmark {name!r}: bearing_sigma_deg must be > 0')

        range_sigma = float(entry.get('range_sigma_m', DEFAULT_RANGE_SIGMA_M))
        if range_sigma <= 0.0:
            raise ValueError(f'landmark {name!r}: range_sigma_m must be > 0')

        color = tuple(float(c) for c in entry.get('color', (0.1, 0.9, 0.2)))
        if len(color) != 3:
            raise ValueError(f'landmark {name!r}: color must be [r, g, b]')

        specs[str(name)] = LandmarkSpec(
            name=str(name),
            model_classes=canonical,
            min_confidence=confidence,
            bearing_sigma_rad=math.radians(bearing_deg),
            range_sigma_m=range_sigma,
            color=color,  # type: ignore[arg-type]
            marker_scale_m=float(entry.get('marker_scale_m', 0.20)),
            note=str(entry.get('note', '')),
        )

    duplicates = _duplicate_classes(specs.values())
    if duplicates:
        raise ValueError(
            f'model class(es) {sorted(duplicates)} claimed by more than one '
            'landmark; one class must map to one landmark')
    return specs


def _duplicate_classes(specs: Iterable[LandmarkSpec]) -> set[str]:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for spec in specs:
        for name in spec.model_classes:
            if name in seen:
                duplicates.add(name)
            seen.add(name)
    return duplicates


def class_lookup(specs: Mapping[str, LandmarkSpec]) -> dict[str, LandmarkSpec]:
    """Reverse index: canonical model class name -> the landmark it feeds."""
    return {
        class_name: spec
        for spec in specs.values()
        for class_name in spec.model_classes
    }


# --------------------------------------------------------------- intrinsics


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    source: str

    def bearing(self, u: float, v: float) -> tuple[float, float, float]:
        """Unit sight line through a pixel, in the optical frame.

        Optical convention, as REP 103 has it and as the camera's frame_id
        advertises: x right, y down, z forward.
        """
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        ray /= float(np.linalg.norm(ray))
        return float(ray[0]), float(ray[1]), float(ray[2])


def resolve_intrinsics(
    width: int,
    height: int,
    k: Sequence[float] | None,
    fallback_hfov_deg: float,
) -> Intrinsics:
    """Prefer a calibration, fall back to the declared field of view.

    v4l2_camera publishes a zeroed K until it is given a calibration file, and
    a zero focal length would send every bearing straight down the optical
    axis -- every landmark would land on the robot's nose. The FOV fallback is
    good to a couple of degrees, which is several centimetres of cross-range
    error at 2 m, so it is a usable default and a poor substitute for
    camera_calibration.
    """
    if k is not None and len(k) >= 6 and float(k[0]) > 0.0:
        return Intrinsics(
            fx=float(k[0]), fy=float(k[4]),
            cx=float(k[2]), cy=float(k[5]), source='camera_info')
    hfov = math.radians(float(fallback_hfov_deg))
    if not 0.0 < hfov < math.pi:
        raise ValueError('fallback_hfov_deg must be within (0, 180)')
    focal = (float(width) / 2.0) / math.tan(hfov / 2.0)
    return Intrinsics(
        fx=focal, fy=focal,
        cx=float(width) / 2.0, cy=float(height) / 2.0, source='fov_fallback')


# ---------------------------------------------------------------- detections


@dataclass(frozen=True)
class LandmarkDetection:
    """One box, already attributed to a landmark.

    box_norm is [x0, y0, x1, y1] in 0-1 with the origin top-left, which keeps
    it independent of whatever resolution the detector happened to run at.
    """

    landmark: str
    model_class: str
    confidence: float
    box_norm: tuple[float, float, float, float]
    range_m: float | None = None
    range_sigma_m: float | None = None
    range_source: str = 'none'

    def centre_px(self, width: int, height: int) -> tuple[float, float]:
        x0, y0, x1, y1 = self.box_norm
        return ((x0 + x1) / 2.0 * width, (y0 + y1) / 2.0 * height)

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            'landmark': self.landmark,
            'model_class': self.model_class,
            'confidence': round(float(self.confidence), 4),
            'box_norm': [round(float(v), 5) for v in self.box_norm],
            'range_source': self.range_source,
        }
        if self.range_m is not None:
            payload['range_m'] = round(float(self.range_m), 4)
            payload['range_sigma_m'] = round(
                float(self.range_sigma_m or DEFAULT_RANGE_SIGMA_M), 4)
        return payload


def normalise_box(
    box_xyxy_px: Sequence[float],
    width: int,
    height: int,
) -> tuple[float, float, float, float]:
    """Clamp a pixel box into a normalised one, or refuse it.

    A degenerate box is rejected rather than nudged to a minimum size: a
    zero-area box means the backend returned something the geometry cannot
    use, and a silently widened box would produce a confident wrong bearing.
    """
    if len(box_xyxy_px) != 4:
        raise ValueError('box must contain exactly four coordinates')
    values = [float(v) for v in box_xyxy_px]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('box coordinates must be finite')
    x0, x1 = sorted((values[0], values[2]))
    y0, y1 = sorted((values[1], values[3]))
    x0 = min(max(x0 / width, 0.0), 1.0)
    x1 = min(max(x1 / width, 0.0), 1.0)
    y0 = min(max(y0 / height, 0.0), 1.0)
    y1 = min(max(y1 / height, 0.0), 1.0)
    if x1 - x0 <= 0.0 or y1 - y0 <= 0.0:
        raise ValueError('box has zero area inside the image')
    return (x0, y0, x1, y1)


def touches_image_edge(
    box_norm: Sequence[float],
    margin: float = 0.01,
) -> bool:
    """True when the box runs off the frame, so the object is cut off.

    A clipped box's centroid is not the object's centroid -- it is pulled
    toward the middle of the image by however much is missing -- so the
    bearing is biased. Worth knowing about rather than fusing blind.
    """
    x0, y0, x1, y1 = (float(v) for v in box_norm)
    return (
        x0 <= margin or y0 <= margin
        or x1 >= 1.0 - margin or y1 >= 1.0 - margin
    )


# --------------------------------------------------------------------- depth


def depth_in_box(
    depth_m: np.ndarray,
    box_norm: Sequence[float],
    shrink: float = 0.5,
    min_depth_m: float = 0.15,
    max_depth_m: float = 4.0,
    min_pixels: int = 20,
) -> tuple[float | None, float, int]:
    """Robust range to the object inside a box, from a metric depth image.

    Returns (range_m, sigma_m, pixel_count). The median over the middle of the
    box, not the whole box: the box edges straddle whatever is behind the
    object, and a mean there would sit somewhere between the object and the
    far wall. Sigma comes from the MAD of the same pixels, so a patch that
    spans a depth discontinuity reports itself as uncertain instead of
    confidently splitting the difference.
    """
    if depth_m.ndim != 2:
        raise ValueError('depth image must be 2-D')
    if not 0.0 < shrink <= 1.0:
        raise ValueError('shrink must be within (0, 1]')
    height, width = depth_m.shape

    x0, y0, x1, y1 = (float(v) for v in box_norm)
    centre_x, centre_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    half_w = (x1 - x0) * shrink / 2.0
    half_h = (y1 - y0) * shrink / 2.0

    u0 = max(int((centre_x - half_w) * width), 0)
    u1 = min(max(int((centre_x + half_w) * width), u0 + 1), width)
    v0 = max(int((centre_y - half_h) * height), 0)
    v1 = min(max(int((centre_y + half_h) * height), v0 + 1), height)

    patch = depth_m[v0:v1, u0:u1]
    valid = patch[
        (patch > min_depth_m) & (patch < max_depth_m) & np.isfinite(patch)]
    if valid.size < int(min_pixels):
        return None, 0.0, int(valid.size)

    median = float(np.median(valid))
    # 1.4826 * MAD is the normal-consistent sigma estimate. Floor it: a flat
    # patch reports MAD 0, and a zero sigma would give that one view infinite
    # weight in the fusion.
    mad = float(np.median(np.abs(valid - median)))
    sigma = max(1.4826 * mad, 0.02)
    return median, sigma, int(valid.size)


def color_image_to_bgr(data: bytes, height: int, width: int,
                       step: int, encoding: str) -> np.ndarray:
    """Decode a ROS colour image into the BGR array the backends expect.

    Done by hand rather than through cv_bridge: the detector runs on the
    Jetson, in its own workspace, and a missing cv_bridge there would take the
    whole node down for the sake of a reshape. The row stride is honoured
    because a camera driver is free to pad rows and some do.
    """
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported colour encoding {encoding!r}')
    array = np.frombuffer(data, dtype=np.uint8)
    row_pixels = max(step // channels, width)
    array = array.reshape(height, row_pixels, channels)[:, :width, :]
    if encoding == 'rgb8':
        array = array[:, :, ::-1]
    elif encoding == 'mono8':
        array = np.repeat(array, 3, axis=2)
    return np.ascontiguousarray(array)


def depth_image_to_metres(data: bytes, height: int, width: int,
                          step: int, encoding: str) -> np.ndarray:
    """Decode a ROS depth image into a 2-D array of metres.

    Kept here, away from ROS, so the millimetre/metre distinction is unit
    tested. Getting it wrong puts every landmark a thousand times too far
    away, which looks like a TF problem and is not one.
    """
    if encoding == '16UC1':
        raw = np.frombuffer(data, dtype=np.uint16).astype(np.float32) * 0.001
        bytes_per_pixel = 2
    elif encoding == '32FC1':
        raw = np.frombuffer(data, dtype=np.float32).copy()
        bytes_per_pixel = 4
    else:
        raise ValueError(f'unsupported depth encoding {encoding!r}')
    row_pixels = max(step // bytes_per_pixel, width)
    return raw.reshape(height, row_pixels)[:, :width]


# ------------------------------------------------------------------ payloads


@dataclass(frozen=True)
class DetectionFrame:
    """A parsed payload: everything the fusion needs, and nothing more."""

    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    width: int
    height: int
    intrinsics: Intrinsics
    detections: tuple[LandmarkDetection, ...]
    inference_ms: float = 0.0
    model: Mapping[str, Any] = field(default_factory=dict)


def build_payload(
    stamp_sec: int,
    stamp_nanosec: int,
    frame_id: str,
    width: int,
    height: int,
    intrinsics: Intrinsics,
    detections: Sequence[LandmarkDetection],
    inference_ms: float = 0.0,
    model: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """The JSON that crosses the network, in place of the image."""
    return {
        'schema_version': SCHEMA_VERSION,
        'stamp': {'sec': int(stamp_sec), 'nanosec': int(stamp_nanosec)},
        'frame_id': str(frame_id),
        'image_size': {'width': int(width), 'height': int(height)},
        'intrinsics': {
            'fx': round(intrinsics.fx, 4),
            'fy': round(intrinsics.fy, 4),
            'cx': round(intrinsics.cx, 4),
            'cy': round(intrinsics.cy, 4),
            'source': intrinsics.source,
        },
        'inference_ms': round(float(inference_ms), 2),
        'model': dict(model or {}),
        'detections': [
            detection.as_dict()
            for detection in sorted(
                detections, key=lambda d: d.confidence, reverse=True)
        ],
    }


def parse_payload(text: str) -> DetectionFrame:
    """Validate a payload into a DetectionFrame, or raise.

    The fusion node runs on the other side of a WiFi link from the detector,
    possibly against a detector someone is mid-way through upgrading, so the
    wire format is checked rather than trusted.
    """
    document = json.loads(text)
    version = int(document.get('schema_version', 0))
    if version != SCHEMA_VERSION:
        raise ValueError(
            f'detection schema {version}, expected {SCHEMA_VERSION}')

    frame_id = str(document.get('frame_id', ''))
    if not frame_id:
        raise ValueError('payload has no frame_id; a bearing needs a frame')

    size = document.get('image_size') or {}
    width, height = int(size.get('width', 0)), int(size.get('height', 0))
    if width <= 0 or height <= 0:
        raise ValueError('payload has no image size')

    raw = document.get('intrinsics') or {}
    if float(raw.get('fx', 0.0)) <= 0.0 or float(raw.get('fy', 0.0)) <= 0.0:
        raise ValueError('payload intrinsics have a non-positive focal length')
    intrinsics = Intrinsics(
        fx=float(raw['fx']), fy=float(raw['fy']),
        cx=float(raw.get('cx', width / 2.0)),
        cy=float(raw.get('cy', height / 2.0)),
        source=str(raw.get('source', 'unknown')))

    stamp = document.get('stamp') or {}
    detections = tuple(
        _parse_detection(entry) for entry in document.get('detections', []))

    return DetectionFrame(
        stamp_sec=int(stamp.get('sec', 0)),
        stamp_nanosec=int(stamp.get('nanosec', 0)),
        frame_id=frame_id,
        width=width,
        height=height,
        intrinsics=intrinsics,
        detections=detections,
        inference_ms=float(document.get('inference_ms', 0.0)),
        model=document.get('model') or {},
    )


def _parse_detection(entry: Mapping[str, Any]) -> LandmarkDetection:
    landmark = str(entry.get('landmark', ''))
    if not landmark:
        raise ValueError('detection has no landmark name')
    box = entry.get('box_norm')
    if not isinstance(box, Sequence) or len(box) != 4:
        raise ValueError(f'{landmark}: box_norm must hold four numbers')
    values = tuple(float(v) for v in box)
    if values[2] <= values[0] or values[3] <= values[1]:
        raise ValueError(f'{landmark}: box_norm has zero area')
    confidence = float(entry.get('confidence', 0.0))
    if not 0.0 <= confidence <= 1.0:
        raise ValueError(f'{landmark}: confidence outside [0, 1]')

    range_m = entry.get('range_m')
    return LandmarkDetection(
        landmark=landmark,
        model_class=str(entry.get('model_class', landmark)),
        confidence=confidence,
        box_norm=values,  # type: ignore[arg-type]
        range_m=None if range_m is None else float(range_m),
        range_sigma_m=(
            None if range_m is None
            else float(entry.get('range_sigma_m', DEFAULT_RANGE_SIGMA_M))),
        range_source=str(entry.get('range_source', 'none')),
    )
