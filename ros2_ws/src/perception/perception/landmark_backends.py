"""Swappable detector backends: the one place a trained model plugs in.

A backend is anything that turns one image into boxes. It is handed a BGR
uint8 array (OpenCV's convention, which is also what Ultralytics expects from
a numpy source) and returns RawDetection list; it knows nothing about ROS, TF,
the map or the landmark names.

Five are built in:

  disabled     No weights, no detections. Keeps the live camera heartbeat
               running while bringing up exploration before a model arrives.

  ultralytics  A model trained with Ultralytics, either a ``.pt`` or a
               TensorRT ``.engine``. The normal path once the model exists;
               detection and segmentation weights both work, because a
               segmentation head still produces boxes.
  onnx         The same model exported to ONNX, run through onnxruntime. For a
               Jetson where getting a JetPack-matched torch plus Ultralytics
               installed is more trouble than it is worth.
  stub         Fixed boxes from the launch file. No model at all, so the whole
               bearing/TF/fusion chain can be driven and checked today.
  claude       Asks Claude for the boxes over the network. Needs an API key
               and sends frames off the robot, so it is the stand-in to use
               only until the trained model lands.

Adding a fifth is one function and one ``register_backend`` call; nothing
upstream of ``create_backend`` needs to change.

Heavy imports are deliberately inside the backends. Importing this module on a
machine with no torch, no onnxruntime and no anthropic must work, because the
pure tests and the stub path do exactly that.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class RawDetection:
    """One box as the model produced it: pixels, and the model's own label."""

    model_class: str
    confidence: float
    box_xyxy_px: tuple[float, float, float, float]


class DetectorBackend:
    """Interface every backend implements.

    ``load`` is separate from ``__init__`` so a bad model path or a missing
    runtime is reported by the node with a usable message, at a point where it
    can decide to keep running (reporting the camera) rather than die.
    """

    name = 'base'

    def load(self) -> None:
        raise NotImplementedError

    def detect(self, image_bgr: np.ndarray) -> list[RawDetection]:
        raise NotImplementedError

    def describe(self) -> dict[str, Any]:
        """What went into the payload's ``model`` field, for the record."""
        return {'backend': self.name}

    @property
    def class_names(self) -> tuple[str, ...]:
        """Labels this backend can emit, for a startup sanity check."""
        return ()


_REGISTRY: dict[str, Callable[[Mapping[str, Any]], DetectorBackend]] = {}


def register_backend(
    name: str,
    factory: Callable[[Mapping[str, Any]], DetectorBackend],
) -> None:
    _REGISTRY[name] = factory


def available_backends() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))


def create_backend(name: str, options: Mapping[str, Any]) -> DetectorBackend:
    try:
        factory = _REGISTRY[name]
    except KeyError:
        raise ValueError(
            f'unknown backend {name!r}; have {", ".join(available_backends())}'
        ) from None
    return factory(options)


# ------------------------------------------------------------- ultralytics


class DisabledBackend(DetectorBackend):
    """Camera/transport heartbeat with no weights and no invented targets."""

    name = 'disabled'

    def __init__(self, options):
        pass

    def load(self):
        pass

    def detect(self, image_bgr):
        return []


register_backend('disabled', DisabledBackend)


class UltralyticsBackend(DetectorBackend):
    """A ``.pt`` or TensorRT ``.engine`` trained with Ultralytics."""

    name = 'ultralytics'

    def __init__(self, options: Mapping[str, Any]):
        self._path = Path(str(options['model_path'])).expanduser()
        self._confidence = float(options.get('confidence_threshold', 0.25))
        self._iou = float(options.get('iou_threshold', 0.60))
        self._image_size = int(options.get('image_size', 640))
        self._device = options.get('device', 0)
        self._task = str(options.get('task', 'detect'))
        self._half = bool(options.get('half', False))
        self._model = None
        self._names: dict[int, str] = {}

    def load(self) -> None:
        if not self._path.is_file():
            raise FileNotFoundError(f'no model at {self._path}')
        from ultralytics import YOLO

        # A TensorRT engine carries no task metadata, so it has to be told;
        # a .pt knows its own and Ultralytics warns if we override it wrongly.
        if self._path.suffix == '.engine':
            self._model = YOLO(str(self._path), task=self._task)
        else:
            self._model = YOLO(str(self._path))
        names = getattr(self._model, 'names', {}) or {}
        self._names = (
            {index: str(name) for index, name in enumerate(names)}
            if isinstance(names, list)
            else {int(k): str(v) for k, v in names.items()}
        )
        if not self._names:
            raise RuntimeError(f'{self._path} exposes no class names')

    def detect(self, image_bgr: np.ndarray) -> list[RawDetection]:
        results = self._model.predict(  # type: ignore[union-attr]
            image_bgr,
            conf=self._confidence,
            iou=self._iou,
            imgsz=self._image_size,
            device=self._device,
            half=self._half,
            verbose=False,
        )
        detections: list[RawDetection] = []
        for result in results:
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                continue
            names = (
                {int(k): str(v) for k, v in result.names.items()}
                if isinstance(result.names, dict) else self._names)
            for xyxy, confidence, class_id in zip(
                boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()
            ):
                label = names.get(int(class_id), str(int(class_id)))
                detections.append(RawDetection(
                    model_class=label,
                    confidence=float(confidence),
                    box_xyxy_px=tuple(float(v) for v in xyxy),  # type: ignore
                ))
        return detections

    def describe(self) -> dict[str, Any]:
        return {
            'backend': self.name,
            'path': str(self._path),
            'task': self._task,
            'device': str(self._device),
            'image_size': self._image_size,
            'classes': list(self._names.values()),
        }

    @property
    def class_names(self) -> tuple[str, ...]:
        return tuple(self._names.values())


register_backend('ultralytics', UltralyticsBackend)


# --------------------------------------------------------------------- onnx


class OnnxBackend(DetectorBackend):
    """An Ultralytics ONNX export, run through onnxruntime.

    The decode assumes the standard Ultralytics export layout: one output of
    shape (1, 4 + nc [+ 32 mask coefficients], anchors), boxes as xywh in the
    letterboxed input's pixels. The mask coefficients of a segmentation export
    sit after the class scores, so knowing nc is enough to ignore them --
    which is why the class names are required rather than inferred.
    """

    name = 'onnx'

    def __init__(self, options: Mapping[str, Any]):
        self._path = Path(str(options['model_path'])).expanduser()
        self._confidence = float(options.get('confidence_threshold', 0.25))
        self._iou = float(options.get('iou_threshold', 0.60))
        self._image_size = int(options.get('image_size', 640))
        self._providers = list(options.get('onnx_providers') or ())
        self._names = tuple(
            str(name) for name in (options.get('class_names') or ()))
        self._names_path = options.get('class_names_path') or ''
        self._session = None

    def load(self) -> None:
        if not self._path.is_file():
            raise FileNotFoundError(f'no model at {self._path}')
        if not self._names:
            self._names = _class_names_from_file(self._names_path, self._path)
        if not self._names:
            raise RuntimeError(
                'the onnx backend needs class_names (or class_names_path): an '
                'ONNX graph does not carry its labels, and guessing them '
                'would silently mislabel every detection')

        import onnxruntime  # noqa: PLC0415 - optional dependency

        providers = self._providers or onnxruntime.get_available_providers()
        self._session = onnxruntime.InferenceSession(
            str(self._path), providers=providers)
        self._input_name = self._session.get_inputs()[0].name

    def detect(self, image_bgr: np.ndarray) -> list[RawDetection]:
        tensor, scale, pad = _letterbox(image_bgr, self._image_size)
        outputs = self._session.run(None, {self._input_name: tensor})  # type: ignore[union-attr]
        predictions = np.asarray(outputs[0], dtype=np.float32)
        if predictions.ndim == 3:
            predictions = predictions[0]
        # (4 + nc + extras, anchors) -> (anchors, 4 + nc + extras)
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.transpose()

        class_count = len(self._names)
        scores = predictions[:, 4:4 + class_count]
        if scores.size == 0:
            return []
        best = scores.argmax(axis=1)
        confidence = scores[np.arange(scores.shape[0]), best]
        keep = confidence >= self._confidence
        if not np.any(keep):
            return []

        boxes = _xywh_to_xyxy(predictions[keep, :4])
        boxes = _undo_letterbox(boxes, scale, pad, image_bgr.shape[:2])
        confidence = confidence[keep]
        classes = best[keep]

        detections: list[RawDetection] = []
        for class_id in np.unique(classes):
            selection = classes == class_id
            for index in _nms(boxes[selection], confidence[selection], self._iou):
                detections.append(RawDetection(
                    model_class=self._names[int(class_id)],
                    confidence=float(confidence[selection][index]),
                    box_xyxy_px=tuple(
                        float(v) for v in boxes[selection][index]),  # type: ignore
                ))
        return detections

    def describe(self) -> dict[str, Any]:
        return {
            'backend': self.name,
            'path': str(self._path),
            'image_size': self._image_size,
            'classes': list(self._names),
        }

    @property
    def class_names(self) -> tuple[str, ...]:
        return self._names


register_backend('onnx', OnnxBackend)


def _class_names_from_file(configured: str, model_path: Path) -> tuple[str, ...]:
    """Read labels from a sidecar, in the orders Ultralytics writes them.

    An Ultralytics export leaves the dataset YAML beside the weights often
    enough to be worth looking for; a plain JSON list is accepted too.
    """
    candidates = [Path(configured).expanduser()] if configured else []
    candidates += [model_path.with_suffix('.yaml'), model_path.with_suffix('.json')]
    for path in candidates:
        if not path.is_file():
            continue
        text = path.read_text(encoding='utf-8')
        if path.suffix == '.json':
            document = json.loads(text)
        else:
            import yaml  # noqa: PLC0415 - only needed on this path

            document = yaml.safe_load(text)
        names = document.get('names', document) if isinstance(
            document, Mapping) else document
        if isinstance(names, Mapping):
            return tuple(str(names[key]) for key in sorted(names, key=int))
        if isinstance(names, Sequence) and not isinstance(names, str):
            return tuple(str(name) for name in names)
    return ()


def _letterbox(image_bgr: np.ndarray, size: int):
    """Resize keeping aspect ratio, pad to square, return an NCHW RGB tensor."""
    import cv2  # noqa: PLC0415 - present wherever a camera is

    height, width = image_bgr.shape[:2]
    scale = min(size / width, size / height)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    tensor = canvas[:, :, ::-1].astype(np.float32) / 255.0  # BGR -> RGB
    return np.ascontiguousarray(tensor.transpose(2, 0, 1)[None]), scale, (pad_x, pad_y)


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    centre = boxes[:, :2]
    half = boxes[:, 2:4] / 2.0
    return np.concatenate([centre - half, centre + half], axis=1)


def _undo_letterbox(boxes: np.ndarray, scale: float, pad, shape) -> np.ndarray:
    height, width = shape
    boxes = boxes.copy()
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad[0]) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad[1]) / scale
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, width)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, height)
    return boxes


def _nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> list[int]:
    """Plain greedy IoU suppression, in numpy to avoid a torch dependency."""
    order = scores.argsort()[::-1]
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    keep: list[int] = []
    while order.size:
        current = int(order[0])
        keep.append(current)
        if order.size == 1:
            break
        rest = order[1:]
        x0 = np.maximum(boxes[current, 0], boxes[rest, 0])
        y0 = np.maximum(boxes[current, 1], boxes[rest, 1])
        x1 = np.minimum(boxes[current, 2], boxes[rest, 2])
        y1 = np.minimum(boxes[current, 3], boxes[rest, 3])
        overlap = np.maximum(x1 - x0, 0.0) * np.maximum(y1 - y0, 0.0)
        union = areas[current] + areas[rest] - overlap
        iou = np.where(union > 0.0, overlap / union, 0.0)
        order = rest[iou <= threshold]
    return keep


# --------------------------------------------------------------------- stub


class StubBackend(DetectorBackend):
    """Fixed boxes, no model.

    This exists so the half of the system that is hard to get right -- the
    extrinsics, the TF timing, the triangulation, the markers -- can be driven
    and verified before anybody has finished training anything. Point it at a
    box where the dispenser actually is in frame and the map should grow a
    landmark in the right place.
    """

    name = 'stub'

    def __init__(self, options: Mapping[str, Any]):
        self._specification = str(options.get('stub_detections', '')).strip()
        self._landmark = str(options.get('stub_landmark', '')).strip()
        self._box_fraction = float(options.get('stub_box_fraction', 0.2))
        self._detections: list[RawDetection] = []

    def load(self) -> None:
        if not self._specification and self._landmark:
            # Point the camera at the real object and it sits in the middle of
            # the frame, so a centred box is the useful default. This path
            # exists because it needs no JSON: the full stub_detections form
            # does not survive the nested ssh/bash quoting that the tmux
            # launcher builds, nor comfortable typing.
            if not 0.0 < self._box_fraction <= 1.0:
                raise ValueError('stub_box_fraction must be within (0, 1]')
            half = self._box_fraction / 2.0
            self._specification = json.dumps([{
                'class': self._landmark,
                'box': [0.5 - half, 0.5 - half, 0.5 + half, 0.5 + half],
            }])
        if not self._specification:
            raise ValueError(
                'the stub backend needs stub_landmark (a centred box), or '
                'stub_detections for full control, e.g. '
                '\'[{"class": "food_dispenser", "box": [0.4,0.4,0.6,0.7]}]\'')
        entries = json.loads(self._specification)
        if isinstance(entries, Mapping):
            entries = [entries]
        for entry in entries:
            box = [float(v) for v in entry['box']]
            if len(box) != 4 or not all(0.0 <= v <= 1.0 for v in box):
                raise ValueError('stub box must be four values in 0-1')
            self._detections.append(RawDetection(
                model_class=str(entry['class']),
                confidence=float(entry.get('confidence', 0.9)),
                # Kept normalised here; scaled to pixels in detect(), because
                # that is where the image size is known.
                box_xyxy_px=tuple(box),  # type: ignore[arg-type]
            ))

    def detect(self, image_bgr: np.ndarray) -> list[RawDetection]:
        height, width = image_bgr.shape[:2]
        return [
            RawDetection(
                model_class=detection.model_class,
                confidence=detection.confidence,
                box_xyxy_px=(
                    detection.box_xyxy_px[0] * width,
                    detection.box_xyxy_px[1] * height,
                    detection.box_xyxy_px[2] * width,
                    detection.box_xyxy_px[3] * height,
                ),
            )
            for detection in self._detections
        ]

    def describe(self) -> dict[str, Any]:
        return {'backend': self.name, 'classes': list(self.class_names)}

    @property
    def class_names(self) -> tuple[str, ...]:
        return tuple(d.model_class for d in self._detections)


register_backend('stub', StubBackend)


# ------------------------------------------------------------------- claude


class ClaudeBackend(DetectorBackend):
    """Boxes from Claude, for use only until the trained model exists.

    Unlike every other backend this one sends the frame off the robot and
    costs a round trip of a second or more, so it cannot keep up with the
    explorer. Run it at a low rate, or not at all once there are weights.
    """

    name = 'claude'

    def __init__(self, options: Mapping[str, Any]):
        self._model = str(options.get('claude_model', 'claude-opus-5-5'))
        self._landmarks = tuple(options.get('landmark_names') or ())
        self._descriptions = dict(options.get('landmark_descriptions') or {})
        self._jpeg_quality = int(options.get('jpeg_quality', 80))
        self._timeout = float(options.get('claude_timeout_sec', 30.0))
        self._client = None

    def load(self) -> None:
        import os

        import anthropic  # noqa: PLC0415 - optional dependency

        if not self._landmarks:
            raise ValueError('the claude backend needs landmark_names')
        if not os.environ.get('ANTHROPIC_API_KEY'):
            raise RuntimeError('ANTHROPIC_API_KEY is not set')
        # The arena WiFi is congested; fail fast rather than stall the loop.
        self._client = anthropic.Anthropic(
            timeout=self._timeout, max_retries=1)

    def detect(self, image_bgr: np.ndarray) -> list[RawDetection]:
        import cv2  # noqa: PLC0415

        import base64

        ok, buffer = cv2.imencode(
            '.jpg', image_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            raise RuntimeError('could not JPEG-encode the frame')

        response = self._client.messages.create(  # type: ignore[union-attr]
            model=self._model,
            max_tokens=1024,
            # Picking a box is not reasoning-heavy and this sits in the
            # robot's loop, so buy latency rather than depth.
            output_config={
                'effort': 'low',
                'format': {'type': 'json_schema', 'schema': self._schema()},
            },
            messages=[{
                'role': 'user',
                'content': [
                    {
                        'type': 'image',
                        'source': {
                            'type': 'base64',
                            'media_type': 'image/jpeg',
                            'data': base64.b64encode(
                                buffer.tobytes()).decode('ascii'),
                        },
                    },
                    {'type': 'text', 'text': self._prompt()},
                ],
            }],
        )
        if response.stop_reason == 'refusal':
            raise RuntimeError('request declined by the safety classifier')
        text = next((b.text for b in response.content if b.type == 'text'), None)
        if not text:
            raise RuntimeError('empty reply')

        height, width = image_bgr.shape[:2]
        detections: list[RawDetection] = []
        for entry in json.loads(text).get('detections', []):
            if not entry.get('visible'):
                continue
            box = [float(v) for v in entry['box']]
            detections.append(RawDetection(
                model_class=str(entry['landmark']),
                confidence=float(entry.get('confidence', 0.0)),
                box_xyxy_px=(
                    box[0] * width, box[1] * height,
                    box[2] * width, box[3] * height),
            ))
        return detections

    def _schema(self) -> dict[str, Any]:
        return {
            'type': 'object',
            'properties': {
                'detections': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'landmark': {
                                'type': 'string',
                                'enum': list(self._landmarks),
                            },
                            'visible': {'type': 'boolean'},
                            'box': {
                                'type': 'array',
                                'description': (
                                    'Normalised [x0, y0, x1, y1] in 0-1, '
                                    'origin top-left.'),
                                'items': {'type': 'number'},
                                'minItems': 4,
                                'maxItems': 4,
                            },
                            'confidence': {'type': 'number'},
                        },
                        'required': ['landmark', 'visible', 'box', 'confidence'],
                        'additionalProperties': False,
                    },
                },
            },
            'required': ['detections'],
            'additionalProperties': False,
        }

    def _prompt(self) -> str:
        wanted = '\n'.join(
            f'- {name}: {self._descriptions.get(name, name.replace("_", " "))}'
            for name in self._landmarks)
        return (
            'This is the forward camera of a small floor robot exploring an '
            'arena. Find any of these, and only these:\n'
            f'{wanted}\n\n'
            'Return a tight normalised box for each one you can see. Report '
            'visible=false, or omit it, when it is not clearly in frame -- '
            'most frames contain none of them, and a guessed box is worse '
            'than no box, because the box is turned into a bearing and '
            'averaged into the map. Prefer the object body over its shadow '
            'or its reflection.'
        )

    def describe(self) -> dict[str, Any]:
        return {'backend': self.name, 'model': self._model}

    @property
    def class_names(self) -> tuple[str, ...]:
        return self._landmarks


register_backend('claude', ClaudeBackend)
