"""Pure helpers for the YOLO segmentation ROS interface."""

import math

import numpy as np


MODEL_VERSION = 'imdl_yolo_seg_v1'
CLASS_NAMES = {
    0: 'red_block',
    1: 'yellow_block',
    2: 'green_block',
    3: 'blue_block',
    4: 'blue_obstacle',
}


def normalize_model_names(names):
    if isinstance(names, list):
        return {index: str(name) for index, name in enumerate(names)}
    return {int(class_id): str(name) for class_id, name in names.items()}


def clamp_bbox_xyxy(bbox, image_width, image_height):
    if len(bbox) != 4:
        raise ValueError('bbox must contain exactly four coordinates')
    x1, y1, x2, y2 = (float(value) for value in bbox)
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        raise ValueError('bbox coordinates must be finite')
    x1 = min(max(x1, 0.0), float(image_width))
    y1 = min(max(y1, 0.0), float(image_height))
    x2 = min(max(x2, 0.0), float(image_width))
    y2 = min(max(y2, 0.0), float(image_height))
    if x2 <= x1 or y2 <= y1:
        raise ValueError('bbox has zero or negative area')
    return [x1, y1, x2, y2]


def sanitize_polygon(polygon, image_width, image_height):
    points = np.asarray(polygon, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError('mask polygon must contain at least three 2D points')
    if not np.isfinite(points).all():
        raise ValueError('mask polygon coordinates must be finite')
    points[:, 0] = np.clip(points[:, 0], 0.0, float(image_width - 1))
    points[:, 1] = np.clip(points[:, 1], 0.0, float(image_height - 1))
    if len(np.unique(points, axis=0)) < 3:
        raise ValueError('mask polygon must contain three distinct points')
    return points


def polygon_centroid(points):
    x = points[:, 0]
    y = points[:, 1]
    cross = x * np.roll(y, -1) - np.roll(x, -1) * y
    signed_area_twice = float(cross.sum())
    if abs(signed_area_twice) < 1e-9:
        return None
    u = float(((x + np.roll(x, -1)) * cross).sum() / (3.0 * signed_area_twice))
    v = float(((y + np.roll(y, -1)) * cross).sum() / (3.0 * signed_area_twice))
    if not math.isfinite(u) or not math.isfinite(v):
        return None
    return u, v


def make_detection(class_id, confidence, bbox, polygon, image_width, image_height):
    class_id = int(class_id)
    if class_id not in CLASS_NAMES:
        raise ValueError(f'unknown class ID: {class_id}')
    confidence = float(confidence)
    if not math.isfinite(confidence) or confidence < 0.0 or confidence > 1.0:
        raise ValueError('confidence must be within [0, 1]')

    bbox = clamp_bbox_xyxy(bbox, image_width, image_height)
    points = sanitize_polygon(polygon, image_width, image_height)
    center = polygon_centroid(points)
    if center is None:
        center = ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)

    return {
        'class_id': class_id,
        'class_name': CLASS_NAMES[class_id],
        'is_block': class_id != 4,
        'confidence': confidence,
        'bbox_xyxy_px': bbox,
        'center_px': {
            'u': min(max(center[0], 0.0), float(image_width - 1)),
            'v': min(max(center[1], 0.0), float(image_height - 1)),
        },
        'mask_polygon_px': points.tolist(),
    }


def make_payload(header, image_width, image_height, inference_ms, detections):
    ordered = sorted(detections, key=lambda item: item['confidence'], reverse=True)
    return {
        'schema_version': 1,
        'model_version': MODEL_VERSION,
        'timestamp': {
            'sec': int(header.stamp.sec),
            'nanosec': int(header.stamp.nanosec),
        },
        'frame_id': str(header.frame_id),
        'image_size': {
            'width': int(image_width),
            'height': int(image_height),
        },
        'inference_ms': float(inference_ms),
        'detections': ordered,
    }
