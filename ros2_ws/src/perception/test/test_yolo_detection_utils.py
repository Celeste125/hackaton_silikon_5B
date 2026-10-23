import json
from pathlib import Path
from types import SimpleNamespace

import jsonschema
import pytest

from perception.yolo_detection_utils import (
    CLASS_NAMES,
    clamp_bbox_xyxy,
    make_detection,
    make_payload,
    normalize_model_names,
)


def make_header(sec=12, nanosec=34, frame_id='camera_color_optical_frame'):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        frame_id=frame_id,
    )


def load_schema():
    path = Path(__file__).parents[1] / 'docs' / 'yolo_detections.schema.json'
    return json.loads(path.read_text(encoding='utf-8'))


def square_detection(class_id=3, confidence=0.9):
    return make_detection(
        class_id=class_id,
        confidence=confidence,
        bbox=[0.0, 0.0, 10.0, 10.0],
        polygon=[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
        image_width=640,
        image_height=480,
    )


def test_model_names_are_normalized_and_frozen():
    assert normalize_model_names(list(CLASS_NAMES.values())) == CLASS_NAMES
    string_key_names = {str(key): value for key, value in CLASS_NAMES.items()}
    assert normalize_model_names(string_key_names) == CLASS_NAMES


def test_blue_obstacle_is_not_a_block():
    detection = square_detection(class_id=4)
    assert detection['class_name'] == 'blue_obstacle'
    assert detection['is_block'] is False


def test_mask_centroid_is_used_for_valid_polygon():
    detection = square_detection()
    assert detection['center_px'] == pytest.approx({'u': 5.0, 'v': 5.0})


def test_bbox_center_is_fallback_for_degenerate_polygon():
    detection = make_detection(
        class_id=0,
        confidence=0.8,
        bbox=[10.0, 20.0, 30.0, 40.0],
        polygon=[[10.0, 20.0], [20.0, 30.0], [30.0, 40.0]],
        image_width=640,
        image_height=480,
    )
    assert detection['center_px'] == pytest.approx({'u': 20.0, 'v': 30.0})


def test_coordinates_are_clamped_to_source_image():
    detection = make_detection(
        class_id=1,
        confidence=0.8,
        bbox=[-10.0, -5.0, 700.0, 500.0],
        polygon=[[-10.0, -5.0], [700.0, 0.0], [700.0, 500.0]],
        image_width=640,
        image_height=480,
    )
    assert detection['bbox_xyxy_px'] == [0.0, 0.0, 640.0, 480.0]
    assert all(
        0.0 <= u <= 639.0 and 0.0 <= v <= 479.0
        for u, v in detection['mask_polygon_px']
    )


def test_invalid_polygon_is_rejected():
    with pytest.raises(ValueError, match='at least three'):
        make_detection(0, 0.8, [0, 0, 10, 10], [[0, 0], [1, 1]], 640, 480)


def test_invalid_bbox_is_rejected():
    with pytest.raises(ValueError, match='zero or negative'):
        clamp_bbox_xyxy([10, 10, 5, 5], 640, 480)


def test_payload_is_sorted_and_schema_valid():
    payload = make_payload(
        make_header(),
        640,
        480,
        8.5,
        [square_detection(0, 0.55), square_detection(3, 0.95)],
    )
    assert [item['confidence'] for item in payload['detections']] == [0.95, 0.55]
    jsonschema.validate(payload, load_schema())


def test_empty_payload_preserves_header_and_schema():
    payload = make_payload(make_header(100, 200, 'rgb_frame'), 640, 480, 3.0, [])
    assert payload['timestamp'] == {'sec': 100, 'nanosec': 200}
    assert payload['frame_id'] == 'rgb_frame'
    assert payload['detections'] == []
    jsonschema.validate(payload, load_schema())
