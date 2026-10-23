import json
from pathlib import Path

import jsonschema


DOCS = Path(__file__).parents[1] / 'docs'


def load_schema(name):
    return json.loads((DOCS / name).read_text(encoding='utf-8'))


def valid_object():
    return {
        'class_id': 3,
        'class_name': 'blue_block',
        'is_block': True,
        'confidence': 0.9,
        'bbox_xyxy_px': [1.0, 1.0, 5.0, 5.0],
        'center_px': {'u': 3.0, 'v': 3.0},
        'mask_polygon_px': [[1.0, 1.0], [5.0, 1.0], [5.0, 5.0]],
        'timestamp': {'sec': 1, 'nanosec': 2},
        'source_frame_id': 'camera_color_optical_frame',
        'depth_m': 0.5,
        'position_camera': {'x': 0.0, 'y': 0.0, 'z': 0.5},
        'position_base': {'x': 0.5, 'y': 0.0, 'z': 0.0},
        'polar_base': {'range_m': 0.5, 'bearing_rad': 0.0},
        'position_map': {'x': 1.0, 'y': 2.0, 'z': 0.0},
        'depth_quality': {'status': 'ok'},
        'transform_status': 'ok',
        'is_stale': False,
        'mask_area_px': 16.0,
        'rect_fill_ratio': 1.0,
        'orientation_rad': 0.0,
        'rotated_bbox_px': [[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]],
        'corners_px': [[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]],
        'corner_status': 'ok',
    }


def test_current_target_schema_accepts_selected_and_empty_states():
    schema = load_schema('current_target_object.schema.json')
    jsonschema.validate({
        'selected': True,
        'requested_class': 'blue_block',
        'is_stale': False,
        'selection_reason': 'fresh_match',
        'object': valid_object(),
    }, schema)
    jsonschema.validate({
        'selected': False,
        'requested_class': 'red_block',
        'is_stale': False,
        'selection_reason': 'no_eligible_target',
        'object': None,
    }, schema)
