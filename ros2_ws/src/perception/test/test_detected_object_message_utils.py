from perception.detected_object_message_utils import make_detected_object_array


def enriched_object(**overrides):
    object_data = {
        'class_id': 3,
        'class_name': 'blue_block',
        'is_block': True,
        'confidence': 0.9,
        'bbox_xyxy_px': [1.0, 2.0, 5.0, 6.0],
        'center_px': {'u': 3.0, 'v': 4.0},
        'mask_polygon_px': [[1.0, 2.0], [5.0, 2.0], [5.0, 6.0]],
        'timestamp': {'sec': 12, 'nanosec': 34},
        'source_frame_id': 'camera_color_optical_frame',
        'depth_m': 0.5,
        'position_camera': {'x': 0.0, 'y': 0.0, 'z': 0.5},
        'position_base': {'x': 0.5, 'y': 0.1, 'z': 0.0},
        'polar_base': {'range_m': 0.51, 'bearing_rad': 0.2},
        'position_map': {'x': 1.0, 'y': 2.0, 'z': 0.0},
        'depth_quality': {
            'valid_depth_pixels': 42,
            'valid_depth_ratio': 0.8,
            'depth_mad_m': 0.01,
        },
        'transform_status': 'ok',
        'is_stale': False,
        'mask_area_px': 50.0,
    }
    object_data.update(overrides)
    return object_data


def payload():
    return {
        'timestamp': {'sec': 12, 'nanosec': 34},
        'frame_id': 'camera_color_optical_frame',
    }


def test_typed_object_preserves_class_image_and_metric_fields():
    message = make_detected_object_array(payload(), [enriched_object()])
    assert message.header.frame_id == 'camera_color_optical_frame'
    assert message.header.stamp.sec == 12
    assert len(message.objects) == 1

    object_message = message.objects[0]
    assert object_message.class_name == 'blue_block'
    assert object_message.is_block
    assert object_message.center_u_px == 3.0
    assert list(object_message.bbox_xyxy_px) == [1.0, 2.0, 5.0, 6.0]
    assert object_message.mask_polygon_px[1].x == 5.0
    assert object_message.depth_valid
    assert object_message.base_pose_valid
    assert object_message.map_pose_valid
    assert object_message.depth_m == 0.5
    assert object_message.position_map_m.y == 2.0
    assert object_message.valid_depth_pixels == 42


def test_typed_object_marks_missing_depth_and_tf_as_invalid():
    message = make_detected_object_array(payload(), [enriched_object(
        depth_m=None,
        position_camera=None,
        position_base=None,
        position_map=None,
        polar_base={'range_m': None, 'bearing_rad': None},
        depth_quality=None,
        transform_status='depth_unavailable',
    )])
    object_message = message.objects[0]
    assert not object_message.depth_valid
    assert not object_message.base_pose_valid
    assert not object_message.map_pose_valid
    assert object_message.transform_status == 'depth_unavailable'


def test_blue_obstacle_remains_non_block_in_typed_interface():
    message = make_detected_object_array(payload(), [enriched_object(
        class_id=4,
        class_name='blue_obstacle',
        is_block=False,
    )])
    assert message.objects[0].class_name == 'blue_obstacle'
    assert not message.objects[0].is_block
