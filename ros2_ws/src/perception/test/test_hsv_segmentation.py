import cv2
import numpy as np

from perception.hsv_segmentation_node import detect_hsv_blocks


def test_detect_hsv_blocks_produces_typed_pipeline_schema():
    image = np.zeros((160, 240, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 40), (70, 90), (0, 0, 255), -1)
    cv2.rectangle(image, (145, 50), (190, 105), (255, 0, 0), -1)

    detections = detect_hsv_blocks(image)

    assert {item['class_name'] for item in detections} == {
        'red_block',
        'blue_block',
    }
    for detection in detections:
        assert detection['is_block']
        assert 0.0 <= detection['confidence'] <= 1.0
        assert len(detection['bbox_xyxy_px']) == 4
        assert len(detection['mask_polygon_px']) >= 3


def test_detect_hsv_blocks_ignores_small_color_noise():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[10:12, 10:12] = (0, 255, 255)

    assert detect_hsv_blocks(image, min_area_px=40.0) == []
