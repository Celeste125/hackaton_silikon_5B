import numpy as np
from visualization_msgs.msg import MarkerArray

from perception.perception_visualizer import PerceptionVisualizer

from perception.perception_visualization_utils import (
    class_color_bgr,
    colorize_depth,
    object_depth_label,
    object_has_map_position,
)


def test_colorize_depth_keeps_invalid_pixels_black_and_colours_valid_depth():
    depth_mm = np.array([[0, 50, 500], [1000, 3000, 4000]], dtype=np.uint16)

    colored = colorize_depth(depth_mm, 0.05, 3.0)

    assert colored.shape == (2, 3, 3)
    assert np.array_equal(colored[0, 0], [0, 0, 0])
    assert np.array_equal(colored[1, 2], [0, 0, 0])
    assert np.any(colored[0, 2] != 0)
    assert np.any(colored[1, 1] != 0)


def test_object_has_map_position_requires_ok_status_and_finite_coordinates():
    valid = {
        'map_pose_valid': True,
        'position_map': {'x': 1.0, 'y': -0.5, 'z': 0.1},
    }
    assert object_has_map_position(valid)
    assert not object_has_map_position({**valid, 'map_pose_valid': False})
    assert not object_has_map_position({
        'map_pose_valid': True,
        'position_map': {'x': float('nan'), 'y': 0.0, 'z': 0.0},
    })


def test_object_depth_label_reports_measured_depth_and_planar_range():
    label = object_depth_label({
        'class_name': 'blue_block',
        'confidence': 0.8,
        'depth_m': 0.63,
        'polar_base': {'range_m': 0.61},
        'transform_status': 'ok',
    })
    assert label == 'blue_block 0.80  Z:0.63m R:0.61m'


def test_class_colours_distinguish_blue_block_from_blue_obstacle():
    assert class_color_bgr('blue_block') != class_color_bgr('blue_obstacle')


def test_visualizer_runtime_dependencies_are_importable():
    assert PerceptionVisualizer.__name__ == 'PerceptionVisualizer'
    assert MarkerArray.__name__ == 'MarkerArray'
