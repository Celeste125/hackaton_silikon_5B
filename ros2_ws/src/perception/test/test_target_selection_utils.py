from perception.target_selection_utils import parse_class_sequence, select_best_target


def object_item(name, confidence, range_m, is_block=True, map_pose_valid=True):
    return {
        'class_name': name,
        'is_block': is_block,
        'confidence': confidence,
        'map_pose_valid': map_pose_valid,
        'is_stale': False,
        'position_map': {'x': 1.0, 'y': 2.0, 'z': 0.0},
        'polar_base': {'range_m': range_m, 'bearing_rad': 0.1},
    }


def test_sequence_parser_discards_empty_values():
    assert parse_class_sequence(' red_block, ,blue_block ') == ['red_block', 'blue_block']


def test_blue_obstacle_cannot_be_selected_as_a_block_target():
    obstacle = object_item('blue_obstacle', 0.99, 0.2, is_block=False)
    assert select_best_target([obstacle], 'blue_obstacle', 0.1) is None


def test_invalid_confidence_cannot_break_target_selection():
    malformed = object_item('red_block', 0.9, 1.0)
    malformed['confidence'] = 'not-a-number'
    assert select_best_target([malformed], 'red_block', 0.1) is None


def test_highest_confidence_wins_before_distance():
    near = object_item('blue_block', 0.80, 0.2)
    confident = object_item('blue_block', 0.90, 1.0)
    assert select_best_target([near, confident], 'blue_block', 0.1) is confident


def test_nearest_candidate_breaks_confidence_tie():
    far = object_item('red_block', 0.9, 2.0)
    near = object_item('red_block', 0.9, 1.0)
    assert select_best_target([far, near], 'red_block', 0.1) is near


def test_position_continuity_breaks_equal_confidence_and_range_tie():
    far = object_item('red_block', 0.9, 1.0)
    near = object_item('red_block', 0.9, 1.0)
    far['position_map'] = {'x': 5.0, 'y': 5.0, 'z': 0.0}
    near['position_map'] = {'x': 1.1, 'y': 1.0, 'z': 0.0}
    previous = {'x': 1.0, 'y': 1.0, 'z': 0.0}
    assert select_best_target([far, near], 'red_block', 0.1, previous) is near
