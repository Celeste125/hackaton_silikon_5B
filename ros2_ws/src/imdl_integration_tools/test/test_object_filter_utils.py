from builtin_interfaces.msg import Time

from imdl_integration_tools.mock_object_fixtures import make_scenario
from imdl_integration_tools.object_filter_utils import (
    filtered_array,
    is_control_candidate,
    is_navigation_candidate,
)


def mixed_message():
    return make_scenario('mixed', Time(sec=42, nanosec=7))


def test_navigation_filter_uses_map_validity_and_staleness_only():
    result = filtered_array(mixed_message(), is_navigation_candidate)

    assert [object_message.class_name for object_message in result.objects] == [
        'red_block',
        'blue_obstacle',
        'green_block',
        'blue_block',
    ]
    assert result.header.stamp.sec == 42
    assert result.header.frame_id == 'camera_color_optical_frame'


def test_control_filter_requires_a_fresh_block_depth_and_base_pose():
    result = filtered_array(mixed_message(), is_control_candidate)

    assert [object_message.class_name for object_message in result.objects] == [
        'red_block',
        'yellow_block',
    ]
    assert all(object_message.is_block for object_message in result.objects)
    assert all(object_message.depth_valid for object_message in result.objects)
    assert all(object_message.base_pose_valid for object_message in result.objects)
