from slam.odom_tf_republisher import normalized_frame_id


def test_normalized_frame_id_removes_topic_style_prefix():
    assert normalized_frame_id('/Team2/odom') == 'Team2/odom'
    assert normalized_frame_id('  /Team2/base_link ') == 'Team2/base_link'


def test_normalized_frame_id_preserves_nested_tf_name():
    assert normalized_frame_id('robot/base_link') == 'robot/base_link'
