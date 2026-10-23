from pathlib import Path


ROOT = Path(__file__).parents[2]


def test_simulation_configures_namespaced_dynamic_tf_pair():
    launch = (
        ROOT / 'imdl_simulation' / 'launch' / 'autonomy.launch.py'
    ).read_text(encoding='utf-8')
    republisher = (
        ROOT / 'slam' / 'slam' / 'odom_tf_republisher.py'
    ).read_text(encoding='utf-8')

    assert "'parent_frame_id': 'Team2/odom'" in launch
    assert "'child_frame_id': 'Team2/base_link'" in launch
    assert "declare_parameter('parent_frame_id'" in republisher
    assert "declare_parameter('child_frame_id'" in republisher
