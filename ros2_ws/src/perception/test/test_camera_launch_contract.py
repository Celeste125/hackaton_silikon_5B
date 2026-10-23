from pathlib import Path


def test_camera_launch_contains_formal_perception_pipeline_nodes():
    launch = (Path(__file__).parents[1] / 'launch' / 'camera.launch.py').read_text(
        encoding='utf-8'
    )
    for executable in (
        'depth_align',
        'hsv_segmentation',
        'yolo_segmentation',
        'target_pose_estimator',
        'target_selector',
        'perception_visualizer',
        'beacon_heading_controller',
    ):
        assert f"executable='{executable}'" in launch


def test_camera_launch_enables_fused_debug_only_in_debug_mode():
    launch = (Path(__file__).parents[1] / 'launch' / 'camera.launch.py').read_text(
        encoding='utf-8'
    )
    assert "'publish_debug_image'" in launch
    assert "'publish_fused_debug'" in launch
    assert 'runtime_mode' in launch


def test_camera_launch_keeps_hardware_optional_for_simulation():
    launch = (Path(__file__).parents[1] / 'launch' / 'camera.launch.py').read_text(
        encoding='utf-8'
    )
    assert "DeclareLaunchArgument(\n            'use_realsense'" in launch
    assert 'condition=IfCondition(use_realsense)' in launch
    assert "get_package_share_directory('realsense2_camera')" not in launch
    assert "DeclareLaunchArgument(\n            'publish_camera_tf'" in launch
    assert 'condition=IfCondition(publish_camera_tf)' in launch
    assert "'detector_backend'" in launch
    assert "choices=['yolo', 'hsv']" in launch


def test_camera_launch_propagates_clock_and_target_frame():
    launch = (Path(__file__).parents[1] / 'launch' / 'camera.launch.py').read_text(
        encoding='utf-8'
    )
    assert "DeclareLaunchArgument(\n            'use_sim_time'" in launch
    assert "'use_sim_time': use_sim_time" in launch
    assert "'target_frame': target_frame" in launch
    assert "'map_frame': target_frame" in launch


def test_perception_config_exposes_typed_consumer_topic():
    config = (Path(__file__).parents[1] / 'config' / 'perception.yaml').read_text(
        encoding='utf-8'
    )
    assert "typed_objects_topic: '/perception/detected_objects'" in config
