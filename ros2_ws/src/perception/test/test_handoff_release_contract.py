from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
PERCEPTION_DIR = REPOSITORY_ROOT / 'ros2_ws' / 'src' / 'perception'
SCRIPTS_DIR = REPOSITORY_ROOT / 'yolo_seg' / 'scripts'


def test_preflight_covers_model_messages_and_typed_topic():
    script = (SCRIPTS_DIR / 'check_perception_release.sh').read_text()

    assert 'yolo11n_seg_imdl.sha256' in script
    assert 'imdl_msgs' in script
    assert 'DetectedObject.msg' in script
    assert 'DetectedObjectArray.msg' in script
    assert "/perception/detected_objects" in script


def test_deployment_builds_both_required_ros_packages():
    script = (SCRIPTS_DIR / 'deploy_perception_to_jetson.sh').read_text()

    assert 'packages=(imdl_msgs perception)' in script
    assert 'colcon build --packages-select imdl_msgs perception' in script
    assert '--delete' not in script


def test_handoff_document_points_consumers_to_typed_interface():
    handoff = (PERCEPTION_DIR / 'docs' / 'perception_handoff_release.md').read_text()

    assert 'imdl_msgs/msg/DetectedObjectArray' in handoff
    assert '/perception/detected_objects' in handoff
    assert 'Jetson Completion Checklist' in handoff
