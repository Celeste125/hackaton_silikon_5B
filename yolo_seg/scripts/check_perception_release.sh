#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
package_dir="$project_root/ros2_ws/src/perception"
messages_dir="$project_root/ros2_ws/src/imdl_msgs"

(
    cd "$package_dir/models"
    sha256sum -c yolo11n_seg_imdl.sha256
)
if rg -n '/detected_color|enable_blue_size_filter|real_size_from_bbox' \
    "$package_dir/perception" "$package_dir/launch"; then
    echo 'Legacy HSV dependency found in active perception code.' >&2
    exit 1
fi
test -f "$package_dir/config/perception.yaml"
test -f "$package_dir/docs/detected_objects.schema.json"
test -f "$package_dir/docs/current_target_object.schema.json"
test -f "$package_dir/docs/typed_detected_objects_interface.md"
test -f "$messages_dir/package.xml"
test -f "$messages_dir/CMakeLists.txt"
test -f "$messages_dir/msg/DetectedObject.msg"
test -f "$messages_dir/msg/DetectedObjectArray.msg"

if ! rg -q "typed_objects_topic: '/perception/detected_objects'" \
    "$package_dir/config/perception.yaml"; then
    echo 'The typed detected-objects topic is not pinned in perception.yaml.' >&2
    exit 1
fi
if ! rg -q 'DetectedObjectArray' "$package_dir/perception/target_pose_estimator.py"; then
    echo 'The target pose estimator no longer publishes the typed message.' >&2
    exit 1
fi

echo 'Perception release preflight passed.'
