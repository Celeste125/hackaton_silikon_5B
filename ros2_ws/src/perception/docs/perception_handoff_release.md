# Perception Handoff Release Candidate

This document freezes the laptop-validated perception handoff state. It is a
release candidate until the Jetson hardware checklist below is completed.

## Included Interface

- Model: `models/yolo11n_seg_imdl.pt`
- Integrity file: `models/yolo11n_seg_imdl.sha256`
- Typed output: `/perception/detected_objects`
- ROS type: `imdl_msgs/msg/DetectedObjectArray`

`imdl_msgs` and `perception` must be built together. The typed topic is the
only supported object output.

## Consumer Contract

- Navigation uses `position_map_m` only when `map_pose_valid` is true and the
  message is fresh. A `blue_obstacle` is not a grasp target.
- Control uses `position_base_m` only when `base_pose_valid` is true, then
  transforms it to `arm_base_link` before commanding the arm.
- `depth_m` is meaningful only when `depth_valid` is true.
- Image coordinates use the RGB source image: origin top-left, `u` to the
  right, and `v` downward.

The complete field definitions and validity rules are in
`typed_detected_objects_interface.md`.

## Laptop Acceptance Checks

From the repository root:

```bash
./yolo_seg/scripts/check_perception_release.sh
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs perception --symlink-install
source install/setup.bash
ros2 interface show imdl_msgs/msg/DetectedObjectArray
```

The repository tests also validate class, image, depth, base-frame, and
map-frame conversion through `test_detected_object_message_utils.py`.

`test_typed_object_publisher_integration.py` starts the real
`target_pose_estimator` with mock YOLO, CameraInfo, aligned-depth, and static
TF publishers. It verifies normal, missing-depth, missing-TF,
depth-unsynchronised, and `blue_obstacle` typed outputs without requiring a
camera or Jetson.

On 2026-07-17, the complete perception test suite passed with 80 tests and no
failures after this integration test was added. This is software-level evidence
only; the Jetson checklist below remains required for final release.

## Jetson Completion Checklist

Before declaring the release final, perform these checks on the robot:

1. Run `deploy_perception_to_jetson.sh` without `--apply`, inspect both package
   transfers, then rerun with `--apply`.
2. Verify the model hash and build `imdl_msgs` plus `perception` successfully.
3. Launch the RGB-D pipeline and inspect one typed message with
   `ros2 topic echo /perception/detected_objects --once`.
4. Confirm the four block colours and the blue obstacle have correct class,
   image position, depth-valid flag, and base/map validity flags.
5. Have navigation and control consume the typed topic independently; retain
   recorded rosbag evidence for the final task validation.

Do not create a final release tag until this checklist and the task-oriented
lab validation in `document/plan.md` are complete.
