# Typed Detected Objects Interface

`/perception/detected_objects` is the shared typed perception output for
navigation, control, and diagnostics. Its type is
`imdl_msgs/msg/DetectedObjectArray`; it is published once for every processed
RGB frame by `target_pose_estimator`.

This is the only external object-detection output. Navigation and control
consumers must subscribe to the typed `/perception/detected_objects` topic.

## Consumer Rules

- Read `class_name`, `is_block`, image coordinates, confidence, and the header
  timestamp for every object.
- `depth_m` and depth diagnostics are valid only when `depth_valid` is true.
- Navigation may use `position_map_m` only when `map_pose_valid` is true and
  the object header is fresh. A `blue_obstacle` is published with
  `is_block=false`; it must not become a grasp target.
- A controller may use `position_base_m` only when `base_pose_valid` is true.
  It must transform this point to `arm_base_link` before commanding the arm.
- `position_camera_m` uses the frame in the object header. `position_base_m`
  and `position_map_m` use the configured `base_frame` and `target_frame`.

The image origin is top-left. `center_u_px` increases to the right and
`center_v_px` increases down. `bbox_xyxy_px` is
`[x_min, y_min, x_max, y_max]` in the source RGB image.

## Inspection

```bash
ros2 topic echo /perception/detected_objects --once
ros2 interface show imdl_msgs/msg/DetectedObject
```
