# Typed YOLO + RealSense Perception Integration

The production perception input for `imdl_autonomy` is the final Member-A
pipeline. It publishes one typed array per processed RGB frame:

| Item | Value |
| --- | --- |
| Topic | `/perception/detected_objects` |
| Type | `imdl_msgs/msg/DetectedObjectArray` |
| Depth source | RealSense aligned RGB-D depth, measured inside the YOLO segmentation mask |

`imdl_autonomy` subscribes only to this typed topic. The former HSV/JSON
perception topics are not part of this integration.

## Required Object Validity

- Object tracking/navigation requires a fresh, non-stale block with
  `map_pose_valid=true`.
- Grasp visual alignment requires a fresh, non-stale block with
  `depth_valid=true` and `base_pose_valid=true`.
- `blue_obstacle` has `is_block=false`; it is never a grasp candidate.

The following fields come from the RealSense/YOLO pipeline and are used by the
autonomy stack: `depth_m`, `range_m`, `bearing_rad`, `position_base_m`,
`position_map_m`, confidence, image centre, depth diagnostics, and timestamps.

LiDAR `/scan` remains a localization and Nav2 safety input. Front ultrasonic
range is optional and disabled on hardware until a real
`/ultrasonic/front/range` publisher exists; neither replaces object depth from
RealSense.
