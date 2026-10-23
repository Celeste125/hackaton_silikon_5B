# IMDL Object Geometry Interface

The supported object-geometry output is
`/perception/detected_objects` with type
`imdl_msgs/msg/DetectedObjectArray`.

`target_pose_estimator` combines YOLO segmentation, RealSense aligned depth,
CameraInfo, and TF.  It measures depth using valid pixels inside each
segmentation mask and fills the message validity flags independently:

- `depth_valid` for `depth_m`;
- `base_pose_valid` for `position_base_m`, `range_m`, and `bearing_rad`;
- `map_pose_valid` for `position_map_m`.

Navigation and control must use those flags rather than infer validity from a
default coordinate value. `blue_obstacle` is published with `is_block=false`.

The YOLO node's `/yolo/detections` JSON is an internal pipeline message between
the segmentation and geometry nodes. It is not a consumer interface. No JSON
object-detection topic is published.

See [Typed Detected Objects Interface](typed_detected_objects_interface.md) for
the complete field contract and consumer rules.
