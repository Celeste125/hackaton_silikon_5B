# TF and Navigation Handoff

## Frame Contract

The perception pipeline uses this frame chain:

```text
map -> odom -> base_link -> camera_link -> camera_color_optical_frame
```

`position_camera_m` is expressed in `camera_color_optical_frame`.
`position_base_m` is expressed in `base_link`; `range_m` is `hypot(x, y)` and
`bearing_rad` is `atan2(y, x)`. `position_map_m` is usable only when
`map_pose_valid=true`.

The camera extrinsic comes from `camera.launch.py` arguments. Any camera-mount
change requires a new measurement before field testing.

## Navigation Handoff

`blue_obstacle` is published on `/perception/detected_objects` with
`is_block=false`. Navigation may consume only objects with `map_pose_valid`,
finite `position_map_m`, a current timestamp, and a confidence threshold
chosen by B.
Perception does not modify Nav2 costmaps directly.

Known static obstacles should be recorded after arena measurement. Do not
invent coordinates before the map is available.

```yaml
frame_id: map
obstacles:
  - id: blue_obstacle_01
    class_name: blue_obstacle
    position_map: {x: null, y: null, z: null}
    measured_at: null
    notes: "Fill after arena measurement"
```

## Minimal Task-Control Messages

To request a block class, publish one of these strings to
`/requested_target_class`:

```text
red_block | yellow_block | green_block | blue_block
```

`blue_obstacle` is rejected. B advances the colour sequence after its task
state confirms the previous target has been handled.
