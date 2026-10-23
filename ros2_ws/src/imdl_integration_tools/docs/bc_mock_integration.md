# B/C Typed Object Mock Integration

`imdl_integration_tools` exercises the shared
`imdl_msgs/msg/DetectedObjectArray` interface without a camera, YOLO, Jetson,
Nav2, or a robot arm. It is an interface rehearsal tool, not a motion-control
tool.

## Start the Demonstration

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs imdl_integration_tools --symlink-install
source install/setup.bash
ros2 launch imdl_integration_tools bc_mock_integration.launch.py namespace:=/demo
```

The launch uses relative topic names, so the command above creates:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/demo/perception/detected_objects` | `imdl_msgs/msg/DetectedObjectArray` | Deterministic mock input. |
| `/demo/integration/navigation/map_valid_objects` | `imdl_msgs/msg/DetectedObjectArray` | B navigation candidates. |
| `/demo/integration/control/grasp_candidates` | `imdl_msgs/msg/DetectedObjectArray` | C grasp candidates. |

Inspect them with:

```bash
ros2 topic echo /demo/perception/detected_objects --once
ros2 topic echo /demo/integration/navigation/map_valid_objects --once
ros2 topic echo /demo/integration/control/grasp_candidates --once
```

## Consumer Rules Demonstrated

The default `mixed` scenario contains a valid red block, a valid blue obstacle,
a block without map TF, a block without depth, a block without base TF, and a
stale block.

Navigation keeps only:

```python
object.map_pose_valid and not object.is_stale
```

It may therefore retain a `blue_obstacle` as supplementary visual information.
It must not treat the mock output as permission to write an obstacle into the
Nav2 costmap: known obstacles remain map/costmap responsibilities.

Control keeps only:

```python
object.is_block and object.depth_valid and object.base_pose_valid and not object.is_stale
```

It never admits `blue_obstacle`. Before an arm command, C must transform the
retained `position_base_m` to `arm_base_link` and perform its own safety checks.

## What This Does Not Validate

This package does not validate RealSense depth accuracy, real TF latency,
Nav2 path planning, collision avoidance, arm kinematics, or the final task
timing. Those require the laboratory robot validation stage.
