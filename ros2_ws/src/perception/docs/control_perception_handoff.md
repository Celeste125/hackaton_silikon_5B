# Control Perception Handoff

This is the control / manipulator team's quick-start document.  The
authoritative field contract remains [Typed Detected Objects Interface](typed_detected_objects_interface.md).

## 1. Subscribe to the Shared Interface

| Item | Value |
| --- | --- |
| Input topic | `/perception/detected_objects` |
| ROS type | `imdl_msgs/msg/DetectedObjectArray` |
| Publisher | `perception/target_pose_estimator.py` |
| Coordinates supplied to control | `position_base_m`, in metres, configured base frame |

Build `imdl_msgs` together with the package that consumes the message:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs <your_control_package> --symlink-install
source install/setup.bash
ros2 topic echo /perception/detected_objects --once
```

Do not consume either the legacy JSON migration topic `/detected_objects` or
the internal JSON detection topic `/yolo/detections`.  The supported control
interface is the typed topic above.

## 2. Required Grasp-Candidate Rule

A message is only a perception candidate.  Before any arm command, control
must apply this minimum filter:

```python
graspable = (
    object_message.is_block
    and object_message.depth_valid
    and object_message.base_pose_valid
    and not object_message.is_stale
)
```

`blue_obstacle` always has `is_block == false`, so it must never enter the
grasp queue.  `position_base_m` is expressed in the configured mobile-base
frame, not necessarily the arm's `arm_base_link`.  Transform it to
`arm_base_link`, then perform the controller's own reachability, collision,
gripper, and safety checks before moving.

`range_m` and `bearing_rad` are also valid only when `base_pose_valid` is
true.  Do not use a default numeric value from an invalid base-frame pose.

## 3. Fields You Need

| Field | Use |
| --- | --- |
| `header.stamp` | Time of the source RGB frame; apply your freshness policy. |
| `class_name`, `is_block` | Select a requested block and reject obstacles. |
| `position_base_m` | Base-frame point in metres; use only when valid. |
| `base_pose_valid` | Required validity guard for `position_base_m`. |
| `range_m` | Planar distance from the `base_link` origin in metres: `sqrt(x^2 + y^2)`. Use only when `base_pose_valid` is true. |
| `bearing_rad` | Object direction relative to the forward axis of `base_link`, in radians: `atan2(y, x)`. Use only when `base_pose_valid` is true. |
| `depth_m`, `depth_valid` | Depth estimate in metres and required validity guard. |
| `is_stale` | Reject when true. |
| `confidence` | Optional quality input; it does not replace validity checks. |
| `center_u_px`, `center_v_px`, `mask_polygon_px` | Optional image / grasp-strategy inputs. |

Image coordinates are in source RGB pixels, with origin at top-left; `u`
increases rightward and `v` downward.  The four graspable class names are
`red_block`, `yellow_block`, `green_block`, and `blue_block`.

### Base-frame Geometry and RealSense Extrinsics

`position_base_m`, `range_m`, and `bearing_rad` are calculated by the
perception node after it transforms the RealSense RGB-D point into the
configured base frame (by default, `base_link`).  In the default hardware
configuration, with `publish_camera_tf:=true`, the following measured static
transform is published at startup:

```text
base_link -> camera_link
translation: x=+0.190 m, y=0.000 m, z=-0.020 m
orientation: roll=0.000 rad, pitch=+0.262 rad (about +15.0 deg), yaw=0.000 rad
```

Thus the camera-link origin is about `0.191 m` from the `base_link` origin.
These values describe the camera mounting; control normally consumes the
already transformed `position_base_m` and does not need to apply this camera
transform itself.

If the controller needs a three-dimensional straight-line distance rather
than the planar `range_m`, calculate it from the supplied base-frame point:

```python
import math

distance_3d_m = math.sqrt(
    point.x ** 2 + point.y ** 2 + point.z ** 2
)
```

## 4. Minimal Python Subscription

```python
import rclpy
from rclpy.node import Node
from imdl_msgs.msg import DetectedObjectArray


class ControlConsumer(Node):
    def __init__(self):
        super().__init__('control_consumer')
        self.create_subscription(
            DetectedObjectArray,
            '/perception/detected_objects',
            self.objects_callback,
            10,
        )

    def objects_callback(self, message):
        for obj in message.objects:
            if not (obj.is_block and obj.depth_valid and
                    obj.base_pose_valid and not obj.is_stale):
                continue
            point = obj.position_base_m
            self.get_logger().info(
                f'{obj.class_name}: base=({point.x:.3f}, {point.y:.3f}, {point.z:.3f})'
            )
            # Transform point from configured base frame to arm_base_link,
            # then execute this package's reachability and safety checks.


rclpy.init()
rclpy.spin(ControlConsumer())
rclpy.shutdown()
```

## 5. Minimal C++ Subscription

```cpp
#include <rclcpp/rclcpp.hpp>
#include <imdl_msgs/msg/detected_object_array.hpp>

class ControlConsumer : public rclcpp::Node {
public:
  ControlConsumer() : Node("control_consumer") {
    subscription_ = create_subscription<imdl_msgs::msg::DetectedObjectArray>(
      "/perception/detected_objects", 10,
      [this](imdl_msgs::msg::DetectedObjectArray::SharedPtr message) {
        for (const auto & object : message->objects) {
          if (!object.is_block || !object.depth_valid ||
              !object.base_pose_valid || object.is_stale) continue;
          const auto & p = object.position_base_m;
          RCLCPP_INFO(get_logger(), "%s base=(%.3f, %.3f, %.3f)",
                      object.class_name.c_str(), p.x, p.y, p.z);
          // Transform base-frame point to arm_base_link and safety-check it.
        }
      });
  }
private:
  rclcpp::Subscription<imdl_msgs::msg::DetectedObjectArray>::SharedPtr subscription_;
};
```

Add `imdl_msgs` as a dependency in both `package.xml` and the relevant
`CMakeLists.txt` / `setup.py` of your package.

## 6. Laptop Mock Acceptance Test

Run this without a camera, YOLO, Jetson, arm, or robot:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs imdl_integration_tools --symlink-install
source install/setup.bash
ros2 launch imdl_integration_tools bc_mock_integration.launch.py namespace:=/demo
```

In another terminal, inspect the control output:

```bash
ros2 topic echo /demo/integration/control/grasp_candidates --once
```

For the default `mixed` scenario, control must keep exactly these candidates:

| Kept | Why |
| --- | --- |
| `red_block` | Block with valid depth and base-frame position. |
| `yellow_block` | Also has valid depth and base-frame position; map TF is not required for this filter. |

It must reject the `blue_obstacle`, the depth-invalid `green_block`, the
base-TF-invalid `blue_block`, and the stale `red_block`.

This mock verifies interface filtering only.  It does not validate arm
kinematics, TF accuracy, collision avoidance, grasp success, or depth accuracy
on the laboratory robot.

## 7. Common Integration Errors

- Treating `blue_obstacle` as a grasp target.
- Using `position_base_m` without checking `base_pose_valid`.
- Using `depth_m` without checking `depth_valid`.
- Sending a mobile-base-frame point directly to the arm without transforming
  it to `arm_base_link`.
- Ignoring `is_stale`, reachability, collision, and gripper safety checks.
- Consuming either legacy `/detected_objects` JSON or internal
  `/yolo/detections` JSON instead of the typed control interface.

For all fields and their units, see
[DetectedObject.msg](../../imdl_msgs/msg/DetectedObject.msg) and
[DetectedObjectArray.msg](../../imdl_msgs/msg/DetectedObjectArray.msg).
