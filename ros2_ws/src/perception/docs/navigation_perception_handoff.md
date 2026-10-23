# Navigation Perception Handoff

This is the navigation team's quick-start document.  The authoritative field
contract remains [Typed Detected Objects Interface](typed_detected_objects_interface.md).

## 1. Subscribe to the Shared Interface

| Item | Value |
| --- | --- |
| Input topic | `/perception/detected_objects` |
| ROS type | `imdl_msgs/msg/DetectedObjectArray` |
| Publisher | `perception/target_pose_estimator.py` |
| Coordinates for navigation | `position_map_m`, in metres, `map` frame |

Build `imdl_msgs` together with the package that consumes the message:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs <your_navigation_package> --symlink-install
source install/setup.bash
ros2 topic echo /perception/detected_objects --once
```

No JSON object-detection topic is published. Use the typed topic above.

## 2. Required Filtering Rule

Navigation may use an object's map position only when:

```python
usable = object_message.map_pose_valid and not object_message.is_stale
```

Never use `position_map_m` when `map_pose_valid` is false.  The point is then
only a default message value, not a measured map position.  Treat the array
header timestamp as the source RGB-frame timestamp; stale messages must not
drive a navigation decision.

`blue_obstacle` has `is_block == false`.  Navigation may keep it as
supplementary visual information, but it is **not** permission to write a
detection directly into the Nav2 costmap.  Known obstacles and the map/costmap
remain the navigation team's responsibility.

## 3. Fields You Need

| Field | Use |
| --- | --- |
| `header.stamp` | Time of the source RGB frame; apply your freshness policy. |
| `class_name` | Object class, including `blue_obstacle`. |
| `position_map_m` | Map-frame point in metres; use only when valid. |
| `map_pose_valid` | Required validity guard for `position_map_m`. |
| `is_stale` | Reject when true. |
| `confidence` | Optional perception-quality input; it does not replace validity checks. |
| `center_u_px`, `center_v_px` | Optional RGB image position; origin is top-left. |

The four block classes are `red_block`, `yellow_block`, `green_block`, and
`blue_block`.  `blue_obstacle` is a fifth class and is never a grasp target.

## 4. Minimal Python Subscription

```python
import rclpy
from rclpy.node import Node
from imdl_msgs.msg import DetectedObjectArray


class NavigationConsumer(Node):
    def __init__(self):
        super().__init__('navigation_consumer')
        self.create_subscription(
            DetectedObjectArray,
            '/perception/detected_objects',
            self.objects_callback,
            10,
        )

    def objects_callback(self, message):
        for obj in message.objects:
            if not obj.map_pose_valid or obj.is_stale:
                continue
            point = obj.position_map_m
            self.get_logger().info(
                f'{obj.class_name}: map=({point.x:.3f}, {point.y:.3f}, {point.z:.3f})'
            )


rclpy.init()
rclpy.spin(NavigationConsumer())
rclpy.shutdown()
```

## 5. Minimal C++ Subscription

```cpp
#include <rclcpp/rclcpp.hpp>
#include <imdl_msgs/msg/detected_object_array.hpp>

class NavigationConsumer : public rclcpp::Node {
public:
  NavigationConsumer() : Node("navigation_consumer") {
    subscription_ = create_subscription<imdl_msgs::msg::DetectedObjectArray>(
      "/perception/detected_objects", 10,
      [this](imdl_msgs::msg::DetectedObjectArray::SharedPtr message) {
        for (const auto & object : message->objects) {
          if (!object.map_pose_valid || object.is_stale) continue;
          const auto & p = object.position_map_m;
          RCLCPP_INFO(get_logger(), "%s map=(%.3f, %.3f, %.3f)",
                      object.class_name.c_str(), p.x, p.y, p.z);
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

Run this without a camera, YOLO, Jetson, Nav2, or robot:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select imdl_msgs imdl_integration_tools --symlink-install
source install/setup.bash
ros2 launch imdl_integration_tools bc_mock_integration.launch.py namespace:=/demo
```

In another terminal, inspect the navigation output:

```bash
ros2 topic echo /demo/integration/navigation/map_valid_objects --once
```

For the default `mixed` scenario, navigation must keep exactly these fresh,
map-valid objects:

| Kept | Why |
| --- | --- |
| `red_block` | Normal valid block. |
| `blue_obstacle` | Valid map position; supplementary visual information only. |
| `green_block` | Map position valid even though depth is unavailable. |
| `blue_block` | Map position valid even though base-frame TF is unavailable. |

It must reject the `yellow_block` without map TF and the stale `red_block`.

## 7. Common Integration Errors

- Using `position_base_m` as a map point: use `position_map_m` only.
- Reading `position_map_m` without checking `map_pose_valid`.
- Ignoring `is_stale` or the message timestamp.
- Treating `blue_obstacle` as an automatic Nav2 costmap update.
- Expecting a JSON object-detection topic instead of the typed topic.

For all fields and their units, see
[DetectedObject.msg](../../imdl_msgs/msg/DetectedObject.msg) and
[DetectedObjectArray.msg](../../imdl_msgs/msg/DetectedObjectArray.msg).
