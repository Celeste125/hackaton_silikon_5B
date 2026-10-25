# Intelligent Machine Design Laboratory ROS 2 Workspace

The current rover workflow is **explore → detect → pause → align → measure
range → save map coordinates → resume exploring**. A camera and replaceable
detector run on the Jetson alongside the lidar driver. Small, timestamped
detections cross ROS 2 to a separate Linux laptop running SLAM, Nav2 and the
frontier explorer, which yields to the approach controller on that laptop.
Range comes from bounding-box growth and measured odometry.
The laptop records one food dispenser and one cow stall in a JSON file and
shows them in RViz. A blocked probe is deferred until a different viewpoint.
Previously recorded targets can be measured again after a cooldown and movement
to another viewpoint; compatible measurements refine the saved position.

Start with the [object approach runbook](ros2_ws/src/imdl_autonomy/docs/object_approach.md).
A run is two commands: `jetson_bringup.launch.py` on the robot and
`autonomy_bringup.launch.py` on whichever machine carries the autonomy --
the laptop, or the Jetson itself with `--onboard`, which keeps the scans and
the detections off the WiFi. The runbook covers both,
plus the model interface, range derivation, configuration and hardware-free
tests. No trained model is required to build
or test the controller. Add weights later through `backend` and `model_path`.

The workspace also retains the earlier SLAM, RGB-D block perception, landmark
mapping, and manipulation pipelines documented below. Use the new runbook for
the dispenser/cow-stall approach; it replaces the old camera-coverage handoff.

The active development branch for this work is:

```bash
feature/slam_perception
```

## Repository Layout

```text
.
├── document/
│   ├── plan.md
│   ├── group-plan_en.md
│   └── task specifications and local demonstration evidence
├── maps/
│   ├── arena_map.yaml
│   ├── arena_map.pgm
│   ├── arena_map.data
│   └── arena_map.posegraph
├── yolo_seg/
│   ├── data/
│   │   ├── releases/final_v1/
│   │   ├── raw_bags/
│   │   ├── cvat_exports/
│   │   └── archive/
│   ├── experiment/
│   │   ├── training/
│   │   ├── evaluation/
│   │   └── archive/
│   ├── model/                 # local generic pretrained weights
│   ├── registry/              # tracked model provenance
│   └── scripts/
└── ros2_ws/
    ├── dependencies.repos     # pinned external ROS package sources
    └── src/
        ├── perception/
        │   ├── config/perception.yaml
        │   ├── launch/
        │   │   ├── camera.launch.py
        │   │   └── yolo_segmentation.launch.py
        │   ├── models/yolo11n_seg_imdl.pt
        │   └── perception/
        │       ├── depth_align.py
        │       ├── yolo_segmentation_node.py
        │       ├── target_pose_estimator.py
        │       ├── target_selector.py
        │       ├── beacon_heading_controller.py
        │       ├── perception_visualizer.py
        │       ├── object_geometry_utils.py
        │       ├── perception_visualization_utils.py
        │       ├── target_selection_utils.py
        │       └── yolo_detection_utils.py
        └── slam/
            ├── config/
            │   ├── amcl_params.yaml
            │   └── slam_toolbox_params.yaml
            ├── launch/
            │   ├── localization.launch.py
            │   └── mapping.launch.py
            └── slam/
                ├── odom_tf_republisher.py
                └── scan_timestamp_fixer.py
```

`yolo_seg/data/`, `yolo_seg/experiment/`, raw recordings, local generic
weights, and Colcon build outputs are intentionally excluded from Git. The
deployable model, runtime schemas, ROS source, maps, documentation, and model
registry remain version-controlled.

## Runtime Assumptions

- ROS 2 Humble is installed.
- The robot publishes Create3 odometry on `/Team2/odom`.
- The SLAMTEC LIDAR is connected on `/dev/ttyUSB0`.
- The Intel RealSense camera publishes RGB and depth images through
  `realsense2_camera`.
- The saved map for localization is available at `~/maps/arena_map.yaml`.
- The Jetson and Create3 use the same ROS 2 network settings:
  `ROS_DOMAIN_ID=2` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

If the map files are stored in this repository, copy them to the runtime path:

```bash
mkdir -p ~/maps
cp maps/arena_map.* ~/maps/
```

## Build

From the ROS 2 workspace:

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
colcon build --packages-select slam perception --symlink-install
source install/setup.bash
```

## External ROS Dependencies

The SLAMTEC LIDAR driver is an external dependency rather than copied into the
project history. Restore its pinned source after cloning the repository:

```bash
sudo apt install python3-vcstool
cd ~/Desktop/intelligent-machine-design-laboratory
vcs import --skip-existing ros2_ws/src < ros2_ws/dependencies.repos
```

## SLAM Package

The `slam` package handles LIDAR input, odometry TF republishing, mapping, and
AMCL localization.

### Key Files

| File | Purpose |
| --- | --- |
| `ros2_ws/src/slam/launch/mapping.launch.py` | Starts the LIDAR driver, fixes scan timestamps, republishes odom TF, and launches `slam_toolbox` for mapping. |
| `ros2_ws/src/slam/launch/localization.launch.py` | Starts localization-only mode with LIDAR, timestamp correction, odom TF, `map_server`, lifecycle manager, and AMCL. |
| `ros2_ws/src/slam/slam/scan_timestamp_fixer.py` | Subscribes to `/scan_raw`, rewrites the timestamp using the Jetson clock, and republishes `/scan`. |
| `ros2_ws/src/slam/slam/odom_tf_republisher.py` | Subscribes to `/Team2/odom` and republishes `odom -> base_link` TF using the Jetson clock. |
| `ros2_ws/src/slam/config/amcl_params.yaml` | AMCL particle filter, laser model, frame, and update parameters. |
| `ros2_ws/src/slam/config/slam_toolbox_params.yaml` | Parameters for `slam_toolbox` mapping. |

### Mapping

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source install/setup.bash
ros2 launch slam mapping.launch.py
```

Important topics and frames:

| Name | Description |
| --- | --- |
| `/scan_raw` | Raw LIDAR scan from `sllidar_node`. |
| `/scan` | Timestamp-corrected scan used by SLAM and AMCL. |
| `/Team2/odom` | Create3 odometry input. |
| `/tf` | Includes the republished `odom -> base_link` transform. |
| `/map` | Occupancy grid from `slam_toolbox`. |
| `/robot_pose` | Remapped robot pose output. |

### Localization

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source install/setup.bash
ros2 launch slam localization.launch.py
```

Useful checks:

```bash
ros2 topic list
ros2 topic echo /robot_pose --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /scan
ros2 topic hz /Team2/odom
```

## Perception Package

The `perception` package starts the RealSense camera pipeline, aligns depth to
the RGB image, performs YOLO instance segmentation, estimates 3D object
positions from mask depth, selects the requested block target, and generates a
beacon heading command.

### Key Files

| File | Purpose |
| --- | --- |
| `ros2_ws/src/perception/launch/camera.launch.py` | Formal single-command launch: RealSense, depth alignment, YOLO, 3D geometry, target selection, beacon control, and static `base_link -> camera_link` TF. |
| `ros2_ws/src/perception/config/perception.yaml` | Shared runtime parameters for all perception nodes. |
| `ros2_ws/src/perception/perception/depth_align.py` | Reprojects raw depth into the RGB camera frame and publishes aligned depth. |
| `ros2_ws/src/perception/perception/yolo_segmentation_node.py` | Detects the four block classes and `blue_obstacle`; publishes raw 2D masks on `/yolo/detections`. |
| `ros2_ws/src/perception/perception/target_pose_estimator.py` | Fuses YOLO masks with aligned depth, camera intrinsics, and TF; publishes all 3D objects on legacy JSON `/detected_objects` and typed `/perception/detected_objects`. |
| `ros2_ws/src/perception/perception/target_selector.py` | Selects only a valid requested block and publishes `/current_target_object` and `/target_block_pose`. |
| `ros2_ws/src/perception/perception/beacon_heading_controller.py` | Computes heading error from `base_link` to `/target_block_pose` and publishes `/beacon_cmd`. |
| `ros2_ws/src/perception/perception/perception_visualizer.py` | Produces a compressed side-by-side YOLO plus aligned-depth display and map-frame RViz markers. It is diagnostic only and does not affect target selection. |

### Camera And Perception Launch

```bash
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source install/setup.bash
ros2 launch perception camera.launch.py
```

Default measured camera extrinsics:

| Transform | x | y | z | roll | pitch | yaw |
| --- | --- | --- | --- | --- | --- | --- |
| `base_link -> camera_link` | `0.19 m` | `0.0 m` | `-0.02 m` | `0.0 rad` | `0.262 rad` | `0.0 rad` |

These values can be overridden at launch:

```bash
ros2 launch perception camera.launch.py camera_x:=0.19 camera_y:=0.0 camera_z:=-0.02 camera_pitch:=0.262
```

### Perception Pipeline

```text
RealSense RGB + depth
  -> depth_align.py
  -> yolo_segmentation_node.py
  -> /yolo/detections
  -> target_pose_estimator.py
  -> /detected_objects (legacy JSON) + /perception/detected_objects (typed)
  -> perception_visualizer.py
  -> /perception/fused_debug/compressed + /perception/object_markers
  -> target_selector.py
  -> /current_target_object + /target_block_pose
  -> beacon_heading_controller.py
  -> /beacon_cmd

TF tree
  map -> odom -> base_link -> camera_link
    -> camera_color_frame -> camera_color_optical_frame
    -> camera_depth_frame -> camera_depth_optical_frame
```

### Perception Topics

| Topic | Type | Publisher | Description |
| --- | --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RealSense | RGB image input. |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | RealSense | Raw depth image input. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `depth_align.py` | Depth image aligned to RGB pixels. |
| `/yolo/detections` | `std_msgs/msg/String` | `yolo_segmentation_node.py` | Raw 2D YOLO masks, boxes, classes, and confidence. |
| `/yolo/debug_image` | `sensor_msgs/msg/Image` | `yolo_segmentation_node.py` | Optional debug image, enabled with `runtime_mode:=debug`. |
| `/yolo/debug_image/compressed` | `sensor_msgs/msg/CompressedImage` | `yolo_segmentation_node.py` | JPEG-compressed debug image for remote laptop visualization. |
| `/detected_objects` | `std_msgs/msg/String` | `target_pose_estimator.py` | All 3D objects, including `blue_obstacle`; contains depth and map/base coordinates where valid. |
| `/perception/detected_objects` | `imdl_msgs/msg/DetectedObjectArray` | `target_pose_estimator.py` | Shared typed object list for navigation and control; includes class, image coordinates, depth, base/map coordinates, and validity flags. |
| `/perception/fused_debug/compressed` | `sensor_msgs/msg/CompressedImage` | `perception_visualizer.py` | Side-by-side YOLO view and colorized aligned depth with per-object `Z` depth and base-frame `R` range. Debug mode only. |
| `/perception/object_markers` | `visualization_msgs/msg/MarkerArray` | `perception_visualizer.py` | Map-frame marker and distance label for every object with a valid `position_map`. |
| `/requested_target_class` | `std_msgs/msg/String` | Task controller | Requested block class: `red_block`, `yellow_block`, `green_block`, or `blue_block`. |
| `/current_target_object` | `std_msgs/msg/String` | `target_selector.py` | Selected target object, target request, stale state, and selection reason. |
| `/target_block_pose` | `geometry_msgs/msg/PoseStamped` | `target_selector.py` | Map-frame pose of the selected valid block only. |
| `/beacon_cmd` | `std_msgs/msg/Float32` | `beacon_heading_controller.py` | Heading error in radians. Positive means target left, negative means target right, near zero means target ahead. |

### Navigation and Control Handoff

New navigation and control consumers should subscribe to the typed
`/perception/detected_objects` topic, rather than the legacy JSON topic.

- [Navigation perception handoff](ros2_ws/src/perception/docs/navigation_perception_handoff.md)
- [Control perception handoff](ros2_ws/src/perception/docs/control_perception_handoff.md)
- [Complete typed-interface contract](ros2_ws/src/perception/docs/typed_detected_objects_interface.md)
- [Hardware-free B/C mock integration](ros2_ws/src/imdl_integration_tools/docs/bc_mock_integration.md)

Example `/detected_objects` payload:

```json
[
  {
    "class_name": "red_block",
    "is_block": true,
    "confidence": 0.98,
    "depth_m": 0.62,
    "position_map": {"x": 1.2, "y": 0.3, "z": -0.1},
    "transform_status": "ok"
  }
]
```

### Perception Checks

```bash
ros2 node list --no-daemon | grep -E "depth_align|yolo_segmentation|target_pose_estimator|target_selector|beacon_heading"
ros2 topic list --no-daemon | grep -E "detected|aligned_depth|target|beacon"
ros2 topic echo /yolo/detections std_msgs/msg/String --once
ros2 topic echo /detected_objects std_msgs/msg/String --once
ros2 topic echo /perception/detected_objects imdl_msgs/msg/DetectedObjectArray --once
ros2 topic echo /target_block_pose geometry_msgs/msg/PoseStamped --once
ros2 topic echo /beacon_cmd std_msgs/msg/Float32 --once
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

If ROS 2 CLI graph discovery behaves strangely, restart the daemon:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

On ROS 2 Humble, `--no-daemon` is placed after the subcommand, for example:

```bash
ros2 topic list --no-daemon
ros2 node list --no-daemon
```

### Beacon Command

`beacon_heading_controller.py` subscribes to `/target_block_pose`, looks up the
latest `target_frame -> base_link` transform, and publishes the angular error
on `/beacon_cmd`.

Interpretation:

| `/beacon_cmd` value | Meaning |
| --- | --- |
| Positive | Target is to the left of the robot heading. |
| Near zero | Target is approximately in front. |
| Negative | Target is to the right of the robot heading. |

Quick check:

```bash
ros2 topic echo /beacon_cmd std_msgs/msg/Float32
```

Move the colored target left and right in front of the camera. The command
should change sign around zero.

## Typical Startup Order

For localization plus perception:

```bash
# Terminal 1
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source install/setup.bash
ros2 launch slam localization.launch.py

# Terminal 2
cd ~/Desktop/intelligent-machine-design-laboratory/ros2_ws
source install/setup.bash
ros2 launch perception camera.launch.py
```

Then verify:

```bash
ros2 topic echo /robot_pose --once
ros2 topic echo /detected_objects std_msgs/msg/String --once
ros2 topic echo /target_block_pose geometry_msgs/msg/PoseStamped --once
ros2 topic echo /beacon_cmd std_msgs/msg/Float32 --once
```

## Interfaces Ready For Navigation Team

These are the main outputs provided by the SLAM and perception stack:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Saved SLAM map for localization and planning. |
| `/scan` | `sensor_msgs/msg/LaserScan` | LIDAR scan for localization and obstacle information. |
| `/robot_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Robot pose in the `map` frame. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Frame tree: `map -> odom -> base_link -> camera_link -> camera frames`. |
| `/detected_objects` | `std_msgs/msg/String` | Legacy JSON object list, retained while consumers migrate. |
| `/perception/detected_objects` | `imdl_msgs/msg/DetectedObjectArray` | Shared typed object list for navigation and control consumers. |
| `/current_target_object` | `std_msgs/msg/String` | Selected requested block and its status. |
| `/target_block_pose` | `geometry_msgs/msg/PoseStamped` | Main target position in the `map` frame. |
| `/beacon_cmd` | `std_msgs/msg/Float32` | Heading command for beacon alignment or target following. |

Useful interface check:

```bash
ros2 topic list --no-daemon | grep -E "^/map$|^/scan$|^/robot_pose$|^/yolo/detections$|^/target_block_pose$|^/detected_objects$|^/current_target_object$|^/beacon_cmd$"
```

## Notes

- `target_pose_estimator.py` uses the latest available TF when transforming
  camera points to the target frame. This avoids failures caused by RealSense
  image timestamps being slightly ahead of the TF buffer.
- `/detected_objects` remains the legacy `std_msgs/msg/String` JSON interface.
  New navigation and control consumers should use
  `/perception/detected_objects` (`imdl_msgs/msg/DetectedObjectArray`);
  see `ros2_ws/src/perception/docs/typed_detected_objects_interface.md`.
- The frozen YOLO model has five classes: four block classes and
  `blue_obstacle`. `blue_obstacle` is published for navigation but is never a
  selectable grasp target.
