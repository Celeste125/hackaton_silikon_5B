---
tags: [IMDL, architecture, team, milestones, english]
created: 2026-04-29
updated: 2026-05-02
---
# I.MDL System Architecture & Team Plan

> **Course**: Intelligent Machine Design Lab — TUM MIRMI
> **Duration**: 2026-04-15 → 2026-07-15 (weekly meetings every Wednesday)
> **Lecturers**: Dr. Mahathi Anand · Dr.-Ing. Lennart Trösken
> **Team**: 3 members

---

## Table of Contents

1. [[#Project Goal]]
2. [[#System Architecture]]
   - [[#Functional Pipeline]]
   - [[#Hardware Layer]]
   - [[#Software Layer ROS2 Node Graph]]
   - [[#Task Execution Data Flow]]
   - [[#TF Frame Tree]]
   - [[#ROS2 Workspace Structure]]
   - [[#Global Topic Action Interface Table]]
3. [[#Module Planning]]
   - [[#M1 — SLAM & Localization]]
   - [[#M2 — Perception & Object Detection]]
   - [[#M3 — Path Planning & Navigation]]
   - [[#M4 — Task Sequencer]]
   - [[#M5 — Manipulator Design & Control]]
   - [[#M6 — System Integration & Infrastructure]]
4. [[#Team Assignment]]
   - [[#Member A — Perception & Localization Lead]]
   - [[#Member B — Navigation Task Sequencing & Project Management Lead]]
   - [[#Member C — Manipulator & Embedded Systems Lead]]
5. [[#Milestone Plan]]
   - [[#Timeline Overview]]
   - [[#Milestone 1 — Localization & Mapping]]
   - [[#Milestone 2 — Planning & Navigation]]
   - [[#Milestone 3 + Final Evaluation — Gripper Design & Trajectory Planning]]
   - [[#Risks & Mitigations]]
6. [[#Grading Structure]]

---

## Project Goal

Design and build an **autonomous mobile robot with search-and-rescue capabilities** (per Introduction-1.pdf):

- Identify colored wooden blocks (4×4 cm "casualties") and classify their color
- Navigate autonomously in the arena, avoiding obstacles
- Use a 2-DoF robot arm to pick up blocks and transport them to a designated drop zone
- Drop the cube in a selected box (A or B)
- Complete the full task pipeline: `Navigate → Pick → Navigate → Place → Return`

---

## System Architecture

### Functional Pipeline

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Functional Pipeline                          │
│                                                                      │
│  ┌──────────┐      ┌──────────┐                                     │
│  │ Sensors  │ ───▶ │Perception│                                     │
│  └──────────┘      └────┬─────┘                                     │
│                         │                                           │
│             ┌───────────┴───────────┐                               │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │ Localization │        │    Object    │                       │
│      │              │        │  Detection   │                       │
│      └──────┬───────┘        └──────┬───────┘                       │
│             │                       │                               │
│             └───────────┬───────────┘                               │
│                         ▼                                           │
│                  ┌──────────────┐                                   │
│                  │   Planning   │                                   │
│                  └──────┬───────┘                                   │
│                         │                                           │
│             ┌───────────┴───────────┐                               │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │ Manipulator  │        │Mobile Platform│                      │
│      │ Traj/Pick    │        │Path/Obstacle  │                      │
│      │ and Place    │        │Avoidance      │                      │
│      └──────┬───────┘        └──────┬───────┘                       │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │ Manipulator  │        │Mobile Platform│                      │
│      │ Motion Ctrl  │        │ Motion Ctrl   │                      │
│      └──────┬───────┘        └──────┬───────┘                       │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │Actuator:     │        │Actuator:     │                       │
│      │Servos        │        │Wheels        │                       │
│      └──────────────┘        └──────┬───────┘                       │
│                                      ▼                               │
│                    Get robot position; feedback to localization      │
└──────────────────────────────────────────────────────────────────────┘
```

| PDF objective | Block in pipeline |
|---------------|-------------------|
| Map the arena (SLAM) | Localization |
| Detect block color + light beacon | Object Detection + Actuators (beacon) |
| Path planning + collision avoidance | Planning (mobile) + Motion Control |
| Task sequencing (nav→pick→nav→place→return) | Planning (top-level state machine, M4) |
| 2-DoF arm IK + trajectory | Planning (manipulator) + Motion Control |
| Pick / place / drop in box A or B | Actuators (servos + gripper) |

---

### Hardware Layer

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Robot Hardware                               │
│                                                                      │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                  Jetson Orin Nano (SBC)                      │   │
│   │              Ubuntu 22.04  ·  ROS2 Humble                   │   │
│   │        Main compute · WiFi comm · All ROS2 nodes run here   │   │
│   └───────────────┬──────────────────────┬───────────────────────┘  │
│                   │ Ethernet / USB        │ USB (micro-ROS)          │
│                   ▼                       ▼                          │
│   ┌───────────────────────┐   ┌───────────────────────┐            │
│   │    iRobot Create3     │   │    Arduino Uno         │            │
│   │    Mobile Base        │   │  (Low-level Ctrl)      │            │
│   │  · Built-in ROS2      │   │  · Arm servos (2×)    │            │
│   │  · Optical odometry   │   │  · Gripper actuator   │            │
│   │  · 7× IR obstacle     │   │  · Light beacon servo │            │
│   │  · Cliff sensors      │   │  · Power LED on/off   │            │
│   │  · Docking sensors    │   └───────────────────────┘            │
│   │  · Multi-zone bumper  │   (Optional alt: STM32 /              │
│   │  · WiFi/BT/Ethernet   │    SAME54 Xplained Pro)               │
│   └───────────────────────┘                                         │
│                                                                      │
│   ┌──────────────────────┐   ┌──────────────────────────────────┐  │
│   │  SLAMTEC C1 LIDAR    │   │  Intel RealSense Camera          │  │
│   │  2D Laser Scanner    │   │  RGB-D Depth Camera              │  │
│   │  → /scan (LaserScan) │   │  → /color/image, /depth/image    │  │
│   └──────────────────────┘   └──────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

| Component | Model | Role |
|-----------|-------|------|
| Mobile base | iRobot Create3 | Motion, **optical odometry**, IR/cliff/docking sensors, multi-zone bumper |
| Main compute | NVIDIA Jetson Orin Nano | ROS2 node host, WiFi communication |
| Low-level controller | Arduino Uno (optional alt: STM32 / SAME54 Xplained Pro) | Servo / gripper / light beacon PWM |
| Laser scanner | SLAMTEC C1 Fusion | LIDAR SLAM, obstacle detection |
| Depth camera | Intel RealSense camera (exact model TBD) | Color recognition, depth, target localization |
| Robot arm | 2-DoF DIY (3D printed) | Pick and place 4×4 cm wooden blocks |
| Light beacon | Power LED + 180° pan servo | Illuminate target for color detection; supports **rotation + ON/OFF** |

**Localization sensor choice**

The PDF allows three localization options: (a) IR localization from Create3, (b) Visual SLAM via RealSense, (c) LIDAR SLAM via SLAMTEC C1. We adopt **LIDAR SLAM as primary** (mature `slam_toolbox` stack, robust geometry-based loop closure). RealSense is reserved for **object detection (RGB-D)**, with visual SLAM as a fallback if LIDAR fails. Create3 IR sensors feed Nav2 local costmap as auxiliary obstacle data.

---

### Software Layer ROS2 Node Graph

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ROS2 Node Graph                                 │
│                                                                         │
│  ┌──────────────┐  /scan      ┌────────────────────────────────────┐   │
│  │ lidar_driver │ ──────────▶ │          SLAM Module               │   │
│  └──────────────┘             │    (slam_toolbox)                  │   │
│                               │  publishes: /map                   │   │
│  ┌──────────────┐  /odom      │             /tf (odom→map)         │   │
│  │ irobot_node  │ ──────────▶ │             /robot_pose            │   │
│  └──────────────┘             └───────────────────┬────────────────┘   │
│                                                   │                    │
│  ┌──────────────┐  /color_img ┌───────────────────▼────────────────┐  │
│  │ realsense_   │ ──────────▶ │       Perception Module             │  │
│  │ node         │  /depth_img │  (object_detection_node)            │  │
│  └──────────────┘ ──────────▶│  publishes: /detected_objects       │  │
│                               │             /target_block_pose      │  │
│  ┌──────────────┐             │             /beacon_cmd             │  │
│  │arduino_bridge│◀─ /beacon_cmd, /beacon_light                     │  │
│  │ (micro-ROS)  │                                 ▼                   │
│  └──────┬───────┘             ┌────────────────────────────────────┐  │
│         │ /joint_cmd          │       Planning Module               │  │
│         │ /gripper_cmd        │  task_sequencer_node (State Machine)│  │
│         │ /beacon_cmd         │  nav2_stack (Path Planner + DWA)   │  │
│         │ /beacon_light       │  publishes: /cmd_vel, /goal_pose    │  │
│  iRobot ◀─── /cmd_vel ──────  │             /pick_place_action      │  │
│  Create3                      └───────────────────┬────────────────┘  │
│                                                   │                    │
│                               ┌───────────────────▼────────────────┐  │
│                               │      Manipulator Module             │  │
│  Arduino ◀─── /joint_cmd ──── │  arm_controller_node (IK + Traj)   │  │
│      ──────▶ /joint_state_fb  │  pick_place_action server (C)       │  │
│                               │  gripper_controller_node            │  │
│                               └────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

### Task Execution Data Flow

```
START
  │
  ▼
[IDLE] ──── receive task command (target_color, target_box ∈ {A, B})
  │
  ▼
[SCAN ARENA] ── LIDAR mapping; camera for object detection/aux perception ──▶ /map updated
  │
  ▼
[NAVIGATE TO BLOCK] ── Nav2 path planning ──▶ arrive near target block
  │
  ▼
[DETECT & ALIGN] ── beacon ON + beacon pan to target
                  ── color confirm + precise alignment ──▶ /target_block_pose
  │
  ▼
[PICK] ── arm IK ──▶ Arduino servos ──▶ gripper close
       ── beacon OFF
  │
  ▼
[NAVIGATE TO DROP] ── select drop pose by target_box (A or B)
                   ── Nav2 ──▶ arrive at drop zone
  │
  ▼
[PLACE] ── arm trajectory ──▶ gripper open ──▶ block placed
  │
  ▼
[RETURN HOME] ── Nav2 ──▶ back to origin
  │
  ▼
END / NEXT TASK
```

---

### TF Frame Tree

```
map
 └── odom
      └── base_link              ← iRobot center
           ├── lidar_link
           ├── camera_link
           │    ├── camera_color_optical_frame
           │    └── camera_depth_optical_frame
           ├── arm_base_link
           │    ├── arm_link1    ← shoulder joint
           │    ├── arm_link2    ← elbow joint
           │    └── gripper_link
           └── beacon_link
```

---

### ROS2 Workspace Structure

```
ros2_ws/
└── src/
    ├── imdl_msgs/           ← Custom message types (shared, owned by B)
    ├── imdl_bringup/        ← Launch files, parameter configs (B)
    ├── imdl_slam/           ← SLAM node (Member A)
    ├── imdl_perception/     ← Perception node (Member A)
    ├── imdl_navigation/     ← Navigation + task sequencer (Member B)
    ├── imdl_manipulation/   ← Arm controller (Member C)
    └── arduino_firmware/    ← Arduino code (Member C)
```

---

### Global Topic Action Interface Table

| Topic / Action          | Message Type                           | Publisher                    | Subscriber            |
| ----------------------- | -------------------------------------- | ---------------------------- | --------------------- |
| `/map`                  | OccupancyGrid                          | A (slam)                     | B (nav2)              |
| `/robot_pose`           | PoseWithCovarianceStamped              | A (slam)                     | B                     |
| `/scan`                 | LaserScan                              | A (lidar driver)             | A (slam), B (costmap) |
| `/odom`                 | Odometry (optical)                     | iRobot Create3               | A (slam)              |
| `/detected_objects`     | DetectedObject[]                       | A (perception)               | B                     |
| `/target_block_pose`    | PoseStamped (frame_id: map)            | A (perception)               | B, C                  |
| `/beacon_cmd`           | Float32 (pan angle, rad)               | A (perception)               | C (arduino bridge)    |
| `/beacon_light`         | Bool (LED on/off)                      | B (task seq, sole publisher) | C (arduino bridge)    |
| `/goal_pose`            | PoseStamped                            | B (task seq)                 | B (nav2)              |
| `/cmd_vel`              | Twist                                  | B (nav2)                     | iRobot Create3        |
| `/pick_place_action`    | Action (goal: PICK/PLACE + target_box) | B (client)                   | C (server)            |
| `/joint_cmd`            | JointState                             | C (arm ctrl)                 | C (arduino bridge)    |
| `/joint_state_feedback` | JointState                             | C (arduino)                  | C (arm ctrl), B       |
| `/gripper_cmd`          | Bool                                   | C (arm ctrl)                 | C (arduino bridge)    |
| `/gripper_status`       | Bool                                   | C (arduino)                  | B                     |

---

## Module Planning

| Module | Name | Owner |
|--------|------|-------|
| M1 | SLAM & Localization | Member A |
| M2 | Perception & Object Detection | Member A |
| M3 | Path Planning & Navigation | Member B |
| M4 | Task Sequencer | Member B |
| M5 | Manipulator Design & Control | Member C |
| M6 | System Integration & Infrastructure | Member B (coordinates all) |

---

### M1 — SLAM & Localization

**Goal**: Build a map of the unknown arena and output real-time robot pose.

**I/O**

| Direction | Topic | Type |
|-----------|-------|------|
| Input | `/scan` | LaserScan |
| Input | `/odom` | Odometry (optical) |
| Output | `/map` | OccupancyGrid |
| Output | `/robot_pose` | PoseWithCovarianceStamped |
| Output | `/tf` (odom → map) | TF2 |

**Algorithm Selection**

| Option | Pros | Cons | Recommendation |
|--------|------|------|----------------|
| slam_toolbox | Mature, lifelong mapping | Many parameters | ✅ First choice |
| cartographer | High accuracy (Google) | Complex setup | Backup |
| gmapping | Simple | Outdated | ❌ |

**Key Configuration**

```yaml
# slam_toolbox online_async mode
resolution: 0.05
max_laser_range: 12.0
minimum_travel_distance: 0.1
minimum_travel_heading: 0.1
map_update_interval: 5.0
```

**Acceptance Criteria**
- [ ] Map shows no obvious tears after a full lap of the arena
- [ ] Localization error < 10 cm (compared to tape measurement)
- [ ] Supports localization-only mode (pure localization on a known map)

---

### M2 — Perception & Object Detection

**Goal**: Identify block colors and output their 3D position in world coordinates.

**I/O**

| Direction | Topic | Type |
|-----------|-------|------|
| Input | `/camera/color/image_raw` | sensor_msgs/Image |
| Input | `/camera/depth/image_rect_raw` | sensor_msgs/Image |
| Output | `/detected_objects` | DetectedObject[] (custom) |
| Output | `/target_block_pose` | geometry_msgs/PoseStamped (map frame) |
| Output | `/beacon_cmd` | std_msgs/Float32 |

**Custom Message `DetectedObject.msg`**

```
string  color            # "red" / "yellow" / "green" / "blue" / "purple"
geometry_msgs/Pose pose  # Position in map frame
float32 confidence       # Detection confidence 0.0 ~ 1.0
```

**Color Detection Pipeline**

```
RGB image → HSV conversion → Color threshold mask → Morphological filter
          → Contour detection → Min bounding rect (size filter)
          → Color label → Corresponding depth pixel → 3D point (camera_frame)
          → TF transform → 3D point (map_frame) → publish
```

**HSV Color Ranges (calibrate on-site)**

| Color | H_min | H_max | S_min | V_min |
|-------|-------|-------|-------|-------|
| Red | 0 / 160 | 10 / 180 | 100 | 100 |
| Yellow | 20 | 35 | 100 | 100 |
| Green | 40 | 80 | 100 | 100 |
| Blue | 100 | 130 | 100 | 100 |
| Purple | 130 | 155 | 60 | 60 |

**Light Beacon Control Logic**

```python
dx = target.x - robot.x
dy = target.y - robot.y
angle_in_world = atan2(dy, dx)
angle_relative = angle_in_world - robot_yaw
beacon_angle = clamp(angle_relative, -90°, 90°)
# /beacon_light is published by B's task state machine:
# True during ALIGN/DETECT, False otherwise
```

**Acceptance Criteria**
- [ ] Detect 5 colors with accuracy > 90% (stationary)
- [ ] 3D position output accuracy < 5 cm
- [ ] Light beacon dynamically tracks target while robot moves; task state machine turns LED ON during align and OFF otherwise

---

### M3 — Path Planning & Navigation

**Goal**: Plan a collision-free path from current position to goal and execute it.

**I/O**

| Direction | Topic | Type |
|-----------|-------|------|
| Input | `/map` | OccupancyGrid |
| Input | `/robot_pose` | PoseWithCovarianceStamped |
| Input | `/goal_pose` | PoseStamped |
| Output | `/cmd_vel` | geometry_msgs/Twist |

**Nav2 Components**

| Component | Function |
|-----------|----------|
| BT Navigator | Behavior tree top-level coordinator |
| Planner Server | Global path planning (NavFn / A*) |
| Controller Server | Local control (DWA) |
| Global Costmap | Static map + inflation layer |
| Local Costmap | Real-time obstacles (LIDAR + Create3 IR) |
| Recovery Behaviors | Auto-recovery when stuck (spin, back up) |

**Key Configuration**

```yaml
global_costmap:
  robot_radius: 0.185
  inflation_radius: 0.30
  obstacle_range: 3.0
  raytrace_range: 5.0

local_costmap:
  width: 3.0
  height: 3.0
  resolution: 0.05
  update_frequency: 10.0

controller_server:
  FollowPath:
    max_vel_x: 0.26
    max_vel_theta: 1.82
    min_vel_x: 0.0
```

**Acceptance Criteria**
- [ ] A → B navigation success rate > 95% (10 trials)
- [ ] Re-plans in real time when a dynamic obstacle is inserted
- [ ] Stopping error at goal < 3 cm

---

### M4 — Task Sequencer

**Goal**: Coordinate all modules to complete the full search-and-rescue pipeline.

**State Machine Design**

```
IDLE
 │
 ▼
NAVIGATE_TO_SCAN ──▶ (mapping phase)
 │
 ▼
NAVIGATE_TO_BLOCK ◀── target_color / target_box
 │
 ▼
ALIGN_TO_BLOCK  (beacon ON; precise alignment < 3 cm)
 │
 ▼
PICK_BLOCK ──▶ /pick_place_action (PICK) ──▶ wait for result
            ──▶ beacon OFF on success
 │
 ▼
NAVIGATE_TO_DROP (drop pose chosen by target_box ∈ {A, B})
 │
 ▼
PLACE_BLOCK ──▶ /pick_place_action (PLACE, target_box) ──▶ wait for result
 │
 ▼
RETURN_HOME ──▶ IDLE / NEXT_TASK
```

**Implementation**

```python
class TaskState(Enum):
    IDLE = 0
    NAVIGATE_TO_SCAN = 1
    NAVIGATE_TO_BLOCK = 2
    ALIGN_TO_BLOCK = 3
    PICK_BLOCK = 4
    NAVIGATE_TO_DROP = 5
    PLACE_BLOCK = 6
    RETURN_HOME = 7
```

**Acceptance Criteria**
- [ ] State transitions are correct with no deadlocks
- [ ] Error recovery on any step failure (retry / skip / alert)
- [ ] Full pipeline completes in < 3 minutes
- [ ] Correctly routes block to box A or B based on `target_box`

---

### M5 — Manipulator Design & Control

**Goal**: Design, manufacture, and control a 2-DoF robot arm to pick and place blocks.

**Mechanical Specifications**

> The following dimensions and joint ranges are initial team design assumptions, not PDF-given requirements; they will be adjusted based on servo torque, chassis mounting space, gripper size, and measured grasping performance on 4×4 cm blocks.

```
Joint 1 (Shoulder) — rotation about Z  Range: -90° ~ +90°
Joint 2 (Elbow)    — rotation about Y  Range:   0° ~ +135°
Gripper            — parallel jaw      Opening: 0 ~ 60 mm

L1 (upper arm) = 150 mm
L2 (forearm)   = 120 mm
Mount height   =  80 mm  (base surface → Joint 1)
```

**Gripper Orientation Constraint**

The PDF states that the gripper should be able to orient itself in front of an object. In this plan, gripper orientation is satisfied jointly by coarse alignment of the mobile base and the 2-DoF arm end-effector position: the base first faces the target, then the arm IK moves the gripper in front of the object. If testing shows this is insufficient, an additional gripper servo or mechanical orientation adjustment will be added.

**Inverse Kinematics (2D planar analytical solution)**

$$r = \sqrt{x^2 + y^2}$$

$$\cos\theta_2 = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

$$\theta_2 = \text{atan2}\!\left(\pm\sqrt{1-\cos^2\theta_2},\ \cos\theta_2\right)$$

$$\theta_1 = \text{atan2}(y,\ x) - \text{atan2}\!\left(L_2 \sin\theta_2,\ L_1 + L_2\cos\theta_2\right)$$

**Arduino Firmware Interface (micro-ROS)**

```
Subscribe:
  /joint_cmd     → JointState → servo PWM
  /gripper_cmd   → Bool       → gripper open / close
  /beacon_cmd    → Float32    → beacon pan angle
  /beacon_light  → Bool       → LED on/off

Publish:
  /joint_state_feedback → JointState
  /gripper_status       → Bool
```

**Acceptance Criteria**
- [ ] micro-ROS communication stable (no disconnects)
- [ ] IK accuracy: end-effector error < 8 mm
- [ ] Grasp success rate > 85% (20 trials)
- [ ] No collision between arm and chassis throughout workspace

---

### M6 — System Integration & Infrastructure

**Launch File Structure**

```
imdl_bringup/launch/
  ├── robot.launch.py        ← Full robot (production)
  ├── slam.launch.py         ← SLAM debug only
  ├── navigation.launch.py   ← Navigation debug only
  ├── perception.launch.py   ← Perception debug only
  ├── arm.launch.py          ← Arm debug only
  └── simulation.launch.py   ← Gazebo simulation
```

**Git Branch Convention**

```
main                 ← merge only tested, stable code
dev                  ← integration branch
feature/slam         ← Member A
feature/perception   ← Member A
feature/navigation   ← Member B
feature/manipulation ← Member C
```

**Optional Bonus Tasks**

| Task | Description | Bonus | Suggested Owner |
|------|-------------|-------|-----------------|
| Multi-sensor fusion | Combine LIDAR + Camera for perception & planning | +5% | A |
| Path optimization | Minimize time and/or energy consumption | +5% | B |

---

## Team Assignment

| Member       | Role                                                  | Core Modules                                                              |
| ------------ | ----------------------------------------------------- | ------------------------------------------------------------------------- |
| **Member A** | Perception & Localization Lead                        | Module M1 SLAM + Module M2 Perception                                     |
| **Member B** | Navigation, Task Sequencing & Project Management Lead | Module M3 Path Planning & Navigation + M4 Task Sequencer + M6 Integration |
| **Member C** | Manipulator & Embedded Systems Lead                   | Module M5 Manipulator + Arduino full stack                                |

**Workload Phasing**

| Phase | Member A | Member B | Member C |
|-------|----------|----------|----------|
| W1–5 (Milestone 1) | Full SLAM stack + sensor calibration | Nav2 baseline + Git/PM + Gazebo + integration checklist | Arduino firmware + micro-ROS + servo + mounts/power wiring |
| W6–10 (Milestone 2) | Perception 3D + beacon bearing control | Task state machine + nav-perception integration | CAD finalize + 3D print + IK sim + IK ROS2 node |
| W11–14 (Milestone 3/Final) | Localization tuning + RViz markers | Full system integration + debug + demo orchestration | Arm mount + TF calibration + grasp tuning |

---

### Member A — Perception & Localization Lead

**Core Responsibilities**
- Configure Jetson Orin Nano (Ubuntu 22.04 + ROS2 Humble)
- Update iRobot Create3 firmware, establish ROS2 communication with Jetson
- Integrate SLAMTEC C1 LIDAR driver, publish `/scan`
- Configure slam_toolbox; publish `/map`, `/robot_pose`, `/tf`
- Extrinsic calibration between LIDAR and chassis frame
- Define sensor mounting positions and calibration references (mounts/wiring implemented by C)
- Integrate Intel RealSense camera driver (RGB + depth via ROS2; exact model TBD)
- HSV-based wooden block color recognition (calibrate thresholds on-site)
- Depth fusion: pixel coordinates → 3D world coordinates
- Light beacon **bearing control logic** (compute bearing → publish `/beacon_cmd`; LED on/off is published as `/beacon_light` by B's task state machine; hardware by C)
- RViz2 MarkerArray visualization of detection results

**Tech Stack**: ROS2 slam_toolbox · SLAMTEC C1 driver · iRobot Create3 SDK · Python OpenCV · realsense2_camera · TF2 · RViz2

**Task List**

| Week | Task | Deadline |
|------|------|----------|
| W1–2 | Flash Jetson, install Ubuntu 22.04 + ROS2 | 04/29 |
| W1–2 | Update Create3 firmware, verify `/odom` | 04/29 |
| W3–4 | Install LIDAR driver, visualize `/scan` | 05/06 |
| W4–5 | Configure slam_toolbox, manual mapping run | 05/13 |
| **M1** | **SLAM mapping demo, error < 10 cm** | **05/13** |
| W6–7 | RealSense driver, RGB+depth in RViz; HSV calibration | 05/27 |
| W7–8 | Color detection node (label + image coordinates) | 06/03 |
| W8–9 | Depth fusion → 3D `/target_block_pose` in map frame | 06/10 |
| W9–10 | Light beacon bearing computation node | 06/17 |
| W6–10 | Localization-only mode (AMCL on known map) in parallel | 06/17 |
| **M2** | **Stable localization + 3D detection < 5 cm + beacon tracking** | **06/17** |
| W11–13 | Provide map-frame target poses to C; help validate map→arm_base_link TF; RViz markers; tuning | 07/08 |
| W11–13 | (Optional bonus) Multi-sensor fusion: LIDAR + camera | 07/08 |
| **Final** | **No localization drift + robust detection under varying lighting** | **07/15** |

**Interfaces**

| Direction | Content |
|-----------|---------|
| Provides to B | `/map`, `/robot_pose`, `/tf`, `/detected_objects`, `/target_block_pose` |
| Provides to C | `/target_block_pose` (map-frame target pose), `/beacon_cmd` |
| Depends on B | Git repo structure, `imdl_msgs` package |
| Depends on C | Light beacon Arduino firmware ready |

---

### Member B — Navigation Task Sequencing & Project Management Lead

**Core Responsibilities**
- Full Nav2 stack configuration (global planner + DWA + costmap + recovery)
- Python task state machine (M4)
- Sole publisher of `/beacon_light`: controls LED on/off by task state
- Dynamic obstacle avoidance
- Integration with perception: detected target → auto-navigate to it
- Gazebo simulation arena
- Git repository management · `imdl_msgs` package · launch files (`imdl_bringup`)
- M1 integration checklist: sensor mounts, power, data connections, launch startup order
- **Project Management**: weekly Project Tracking Chart, Wednesday meeting host, GitHub Issues

**Tech Stack**: ROS2 Nav2 · Python state machine · BehaviorTree.CPP (optional) · Gazebo · Git

**Task List**

| Week | Task | Deadline |
|------|------|----------|
| **W1** | **Create Git repo, workspace skeleton, branch convention** | **04/22** |
| W1–2 | Build test arena in Gazebo | 04/29 |
| W1–2 | Create `imdl_msgs` package, define all custom messages | 04/29 |
| W2–4 | Nav2 stack baseline configuration | 05/06 |
| W4–5 | Verify A→B navigation in simulation; physical robot baseline demo (stretch) | 05/13 |
| **M1** | **Nav2 baseline runs; physical A→B navigation >80% is a stretch goal** | **05/13** |
| W6–8 | Design and implement Python task state machine | 06/03 |
| W6–8 | `/pick_place_action` protocol definition (incl. target_box) | 06/03 |
| W8–10 | Perception-navigation integration: detect target → auto navigate | 06/17 |
| **M2** | **Full A→B→C→A dry-run sequence (no arm yet)** | **06/17** |
| W11–12 | Joint test with C: full pick-place pipeline | 07/01 |
| W13–14 | Full system integration testing, in-field challenge prep | 07/13 |
| W11–14 | (Optional bonus) Path/trajectory optimization | 07/13 |
| **Final** | **Full demo (navigate→pick→navigate→place→return) + final presentation** | **07/15** |

**Project Management (weekly routine)**
- [ ] Update Project Tracking Chart before every Wednesday meeting
- [ ] Maintain GitHub Issues (each task → issue)
- [ ] Integration dry-run 2 days before each milestone

**Interfaces**

| Direction | Content |
|-----------|---------|
| Depends on A | `/map`, `/robot_pose`, `/detected_objects`, `/target_block_pose` |
| Sends to C | `/pick_place_action` (ROS2 Action, with target_box), `/beacon_light` |
| Depends on C | `/gripper_status`, `/joint_state_feedback` |
| Provides to all | `imdl_msgs` package, launch files, repo structure |

---

### Member C — Manipulator & Embedded Systems Lead

**Core Responsibilities**
- 2-DoF arm + gripper CAD design (Solidworks / FreeCAD)
- 3D printing, assembly, mounting on iRobot chassis
- Sensor/peripheral mounts, power wiring, and data cable fixation (with A for calibration)
- Arduino Uno firmware: servo control, gripper drive, light beacon PWM + LED on/off
- Jetson ↔ Arduino communication bridge (micro-ROS)
- MATLAB / Simulink IK simulation and verification
- ROS2-side IK solver node: PoseStamped in `arm_base_link` frame → JointState
- `map` → `arm_base_link` TF transform and validation (based on C's arm mounting calibration)
- `pick_place_action` server implementation (handles PICK / PLACE goals + target_box)
- Grasp success rate testing and tuning

**Tech Stack**: Solidworks / FreeCAD · 3D Printer · Arduino IDE · micro-ROS · MATLAB Simulink · Python ikpy

**Task List**

| Week | Task | Deadline |
|------|------|----------|
| W1–2 | micro-ROS running on Arduino, Jetson receives heartbeat | 04/29 |
| W1–2 | Sensor/peripheral mount plan + power/data connection checklist | 04/29 |
| W2–3 | Single servo control verified via ROS2 topic | 05/06 |
| W3–4 | 2-DoF arm + gripper CAD first draft (parametric) | 05/06 |
| W4–5 | Arduino firmware: 2 servos + gripper + beacon (rotate + ON/OFF) unified | 05/13 |
| **M1** | **Full firmware complete, all actuators ROS2-controllable** | **05/13** |
| W6–8 | Finalize CAD, 3D print all parts, sub-assemble | 06/03 |
| W8–9 | MATLAB IK simulation: target position → joint angles | 06/10 |
| W9–10 | ROS2 IK node: arm_base_link-frame PoseStamped → JointState | 06/17 |
| **M2** | **CAD finalized, IK simulation verified (sim error < 5 mm)** | **06/17** |
| W11–12 | Mount arm on chassis, full TF calibration | 07/01 |
| W11–12 | `pick_place_action` server (PICK / PLACE goals + target_box) | 07/01 |
| W12–13 | Grasp success rate testing + tuning (target > 85%) | 07/08 |
| W13–14 | Final integration with B, redundancy testing | 07/13 |
| **M3 + Final** | **Physical arm pick-place demo, reliable 4×4 block handling + final presentation** | **07/15** |

**Interfaces**

| Direction | Content |
|-----------|---------|
| Depends on A | `/target_block_pose` (map-frame target pose), `/beacon_cmd` |
| Depends on B | `/pick_place_action` command (with target_box), `/beacon_light` |
| Provides to B | `/gripper_status`, `/joint_state_feedback` |
| Provides to A | Light beacon hardware interface (PWM-driven 180° servo + LED on/off) |

---

## Milestone Plan

### Timeline Overview

```
2026-04-15  Course begins
     │
     ├── Week 1-2   Environment setup · hardware exploration · Git init
     ├── Week 3-5   SLAM · basic navigation · Arduino firmware
     │
2026-05-13  ★ MILESTONE 1 — Localization & Mapping
     │
     ├── Week 6-8   Localization tuning · Nav2 tuning · 3D detection · CAD finalize
     ├── Week 9-10  Task sequencing · perception+nav integration · IK simulation
     │
2026-06-17  ★ MILESTONE 2 — Planning & Navigation
     │
     ├── Week 11-12 Arm assembly · mount on chassis · pick_place_action server
     ├── Week 13-14 Full system integration · grasp tuning · Final prep
     │
2026-07-15  ★ MILESTONE 3 (Gripper + Trajectory) + FINAL EVALUATION
```

---

### Milestone 1 — Localization & Mapping

**Date**: 2026-05-13 (Week 5)

| Member | Deliverable                                                | Acceptance Criteria                                  |
| ------ | ---------------------------------------------------------- | ---------------------------------------------------- |
| A      | Full SLAM mapping                                          | Error < 10 cm, no map tears                          |
| B      | Nav2 baseline + A→B basic navigation stretch demo          | Simulation runs; physical A→B >80% is a stretch goal |
| C      | Arduino firmware (servos + gripper + beacon rotate/on-off) | All actuators controllable via ROS2 topic            |

**Day-of Checklist**
- [ ] SLAM map updates in real time in RViz (A)
- [ ] Nav2 baseline runs; physical A→B live navigation is stretch (B)
- [ ] Arduino servo + gripper + beacon (rotate + ON/OFF) demo via ROS2 topic (C)
- [ ] 5-minute presentation + demo

---

### Milestone 2 — Planning & Navigation

**Date**: 2026-06-17 (Week 10)

| Member | Deliverable | Acceptance Criteria |
|--------|-------------|---------------------|
| A | Stable localization + 3D target pose + beacon tracking | AMCL stable on known map (error < 8 cm); 3D accuracy < 5 cm; beacon follows target with LED ON during align |
| B | Full task sequence dry-run | A→B→C→A completes in one go (no arm yet); target_box routing works |
| C | Final CAD + IK simulation + IK ROS2 node | MATLAB IK sim error < 5 mm; ROS2 node accepts arm_base_link-frame PoseStamped → JointState |

**Day-of Checklist**
- [ ] Robot auto-navigates to a specified color block (A+B)
- [ ] Light beacon tracks target in real time + LED on/off (A+C)
- [ ] A→B→C→A task sequence full demo (B)
- [ ] MATLAB IK simulation results presentation (C)
- [ ] 5-minute presentation + demo + innovation discussion

---

### Milestone 3 + Final Evaluation — Gripper Design & Trajectory Planning

**Date**: 2026-07-15 (Week 14)

| Member | Deliverable | Acceptance Criteria |
|--------|-------------|---------------------|
| A | Stable localization + vision-guided grasping | No localization loss during full task; real-time precise grasp coordinates |
| B | Complete task pipeline integration | navigate→pick→navigate→place→return completes in one run; correct routing to box A or B |
| C | Physical arm grasp demo | Success rate > 85% (20 trials); IK accuracy < 8 mm end-effector error |

**Final Evaluation Checklist**
- [ ] Full search-and-rescue demo (one successful run, including drop into selected box A or B)
- [ ] 10-minute final presentation: concept, system design, technical details, algorithm reasoning
- [ ] Complete technical documentation
- [ ] Self-evaluation form submitted (each member)
- [ ] Project Tracking Chart final version
