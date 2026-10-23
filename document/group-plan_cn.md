---
tags: [IMDL, architecture, team, milestones]
created: 2026-04-29
updated: 2026-05-02
---
# I.MDL 系统架构与分工方案

> **课程**：Intelligent Machine Design Lab — TUM MIRMI
> **周期**：2026-04-15 → 2026-07-15（每周三例会）
> **讲师**：Dr. Mahathi Anand · Dr.-Ing. Lennart Trösken
> **团队**：3 人

---

## 目录

1. [[#项目目标]]
2. [[#系统架构]]
   - [[#功能流水线]]
   - [[#硬件层]]
   - [[#软件层 ROS2 节点图]]
   - [[#任务执行数据流]]
   - [[#TF 坐标系树]]
   - [[#ROS2 Workspace 结构]]
   - [[#全局 Topic Action 接口表]]
3. [[#模块规划]]
   - [[#M1 — SLAM & 定位]]
   - [[#M2 — 感知 & 目标检测]]
   - [[#M3 — 路径规划 & 导航]]
   - [[#M4 — 任务序列器]]
   - [[#M5 — 机械臂设计 & 控制]]
   - [[#M6 — 系统集成 & 基础设施]]
4. [[#三人分工方案]]
   - [[#成员 A — 感知 & 定位 Lead]]
   - [[#成员 B — Navigation 任务序列 & 项目管理 Lead]]
   - [[#成员 C — Manipulator & Embedded Systems Lead]]
5. [[#里程碑计划]]
   - [[#时间线总览]]
   - [[#Milestone 1 — Localization & Mapping]]
   - [[#Milestone 2 — Planning & Navigation]]
   - [[#Milestone 3 + Final Evaluation — Gripper Design & Trajectory Planning]]
   - [[#风险与应对]]
6. [[#评分结构]]

---

## 项目目标

设计并制造一个**具备搜索与救援能力的自主移动机器人**（依据 Introduction-1.pdf）：

- 识别彩色木块（4×4cm "伤员"），判断颜色
- 在场地中自主导航，绕过障碍物
- 用 2-DoF 机械臂抓取木块，运送到指定放置区
- 投入指定盒子（A 或 B）
- 完整任务流：`导航 → 抓取 → 导航 → 放置 → 返回`

---

## 系统架构

### 功能流水线

```
┌──────────────────────────────────────────────────────────────────────┐
│                            功能流水线                                 │
│                                                                      │
│  ┌──────────┐      ┌──────────┐                                     │
│  │  传感器   │ ───▶ │   感知    │                                     │
│  └──────────┘      └────┬─────┘                                     │
│                         │                                           │
│             ┌───────────┴───────────┐                               │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │     定位      │        │   目标检测    │                       │
│      └──────┬───────┘        └──────┬───────┘                       │
│             │                       │                               │
│             └───────────┬───────────┘                               │
│                         ▼                                           │
│                  ┌──────────────┐                                   │
│                  │     规划      │                                   │
│                  └──────┬───────┘                                   │
│                         │                                           │
│             ┌───────────┴───────────┐                               │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │   机械臂规划  │        │  移动平台规划 │                       │
│      │ 轨迹/抓取放置 │        │ 路径/避障     │                       │
│      └──────┬───────┘        └──────┬───────┘                       │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │ 机械臂运动控制│        │ 移动平台运动控制│                       │
│      └──────┬───────┘        └──────┬───────┘                       │
│             ▼                       ▼                               │
│      ┌──────────────┐        ┌──────────────┐                       │
│      │ 执行器：舵机  │        │ 执行器：轮子  │                       │
│      └──────────────┘        └──────┬───────┘                       │
│                                      ▼                               │
│                         获取机器人位置；反馈到定位                  │
└──────────────────────────────────────────────────────────────────────┘
```

| PDF 目标 | 流水线模块 |
|---------|-----------|
| 场地建图（SLAM） | 定位 |
| 木块颜色检测 + 光信标 | 目标检测 + 执行器（信标） |
| 路径规划 + 避障 | 规划（移动）+ 运动控制 |
| 任务序列（导航→抓→导航→放→返回） | 规划（顶层状态机，M4） |
| 2-DoF 臂 IK + 轨迹 | 规划（机械臂）+ 运动控制 |
| 抓取 / 放置 / 投入 A 或 B 盒 | 执行器（舵机 + 夹爪） |

---

### 硬件层

```
┌──────────────────────────────────────────────────────────────────────┐
│                              机器人硬件                               │
│                                                                      │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                  Jetson Orin Nano (SBC)                      │   │
│   │              Ubuntu 22.04  ·  ROS2 Humble                   │   │
│   │          主计算平台 · WiFi通信 · 所有ROS2节点运行于此           │   │
│   └───────────────┬──────────────────────┬───────────────────────┘  │
│                   │ Ethernet / USB        │ USB (micro-ROS)          │
│                   ▼                       ▼                          │
│   ┌───────────────────────┐   ┌───────────────────────┐            │
│   │    iRobot Create3     │   │    Arduino Uno         │            │
│   │    移动底盘             │   │   （低级控制器）          │            │
│   │  · ROS2 内置           │   │  · 机械臂舵机 (2×)     │            │
│   │  · 光学里程计           │   │  · 夹爪执行器           │            │
│   │  · 7× IR 障碍传感器    │   │  · 光信标舵机          │            │
│   │  · Cliff 传感器        │   │  · Power LED 开/关    │            │
│   │  · Docking 传感器      │   └───────────────────────┘            │
│   │  · Multi-zone bumper  │   （备选：STM32 / SAME54              │
│   │  · WiFi/BT/Ethernet   │     Xplained Pro）                    │
│   └───────────────────────┘                                         │
│                                                                      │
│   ┌──────────────────────┐   ┌──────────────────────────────────┐  │
│   │  SLAMTEC C1 LIDAR    │   │  Intel RealSense Camera          │  │
│   │  2D 激光雷达           │   │  RGB-D 深度相机                  │  │
│   │  → /scan (LaserScan) │   │  → /color/image, /depth/image    │  │
│   └──────────────────────┘   └──────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

| 组件 | 型号 | 作用 |
|------|------|------|
| 移动底盘 | iRobot Create3 | 运动执行、**光学里程计**、IR/cliff/docking 传感器、multi-zone bumper |
| 主处理板 | NVIDIA Jetson Orin Nano | ROS2 主节点运行、WiFi 通信 |
| 低级控制器 | Arduino Uno（可选备选：STM32 / SAME54 Xplained Pro） | 舵机 / 夹爪 / 光信标 PWM 驱动 |
| 激光雷达 | SLAMTEC C1 Fusion | LIDAR SLAM、障碍检测 |
| 深度相机 | Intel RealSense camera（具体型号待确认） | 颜色识别、深度测量、目标定位 |
| 机械臂 | 2-DoF DIY（3D 打印） | 抓取和放置 4×4 cm 木块 |
| 光信标 | Power LED + 180° 旋转舵机 | 照亮目标辅助颜色识别；支持**旋转 + 开/关** |

**定位传感器方案选择**

PDF 允许三种定位方案：(a) Create3 自带 IR 定位，(b) RealSense 视觉 SLAM，(c) SLAMTEC C1 LIDAR SLAM。我们采用 **LIDAR SLAM 作为主方案**（`slam_toolbox` 栈成熟，几何特征闭环鲁棒）。RealSense 留给**物体检测（RGB-D）**，必要时可作视觉 SLAM 退路。Create3 的 IR 障碍传感器作为辅助障碍信息进入 Nav2 局部 costmap。

---

### 软件层 ROS2 节点图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              ROS2节点图                                 │
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
│         │ /gripper_cmd        │  task_sequencer_node (状态机)        │  │
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

### 任务执行数据流

```
START
  │
  ▼
[IDLE] ──── 收到任务指令（target_color、target_box ∈ {A, B}）
  │
  ▼
[SCAN ARENA] ── LIDAR 建图；Camera 用于目标检测/辅助感知 ──▶ /map 更新完成
  │
  ▼
[NAVIGATE TO BLOCK] ── Nav2 路径规划 ──▶ 到达目标色块附近
  │
  ▼
[DETECT & ALIGN] ── 灯光 ON + 光信标转向目标
                  ── 颜色确认 + 精确对准 ──▶ /target_block_pose
  │
  ▼
[PICK] ── 机械臂 IK ──▶ Arduino 舵机 ──▶ 夹爪闭合
       ── 灯光 OFF
  │
  ▼
[NAVIGATE TO DROP] ── 根据 target_box (A/B) 选择放置位姿
                   ── Nav2 ──▶ 到达放置区
  │
  ▼
[PLACE] ── 机械臂轨迹 ──▶ 夹爪张开 ──▶ 放下木块
  │
  ▼
[RETURN HOME] ── Nav2 ──▶ 回到起点
  │
  ▼
END / NEXT TASK
```

---

### TF 坐标系树

```
map
 └── odom
      └── base_link              ← iRobot 中心
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

### ROS2 Workspace 结构

```
ros2_ws/
└── src/
    ├── imdl_msgs/           ← 自定义消息类型（全员共用，B 维护）
    ├── imdl_bringup/        ← launch 文件、参数配置（B）
    ├── imdl_slam/           ← SLAM 节点（成员 A）
    ├── imdl_perception/     ← 感知节点（成员 A）
    ├── imdl_navigation/     ← 导航 + 任务序列（成员 B）
    ├── imdl_manipulation/   ← 机械臂控制（成员 C）
    └── arduino_firmware/    ← Arduino 代码（成员 C）
```

---

### 全局 Topic Action 接口表

| Topic / Action | 消息类型 | 发布者 | 订阅者 |
|----------------|----------|--------|--------|
| `/map` | OccupancyGrid | A (slam) | B (nav2) |
| `/robot_pose` | PoseWithCovarianceStamped | A (slam) | B |
| `/scan` | LaserScan | A (lidar 驱动) | A (slam), B (costmap) |
| `/odom` | Odometry（光学） | iRobot Create3 | A (slam) |
| `/detected_objects` | DetectedObject[] | A (perception) | B |
| `/target_block_pose` | PoseStamped（frame_id: map） | A (perception) | B, C |
| `/beacon_cmd` | Float32（旋转角度，rad） | A (perception) | C (arduino bridge) |
| `/beacon_light` | Bool（LED 开/关） | B (task seq，唯一发布者) | C (arduino bridge) |
| `/goal_pose` | PoseStamped | B (task seq) | B (nav2) |
| `/cmd_vel` | Twist | B (nav2) | iRobot Create3 |
| `/pick_place_action` | Action（goal: PICK/PLACE + target_box） | B (client) | C (server) |
| `/joint_cmd` | JointState | C (arm ctrl) | C (arduino bridge) |
| `/joint_state_feedback` | JointState | C (arduino) | C (arm ctrl), B |
| `/gripper_cmd` | Bool | C (arm ctrl) | C (arduino bridge) |
| `/gripper_status` | Bool | C (arduino) | B |

---

## 模块规划

| 模块 | 名称 | 负责人 |
|------|------|--------|
| M1 | SLAM & 定位 | 成员 A |
| M2 | 感知 & 目标检测 | 成员 A |
| M3 | 路径规划 & 导航 | 成员 B |
| M4 | 任务序列器 | 成员 B |
| M5 | 机械臂设计 & 控制 | 成员 C |
| M6 | 系统集成 & 基础设施 | 成员 B（协调全员） |

---

### M1 — SLAM & 定位

**目标**：机器人在未知场地中自主建图，实时输出自身位姿。

**输入 / 输出**

| 方向 | Topic | 类型 |
|------|-------|------|
| 输入 | `/scan` | LaserScan |
| 输入 | `/odom` | Odometry（光学） |
| 输出 | `/map` | OccupancyGrid |
| 输出 | `/robot_pose` | PoseWithCovarianceStamped |
| 输出 | `/tf` (odom → map) | TF2 |

**算法选型**

| 方案 | 优点 | 缺点 | 推荐 |
|------|------|------|------|
| slam_toolbox | 成熟、支持 lifelong mapping | 参数较多 | ✅ 首选 |
| cartographer | Google 出品、精度高 | 配置复杂 | 备选 |
| gmapping | 简单 | 已过时 | ❌ |

**关键配置参数**

```yaml
# slam_toolbox online_async 模式
resolution: 0.05
max_laser_range: 12.0
minimum_travel_distance: 0.1
minimum_travel_heading: 0.1
map_update_interval: 5.0
```

**验收标准**
- [ ] 机器人绕场地一圈后，地图无明显撕裂
- [ ] 定位误差 < 10cm（与卷尺测量对比）
- [ ] 支持 localization-only 模式（已有地图下纯定位）

---

### M2 — 感知 & 目标检测

**目标**：识别木块颜色，输出木块在世界坐标系中的三维位置。

**输入 / 输出**

| 方向 | Topic | 类型 |
|------|-------|------|
| 输入 | `/camera/color/image_raw` | sensor_msgs/Image |
| 输入 | `/camera/depth/image_rect_raw` | sensor_msgs/Image |
| 输出 | `/detected_objects` | DetectedObject[]（自定义） |
| 输出 | `/target_block_pose` | geometry_msgs/PoseStamped（map 坐标系） |
| 输出 | `/beacon_cmd` | std_msgs/Float32 |

**自定义消息 `DetectedObject.msg`**

```
string  color            # "red" / "yellow" / "green" / "blue" / "purple"
geometry_msgs/Pose pose  # 在 map 坐标系中的位置
float32 confidence       # 检测置信度 0.0 ~ 1.0
```

**颜色识别流程**

```
RGB图 → HSV转换 → 颜色阈值 Mask → 形态学滤波
      → 轮廓检测 → 最小外接矩形（尺寸过滤） → 颜色标签
      → 对应深度像素 → 3D点（camera_frame）
      → TF 变换 → 3D点（map_frame） → 发布
```

**HSV 颜色范围参考（需现场标定）**

| 颜色 | H_min | H_max | S_min | V_min |
|------|-------|-------|-------|-------|
| 红色 | 0 / 160 | 10 / 180 | 100 | 100 |
| 黄色 | 20 | 35 | 100 | 100 |
| 绿色 | 40 | 80 | 100 | 100 |
| 蓝色 | 100 | 130 | 100 | 100 |
| 紫色 | 130 | 155 | 60 | 60 |

**光信标控制逻辑**

```python
dx = target.x - robot.x
dy = target.y - robot.y
angle_in_world = atan2(dy, dx)
angle_relative = angle_in_world - robot_yaw
beacon_angle = clamp(angle_relative, -90°, 90°)
# /beacon_light 由 B 的任务状态机在 ALIGN/DETECT 阶段发布 True，其他阶段发布 False
```

**验收标准**
- [ ] 静止状态下识别 5 种颜色，准确率 > 90%
- [ ] 输出 3D 位置精度 < 5cm
- [ ] 光信标动态指向目标；任务状态机在对准阶段打开 LED，其他阶段关闭 LED

---

### M3 — 路径规划 & 导航

**目标**：从当前位置规划无碰撞路径到达目标点并执行。

**输入 / 输出**

| 方向 | Topic | 类型 |
|------|-------|------|
| 输入 | `/map` | OccupancyGrid |
| 输入 | `/robot_pose` | PoseWithCovarianceStamped |
| 输入 | `/goal_pose` | PoseStamped |
| 输出 | `/cmd_vel` | geometry_msgs/Twist |

**Nav2 组件**

| 组件 | 功能 |
|------|------|
| BT Navigator | 行为树顶层协调 |
| Planner Server | 全局路径规划（NavFn / A*） |
| Controller Server | 局部控制（DWA） |
| Global Costmap | 静态地图 + 膨胀层 |
| Local Costmap | 实时障碍（LIDAR + Create3 IR） |
| Recovery Behaviors | 卡住时自动恢复（旋转、后退） |

**关键配置**

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

**验收标准**
- [ ] A → B 导航成功率 > 95%（10 次测试）
- [ ] 动态障碍物插入时，能实时重规划绕行
- [ ] 到达目标点后停车误差 < 3cm

---

### M4 — 任务序列器

**目标**：协调所有模块，按正确顺序完成完整搜救任务流程。

**状态机设计**

```
IDLE
 │
 ▼
NAVIGATE_TO_SCAN ──▶ (建图阶段)
 │
 ▼
NAVIGATE_TO_BLOCK ◀── target_color / target_box
 │
 ▼
ALIGN_TO_BLOCK  (灯光 ON；精确对准 < 3cm)
 │
 ▼
PICK_BLOCK ──▶ /pick_place_action (PICK) ──▶ 等待结果
            ──▶ 成功后灯光 OFF
 │
 ▼
NAVIGATE_TO_DROP（根据 target_box ∈ {A, B} 选择放置位姿）
 │
 ▼
PLACE_BLOCK ──▶ /pick_place_action (PLACE, target_box) ──▶ 等待结果
 │
 ▼
RETURN_HOME ──▶ IDLE / NEXT_TASK
```

**实现方式**

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

**验收标准**
- [ ] 状态转换正确，无死锁
- [ ] 任意一步失败后有错误恢复（重试 / 跳过 / 报警）
- [ ] 全流程一次跑通耗时 < 3 分钟
- [ ] 根据 `target_box` 正确路由到 A 盒或 B 盒

---

### M5 — 机械臂设计 & 控制

**目标**：设计、制造、控制 2-DoF 机械臂完成抓取与放置。

**机械结构规格**

> 以下尺寸和关节范围是团队初始设计假设，并非 PDF 给定条件；后续根据舵机扭矩、底盘安装空间、夹爪尺寸和 4×4 cm 木块实测抓取效果调整。

```
Joint 1 (Shoulder) — 绕 Z 轴旋转  范围: -90° ~ +90°
Joint 2 (Elbow)    — 绕 Y 轴旋转  范围:   0° ~ +135°
Gripper            — 平行夹爪      开口: 0 ~ 60mm

L1 (大臂) = 150 mm
L2 (小臂) = 120 mm
安装高度   =  80 mm（底盘面 → Joint1）
```

**夹爪姿态约束**

PDF 要求 gripper can orient itself in front of an object。本方案中，夹爪朝向由底盘朝向粗对准 + 2-DoF 机械臂末端位置共同满足：移动底盘先将机器人正面对准目标，机械臂再通过 IK 将夹爪移动到目标前方；若实测发现仅靠底盘朝向不足，再补充夹爪舵机或机械限位调整。

**逆运动学（2D 平面解析解）**

$$r = \sqrt{x^2 + y^2}$$

$$\cos\theta_2 = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

$$\theta_2 = \text{atan2}\!\left(\pm\sqrt{1-\cos^2\theta_2},\ \cos\theta_2\right)$$

$$\theta_1 = \text{atan2}(y,\ x) - \text{atan2}\!\left(L_2 \sin\theta_2,\ L_1 + L_2\cos\theta_2\right)$$

**Arduino 固件接口（micro-ROS）**

```
订阅:
  /joint_cmd     → JointState → 舵机 PWM
  /gripper_cmd   → Bool       → 夹爪开 / 闭
  /beacon_cmd    → Float32    → 光信标转向角
  /beacon_light  → Bool       → LED 开/关

发布:
  /joint_state_feedback → JointState
  /gripper_status       → Bool
```

**验收标准**
- [ ] micro-ROS 通信稳定（无掉线）
- [ ] IK 精度：末端误差 < 8mm
- [ ] 夹爪抓取成功率 > 85%（20 次测试）
- [ ] 全臂不与底盘碰撞

---

### M6 — 系统集成 & 基础设施

**Launch 文件结构**

```
imdl_bringup/launch/
  ├── robot.launch.py        ← 完整机器人（正式运行）
  ├── slam.launch.py         ← 仅 SLAM 调试
  ├── navigation.launch.py   ← 仅导航调试
  ├── perception.launch.py   ← 仅感知调试
  ├── arm.launch.py          ← 仅机械臂调试
  └── simulation.launch.py   ← Gazebo 仿真
```

**Git 分支规范**

```
main                 ← 仅合并经过测试的稳定代码
dev                  ← 集成分支
feature/slam         ← 成员 A
feature/perception   ← 成员 A
feature/navigation   ← 成员 B
feature/manipulation ← 成员 C
```

**可选加分任务**

| 任务 | 描述 | 加分 | 建议负责人 |
|------|------|------|-----------|
| 多传感器融合 | LIDAR + Camera 联合用于感知与规划 | +5% | A |
| 路径优化 | 最短时间 / 最低能耗路径规划 | +5% | B |

---

## 三人分工方案

| 成员       | 角色                                  | 核心模块                              |
| -------- | ----------------------------------- | --------------------------------- |
| **成员 A** | 感知 & 定位 Lead                        | 模块 M1 SLAM + 模块 M2 感知             |
| **成员 B** | Navigation 任务序列 & 项目管理 Lead         | 模块 M3 路径规划 & 导航 + M4 任务序列 + M6 集成 |
| **成员 C** | Manipulator & Embedded Systems Lead | 模块 M5 机械臂 + Arduino 全栈            |

**工作量阶段分布**

| 阶段 | 成员 A | 成员 B | 成员 C |
|------|--------|--------|--------|
| W1–5（Milestone 1） | SLAM 全套 + 传感器标定 | Nav2 baseline + Git/PM + Gazebo + 集成 checklist | Arduino 固件 + micro-ROS + 单舵机 + 支架/供电走线 |
| W6–10（Milestone 2） | 感知 3D + 光信标朝向控制 | 任务状态机 + 感知-导航集成 | CAD 定稿 + 3D 打印 + IK 仿真 + IK ROS2 节点 |
| W11–14（Milestone 3/Final） | 定位调参 + RViz Marker | 整系统集成 + 调试 + 演示编排 | 机械臂上车 + TF 标定 + 抓取调参 |

---

### 成员 A — 感知 & 定位 Lead

**核心职责**
- Jetson Orin Nano 系统配置（Ubuntu 22.04 + ROS2 Humble）
- iRobot Create3 固件更新，建立与 Jetson 的 ROS2 通信
- SLAMTEC C1 LIDAR 驱动集成，发布 `/scan`
- slam_toolbox 配置；发布 `/map`、`/robot_pose`、`/tf`
- LIDAR 与底盘外参标定
- 传感器安装位置确认与标定基准定义（支架/走线由 C 实现）
- Intel RealSense camera 驱动集成（RGB + 深度；具体型号待确认）
- HSV 颜色识别（现场标定阈值）
- 深度图融合：像素坐标 → 3D 世界坐标
- 光信标**朝向控制逻辑**（计算朝向角 → 发布 `/beacon_cmd`；LED 开关由 B 的任务状态机发布 `/beacon_light`；硬件由 C 实现）
- RViz2 MarkerArray 检测结果可视化

**技术栈**：ROS2 slam_toolbox · SLAMTEC C1 driver · iRobot Create3 SDK · Python OpenCV · realsense2_camera · TF2 · RViz2

**任务清单**

| 周次 | 任务 | 截止 |
|------|------|------|
| W1–2 | Jetson 刷机；Ubuntu 22.04 + ROS2 安装 | 04/29 |
| W1–2 | Create3 固件更新；`/odom` 验证 | 04/29 |
| W3–4 | LIDAR 驱动 → `/scan` 在 RViz 可视化 | 05/06 |
| W4–5 | slam_toolbox 配置；手动建图运行 | 05/13 |
| **M1** | **SLAM 实时建图 demo，误差 < 10 cm** | **05/13** |
| W6–7 | RealSense 驱动；RGB+深度在 RViz 可视化；HSV 颜色阈值标定 | 05/27 |
| W7–8 | 颜色检测节点（标签 + 像素坐标） | 06/03 |
| W8–9 | 深度融合 → 3D `/target_block_pose`（map 系） | 06/10 |
| W9–10 | 光信标朝向计算节点 | 06/17 |
| W6–10 | localization-only（AMCL）模式并行开发 | 06/17 |
| **M2** | **稳定定位 + 3D 检测 < 5 cm + 光信标跟随** | **06/17** |
| W11–13 | 向 C 提供 map 系目标位姿；配合 C 验证 map→arm_base_link TF；RViz Marker；调参 | 07/08 |
| W11–13 | （可选加分）多传感器融合：LIDAR + 相机 | 07/08 |
| **Final** | **整任务过程无定位漂失 + 不同光照下鲁棒检测** | **07/15** |

**接口**

| 方向 | 内容 |
|------|------|
| 提供给 B | `/map`, `/robot_pose`, `/tf`, `/detected_objects`, `/target_block_pose` |
| 提供给 C | `/target_block_pose`（map 系目标位姿），`/beacon_cmd` |
| 依赖 B | Git 仓库结构，`imdl_msgs` 包 |
| 依赖 C | 光信标 Arduino 固件就绪 |

---

### 成员 B — Navigation 任务序列 & 项目管理 Lead

**核心职责**
- Nav2 全栈配置（global planner + DWA + costmap + recovery）
- Python 任务状态机（M4）
- `/beacon_light` 唯一发布者：按任务状态控制 LED 开/关
- 动态障碍物避障
- 与感知联调：检测目标 → 自动导航
- Gazebo 仿真测试场
- Git 仓库管理 · `imdl_msgs` 包 · launch 文件（`imdl_bringup`）
- M1 集成 checklist：传感器支架、供电、数据连接、launch 启动顺序
- **项目管理**：每周 Project Tracking Chart、周三例会主持、GitHub Issues 维护

**技术栈**：ROS2 Nav2 · Python 状态机 · BehaviorTree.CPP（可选） · Gazebo · Git

**任务清单**

| 周次 | 任务 | 截止 |
|------|------|------|
| **W1** | **建 Git 仓库 + workspace 骨架 + 分支规范** | **04/22** |
| W1–2 | Gazebo 测试场搭建 | 04/29 |
| W1–2 | `imdl_msgs` 包，定义全部自定义消息 | 04/29 |
| W2–4 | Nav2 baseline 配置 | 05/06 |
| W4–5 | 仿真验证 A→B；物理机器人 baseline demo（stretch） | 05/13 |
| **M1** | **Nav2 baseline 可运行；A→B 物理导航 >80% 作为 stretch goal** | **05/13** |
| W6–8 | Python 任务状态机设计与实现 | 06/03 |
| W6–8 | `/pick_place_action` 协议定义（含 target_box） | 06/03 |
| W8–10 | 感知-导航集成：识别目标 → 自动导航 | 06/17 |
| **M2** | **完整 A→B→C→A 序列 dry-run（无臂）** | **06/17** |
| W11–12 | 与 C 联调完整 pick-place 流程 | 07/01 |
| W13–14 | 整系统集成测试 + 现场挑战准备 | 07/13 |
| W11–14 | （可选加分）路径/轨迹优化 | 07/13 |
| **Final** | **完整 demo（导航→抓→导航→放→返回）+ 终期答辩** | **07/15** |

**项目管理例行**
- [ ] 每周三例会前更新 Project Tracking Chart
- [ ] 维护 GitHub Issues（每个任务 → issue）
- [ ] 各里程碑前 2 天组织全员集成 dry-run

**接口**

| 方向 | 内容 |
|------|------|
| 依赖 A | `/map`, `/robot_pose`, `/detected_objects`, `/target_block_pose` |
| 发送给 C | `/pick_place_action`（ROS2 Action，含 target_box），`/beacon_light` |
| 依赖 C | `/gripper_status`, `/joint_state_feedback` |
| 提供给全员 | `imdl_msgs` 包，launch 文件，仓库结构 |

---

### 成员 C — Manipulator & Embedded Systems Lead

**核心职责**
- 2-DoF 机械臂 + 夹爪 CAD 设计（Solidworks / FreeCAD）
- 3D 打印、装配、安装到底盘
- 传感器/外设支架、供电与数据线固定（配合 A 标定）
- Arduino Uno 固件：舵机 + 夹爪 + 光信标 PWM + LED 开/关
- Jetson ↔ Arduino 通信桥（micro-ROS）
- MATLAB / Simulink IK 仿真验证
- ROS2 端 IK 求解节点：PoseStamped（arm_base_link 坐标系）→ JointState
- `map` → `arm_base_link` TF 转换与校验（基于 C 的机械臂安装标定）
- `pick_place_action` server 实现（处理 PICK / PLACE goal + target_box）
- 抓取成功率测试与调优

**技术栈**：Solidworks / FreeCAD · 3D Printer · Arduino IDE · micro-ROS · MATLAB Simulink · Python ikpy

**任务清单**

| 周次 | 任务 | 截止 |
|------|------|------|
| W1–2 | micro-ROS 在 Arduino 跑通；Jetson 收到心跳 | 04/29 |
| W1–2 | 传感器/外设支架方案 + 供电/数据连接 checklist | 04/29 |
| W2–3 | 单舵机 ROS2 topic 控制验证 | 05/06 |
| W3–4 | 2-DoF 臂 + 夹爪 CAD 初稿（参数化） | 05/06 |
| W4–5 | Arduino 固件：2 舵机 + 夹爪 + 光信标（旋转 + 开/关）统一控制 | 05/13 |
| **M1** | **Arduino 固件完成，所有执行器 ROS2 可控** | **05/13** |
| W6–8 | CAD 定稿 + 3D 打印全部零件 + 单元装配 | 06/03 |
| W8–9 | MATLAB IK 仿真：目标位置 → 关节角 | 06/10 |
| W9–10 | ROS2 IK 节点：arm_base_link 系 PoseStamped → JointState | 06/17 |
| **M2** | **CAD 定稿 + IK 仿真验证（仿真误差 < 5 mm）** | **06/17** |
| W11–12 | 机械臂上车装配 + TF 标定 | 07/01 |
| W11–12 | `pick_place_action` server（PICK / PLACE goal + target_box） | 07/01 |
| W12–13 | 抓取成功率测试 + 调参（目标 > 85%） | 07/08 |
| W13–14 | 与 B 最终集成；冗余测试 | 07/13 |
| **M3 + Final** | **物理臂 pick-place 完整 demo + 可靠处理 4×4 木块 + 终期答辩** | **07/15** |

**接口**

| 方向 | 内容 |
|------|------|
| 依赖 A | `/target_block_pose`（map 系目标位姿），`/beacon_cmd` |
| 依赖 B | `/pick_place_action` 指令（含 target_box），`/beacon_light` |
| 提供给 B | `/gripper_status`, `/joint_state_feedback` |
| 提供给 A | 光信标硬件接口（PWM 驱动 180° 舵机 + LED 开/关） |

---

## 里程碑计划

### 时间线总览
```
2026-04-15  课程开始
     │
     ├── Week 1-2   环境搭建 · 硬件探索 · Git 初始化
     ├── Week 3-5   SLAM · 基础导航 · Arduino 固件
     │
2026-05-13  ★ MILESTONE 1 — Localization & Mapping
     │
     ├── Week 6-8   定位优化 · Nav2 调优 · 3D 目标检测 · CAD 定稿
     ├── Week 9-10  任务序列 · 感知+导航集成 · IK 仿真
     │
2026-06-17  ★ MILESTONE 2 — Planning & Navigation
     │
     ├── Week 11-12 机械臂上车 · pick_place_action server
     ├── Week 13-14 整系统集成 · 抓取调参 · Final 准备
     │
2026-07-15  ★ MILESTONE 3（Gripper + Trajectory）+ FINAL EVALUATION
```

---

### Milestone 1 — Localization & Mapping

**日期**：2026-05-13（Week 5）

| 成员 | 交付内容 | 验收标准 |
|------|----------|----------|
| A | SLAM 建图完整运行 | 定位误差 < 10cm，地图无撕裂 |
| B | Nav2 baseline + A→B 基础导航 stretch demo | 仿真可运行；物理 A→B >80% 作为 stretch goal |
| C | Arduino 固件（舵机 + 夹爪 + 光信标 旋转/开关） | 全部执行器通过 ROS2 话题可控 |

**当天 Checklist**
- [ ] SLAM 地图在 RViz 实时更新（A）
- [ ] Nav2 baseline 可运行；A→B 物理导航现场演示作为 stretch（B）
- [ ] Arduino 舵机 + 夹爪 + 光信标（旋转 + 开/关）ROS2 话题演示（C）
- [ ] 5 分钟汇报 + 演示

---

### Milestone 2 — Planning & Navigation

**日期**：2026-06-17（Week 10）

| 成员 | 交付内容 | 验收标准 |
|------|----------|----------|
| A | 稳定定位 + 3D 目标位姿 + 光信标联动 | AMCL 已知地图下误差 < 8 cm；3D 精度 < 5 cm；信标跟随目标，对准时 LED ON |
| B | 完整任务序列 dry-run | A→B→C→A 一次成功（无机械臂）；target_box 路由生效 |
| C | CAD 定稿 + IK 仿真 + IK ROS2 节点 | MATLAB 仿真 IK 误差 < 5 mm；ROS2 节点可接收 arm_base_link 系 PoseStamped 输出 JointState |

**当天 Checklist**
- [ ] 机器人自动导航到指定颜色木块旁（A+B）
- [ ] 光信标实时指向目标 + LED 开关（A+C）
- [ ] A→B→C→A 任务序列完整演示（B）
- [ ] MATLAB IK 仿真结果展示（C）
- [ ] 5 分钟汇报 + 演示 + 创新点讨论

---

### Milestone 3 + Final Evaluation — Gripper Design & Trajectory Planning

**日期**：2026-07-15（Week 14）

| 成员 | 交付内容 | 验收标准 |
|------|----------|----------|
| A | 全程定位稳定 + 视觉引导抓取 | 任务流中不丢失定位；实时提供精确夹爪目标坐标 |
| B | 完整任务流集成 | 导航→抓→导航→放→返回 一次跑通；正确路由到 A 盒或 B 盒 |
| C | 实体臂抓取 demo | 成功率 > 85%（20 次）；末端 IK 误差 < 8 mm |

**Final Evaluation Checklist**
- [ ] 完整搜救流程现场演示（一次成功，包含投入指定 A 或 B 盒）
- [ ] 10 分钟终期答辩：概念、系统设计、技术细节、算法依据
- [ ] 完整技术文档
- [ ] 自评表提交（每人一份）
- [ ] Project Tracking Chart 最终版
