---
tags:
  - IMDL
  - 成员A
  - slam
  - perception
  - 个人计划
created: 2026-04-29
updated: 2026-07-17
owner: 成员A
---

# 成员A — 个人计划
## 感知 & 定位 Lead（M1 SLAM + M2 感知）
---

## 你在系统中的角色

你是**感知 & 定位 Lead**，负责 M1（SLAM）和 M2（感知）两个模块。你的输出是所有其他模块的依赖：

| 话题 | 类型 | 谁依赖它 |
|------|------|---------|
| `/map` | `nav_msgs/OccupancyGrid` | B（Nav2 代价地图） |
| `/robot_pose` | `geometry_msgs/PoseWithCovarianceStamped` | B、C |
| `/tf`（odom → map） | TF2 | C（深度融合），机械臂TF链 |
| `/scan` | `sensor_msgs/LaserScan` | B（避障代价地图） |
| `/detected_objects` | `std_msgs/String` JSON（当前实现；后续可替换为 `imdl_msgs/DetectedObject[]`） | B（任务调度） |
| `/target_block_pose` | `geometry_msgs/PoseStamped`（map 系） | B、C |
| `/beacon_cmd` | `std_msgs/Float32`（旋转角度 rad） | C（Arduino 光信标驱动） |

如果 SLAM 漂移或崩溃，**一切都会停止**。

---

## 当前进度总览

> 最后更新：2026-07-14

### 第1周（2026-04-15 → 2026-04-21）

| # | 任务 | 状态 |
|---|------|------|
| 1 | 刷写 Jetson Ubuntu 22.04 | ✅ |
| 2 | 安装 ROS2 Humble | ✅ distrobox imdl 已完成 |
| 3 | 安装 Create3 ROS2 软件包 | ✅ `ros-humble-irobot-create-msgs` 已安装 |

### 第2周（2026-04-22 → 2026-04-28）

| # | 任务 | 状态 |
|---|------|------|
| 4 | 更新 Create3 固件 | ✅ |
| 5 | 验证 `/odom` 话题 | ✅ 20Hz，ROS_DOMAIN_ID=2，namespace /Team2 |
| 6 | 安装 SLAMTEC C1 ROS2 驱动 | ✅ 源码编译完成（`sllidar_ros2`） |

### 第3周（2026-04-29 → 2026-05-06）✅ 全部完成

| 日期 | 任务 | 状态 |
|------|------|------|
| 5月1日 | 启动雷达驱动，验证话题频率 | ✅ 10Hz |
| 5月2日 | RViz2 可视化 LaserScan | ✅ |
| 5月3日 | 编写 `base_link → lidar_link` 静态TF | ✅ |
| 5月5日 | 15分钟压力测试（无掉帧） | ✅ |

### 第4周（2026-05-07 → 2026-05-10）✅ 全部完成

| 日期 | 任务 | 状态 |
|------|------|------|
| 5月7日 | 安装 slam_toolbox | ✅ |
| 5月8日 | 编写 `slam_toolbox_params.yaml` | ✅ |
| 5月9日 | 启动 slam_toolbox + 键盘控制 Create3 → 确认地图增长 | ✅ RViz2 确认 |
| 5月10日 | 保存地图（`map_saver_cli`） | ✅ `maps/arena_map.pgm` + `.yaml` |
| 5月12日 | 重构：包名 `imdl_slam` → `slam`，添加中文注释 | ✅ |
| 5月12日 | 推送代码到 GitLab `feature/slam` 分支 | ✅ |

### 第5周（2026-05-10 → 2026-05-13）✅ 全部完成

| 日期        | 任务                                   | 状态                      |
| --------- | ------------------------------------ | ----------------------- |
| 5月12日     | `/robot_pose` 话题（slam_toolbox remap） | ✅                       |
| 5月12日     | 编写 `localization.launch.py`（仅定位模式）   | ✅                       |
| 5月12日     | 重新建图，pgm/yaml + posegraph/data 同步保存  | ✅ 2.25m×2.35m，覆盖2m×2m场地 |
| 5月12日     | 地图文件推送到 GitLab `feature/slam`        | ✅                       |
| **5月13日** | **里程碑1演示**                           | ✅                       |

---

### Week 6-8 (2026-05-14 to 2026-06-05)

**目标**：启动 RealSense 相机，完成颜色识别节点，输出 2D 检测结果。

| 周次    | 任务                                       | 截止    | 状态                                                                                         |
| ----- | ---------------------------------------- | ----- | ------------------------------------------------------------------------------------------ |
| W6–7  | Intel RealSense 驱动安装；RGB + 深度在 RViz2 可视化 | 05/27 | ✅ 2026-05-20 完成，固件升级至 5.17.0.10，`perception` 包 + `depth_align` 节点                          |
| W6–7  | 配置 AMCL 仅定位模式（`localization_launch.py`）  | 05/27 | ✅ 2026-05-20 完成，`/robot_pose` 发布验证，RViz2 Map+LaserScan+PoseWithCovariance 正常               |
| W6–10 | AMCL 仅定位模式；误差 < 8 cm                     | 06/17 | ✅ 已完成；后续若现场复测异常再调参                                                                         |
| W7–8  | HSV 颜色阈值现场标定（4种颜色）；编写颜色识别节点              | 06/03 | ✅ 2026-05-27 完成，`color_detector.py` 节点，已移除 ROI，整张 RGB 图像检测；4 种颜色 red/blue/yellow/green 已实现 |
| W6–8  | 精确测量 `base_link → camera_link` 外参，提供给成员C | 06/05 | ✅ 2026-06-03 完成，已写入 `camera.launch.py` 静态 TF，并在 Jetson 上用 `tf2_echo` 验证                    |

---
### Week 8–10 (2026-06-05 to 2026-06-17)

**目标**：3D 目标位姿检测，光信标朝向控制节点，AMCL 稳定验证。

| 周次     | 任务                                 | 截止        | 状态                                                                                                                                                    |
| ------ | ---------------------------------- | --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| W8–9   | 深度图融合 → map 系 `/target_block_pose` | 06/10     | ✅ 2026-06-03 已实现，`target_pose_estimator.py` 发布 `PoseStamped`；精度 <5cm 待现场验证                                                                            |
| W8–9   | 发布 `/detected_objects`             | 06/10     | ✅ 2026-06-03 已实现旧 `std_msgs/String` JSON；该历史接口已在 2026-07-17 补充正式类型化输出 `/perception/detected_objects`（`imdl_msgs/msg/DetectedObjectArray`） |
| W9–10  | 光信标朝向计算节点 → `/beacon_cmd`          | 06/17     | ✅ 2026-06-10 已实现并在 Jetson 实机验证，发布 `std_msgs/Float32` 角度命令；左右摆动目标时输出正负角度变化正常                                                                           |
| W9–10  | 与B联调：感知 → 导航全流程                    | 06/14     | ✅ 2026-06-10 成员A侧接口已就绪并在 Jetson 实机验证：`/map`、`/scan`、`/robot_pose`、`/target_block_pose`、`/detected_objects`、`/beacon_cmd` 正常；待 B 侧 Nav2 / 任务状态机完成端到端导航 |
| **M2** | **稳定定位 + 3D检测精度 < 5cm + 光信标跟随**    | **06/17** | ✅ 成员A侧定位、感知与光信标输出已打通；精度验证/与B端到端导航联调仍是主风险                                                                                                              |

---

### Week 11–14 (2026-06-18 to 2026-07-15)

#### 2026-06-24 — YOLO Segmentation

| 任务组               | 任务                                | 输出 / 验收标准                                                                                                                | 状态                                                                                      |
| ----------------- | --------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------- |
| YOLO segmentation | 设计 segmentation 类别与标注规范           | 类别为 `red_block`、`yellow_block`、`green_block`、`blue_block`、`blue_obstacle`；每个实例标 polygon mask；目标是替代 HSV 颜色阈值并区分蓝色木块/蓝色障碍物 | ✅ 2026-06-24 已固定 5 类及 `data.yaml`                                                       |
| YOLO segmentation | 采集 segmentation 训练数据与负样本          | 覆盖近/中/远、不同光照、机器人移动模糊、蓝色木块靠近蓝色障碍物、只有障碍物无木块；数据能支撑 Task 4/5 未知位置目标检测                                                        | ✅ 已录制两段 RGB rosbag、统一抽帧并人工清理。初始候选 628 张；最终训练集为 train 424 / val 58 / test 28，共 510 张 |
| YOLO segmentation | 标注 YOLO segmentation 数据集          | 所有木块和蓝色障碍物有 mask；优先保证 `blue_block` 与 `blue_obstacle` 同框样本质量                                                              | ✅ 已完成 CVAT polygon 人工种子标注、SAM 辅助轮廓、YOLO 伪标签与人工复核；最终数据位于 `yolo_seg/data/releases/final_v1` |
| YOLO segmentation | 训练并导出第一版 YOLO segmentation 模型     | 模型输出 class、confidence、bbox、mask/contour；`blue_obstacle` 不应被发布为 `blue_block`；导出 Jetson 可用权重                               | ✅ 2026-07-05 完成 YOLO11n-seg 最终训练；训练记录为 `yolo_seg/experiment/training/final_yolo11n_seg`，并冻结部署副本 `perception/models/yolo11n_seg_imdl.pt`。修正标签后的离线测试在 conf=0.35 下为 45 TP / 0 FP / 1 FN；蓝色障碍物未被误分类为蓝色木块 |
| YOLO segmentation | Jetson segmentation 推理 smoke test | Jetson 上能实时输出 bbox、mask/contour、confidence；推理速度不造成导航等待                                                                   | ✅ 2026-07-14 完成：Jetson Orin CUDA YOLO runtime、RealSense RGB 输入、`/yolo/detections` 与实时分割调试画面已验证 |

#### 2026-07-01 — Integration And Validation

| 任务组       | 任务                                      | 输出 / 验收标准                                                                                                                                                                                 | 状态  |
| --------- | --------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --- |
| 感知接口      | 冻结 YOLO 模型与 2D 检测接口                    | 固定 5 类 ID、`imdl_yolo_seg_v1` 模型版本、SHA256、`/yolo/detections` JSON schema 和 `/yolo/debug_image`；`blue_obstacle.is_block=false`，不能作为抓取目标                                      | ✅ 接口文档、JSON Schema、部署模型 `yolo11n_seg_imdl.pt`、校验文件已完成；旧 `/detected_color` 已从活动几何链路移除 |
| 感知接口      | 实现 YOLO ROS2 推理节点                      | 订阅 RGB；发布检测 JSON 与调试图；参数化模型路径、置信度、IoU、图像尺寸、设备和最大推理频率；无目标时发布空数组                                                        | ✅ 已在 Jetson + RealSense 运行：`yolo_segmentation_node.py`、独立 launch、`/yolo/detections`、原始及 JPEG 调试画面均已验证 |
| 感知接口      | 替换 perception 输出接口                      | 旧 `/detected_objects` 保持 JSON 兼容；正式跨团队接口为 `/perception/detected_objects`，类型 `imdl_msgs/msg/DetectedObjectArray`，包含 class、image position、depth、base/map position、timestamp 和 validity flags | ✅ 类型化消息、转换发布、接口文档和 36 项本地测试已完成 |
| 几何融合      | 融合 segmentation + mask 后处理 + depth + TF | YOLO segmentation 负责类别/mask；几何后端只用 aligned depth + camera intrinsics + TF 计算 3D 坐标、base_link 极坐标和 map 坐标；不使用木块尺寸或 bbox 大小估距                       | ✅ 已实现有效像素比例/MAD 质量检查、时间不同步状态和 2D `corners_px`；Jetson 已验证 aligned depth、mask 内深度估计和 RGB-D 融合画面 |
| Task 5 支持 | 目标选择与检测丢失处理                           | 根据颜色请求选择当前目标；同色多个候选时按 confidence、距离、深度有效性排序；障碍物类别不能被选为木块目标；短暂丢失沿用 last known position，超过 stale timeout 后失效                                                                            | ✅ 已实现 `target_selector`、`/current_target_object`、`/requested_target_class`、selection reason、last known position 和 stale timeout |
| 联调接口      | 定义 B/C 消费接口                  | B/C 使用 `/perception/detected_objects`；导航过滤 `map_pose_valid=true`，控制过滤 `is_block && depth_valid && base_pose_valid` 并转换到 `arm_base_link`                                                                                         | ✅ 共享消息、接口文档和本地转换测试已完成 |

#### 2026-07-15 — Milestone 3

| 任务组 | 任务 | 输出 / 验收标准 | 状态 |
|---|---|---|---|
| Milestone 3 | Milestone 3 演示与汇报 | 这不是最终演示；展示截至 Milestone 3 的 SLAM、learning-based perception、目标位姿输出和与B/C联调进展；`corners_px` 仅作为 mask 后处理增强项展示 | ✅ 离线模型、部署接口、Jetson 实时 YOLO、mask 深度融合、RGB-D 可视化和目标选择代码已完成 |

---

### Week 15 onward (from 2026-07-16) — Final Task-Oriented Validation

> 唯一最终验收基准：`document/Tasks - known obstacles.pdf`。Task 1–4 每项最多 3 分钟，Task 5 最多 10 分钟；定位 ≤ 10 cm、朝向 ≤ 45°、物体放置 ≤ 1.5 cm 为满分评分线。已知障碍物必须由地图/Nav2 costmap 处理，视觉 `blue_obstacle` 只作为补充感知信息。

#### 笔记本阶段 — 代码、接口与离线准备

> 本阶段不依赖 RealSense、Jetson、地图 TF、机械臂或场地；完成后只交付可复现的软件、离线证据和联调材料。

| 优先级 | 工作 | 输出 / 验收标准 | 状态 |
| --- | --- | --- | --- |
| P0 | 冻结可交接感知版本 | `imdl_msgs`、`perception`、模型和文档提交到共享分支；`colcon build --packages-select imdl_msgs perception --symlink-install` 与 `colcon test --packages-select perception` 通过 | ✅ 2026-07-17，提交 `0d30c78`；本机构建、模型哈希和测试通过 |
| P0 | 类型化对象发布集成测试 | 用模拟 YOLO、aligned depth、CameraInfo 与静态 TF 启动 `target_pose_estimator`；验证 `/perception/detected_objects` 的正常、无深度、缺 TF、时间不同步和 `blue_obstacle` 情况 | ✅ 2026-07-17，`test_typed_object_publisher_integration.py` 覆盖五种情况；80 项本机测试通过 |
| P0 | B/C 模拟联调包 | 提供对象模拟发布器和最小订阅示例；B 能过滤 `map_pose_valid`，C 能过滤 `is_block && depth_valid && base_pose_valid` | ✅ 2026-07-17，新增 `imdl_integration_tools`；模拟发布、B/C 过滤、ROS 端到端测试和 launch 烟雾测试通过 |
| P1 | 本地 CPU 离线 YOLO 回放 | 修复独立 ML venv 的 `torch`/`ultralytics` 后，用两段 RGB rosbag 验证五类的类别、mask、bbox 与像素中心；RGB-only bag 不得宣称验证深度或 map 坐标 | ✅ 2026-07-17，Conda `imdl` 以 CPU-only 回放两段 bag 共 416 张；五类均出现，逐帧 JSON/标注图与可视化人工审核确认无问题。未保留完整逐条 TP/FP/FN 复核记录；不宣称深度或 map 坐标验证。 |
| P1 | 离线错误分析与阈值记录 | 对近/中/远、模糊、遮挡、暗光及蓝色同框样本记录 TP/FP/FN 和混淆；输出 confidence 阈值选择依据 | ⬜ |
| P1 | 实验室量测与故障测试表 | 为深度、base/map 坐标、TF 丢失、短暂漏检、颜色顺序和时间预算准备记录表、尺量方法与通过条件 | ⬜ |

#### 实验室真机阶段 — 必须依赖硬件的证据

> 本阶段使用 Jetson、RealSense、Create3、LIDAR、真实 TF、地图和机械臂；所有 PDF 精度、碰撞和时间结论只能在此阶段记录。

| 优先级 | 工作 | 输出 / 验收标准 | 状态 |
| --- | --- | --- | --- |
| P0 | 已知障碍物 map/costmap 对齐（A/B 联合） | 将已知障碍物配置到地图/Nav2 costmap 或等价规划表示；Task 2–5 路径避开障碍物，记录零碰撞演练结果 | ⬜ |
| P0 | 定位位置与朝向精度复测（A 主责） | 尺量 `/robot_pose`：位置误差 ≤ 10 cm、朝向误差 ≤ 45°，满足 PDF 满分评分线 | ⬜ |
| P0 | Jetson 类型化接口 smoke test | Jetson 上 `/perception/detected_objects` 正确发布，`ros2 topic echo` 显示五类、像素位置、深度、有效性标志；模型 SHA256 通过 | ⬜ |
| P0 | 深度与坐标精度量测 | 使用尺量真值记录 `depth_m`、`position_base_m`、`position_map_m` 误差；校准外参后复测，并说明是否足以支持 ≤ 1.5 cm 放置目标 | ⬜ |
| P0 | B/C 实机接口联调 | B 消费有效 map 坐标，C 将有效 base 坐标转换到 `arm_base_link`；无效/过期/障碍物对象不会触发导航或抓取 | ⬜ |
| P0 | Task 4/5 感知演练 | 未知物体搜索时稳定发现目标；按外部给定颜色顺序选目标；抓取/放置确认后才由状态机请求下一个颜色 | ⬜ |
| P1 | 长时间鲁棒性与总任务时间 | 记录漏检、TF 断链、定位丢失、明显减速和恢复时间；对齐 Task 1–4 ≤ 3 min、Task 5 ≤ 10 min 的限制 | ⬜ |

#### Future code decision rule

任何新感知功能必须明确改善以下至少一项：Task 4/5 未知目标发现率、三维坐标/抓取精度、`blue_obstacle` 与 `blue_block` 区分、接口安全失效行为或最终任务时间。不能映射到这些验收项的改动不应优先于上述计划。

---
## 里程碑检查清单

### 里程碑1 — 2026-05-13

- [x] `/scan` 以 10Hz 发布
- [x] slam_toolbox online_async 模式运行
- [x] RViz2 实时显示地图
- [x] 地图已保存（`maps/arena_map.pgm` + `.yaml` + `.posegraph` + `.data`）
- [x] 代码推送到 GitLab `feature/slam`
- [x] `/robot_pose` 话题可访问
- [x] 仅定位模式可在已保存地图上运行（`localization.launch.py`）
- [x] `/map` 和 `/robot_pose` 可被成员B访问（联调）
- [x] 演示前练习两遍流程
- [x] 准备5分钟PPT

### 里程碑2 — 2026-06-17

- [x] AMCL 仅定位模式 launch 已配置，`/robot_pose` 发布验证完成（2026-05-20）
- [x] AMCL 现场连续运行稳定性验证完成（后续若复测异常再调参）
- [x] 已知地图下定位误差 < 8 cm（后续若复测异常再补充记录与调参）
- [x] RealSense 驱动稳定，RGB + 深度在 RViz 可视化（2026-05-20 完成）
- [x] HSV 颜色阈值现场标定与 `color_detector.py` 完成（发布 `/detected_color`，2026-05-27；已移除 ROI，整张 RGB 图像检测）
- [x] 4种颜色静止识别准确率 > 90%（red/blue/yellow/green；后续若复测异常再复调阈值）
- [x] `base_link → camera_link` 外参TF已实测并提供给成员C（2026-06-03：x=0.19, y=0.0, z=-0.02, roll=0, pitch=0.262, yaw=0）
- [x] 3D 目标位姿 `/target_block_pose` 已发布；精度 < 5 cm 待现场验证
- [x] 旧 `/detected_objects` JSON 正确发布；后续已新增正式 `/perception/detected_objects`（`imdl_msgs/msg/DetectedObjectArray`）供 B/C 消费
- [x] 光信标朝向节点正常工作（2026-06-10：`/beacon_cmd` 已在 Jetson 上验证，随目标左右变化输出正负角度）
- [x] 成员A侧感知 → 导航联调接口已就绪（2026-06-10：`/map`、`/scan`、`/robot_pose`、`/target_block_pose`、`/detected_objects`、`/beacon_cmd` 已实机验证；待 B 侧完成 Nav2 端到端执行）

### Milestone 3 — 2026-07-15

- [x] 已完成五类 YOLO11n-seg 数据集构建：CVAT 人工标注、SAM 辅助轮廓、伪标签和人工复核
- [x] 已训练并冻结 `imdl_yolo_seg_v1`；离线测试可区分四种木块颜色与 `blue_obstacle`
- [x] 已实现 `/yolo/detections`、`/yolo/debug_image` 接口、YOLO ROS2 节点和独立 launch；`blue_obstacle` 标记为 `is_block=false`
- [x] 已将 YOLO、mask 深度融合、目标选择和光信标合并进 `camera.launch.py`；旧 HSV 输入与木块尺寸估距不在活动感知路径中
- [x] 已完成代码冻结前的质量诊断、2D `corners_px`、YAML 配置、JSON Schema、自动化测试、`imdl_msgs` 类型化接口和 Jetson 同步/预检脚本
- [x] 已在 Jetson Orin + RealSense 上验证实时 YOLO segmentation、`/yolo/detections`、`/detected_objects`、mask 深度估计与 RGB-D 融合画面
- [x] 已实现 `/perception/fused_debug/compressed` 和 `/perception/object_markers`，用于远程 RGB-D 诊断与 RViz 地图对象可视化
- [x] 已输出木块 `corners_px` 作为图像平面姿态/抓取增强项；它不是经标定的 6D 抓取姿态，暂不强制训练 YOLO Pose 模型
- [x] 已实现 last known position / stale timeout，避免单帧漏检直接清空目标；仍待实机验证不会造成导航抖动
