# SLAM Module — Delivery Information for Member B

> Last updated: 2026-05-12
> Author: Member A (Perception & Localization Lead)

---

## Code Location

- GitLab: `https://gitlab.lrz.de/00000000014C6004/imdl`
- Branch: `feature/slam`

## Connect to Jetson

```bash
ssh jetson@10.23.64.137
```

## Launch Commands

Localization mode (normal use):
```bash
ros2 launch slam localization.launch.py
```

Mapping mode (only if remapping is needed):
```bash
ros2 launch slam mapping.launch.py
```

## Available Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 0.2Hz | Occupancy grid map, 0.05m resolution |
| `/robot_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 10Hz | Robot localization pose |
| `/scan` | `sensor_msgs/LaserScan` | 10Hz | Laser scan data |
| `/tf` | TF2 | 50Hz | odom→base_link→map transform chain |

## Notes

- Set `ROS_DOMAIN_ID=2` before running any ROS2 commands
- After launching localization mode, use **2D Pose Estimate** in RViz2 to set the robot's initial position on the map
- Map file `~/maps/arena_map.posegraph` is already saved on the Jetson
