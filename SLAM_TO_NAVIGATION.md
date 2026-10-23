# From SLAM to Navigation

Build a map of the arena, then click goals in RViz and let the robot drive there.
No state machine, no mission files.

Values used below (replace them with your team's):

| Value | Here |
| --- | --- |
| Robot namespace | `/Team2` |
| ROS domain ID | `2` |
| Jetson | `jetson@192.168.178.138` |
| Lidar mounting yaw | `3.14159265359` |
| Map name | `my_map` |

Every laptop terminal below is a **container terminal**:

```bash
cd ~/imdl-main/docker && ./run.sh
```

---

## Once: build the workspace

In a container terminal:

```bash
ws_build
```

Repeat this only after new files or packages were added to `ros2_ws/src`.
Changes to existing files work without rebuilding.

---

## Shortcut: map and navigate at the same time

No saved map, no AMCL, no 2D Pose Estimate. slam_toolbox keeps building the map
while Nav2 plans on it, so you can click goals from the first second.

| Terminal | Command |
| --- | --- |
| Jetson 1 | `ros2 launch slam mapping.launch.py` |
| Container 1 | `ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359` |
| Container 2 | `ros2 launch imdl_autonomy slam_navigation.launch.py` |

Wait for `Managed nodes are active`, set the Map display's **Color Scheme to
`raw`**, then select **2D Goal Pose** and click. Unexplored area counts as
free, so the first goals may lie outside the mapped region; the robot replans
as the map grows. Drive short hops and keep goals **0.3 m from walls**.

Want the map afterwards? Save it while this is still running:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/my_map
```

Teleop can run alongside it (Jetson 2) to fill in corners by hand.

The two parts below are the other route: map first, then navigate on the saved
map with AMCL. Use it when you need the exact same map across several runs.

---

## Part 1: Build the map

### 1. Mark the start pose

Put the robot where it should always start and mark position and heading on
the floor with tape. This pose becomes `(0, 0, yaw 0)` of the map.

### 2. Start everything

| Terminal | Command |
| --- | --- |
| Jetson 1 | `ros2 launch slam mapping.launch.py` |
| Container 1 | `ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359` |
| Jetson 2 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Team2/cmd_vel` |

In RViz, set the Map display's **Color Scheme to `raw`**.

### 3. Drive

In the teleop terminal (hold the key):

```
   u   i   o      i = forward    , = backward    j / l = turn    k = stop
   j   k   l      z = slower (press ~6x first)   c = slower turning
   m   ,   .
```

Drive slowly through the whole arena, turn slowly, and finish back on the
start mark. The red scan points must stay on the black walls.

### 4. Save the map (SLAM still running)

Container 2:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/my_map
```

Wait for `Map saved successfully`. The map is now in `/root/maps` in the
container, which is `~/maps` on the laptop. You don't need to copy anything.

---

## Part 2: Switch to navigation

| Terminal | Now | Action |
| --- | --- | --- |
| Jetson 1 | `mapping.launch.py` | **Ctrl-C**, then `ros2 launch slam lidar.launch.py` |
| Jetson 2 | teleop | **Ctrl-C** |
| Container 1 | RViz | **keep running** |
| Container 2 | `map_saver_cli` (done) | start navigation, see below |

`lidar.launch.py` starts only the lidar. `slam` is just the package name.
SLAM and navigation must never run at the same time.

---

## Part 3: Navigate with 2D Goal Pose

### 1. Put the robot on the start mark

Same place, same heading as when SLAM was started.

### 2. Start localization + Nav2

Container 2:

```bash
ros2 launch imdl_autonomy map_navigation.launch.py map:=/root/maps/my_map.yaml
```

Wait until the output shows `Managed nodes are active`, about 10–20 s.

### 3. Check the start pose in RViz

The launch file tells the robot that it stands on the start mark `(0, 0, 0)`.
The red scan points should now lie on the black walls of the map.

If they don't, select **2D Pose Estimate**, click on the robot's real position
on the map and drag in its heading. Repeat until scan and map match.

Starting somewhere else than the start mark? Pass the pose when launching:
`... map_navigation.launch.py map:=/root/maps/my_map.yaml initial_x:=0.5 initial_y:=-0.2 initial_yaw:=1.57`

### 4. Click goals

Select **2D Goal Pose**, click the goal and drag in the desired heading. The
robot drives there. Repeat for further goals.

Pick goals in open space, at least **0.3 m from walls**. The robot has a
radius of about 0.19 m.

**Stop:** Ctrl-C in container 2, or in any container terminal:

```bash
ros2 topic pub --rate 10 /Team2/cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `file 'map_navigation.launch.py' was not found` | Run `ws_build` once. |
| RViz: `Message Filter dropping message` | Normal until step 2 of Part 3 runs. |
| Log: `AMCL cannot publish a pose ... Please set the initial pose` | Normal for the first seconds. If it keeps repeating, set the pose with **2D Pose Estimate** in RViz. |
| Log: `transform from base_link to odom ... frame does not exist` | No odometry from the Create3. Check with `ros2 topic hz /Team2/odom`. If nothing arrives, wake the robot with its centre button. |
| Scan does not match the map | Robot not on the start mark? Set **2D Pose Estimate** carefully. Check that `mapping.launch.py` is **not** running: `ros2 node list \| grep slam_toolbox` must be empty. |
| Map rotated by 180° compared to the scan | RViz must use the same `lidar_yaw` as during mapping. Otherwise build the map again. |
| Goal fails: `Starting point in lethal space` | Localization is wrong (see above), or the lidar sees parts of the robot itself. Check in RViz with Fixed Frame `base_link`. |
| Goal fails right away (`ABORTED`) | Goal too close to a wall. Click further into open space. |
| Container sees no robot topics | `echo $ROS_DOMAIN_ID` must be `2` on the Jetson and in the container. Then `ros2 daemon stop`. |
