# Hackathon Guide: SLAM, Navigation and Your Own State Machine

This guide gets you from "robot switched on" to "robot runs my own state machine".
You will work through four stages:

1. **Get connected** to the robot and drive it by keyboard.
2. **Build a map** of the room with SLAM.
3. **Navigate** to single goals on that map.
4. **Write a state machine** that drives to positions, runs actions there,
   follows waypoints and turns around.

The robot is an iRobot Create3 with a SLAMTEC 2D lidar on a Jetson.
Only the lidar is used, no camera.

---

## 0. Before you start

### Your team's values

Ask your supervisor for these and use them wherever the guide shows the examples:

| Value | Example in this guide | Yours |
| --- | --- | --- |
| Robot namespace | `/Team2` | |
| ROS domain ID | `2` | |
| Jetson IP address | `192.168.178.138` | |
| Lidar mounting yaw | `3.14159265359` (lidar turned 180°) | |

**Every team needs its own domain ID.** If two teams share a domain, your
laptop will see and command the other team's robot.

### Who runs what

| Machine | What runs there |
| --- | --- |
| **Create3** | Robot firmware: odometry (`/Team2/odom`), wheel commands (`/Team2/cmd_vel`), bumpers |
| **Jetson** (via SSH) | Lidar driver; during mapping also SLAM; keyboard driving |
| **Laptop** (Docker container) | RViz, saving maps, localization (AMCL), Nav2, your state machine |

All three talk over Wi-Fi via ROS 2 (Humble). Jetson and laptop must be in the
same network.

### Safety

- One person always watches the robot and can pick it up or press its button.
- Software stop: press **Ctrl-C** in the terminal that sent the goal, then send
  zero velocity:
  ```bash
  ros2 topic pub --rate 10 /Team2/cmd_vel geometry_msgs/msg/Twist "{}"
  ```
- Start with slow speeds and short distances.

---

## 1. Setup

### 1.1 Laptop (once)

You need **Linux** with Docker. The container provides ROS 2 Humble, so the
ROS version installed on your laptop does not matter.

```bash
# allow Docker without sudo, then log out and back in
sudo usermod -aG docker $USER

# get the repository, then build the image and start the container
cd ~/imdl-main/docker
./run.sh --build
```

Inside the container, build the workspace once:

```bash
ws_build
```

### 1.2 Opening a terminal in the container (every time)

Every laptop terminal that runs ROS commands must be inside the container:

```bash
cd ~/imdl-main/docker && ./run.sh
```

- The first call starts the container, every further call opens another shell
  in the same container.
- ROS, the domain ID and the workspace are loaded automatically.
- Do **not** start it with `sudo`. Maps would end up in `/root/maps` instead
  of your home directory.
- Another domain ID: `ROS_DOMAIN_ID=5 ./run.sh`
- Your maps live in `~/maps` on the laptop and in `/root/maps` inside the container.
- The workspace `ros2_ws` is mounted into the container as `/ws`, so edit code
  on the laptop with your normal editor.

### 1.3 Jetson (once)

```bash
ssh jetson@192.168.178.138
echo 'export ROS_DOMAIN_ID=2' >> ~/.bashrc
source ~/.bashrc
```

### 1.4 Check the connection

In a container terminal:

```bash
ros2 topic list | grep Team2      # Create3 topics must appear
ros2 topic hz /Team2/odom         # odometry arrives (Ctrl-C to stop)
```

If nothing shows up, see [Troubleshooting](#troubleshooting).

---

## 2. Drive by keyboard

On the **Jetson** (SSH terminal):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Team2/cmd_vel
```

Click into the terminal and **hold** a key:

```
   u   i   o      i = forward        , = backward
   j   k   l      j / l = turn       k = stop
   m   ,   .      u o m . = curves

   q / z  all speeds up / down
   w / x  linear speed up / down
   e / c  turning speed up / down
```

Press `z` about six times first to slow down. If the robot sits in its dock,
undock it first:

```bash
ros2 action send_goal /Team2/undock irobot_create_msgs/action/Undock "{}"
```

---

## 3. Build a map (SLAM)

### 3.1 Pick and mark the start pose

Where the robot stands **and which way it faces** when SLAM starts becomes the
origin of your map: position `(0, 0)` and heading `yaw = 0`. Mark this spot on
the floor with tape, front and back. You will need it again later.
Keep the start mark at least **0.3 m away from walls**, otherwise navigation
cannot start from there.

### 3.2 Start SLAM on the Jetson

```bash
ros2 launch slam mapping.launch.py
```

### 3.3 Start RViz on the laptop

Container terminal 1:

```bash
ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359
```

This also publishes where the lidar sits on the robot (`base_link → lidar_link`).
**Keep it running whenever you work with the robot.** Without it, SLAM and
navigation get no usable lidar data.

In RViz:

- **Fixed Frame** (Global Options, top left) must be `map`. With any other
  frame, 2D Pose Estimate is ignored and 2D Goal Pose sends wrong goals.
- In the Map display set **Color Scheme = raw**. Some graphics drivers draw the
  default scheme wrongly.
- **Check the lidar orientation once:** set Fixed Frame to `base_link`, stand
  in front of the bumper. Your scan points must appear on the side of the red
  x-axis. If they appear behind the robot, change `lidar_yaw`, e.g. to `0.0`.

### 3.4 Drive around

Drive the robot slowly through the room with teleop (section 2):

- Turn **especially slowly**. Fast turns blur the map.
- Pause briefly before changing direction.
- Finish with a loop back to the start point.

The map grows in RViz: white is free, black is a wall, grey is unknown.

### 3.5 Save the map

While SLAM is still running, open a second container terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/my_map
```

Wait for `Map saved successfully`. The map now consists of two files:

| Inside the container | On the laptop (same files) |
| --- | --- |
| `/root/maps/my_map.yaml` | `~/maps/my_map.yaml` |
| `/root/maps/my_map.pgm` | `~/maps/my_map.pgm` |

`/root/maps` in the container *is* `~/maps` on the laptop, so you don't need to
copy anything. All later commands use the `/root/maps/...` path.

Saving again with the same name overwrites the old map, so use different
names to keep several maps.

### 3.6 Switch from mapping to navigation

| Where | Mapping (section 3) | Navigation (section 4) | What to do now |
| --- | --- | --- | --- |
| Jetson terminal 1 | `mapping.launch.py` | `lidar.launch.py` | **Ctrl-C**, then start `ros2 launch slam lidar.launch.py` |
| Jetson terminal 2 | teleop | – | **Ctrl-C** |
| Container terminal 1 | RViz | RViz | **keep running** |
| Container terminal 2 | `map_saver_cli` (finished) | `map_navigation.launch.py` | start it, see section 4.1 |

SLAM and navigation must never run at the same time. Both would try to decide
where the robot is.

---

## 4. Navigate to single goals

### 4.1 Start navigation

You need three things running:

| Where | Command |
| --- | --- |
| Jetson | `ros2 launch slam lidar.launch.py` (lidar only, no SLAM) |
| Container terminal 1 | RViz from section 3.3 (keep it running or start it again) |
| Container terminal 2 | `map_navigation.launch.py`, see step 3 below |

1. **Robot:** put it on the start mark from section 3.1, facing the same way
   as when you started SLAM.
2. **RViz** (container terminal 1), if it is not running anymore:
   ```bash
   ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=3.14159265359
   ```
3. **Localization + Nav2** (container terminal 2), loading the map you saved:
   ```bash
   ros2 launch imdl_autonomy map_navigation.launch.py map:=/root/maps/my_map.yaml
   ```
   Wait until the output shows `Managed nodes are active`, about 10–20 s.
   If the launch file is not found, run `ws_build` once.
4. **Check where the robot thinks it is:** the launch file sets the start pose
   to the start mark `(0, 0, 0)`. The red scan points must lie on the black
   walls of the map. If they don't, select **2D Pose Estimate** in RViz, click
   on the robot's real position and drag in its heading. Repeat until they match.

   Starting elsewhere? Add `initial_x:=... initial_y:=... initial_yaw:=...`
   to the launch command.

Until step 3 is running, RViz prints `Message Filter dropping message`. That is
expected, because the transform chain `map → odom → base_link` only exists
once Nav2 and AMCL run.

### 4.2 Coordinates

All goals are given in the **map frame**:

- `(0, 0)` is the SLAM start pose, `+x` is the direction the robot faced there,
  `+y` is to its left.
- Units are metres.
- `yaw` is the heading in radians, counter-clockwise from `+x`.

ROS stores the heading as a quaternion. For a pure rotation in the plane:
`z = sin(yaw/2)`, `w = cos(yaw/2)`.

| Heading | yaw | z | w |
| --- | --- | --- | --- |
| +x (as at start) | 0 | 0.0 | 1.0 |
| +y (left) | 1.5708 | 0.7071 | 0.7071 |
| −x (turned around) | 3.1416 | 1.0 | 0.0 |
| −y (right) | −1.5708 | −0.7071 | 0.7071 |
| any | yaw | `sin(yaw/2)` | `cos(yaw/2)` |

### 4.3 Read a position from the map

1. In a container terminal, start listening. The message type is needed
   because the topic only exists after the first click:
   ```bash
   ros2 topic echo /clicked_point geometry_msgs/msg/PointStamped
   ```
2. In RViz select **Publish Point** in the toolbar. If it is missing, add it
   with the `+` at the right end of the toolbar
   (`rviz_default_plugins/PublishPoint`).
3. Click on the map. The terminal shows the position:
   ```
   header:
     frame_id: map
   point:
     x: 0.098
     y: 1.114
   ```
   RViz switches back to the camera tool after every click. Select
   Publish Point again for the next one.

Good goals:

- lie in **white** (free) space; grey (unknown) space is never planned through,
- are at least **0.25–0.3 m away from black walls**; the robot's radius is
  about 0.2 m,
- can be reached through a gap at least ~0.5 m wide.

### 4.4 Drive to a goal

**Option A, click:** in RViz select **2D Goal Pose**, click the goal and drag
in the desired heading.

**Option B, exact coordinates** (container terminal 3). Insert the `x` and `y`
from section 4.3 and the heading from the table in section 4.2:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.10, y: 1.11}, orientation: {z: 0.0, w: 1.0}}}}"
```

- Every line except the last must end with exactly **one** `\`.
- Add `--feedback` at the end to see the remaining distance while driving.
  It prints a lot.
- Ctrl-C cancels the goal and stops the robot.

The output ends with the status:

| Status | Meaning |
| --- | --- |
| `SUCCEEDED` | Goal reached |
| `ABORTED` | Nav2 gave up. Find the reason in section 4.5. |
| `CANCELED` | You pressed Ctrl-C |

Several goals in a row: send the next goal after the previous one finished
with `SUCCEEDED`. To automate this, write a state machine (section 5).

### 4.5 When a goal fails

The reason is in the terminal running `map_navigation.launch.py`
(container terminal 2):

| Message there | Cause | Fix |
| --- | --- | --- |
| `Starting point in lethal space` | The **robot** stands too close to a wall, or localization is wrong | Check that the scan matches the map; push the robot ~20 cm into open space and set **2D Pose Estimate** |
| `Goal ... in lethal space` / `outside bounds` | The **goal** is inside or too close to a wall, or off the map | Pick a goal further into white space |
| `failed to create plan` (no other reason) | No passage wide enough between robot and goal | Pick another goal, or map the area better |
| `collision_monitor: Robot to approach ... away from collision` and the robot does not turn | Something is closer than ~0.2 m to the robot | Move the robot away from the wall |
| `Exceeded time allowance ... Spin` | Recovery turn blocked by a nearby obstacle | As above |

The `--feedback` output also gives hints: `distance_remaining: 0.0` together
with a growing `number_of_recoveries` means Nav2 never found a path.

### 4.6 Drive closer to walls

Nav2 keeps a safety margin around obstacles. You can make it smaller when
launching navigation:

```bash
ros2 launch imdl_autonomy map_navigation.launch.py map:=/root/maps/my_map.yaml \
  inflation_radius:=0.21 cost_scaling_factor:=15.0
```

| Argument | Default | Effect |
| --- | --- | --- |
| `inflation_radius` | `0.23` | Distance (m) around walls with extra cost. **Never below ~0.21**, because the robot itself has a radius of ~0.20 m. |
| `cost_scaling_factor` | `10.0` | Higher = the cost falls off faster = the robot may pass closer. |
| `cost_travel_multiplier` | `1.0` | Lower = planned paths may hug the walls more. |

See the effect in RViz: add a Map display for `/global_costmap/costmap`. The
coloured band around the walls gets thinner.

The robot centre can never get closer to a wall than its own radius, and the
collision monitor still slows the robot down near obstacles.

### 4.7 Exercises

1. Read three positions with Publish Point and drive to each of them with
   `send_goal`.
2. Drive to the same point with headings `0`, `1.5708` and `3.1416`. Watch how
   the robot turns at the goal.
3. Send a goal inside a wall. What does Nav2 answer?
4. Find the smallest `inflation_radius` that still works reliably in your
   arena.
5. Push the robot by hand (gently) during a run. What does AMCL do?

---

## 5. Your own state machine

### 5.1 The example

The package `imdl_statemachine` contains a working example you can copy and
extend:

```
INIT → GO_TO_ACTION_POINT → DO_ACTION → GO_TO_ROUTE_START → SLOW_ROUTE → TURN → DONE
                        (any failure or timeout → FAILED)
```

| File | Purpose |
| --- | --- |
| `ros2_ws/src/imdl_statemachine/config/waypoints.yaml` | All poses, speeds and timeouts |
| `ros2_ws/src/imdl_statemachine/imdl_statemachine/state_machine.py` | The state machine |
| `ros2_ws/src/imdl_autonomy/launch/map_navigation.launch.py` | AMCL + Nav2 (section 4.1) |

**The example poses in `waypoints.yaml` belong to a different map.** Replace
them with free points from *your* map (section 4.3) before the first run.

### 5.2 Run it

1. Start lidar, RViz and `map_navigation.launch.py` as in section 4.1, steps 1–3.
2. Put the robot on the start mark. The state machine sets the start pose
   from `initial_pose` in the YAML, so no 2D Pose Estimate is needed.
3. Container terminal 3:
   ```bash
   ros2 run imdl_statemachine state_machine
   ```
4. Watch the current state in another terminal:
   ```bash
   ros2 topic echo /state_machine/state
   ```

Ctrl-C stops the state machine and cancels the running navigation goal.

Use your own waypoint file:

```bash
ros2 run imdl_statemachine state_machine --ros-args -p waypoints_file:=/root/maps/our_points.yaml
```

Changes to `.py` and `.yaml` files take effect on the next start without
rebuilding. Run `ws_build` only after adding new files or packages.

### 5.3 How it works

Each state is one method that does its work and **returns the next state**:

```python
def on_go_to_action_point(self) -> State:
    # drive to 'action_point' from the YAML; on success continue with DO_ACTION
    return self.go_to('action_point', State.DO_ACTION)
```

`go_to(target, next_state)` takes either a **name from the YAML** or a pose
given directly as `[x, y, yaw]`. If driving fails, it returns `State.FAILED`.
You can pass a different third argument, e.g. a state that retries.

```python
return self.go_to('kitchen', State.BEEP)                 # named point in the YAML
return self.go_to([-1.0, 0.3, 3.14], State.BEEP)         # pose written directly
return self.go_to('kitchen', State.BEEP, State.GO_HOME)  # on failure go to GO_HOME
```

A named point is just one more line in the YAML:

```yaml
kitchen: [-1.0, 0.3, 3.14159]
```

`run()` calls the method of the current state over and over until the state is
`DONE` or `FAILED`. The mapping from state to method is in `self._handlers`.

`run_task(label, start)` starts a Nav2 task, waits until it finishes, prints
progress, and returns `True` on success. On timeout it cancels the task.

### 5.4 Building blocks

`self.nav` is a `BasicNavigator` from `nav2_simple_commander`:

| What | Code |
| --- | --- |
| Drive to a pose | `return self.go_to('name_in_yaml', State.NEXT)` or `self.go_to([x, y, yaw], State.NEXT)` |
| Drive through several poses | `self.run_task('label', lambda: self.nav.goThroughPoses([p1, p2, p3]))` |
| Turn in place (rad, + = left) | `self.run_task('label', lambda: self.nav.spin(spin_dist=3.14, time_allowance=30))` |
| Limit speed (m/s, 0 = no limit) | `self.set_speed_limit(0.08)` |
| Wait without blocking ROS | `self.sleep(2.0)` |
| Build a pose | `self.pose([x, y, yaw])` |
| Read a value from the YAML | `self.config['my_key']` |
| Log | `self.nav.info('text')`, `self.nav.warn(...)`, `self.nav.error(...)` |

### 5.5 Adding a state: step by step

Example: the robot beeps at the action point.

**1. Add the state to the enum:**

```python
class State(Enum):
    ...
    BEEP = auto()
```

**2. Write the method:**

```python
from builtin_interfaces.msg import Duration
from irobot_create_msgs.msg import AudioNote, AudioNoteVector

def on_beep(self) -> State:
    msg = AudioNoteVector()
    msg.notes = [AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000))]
    self.audio_pub.publish(msg)
    self.sleep(0.5)
    return State.GO_TO_ROUTE_START
```

**3. Create the publisher in `__init__`:**

```python
self.audio_pub = navigator.create_publisher(AudioNoteVector, '/Team2/cmd_audio', 10)
```

**4. Register the method in `self._handlers` and connect it:**

```python
State.BEEP: self.on_beep,
```

Then let a previous state return `State.BEEP`, e.g. `on_do_action`.

Other Create3 topics you can use for actions:

| Topic | Type | Effect |
| --- | --- | --- |
| `/Team2/cmd_audio` | `irobot_create_msgs/msg/AudioNoteVector` | Play tones |
| `/Team2/cmd_lightring` | `irobot_create_msgs/msg/LightringLeds` | Set the LED ring colours |
| `/Team2/interface_buttons` | `irobot_create_msgs/msg/InterfaceButtons` | Read the robot's buttons |
| `/Team2/hazard_detection` | `irobot_create_msgs/msg/HazardDetectionVector` | Bumper, cliff and other hazards |

Show a message definition with `ros2 interface show irobot_create_msgs/msg/LightringLeds`.

### 5.6 Hackathon challenges

Start simple and build up:

1. **Your points:** change `waypoints.yaml` to points on your map and run the
   example unchanged.
2. **Beep:** add the `BEEP` state from section 5.5 at the action point.
3. **Patrol:** visit a list of points from the YAML in a loop, three times,
   then return to the start.
4. **Turn and look:** at every point, turn 360° slowly before moving on.
5. **Recover:** when a goal fails, try the next point instead of going to
   `FAILED`. Count the failures.
6. **Button start:** wait in a `WAIT_FOR_BUTTON` state until a robot button is
   pressed (`/Team2/interface_buttons`).
7. **Your idea:** a delivery route, a light show, a "follow the walls" tour …

---

## Troubleshooting

| Symptom | Check / fix |
| --- | --- |
| Container sees no robot topics | Same `ROS_DOMAIN_ID` everywhere? Check with `echo $ROS_DOMAIN_ID` on Jetson and in the container. Then `ros2 daemon stop` and retry. Laptop firewall: `sudo ufw allow from <your-subnet>/24`. |
| `Message Filter dropping message: frame 'lidar_link'` | Is `real_rviz.launch.py` running? Is `map_navigation` or SLAM running? `ros2 run tf2_ros tf2_echo map lidar_link` |
| Teleop does nothing | Is the teleop terminal focused? Is the robot docked? Any other `ros2 topic pub` still running? |
| Map is blurry or has double walls | Drive and turn more slowly. Only one SLAM/AMCL may run: `ros2 node list` |
| Map looks rotated in RViz (scan and map still match) | Normal: `+x` is where the robot faced at SLAM start. Rotate the RViz view with the mouse. |
| Scan does not match the map walls | Is the robot really on the start mark? Set the pose with **2D Pose Estimate**. SLAM must **not** run during navigation: `ros2 node list \| grep slam_toolbox` must be empty. |
| Scan rotated by 180° compared to the map | RViz must use the same `lidar_yaw` as during mapping. Otherwise build the map again. |
| Goal `ABORTED` | See section 4.5. |
| `file 'map_navigation.launch.py' was not found` | Run `ws_build` once in the container. |
| Log: `transform from base_link to odom ... frame does not exist` | No odometry from the Create3. Check `ros2 topic hz /Team2/odom`. If nothing arrives, wake the robot with its centre button and wait 1–2 min for its Wi-Fi. |
| `ros2 topic echo /clicked_point`: `Could not determine the type` | Add the type: `ros2 topic echo /clicked_point geometry_msgs/msg/PointStamped` |
| 2D Pose Estimate has no effect, goals end up in strange places | RViz Fixed Frame must be `map`. |
| Log: `AMCL cannot publish a pose ... Please set the initial pose` | Normal for the first seconds. If it keeps repeating, set **2D Pose Estimate**. |
| Robot does not move after goal accepted | `ros2 lifecycle get /controller_server` must say `active`. `ros2 topic info /Team2/cmd_vel -v` shows who publishes. |
| Saved map is missing in `~/maps` | The container was started with `sudo`. Copy it out with `sudo docker cp imdl:/root/maps/my_map.pgm ~/maps/` (same for `.yaml`). |
| TF errors such as "extrapolation into the future" | Clocks out of sync: `ssh jetson@<ip> 'date +%s.%N'; date +%s.%N` should differ by less than 0.1 s. |
| `No module named nav2_simple_commander` | `apt-get install -y ros-humble-nav2-simple-commander` in the container, or rebuild with `./run.sh --build` |

Useful commands:

```bash
ros2 topic list                         # all topics
ros2 topic hz /scan                     # lidar rate (~10 Hz)
ros2 topic echo /Team2/odom --once      # one odometry message
ros2 node list                          # running nodes
ros2 run tf2_ros tf2_echo map base_link # robot pose in the map
ros2 lifecycle get /bt_navigator        # Nav2 ready? (active)
```
