# Feed mixer robot

An iRobot Create 3 that maps the arena on its own, fills feed from a three-silo
dispenser, mixes it, and drives laps around the cow area unloading it through a
side door.

| Machine | Runs |
| --- | --- |
| Jetson (on the robot) | lidar, SLAM + Nav2, frontier explorer, mixer ESP driver, web panel |
| ESP32 (USB serial to the Jetson) | 4 load cells, 2 mixer motors, door servo |
| Raspberry Pi | silo valve servos (`silo_controller`) |
| Create 3 base | its own ROS 2 nodes under `/Team2` (odometry, `cmd_vel`, bumpers) |

## Repository layout

```text
.
├── mixer_esp32/                  ESP32 firmware (PlatformIO), see its README
└── ros2_ws/
    ├── dependencies.repos        pinned sllidar_ros2 source (vcs import)
    └── src/
        ├── frontier_explorer/    lidar, live SLAM + Nav2, frontier exploration, map saving
        ├── mixer_esp_driver/     ESP driver: load cells, mixer motors, door; web panel
        ├── silo_controller/      silo valve servos on the Raspberry Pi
        └── sllidar_ros2/         SLAMTEC lidar driver (downloaded, not in git)
```

## Packages

| Package | Runs on | What it is for |
| --- | --- | --- |
| `frontier_explorer` | Jetson | Everything for driving around: starts the lidar, builds the map live (slam_toolbox), plans and drives (Nav2), explores the arena on its own and saves the map when it is done. |
| `mixer_esp_driver` | Jetson | Talks to the ESP32 over USB serial: weight from the 4 load cells, the 2 mixer motors (ramped, plus the shake pattern), the door servo. Also serves the web panel, and launches the two webcams it shows. |
| `silo_controller` | Raspberry Pi | Opens and closes the 3 silo valves (servos on a PCA9685). Runs in Docker on the Pi; `valve_cmd` commands it from the Jetson. |
| `sllidar_ros2` | Jetson | SLAMTEC's lidar driver, pinned in `dependencies.repos`. We only use its `sllidar_node`, through our `lidar.launch.py`. |

The Create 3 base needs no package: its firmware publishes `/Team2/odom`,
bumpers and so on, and listens on `/Team2/cmd_vel`.

## Launch files

| Launch file | What it starts | When to use it |
| --- | --- | --- |
| `frontier_explorer lidar.launch.py` | `sllidar_node` (`/scan_raw`) and `scan_timestamp_fixer` (`/scan`) | Test the lidar; always runs under SLAM |
| `frontier_explorer slam_navigation.launch.py` | `odom_tf_republisher`, `scan_timestamp_fixer`, slam_toolbox, Nav2, twist_mux | Map and navigate at the same time |
| `frontier_explorer explore.launch.py` | `frontier_explorer` | Explore the whole arena on its own, then save the map |
| `frontier_explorer mapping.launch.py` | lidar, `scan_timestamp_fixer`, `odom_tf_republisher`, slam_toolbox (no Nav2) | Map by driving with teleop yourself |
| `frontier_explorer nav2.launch.py` | Nav2 servers and lifecycle managers | Not by hand: `slam_navigation.launch.py` includes it |
| `mixer_esp_driver cameras.launch.py` | two `v4l2_camera` nodes: `/camera0`, `/camera1` (`image_raw` + `image_raw/compressed` JPEG) | The camera feeds in the web panel |
| `mixer_esp_driver base_teleop.launch.py` | `esp_driver_node`, `web_control` (`web:=false` to skip), optional keyboard teleop (`teleop:=true`) | Mixer, door, scale and the web panel |

`silo_controller` has no launch file: the Pi runs `valve_controller` in Docker
(see its `Dockerfile`). From the Jetson:
`ros2 run silo_controller valve_cmd 2 calibrate`.

| Node | Does |
| --- | --- |
| `scan_timestamp_fixer` | `/scan_raw` -> `/scan`, restamped with the Jetson's clock so scans and TF match |
| `odom_tf_republisher` | `/Team2/odom` -> `odom -> base_link` TF |
| `frontier_explorer` | picks the best unexplored edge of `/map`, sends it to Nav2; at the end drives home, saves the map, reports done |
| `esp_driver_node` | serial link to the ESP: `/loadcell/*`, `/mixer/motor_*`, `/mixer/shake`, `/door/*` |
| `web_control` | the web panel on port 8000; streams `/camera0` and `/camera1` as MJPEG |
| `valve_controller` | `/valve1..3/percent` -> servo angles, with saved open/closed calibration |

Parameters: `frontier_explorer/config/` (`nav2_params.yaml`,
`slam_toolbox_params.yaml`, `twist_mux.yaml`) and
`mixer_esp_driver/config.py`. Nav2 and slam_toolbox themselves come from apt.

## How mapping works

The lidar alone only gives distances (`/scan`); it does not make a map.
slam_toolbox makes the map out of two inputs: the scans, and the Create 3's
wheel odometry (`/Team2/odom`, turned into the `odom -> base_link` transform by
`odom_tf_republisher`). Odometry says roughly how the robot moved, the scans
correct that, and the result is `/map` plus the `map -> odom` transform.

`map` (0, 0) is where the robot stood when SLAM started. Start from the same
marked spot every run, so saved positions stay valid.

Nav2 plans on that growing map. `frontier_explorer` picks where to go next:
the edges between known free space and unknown space. When none is left it
drives back to the start and saves the map:

| Output | |
| --- | --- |
| `~/maps/arena.yaml` + `.pgm` | the map, in nav2 `map_server` format |
| `~/maps/arena_<date>_<time>.yaml` + `.pgm` | a copy, so a later run does not overwrite it |
| `~/maps/arena.posegraph` + `.data` | slam_toolbox's pose graph, to localize on this map later |
| `/frontier_explorer/status` | latched: `starting`, `exploring`, `returning`, `saving`, `done`, or `failed: ...` |

Change the place with `map_dir:=... map_name:=...` on `explore.launch.py`.
Save by hand at any time (e.g. while mapping by teleop with the explorer
running): `ros2 service call /frontier_explorer/save_map std_srvs/srv/Trigger`.
Stop exploring early, drive home and save:
`ros2 topic pub --once /frontier_explorer/return_home std_msgs/msg/Empty`.

## Setup on the Jetson

```bash
sudo apt install python3-vcstool ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
  ros-humble-irobot-create-msgs
cd ~/imdl-main
vcs import --skip-existing ros2_ws/src < ros2_ws/dependencies.repos
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Build from a shell that has only `/opt/ros/humble` sourced. A workspace built
while another one is sourced keeps that one as an underlay for good.

ROS settings on the Jetson and the robot: `ROS_DOMAIN_ID=2`,
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, robot namespace `/Team2`.

## Run

Run `sws` first in every terminal (`cd ~/imdl-main/ros2_ws && source install/setup.bash`).

### Test the lidar

```bash
ros2 launch frontier_explorer lidar.launch.py
ros2 topic hz /scan
```

Defaults: `serial_port:=/dev/ttyUSB0`, `serial_baudrate:=460800` (SLAMTEC C1),
`frame_id:=base_link`. Override them if the lidar enumerates elsewhere or is a
different model (A1/A2M8: 115200, A2M12/A3/S1: 256000).

### Map the arena

Put the robot on its start mark, then one terminal each:

```bash
ros2 launch frontier_explorer lidar.launch.py fix_timestamps:=false
ros2 launch frontier_explorer slam_navigation.launch.py   # wait for "Managed nodes are active"
ros2 launch frontier_explorer explore.launch.py           # the robot starts driving
ros2 topic echo /frontier_explorer/status                 # "done" = map saved
```

`slam_navigation.launch.py` runs its own scan timestamp fixer, hence
`fix_timestamps:=false` on the lidar. Teleop on `/Team2/cmd_vel_teleop`
overrides Nav2 through twist_mux; Nav2 resumes 0.5 s after the last key press:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Team2/cmd_vel_teleop
```

### Mixer, door and web panel

```bash
ros2 launch mixer_esp_driver base_teleop.launch.py
```

The panel is on `http://<jetson>:8000`. The ESP is on `/dev/ttyCH341USB0`
(`SERIAL_PORT` in `mixer_esp_driver/config.py`).

Camera feeds in the panel:

```bash
ros2 launch mixer_esp_driver cameras.launch.py
```

Camera 0 is the C920 in USB port 2.2, camera 1 the one in port 2.1. The
launch uses the `/dev/v4l/by-path/` links for those ports, because
`/dev/videoN` numbers change when a camera is replugged. Override with
`camera0_device:=/dev/video0 camera1_device:=/dev/video2`. A camera can only be
opened by one node, so stop any other `v4l2_camera_node` first. The panel
streams at up to 10 fps per browser and pauses while its tab is hidden or the
cameras are turned off there.

### Silo valves (Raspberry Pi)

On the Pi, from `ros2_ws/src/silo_controller`: build and run the Docker image
(commands at the top of its `Dockerfile`). `data/` holds the open/closed
calibration and is mounted into the container. From the Jetson:

```bash
ros2 run silo_controller valve_cmd 2 calibrate   # interactive: jog, save open/closed
ros2 run silo_controller valve_cmd 2 50          # 50 % open
```

## Jetson to Create 3 over the USB-Ethernet dongle

The Create 3 is wired to the Jetson with a USB-C to Ethernet dongle on the
robot and a cable to the Jetson's RJ45 port. The robot has no Wi-Fi network
saved; only the Jetson is on Wi-Fi (`imdl_5Ghz`, the only Wi-Fi profile it keeps).
The Create 3 supports 2.4 GHz only, so it could not join the 5 GHz network anyway.

A direct USB-C to USB-C cable to the Jetson does not work: the Jetson never
gets a USB data link from the robot (`/sys/class/udc/*/state` stays `default`),
and `A`-to-`C` cables cannot work because both ends are USB hosts. Use the
dongle.

| | Address | Where it is set |
| --- | --- | --- |
| Create 3 (web server, ROS) | `192.168.186.2` | fixed by the robot's `usb0`/dongle |
| Jetson wired port `enP8p1s0` | `192.168.186.3/24`, no gateway | NetworkManager profile `Wired connection 1` (persistent) |
| Jetson Wi-Fi | DHCP, changes between reboots | find it with `arp -a` on the MAC `f8:3d:c6:ad:a5:11` |

Robot settings (web page at `http://192.168.186.2`, Application ->
Configuration): domain `2`, namespace `/Team2`, `rmw_fastrtps_cpp`, no
discovery server.

### Reach the robot's web page from your laptop

```bash
ssh -N -L 8081:192.168.186.2:80 jetson@<jetson-wifi-ip>
# open http://localhost:8081
```

### Discovery needs a Fast DDS profile

Without Wi-Fi the robot no longer announces itself by multicast on the wired
link, so plain `ros2 topic list` on the Jetson shows only
`/parameter_events` and `/rosout`. The Jetson therefore contacts the robot
directly. `/home/jetson/fastdds_create3_local.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="create3_unicast_and_local" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          <locator><udpv4><address>192.168.186.3</address></udpv4></locator>
          <locator><udpv4><address>192.168.186.2</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

The Jetson and its own address must be in the list. A profile with only the
robot makes the robot visible and every node on the Jetson invisible, and the
reverse without a profile. `~/.bashrc` on the Jetson exports
`FASTRTPS_DEFAULT_PROFILES_FILE` for this profile and `ROS_LOCALHOST_ONLY=0`,
and defines `sws`. Every ROS process on the Jetson needs that variable. After
changing it, run `ros2 daemon stop`, and give `ros2 node list` a few seconds
(or `--spin-time 10`).

### After a reboot

- Plug the dongle in before the robot starts its ROS app. If the robot booted
  without it, restart the app from its web page ("Restart application"), since
  the app only uses the wired interface that exists when it starts.
- Nothing launches automatically. Start the ESP driver, the lidar and
  navigation yourself, after the wired link is up.
- Run `sws` first and launch from `~/imdl-main/ros2_ws`. The ESP driver reads
  `loadcell_calibration.json` and `door_calibration.json` from the **current
  directory**; launched from `~` it used the wrong load-cell calibration and
  the default door ends (1000/2000 us) instead of 500/2600 us.

## Web panel: one controller at a time

Open the panel (`http://<jetson>:8000`, or through an SSH tunnel) on as many
devices as you like. Only one holds control; the rest are view-only. Press
**Take control** to claim it (taking it over from another device asks first and
stops their motors and the base). The holder keeps control while its page is
open, and it frees itself a few seconds after the page closes or loses contact.
**STOP ALL**, **Close all valves** and **Cancel dose** work from every device.

Do not run `ros2 topic pub` on `/mixer/motor_*` or `/Team2/cmd_vel` while a
device holds control; the panel publishes there too. It is protection against
collisions between people, not security: there is still no login.

Code: `mixer_esp_driver/control_lease.py` and `web_control.py`. Tests:
`pytest ros2_ws/src/mixer_esp_driver/test` (needs `fastapi` and `httpx`).
