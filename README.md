# Feed mixer robot

An iRobot Create 3 that maps the arena on its own, fills feed from a three-silo
dispenser, mixes it, and drives laps around the cow area unloading it through a
side door.

| Machine | Runs |
| --- | --- |
| Jetson (on the robot) | lidar, SLAM + Nav2, frontier explorer, mixer ESP driver, web panel |
| ESP32 (USB serial to the Jetson) | 4 load cells, 2 mixer motors, door servo |
| Raspberry Pi | silo valve servos |
| Create 3 base | its own ROS 2 nodes under `/Team2` (odometry, `cmd_vel`, bumpers) |

## Repository layout

```text
.
├── mixer_esp32/                ESP32 firmware (PlatformIO), see its README
├── raspberry/
│   ├── valve_controller.py     ROS 2 node for the valve servos (PCA9685)
│   ├── servo_sweep.py          find a servo's usable pulse range
│   ├── data/valve_calibration.json
│   └── Dockerfile
├── valve_cmd.py                command / calibrate the valves from the Jetson
└── ros2_ws/
    ├── dependencies.repos      pinned sllidar_ros2 source
    └── src/
        ├── frontier_explorer/  lidar launch, SLAM + Nav2, frontier exploration
        └── mixer_esp_driver/   ROS driver and web panel for the ESP
```

### `frontier_explorer`

| Launch file | What it starts |
| --- | --- |
| `lidar.launch.py` | The lidar alone: `sllidar_node` (`/scan_raw`) and `scan_timestamp_fixer` (`/scan`). Use it to test the lidar. |
| `slam_navigation.launch.py` | `odom_tf_republisher`, `scan_timestamp_fixer`, slam_toolbox, Nav2 and twist_mux. Map and navigate at the same time. |
| `explore.launch.py` | The frontier explorer: sends Nav2 goals until no frontier is left, then drives back to the start. |
| `mapping.launch.py` | Lidar + slam_toolbox only, for mapping by teleop without Nav2. |
| `nav2.launch.py` | Nav2 itself; included by `slam_navigation.launch.py`. |

| Node | Does |
| --- | --- |
| `scan_timestamp_fixer` | `/scan_raw` -> `/scan`, stamped with the Jetson's clock |
| `odom_tf_republisher` | `/Team2/odom` -> `odom -> base_link` TF |
| `frontier_explorer` | picks the best frontier in `/map` and sends it to Nav2 |

Parameters are in `config/`: `nav2_params.yaml`, `slam_toolbox_params.yaml`,
`twist_mux.yaml`. Nav2 and slam_toolbox themselves come from apt.

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

One terminal each:

```bash
ros2 launch frontier_explorer lidar.launch.py fix_timestamps:=false
ros2 launch frontier_explorer slam_navigation.launch.py   # wait for "Managed nodes are active"
ros2 launch frontier_explorer explore.launch.py           # the robot starts driving
ros2 run nav2_map_server map_saver_cli -f ~/maps/arena    # once exploration is done
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

The ESP is on `/dev/ttyCH341USB0` (`mixer_esp_driver/config.py`). The Create 3
base needs no launch file: its firmware publishes `/Team2/*` and listens on
`/Team2/cmd_vel`.

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
