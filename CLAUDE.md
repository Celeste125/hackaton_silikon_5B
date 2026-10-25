# CLAUDE.md — branch `dispenser-localization`

Context for a Claude session picking this up cold. It covers what this branch
adds, the decisions worth not undoing, and the hardware facts that cost hours
to learn. Detailed docs are linked rather than repeated.

## What this branch is for

Put landmarks the camera can see onto the SLAM map, and make the position
improve as the robot sees them from more angles. The lidar cannot see the
landmarks, which is the entire reason a camera pipeline exists here.

Three things run, two of them new on this branch:

```
  JETSON (camera + lidar)                LAPTOP (SLAM, Nav2, RViz)
  ───────────────────────                ─────────────────────────
  landmark_detector  ─── JSON, <1 kB/frame ───►  landmark_mapper
   (perception)                                  (imdl_autonomy)
   runs the model                                TF at the image stamp,
   resolves intrinsics                           fuses all views,
   samples depth locally                         publishes markers + poses
                                                        │
  lidar ──── /scan ────► SLAM ──── /map ─────────────────┤
                                                        │
                           frontier_explorer + camera_coverage_explorer
```

## The two landmarks, and the name mismatch

`landmarks.yaml` defines **two** landmarks, and between them they claim all
three of the detector's classes:

| Landmark | Model classes | What it is |
|---|---|---|
| `cow_enclosure` | `cow_head`, `cow_body` | Paper cow cutouts on and behind cardboard boxes. |
| `food_dispenser` | `pipe` | The vertical pipes hanging from the wooden frame. |

**The dispenser's class is called `pipe`.** The landmark name is the robot's,
the class name is the model's, and `canonical_class` exists so the two need
not match. `food_dispenser` was removed in `40e7cf3` when no model could see
it and restored once `cows_pipes_yolov8.pt` could. So the branch name
`dispenser-localization` is accurate again, not just history.

One landmark is one point. The training README says "the vertical pipes",
plural — if several are visible, `food_dispenser` as a single landmark gets
you *one* of them plus a rising `rejected_measurement_count`, because both
`solve_landmark`'s gating and `object_positions`'s consensus grouping keep the
largest agreeing group rather than averaging. Split it into separate entries
if more than one pipe has to be mapped.

No longer a leftover: `landmark_mapper`'s `legacy_dispenser_landmark`
defaults to `food_dispenser`, which exists again, so the legacy
`/dispenser/pose` republish now actually fires.

## The two models

| | Detector (model 1) | Location prior (model 2) |
|---|---|---|
| Question | "is it in this image, and where in the image" | "given the map so far, where is it likely to be" |
| Status | **trained**, `perception/models/cows_pipes_yolov8.pt` | **not trained**, code complete |
| Called | object detection | semantic map completion |
| Required? | yes, nothing works without it | no, off by default |

The detector is never asked for distance. A camera measures direction, not
range; a model asked for range invents plausible numbers. Range comes from
geometry.

`cows_pipes_yolov8.pt` (`511a9c0`, replacing `cow_yolov8.pt` from `40e7cf3`)
is YOLOv8s detection, 640 input, classes `cow_head`/`cow_body`/`pipe`, conf
0.35. Training code is in [`cow_pipe_detector/`](cow_pipe_detector/README.md),
which also has a ROS-free `test_camera.py` for trying it on a laptop webcam.

The two halves of its training set are not equally trustworthy. The cows are
170 frames from 6 of **this arena's** videos, reviewed in CVAT. The pipes are
134 images from someone else's Roboflow dataset (`pipe-detection-rdfwn` v1,
polygons converted to boxes) — so the pipe class has never seen this arena,
and arena furniture with a strong vertical edge is the false positive to
expect. Its own manifest
(`perception/models/cows_pipes_yolov8.yaml`) is honest that `val_map50`,
`val_map50_95` and `test_map50_pipe` are all still TODO, that val frames come
from the same recordings as training frames so the score is optimistic, and
that Jetson latency is unmeasured.

Collecting data and training the prior: [`landmark_prior/README.md`](landmark_prior/README.md).
The detector contract: [`perception/docs/landmark_model_handoff.md`](ros2_ws/src/perception/docs/landmark_model_handoff.md).

## Decisions not to undo

**Only JSON crosses the WiFi.** The detector publishes boxes plus resolved
intrinsics plus, with a depth camera, the range it already sampled. The mapper
therefore needs no camera topics at all. The arena WiFi is congested and
already carries lidar and Nav2.

**The mapper runs where TF lives.** It does TF lookups at each image's *own*
timestamp, not "now", because the robot is usually turning and 200 ms is
several degrees of bearing. The project has already learned that TF cannot be
split across the WiFi — see the hardware section.

**Geometry locates things, not a model.** Pixel → bearing (intrinsics) →
map-frame sight line (TF) → plus one range constraint → least squares over all
views. See the module docstring in
`imdl_autonomy/imdl_autonomy/landmark_estimation.py`, which explains why
information form rather than averaging: two views from nearly the same spot
must report a long thin uncertainty ellipse, not a confident wrong answer.

**A prior is never "converged".** `Prior` enters the solve as one more
information contribution, so real bearings outvote it in proportion to their
own information, and a stale or wrong prior cannot survive contact with
observations. It is never reported as a measurement the robot made.

**The prior network's input excludes the camera detections.** Feeding them in
would double count: the estimator already uses those bearings as a likelihood,
and a prior that had secretly seen them would report a tight covariance it had
not earned.

**The prior window is robot-centric and heading-aligned**, not map-aligned,
because the map frame's origin is wherever SLAM started. `WINDOW_CELLS` and
`WINDOW_RESOLUTION_M` are constants, not parameters: changing them invalidates
every recorded sample and every trained network.

**`frontier_explorer.py` stays diffable against upstream.** It was ported from
the AckermannRover project with its logic unchanged. The camera-coverage
handover added 15 lines and changed none — a latched `/frontier_explorer/done`
publisher, published from `_return_to_start()`, which all three completion
paths already route through. Keep any future change there additive.

**The two explorers never drive at once.** Both send `navigate_to_pose` goals,
so `camera_coverage_explorer` waits for `/frontier_explorer/done`. Only pass
`start_mode:=immediate` when the frontier explorer is not running.

## Hardware facts that cost hours

Most of these are also in the user's memory file; they are repeated here
because they are easy to rediscover the hard way.

**Addresses change (DHCP), and they changed again mid-session.** As of
2026-10-25, second check: Jetson `jetson@10.23.60.116` (ARP-confirmed, was
`.90`), laptop `frl@10.23.60.146`, feeding-station Pi `pi@10.23.60.130`
(was `.114`, and `.114` is now **this Mac** — so an address can be reassigned
to a different machine, not just go quiet). Create3 namespace `/Team2`,
`ROS_DOMAIN_ID=2`. Never trust an address in a comment, this one included.

`arp -a -n | grep 10.23.60` lists what is actually on the subnet, and works
even from an app that cannot open a socket to it — see the macOS note below.
A Raspberry Pi answers with a `2c:cf:67:…` / `b8:27:eb:…` / `dc:a6:32:…` MAC.

**The TF chain must live on one machine.** Splitting slam_toolbox from Nav2
made slam_toolbox drop every scan with `queue is full` while Nav2 missed
transforms by 0.6–2.2 s. Sensor topics tolerate WiFi lag; TF does not.

**Nav2 bringup races the Create3's odometry.** `/Team2/odom` can take ~30 s to
start and none flows while the robot is docked or asleep. If odom is late, the
local costmap activation fails *permanently*. Always confirm
`ros2 topic hz /Team2/odom` before starting Nav2.

**The Jetson has no colcon workspace for this repo.** Everything on it is
launched by path out of `~/imdl-main`, so a `git pull` there is the whole
deployment. That is why `landmark_detector_node.py` is run with `python3`
directly and given `landmark_config` explicitly, and why both new nodes have
`if __name__ == '__main__'` guards.

**`camera_pitch` does not tilt the camera down.** With the optical-frame
convention these launch files use, `pitch` rotates the image about the view
axis and leaves the view direction straight ahead. Downward tilt is
`roll = -(1.5707963 + tilt_in_radians)`. Verified numerically.

**Measured camera mounting** (2026-10-25), relative to the **lidar**, because
`lidar.launch.py` stamps scans with `frame_id: base_link` so SLAM treats the
lidar's centre as `base_link`'s origin:

```
IMDL_CAM_X=0.08   IMDL_CAM_Y=0.0   IMDL_CAM_Z=0.35   IMDL_CAM_ROLL=-1.5707963
```

Camera 50 cm off the ground, lidar 15 cm, both centred in y, camera level.
Consequence: a level camera at 50 cm cannot see floor nearer than
`0.50 / tan(27.9°) ≈ 0.95 m`, so the coverage pass wants
`camera_min_range_m:=1.0`, not its 0.30 default.

**With a level camera, only horizontal aiming matters.** The 2-D fusion
discards the vertical component, so a pixel's azimuth is identical at the top,
middle and bottom of the frame. Aim left-right only.

**macOS blocks LAN access per app.** Claude's terminal could not reach any
robot while Terminal.app could, with a valid kernel route and a resolved ARP
entry. That is System Settings → Privacy & Security → **Local Network**. It
makes `ping`, `nc` and `ssh` all fail with `No route to host`.

Diagnosing it: `arp -n <ip>` resolving to a MAC while `ping` gets nothing is
the signature — the kernel's ARP is unaffected, only the socket is blocked.
An address with nothing on it reads `(incomplete)` instead.

**So start the tmux session from Terminal.app, not from Claude.** The launcher
itself only *types* commands into panes and needs no network, but every pane
is a child of whichever process started the tmux server and inherits its
Local Network permission. Started from a blocked app, all the SSH panes fail
at once and it reads as a dead network.

## Known broken, as of 2026-10-25

**Nav2 stalls on any goal.** Not a Nav2 bug. The collision monitor's
`StopZone` — which is `action_type: approach`, despite the name — projects the
footprint along the commanded velocity and scales it so the robot stops before
collision. With a point inside or touching the footprint, the scale goes to
zero in every direction, including the in-place turn that would escape. The log
signature is `Robot to approach for 1.000000 seconds away from collision`
alternating with `Robot to continue normal operation`, and
`controller_server: Failed to make progress` every ~8 s, then `spin failed`,
then the BT aborts. **This config is byte-identical on `main`**, so switching
branches does not change it.

**Teleop cannot drive forward either.** `i` and `,` produce rotation, direction
inconsistent. teleop sends `angular.z = 0` for both, so the rotation is not
commanded — it is the robot acting on its own, most likely a Create3 bump
reflex turning away from whichever bumper fired.

**Leading hypothesis for both: the camera mast.** It rises from the 15 cm lidar
to the 50 cm camera, so it passes through the lidar's scan plane — the lidar
sees a permanent obstacle stuck to the robot — and if it loads the bumper
shell, the bumper reads pressed. One physical cause, both symptoms. Not yet
confirmed; confirm by looking at a LaserScan display for a return that rotates
with the robot, and by `/Team2/hazard_detection`.

If it is the mast, the proper fix is
`imdl_simulation/scripts/scan_self_filter.py`, which is generic despite its
docstring and already does polygon self-filtering. It is only wired into the
Gazebo launch; the real stack has no self filter, and `lidar.launch.py` has no
angle masking. Note `scan_timestamp_fixer` already owns `/scan_raw → /scan`, so
a filter has to chain rather than collide with it.

**Separately, reverse is refused by design.** The Create3's `safety_override`
parameter on `/Team2/motion_control` defaults to `none`, which blocks reverse
because the cliff sensors cannot see behind. `create3_adapter.py` exists to set
it to `backup_only`.

## Running it

`scripts/imdl_tmux.sh` opens one tmux session with every terminal a run needs,
each command **typed but not executed**. Window 0 is a generated guide; read it
rather than this file for the per-window detail.

The measured camera mounting and the trained model are now the launcher's
own defaults, so a normal run needs no environment variables — only the three
addresses, which is the one thing that does change:

```bash
./scripts/imdl_tmux.sh --jetson jetson@10.23.60.116 \
  --laptop frl@<current-ip> --feeder pi@10.23.60.130
```

Windows: `guide setup jetson tools laptop status teleop objects feeder`.
Only two of them are a run: window 2 is one launch file on the Jetson
(lidar, camera, camera TF, detector) and window 4 is one on the laptop
(scan restamp, odom TF, SLAM, Nav2, twist_mux, frontier explorer, approach
controller, RViz). The rest are read-only monitors, plus teleop.

To run the real model instead of the stub:

```bash
IMDL_LANDMARK_BACKEND=ultralytics \
IMDL_LANDMARK_MODEL=~/imdl-main/ros2_ws/src/perception/models/cows_pipes_yolov8.pt
```

That needs **ultralytics installed on the Jetson**, which is deliberately not
rosdep'd — see `perception/docs/jetson_yolo_runtime.md`. The `onnx` backend
exists for when a JetPack-matched torch is more trouble than it is worth.

### Launcher traps

- **A live session makes the script a no-op.** It prints
  `session 'imdl' already exists -- attaching` and silently ignores every
  environment variable. Always `tmux kill-session -t imdl` before relaunching
  with different settings.
- **Switching branches breaks the container build.** `perception`'s `setup.py`
  globs `launch/*.py`, so a stale build directory references a launch file the
  other branch does not have and setuptools fails to copy it. Fix:
  `rm -rf /ws/build_humble /ws/install_humble` in the container, then
  `ws_build`. The whole workspace rebuilds in ~14 s.
- **Sharing a machine: use a second checkout AND a second container name.**
  `IMDL_REPO_DIR=~/imdl-dispenser` moves every robot-side path, so a
  teammate's `~/imdl-main` is never touched — `git checkout -B <branch>
  FETCH_HEAD` would otherwise reset their branch, and it aborts outright if
  they have uncommitted changes. But `run.sh` keys on the container NAME:
  with a container called `imdl` already up, it attaches to that one instead
  of mounting your checkout, and the launch files you just pulled look
  missing. So pass `IMDL_CONTAINER=imdl-dispenser` too; the launcher forwards
  it to `run.sh` and uses it for every `docker exec`. This separates the
  *code*, not the *topics*: `ROS_DOMAIN_ID=2` and namespace `/Team2` are
  fixed by the Create3, so two people still cannot drive it at once.
- **A node that owns a device outlives the SSH drop that started it.** Two
  readers on one serial port makes sllidar read its serial number fine and then
  time out on the health code with `RESULT_OPERATION_TIMEOUT`, which reads as
  broken hardware. Setup pane 3 clears them.
- Teleop's default 0.50 m/s exceeds the Create3's ~0.31 m/s limit. Press `z`
  about six times.

## Tests

166 tests, none needing a robot, a camera, a model or ROS — they are
deliberately importable with nothing but numpy, pyyaml and pytest:

```bash
cd ros2_ws/src/perception    && python3 -m pytest test/test_landmark_detection.py
cd ros2_ws/src/imdl_autonomy && python3 -m pytest test/test_landmark_estimation.py \
    test/test_landmark_pipeline.py test/test_landmark_prior.py \
    test/test_landmark_wiring.py test/test_camera_coverage.py
```

The other tests in those packages need ROS and will fail to collect on macOS.
That is expected: macOS cannot take part in a run at all (no working Humble,
and Docker Desktop has no route to the robot LAN). Drive the two Linux
machines over SSH.

`test_landmark_pipeline.py` is the one worth reading: it places a landmark,
projects it into a camera at several poses, pushes the result through the real
wire format and the real fusion, and checks both that the answer is right and
that the reported uncertainty actually bounds the true error. It also pins the
distinction that matters most — random box noise averages out across views, a
camera extrinsic error never does.

## Conventions

Python, ROS 2 Humble, `ament_python`. Comments explain *why*, not *what*, and
the existing code is dense with hard-won reasons — match that. Nodes are
non-blocking: nothing in `imdl_autonomy` sits in a loop waiting, so one slow
step cannot stall a mission. Pure geometry and statistics live in ROS-free
modules so they can be tested; the ROS node is a thin adapter over them.
