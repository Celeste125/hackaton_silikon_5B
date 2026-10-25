#!/usr/bin/env bash
# Open one tmux session with every terminal a live SLAM + navigation run needs.
#
# Two machines (the usual case): the Jetson carries the lidar, a Linux laptop
# runs Nav2 and RViz inside the imdl container.
#
#   ./scripts/imdl_tmux.sh --jetson jetson@10.23.60.90 --laptop frl@10.23.60.77
#
# Add the feeding station's Pi, which carries the mixer load cell, to get its
# window too:
#
#   ./scripts/imdl_tmux.sh --jetson jetson@10.23.60.90 \
#       --laptop frl@10.23.60.77 --feeder pi@10.23.60.114
#
# One machine, no container (everything on the Jetson):
#
#   ./scripts/imdl_tmux.sh --host jetson@10.23.60.90
#
# This machine's own container (you are on Linux):
#
#   ./scripts/imdl_tmux.sh --docker
#
# Every window comes up with its command typed in but NOT executed. Read the
# guide window, then press Enter window by window in the order it lists, so the
# start order stays yours.
#
# macOS note: --docker cannot work there. Docker Desktop puts containers on the
# VM's network, which has no route to the robot's LAN. Use --jetson/--laptop.
set -euo pipefail

SESSION=imdl
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_HOST=""
LAPTOP_HOST=""
FEEDER_HOST=""
SINGLE_HOST=""
USE_LOCAL_DOCKER=false
DOMAIN="${ROS_DOMAIN_ID:-2}"
NAMESPACE="${IMDL_NAMESPACE:-/Team2}"
LIDAR_YAW="${IMDL_LIDAR_YAW:-3.14159265359}"
MAP_NAME="${IMDL_MAP_NAME:-my_map}"
# Where the C920 sits on the robot, base_link -> webcam_optical_frame. The
# roll/yaw pair turns base_link (x forward, z up) into the optical convention
# (z forward, y down). MEASURE x/y/z: with bearing-only geometry an angular
# error scales with range and averaging will not remove it.
CAM_X="${IMDL_CAM_X:-0.10}"
CAM_Y="${IMDL_CAM_Y:-0.0}"
CAM_Z="${IMDL_CAM_Z:-0.10}"
CAM_ROLL="${IMDL_CAM_ROLL:--1.5707963}"
CAM_PITCH="${IMDL_CAM_PITCH:-0.0}"
CAM_YAW="${IMDL_CAM_YAW:--1.5707963}"
# Frame size for the camera window. Deliberately not VGA: a raw 640x480 rgb8
# frame is ~900 KB, which DDS splits into ~650 UDP packets, and a frame is
# discarded whole if any one of them is lost. Across the arena WiFi that shows
# up as a single frozen picture in the laptop's RViz while the Jetson's own
# RViz stays live, because a local subscriber reads shared memory instead.
# 320x240 is a quarter of the bytes; drop to IMDL_CAM_WIDTH=160
# IMDL_CAM_HEIGHT=120 if it still stutters.
# Which landmark the stub detector reports, and how much of the frame its box
# covers. The stub needs no trained model: aim the camera at the real object
# and it lands in the middle of the frame, which is what a centred box means.
# Set IMDL_LANDMARK_BACKEND=ultralytics IMDL_LANDMARK_MODEL=~/models/x.pt once
# the weights exist.
STUB_LANDMARK="${IMDL_STUB_LANDMARK:-cow_enclosure}"
LANDMARK_BACKEND="${IMDL_LANDMARK_BACKEND:-stub}"
LANDMARK_MODEL="${IMDL_LANDMARK_MODEL:-}"
CAM_WIDTH="${IMDL_CAM_WIDTH:-320}"
CAM_HEIGHT="${IMDL_CAM_HEIGHT:-240}"
CAM_FPS="${IMDL_CAM_FPS:-5}"
MAP_DIR="${IMDL_MAP_DIR:-/root/maps}"
REPO_DIR="${IMDL_REPO_DIR:-~/imdl-main}"
# Default to whatever branch this checkout is on, so the laptop gets the code
# you are actually running, not whichever branch was current when this was
# written. Override with IMDL_BRANCH.
BRANCH="${IMDL_BRANCH:-$(git -C "${SCRIPT_DIR:-.}/.." rev-parse --abbrev-ref HEAD 2>/dev/null || echo main)}"
# Public HTTPS URL, so the laptop needs no GitHub key of its own.
REPO_URL="${IMDL_REPO_URL:-https://github.com/Celeste125/hackaton_silikon_5B.git}"
CONTAINER="${IMDL_CONTAINER:-imdl}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --jetson) JETSON_HOST="$2"; shift 2 ;;
        --laptop) LAPTOP_HOST="$2"; shift 2 ;;
        --feeder) FEEDER_HOST="$2"; shift 2 ;;
        --host) SINGLE_HOST="$2"; shift 2 ;;
        --docker) USE_LOCAL_DOCKER=true; shift ;;
        --session) SESSION="$2"; shift 2 ;;
        -h|--help) sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 1 ;;
    esac
done

if [[ -n "${SINGLE_HOST}" ]]; then
    JETSON_HOST="${SINGLE_HOST}"
elif [[ -z "${JETSON_HOST}" && "${USE_LOCAL_DOCKER}" == false ]]; then
    echo "error: pass --jetson/--laptop, or --host, or --docker (see --help)" >&2
    exit 1
fi

command -v tmux >/dev/null || { echo "error: tmux is not installed" >&2; exit 1; }

# Single-quote for the shell, keeping the result readable: these commands are
# typed into the panes for a human to check before pressing Enter.
sq() { printf "'%s'" "${1//\'/\'\\''}"; }

# Put the branch on a machine, first time or tenth: clone if the directory is
# not there, then move the branch to whatever the remote has. The Jetson gets
# it the same way as the laptop, so the camera launch file is pulled rather
# than copied, and nothing on either machine has to be built to run it.
# Straight from the URL: no remote to configure and no GitHub key needed.
repo_sync() {
    printf '[ -d %s ] || git clone %s %s; cd %s && git fetch %s %s && git checkout -B %s FETCH_HEAD' \
        "${REPO_DIR}" "${REPO_URL}" "${REPO_DIR}" \
        "${REPO_DIR}" "${REPO_URL}" "${BRANCH}" "${BRANCH}"
}

# Hand a whole command line to a remote shell as ONE argument. ssh joins its
# arguments with spaces and drops the local quoting, so the remote shell
# re-parses whatever is left: without this outer layer, a nested bash -ic '...'
# is split and its tail runs on the remote host instead of being passed on.
# $ is escaped so the remote, not this machine, expands it.
ssh_cmd() {
    local flags="$1" host="$2" remote="$3"
    printf 'ssh %s %s "%s"' "${flags}" "${host}" "${remote//\$/\\\$}"
}

# A ROS command on a host's own system. bash -ic, not plain 'ssh host cmd':
# ROS lives in ~/.bashrc, which a non-interactive remote shell never reads.
on_host() {
    local host="$1" want_x11="$2" cmd="$3" flags="-t"
    [[ "${want_x11}" == "x11" ]] && flags="-Y -t"
    ssh_cmd "${flags}" "${host}" "$(printf 'bash -ic %s' \
        "$(sq "export ROS_DOMAIN_ID=${DOMAIN}; ${cmd}")")"
}

# A ROS command inside the imdl container, which already holds the workspace
# and the ROS environment. Locally when --docker, otherwise over SSH.
in_container() {
    local cmd="$1"
    local inner
    inner="$(printf 'docker exec -it %s bash -ic %s' "${CONTAINER}" \
        "$(sq "export ROS_DOMAIN_ID=${DOMAIN}; ${cmd}")")"
    if [[ "${USE_LOCAL_DOCKER}" == true ]]; then
        printf '%s' "${inner}"
    else
        ssh_cmd "-t" "${LAPTOP_HOST}" "${inner}"
    fi
}

# Where the lidar, SLAM and teleop run: the Jetson when there is one,
# otherwise the container (simulation or a rosbag replay).
robot_side() {
    local cmd="$1"
    if [[ -n "${JETSON_HOST}" ]]; then
        on_host "${JETSON_HOST}" plain "${cmd}"
    else
        in_container "${cmd}"
    fi
}

# The feeding station's Pi: the mixer load cell hangs off its GPIO, so the
# driver has to run there and nowhere else.
feeder_side() {
    on_host "${FEEDER_HOST}" plain "$1"
}

# Where Nav2, RViz and the map saver run.
nav_side() {
    local cmd="$1" want_x11="${2:-plain}"
    if [[ -n "${LAPTOP_HOST}" || "${USE_LOCAL_DOCKER}" == true ]]; then
        in_container "${cmd}"
    else
        on_host "${JETSON_HOST}" "${want_x11}" "${cmd}"
    fi
}

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "session '${SESSION}' already exists -- attaching"
    exec tmux attach -t "${SESSION}"
fi

SPLIT=false
[[ -n "${LAPTOP_HOST}" && -n "${JETSON_HOST}" ]] && SPLIT=true

GUIDE="$(mktemp -t imdl_guide)"

# Every one-off job is a pane of the 'setup' window, so a normal run never
# opens one. Built before the guide, which lists the same panes from
# setup_help, and so cannot drift from what is actually in the window.
setup_cmds=()
setup_help=""
add_setup() {
    setup_cmds+=("$1")
    setup_help="${setup_help}
              - $2"
}
if [[ "${SPLIT}" == true ]]; then
    # DISPLAY/XAUTHORITY let run.sh's xhost reach the laptop's own X session
    # from this SSH shell, so RViz can open on its screen.
    add_setup "$(ssh_cmd "-t" "${LAPTOP_HOST}" \
        "$(repo_sync) && cd docker && DISPLAY=:0 XAUTHORITY=\$HOME/.Xauthority ./run.sh --build")" \
        "pull the code + start the container (LEAVE IT RUNNING)"
    add_setup "$(in_container 'ws_build')" \
        "ws_build, after a new or changed launch file"
elif [[ "${USE_LOCAL_DOCKER}" == true ]]; then
    add_setup "$(in_container 'ws_build')" \
        "ws_build, after a new or changed launch file"
fi
if [[ -n "${JETSON_HOST}" ]]; then
    add_setup "$(robot_side 'v4l2-ctl --list-devices; sudo apt install -y ros-humble-v4l2-camera v4l-utils')" \
        "the Jetson's camera list, then its driver (wants sudo)"
    add_setup "$(ssh_cmd "-t" "${JETSON_HOST}" "$(repo_sync)")" \
        "pull the code on the Jetson (it runs the camera from there)"
fi

if [[ "${SPLIT}" == true ]]; then
    RVIZ_WHERE="the LAPTOP's"
else
    RVIZ_WHERE="its own machine's"
fi

{
cat <<GUIDE_TEXT
  Live SLAM + navigation -- press Enter in each window, in this order
  ===================================================================

  ROS_DOMAIN_ID  ${DOMAIN}
  namespace      ${NAMESPACE}
GUIDE_TEXT
if [[ "${SPLIT}" == true ]]; then
cat <<GUIDE_TEXT
  lidar + camera ${JETSON_HOST}
  Nav2 + RViz    ${LAPTOP_HOST}, inside the '${CONTAINER}' container
GUIDE_TEXT
else
cat <<GUIDE_TEXT
  everything     ${JETSON_HOST:-this machine, in the '${CONTAINER}' container}
GUIDE_TEXT
fi
cat <<GUIDE_TEXT

  1  setup    Once per machine -- afterwards only the pull panes, and
              only when there is new code. One job per pane, and
              ctrl-b o cycles them.${setup_help}

  Then, every run:

  2  lidar    The lidar only, out of the checkout -- the Jetson's own
              workspace has an older copy whose scans slam_toolbox
              drops. Put the robot on its start mark first; that pose
              becomes map (0, 0, yaw 0).
  3  tools    Three panes, all on demand: the odom rate (use it before
              window 4 -- see the gate below), the map saver, and a
              camera check.
  4  nav      SLAM and Nav2 together, plus twist_mux for teleop. The
              whole TF chain runs here, on one clock, because TF
              cannot cross the WiFi within its 0.2 s tolerances.
              Wait for "Managed nodes are active" (10-20 s).
  5  rviz     Opens on ${RVIZ_WHERE} screen, not in this terminal. Set
              the Map display's Color Scheme to 'raw', then 2D Goal
              Pose. The 'Webcam' panel fills in once window 6 runs.
  6  camera   The C920 and its mounting transform, one pane, run
              straight out of the Jetson's checkout -- no colcon there.
              ${CAM_WIDTH}x${CAM_HEIGHT} at ${CAM_FPS} fps, NOT VGA: a raw frame has to arrive
              as ~650 intact UDP packets or it is dropped whole, and
              VGA over this WiFi leaves RViz frozen on a single
              picture. Smaller still with IMDL_CAM_WIDTH/HEIGHT.
  7  explore  The robot maps the arena by itself. Only after window 4
              says active. It drives as soon as you press Enter;
              teleop (8) still overrides it, ctrl-c cancels.
  8  coverage The lidar sweep in window 7 finishes when the LIDAR has
              seen everything; the lidar sees 360 degrees and the camera
              about 70 out of the front, so the dispenser may never have
              been in frame. Start this at the same time as 7. It banks
              coverage from the start whoever is driving, then takes
              over when 7 is done and drives until the camera has swept
              the floor. Watch it in window 3:
                ros2 topic echo /camera_coverage/status
              In RViz enable the 'Camera Coverage' map: floor the camera
              has NOT looked at shows as occupied, and should drain.
  9  teleop   Only when you need it. Press z about 6 times to slow
              down. Outranks Nav2 at once; Nav2 drives again 0.5 s
              after your last key press.
 10  detect   The landmark model, on the Jetson with the camera, so no
              frames cross the WiFi -- it publishes boxes as JSON, well
              under a kilobyte a frame. Backend '${LANDMARK_BACKEND}'.
              With the stub it needs NO trained model: it reports
              '${STUB_LANDMARK}' in a box at the centre of every frame,
              so aim the camera at the real object and the rest of the
              chain -- bearings, TF, the fusion -- is exercised for
              real. That is the point: it proves the camera mounting,
              which is the one error no averaging ever removes.
              Once the weights exist:
                IMDL_LANDMARK_BACKEND=ultralytics \\
                IMDL_LANDMARK_MODEL=~/models/imdl_landmarks.pt
11  landmark  Turns those boxes into map positions and keeps improving
              them. Runs on the nav side, where the TF chain is. Add a
              MarkerArray on /landmarks/markers in RViz: a sphere, a
              2-sigma ellipse and a label per landmark. Watch it in
              window 3 with:
                ros2 topic echo /landmarks/state
              A bearing alone is a LINE, not a position, so one view
              fixes nothing. Drive a metre sideways (window 7 or 8) and
              the ellipse should round off and shrink. An ellipse that
              stays a long streak means every view came from one side;
              /landmarks/<name>/next_view holds a pose that would fix
              it. The prior is off by default; see
              landmark_prior/README.md.
12  feeder   Only with --feeder. Two panes: the mixer load cell on
              the Pi (its GPIO, so nowhere else will do) and the silo
              bookkeeping on the nav side. Check it with:
                ros2 topic echo /silo/status
              Assumes the Pi's own workspace is built.

  2, 4, 5, 6, 7, 8, 10 and 11 keep running on their own once started.
  window 7 there is nothing left to type -- just watch RViz.

  BEFORE window 4, two gates. Nav2 activates its local costmap during
  bringup and that needs the odom -> base_link TF; if odom is not
  flowing yet the activation fails for good and no amount of waiting
  fixes it. The Create3 can take ~30 s to start publishing odom, and
  publishes none while asleep or docked.

  1) wake it: press the Create3's centre button. Docked? In window 3:
       ros2 action send_goal ${NAMESPACE}/undock irobot_create_msgs/action/Undock "{}"
  2) in window 3 (odom pane), confirm odom actually streams:
       ros2 topic hz ${NAMESPACE}/odom
     Wait for a steady rate. Only then press Enter in window 4.

  A docked or sleeping robot also refuses to drive forward while still
  turning, so if teleop only turns, start here too.

  switch window   ctrl-b <number>         detach  ctrl-b d
  stop a window   ctrl-b <number> ctrl-c  kill    tmux kill-session -t ${SESSION}

  Emergency stop: ctrl-c in the nav window, then in window 3
    ros2 topic pub --rate 10 ${NAMESPACE}/cmd_vel geometry_msgs/msg/Twist "{}"
GUIDE_TEXT
} > "${GUIDE}"

tmux new-session -d -s "${SESSION}" -n guide
tmux send-keys -t "${SESSION}:guide" "clear && cat ${GUIDE}" C-m

# 1  setup. send-keys targets the window, which means its active pane, and a
# fresh split is always active -- so this needs no pane indices.
tmux new-window -t "${SESSION}" -n setup
if [[ ${#setup_cmds[@]} -gt 0 ]]; then
    tmux send-keys -t "${SESSION}:setup" "${setup_cmds[0]}"
    for ((i = 1; i < ${#setup_cmds[@]}; i++)); do
        tmux split-window -t "${SESSION}:setup"
        tmux send-keys -t "${SESSION}:setup" "${setup_cmds[i]}"
    done
    tmux select-layout -t "${SESSION}:setup" tiled
fi

# 2  lidar, by path out of the checkout rather than 'ros2 launch slam ...'.
# The Jetson's own workspace holds an older copy that stamps lidar_link and
# publishes straight to /scan, ignoring fix_timestamps, so its frames reached
# slam_toolbox ~0.28 s in the future on the Jetson's clock and every scan was
# dropped. This copy remaps to /scan_raw for the nav side to restamp, and with
# fix_timestamps:=false the only package it needs is sllidar_ros2 -- so still
# no colcon on the Jetson.
if [[ -n "${JETSON_HOST}" ]]; then
    LIDAR_LAUNCH_CMD="ros2 launch ${REPO_DIR}/ros2_ws/src/slam/launch/lidar.launch.py"
else
    LIDAR_LAUNCH_CMD="ros2 launch /ws/src/slam/launch/lidar.launch.py"
fi
tmux new-window -t "${SESSION}" -n lidar
tmux send-keys -t "${SESSION}:lidar" \
    "$(robot_side "${LIDAR_LAUNCH_CMD} fix_timestamps:=false")"

# 3  tools: the odom gate first, because it is needed before window 4.
tmux new-window -t "${SESSION}" -n tools
tmux send-keys -t "${SESSION}:tools" \
    "$(nav_side "ros2 topic hz ${NAMESPACE}/odom")"
tmux split-window -h -t "${SESSION}:tools"
tmux send-keys -t "${SESSION}:tools" \
    "$(nav_side "ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/${MAP_NAME}")"
tmux split-window -v -t "${SESSION}:tools"
tmux send-keys -t "${SESSION}:tools" \
    "$(nav_side 'bash $(ros2 pkg prefix imdl_autonomy)/share/imdl_autonomy/scripts/check_camera.sh')"

tmux new-window -t "${SESSION}" -n nav
tmux send-keys -t "${SESSION}:nav" \
    "$(nav_side 'ros2 launch imdl_autonomy slam_navigation.launch.py')"

tmux new-window -t "${SESSION}" -n rviz
tmux send-keys -t "${SESSION}:rviz" \
    "$(nav_side "ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=${LIDAR_YAW}" x11)"

# 6  camera. One pane: the launch file carries the measured mounting pose, so
# the driver and its transform start and stop together.
#
# By path, and a launch file rather than 'ros2 run': v4l2_camera takes
# image_size as a list, and a [320,240] literal cannot survive the nested
# quoting these panes are built from, while plain integers can. The path keeps
# the Jetson free of colcon: it only has to have pulled the branch.
if [[ -n "${JETSON_HOST}" ]]; then
    WEBCAM_LAUNCH_CMD="ros2 launch ${REPO_DIR}/ros2_ws/src/perception/launch/webcam_only.launch.py"
else
    WEBCAM_LAUNCH_CMD="ros2 launch /ws/src/perception/launch/webcam_only.launch.py"
fi
tmux new-window -t "${SESSION}" -n camera
tmux send-keys -t "${SESSION}:camera" \
    "$(robot_side "${WEBCAM_LAUNCH_CMD} width:=${CAM_WIDTH} height:=${CAM_HEIGHT} fps:=${CAM_FPS} x:=${CAM_X} y:=${CAM_Y} z:=${CAM_Z} roll:=${CAM_ROLL} pitch:=${CAM_PITCH} yaw:=${CAM_YAW}")"

# Autonomous exploration. Runs where Nav2 runs, since it drives through the
# navigate_to_pose action and reads /map.
tmux new-window -t "${SESSION}" -n explore
tmux send-keys -t "${SESSION}:explore" \
    "$(nav_side 'ros2 launch imdl_autonomy frontier_explore.launch.py')"

# The camera-coverage pass. Started alongside window 7, not after it: it
# accumulates which floor the camera has looked at from the moment it runs,
# whoever is driving, and only sends its own goals once window 7 reports done.
tmux new-window -t "${SESSION}" -n coverage
tmux send-keys -t "${SESSION}:coverage" \
    "$(nav_side 'ros2 launch imdl_autonomy camera_coverage.launch.py')"

tmux new-window -t "${SESSION}" -n teleop
tmux send-keys -t "${SESSION}:teleop" \
    "$(robot_side "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=${NAMESPACE}/cmd_vel_teleop")"

# 9  detect. The landmark model, on the machine the camera is plugged into, so
# the frames never cross the WiFi -- it publishes boxes as JSON instead.
#
# Run by path with python3, not 'ros2 launch perception ...', for the same
# reason as the lidar and camera windows: the Jetson has no colcon workspace,
# and a launch file would need the package installed to find the executable.
# landmark_config is passed explicitly so the node never has to look the
# package up either.
PERCEPTION_DIR="${REPO_DIR}/ros2_ws/src/perception"
if [[ "${LANDMARK_BACKEND}" == "stub" ]]; then
    LANDMARK_ARGS="-p backend:=stub -p stub_landmark:=${STUB_LANDMARK}"
else
    LANDMARK_ARGS="-p backend:=${LANDMARK_BACKEND} -p model_path:=${LANDMARK_MODEL}"
fi
tmux new-window -t "${SESSION}" -n detect
tmux send-keys -t "${SESSION}:detect" \
    "$(robot_side "PYTHONPATH=${PERCEPTION_DIR} python3 ${PERCEPTION_DIR}/perception/landmark_detector_node.py --ros-args ${LANDMARK_ARGS} -p landmark_config:=${PERCEPTION_DIR}/config/landmarks.yaml -p image_topic:=/webcam/image_raw -p camera_info_topic:=/webcam/camera_info -p max_inference_rate_hz:=5.0")"

# 10  landmark. The fusion, on the Nav2 side, because it does TF lookups at
# each image's own timestamp and the TF chain lives there. publish_camera_tf
# is false: window 6 already publishes base_link -> webcam_optical_frame, and
# two publishers of one static transform is asking for trouble.
tmux new-window -t "${SESSION}" -n landmark
tmux send-keys -t "${SESSION}:landmark" \
    "$(nav_side 'ros2 launch imdl_autonomy landmark_mapping.launch.py publish_camera_tf:=false')"

# 11  feeder. Only with --feeder. The load cell is on the Pi's GPIO, so its
# driver cannot run anywhere else.
if [[ -n "${FEEDER_HOST}" ]]; then
    tmux new-window -t "${SESSION}" -n feeder
    tmux send-keys -t "${SESSION}:feeder" \
        "$(feeder_side 'ros2 run loadcell_wrapper loadcell_node')"
    tmux split-window -t "${SESSION}:feeder"
    tmux send-keys -t "${SESSION}:feeder" \
        "$(nav_side 'ros2 launch imdl_autonomy silo_status.launch.py')"
    tmux select-layout -t "${SESSION}:feeder" tiled
fi

tmux select-window -t "${SESSION}:guide"
exec tmux attach -t "${SESSION}"
