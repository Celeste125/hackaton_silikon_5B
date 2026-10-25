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
CAM_X="${IMDL_CAM_X:-0.08}"
CAM_Y="${IMDL_CAM_Y:-0.0}"
CAM_Z="${IMDL_CAM_Z:-0.35}"
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
# The trained weights ship in the repo, so the default is the real model and
# a normal run needs no environment variables at all. Fallbacks:
#   IMDL_LANDMARK_BACKEND=disabled  empty frames; camera heartbeat, no weights
#   IMDL_LANDMARK_BACKEND=stub      a fixed centred box, for mapping only --
#                                   it cannot measure expansion, so it is
#                                   deliberately ignored by the approach
#   IMDL_LANDMARK_BACKEND=onnx      when a JetPack-matched torch is a fight
STUB_LANDMARK="${IMDL_STUB_LANDMARK:-cow_enclosure}"
LANDMARK_BACKEND="${IMDL_LANDMARK_BACKEND:-ultralytics}"
CAM_WIDTH="${IMDL_CAM_WIDTH:-320}"
CAM_HEIGHT="${IMDL_CAM_HEIGHT:-240}"
CAM_FPS="${IMDL_CAM_FPS:-5}"
MAP_DIR="${IMDL_MAP_DIR:-/root/maps}"
REPO_DIR="${IMDL_REPO_DIR:-~/imdl-main}"
# Path on the ROBOT: the detector runs where the camera is plugged in.
LANDMARK_MODEL="${IMDL_LANDMARK_MODEL:-${REPO_DIR}/ros2_ws/src/perception/models/cows_pipes_yolov8.pt}"
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
    # A node that owns the lidar or the camera survives the SSH drop that
    # started it, and the next run then fails in a way that looks like broken
    # hardware: sllidar reads the serial number fine and then times out on the
    # health code, because two processes are racing for one serial port. The
    # bracketed first letters stop the patterns matching this command's own
    # line, which would otherwise kill the shell running it.
    add_setup "$(ssh_cmd "-t" "${JETSON_HOST}" \
        "pkill -f '[s]llidar_node|[v]4l2_camera_node|[l]andmark_detector_node'; sleep 1; pgrep -af '[s]llidar_node|[v]4l2_camera_node|[l]andmark_detector_node' || echo 'no stale nodes, devices free'")" \
        "clear stale nodes still holding the lidar or camera (run after any crash or rebuild)"
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
  sensors+model  ${JETSON_HOST}
  SLAM+Nav2+RViz ${LAPTOP_HOST}, inside the '${CONTAINER}' container
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

  2  jetson   ONE launch file on the robot: lidar, camera, camera TF and
              the detector. Out of the checkout, because the Jetson has
              no colcon workspace -- a git pull there is the whole
              deployment. Put the robot on its start mark first; that
              pose becomes map (0, 0, yaw 0).
              Camera ${CAM_WIDTH}x${CAM_HEIGHT} at ${CAM_FPS} fps, NOT VGA: a raw frame has
              to arrive as ~650 intact UDP packets or it is dropped
              whole, and VGA over this WiFi leaves RViz frozen on one
              picture. Smaller still with IMDL_CAM_WIDTH/HEIGHT.
              Detector backend '${LANDMARK_BACKEND}', weights
                ${LANDMARK_MODEL}
              Three classes: cow_head and cow_body map to the
              cow_enclosure landmark, and pipe to food_dispenser --
              the dispenser's model class is called 'pipe'.
              Needs ultralytics ON THE JETSON; it is deliberately not
              rosdep'd (perception/docs/jetson_yolo_runtime.md). If
              that is a fight, IMDL_LANDMARK_BACKEND=onnx, or
              =disabled to explore with no detector at all.
  3  tools    Three panes, all on demand: the odom rate (use it before
              window 4 -- see the gate below), the map saver, and a
              camera check.
  4  laptop   ONE launch file on the nav machine: scan restamp, odom TF,
              SLAM, Nav2, twist_mux, the frontier explorer, the approach
              controller, and RViz on ${RVIZ_WHERE} screen. All of it
              together because the whole TF chain has to run on one
              clock -- TF cannot cross the WiFi within its 0.2 s
              tolerances. Wait for "Managed nodes are active" (10-20 s).
              In RViz set the Map display's Color Scheme to 'raw'.
              Then it drives itself: a real detection stops exploration,
              aligns, takes a short straight measurement, saves map
              coordinates, and resumes. A blocked or imprecise probe is
              retried from another viewpoint. Saved targets can be
              remeasured from new viewpoints to refine them; unmeasured
              targets go first.
              Add rviz:=false to run RViz separately, or
              require_detector:=false to let Nav2 drive goals with no
              detector running -- nothing can be measured in that mode.
  5  status   Read-only: /approach/status -- state, target, travel,
              range and its sigma, and what has been saved so far.
  6  teleop   Only when you need it. Press z about 6 times to slow
              down. Outranks Nav2 at once, downstream of everything
              above, so it still drives when the approach gate does
              not. Nav2 drives again 0.5 s after your last key press.
  7  objects  Read-only: /approach/object_positions -- fused coordinates,
              view counts and estimated uncertainty, plus the JSON file
              path on the nav machine. RViz shows the same points under
              'Measured Objects' on /approach/object_markers.
  8  feeder   Only with --feeder. Two panes: the mixer load cell on
              the Pi (its GPIO, so nowhere else will do) and the silo
              bookkeeping on the nav side. Check it with:
                ros2 topic echo /silo/status
              Assumes the Pi's own workspace is built.

  Only 2 and 4 are a run: press Enter in 2, clear the odom gate below,
  then press Enter in 4 and watch RViz. Both keep running on their own.

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

  Emergency stop: ctrl-c in the laptop window, then in window 3
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
        # Retile after every split, not once at the end: tmux splits the
        # *current* pane, so without rebalancing each one is half the last and
        # the fifth fails with "no space for a new pane". That used to abort
        # the whole script under set -e and leave a session with two windows,
        # which looks like a launcher bug rather than a full window.
        if ! tmux split-window -t "${SESSION}:setup" 2>/dev/null; then
            echo "setup pane $((i + 1)) did not fit in this terminal;" >&2
            echo "  its command is in the guide (window 0). Enlarge and rerun" >&2
            echo "  to get a pane for it." >&2
            continue
        fi
        tmux select-layout -t "${SESSION}:setup" tiled >/dev/null
        tmux send-keys -t "${SESSION}:setup" "${setup_cmds[i]}"
    done
    tmux select-layout -t "${SESSION}:setup" tiled >/dev/null
fi

# 2  jetson: lidar, camera, camera TF and the detector, in one launch file.
#
# By path out of the checkout, not 'ros2 launch perception ...': the Jetson has
# no colcon workspace for this repo, so a git pull there is the whole
# deployment. jetson_bringup resolves its siblings relative to itself for the
# same reason, and runs the detector as a plain python3 process.
#
# The lidar inside it publishes /scan_raw with fix_timestamps:=false. The
# Jetson's own workspace holds an older copy that stamps lidar_link and
# publishes straight to /scan, whose frames reached slam_toolbox ~0.28 s in the
# future on the Jetson's clock, so every scan was dropped. The nav side
# restamps instead.
if [[ -n "${JETSON_HOST}" ]]; then
    JETSON_SRC="${REPO_DIR}/ros2_ws/src"
else
    JETSON_SRC="/ws/src"
fi
tmux new-window -t "${SESSION}" -n jetson
tmux send-keys -t "${SESSION}:jetson" \
    "$(robot_side "ros2 launch ${JETSON_SRC}/perception/launch/jetson_bringup.launch.py backend:=${LANDMARK_BACKEND} model:=${LANDMARK_MODEL} stub_landmark:=${STUB_LANDMARK} width:=${CAM_WIDTH} height:=${CAM_HEIGHT} fps:=${CAM_FPS} x:=${CAM_X} y:=${CAM_Y} z:=${CAM_Z} roll:=${CAM_ROLL} pitch:=${CAM_PITCH} yaw:=${CAM_YAW}")"

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

# 4  laptop: scan restamp, odom TF, SLAM, Nav2, twist_mux, the frontier
# explorer, the approach controller and RViz. One launch file because they
# belong on one machine: splitting the TF chain across the WiFi made
# slam_toolbox drop every scan with 'queue is full' while Nav2 missed
# transforms by 0.6-2.2 s. Pass rviz:=false to run RViz somewhere else.
tmux new-window -t "${SESSION}" -n laptop
tmux send-keys -t "${SESSION}:laptop" \
    "$(nav_side "ros2 launch imdl_autonomy laptop_bringup.launch.py namespace:=${NAMESPACE} map_id:=${MAP_NAME} lidar_yaw:=${LIDAR_YAW}" x11)"

# 5  status. Read-only. The legacy camera_coverage.launch.py sends its own
# Nav2 goals and must not run alongside this workflow.
tmux new-window -t "${SESSION}" -n status
tmux send-keys -t "${SESSION}:status" \
    "$(nav_side 'ros2 topic echo /approach/status')"

tmux new-window -t "${SESSION}" -n teleop
tmux send-keys -t "${SESSION}:teleop" \
    "$(robot_side "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=${NAMESPACE}/cmd_vel_teleop")"

# 7  objects. Read-only: the saved coordinates, as JSON.
tmux new-window -t "${SESSION}" -n objects
tmux send-keys -t "${SESSION}:objects" \
    "$(nav_side 'ros2 topic echo /approach/object_positions')"

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
