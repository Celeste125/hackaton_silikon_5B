#!/usr/bin/env bash
# Open one tmux session with every terminal a live SLAM + navigation run needs.
#
# Two machines (the usual case): the Jetson carries the lidar, a Linux laptop
# runs Nav2 and RViz inside the imdl container.
#
#   ./scripts/imdl_tmux.sh --jetson jetson@10.23.60.90 --laptop frl@10.23.60.77
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
SINGLE_HOST=""
USE_LOCAL_DOCKER=false
DOMAIN="${ROS_DOMAIN_ID:-2}"
NAMESPACE="${IMDL_NAMESPACE:-/Team2}"
LIDAR_YAW="${IMDL_LIDAR_YAW:-3.14159265359}"
MAP_NAME="${IMDL_MAP_NAME:-my_map}"
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
{
cat <<GUIDE_TEXT
  Live SLAM + navigation -- press Enter in each window, in this order
  ===================================================================

  ROS_DOMAIN_ID  ${DOMAIN}
  namespace      ${NAMESPACE}
GUIDE_TEXT
if [[ "${SPLIT}" == true ]]; then
cat <<GUIDE_TEXT
  lidar + SLAM   ${JETSON_HOST}
  Nav2 + RViz    ${LAPTOP_HOST}, inside the '${CONTAINER}' container

  Once per laptop, then skip these two:

  1  setup    Fetch ${BRANCH}, build the image, start the container.
              This window OWNS the container: leave it running, and
              everything else dies if you Ctrl-C it.
  2  build    ws_build inside it. Needed after any new launch file.

  Every run:

  3  lidar    Jetson: the lidar only. Put the robot on its start mark
              first; that pose becomes map (0, 0, yaw 0).
  4  nav      SLAM and Nav2 together, plus twist_mux for teleop. The
              whole TF chain runs here, on one clock, because TF
              cannot cross the WiFi within its 0.2 s tolerances.
              Wait for "Managed nodes are active" (10-20 s).
  5  rviz     Opens on the LAPTOP's screen, not here. Set the Map
              display's Color Scheme to 'raw', then 2D Goal Pose.
  6  teleop   Only when you need it. Press z about 6 times to slow down.
              Outranks Nav2 at once; Nav2 drives again 0.5 s after your
              last key press.
  7  tools    Left: save the map (while window 4 still runs).
              Right: check the odom feed.
  8  explore  Optional: the robot maps the arena by itself. Only after
              window 4 says active. It drives as soon as you press
              Enter; teleop (6) still overrides it, ctrl-c cancels.
  9  camera    RealSense driver + the perception pipeline. Needed by
              windows 10 and 11.
 10  cam-check Left: are the camera topics publishing? Right: the live
              picture (rqt_image_view). Start here if unsure.
 11  dispenser Asks Claude which pixels are the food dispenser, then
              uses aligned depth + TF for the position. Take a view,
              from 2-3 angles:
                ros2 topic pub --once /dispenser/locate std_msgs/msg/Empty "{}"
              Needs ANTHROPIC_API_KEY in the LAPTOP's environment before
              window 1 starts the container -- put the export in the
              laptop's ~/.bashrc. Exporting it in a container shell does
              not reach this window: every window opens its own shell and
              gets the environment the container was started with.

  BEFORE window 4, two gates. Nav2 activates its local costmap during
  bringup and that needs the odom -> base_link TF; if odom is not
  flowing yet the activation fails for good and no amount of waiting
  fixes it. The Create3 can take ~30 s to start publishing odom, and
  publishes none while asleep or docked.

  1) wake it: press the Create3's centre button. Docked? In window 2:
       ros2 action send_goal ${NAMESPACE}/undock irobot_create_msgs/action/Undock "{}"
  2) in window 7 (right pane), confirm odom actually streams:
       ros2 topic hz ${NAMESPACE}/odom
     Wait for a steady rate. Only then press Enter in window 4.

  A docked or sleeping robot also refuses to drive forward while still
  turning, so if teleop only turns, start here too.
GUIDE_TEXT
else
cat <<GUIDE_TEXT

  1  lidar    The lidar only. Put the robot on its start mark first;
              that pose becomes map (0, 0, yaw 0).
  2  nav      Nav2 on the live SLAM map, plus twist_mux for teleop.
              Wait for "Managed nodes are active" (10-20 s).
  3  rviz     Set the Map display's Color Scheme to 'raw', then use
              2D Goal Pose to send the robot somewhere.
  4  teleop   Only when you need it. Press z about 6 times to slow down.
  5  tools    Save the map, and check the odom feed.
  6  explore  Optional: the robot maps the arena by itself. Only after
              window 2 says active. It drives as soon as you press
              Enter; teleop still overrides it, ctrl-c cancels.
GUIDE_TEXT
fi
cat <<GUIDE_TEXT

  switch window   ctrl-b <number>         detach  ctrl-b d
  stop a window   ctrl-b <number> ctrl-c  kill    tmux kill-session -t ${SESSION}

  Emergency stop: ctrl-c in the nav window, then in the tools window
    ros2 topic pub --rate 10 ${NAMESPACE}/cmd_vel geometry_msgs/msg/Twist "{}"
GUIDE_TEXT
} > "${GUIDE}"

tmux new-session -d -s "${SESSION}" -n guide
tmux send-keys -t "${SESSION}:guide" "clear && cat ${GUIDE}" C-m

if [[ "${SPLIT}" == true ]]; then
    # Fetch straight from the URL: no remote to reconfigure, no GitHub key
    # needed on the laptop. DISPLAY/XAUTHORITY let run.sh's xhost reach the
    # laptop's own X session from this SSH shell, so RViz can open there.
    tmux new-window -t "${SESSION}" -n setup
    tmux send-keys -t "${SESSION}:setup" "$(ssh_cmd "-t" "${LAPTOP_HOST}" \
        "cd ${REPO_DIR} && git fetch ${REPO_URL} ${BRANCH} && git checkout -B ${BRANCH} FETCH_HEAD && cd docker && DISPLAY=:0 XAUTHORITY=\$HOME/.Xauthority ./run.sh --build")"

    tmux new-window -t "${SESSION}" -n build
    tmux send-keys -t "${SESSION}:build" "$(in_container 'ws_build')"
fi

tmux new-window -t "${SESSION}" -n lidar
tmux send-keys -t "${SESSION}:lidar" \
    "$(robot_side 'ros2 launch slam lidar.launch.py fix_timestamps:=false')"

tmux new-window -t "${SESSION}" -n nav
tmux send-keys -t "${SESSION}:nav" \
    "$(nav_side 'ros2 launch imdl_autonomy slam_navigation.launch.py')"

tmux new-window -t "${SESSION}" -n rviz
tmux send-keys -t "${SESSION}:rviz" \
    "$(nav_side "ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=${LIDAR_YAW}" x11)"

tmux new-window -t "${SESSION}" -n teleop
tmux send-keys -t "${SESSION}:teleop" \
    "$(robot_side "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=${NAMESPACE}/cmd_vel_teleop")"

# Two panes: the map saver on the left, a free shell for checks on the right.
tmux new-window -t "${SESSION}" -n tools
tmux send-keys -t "${SESSION}:tools" \
    "$(nav_side "ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/${MAP_NAME}")"
tmux split-window -h -t "${SESSION}:tools"
tmux send-keys -t "${SESSION}:tools.1" \
    "$(nav_side "ros2 topic hz ${NAMESPACE}/odom")"

# Autonomous exploration. Runs where Nav2 runs, since it drives through the
# navigate_to_pose action and reads /map. Last, so the numbers above are stable.
tmux new-window -t "${SESSION}" -n explore
tmux send-keys -t "${SESSION}:explore" \
    "$(nav_side 'ros2 launch imdl_autonomy frontier_explore.launch.py')"

# Camera, its diagnostics, and dispenser localisation. All on the Nav2 side:
# the locator needs the map frame and the camera on the same machine.
tmux new-window -t "${SESSION}" -n camera
tmux send-keys -t "${SESSION}:camera" \
    "$(nav_side 'ros2 launch perception camera.launch.py' x11)"

# Two panes: left answers "is the camera publishing", right shows the picture.
tmux new-window -t "${SESSION}" -n cam-check
tmux send-keys -t "${SESSION}:cam-check" \
    "$(nav_side 'bash $(ros2 pkg prefix imdl_autonomy)/share/imdl_autonomy/scripts/check_camera.sh')"
tmux split-window -h -t "${SESSION}:cam-check"
tmux send-keys -t "${SESSION}:cam-check.1" \
    "$(nav_side 'rqt_image_view /camera/camera/color/image_raw' x11)"

tmux new-window -t "${SESSION}" -n dispenser
tmux send-keys -t "${SESSION}:dispenser" \
    "$(nav_side 'ros2 launch imdl_autonomy dispenser_locate.launch.py')"

tmux select-window -t "${SESSION}:guide"
exec tmux attach -t "${SESSION}"
