#!/usr/bin/env bash
# Open one tmux session with every terminal a live SLAM + navigation run needs.
#
#   ./scripts/imdl_tmux.sh --host jetson@10.23.60.90     # drive the Jetson over SSH
#   ./scripts/imdl_tmux.sh --docker                      # drive the local container
#
# Each window comes up with its command typed in but NOT executed. Read the
# guide window, then press Enter window by window in the order it lists. That
# keeps the start order (SLAM before Nav2) in your hands.
#
# On macOS only --host works: Docker Desktop puts containers on the VM's
# network, which has no route to the robot's LAN, so a container here can
# never see the Create3 or the Jetson over DDS.
set -euo pipefail

SESSION=imdl
BACKEND=""
ROS_HOST="${IMDL_ROS_HOST:-}"
DOMAIN="${ROS_DOMAIN_ID:-2}"
NAMESPACE="${IMDL_NAMESPACE:-/Team2}"
LIDAR_YAW="${IMDL_LIDAR_YAW:-3.14159265359}"
MAP_NAME="${IMDL_MAP_NAME:-my_map}"
MAP_DIR="${IMDL_MAP_DIR:-/root/maps}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host) BACKEND=ssh; ROS_HOST="$2"; shift 2 ;;
        --docker) BACKEND=docker; shift ;;
        --session) SESSION="$2"; shift 2 ;;
        -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 1 ;;
    esac
done

if [[ -z "${BACKEND}" ]]; then
    if [[ -n "${ROS_HOST}" ]]; then
        BACKEND=ssh
    else
        echo "error: pass --host user@ip (ROS on another machine) or --docker" >&2
        echo "       on macOS only --host can reach the robot; see the header." >&2
        exit 1
    fi
fi

command -v tmux >/dev/null || { echo "error: tmux is not installed" >&2; exit 1; }

# Single-quote for the shell, keeping the result readable: these commands are
# typed into the panes for a human to check before pressing Enter.
sq() { printf "'%s'" "${1//\'/\'\\''}"; }

# wrap <x11|plain> <command> -- how a ROS command reaches the machine that runs
# it. Both forms set ROS_DOMAIN_ID so a pane can never land in the wrong domain.
wrap() {
    local want_x11="$1" cmd="$2"
    # bash -ic, not plain 'ssh host cmd': ROS lives in ~/.bashrc, which a
    # non-interactive remote shell never reads, so ros2 would not be on PATH.
    if [[ "${BACKEND}" == "ssh" ]]; then
        local ssh_flags="-t"
        [[ "${want_x11}" == "x11" ]] && ssh_flags="-Y -t"
        printf 'ssh %s %s bash -ic %s' "${ssh_flags}" "${ROS_HOST}" \
            "$(sq "export ROS_DOMAIN_ID=${DOMAIN}; ${cmd}")"
    else
        printf 'docker exec -it imdl bash -ic %s' \
            "$(sq "export ROS_DOMAIN_ID=${DOMAIN}; ${cmd}")"
    fi
}

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "session '${SESSION}' already exists -- attaching"
    exec tmux attach -t "${SESSION}"
fi

if [[ "${BACKEND}" == "docker" ]]; then
    if ! docker ps --format '{{.Names}}' | grep -qx imdl; then
        echo "starting the imdl container ..."
        "${SCRIPT_DIR}/../docker/run.sh" >/dev/null 2>&1 &
        for _ in $(seq 1 30); do
            docker ps --format '{{.Names}}' | grep -qx imdl && break
            sleep 1
        done
    fi
    docker ps --format '{{.Names}}' | grep -qx imdl \
        || { echo "error: the imdl container did not come up" >&2; exit 1; }
fi

GUIDE="$(mktemp -t imdl_guide)"
cat > "${GUIDE}" <<GUIDE_TEXT
  Live SLAM + navigation -- press Enter in each window, in this order
  ===================================================================

  backend        ${BACKEND}${ROS_HOST:+  (${ROS_HOST})}
  ROS_DOMAIN_ID  ${DOMAIN}
  namespace      ${NAMESPACE}

  1  slam     lidar + slam_toolbox + odom TF. Put the robot on its start
              mark first; that pose becomes map (0, 0, yaw 0).
  2  nav      Nav2 on the live SLAM map, plus twist_mux for the teleop
              fallback. Wait for "Managed nodes are active" (10-20 s).
  3  rviz     Set the Map display's Color Scheme to 'raw', then use
              2D Goal Pose to send the robot somewhere.
  4  teleop   Only when you need it. Press z about 6 times to slow down.
              It outranks Nav2 instantly; Nav2 drives again 0.5 s after
              your last key press.
  5  tools    Save the map (while window 2 is still running), and a few
              checks for when something looks wrong.

  switch window   ctrl-b <number>        detach   ctrl-b d
  stop a window   ctrl-b <number> ctrl-c  kill it  tmux kill-session -t ${SESSION}

  Emergency stop: ctrl-c in window 2, then in window 5
    ros2 topic pub --rate 10 ${NAMESPACE}/cmd_vel geometry_msgs/msg/Twist "{}"
GUIDE_TEXT

tmux new-session -d -s "${SESSION}" -n guide
tmux send-keys -t "${SESSION}:guide" "clear && cat ${GUIDE}" C-m

tmux new-window -t "${SESSION}" -n slam
tmux send-keys -t "${SESSION}:slam" \
    "$(wrap plain 'ros2 launch slam mapping.launch.py')"

tmux new-window -t "${SESSION}" -n nav
tmux send-keys -t "${SESSION}:nav" \
    "$(wrap plain 'ros2 launch imdl_autonomy slam_navigation.launch.py')"

tmux new-window -t "${SESSION}" -n rviz
tmux send-keys -t "${SESSION}:rviz" \
    "$(wrap x11 "ros2 launch imdl_simulation real_rviz.launch.py model_visual_yaw:=0.0 lidar_yaw:=${LIDAR_YAW}")"

tmux new-window -t "${SESSION}" -n teleop
tmux send-keys -t "${SESSION}:teleop" \
    "$(wrap plain "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=${NAMESPACE}/cmd_vel_teleop")"

# Two panes: the map saver on the left, a free shell for checks on the right.
tmux new-window -t "${SESSION}" -n tools
tmux send-keys -t "${SESSION}:tools" \
    "$(wrap plain "ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/${MAP_NAME}")"
tmux split-window -h -t "${SESSION}:tools"
tmux send-keys -t "${SESSION}:tools.1" \
    "$(wrap plain "ros2 topic hz ${NAMESPACE}/odom")"

tmux select-window -t "${SESSION}:guide"
exec tmux attach -t "${SESSION}"
