#!/usr/bin/env bash
# Start the ROS 2 Humble container, or open another shell in it if it is
# already running.
#   ./run.sh           start / attach
#   ./run.sh --build   rebuild the image first
#
# ANTHROPIC_API_KEY is passed through from this shell, for dispenser_locator.
# Exporting it inside an already-running container does not work: each
# 'docker exec' shell gets the environment 'docker run' created, so the value
# has to be present here, before the container starts. Put
#   export ANTHROPIC_API_KEY=...
# in the laptop's ~/.bashrc - never in this repo.
set -euo pipefail

IMAGE=imdl-humble
CONTAINER=imdl
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../ros2_ws" && pwd)"
# Under sudo, $HOME is /root; keep using the invoking user's ~/maps.
if [[ -n "${SUDO_USER:-}" ]]; then
    USER_HOME="$(getent passwd "${SUDO_USER}" | cut -d: -f6)"
else
    USER_HOME="${HOME}"
fi
MAPS_DIR="${USER_HOME}/maps"

if [[ "${1:-}" == "--build" ]] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    exec docker exec -it "${CONTAINER}" bash
fi

mkdir -p "${MAPS_DIR}"
# The container carries no X cookie of its own: it relies on this xhost call
# opening the laptop's X server to local connections. '+local:' and not
# '+local:docker': the named form adds an entry nothing matches on an Xwayland
# session, so xhost succeeds and RViz still dies with "Authorization required,
# but no authorization protocol specified". Over SSH, DISPLAY=:0
# alone is not enough -- GDM keeps the cookie under /run/user, not in
# ~/.Xauthority, so xhost fails and RViz never opens.
if [[ -z "${XAUTHORITY:-}" || ! -f "${XAUTHORITY:-}" ]]; then
    for candidate in \
        "/run/user/$(id -u)/gdm/Xauthority" \
        "/run/user/$(id -u)/.mutter-Xwaylandauth."* \
        "${USER_HOME}/.Xauthority"; do
        if [[ -f "${candidate}" ]]; then
            export XAUTHORITY="${candidate}"
            break
        fi
    done
fi

if ! xhost +local: >/dev/null 2>&1; then
    echo "warning: xhost failed, so RViz will not open on this machine's screen."
    echo "  DISPLAY=${DISPLAY:-unset} XAUTHORITY=${XAUTHORITY:-unset}"
    echo "  Fix: run 'xhost +local:' once in a terminal ON this machine's"
    echo "  desktop (not over SSH), then start this script again."
fi

GPU_ARGS=()
[[ -e /dev/dri ]] && GPU_ARGS+=(--device /dev/dri)

# --net=host: DDS discovery of Jetson and Create3.
# --ipc=host: FastDDS shared-memory transport works between processes.
exec docker run -it --rm --name "${CONTAINER}" \
    --net=host --ipc=host --pid=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}" \
    -e ANTHROPIC_API_KEY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "${GPU_ARGS[@]}" \
    -v "${WS_DIR}":/ws \
    -v "${MAPS_DIR}":/root/maps \
    -v "${SCRIPT_DIR}/bashrc_ros.sh":/etc/imdl_bashrc.sh:ro \
    "${IMAGE}" bash
