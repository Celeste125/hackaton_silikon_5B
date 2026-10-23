#!/usr/bin/env bash
# Start the ROS 2 Humble container, or open another shell in it if it is
# already running.
#   ./run.sh           start / attach
#   ./run.sh --build   rebuild the image first
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
xhost +local:docker >/dev/null 2>&1 || echo "warning: xhost failed, RViz may not open"

GPU_ARGS=()
[[ -e /dev/dri ]] && GPU_ARGS+=(--device /dev/dri)

# --net=host: DDS discovery of Jetson and Create3.
# --ipc=host: FastDDS shared-memory transport works between processes.
exec docker run -it --rm --name "${CONTAINER}" \
    --net=host --ipc=host --pid=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "${GPU_ARGS[@]}" \
    -v "${WS_DIR}":/ws \
    -v "${MAPS_DIR}":/root/maps \
    -v "${SCRIPT_DIR}/bashrc_ros.sh":/etc/imdl_bashrc.sh:ro \
    "${IMAGE}" bash
