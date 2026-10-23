#!/usr/bin/env bash
# Answer one question: is the RealSense actually publishing anything usable?
#
# Run it INSIDE the container (the tmux 'cam-check' window does this for you):
#   ros2 launch perception camera.launch.py      # in another window first
#   ./scripts/check_camera.sh
#
# It reports, per topic, whether it exists and at what rate, then the image
# geometry and the intrinsics the dispenser locator needs, then the camera TF.
# Anything marked MISSING means the locator cannot work yet.
set -uo pipefail

# Pick whichever camera is actually publishing, so one script covers the
# webcam and the RealSense without being told which is plugged in.
if ros2 topic list 2>/dev/null | grep -qx "/webcam/image_raw"; then
    DETECTED="webcam (no depth -- range comes from geometry)"
    COLOR="${IMDL_COLOR_TOPIC:-/webcam/image_raw}"
    DEPTH="${IMDL_DEPTH_TOPIC:-/webcam/none}"
    INFO="${IMDL_INFO_TOPIC:-/webcam/camera_info}"
else
    DETECTED="realsense (or nothing yet)"
    COLOR="${IMDL_COLOR_TOPIC:-/camera/camera/color/image_raw}"
    DEPTH="${IMDL_DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
    INFO="${IMDL_INFO_TOPIC:-/camera/camera/aligned_depth_to_color/camera_info}"
fi
TARGET_FRAME="${IMDL_TARGET_FRAME:-map}"
HZ_SECS="${IMDL_HZ_SECS:-5}"

hr() { printf '%s\n' "------------------------------------------------------------"; }

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "camera detected: ${DETECTED}"
hr
echo "All camera-ish topics currently advertised:"
mapfile -t found < <(ros2 topic list 2>/dev/null | grep -iE "camera|image|depth|points" || true)
if [[ ${#found[@]} -eq 0 ]]; then
    echo "  (none) -- the camera driver is not running."
    echo "  Start it with: ros2 launch perception camera.launch.py"
else
    printf '  %s\n' "${found[@]}"
fi
hr

# rate <label> <topic> -- existence first, so a missing topic is not reported
# as a zero rate, which looks like a camera fault rather than a missing node.
rate() {
    local label="$1" topic="$2"
    printf '%-26s ' "${label}"
    if ! ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
        echo "MISSING   ${topic}"
        return
    fi
    local out
    out="$(timeout "${HZ_SECS}" ros2 topic hz "${topic}" 2>/dev/null \
        | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}')"
    if [[ -n "${out}" ]]; then
        printf 'OK  %6.2f Hz  %s\n' "${out}" "${topic}"
    else
        echo "NO DATA   ${topic}  (advertised, but nothing published)"
    fi
}

echo "Publish rates (${HZ_SECS}s sample; the driver is configured for 15 Hz):"
rate "color" "${COLOR}"
rate "aligned depth" "${DEPTH}"
rate "camera_info" "${INFO}"
hr

echo "Colour image geometry:"
timeout 8 ros2 topic echo --once --field header.frame_id "${COLOR}" 2>/dev/null \
    | sed 's/^/  frame_id: /' || echo "  (no message)"
for f in width height encoding; do
    v="$(timeout 8 ros2 topic echo --once --field "${f}" "${COLOR}" 2>/dev/null | head -1)"
    printf '  %-9s %s\n' "${f}:" "${v:-(none)}"
done
hr

echo "Intrinsics the locator deprojects with (from ${INFO}):"
timeout 8 ros2 topic echo --once --field k "${INFO}" 2>/dev/null | head -3 \
    | sed 's/^/  K: /' || echo "  (no camera_info)"
hr

OPTICAL="${IMDL_OPTICAL_FRAME:-camera_color_optical_frame}"
case "${DETECTED}" in webcam*) OPTICAL="webcam_optical_frame";; esac
echo "Camera extrinsics -- base_link to ${OPTICAL}:"
if timeout 8 ros2 run tf2_ros tf2_echo base_link "${OPTICAL}" 2>&1 \
    | grep -A4 -m1 "Translation" | sed 's/^/  /'; then :; else
    echo "  MISSING -- webcam.launch.py (or camera.launch.py) publishes this"
    echo "  with publish_camera_tf:=true. Without it a bearing is meaningless."
fi
hr
echo "Dispenser pose -> ${TARGET_FRAME}: needs the above plus ${TARGET_FRAME}->base_link."
printf '%-26s ' "${TARGET_FRAME} -> base_link"
if timeout 6 ros2 run tf2_ros tf2_echo "${TARGET_FRAME}" base_link >/dev/null 2>&1; then
    echo "OK"
else
    echo "MISSING -- start SLAM (window 4) first"
fi
hr
echo "To look at the picture:"
echo "  rqt_image_view ${COLOR}          # or the RViz 'RealSense Color Image' display"
