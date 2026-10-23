# Sourced by every interactive shell in the imdl-humble container.
source /opt/ros/humble/setup.bash

# Must match the Jetson and the Create3.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Separate build/install dirs so the Humble build never collides with a
# host-side build in ros2_ws/build and ros2_ws/install.
ws_build() {
    (cd /ws && colcon build --symlink-install \
        --build-base build_humble --install-base install_humble \
        --packages-up-to slam imdl_autonomy imdl_simulation imdl_statemachine \
        "$@") && source /ws/install_humble/setup.bash
}

if [ -f /ws/install_humble/setup.bash ]; then
    source /ws/install_humble/setup.bash
else
    echo "Workspace not built yet: run 'ws_build'"
fi
