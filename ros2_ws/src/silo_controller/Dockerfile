# ROS 2 Humble valve controller for Raspberry Pi 5 + PCA9685.
# Build from this folder (ros2_ws/src/silo_controller); the Pi needs no colcon
# build. For example:
#   docker build -t silo-controller .
#   docker run -d --restart unless-stopped --net host --device /dev/i2c-1 \
#     -v "$PWD/data:/data" --name silo-controller silo-controller
FROM ros:humble-ros-base

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip i2c-tools \
 && pip3 install --no-cache-dir smbus2 \
 && rm -rf /var/lib/apt/lists/*

COPY silo_controller/valve_controller.py /app/valve_controller.py

# ros_entrypoint.sh sources ROS, so extra "--ros-args ..." on docker run are passed through
ENTRYPOINT ["/ros_entrypoint.sh", "python3", "/app/valve_controller.py"]
ENV ROS_DOMAIN_ID=2
