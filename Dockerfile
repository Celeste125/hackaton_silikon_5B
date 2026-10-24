# ROS 2 Humble valve controller for Raspberry Pi 5 + PCA9685
FROM ros:humble-ros-base

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip i2c-tools \
 && pip3 install --no-cache-dir smbus2 \
 && rm -rf /var/lib/apt/lists/*

COPY valve_controller.py /app/valve_controller.py

# ros_entrypoint.sh sources ROS, so extra "--ros-args ..." on docker run are passed through
ENTRYPOINT ["/ros_entrypoint.sh", "python3", "/app/valve_controller.py"]
ENV ROS_DOMAIN_ID=2
