# ROS 2 Humble environment for the laptop side (RViz, SLAM tools, AMCL, Nav2,
# imdl_autonomy). Matches the Humble install on the Jetson and the Create3.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-twist-mux \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-irobot-create-description \
    ros-humble-irobot-create-control \
    ros-humble-irobot-create-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-tf2-tools \
    ros-humble-realsense2-camera \
    ros-humble-v4l2-camera \
    ros-humble-usb-cam \
    ros-humble-rqt-image-view \
    ros-humble-camera-calibration \
    ros-humble-image-transport-plugins \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    python3-pil \
 && rm -rf /var/lib/apt/lists/*

# The Anthropic SDK has no apt package. dispenser_locator asks Claude which
# pixels are the food dispenser; depth and TF still do all the geometry.
RUN pip3 install --no-cache-dir anthropic

COPY bashrc_ros.sh /etc/imdl_bashrc.sh
RUN echo "source /etc/imdl_bashrc.sh" >> /root/.bashrc

WORKDIR /ws
