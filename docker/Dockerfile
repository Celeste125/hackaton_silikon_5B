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
    python3-colcon-common-extensions \
    python3-numpy \
    python3-yaml \
    python3-pil \
 && rm -rf /var/lib/apt/lists/*

COPY bashrc_ros.sh /etc/imdl_bashrc.sh
RUN echo "source /etc/imdl_bashrc.sh" >> /root/.bashrc

WORKDIR /ws
