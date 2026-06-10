FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# Dependencias del sistema
RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        ros-humble-nav2-amcl \
        ros-humble-nav2-bringup \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-msgs \
        ros-humble-nav2-util \
        ros-humble-nav2-common \
        ros-humble-rmw-fastrtps-cpp \
        libyaml-cpp-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Build context = MontecarlaRosbot/
COPY montecarla_msgs     src/montecarla_msgs
COPY montecarla_amcl     src/montecarla_amcl

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
        --packages-select montecarla_msgs montecarla_amcl

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY montecarla_amcl/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
