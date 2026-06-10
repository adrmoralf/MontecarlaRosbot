FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# Dependencias del sistema
RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-scipy \
        python3-yaml \
        ros-humble-nav-msgs \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rosbag2-py \
        ros-humble-rosbag2-storage-default-plugins && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Copiar paquetes en orden de dependencia
# Build context = MontecarlaRosbot/ (context: .. en compose.sim.yaml)
COPY montecarla_msgs                  src/montecarla_msgs
COPY montecarla_sim/wifi_simulator    src/wifi_simulator

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
        --packages-select montecarla_msgs wifi_simulator

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY montecarla_sim/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
