FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# iw: herramienta nl80211 para leer caché de APs del kernel (scan dump)
# iproute2: ip link show (verificar interfaz)
# El nodo intenta primero iw; si falla por permisos, usa nmcli via D-Bus
RUN apt-get update && apt-get install -y \
        iw \
        iproute2 \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-yaml \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rosbag2-py \
        ros-humble-rosbag2-storage-default-plugins && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Build context = MontecarlaRosbot/
COPY montecarla_msgs                   src/montecarla_msgs
COPY montecarla_real/wifi_scanner      src/wifi_scanner

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
        --packages-select montecarla_msgs wifi_scanner

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY montecarla_real/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
