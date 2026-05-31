FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY waypoint_publisher_package /ros2_ws/src/waypoint_publisher_package
COPY rssi_heatmap_generator_package /ros2_ws/src/rssi_heatmap_generator_package
COPY rosbot_interfaces /ros2_ws/src/rosbot_interfaces
COPY mappers_bringup /ros2_ws/src/mappers_bringup

RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-tk \
        python3-scipy \
        python3-numpy \
        python3-matplotlib \
        python3-opencv \
        ros-$ROS_DISTRO-nav2-map-server \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp && \
    pip install --upgrade scipy && \
    apt-get upgrade -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install && \
    apt-get remove -y --purge \
        python3-pip \
        python3-colcon-common-extensions && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

COPY ./run.sh /
RUN chmod +x /run.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
