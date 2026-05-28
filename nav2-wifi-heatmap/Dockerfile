FROM ros:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . .

RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        ros-humble-nav2-core \
        ros-humble-nav2-util \
        ros-humble-nav2-bringup \
        ros-humble-nav2-waypoint-follower \
        ros-humble-pluginlib \
        ros-humble-rclcpp \
        ros-humble-rclcpp-action \
        ros-humble-rclcpp-lifecycle \
        ros-humble-rmw-fastrtps-cpp && \
    apt-get upgrade -y && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    apt-get remove -y --purge \
        python3-colcon-common-extensions && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros2_ws/ros_entrypoint.sh"]

CMD ["bash"]
