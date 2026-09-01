FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        fastdds-tools \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-control-msgs \
        ros-humble-moveit \
        ros-humble-moveit-configs-utils \
        ros-humble-trajectory-msgs \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /ros_entrypoint_exp2.sh
RUN chmod +x /ros_entrypoint_exp2.sh

WORKDIR /workspace/mecharm_exp2
ENTRYPOINT ["/ros_entrypoint_exp2.sh"]
CMD ["bash"]
