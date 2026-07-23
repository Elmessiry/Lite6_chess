FROM osrf/ros:humble-desktop-full

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Pinned for reproducible builds: xArm-Developer/xarm_ros2, humble branch.
ARG XARM_ROS2_COMMIT=d0b95117dabd3883f41155125aa3f67d37901c18

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    sudo \
    ros-humble-moveit \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ament-cmake \
    python3-pip \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && pip3 install --no-cache-dir pika==1.3.2 PyYAML==6.0.2 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep fix-permissions \
    && chown -R $USERNAME:$USERNAME /etc/ros/rosdep

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && mkdir -p /home/dev_ws/src \
    && chown -R ${USER_UID}:${USER_GID} /home/dev_ws/

WORKDIR /home/dev_ws/src
RUN git clone --recursive https://github.com/xArm-Developer/xarm_ros2.git \
    && cd xarm_ros2 \
    && git checkout $XARM_ROS2_COMMIT \
    && git submodule update --init --recursive

RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/dev_ws/
COPY ./xarm_user_params.yaml /home/dev_ws/src/xarm_ros2/xarm_api/config/xarm_user_params.yaml
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && colcon build" \
    && echo "source /home/dev_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER $USERNAME
ENV ROS_DISTRO=humble
# Interactive bash sources ~/.bashrc on its own; no --init-file needed.
CMD ["bash"]
