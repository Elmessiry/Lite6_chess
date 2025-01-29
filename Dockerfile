FROM osrf/ros:humble-desktop-full

ARG USERNAME=dozal
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && chown -R $USER_UID:$USER_GID /home/${USERNAME}/ \
    && apt-get install -y \
    ros-humble-moveit \
    ros-humble-gazebo-ros-pkgs \
    python3-pip \
    evtest \
    python3-serial \
    nano \
    && curl -sSL http://get.gazebosim.org | sh \
    && pip3 install opencv-python \
    && pip3 install pika \
    && apt-get install ros-$ROS_DISTRO-ament-cmake -y \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep fix-permissions \
    && chown -R $USERNAME:$USERNAME /etc/ros/rosdep

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && mkdir -p /home/dev_ws/src \
    && chown -R ${USER_UID}:${USER_GID} /home/dev_ws/

WORKDIR /home/dev_ws/src
RUN git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO

WORKDIR /home/dev_ws/src/xarm_ros2
RUN git pull && git submodule sync && git submodule update --init --remote

WORKDIR /home/dev_ws/src
RUN apt-get update && apt-get upgrade -y \
    && rosdep update && rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/dev_ws/
COPY ./xarm_user_params.yaml /home/dev_ws/src/xarm_ros2/xarm_api/config/xarm_user_params.yaml
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && colcon build" \
    && echo "source /home/dev_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER $USERNAME
ENV ROS_DISTRO=humble
CMD ["bash", "--init-file", "~/.bashrc"]