FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y nano nala apt-utils
ARG USERNAME=alan
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if id -u $USER_UID >/dev/null 2>&1; then \
    # 如果 UID 1000 已經存在 (通常是 ubuntu)，將其改名為 ros
    EXISTING_USER=$(id -nu $USER_UID); \
    usermod -l ${USERNAME} -d /home/${USERNAME} -m ${EXISTING_USER}; \
    groupmod -n ${USERNAME} ${EXISTING_USER} || true; \
    else \
    # 如果不存在，正常建立
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME; \
    fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN nala install -y \
    usbutils \
    jstest-gtk \ 
    git \
    curl \
    lsb-release \
    gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y gz-harmonic

RUN mkdir -p /home/$USERNAME/Moveit2/ar4_mk_5_ws/src

COPY entrypoint.sh entrypoint.sh
COPY bashrc /home/$USERNAME/.bashrc

COPY src /home/$USERNAME/Moveit2/ar4_mk_5_ws/src/

USER ${USERNAME}

RUN sudo nala install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-gripper-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-forward-command-controller \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-opencv \
    ros-jazzy-moveit-servo

RUN sudo nala full-upgrade -y

RUN cd /home/$USERNAME/Moveit2/ar4_mk_5_ws \
    && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro jazzy -r\
    && sudo chown -R $USERNAME:$USERNAME .

RUN sudo nala install -y libgflags-dev nlohmann-json3-dev \
    ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
    ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-backward-ros libdw-dev libssl-dev mesa-utils libgl1 libgoogle-glog-dev

RUN eval "$(register-python-argcomplete3 ros2)"
RUN eval "$(register-python-argcomplete3 colcon)"

RUN sudo nala install -y ros-jazzy-orbbec-camera ros-jazzy-orbbec-description

RUN sudo nala full-upgrade -y

RUN pip install --break-system-packages ultralytics 
    # opencv-python \
    # numpy \
    # scipy \
    # open3d \
    # transforms3d \
    # supervision \
    # filterpy \
    # matplotlib

RUN sudo nala install -y ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos

RUN sudo nala install -y \
    mesa-utils \
    libgl1-mesa-dri \
    mesa-vulkan-drivers \
    mesa-utils-extra

ENTRYPOINT [ "/bin/bash", "entrypoint.sh" ]

CMD [ "bash" ]

# USER ros

