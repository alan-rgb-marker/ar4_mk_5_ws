FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y nano

ARG USERNAME=alan
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# RUN groupadd --gid $USER_GID $USERNAME \
#   && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#   && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# ARG USERNAME=ros

# RUN groupadd \
#   && useradd -s /bin/bash -m $USERNAME \
#   && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

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

RUN apt-get update \
    && apt-get install -y nala apt-utils 

RUN nala install -y \
    usbutils \
    jstest-gtk

RUN mkdir -p /home/$USERNAME/Moveit2/ar4_mk_3_ws/src

COPY entrypoint.sh entrypoint.sh
COPY bashrc /home/$USERNAME/.bashrc

COPY src /home/$USERNAME/Moveit2/ar4_mk_3_ws/src/

USER ${USERNAME}

RUN cd /home/$USERNAME/Moveit2/ar4_mk_3_ws \
    && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro jazzy -r\
    && sudo chown -R $USERNAME:$USERNAME .

RUN sudo nala install ros-jazzy-controller-manager \
    && sudo nala install ros-jazzy-moveit-ros-planning-interface 

RUN sudo nala install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-gripper-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller

# You must also install the following packages when using MoveIt2.
RUN sudo nala install -y \
    ros-jazzy-forward-command-controller \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-simple-controller-manager

RUN sudo nala full-upgrade -y
    
# RUN sudo nala update && sudo rm -rf /var/lib/apt/lists/*


ENTRYPOINT [ "/bin/bash", "entrypoint.sh" ]

CMD [ "bash" ]

# USER ros
