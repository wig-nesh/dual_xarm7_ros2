FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libasound2 \
    libatk-bridge2.0-0 \
    libgbm1 \
    libgtk-3-0 \
    libnss3 \
    libxcomposite1 \
    libxdamage1 \
    libxrandr2 \
    libxss1 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-controller-manager \
    ros-humble-cv-bridge \
    ros-humble-desktop \
    ros-humble-diff-drive-controller \
    ros-humble-find-object-2d \
    ros-humble-gripper-controllers \
    ros-humble-image-view \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY init_workspace.sh /usr/local/bin/init_workspace
COPY ufactory_studio.sh /usr/local/bin/ufactory-studio
RUN chmod 0755 /usr/local/bin/entrypoint.sh /usr/local/bin/init_workspace /usr/local/bin/ufactory-studio

WORKDIR /colcon_ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
