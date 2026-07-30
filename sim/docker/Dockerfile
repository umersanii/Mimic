# Gazebo Sim (Harmonic) + ROS2 Jazzy (ros-base) + ros_gz_bridge, on Ubuntu 24.04 (Noble).
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    software-properties-common \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Gazebo Harmonic (OSRF apt repo)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Jazzy (ros-base) + ros_gz_bridge, via the current ros2-apt-source method
RUN add-apt-repository universe \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-dev-tools \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# For sim/bridge/gz_camera_dashboard.py - re-serves a gz-transport camera topic as a
# WebSocket JPEG stream. Ubuntu 24.04's system pip blocks plain installs (PEP 668);
# --break-system-packages matches this image's existing pattern of installing everything
# system-wide rather than standing up a venv for one script. --ignore-installed avoids
# pip trying (and failing) to uninstall apt/dpkg-managed packages like typing_extensions
# that have no pip RECORD file.
RUN apt-get update && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --break-system-packages --ignore-installed fastapi "uvicorn[standard]" pillow

CMD ["gz", "sim"]
