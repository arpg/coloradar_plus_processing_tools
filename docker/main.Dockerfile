ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8


# Base OS
RUN UBUNTU_CODENAME=$(grep -oP '(?<=UBUNTU_CODENAME=)\w+' /etc/os-release) && \
    echo "deb http://archive.ubuntu.com/ubuntu ${UBUNTU_CODENAME} main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://archive.ubuntu.com/ubuntu ${UBUNTU_CODENAME}-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://archive.ubuntu.com/ubuntu ${UBUNTU_CODENAME}-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://security.ubuntu.com/ubuntu ${UBUNTU_CODENAME}-security main restricted universe multiverse" >> /etc/apt/sources.list
RUN apt update && apt upgrade -y
RUN apt install --fix-missing -y apt-utils wget lsb-release gnupg software-properties-common build-essential cmake
WORKDIR /tmp


# CUDA
RUN if [ -n "$CUDA_VERSION" ]; then \
        CUDA_VERSION_X_Y=$(echo "$CUDA_VERSION" | awk -F. '{print $1"-"$2}'); \
        ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
        echo "tzdata tzdata/Areas select Etc" | debconf-set-selections && \
        echo "tzdata tzdata/Zones/Etc select UTC" | debconf-set-selections && \
        apt update && apt install -y cuda-toolkit-${CUDA_VERSION_X_Y}; \
        nvcc --version; \
    fi


# ROS
ARG ROS_DISTRO=""
RUN if [ -n "$ROS_DISTRO" ]; then \
    wget -O - https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    if [ "$ROS_DISTRO" = "noetic" ]; then \
        echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list; \
    else \
        echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list; \
    fi && \
    apt update && apt install -y ros-${ROS_DISTRO}-desktop-full ros-${ROS_DISTRO}-catkin; \
    apt install -y python3-rosbag python3-rosdep python3-colcon-common-extensions python3-catkin-tools python3-osrf-pycommon; \
    rosdep init && \
    rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    catkin config --extend /opt/ros/$ROS_DISTRO; \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc; \
fi


RUN rm -rf /tmp/*
CMD ["bash"]
