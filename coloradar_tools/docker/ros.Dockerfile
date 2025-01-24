ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG ROS_DISTRO

RUN apt update && apt upgrade -y
RUN apt install -y apt-utils

RUN apt install -y wget lsb-release
RUN wget -O - http://packages.ros.org/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt update && apt install -y ros-${ROS_DISTRO}-desktop-full python3-rosdep python3-rosbag python3-catkin-tools
RUN rosdep init && rosdep update
RUN apt install -y ros-${ROS_DISTRO}-octomap ros-${ROS_DISTRO}-octomap-ros

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc


CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && exec bash"]
