ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Install Python 3.10 and set it as default
# RUN add-apt-repository ppa:deadsnakes/ppa -y
# RUN apt update && apt install -y python3.10 python3.10-venv python3.10-dev python3-pip
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
# RUN update-alternatives --set python3 /usr/bin/python3.10
# RUN apt install -y python3.10-distutils
# RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3

# Install Python dependencies
RUN apt install -y python3.8-venv python3.8-dev python3-pip
RUN python3 --version && pip3 --version
RUN pip install -U pip
RUN pip install pip-review tqdm scikit-learn
# RUN pip install open3d
RUN pip-review --local --auto

# Install ROS Noetic
RUN wget -O - http://packages.ros.org/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt update && apt install -y ros-noetic-desktop-full python3-rosdep python3-catkin-tools python3-rosbag
RUN rosdep init && rosdep update

# Coloradar dependencies
RUN apt install libpcl-dev liboctomap-dev libgtest-dev libopencv-dev ros-noetic-octomap ros-noetic-octomap-ros pybind11-dev

# Source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && exec bash"]
