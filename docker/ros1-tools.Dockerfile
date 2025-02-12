#ARG BASE_IMAGE
#FROM ${BASE_IMAGE}
FROM coloradar-tools:20.04-cuda12.6-noetic


# Binarization Utils
WORKDIR /src/coloradar_plus_processing_tools
COPY coloradar_plus_processing_tools /src/coloradar_plus_processing_tools/coloradar_plus_processing_tools
COPY configs /src/coloradar_plus_processing_tools/configs
COPY main.py /src/coloradar_plus_processing_tools


# DCA1000 Utils
WORKDIR /catkin_ws
RUN mkdir src
COPY dca1000_device /catkin_ws/src/msgs
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin init && catkin config --extend /opt/ros/noetic && catkin build"


CMD ["bash"]