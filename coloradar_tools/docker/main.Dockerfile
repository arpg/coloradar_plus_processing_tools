ARG BASE_IMAGE
FROM ${BASE_IMAGE}


RUN apt update && apt upgrade -y
RUN apt install -y apt-utils

RUN apt install -y gnupg software-properties-common build-essential pybind11-dev
RUN apt install -y libpcl-dev liboctomap-dev libgtest-dev libopencv-dev
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt update
RUN apt install -y gcc-12 g++-12
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100
RUN apt install -y cmake
RUN gcc --version && g++ --version && cmake --version

RUN PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')") \
    && apt install -y python${PYTHON_VERSION}-dev
RUN apt install -y python3-pip python3-virtualenv
RUN python3 --version && pip3 --version

COPY coloradar_tools /src/coloradar_tools
WORKDIR /src/coloradar_tools

RUN mkdir build
RUN cmake -B build
RUN make -C build
RUN ./build/coloradar_tests

# RUN pip3 install -r requirements.txt


CMD ["bash"]