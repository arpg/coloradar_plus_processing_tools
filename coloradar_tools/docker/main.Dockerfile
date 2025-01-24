ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GCC_VERSION


RUN apt update && apt upgrade -y
RUN apt install -y apt-utils

RUN apt install -y gnupg software-properties-common build-essential pybind11-dev
RUN apt install -y libpcl-dev liboctomap-dev libgtest-dev libopencv-dev libopenmpi-dev openmpi-bin libjsoncpp-dev

RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt update
RUN apt install -y gcc-${GCC_VERSION} g++-${GCC_VERSION}
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${GCC_VERSION} 100
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-${GCC_VERSION} 100

RUN apt install -y cmake
RUN gcc --version && g++ --version && cmake --version

#WORKDIR /tmp
#RUN wget https://boostorg.jfrog.io/artifactory/main/release/1.78.0/source/boost_1_78_0.tar.gz && \
#    tar xzf boost_1_78_0.tar.gz && \
#    rm boost_1_78_0.tar.gz
#WORKDIR /tmp/boost_1_78_0
#RUN ./bootstrap.sh --prefix=/usr/local && \
#    ./b2 install --with=all -j$(nproc)
#WORKDIR /
#RUN rm -rf /tmp/boost_1_78_0
#RUN ldconfig && \
#    echo "Boost version:" && \
#    ls /usr/local/include/boost
WORKDIR /tmp
RUN wget https://sourceforge.net/projects/boost/files/boost/1.78.0/boost_1_78_0.tar.gz/download -O boost_1_78_0.tar.gz
RUN tar xzf boost_1_78_0.tar.gz && \
    rm boost_1_78_0.tar.gz && \
    cd boost_1_78_0 && \
    ./bootstrap.sh --prefix=/usr/local && \
    ./b2 install --with=all -j$(nproc)
# Clean up
RUN rm -rf /tmp/boost_1_78_0


COPY coloradar_tools /src/coloradar_tools
WORKDIR /src/coloradar_tools

RUN mkdir build
RUN cmake -B build
RUN make -C build
RUN ./build/coloradar_tests

#RUN PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')") \
#    && apt install -y python${PYTHON_VERSION}-dev
#RUN apt install -y python3-pip python3-virtualenv
#RUN python3 --version && pip3 --version
# RUN pip3 install -r requirements.txt


CMD ["bash"]