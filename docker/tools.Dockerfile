ARG BASE_IMAGE
FROM ${BASE_IMAGE}


SHELL ["/bin/bash", "-c"]
WORKDIR /tmp


# GCC
ARG DOCKER_GCC_VERSION=""
RUN if [ -n "$DOCKER_GCC_VERSION" ]; then \
        add-apt-repository ppa:ubuntu-toolchain-r/test && \
        apt update && \
        apt install -y gcc-${DOCKER_GCC_VERSION} g++-${DOCKER_GCC_VERSION} && \
        update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${DOCKER_GCC_VERSION} 100 && \
        update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-${DOCKER_GCC_VERSION} 100; \
    fi
RUN apt install -y cmake
RUN gcc --version && g++ --version && cmake --version


# BOOST
ARG DOCKER_BOOST_VERSION=""
RUN if [ -n "$DOCKER_BOOST_VERSION" ]; then \
        CONVERTED_VERSION=${DOCKER_BOOST_VERSION//./_} && \
        echo ${CONVERTED_VERSION}; \
        wget https://sourceforge.net/projects/boost/files/boost/${DOCKER_BOOST_VERSION}/boost_${CONVERTED_VERSION}.tar.gz/download -O boost_${CONVERTED_VERSION}.tar.gz && \
        tar xzf boost_${CONVERTED_VERSION}.tar.gz && \
        cd boost_${CONVERTED_VERSION} && \
        ./bootstrap.sh --prefix=/usr/local && \
        ./b2 install --with=all -j4; \
    fi


# PCL
ARG DOCKER_PCL_VERSION=""
RUN if [ -n "$DOCKER_PCL_VERSION" ]; then \
        wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${DOCKER_PCL_VERSION}.tar.gz -O pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        tar xzf pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        cd pcl-pcl-${DOCKER_PCL_VERSION} && \
        mkdir build && \
        cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DBUILD_apps=OFF -DBUILD_examples=OFF .. && \
        make -j4 && \
        make install; \
    fi


# Pybind11
ARG DOCKER_PYBIND_VERSION=""
RUN if [ -n "$DOCKER_PYBIND_VERSION" ]; then \
        wget https://github.com/pybind/pybind11/archive/refs/tags/v${DOCKER_PYBIND_VERSION}.tar.gz -O pybind11.tar.gz && \
        tar xzf pybind11.tar.gz && \
        cd pybind11-${DOCKER_PYBIND_VERSION} && \
        mkdir build && \
        cd build && \
        cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
        make -j4 && \
        make install; \
    fi


# Other dependencies
RUN apt install -y liboctomap-dev libgtest-dev libopencv-dev libopenmpi-dev openmpi-bin libjsoncpp-dev


# Cleanup
RUN rm -rf tmp/*


# Build Tools
COPY coloradar_tools /src/coloradar_tools
WORKDIR /src/coloradar_tools

RUN mkdir build
RUN cmake -B build
RUN make -C build
RUN ./build/coloradar_tests

#RUN python3.11 -m venv /opt/venv
#RUN pip3.11 install --upgrade pip
#RUN pip3.11 install -r requirements.txt

CMD ["bash"]
