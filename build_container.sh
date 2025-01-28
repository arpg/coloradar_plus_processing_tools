#!/bin/bash


# Check requirements
if ! command -v yq &> /dev/null; then
    echo "Error: yq is not installed. Please install yq."
    exit 1
fi


# Set ROS version
AVAILABLE_ROS_DISTROS=("", "noetic")
ROS_DISTRO=""
if [ ! $# -eq 0 ]; then
  ROS_DISTRO="$1"
  if [[ ! " ${AVAILABLE_ROS_DISTROS[@]} " =~ " ${ROS_DISTRO} " ]]; then
    echo "Error: Invalid ROS distribution '${ROS_DISTRO}'."
    echo "Available options: ${AVAILABLE_ROS_DISTROS[*]}"
    exit 1
  fi
fi
echo "Using ROS distribution: $ROS_DISTRO"


# Set cuda version
if ! command -v nvcc &> /dev/null; then
    DOCKER_CUDA_VERSION=""
else
    DOCKER_CUDA_VERSION=$(nvcc --version | grep -oP "release \K[0-9]+\.[0-9]+")
    if [ -z "$DOCKER_CUDA_VERSION" ]; then
        DOCKER_CUDA_VERSION=""
    fi
fi
echo "Using CUDA version: $DOCKER_CUDA_VERSION"


# Set base image
PYTHON_SCRIPT="coloradar_tools/scripts/find_base_image.py"
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script '$PYTHON_SCRIPT' not found. Ensure it exists and is executable."
    exit 1
fi
BASE_IMAGE=$(python3 "$PYTHON_SCRIPT" --cuda "$DOCKER_CUDA_VERSION" --ros "$ROS_DISTRO")
if [ $? -ne 0 ] || [ -z "$BASE_IMAGE" ]; then
    echo "Error: Failed to determine base image."
    echo "$BASE_IMAGE"
    exit 1
fi
echo "Using base image: $BASE_IMAGE"


# Build OS image
UBUNTU_VERSION=$(echo "$BASE_IMAGE" | grep -oP "ubuntu:\K[0-9]+\.[0-9]+|ubuntu\K[0-9]+\.[0-9]+$")
echo "Using Ubuntu $UBUNTU_VERSION"
OS_IMAGE_NAME="ubuntu:$UBUNTU_VERSION"
if [ -n "$DOCKER_CUDA_VERSION" ]; then
    OS_IMAGE_NAME="${OS_IMAGE_NAME}-cuda$DOCKER_CUDA_VERSION"
fi
if [ -n "$ROS_DISTRO" ]; then
    OS_IMAGE_NAME="${OS_IMAGE_NAME}-$ROS_DISTRO"
fi
echo "Building OS image: $OS_IMAGE_NAME"
docker build --build-arg BASE_IMAGE="$BASE_IMAGE" --build-arg ROS_DISTRO="$ROS_DISTRO" -f "docker/main.Dockerfile" -t "$OS_IMAGE_NAME" .



# Build lib image
TAG=$(echo "$OS_IMAGE_NAME" | awk -F: '{print $2}')
LIB_IMAGE_NAME="coloradar-tools:$TAG"
echo "Building lib image: $LIB_IMAGE_NAME"

CONFIG_FILE="docker/ros-versions.yaml"
DOCKER_GCC_VERSION=$(yq ".ubuntu_versions.\"$UBUNTU_VERSION\".gcc" "$CONFIG_FILE" | tr -d '"')
DOCKER_BOOST_VERSION=$(yq ".ubuntu_versions.\"$UBUNTU_VERSION\".boost" "$CONFIG_FILE" | tr -d '"')
DOCKER_PCL_VERSION=$(yq ".ubuntu_versions.\"$UBUNTU_VERSION\".pcl" "$CONFIG_FILE" | tr -d '"')
DOCKER_PYBIND_VERSION=$(yq ".ubuntu_versions.\"$UBUNTU_VERSION\".pybind" "$CONFIG_FILE" | tr -d '"')

docker build --build-arg BASE_IMAGE=$OS_IMAGE_NAME --build-arg DOCKER_GCC_VERSION="$DOCKER_GCC_VERSION" --build-arg DOCKER_BOOST_VERSION="$DOCKER_BOOST_VERSION" --build-arg DOCKER_PCL_VERSION="$DOCKER_PCL_VERSION" --build-arg DOCKER_PYBIND_VERSION="$DOCKER_PYBIND_VERSION" -f "docker/tools.Dockerfile" -t "$LIB_IMAGE_NAME" .


# Create container
COMMAND="docker run -it --name coloradar-tools"
if [ -n "$CUDA_VERSION" ]; then
    COMMAND="${COMMAND} --gpus all"
fi
COMMAND="${COMMAND} ${LIB_IMAGE_NAME} bash"
echo "Run the command below to start the container. Add volumes via -v if necessary."
echo "$COMMAND"



#DOCKER_CACHE_SEED=$(date +%s)
#export DOCKER_CACHE_SEED="$DOCKER_CACHE_SEED"
#export ROS_BASE_IMAGE="$ROS_BASE_IMAGE"
# docker compose -f coloradar_plus_processing_tools/docker-compose.yml build  --no-cache
# docker compose -f coloradar_plus_processing_tools/docker-compose.yml up --detach
#docker compose -f coloradar_plus_processing_tools/docker-compose.yml up --detach
#if [ ! $? -eq 0 ]; then
#    echo "Error: Failed to start the container."
#    exit 1
#fi

# PYTHON
#wget https://www.python.org/ftp/python/3.11.5/Python-3.11.5.tgz
#tar xzf Python-3.11.5.tgz && \
#cd Python-3.11.5 && \
#    ./configure --enable-optimizations && \
#    make -j$(nproc) && \
#    make altinstall

