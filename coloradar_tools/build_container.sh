#!/bin/bash


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
    CUDA_VERSION=""
else
    CUDA_VERSION=$(nvcc --version | grep -oP "release \K[0-9]+\.[0-9]+")
    if [ -z "$CUDA_VERSION" ]; then
        CUDA_VERSION=""
    fi
fi
echo "Using CUDA version: $CUDA_VERSION"


# Set base image
PYTHON_SCRIPT="coloradar_tools/scripts/find_base_image.py"
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script '$PYTHON_SCRIPT' not found. Ensure it exists and is executable."
    exit 1
fi
BASE_IMAGE=$(python3 "$PYTHON_SCRIPT" --cuda "$CUDA_VERSION" --ros "$ROS_DISTRO")
if [ $? -ne 0 ] || [ -z "$BASE_IMAGE" ]; then
    echo "Error: Failed to determine base image."
    echo "$BASE_IMAGE"
    exit 1
fi
echo "Using base image: $BASE_IMAGE"


# Build OS image
UBUNTU_VERSION=$(echo "$BASE_IMAGE" | grep -oP "ubuntu:\K[0-9]+\.[0-9]+|ubuntu\K[0-9]+\.[0-9]+$")
OS_IMAGE_NAME="ubuntu-local:$UBUNTU_VERSION"
if [ -n "$CUDA_VERSION" ]; then
    OS_IMAGE_NAME="${OS_IMAGE_NAME}-cuda$CUDA_VERSION"
    CUDA_IMAGE_NAME="cuda-local:$CUDA_VERSION"
    echo "Building CUDA image: $CUDA_IMAGE_NAME"
    docker build --build-arg BASE_IMAGE="$BASE_IMAGE" --build-arg DOCKER_CUDA_VERSION="$CUDA_VERSION" -f "coloradar_tools/docker/cuda.Dockerfile" -t "$CUDA_IMAGE_NAME" .
    BASE_IMAGE=$CUDA_IMAGE_NAME
fi
if [ -n "$ROS_DISTRO" ]; then
    OS_IMAGE_NAME="${OS_IMAGE_NAME}-$ROS_DISTRO"
    ROS_IMAGE_NAME="ros-local:$ROS_DISTRO"
    if [ -n "$CUDA_VERSION" ]; then
        ROS_IMAGE_NAME="${ROS_IMAGE_NAME}-cuda$CUDA_VERSION"
    fi
    echo "Building ROS image: $ROS_IMAGE_NAME"
    docker build --build-arg BASE_IMAGE="$BASE_IMAGE" --build-arg ROS_DISTRO="$ROS_DISTRO" -f "coloradar_tools/docker/ros.Dockerfile" -t "$ROS_IMAGE_NAME" .
    BASE_IMAGE=$ROS_IMAGE_NAME
fi
#echo "Building OS image: $OS_IMAGE_NAME"
#docker build --build-arg BASE_IMAGE="$BASE_IMAGE" -f "coloradar_tools/docker/os.Dockerfile" -t "$OS_IMAGE_NAME" --no-cache .
#BASE_IMAGE=$OS_IMAGE_NAME


# Build lib image
TAG=$(echo "$OS_IMAGE_NAME" | awk -F: '{print $2}')
MAIN_IMAGE_NAME="coloradar-tools:$TAG"
echo "Building main image: $MAIN_IMAGE_NAME"
docker build --build-arg BASE_IMAGE="$BASE_IMAGE" -f "coloradar_tools/docker/main.Dockerfile" -t "$MAIN_IMAGE_NAME" .


# Create container
COMMAND="docker run -d -it --gpus all --name coloradar-tools"
if [ -n "$CUDA_VERSION" ]; then
    COMMAND="${COMMAND} --gpus all"
fi
COMMAND="${COMMAND} ${MAIN_IMAGE_NAME}"
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
