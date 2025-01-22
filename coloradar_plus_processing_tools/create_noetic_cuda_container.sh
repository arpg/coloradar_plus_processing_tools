#!/bin/bash

# Step 1: Get the host CUDA version
if ! command -v nvidia-smi &> /dev/null; then
    echo "Error: CUDA is not installed or NVIDIA drivers are not available on the host system."
    exit 1
fi

CUDA_VERSION=$(nvidia-smi | grep -oP "CUDA Version: \K[0-9]+\.[0-9]+")
if [ -z "$CUDA_VERSION" ]; then
    echo "Error: Unable to detect CUDA version. Please ensure CUDA is properly installed."
    exit 1
fi
echo "Detected CUDA version: $CUDA_VERSION"

# Step 2: Call the Python script to get the appropriate Docker image
PYTHON_SCRIPT="coloradar_plus_processing_tools/utils/find_cuda_image.py"
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script '$PYTHON_SCRIPT' not found. Ensure it exists and is executable."
    exit 1
fi

CUDA_IMAGE=$(python3 "$PYTHON_SCRIPT" "$CUDA_VERSION")
CUDA_IMAGE="nvidia/cuda:$CUDA_IMAGE"
if [ $? -ne 0 ]; then
    echo "Error: Failed to fetch CUDA image. Check the output above for details."
    exit 1
fi
echo "Using CUDA image: $CUDA_IMAGE"

# Step 3: Build Cuda Image
ROS_BASE_IMAGE="cuda-local:$CUDA_VERSION"
echo "Building $ROS_BASE_IMAGE"
docker build --build-arg BASE_IMAGE="$CUDA_IMAGE" --build-arg DOCKER_CUDA_VERSION="$CUDA_VERSION" -f "coloradar_plus_processing_tools/Dockerfile.cuda-local" -t "$ROS_BASE_IMAGE" .

# Step 4: Run Docker Compose with the correct base image and CUDA version
DOCKER_CACHE_SEED=$(date +%s)
export DOCKER_CACHE_SEED="$DOCKER_CACHE_SEED"
export ROS_BASE_IMAGE="$ROS_BASE_IMAGE"
docker compose -f coloradar_plus_processing_tools/create_noetic_cuda_container.yml up --detach
#docker compose -f coloradar_plus_processing_tools/create_noetic_cuda_container.yml up --detach
#if [ ! $? -eq 0 ]; then
#    echo "Error: Failed to start the container."
#    exit 1
#fi
