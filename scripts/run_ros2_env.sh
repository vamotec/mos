#!/bin/bash

# This script starts the Docker container for the ROS2 development environment.
# It mounts the current project directory into the container and maps the gRPC port.

PROJECT_DIR=$(pwd)
IMAGE_NAME="mos-ros2-env"

echo "Starting ROS2 development environment..."
echo "Mounting project directory: ${PROJECT_DIR}"

docker run -it --rm \
  -v "${PROJECT_DIR}:/ros2_ws/src/mos" \
  -p "50051:50051" \
  ${IMAGE_NAME}
