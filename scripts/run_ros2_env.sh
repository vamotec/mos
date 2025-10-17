#!/bin/bash

# This script starts the Docker container for the ROS2 development environment.
# It mounts the ENTIRE project root directory into the container's workspace
# and maps all necessary ports.

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
# Navigate up one level to get the project root
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

IMAGE_NAME="mos-ros2-env"
CONTAINER_NAME="mos-dev-container"

echo "Project root directory: ${PROJECT_ROOT}"
echo "Stopping and removing existing container named ${CONTAINER_NAME}..."
docker stop ${CONTAINER_NAME} || true
docker rm ${CONTAINER_NAME} || true

echo "Starting new ROS2 development container..."

docker run -it --name ${CONTAINER_NAME} \
  -v "${PROJECT_ROOT}:/ros2_ws/src" \
  -p "50051:50051" \
  -p "9090:9090" \
  ${IMAGE_NAME}
