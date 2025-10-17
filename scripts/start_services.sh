#!/bin/bash

# This script starts both rosbridge and the gRPC server for development.
# It handles shutting down both services when you press Ctrl+C.

# Function to clean up background processes on exit
cleanup() {
    echo "Shutting down background services..."
    # The check ensures we don't try to kill a non-existent process
    if kill -0 $ROSBRIDGE_PID 2>/dev/null; then
        kill $ROSBRIDGE_PID
        wait $ROSBRIDGE_PID
    fi
    echo "All services stopped."
}

# Trap the exit signal (e.g., from Ctrl+C) to run the cleanup function
trap cleanup EXIT

# Source the workspace
echo "Sourcing ROS2 environment..."
. /opt/ros/humble/setup.bash
. /ros2_ws/install/setup.bash

# Start rosbridge in the background
echo "Starting rosbridge_server in the background..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Save the Process ID (PID) of the background process
ROSBRIDGE_PID=$!

# Start the gRPC server in the foreground
echo "Starting mos-ros2 gRPC server in the foreground..."
ros2 run mos_ros2 ros2_grpc_server
