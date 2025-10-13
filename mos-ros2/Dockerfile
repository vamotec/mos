# Use the official ROS2 Humble base image
FROM ros:humble-ros-base

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

ARG HTTP_PROXY=http://host.docker.internal:1088
ARG HTTPS_PROXY=http://host.docker.internal:1088
ENV http_proxy=$HTTP_PROXY
ENV https_proxy=$HTTPS_PROXY

# Install system dependencies for building C++ code, gRPC, and Protobuf
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN git config --global http.proxy $HTTP_PROXY \
    && git config --global https.proxy $HTTPS_PROXY

RUN git clone https://github.com/microsoft/vcpkg.git /vcpkg \
    && cd /vcpkg \
    && ./bootstrap-vcpkg.sh \
    && ./vcpkg integrate install

ENV VCPKG_ROOT=/vcpkg
ENV PATH=$VCPKG_ROOT:$PATH

RUN /vcpkg/vcpkg install grpc protobuf

# Create a ROS2 workspace
WORKDIR /ros2_ws

# Source the ROS2 setup file in the entrypoint, so all new shells are pre-configured
# This makes it convenient to run ros2 commands immediately after entering the container.
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && /bin/bash"]

# By default, drop into an interactive shell
CMD []
