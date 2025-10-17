# Use the official ROS2 Humble base image
FROM ros:humble-ros-base

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set ARG for proxies if needed during build
ARG HTTP_PROXY=http://host.docker.internal:1088
ARG HTTPS_PROXY=http://host.docker.internal:1088
ENV http_proxy=$HTTP_PROXY
ENV https_proxy=$HTTPS_PROXY

# Install all system and Python dependencies for the development environment
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    ros-humble-rosbridge-server \
    ros-humble-trajectory-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Rust toolchain (rustup, cargo, rustc)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install vcpkg and its dependencies (grpc, protobuf)
RUN git config --global http.proxy $HTTP_PROXY \
    && git config --global https.proxy $HTTPS_PROXY

RUN git clone https://github.com/microsoft/vcpkg.git /vcpkg \
    && cd /vcpkg \
    && ./bootstrap-vcpkg.sh \
    && ./vcpkg integrate install

ENV VCPKG_ROOT=/vcpkg
ENV PATH=$VCPKG_ROOT:$PATH

RUN /vcpkg/vcpkg install grpc protobuf

# Set the PROTOC env var to point to the vcpkg-installed compiler
# Assuming arm64 based on user's previous feedback
ENV PROTOC=/vcpkg/installed/arm64-linux/tools/protobuf/protoc

# Directly install Python dependencies
RUN pip3 install --no-cache-dir pymycobot==3.0.0 opencv-python-headless==4.8.0.74
ENV USE_SIM_ROBOT=true
# Create and set the working directory
WORKDIR /ros2_ws

# The entrypoint now just sources ROS and drops into a shell for interactive development
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && /bin/bash"]

# No default command
CMD []