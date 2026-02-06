# Simplified Dockerfile for Interceptor Drone System
# ROS2 Humble + Gazebo + OpenCV + CUDA support

FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set working directory
WORKDIR /workspace

# 1. Install Basic Utilities and General Dependencies
RUN apt-get update && apt-get install -y \
    curl wget git vim build-essential cmake \
    python3-pip \
    locales software-properties-common apt-transport-https gnupg lsb-release \
    # Debugging & Analysis
    gdb valgrind clang-format clang-tidy cppcheck \
    # GUI Support
    libgl1-mesa-glx libgl1-mesa-dri \
    # Network tools
    net-tools iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 2. Set locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 3. Install ROS2 Humble Desktop and ROS-specific Python tools
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    # ROS-specific build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 4. Install Gazebo & OpenCV with CUDA support
RUN apt-get update && apt-get install -y \
    gazebo libgazebo-dev \
    libopencv-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 5. Install Python packages for YOLO and development
RUN pip3 install --no-cache-dir \
    ultralytics onnxruntime-gpu \
    opencv-python numpy scipy matplotlib pyyaml \
    black pylint pytest

# 6. Environment Setup
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 7. Workspace and Entrypoint
RUN mkdir -p /workspace/interceptor_ws/src
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]