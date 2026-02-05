#!/bin/bash
# Installation script for Interceptor Drone dependencies
# Run with: sudo ./install_dependencies.sh

set -e

echo "=========================================="
echo "Installing Interceptor Drone Dependencies"
echo "=========================================="

# Update package list
apt-get update

# Install ROS2 Humble (if not already installed)
echo "Installing ROS2 Humble..."
apt-get install -y software-properties-common
add-apt-repository universe -y
apt-get update
apt-get install -y curl gnupg lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update
apt-get install -y ros-humble-desktop ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs

# Install build tools
echo "Installing build tools..."
apt-get install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || true
fi
rosdep update || true

# Install OpenCV with CUDA support (or regular OpenCV if CUDA not available)
echo "Installing OpenCV..."
apt-get install -y libopencv-dev libopencv-contrib-dev

# Install Eigen3
echo "Installing Eigen3..."
apt-get install -y libeigen3-dev

# Install additional ROS2 packages
echo "Installing ROS2 packages..."
apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# Install hector_quadrotor (will need to be built from source)
echo "Note: hector_quadrotor needs to be built from source"
echo "Clone from: https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor"

# Setup environment
echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "To setup the environment, add to your ~/.bashrc:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Then build the workspace:"
echo "  cd /home/diego/code/SkyInterceptor/interceptor_ws"
echo "  colcon build --symlink-install"
echo ""
