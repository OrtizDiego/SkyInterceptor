#!/bin/bash
# Entrypoint script for Interceptor Drone container

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    rosdep init 2>/dev/null || true
fi

# Update rosdep
rosdep update 2>/dev/null || true

# Install dependencies if src directory exists and has packages
if [ -d /workspace/interceptor_ws/src ] && [ "$(ls -A /workspace/interceptor_ws/src)" ]; then
    echo "Installing ROS dependencies..."
    cd /workspace/interceptor_ws
    rosdep install --from-paths src --ignore-src -y --skip-keys "
        gazebo_ros 
        gazebo_ros_pkgs 
        gazebo_plugins
        hector_quadrotor
        hector_quadrotor_description
        hector_quadrotor_gazebo
    " 2>/dev/null || true
fi

# Source workspace if built
if [ -f /workspace/interceptor_ws/install/setup.bash ]; then
    echo "Sourcing workspace..."
    source /workspace/interceptor_ws/install/setup.bash
fi

# Set up Gazebo environment
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}

# Print welcome message
echo "========================================================================"
echo "  Interceptor Drone Development Environment"
echo "========================================================================"
echo ""
echo "ROS2 Version: $(ros2 --version 2>/dev/null || echo 'Humble')"
echo "OpenCV Version: $(pkg-config --modversion opencv4 2>/dev/null || echo '4.x')"
echo "CUDA Available: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo 'No GPU')"
echo ""
echo "Workspace: /workspace/interceptor_ws"
echo ""
echo "Quick commands:"
echo "  Build:          colcon build --symlink-install"
echo "  Test:           colcon test"
echo "  Launch sim:     ros2 launch interceptor_drone simulation.launch.py"
echo "  Full system:    ros2 launch interceptor_drone interceptor_full.launch.py"
echo ""
echo "========================================================================"

# Execute the provided command
exec "$@"
