# Interceptor Drone System

ROS2 Humble implementation of an active interceptor drone with stereo vision and pursuit-evasion capabilities.

> **Quick Start with Docker (Recommended):**
> ```bash
> make build && make up && make shell && make build-ws && make full
> ```
> See [Docker Instructions](#quick-start-docker) below.

## Overview

This project implements a sophisticated drone interception system featuring:
- Stereo camera depth estimation (GPU-accelerated)
- YOLO object detection for target identification
- IMM-EKF for robust target tracking
- Augmented Proportional Navigation guidance
- Active target evasion with multiple strategies
- Real-time collision course interception

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     INTERCEPTOR DRONE                          │
├─────────────────────────────────────────────────────────────────┤
│  Perception → Estimation → Guidance → Control → Platform        │
└─────────────────────────────────────────────────────────────────┘
```

## Prerequisites

- Docker with Docker Compose
- NVIDIA Docker runtime (nvidia-docker2) for GPU support
- X11 forwarding capability (for GUI applications)

### NVIDIA Docker Setup

If you haven't installed NVIDIA Docker runtime yet:

```bash
# Add NVIDIA package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install nvidia-docker2
sudo apt-get update
sudo apt-get install -y nvidia-docker2

# Restart Docker daemon
sudo systemctl restart docker

# Test NVIDIA runtime
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## Quick Start (Docker)

The recommended way to run this project is using Docker containers, which provides a reproducible environment with all dependencies pre-installed.

### 1. Build the Docker Image

```bash
cd /home/diego/code/SkyInterceptor

# Build using Makefile (recommended)
make build

# Or build directly with docker-compose
docker-compose build interceptor-dev

# Build without cache (clean build)
make build-no-cache
```

**Build time:** ~15-30 minutes depending on internet connection and system specs.

### 2. Start the Development Container

```bash
# Start container (automatically enables X11 forwarding)
make up

# Or manually with docker-compose
docker-compose up -d interceptor-dev

# Check container status
make status
```

### 3. Enter the Container Shell

```bash
make shell

# You should see the welcome message and be in /workspace/interceptor_ws
```

### 4. Build the ROS2 Workspace

Inside the container:

```bash
# Build workspace
make build-ws

# Or manually:
cd /workspace/interceptor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Run the Simulation

```bash
# Launch full system
make full

# Or launch just the simulation (Gazebo)
make sim

# Or launch specific components manually inside container:
ros2 launch interceptor_drone interceptor_full.launch.py
```

## Docker Commands Reference

### Container Management

```bash
# Start container
make up

# Stop container
make down

# Restart container
make down && make up

# View container logs
make logs

# Clean everything (containers + volumes)
make clean-all

# Complete rebuild from scratch
make rebuild
```

### Development Commands

```bash
# Build workspace
make build-ws

# Run tests
make test

# Clean build artifacts
make clean

# Open shell in running container
make shell
```

### Launch Commands

```bash
# Launch simulation only
make sim

# Launch full system with all nodes
make full

# Start bag recording service
make record

# Run production/runtime container
make runtime
```

## Docker Architecture

### Services

The `docker-compose.yml` defines multiple services:

1. **interceptor-dev** (main development environment)
   - Full ROS2 Humble + Gazebo + OpenCV
   - GPU support via NVIDIA runtime
   - X11 forwarding for GUI applications
   - Source code mounted as volume
   - Build artifacts persisted in Docker volumes

2. **interceptor-runtime** (minimal production environment)
   - Stripped-down image for deployment
   - Runs the full system automatically

3. **bag-recorder** (optional)
   - Records ROS2 bag files
   - Activate with: `make record`

4. **test-runner** (optional)
   - Runs automated tests
   - Activate with: `make test`

### Volume Mounts

- `./interceptor_ws/src` → `/workspace/interceptor_ws/src` (source code)
- `interceptor-build` → `/workspace/interceptor_ws/build` (build artifacts)
- `interceptor-install` → `/workspace/interceptor_ws/install` (install space)
- `/tmp/.X11-unix` → `/tmp/.X11-unix` (X11 socket for GUI)

### Environment Variables

- `DISPLAY` - X11 display forwarding
- `NVIDIA_VISIBLE_DEVICES=all` - GPU access
- `ROS_DOMAIN_ID=0` - ROS2 domain isolation
- `GAZEBO_MODEL_PATH` - Gazebo model paths

## Development Workflow

### Typical Development Session

```bash
# 1. Start the container
cd /home/diego/code/SkyInterceptor
make up

# 2. Enter the container
make shell

# 3. Build workspace (first time only)
make build-ws

# 4. Launch simulation
ros2 launch interceptor_drone simulation.launch.py

# 5. In another terminal, launch perception
make shell  # Opens new shell in same container
ros2 launch interceptor_drone perception.launch.py

# 6. When done, stop the container
make down
```

### Modifying Code

1. Edit files in `./interceptor_ws/src/` on your host machine
2. Changes are immediately reflected in the container (volume mount)
3. Rebuild with `make build-ws` inside container
4. Source the workspace: `source install/setup.bash`

### Running Tests

```bash
# Run all tests
make test

# Or inside container
colcon test --packages-select interceptor_drone interceptor_interfaces
colcon test-result --verbose
```

## Troubleshooting Docker Issues

### Container Won't Start

```bash
# Check if NVIDIA runtime is available
docker info | grep -i nvidia

# Verify GPU access
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### GUI Applications Not Displaying

```bash
# Allow Docker to access X11
xhost +local:docker

# Or add to your ~/.bashrc:
echo "xhost +local:docker > /dev/null 2>&1" >> ~/.bashrc
```

### Permission Denied Errors

```bash
# Fix ownership of build directories
sudo chown -R $USER:$USER ./interceptor_ws/src
```

### Out of Disk Space

```bash
# Clean Docker system
docker system prune -a

# Remove old volumes
docker volume prune
```

## Alternative: Local Installation (Without Docker)

If you prefer not to use Docker, you can install dependencies locally:

### 1. Install Dependencies

```bash
cd /home/diego/code/SkyInterceptor
sudo ./install_dependencies.sh
```

### 2. Build the Workspace

```bash
cd interceptor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Run the Simulation

```bash
# Launch full system
ros2 launch interceptor_drone interceptor_full.launch.py

# Or launch components separately
ros2 launch interceptor_drone simulation.launch.py
ros2 launch interceptor_drone perception.launch.py
ros2 launch interceptor_drone guidance.launch.py
```

## Package Structure

```
interceptor_ws/
├── src/
│   ├── interceptor_drone/          # Main drone package
│   │   ├── include/                # Header files
│   │   ├── src/                    # Source files
│   │   ├── launch/                 # Launch files
│   │   ├── config/                 # Parameter files
│   │   ├── urdf/                   # Robot description
│   │   ├── worlds/                 # Gazebo worlds
│   │   └── rviz/                   # RViz configurations
│   └── interceptor_interfaces/     # Custom messages
│       ├── msg/                    # Message definitions
│       └── srv/                    # Service definitions
```

## Nodes

### Perception Layer
- `stereo_sync_node` - Synchronizes stereo camera pair
- `stereo_depth_processor` - Computes disparity and depth (GPU)
- `target_3d_localizer` - Extracts 3D position from detections

### Estimation Layer
- `target_tracker_node` - IMM-EKF for target state estimation

### Guidance Layer
- `guidance_controller_node` - Proportional navigation guidance

### Control Layer
- `trajectory_controller_node` - Cascade PID controller
- `hector_interface_node` - Hector quadrotor interface

### Evasion Layer
- `evasion_controller_node` - Target evasion behavior

## Configuration

Parameters are defined in YAML files:
- `config/perception_params.yaml` - Stereo and detection parameters
- `config/ekf_params.yaml` - Filter parameters
- `config/guidance_params.yaml` - Navigation constants
- `config/controller_params.yaml` - PID gains and limits

## Messages

### TargetDetection
Detection output from perception system with 2D bbox and 3D position.

### TargetState
Filtered target state from EKF with position, velocity, acceleration.

### GuidanceCommand
Acceleration commands and navigation parameters.

## Performance Specifications

| Component | Target |
|-----------|--------|
| Stereo Depth | 60 FPS @ 640x480 |
| YOLO Detection | 30 FPS |
| EKF Update | 100 Hz |
| Guidance | 100 Hz |
| Control | 100 Hz |

## Development

### Running Tests

```bash
colcon test --packages-select interceptor_drone
```

### Code Style

This project follows the ROS2 C++ style guide:
- camelCase for functions
- snake_case for variables
- PascalCase for classes
- UPPER_CASE for constants

## Troubleshooting

### Gazebo not starting
Make sure GAZEBO_MODEL_PATH is set:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models
```

### Camera images not showing
Check that the stereo camera topics are publishing:
```bash
ros2 topic list | grep stereo
ros2 topic hz /stereo/left/image_raw
```

### TF errors
Ensure the static transform publisher is running:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

## License

MIT License - See LICENSE file for details

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## Acknowledgments

- Hector Quadrotor package (TU Darmstadt)
- ROS2 community
- OpenCV and Eigen3 libraries
