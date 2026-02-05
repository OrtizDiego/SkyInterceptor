# Docker Configuration Guide

This document provides detailed information about the simplified Docker setup for the Interceptor Drone System.

## Overview

The project uses a streamlined Docker setup designed for efficient development:

- **Single Stage Dockerfile**: Combines ROS 2 Humble, CUDA, and development tools into one robust image.
- **Unified Service**: A single `interceptor` service in `docker-compose.yml` handles all tasks (dev, test, launch).
- **GPU Support**: NVIDIA CUDA runtime for OpenCV and TensorRT.
- **X11 Forwarding**: GUI applications (Gazebo, RViz) work seamlessly.
- **Volume Mounts**: Source code on host is mounted to the container for live editing.
- **Persistent Build Cache**: Named Docker volumes persist build artifacts, making recompilation fast.

## Dockerfile

The Dockerfile starts from `nvidia/cuda:11.8.0-devel-ubuntu22.04` and installs:
- ROS 2 Humble Desktop
- Gazebo 11
- OpenCV & Eigen
- Python dependencies (YOLOv8, TensorRT support)
- Development tools (GDB, Valgrind, Linter, Formatter)

## Docker Compose

The `docker-compose.yml` defines the `interceptor` service with:
- `privileged: true` and `runtime: nvidia` for hardware/GPU access.
- `network_mode: host` for seamless ROS 2 communication.
- Automatic volume mounting of your source code and SSH/Git configs.

## Makefile Commands

The `Makefile` simplifies interaction with the Docker container:

| Command | Description |
|---------|-------------|
| `make build` | Build the Docker image |
| `make up` | Start the container in the background |
| `make down` | Stop and remove the container |
| `make shell` | Open a new bash shell inside the running container |
| `make build-ws` | Build the ROS 2 workspace inside the container |
| `make sim` | Launch the Gazebo simulation |
| `make full` | Launch the full interceptor system |
| `make test` | Run automated tests |
| `make clean` | Remove build, install, and log artifacts |

## Development Workflow

1.  **Start the environment**: `make up`
2.  **Enter the container**: `make shell`
3.  **Build the code**: `make build-ws` (or run it from the host with `make build-ws`)
4.  **Run the system**: `make full`
5.  **Iterate**: Edit code on your host; it's instantly reflected in the container. Re-run `make build-ws` to compile.

## Troubleshooting

### GPU Not Detected
Ensure the NVIDIA Container Toolkit is installed on your host and run:
`docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`

### GUI Applications Not Displaying
The `make up` command automatically runs `xhost +local:docker`. If displays still don't work, ensure your `DISPLAY` environment variable is set correctly on the host.

### Network Issues
Since we use `network_mode: host`, ensure there are no port conflicts with other ROS 2 instances or local services.