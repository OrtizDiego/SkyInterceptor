# SkyInterceptor

## Project Overview

**SkyInterceptor** is an advanced active interceptor drone system implemented in **ROS 2 Humble**. It is designed to autonomously track and intercept evading targets in a simulated environment using **Gazebo**.

## Architecture

The system is modularized into the following layers:
*   **Perception**: Stereo Vision depth estimation and YOLOv8 object detection.
*   **Estimation**: IMM-EKF for robust target tracking.
*   **Guidance**: Augmented Proportional Navigation (APN).
*   **Control**: Cascade PID controller for trajectory execution.
*   **Evasion**: Intelligent target behavior.

## Development Environment (Docker)

The project uses a simplified, high-performance Docker setup:
*   **Unified Container**: One container for development, testing, and execution.
*   **GPU Accelerated**: Built on NVIDIA CUDA 11.8 for real-time vision processing.
*   **ROS 2 Humble**: Full desktop installation.
*   **Persistent Cache**: Dedicated volumes for fast compilation.

### Common Commands

All operations are managed through a `Makefile`:

*   `make build`: Build the Docker image.
*   `make up`: Start the development environment.
*   `make down`: Stop the environment.
*   `make shell`: Enter the container.
*   `make build-ws`: Compile the ROS 2 workspace.
*   `make full`: Launch the complete system.
*   `make test`: Run project tests.

## Quick Start Guide

1.  **Start Environment**: `make up`
2.  **Enter Shell**: `make shell`
3.  **Build Workspace**: `make build-ws`
4.  **Launch System**: `make full`

For detailed Docker information, see [DOCKER.md](./DOCKER.md).