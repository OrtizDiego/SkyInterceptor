# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SkyInterceptor is a ROS2-based autonomous drone interception system written in C++17. It uses GPU-accelerated stereo vision, a YOLO-based detector (Python), an IMM-EKF tracker, and Proportional Navigation guidance to intercept aerial targets in simulation (Gazebo/Hector Quadrotor).

All development runs inside a Docker container with CUDA 11.8 + ROS2 Humble.

## Commands

All make targets call `docker-compose exec interceptor` internally — the container must be running (`make up`) first.

```bash
make build      # Build Docker image (first time setup)
make up         # Start container in background
make down       # Stop container
make shell      # Open bash shell inside container
make build-ws   # colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
make test       # colcon test + colcon test-result --verbose
make sim        # ros2 launch interceptor_drone simulation.launch.py
make full       # ros2 launch interceptor_drone interceptor_full.launch.py
make clean      # Remove build/, install/, log/ inside container
make status     # docker-compose ps
```

**Building a single package inside the container:**
```bash
colcon build --symlink-install --packages-select interceptor_drone
```

**Running a single node manually inside the container:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run interceptor_drone stereo_sync_node
```

**Note:** GTest tests for EKF, guidance, and controller are commented out in `CMakeLists.txt` (lines 154–161). `make test` currently only runs linting.

## Architecture

The system is a pipeline of ROS2 nodes organized into five layers:

```
Perception → Estimation → Guidance → Control → Platform
```

### Layer breakdown

| Layer | Node | Key algorithm |
|---|---|---|
| Perception | `stereo_sync_node` | ApproximateTime stereo synchronization |
| Perception | `stereo_depth_processor` | OpenCV CUDA SGBM disparity → depth |
| Perception | `target_3d_localizer` | Back-project 2D YOLO detections to 3D |
| Perception | `target_detector.py` | YOLOv8 via Ultralytics |
| Estimation | `target_tracker_node` | IMM-EKF (skeleton) |
| Guidance | `guidance_controller_node` | Proportional Navigation / Augmented PN (skeleton) |
| Control | `trajectory_controller_node` | Cascade PID |
| Control | `hector_interface_node` | Hector Quadrotor simulator bridge |
| Evasion | `evasion_controller_node` | Target evasion strategies (skeleton) |

### Shared library

`interceptor_drone_lib` is linked by every node and contains:
- `include/common/types.hpp` — `Detection`, `TargetState`, `GuidanceOutput` structs (Eigen3-based)
- `include/common/parameters.hpp` — All parameter structs (stereo, EKF, guidance, controller)
- `include/common/math_utils.hpp` — Quaternion conversion, skew-symmetric matrices, vector saturation
- `src/common/parameters.cpp` / `src/common/math_utils.cpp` — implementations

Everything lives under the `interceptor` namespace.

### Custom ROS2 messages (`interceptor_interfaces` package)

- `TargetDetection.msg` — 2D bounding box + 3D position
- `TargetState.msg` — Filtered position, velocity, acceleration, covariances
- `GuidanceCommand.msg` — Acceleration commands, navigation constants, intercept params
- `TargetTrajectory.msg` — Predicted trajectory
- `StereoImagePair.msg` — Synchronized stereo pair with calibration
- `SetInterceptMode.srv` — Mode switching service

### Configuration (`config/` directory)

YAML files loaded by launch files; key values to know:

| File | Notable params |
|---|---|
| `perception_params.yaml` | `baseline=0.12m`, `fx=535.4` |
| `stereo_sync_params.yaml` | sync tolerance `5ms` |
| `ekf_params.yaml` | process/measurement noise |
| `guidance_params.yaml` | `nav_constant_far=4.0`, `max_accel=20.0 m/s²` |
| `controller_params.yaml` | PID gains and limits |

### Launch files (`launch/` directory)

- `interceptor_full.launch.py` — Full system (simulation + perception + guidance + control + RViz)
- `simulation.launch.py` — Gazebo only
- `perception.launch.py` — Perception pipeline only
- `guidance.launch.py` — Guidance controller only

## ROS2 Workspace Layout

```
interceptor_ws/
  src/
    interceptor_drone/          # Main C++ package
      include/common/           # Shared headers (types, params, math)
      src/
        common/                 # Shared library sources
        perception/             # stereo_sync, depth_processor, 3d_localizer, target_detector.py
        estimation/             # target_tracker_node (IMM-EKF)
        guidance/               # guidance_controller_node (PN)
        control/                # trajectory_controller, hector_interface
        evasion/                # evasion_controller_node
      config/                   # YAML parameter files
      launch/                   # Python launch files
      urdf/                     # Robot model
      worlds/                   # Gazebo world files
      rviz/                     # RViz configs
    interceptor_interfaces/     # Custom msg/srv definitions
```

## Docker & GPU

- Base image: `nvidia/cuda:11.8.0-devel-ubuntu22.04`
- NVIDIA runtime required; GPU must be accessible from the host
- Source code is volume-mounted (`./interceptor_ws/src` → `/workspace/interceptor_ws/src`) so edits on the host are immediately visible inside the container without rebuilding the image
- Build artifacts (`build/`, `install/`, `log/`) are stored in named Docker volumes, not on the host

## Code Style

Follow the ROS2 C++ style guide: `PascalCase` classes, `camelCase` methods, `snake_case` variables/members, `UPPER_CASE` constants. Compiler flags: `-Wall -Wextra -Wpedantic -O3`.
