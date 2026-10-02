# SkyInterceptor

SkyInterceptor is an autonomous drone system built on **ROS 2 Humble** (C++17 and Python). It runs entirely in simulation (Gazebo Classic) and has two mission modes:

- **FOLLOW**: an aerial filming drone that follows a person, bicycle or car while always keeping a minimum safety distance from every person and vehicle.
- **INTERCEPT**: counter-UAS capture of an intruding small drone. Only drones are valid targets; ground targets are never engaged.

The pipeline is stereo vision → YOLO detection → IMM-EKF tracking → mode-specific guidance → trajectory control. An independent safety filter has the final say on every setpoint.

> **Status:** early stage. The perception layer (stereo sync, depth, YOLO detection, 3D localization) works. Tracking, guidance, control and evasion nodes are still skeletons. See [`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md) for the roadmap and current status.

## Repository layout

```
SkyInterceptor/
├── Dockerfile, docker-compose.yml   # Dev container: CUDA 11.8 + ROS 2 Humble + Gazebo 11
├── docker/entrypoint.sh             # Container entrypoint (rosdep, env, welcome banner)
├── Makefile                         # All day-to-day commands (see below)
├── scripts/quick_test.sh            # Perception pipeline smoke test
├── IMPLEMENTATION_PLAN.md           # Architecture, roadmap and status
├── DOCKER.md                        # Docker details, WSL2 GPU rendering, troubleshooting
├── docs/AGENT_PROMPTS.md            # Task prompts for coding agents
├── .github/workflows/ci.yml         # CI: static checks + build & test
└── interceptor_ws/                  # ROS 2 (colcon) workspace
    └── src/
        ├── interceptor_interfaces/  # Custom messages (msg/) and services (srv/)
        └── interceptor_drone/       # Main package
            ├── include/common/      # Shared headers: types, parameters, math utils
            ├── src/
            │   ├── common/          # Shared library (interceptor_drone_lib)
            │   ├── perception/      # stereo_sync_node, stereo_depth_processor,
            │   │                    # target_3d_localizer, target_detector.py
            │   ├── estimation/      # target_tracker_node (IMM-EKF)
            │   ├── guidance/        # guidance_controller_node (proportional navigation)
            │   ├── control/         # trajectory_controller_node, hector_interface_node
            │   └── evasion/         # evasion_controller_node
            ├── test/                # GTest unit tests
            ├── config/              # YAML parameter files
            ├── launch/              # Launch files
            ├── urdf/, worlds/, rviz/  # Drone model, Gazebo world, RViz config
            ├── CMakeLists.txt
            └── package.xml
```

### Architecture

```
Perception  →  Estimation  →  Guidance  →  Control  →  Platform (Gazebo)
```

| Layer | Node | Purpose |
|---|---|---|
| Perception | `stereo_sync_node` | Time-synchronizes the left/right camera images |
| Perception | `stereo_depth_processor` | SGBM disparity → depth image |
| Perception | `target_detector.py` | YOLOv8 object detection (Ultralytics) |
| Perception | `target_3d_localizer` | Back-projects 2D detections to 3D positions |
| Estimation | `target_tracker_node` | IMM-EKF target tracking *(skeleton)* |
| Guidance | `guidance_controller_node` | Proportional navigation *(skeleton)* |
| Control | `trajectory_controller_node` | Cascade PID *(skeleton)* |
| Control | `hector_interface_node` | Simulator bridge *(skeleton)* |
| Evasion | `evasion_controller_node` | Target-drone behaviour *(skeleton)* |

## Installation

Everything runs inside a Docker container, so the host only needs Docker.

### Prerequisites

- Linux, or Windows with WSL2
- [Docker](https://docs.docker.com/engine/install/) with the `docker-compose` command
- An NVIDIA GPU with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) (the compose file uses `runtime: nvidia`)
- An X server for Gazebo and RViz (WSLg on Windows)

Check that Docker can see the GPU:

```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### Setup

```bash
git clone https://github.com/OrtizDiego/SkyInterceptor.git
cd SkyInterceptor

make build      # Build the Docker image (first time only, ~15–30 min)
make up         # Start the container in the background
make build-ws   # Build the ROS 2 workspace inside the container
```

Source code is mounted into the container, so edits on the host are visible immediately; just run `make build-ws` again. Build output lives in Docker volumes, not on the host.

For WSL2 GPU rendering and other Docker details, see [`DOCKER.md`](DOCKER.md).

## Commands

Run all `make` targets **from the host**. Everything except `make build` and `make up` needs the container to be running.

| Command | What it does |
|---|---|
| `make help` | List the available targets |
| `make build` | Build the Docker image (log written to `logs/docker-build.log`) |
| `make up` | Start the container (also allows X11 access for GUIs) |
| `make down` | Stop and remove the container |
| `make status` | Show container status |
| `make shell` | Open a bash shell inside the container |
| `make build-ws` | `colcon build` the workspace (Release, `--symlink-install`) |
| `make test` | Run the unit tests and linters (`colcon test` + `colcon test-result`) |
| `make sim` | Launch Gazebo with the drone and the scenario world |
| `make full` | Launch the full system: simulation, perception, guidance, control and RViz |
| `make clean` | Delete `build/`, `install/` and `log/` inside the container |

### Running individual parts

Inside the container (`make shell`), source the workspace first:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

Launch files:

```bash
ros2 launch interceptor_drone simulation.launch.py      # Gazebo only
ros2 launch interceptor_drone perception.launch.py      # Perception pipeline only
ros2 launch interceptor_drone guidance.launch.py        # Tracker + guidance only
ros2 launch interceptor_drone interceptor_full.launch.py
```

A single node:

```bash
ros2 run interceptor_drone stereo_sync_node
ros2 run interceptor_drone target_detector.py
```

Rebuild only one package:

```bash
colcon build --symlink-install --packages-select interceptor_drone
```

## Testing

### Unit tests and linters

```bash
make test
```

This runs:

- **GTest unit tests** in `interceptor_drone/test/` (`test_math_utils`, `test_parameters`)
- **ament linters**: uncrustify, cpplint, cppcheck, flake8, pep257, lint_cmake, xmllint

Inside the container you can also run the tests for a single package and see the results:

```bash
colcon test --packages-select interceptor_drone
colcon test-result --verbose
```

To auto-fix C++ formatting (from `interceptor_ws/src/interceptor_drone` inside the container):

```bash
ament_uncrustify --reformat src include test
```

### Perception smoke test

`scripts/quick_test.sh` checks that the perception pipeline starts and publishes its topics. Start the simulation with `make sim`, then in another terminal run:

```bash
make shell
bash scripts/quick_test.sh
```

### Continuous integration

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs on every pull request and every push to `main`:

1. **Static checks**: yamllint, shellcheck and Python syntax.
2. **Build & test**: `colcon build` with `-Werror`, then `colcon test` in the `ros:humble-perception` container.

The CI container has no CUDA, so the code must also build without a GPU.

You can run the static checks locally without ROS:

```bash
pip install yamllint shellcheck-py
yamllint --strict .
git ls-files '*.sh' | xargs shellcheck --severity=warning
git ls-files '*.py' | xargs python3 -m py_compile
```

## Configuration

Parameters are in `interceptor_ws/src/interceptor_drone/config/`:

| File | Contents |
|---|---|
| `perception_params.yaml` | Stereo camera (baseline, intrinsics) and detector settings |
| `stereo_sync_params.yaml` | Stereo synchronization tolerance |
| `ekf_params.yaml` | Tracker process and measurement noise |
| `guidance_params.yaml` | Navigation constants and acceleration limits |
| `controller_params.yaml` | PID gains and limits |

Custom messages are defined in `interceptor_ws/src/interceptor_interfaces/`: `TargetDetection`, `TargetState`, `GuidanceCommand`, `TargetTrajectory`, `StereoImagePair` and the `SetInterceptMode` service.
