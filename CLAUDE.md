# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SkyInterceptor is a ROS2-based autonomous drone system written in C++17 with two mission modes (see `IMPLEMENTATION_PLAN.md` v2):

- **FOLLOW**: aerial filming drone that follows a person, bicycle or car while always keeping a minimum safety distance (`d_min`) from every person or vehicle and `d_obstacle_min` from every static obstacle. Safety distance is the top-level rule in both modes.
- **INTERCEPT**: counter-UAS capture of an intruding small drone (class `uav` only). Ground targets are never eligible.

It uses stereo vision, a YOLO-based detector (Python), an IMM-EKF tracker, mode-specific planners (follow planner / PN-based intercept guidance), and an independent safety filter node that has the final say on every setpoint. Everything runs in simulation (Gazebo Classic). Agent task prompts live in `docs/AGENT_PROMPTS.md`.

**Status note:** the tables below describe the current code. The target architecture (new nodes such as `safety_filter_node` and `follow_planner_node`) is in `IMPLEMENTATION_PLAN.md` §3.

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
make teleop     # ros2 run interceptor_drone drone_teleop_keyboard.py (fly with the keyboard)
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

**Tests:** `make test` runs the GTest unit tests in `interceptor_drone/test/` (`test_math_utils`, `test_mission`, `test_groundtruth_sensor`, `test_flight_dynamics`, `test_imm_filter`, `test_track_manager`, `test_frame_assembler`, `test_target_selector`) plus the ament linters (uncrustify, cpplint, cppcheck, flake8, pep257, lint_cmake, xmllint). `ament_copyright` is excluded because the sources have no license headers yet. GTest targets for guidance and controller are still commented out in `CMakeLists.txt` until those tests exist. To auto-fix C++ formatting inside the container: `ament_uncrustify --reformat src include test` (from the package directory).

**CI:** `.github/workflows/ci.yml` runs on every PR and on pushes to `main`. It has two jobs: a fast static-checks job (yamllint, shellcheck, Python syntax) and a build-and-test job in the `ros:humble-perception` container (`colcon build` with `-Werror`, then `colcon test`). The container has no CUDA, so code must also build without GPU support.

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
| Perception | `groundtruth_target_node` | Gazebo entity poses (`/get_entity_state`) + noise, dropout and tree occlusion → `/target/detection_3d` (used when `perception_source:=groundtruth`) |
| Estimation | `target_tracker_node` | IMM-EKF library: frames by stamp → tracks; `/tracks`, selected `/target/state`, `/tracks/markers` |
| Guidance | `guidance_controller_node` | Proportional Navigation / Augmented PN (skeleton) |
| Control | `trajectory_controller_node` | Cascade PID |
| Evasion | `evasion_controller_node` | Target evasion strategies (skeleton) |
| Mission | `mission_manager_node` | Serves `/mission/set_mode` (`SetMissionMode`), latches `/mission/mode` (`MissionMode`, transient local); always starts disarmed |
| Platform | `quadrotor_dynamics` (Gazebo plugin, `libquadrotor_dynamics_plugin.so`) | Rotor/aero/wind model + onboard flight controller; `/cmd_vel` in, `/odom` + TF out |
| Platform | `drone_teleop_keyboard.py` | Keyboard teleop: `/cmd_vel` + `/drone/arm` |

### Flight dynamics library

`interceptor_drone_flight` (`include/flight/`, `src/flight/`) is plain C++/Eigen with no ROS or Gazebo dependency, so gtests can fly the drone in a standalone 6-DOF sim:
- `quadrotor_model.hpp` — motor lag, rotor thrust/reaction torque/rotor drag, airframe drag, ground effect; `integrateRigidBody` for tests
- `wind_model.hpp` — steady wind + Gauss–Markov gusts
- `flight_controller.hpp` — velocity PI → attitude P (SO(3)) → body-rate PI → mixer, landed/take-off state

The Gazebo plugin (`src/simulation/quadrotor_dynamics_plugin.cpp`) wraps it; its parameters are in the `<plugin>` block of the URDF xacro. Frames are ENU world / FLU body. See `docs/FLIGHT_DYNAMICS.md`.

### Estimation library

`interceptor_drone_estimation` (`include/estimation/`, `src/estimation/`) is plain C++/Eigen with no ROS dependency, like the flight library. See `docs/TRACKER.md`:
- `ekf_models.hpp` — CV / CA / CT EKF models on one padded 10-state layout `[p, v, a, omega]`, Joseph-form position update
- `imm_filter.hpp` — IMM: mixing (borrows the receiving model's estimate for components the source model lacks), time-scaled Markov chain (`transition_interval`), log-domain mode probabilities, const `extrapolate()` for publishing between frames
- `track_manager.hpp` — multi-target tracking: chi-square gate (same class only), Hungarian GNN with confirmed tracks first, spawn gate against duplicates, 3-of-5 confirmation, coasting, deletion, heading. Model set per class: `uav` → CV+CT, else CV+CA
- `frame_assembler.hpp` — groups single `TargetDetection` messages into frames by stamp (tolerance 5 ms; closed by a newer frame or a 10 ms timeout; late detections dropped). One frame = one `processFrame()` call
- `target_selector.hpp` — `/target/state` selection: mode class whitelist only, operator track (no fallback), else closest confirmed valid track with a switch margin

`target_tracker_node` wraps these: 50 Hz timer on the node clock (close frames, prune, extrapolate, select, publish), resets on a backward clock jump.

Its defaults match `config/ekf_params.yaml`; they were tuned on the synthetic trajectories in the tests, so re-run them after changing noise or IMM parameters.

### Shared library

`interceptor_drone_lib` is linked by every node and contains:
- `include/common/types.hpp` — `Detection`, `TargetState`, `GuidanceOutput` structs (Eigen3-based)
- `include/common/mission.hpp` — `MissionMode` and `TargetClass` enums (values match the msg constants), string parsing, `isGroundClass`
- `include/common/math_utils.hpp` — Quaternion conversion, skew-symmetric matrices, vector saturation
- `include/perception/groundtruth_sensor.hpp` — Ground-truth measurement model (Gaussian noise, dropout) and line-of-sight occlusion by vertical cylinders
- `src/common/mission.cpp` / `src/common/math_utils.cpp` — implementations

There is no global parameter class: each node declares and reads its own parameters.

Everything lives under the `interceptor` namespace.

### Custom ROS2 messages (`interceptor_interfaces` package)

- `TargetDetection.msg` — 2D bounding box + 3D position; class constants `PERSON=0, CAR=1, TRUCK=2, BICYCLE=3, UAV=4`
- `TargetState.msg` — Filtered position, velocity, acceleration, covariances, class, heading, `confirmed` and `is_valid`
- `TargetStateArray.msg` — All tracks (`/tracks`)
- `FlightSetpoint.msg` — Mode-agnostic setpoint (`/setpoint/raw` → safety filter → `/setpoint/safe`)
- `MissionMode.msg` — Current mode, armed flag, operator track id (`/mission/mode`)
- `MissionStatus.msg` — Mode, phase, target, safety-filter activity (`/mission/status`)
- `GuidanceCommand.msg` — Acceleration commands, navigation constants, intercept params
- `TargetTrajectory.msg` — Predicted trajectory
- `StereoImagePair.msg` — Synchronized stereo pair with calibration
- `SetMissionMode.srv` — Mode switching service (HOLD / FOLLOW / INTERCEPT, armed, track id)

### Configuration (`config/` directory)

Every file uses `<node_name>: ros__parameters:` and is loaded by the launch files (`parameters=[yaml_path, {'use_sim_time': ...}]`); never hard-code tunables in launch files. `follow_params.yaml` and `intercept_params.yaml` use `/**:` because several nodes (planner, tracker, safety filter) read them. Keys for stub nodes are marked "reserved for <task>". Key values to know:

| File | Notable params |
|---|---|
| `perception_params.yaml` | `baseline=0.12m`, `fx=535.4` |
| `stereo_sync_params.yaml` | sync tolerance `5ms` |
| `ekf_params.yaml` | process noise 0.3 / 1.0 / 0.2, IMM sets per class, transition matrices per 1 s, gate 0.99, 3-of-5, coast 2 s, frame grouping 5 ms / 10 ms, selection switch margin 3 m |
| `groundtruth_params.yaml` | entities + classes, `rate_hz=30`, `pos_noise_std=0.2`, `dropout_prob=0.05`, tree occlusion cylinders |
| `gazebo_params.yaml` | gzserver `/clock` rate (250 Hz; the 10 Hz default caps every sim-time timer) |
| `controller_params.yaml` | PID gains and output limits (reserved for P2.1) |
| `safety_params.yaml` | `d_min_horizontal=5.0`, speed caps 15 / 30 m/s |
| `follow_params.yaml` | preset `BEHIND`, eligible `person, bicycle, car` |
| `intercept_params.yaml` | `nav_constant=4.0`, `capture_radius=1.5`, eligible `uav` |

### Launch files (`launch/` directory)

- `interceptor_full.launch.py` — Full system (simulation + perception + mission manager + guidance + control + RViz). Args: `mission_mode:=follow|intercept` (default follow), `perception_source:=vision|groundtruth` (default groundtruth; vision launches the perception pipeline)
- `follow.launch.py` / `intercept.launch.py` — `interceptor_full` with the mode set
- `simulation.launch.py` — Gazebo + drone (args: `gui`, `x`, `y`, `yaw`, `wind_x`, `wind_y`, `wind_z`, `wind_gust_stddev`)
- `perception.launch.py` — Perception pipeline only (`perception_source:=vision`)
- `groundtruth.launch.py` — `groundtruth_target_node` only (`perception_source:=groundtruth`)
- `guidance.launch.py` — Tracker, plus the guidance node in intercept mode

## ROS2 Workspace Layout

```
interceptor_ws/
  src/
    interceptor_drone/          # Main C++ package
      include/common/           # Shared headers (types, params, math)
      include/flight/           # Flight dynamics headers
      include/estimation/       # IMM-EKF and track management headers
      src/
        common/                 # Shared library sources
        perception/             # stereo_sync, depth_processor, 3d_localizer, target_detector.py, groundtruth_target_node
        estimation/             # IMM-EKF library (no ROS) + target_tracker_node
        guidance/               # guidance_controller_node (PN)
        control/                # trajectory_controller
        evasion/                # evasion_controller_node
        mission/                # mission_manager_node
        flight/                 # flight dynamics library (no ROS)
        simulation/             # quadrotor_dynamics Gazebo plugin
        teleop/                 # drone_teleop_keyboard.py
      config/                   # YAML parameter files
      launch/                   # Python launch files
      urdf/                     # Robot model (Holybro X500 V2 look)
      meshes/                   # Visual meshes used by the URDF (from PX4, BSD-3)
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
