# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SkyInterceptor is a ROS2-based autonomous drone system written in C++17 with two mission modes (see `IMPLEMENTATION_PLAN.md` v2):

- **FOLLOW**: aerial filming drone that follows a person, bicycle or car while always keeping a minimum safety distance (`d_min`) from every person or vehicle and `d_obstacle_min` from every static obstacle. Safety distance is the top-level rule in both modes.
- **INTERCEPT**: counter-UAS capture of an intruding small drone (class `uav` only). Ground targets are never eligible.

It uses stereo vision, a YOLO-based detector (Python), an IMM-EKF tracker, mode-specific planners (follow planner / PN-based intercept guidance), and an independent safety filter node that has the final say on every setpoint. Everything runs in simulation (Gazebo Classic). Agent task prompts live in `docs/AGENT_PROMPTS.md`.

**Roadmap:** [`ROADMAP.md`](ROADMAP.md) is the source of truth for what is done and what is still to do (one section per plan task). Run `python3 scripts/roadmap.py ready` to see what can start, and update the task's section in the same PR as the work (rules at the top of the file). Any progress board is only a view of it; if they disagree, the file wins.

**Status note:** the tables below describe the current code, which now carries the whole pipeline: the tracker, the follow planner, the intercept guidance, the safety filter and the controller. The architecture they implement is in `IMPLEMENTATION_PLAN.md` §3.

All development runs inside a Docker container with CUDA 11.8 + ROS2 Humble.

## Commands

All make targets call `docker compose exec interceptor` internally (with the compose overlay for the host: `docker-compose.wsl.yml` on WSL2, `docker-compose.native.yml` on native Linux, picked from `/proc/version`) — the container must be running (`make up`) first. Persistent data (datasets, weights, batch results) goes in `/workspace/data` (host `~/skyinterceptor-data`), never in the container's `/tmp`.

```bash
make build      # Build Docker image (first time setup)
make up         # Start container in background
make down       # Stop container
make shell      # Open bash shell inside container
make build-ws   # colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
make test       # colcon test + colcon test-result --verbose
make check      # the local mirror of CI: static checks, a -Werror build and the tests
make sim        # ros2 launch interceptor_drone simulation.launch.py
make teleop     # ros2 run interceptor_drone drone_teleop_keyboard.py (fly with the keyboard)
make full       # ros2 launch interceptor_drone interceptor_full.launch.py
make clean      # Remove build/, install/, log/ inside container
make status     # docker compose ps
make platform   # the detected host (wsl/native) and the compose command
make doctor     # check the GPU, the OpenGL renderer, torch CUDA and the mounts
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

**Tests:** `make test` runs the GTest unit tests in `interceptor_drone/test/` (`test_math_utils`, `test_mission`, `test_groundtruth_sensor`, `test_flight_dynamics`, `test_imm_filter`, `test_track_manager`, `test_frame_assembler`, `test_target_selector`, `test_trajectory_controller`, `test_guidance`, `test_target_drone_behaviour`, `test_safety_filter`, `test_follow_planner`) plus the ament linters (uncrustify, cpplint, cppcheck, flake8, pep257, lint_cmake, xmllint). `ament_copyright` is excluded because the sources have no license headers yet. `make check` (`scripts/check.sh`) is the local mirror of CI: the same static checks, then a `-Werror` build and the full test suite in a scratch workspace inside the container. To auto-fix C++ formatting inside the container: `ament_uncrustify --reformat src include test` (from the package directory).

**CI:** `.github/workflows/ci.yml` runs on every PR and on pushes to `main`. It has two jobs: a fast static-checks job (yamllint, shellcheck, Python syntax and `scripts/roadmap.py check`) and a build-and-test job in the `ros:humble-perception` container (`colcon build` with `-Werror`, then `colcon test`). The container has no CUDA, so code must also build without GPU support.

## Architecture

The system is a pipeline of ROS2 nodes organized into six layers:

```
Perception → Estimation → Guidance → Safety → Control → Platform
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
| Guidance | `follow_planner_node` | FOLLOW planner: frames the target from a preset (BEHIND / SIDE / LEAD / ORBIT) in its heading frame; `/target/state` + `/odom` → `/setpoint/raw` (a velocity setpoint, `position_valid = false`). Logic in the ROS-free `follow::FollowPlanner` (`include/follow/`) |
| Guidance | `intercept_guidance_node` | PN / augmented PN laterally plus the terminal capture law, phase machine SEARCH → PURSUE → CAPTURE → DONE. Logic in the ROS-free `guidance::PnGuidance` (`include/guidance/`) |
| Safety | `safety_filter_node` | `/setpoint/raw` + `/tracks` + `/odom` → `/setpoint/safe`: keep-out barrier for every person/vehicle track and static obstacle (inflated by the track covariance), speed/altitude/geofence limits, latching e-stop, INTERCEPT engagement gate. Logic in the ROS-free `safety::SafetyFilter` (`include/safety/`) |
| Simulation | `target_drone_behavior_node` | Flies the target drone (the intruder) with one of five behaviours (hover, waypoints, straight, weave, curve), kinematically through `/set_entity_state`; `/target_drone/odom` for reference only |
| Control | `trajectory_controller_node` | `/setpoint/safe` + `/odom` → `/cmd_vel` (world-frame velocity + yaw rate): position PID with velocity feed-forward, anti-windup (integral zone, clamp, back-calculation), yaw P with rate limit, 0.3 s setpoint timeout. Logic in the ROS-free `control::TrajectoryController` (`include/control/`) |
| Evasion | `evasion_controller_node` | Target evasion strategies (skeleton; the intruder that actually flies is `target_drone_behavior_node`) |
| Mission | `mission_manager_node` | Serves `/mission/set_mode` (`SetMissionMode`), latches `/mission/mode` (`MissionMode`, transient local); always starts disarmed |
| Platform | `quadrotor_dynamics` (Gazebo plugin, `libquadrotor_dynamics_plugin.so`) | Rotor/aero/wind model + onboard flight controller; `/cmd_vel` in, `/odom` + TF out |
| Platform | `drone_teleop_keyboard.py` | Keyboard teleop: `/cmd_vel` + `/drone/arm` |
| Tooling | `record_intercept.py` | Prints the range, closing speed, relative speed and phase of an engagement, for checking the intercept from outside the guidance node |
| Tooling | `intercept_metrics_node`, `intercept_metrics_report`, `run_intercept_batch.py` | The INTERCEPT scenario batch (P3.5): one CSV row per run (capture, envelope, abort, non-uav engagements), scored against the plan's Phase 3 criteria. Logic in the ROS-free `metrics::EngagementScorer` (`include/metrics/intercept_metrics.hpp`); see `docs/INTERCEPT.md` |

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

### Guidance library

`interceptor_drone_guidance` (`include/guidance/`, `src/guidance/`) is plain C++/Eigen with no ROS dependency, like the flight and estimation libraries. See `docs/INTERCEPT.md`:
- `pn_guidance.hpp` — the proportional navigation law: `computeLosKinematics()` (range, LOS unit vector, closing speed `V_c`, LOS rate `lambda_dot`, time to go), `purePn()` (`a = N * V_c * lambda_dot`), `augmentedPn()` (pure PN plus the textbook `(N/2) * a_T_perp` term, the target acceleration projected perpendicular to the LOS), `zeroEffortMiss()` and `saturateAcceleration()`, plus the **terminal capture law**: `closureProfile()` (the braking curve of a pursuer), `closureSpeed()` (the profile corrected by the measured closing speed) and `captureVelocitySetpoint()`, which commands the relative velocity `v_M = v_T + u·v_close(r) + a_pn·tau`
- `target_drone_behaviour.hpp` — the flight behaviours of the intruder (hover, waypoints, straight, weave, curve). Every behaviour keeps two properties: the published position is the exact integral of the published velocity, and the published velocity is continuous

`intercept_guidance_node` wraps the first: the SEARCH → PURSUE → CAPTURE → DONE phase machine, the engagement gate, the setpoint and the `GuidanceCommand` telemetry. The capture is flown, not detected: the terminal capture law commands the relative velocity (the target's velocity as a feedforward, a range-governed closure along the LOS, and the PN lateral term), so the interceptor arrives inside the envelope at 1.5-3.3 m/s relative speed instead of flying through the target at 12 m/s. The law and its numerical sweep live in `include/guidance/pn_guidance.hpp` and `test/test_guidance.cpp`.

`target_drone_behavior_node` wraps the second: it integrates the behaviour and pushes the pose into Gazebo with `/set_entity_state`. The behaviour is chosen with the `behaviour` parameter and can be switched at runtime (`ros2 param set /target_drone_behavior_node behaviour weave`); the launch argument `target_behaviour` picks it at startup. It publishes `/target_drone/odom` and `/target_drone/accel` for reference only — the guidance never reads them, it only sees the tracker's estimate.

The guidance also commands the yaw: the stereo camera hangs off the front of the airframe, so the nose has to follow the target for the cameras to see it. The yaw setpoint is the bearing of the target in the world frame.

### Follow planner library

`interceptor_drone_follow` (`include/follow/`, `src/follow/`) is plain C++/Eigen with no ROS dependency, like the other libraries. See `docs/FOLLOW.md`:
- `follow_planner.hpp` — the framing presets in the target's heading frame (`p_des = p_t + R_z(psi_t)·[dx, dy, dz]`), the low-pass filter on the target's heading, the velocity feed-forward and the lost-track machine (COAST → HOLD → RTL)

`follow_planner_node` wraps it: `/target/state` + `/odom` + `/mission/mode` → `/setpoint/raw` (a velocity setpoint, `position_valid = false`, `source = SOURCE_FOLLOW`) and `/drone/arm`. It publishes **nothing** when the mode is not FOLLOW, so it can never fight the intercept guidance on the same topic. The safety filter owns `/setpoint/safe` and the controller flies that.

### Control library

The trajectory controller is ROS-free too (`include/control/trajectory_controller.hpp`, `control::TrajectoryController`), but it is compiled into `interceptor_drone_lib` rather than into a library of its own:
- `trajectory_controller.hpp` — the cascade: a position PID with velocity feed-forward, anti-windup (integral zone, clamp, back-calculation) and a yaw P loop with a rate limit; a stale setpoint or stale odometry commands zero velocity

`trajectory_controller_node` wraps it: `/setpoint/safe` + `/odom` → `/cmd_vel` (world-frame linear velocity + yaw rate). The gains and the output limits are in `config/controller_params.yaml`.

### Safety filter library

`interceptor_drone_safety` (`include/safety/`, `src/safety/`) is plain C++/Eigen with no ROS dependency, like the flight and estimation libraries:
- `safety_filter.hpp` — the keep-out barrier and the flight limits. The barrier is a half-space in the *velocity*, because the controller flies a velocity loop.

`safety_filter_node` is the only writer of `/setpoint/safe`, so the safety filter is always in series with the planner: `follow_planner_node` or `intercept_guidance_node` → `/setpoint/raw` → `safety_filter_node` → `/setpoint/safe` → `trajectory_controller_node`. It also publishes `/mission/status` (what it did and why) and the keep-out circles on `/safety/keepouts` for RViz.

**The barrier is relative degree 2, not 1.** The plan's condition `ḣ ≥ −α·h` constrains the
*commanded* velocity, but the distance depends on the *achieved* velocity, and the
autopilot's velocity loop lags: the time constant measured on the X500 flight model is
0.34 s. The relative degree 1 form overshoots the keep-out margin by up to 6 m against a
5 m standoff, so the filter takes the stricter of the plan's bound and the lag correction
`c_cmd ≤ c·(1 − 2ω·τ) + ω²·τ·h`. `velocity_lag_tau` and `barrier_omega` are in
`config/safety_params.yaml`; `docs/SAFETY.md` has the sweep and says what to re-check
after changing them.

### TF structure

Two trees hang off `map`, and both need a root transform that nothing else
provides:

```
map -> odom -> base_link -> prop_*  (the dynamics plugin, and robot_state_publisher)
map -> target_base_link -> target_prop_*  (target_drone_behavior_node, and its
                                             own robot_state_publisher)
```

`robot_state_publisher` never publishes a transform for its **root** link: the
root appears only as the parent of its children. The interceptor's root comes
from the dynamics plugin (`odom -> base_link`), but the target drone is moved by
`/set_entity_state` and has no plugin, so `target_drone_behavior_node`
broadcasts `map -> target_base_link` itself. Without it RViz reports the
target's propeller frames as missing.

Each `robot_state_publisher` also needs the joint states of **its own** model:
the interceptor's props come from `/joint_states` (the plugin) and the target's
from `/target_drone/joint_states` (the behaviour node). Pointing the target's
publisher at the global `/joint_states` makes it look for the interceptor's
joint names.

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

Every file uses `<node_name>: ros__parameters:` and is loaded by the launch files (`parameters=[yaml_path, {'use_sim_time': ...}]`); never hard-code tunables in launch files. `follow_params.yaml` and `intercept_params.yaml` use `/**:` because several nodes (planner, tracker, safety filter) read them; `target_drone_params.yaml` is a per-node file, so the `/**:` block of `intercept_params.yaml` does not leak into the behaviour node. Key values to know:

| File | Notable params |
|---|---|
| `perception_params.yaml` | `stereo.baseline=0.12m`, `stereo.fx=554.4`, SGBM `num_disparities=64`, the detector's class list (`person, bicycle, car, truck`) |
| `stereo_sync_params.yaml` | sync tolerance `5ms` |
| `ekf_params.yaml` | process noise 0.3 / 1.0 / 0.2, IMM sets per class, transition matrices per 1 s, gate 0.99, 3-of-5, coast 2 s, frame grouping 5 ms / 10 ms, selection switch margin 3 m |
| `groundtruth_params.yaml` | entities + classes (`target_person` as `person`, `target_drone` as `uav`), `rate_hz=30`, `pos_noise_std=0.2`, `dropout_prob=0.05`, tree occlusion cylinders |
| `gazebo_params.yaml` | gzserver `/clock` rate (250 Hz; the 10 Hz default caps every sim-time timer) |
| `controller_params.yaml` | `kp/ki/kd_pos=1.0/0.05/0.2`, anti-windup (`integral_zone=2m`, `max_integral_velocity=1m/s`), `kp_yaw=1.5` and `max_yaw_rate=1.5` (the planners send a bearing-rate yaw feed-forward), output limits, `setpoint_timeout=0.3s` |
| `safety_params.yaml` | `d_min_horizontal=5.0`, `d_obstacle_min=3.0`, `h_min_above=3.0`, speed caps 15 / 30 m/s, the obstacle list (the trees and benches of the world file), `velocity_lag_tau=0.5` and `barrier_omega=3.0` (the lag correction, `docs/SAFETY.md`), the engagement band 5 m with a 2 m hysteresis once engaged (an armed mission may climb, not close, while the gate is shut) |
| `follow_params.yaml` | preset `BEHIND`, offsets behind/side/height 8/6/5 m, `heading_tau=0.8s`, `lost_hold_time=2s`, `lost_rtl_time=10s`, eligible `person, bicycle, car` |
| `intercept_params.yaml` | `guidance.law=apn` (`pn`/`apn`), `nav_constant=4.0`, `max_accel=15`, `target_accel_gain=0.5` (the textbook (N/2) factor), `hold_radius=1.2`, `approach_decel=2.0`, `closing_speed_feedback=0.5`, `capture_radius=1.5`, `capture_max_rel_speed=5.0`, `min_engage_altitude_agl=10`, eligible `uav` |
| `target_drone_params.yaml` | the intruder's behaviour (`hover, waypoints, straight, weave, curve`), its start pose, the waypoint loop, and the speeds of each behaviour. It is a per-node file, not a `/**:` one, so the `/**:` block of `intercept_params.yaml` does not leak into the behaviour node |

### Launch files (`launch/` directory)

- `interceptor_full.launch.py` — Full system (simulation + perception + mission manager + tracker + planner + safety filter + control + RViz). Args: `mission_mode:=follow|intercept` (default follow), `perception_source:=vision|groundtruth` (default groundtruth; vision launches the perception pipeline), `guidance_law:=pn|apn`, `target_behaviour:=hover|waypoints|straight|weave|curve`, `auto_arm:=true|false`, `groundtruth_seed:=0`, `interceptor_x`, `interceptor_y`, `target_x`, `target_y`
- `follow.launch.py` / `intercept.launch.py` — `interceptor_full` with the mode set
- `intercept_demo.launch.py` — The INTERCEPT demo: the Crazyflie intruder flies while the interceptor engages it with PN or augmented PN. Arms the mission automatically. Args: `guidance_law`, `target_behaviour`, `auto_arm` (default true), `groundtruth_seed`, `perception_source`
- `intercept_scenario.launch.py` — One scripted, headless INTERCEPT run plus `intercept_metrics_node`; driven by `run_intercept_batch.py`, which writes the intruder's parameter file per run (`target_drone_params_file`). Args: `scenario`, `seed`, `output_path`, `timeout`, `guidance_law`, spawn positions, `gui`, `rviz`
- `simulation.launch.py` — Gazebo + drone + target drone (args: `gui`, `x`, `y`, `z`, `yaw`, `target_x`, `target_y`, `target_z`, `target_yaw`, `wind_x`, `wind_y`, `wind_z`, `wind_gust_stddev`, `spawn_timeout` (default 180 s: `gzserver` fetches the Gazebo model database on the first start, so `/spawn_entity` is not available for over a minute), `command_frame`: `heading` for the keyboard teleop (default), `world` for `trajectory_controller_node`; `interceptor_full` sets `world`)
- `perception.launch.py` — Perception pipeline only (`perception_source:=vision`)
- `groundtruth.launch.py` — `groundtruth_target_node` only (`perception_source:=groundtruth`, arg `seed`)
- `guidance.launch.py` — Tracker, plus the mode's planner: the follow planner in FOLLOW, the intercept guidance and the target drone in INTERCEPT

## ROS2 Workspace Layout

```
interceptor_ws/
  src/
    interceptor_drone/          # Main C++ package
      include/common/           # Shared headers (types, params, math)
      include/flight/           # Flight dynamics headers
      include/estimation/       # IMM-EKF and track management headers
      include/guidance/         # PN law, capture law, target drone behaviours
      include/follow/           # Follow planner headers
      include/control/          # Trajectory controller headers
      include/safety/           # Safety filter headers
      src/
        common/                 # Shared library sources
        perception/             # stereo_sync, depth_processor, 3d_localizer, target_detector.py, groundtruth_target_node
        estimation/             # IMM-EKF library (no ROS) + target_tracker_node
        guidance/               # Guidance library (no ROS) + intercept_guidance_node, target_drone_behavior_node
        follow/                 # Follow planner library (no ROS) + follow_planner_node
        control/                # trajectory_controller
        safety/                 # safety filter library (no ROS) + safety_filter_node
        evasion/                # evasion_controller_node (stub)
        mission/                # mission_manager_node
        flight/                 # flight dynamics library (no ROS)
        simulation/             # quadrotor_dynamics Gazebo plugin
        teleop/                 # drone_teleop_keyboard.py
        tools/                  # record_intercept.py, run_intercept_batch.py
        metrics/                # intercept metrics library (no ROS), node and report tool
      config/                   # YAML parameter files
      launch/                   # Python launch files
      urdf/                     # Robot model (Holybro X500 V2 look) + the target drone
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

## Documentation

| Document | What it explains |
|---|---|
| [`README.md`](README.md) | The front door: what the system does, how to run it, the architecture at a glance |
| [`ROADMAP.md`](ROADMAP.md) | What is done and what is still to do, one section per plan task |
| [`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md) | The architecture, the topics, the messages and the default parameters the plan asks for |
| [`docs/TRACKER.md`](docs/TRACKER.md) | The IMM-EKF, the track management, the frame grouping and the target selection |
| [`docs/INTERCEPT.md`](docs/INTERCEPT.md) | The PN law, the terminal capture law, the phases and the intruder |
| [`docs/FOLLOW.md`](docs/FOLLOW.md) | The framing presets, the heading smoothing, the lost-track machine and the safety filter in series |
| [`docs/SAFETY.md`](docs/SAFETY.md) | The keep-out barrier, the lag correction, the flight limits and the engagement gate |
| [`docs/FLIGHT_DYNAMICS.md`](docs/FLIGHT_DYNAMICS.md) | The flight model, the onboard flight controller, the wind and the teleop |
| [`docs/AGENT_PROMPTS.md`](docs/AGENT_PROMPTS.md) | The task prompts for coding agents |

## Code Style

Follow the ROS2 C++ style guide: `PascalCase` classes, `camelCase` methods, `snake_case` variables/members, `UPPER_CASE` constants. Compiler flags: `-Wall -Wextra -Wpedantic -O3`.

## Roadmap

`ROADMAP.md` is the source of truth for what is done and what is still to do. Update the section of the task you are working on **in the same PR as the work** (the rules are at the top of that file), and run `python3 scripts/roadmap.py check` before you push: CI runs it. `python3 scripts/roadmap.py ready` lists the tasks whose dependencies are all met.
