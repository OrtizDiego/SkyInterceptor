# SkyInterceptor – Implementation Plan v2.1

**Version:** 2.1
**Last Updated:** 2026-10-03
**Platform:** Simulation (ROS 2 Humble + Gazebo Classic 11)
**Replaces:** v1.0 (2026-02-05)

---

## 1. Scope

The project now has **two mission modes** that share one perception → tracking → control pipeline:

| | **FOLLOW** (aerial filming) | **INTERCEPT** (counter-UAS) |
|---|---|---|
| Use case | Camera drone that follows an athlete, cyclist or car | Capture an intruding small drone |
| Eligible targets | `person`, `bicycle`, `car` | `uav` only |
| Objective | Hold a camera framing at a standoff offset | Reach the capture envelope of the target drone (net-capture style: close range at low relative speed) |
| Hard constraint | Never closer than `d_min` (default 5 m horizontal, 3 m above) to **any** person or vehicle, and `d_obstacle_min` (3 m) to any static obstacle | Same keep-out distances as FOLLOW; never engage a non-aerial track; abort if a person, vehicle or obstacle is near the predicted capture point |
| Speed cap | 15 m/s | 30 m/s |

**Design principle: keep your distance.** In both modes the drone never touches or approaches a person, vehicle or static obstacle (trees, benches, buildings) closer than a configured standoff distance. This holds whatever a planner commands, and it is enforced by the safety filter. The only thing the drone ever closes in on is a hostile `uav` in INTERCEPT mode, and even there it arrives at a low relative speed (a net-capture envelope, not an impact), and never near people, vehicles or obstacles.

| Keep-out class | Default standoff | Applies in |
|---|---|---|
| `person` | 5 m horizontal, 3 m above | both modes |
| `bicycle`, `car`, `truck` | 5 m horizontal, 3 m above | both modes |
| Static obstacle (tree, bench, building, pole) | 3 m from the obstacle surface | both modes |

**Removed from v1:** collision or impact with ground targets, "terminal collision guidance", the car/person target vehicle, and evasion by ground targets. The evasion node becomes the behaviour script for the simulated **target drone** (intercept mode only).

---

## 2. Status quo (2026-09-24)

| Area | State | Notes |
|---|---|---|
| Docker image | ✅ Builds | `make build` (log written to `logs/docker-build.log`, not committed) |
| Workspace build | ❓ Never verified | No `colcon build` on record. Humble header and include fixes pushed in `feat/funny-albattani-l2q2c6` |
| Interfaces | ✅ Done (P0.2) | `TargetStateArray`, `FlightSetpoint`, `MissionMode`, `MissionStatus`, `SetMissionMode` added; class constants in `TargetDetection` |
| `stereo_sync_node` | ✅ Done | Timestamp bug fixed (kept the capture stamp) |
| `stereo_depth_processor` | ✅ Done (CPU) | SGBM + WLS on CPU, realistically 10–20 FPS rather than 60. Needs `opencv_ximgproc` (contrib) in the image |
| `target_detector.py` | ✅ Done | COCO person/car/truck on **CPU**. No `bicycle`, and COCO has no drone class |
| `target_3d_localizer` | ✅ Done | Needs a TF path from the camera frame to `map` (not published yet: no odometry) |
| `target_tracker_node` | ✅ Done (P1.1, P1.2) | IMM-EKF library `interceptor_drone_estimation` with tests, node publishes `/tracks`, the selected `/target/state` and `/tracks/markers`; detections are grouped into frames by stamp (`docs/TRACKER.md`). Not yet run against `groundtruth_target_node` in Gazebo |
| `guidance_controller_node` | ❌ Stub | Replaced by `follow_planner_node` and `intercept_guidance_node` |
| `trajectory_controller_node` | ✅ Done (P2.1) | `/setpoint/safe` + `/odom` → `/cmd_vel`. ROS-free `control::TrajectoryController` with tests, also closed-loop on the 6-DOF model |
| `hector_interface_node` | 🗑️ Deleted | Superseded by the `quadrotor_dynamics` Gazebo plugin |
| `evasion_controller_node` | ❌ Stub | Becomes `target_drone_behavior_node` |
| Simulation | ✅ Flies | `quadrotor_dynamics` Gazebo plugin: rotor thrust and torques, motor lag, airframe and rotor drag, wind with gusts, ground effect, plus an onboard velocity → attitude → rate controller. Publishes `/odom` and TF. Keyboard teleop node. See `docs/FLIGHT_DYNAMICS.md`. World has a walking `target_person` actor and no target drone |
| Parameters | ✅ Done (P0.2) | Launch files load `config/*.yaml` (`<node>: ros__parameters:`); keys for stub nodes are marked reserved for their task. Static `Parameters` class removed. `mission_mode` and `perception_source` launch args |
| `mission_manager_node` | ✅ Done (P0.2) | Serves `/mission/set_mode`, latches `/mission/mode` (`MissionMode`, transient local). Starts disarmed; only INTERCEPT can be armed |
| Tests | ❌ None | gtest targets are commented out in `CMakeLists.txt` |

**Critical path:** the drone can't fly and nothing downstream of perception exists. The fastest route is (1) a kinematic flyable drone, (2) a ground-truth target source so control work doesn't wait for vision, (3) FOLLOW mode end-to-end, then (4) INTERCEPT mode.

---

## 3. Architecture

```
 stereo + YOLO + localizer ─┐
                            ├─► /target/detection_3d ─► target_tracker_node ─► /tracks (all tracks)
 groundtruth_target_node ───┘   (param perception_source: vision | groundtruth)
                                                             │
                                          target_selector (mode-aware class whitelist)
                                                             │ /target/state
                         ┌───────────────────────────────────┴──────────────────────┐
                 follow_planner_node  (FOLLOW)                  intercept_guidance_node (INTERCEPT)
                         └──────────────┬────────────────────────────────────────────┘
                                        │ /setpoint/raw   (FlightSetpoint)
                              safety_filter_node   ◄── /tracks, /odom, /mission/estop, obstacles (config)
                                        │ /setpoint/safe
                           trajectory_controller_node  ◄── /odom
                                        │ /cmd_vel
                    quadrotor_dynamics plugin (in Gazebo)  ──► /odom, TF odom→base_link
                    (flight controller + rotor/aero model)
```

`safety_filter_node` is always in the loop and has the final say, whatever the planner commands.

### Node inventory

| Node | Status | Mode |
|---|---|---|
| `stereo_sync_node`, `stereo_depth_processor`, `target_3d_localizer`, `target_detector.py` | keep | both |
| `groundtruth_target_node` (new) | Reads Gazebo entity states, publishes noisy `TargetDetection` | both |
| `target_tracker_node` | ✅ done (IMM-EKF, multi-track, publishes `/tracks`) | both |
| `target_selector` | ✅ done, a component inside the tracker node (`TargetSelector`) | both |
| `mission_manager_node` (new) | ✅ done: `/mission/set_mode` → latched `/mission/mode` | both |
| `follow_planner_node` (new) | | FOLLOW |
| `intercept_guidance_node` (replaces `guidance_controller_node`) | | INTERCEPT |
| `safety_filter_node` (new) | | both |
| `trajectory_controller_node` | ✅ done | both |
| `quadrotor_dynamics` Gazebo plugin (replaces `hector_interface_node`) | ✅ done | both |
| `target_drone_behavior_node` (replaces `evasion_controller_node`) | | INTERCEPT sim only |

### Interface changes (`interceptor_interfaces`)

- `TargetDetection.msg`: class IDs become `0=person, 1=car, 2=truck, 3=bicycle, 4=uav`. Add constants.
- `TargetState.msg`: add `int32 class_id`, `string class_name`, `float64 heading`.
- **new** `TargetStateArray.msg`: `std_msgs/Header header`, `TargetState[] tracks`.
- **new** `FlightSetpoint.msg`: header, `geometry_msgs/Point position`, `geometry_msgs/Vector3 velocity`, `geometry_msgs/Vector3 acceleration_ff`, `float64 yaw`, `float64 yaw_rate`, `uint8 source` (FOLLOW / INTERCEPT / HOLD / RTL), `bool position_valid`.
- **new** `MissionStatus.msg`: mode, phase, target track id, distance to target, safety-filter active flag, reason string.
- **replace** `SetInterceptMode.srv` with `SetMissionMode.srv`: request `uint8 mode` (HOLD=0, FOLLOW=1, INTERCEPT=2), `bool armed`, `int32 track_id` (-1 = auto). Response `bool success`, `string message`.
- **new** `MissionMode.msg` (header, mode, armed, track_id): `mission_manager_node` serves `/mission/set_mode` and latches the result on `/mission/mode` (transient local), so the tracker, planners and safety filter all see the same mode.
- `GuidanceCommand.msg`: keep as intercept telemetry (N', V_c, t_go, ZEM). It's not a control input.

---

## 4. Algorithms

### 4.1 Tracker (shared)
- Per-track IMM with **CV + CA** (FOLLOW, ground targets, z weakly constrained) or **CV + CT** (INTERCEPT, 3D). Pick the model set per class.
- Gating on the Mahalanobis distance of the innovation (χ², 3 DOF, 99%). Association by global nearest neighbour (Hungarian), confirmed tracks first. Nearest neighbour lost ids on crossing targets.
- Mixing across models of unequal dimension borrows the receiving model's estimate (not zero padding), and the Markov transition matrix is defined per second and rescaled to the actual time step. Both were needed for the IMM to switch modes at 30 Hz (`docs/TRACKER.md`).
- Track management: confirm after M-of-N (3 of 5), delete after `max_missed_frames` or covariance growth. Keep predicting through occlusion for up to 2 s, then flag the track `is_valid=false`.
- Heading = `atan2(vy, vx)` when speed > 0.5 m/s; otherwise hold the last value.
- Publish `/tracks` at 50 Hz (predict-only between measurements).

### 4.2 FOLLOW planner
- Framing presets in the target's heading frame: `BEHIND`, `SIDE`, `LEAD` (in front, looking back), `ORBIT` (fixed angular rate around the target).
- Desired position: `p_des = p_t + R_z(ψ_t) · [dx, dy, dz]`. Desired velocity: `v_des = v_t + K_p (p_des − p)`, capped at 15 m/s. Yaw points at the target.
- Smooth the target heading (low-pass or rate limit) so the camera doesn't whip around when the athlete turns.
- Lost track → HOLD (hover), then RTL after `lost_timeout` (default 10 s).

### 4.3 Safety filter (shared, independent node)
- **Keep-out barrier** (control barrier function) for every person or vehicle track, not only the selected one:
  `h = ‖p_xy − p_t,xy‖ − d_min` and require `ḣ ≥ −α·h`.
  In closed form, project `v_cmd` onto the half-space `n·(v − v_t) ≥ −α·h`, where `n` is the unit vector from the target to the drone. With several constraints, apply them sequentially or with a tiny QP.
- **Static obstacles** (trees, benches, buildings) use the same barrier with `v_t = 0`: `h = ‖p_xy − c_o,xy‖ − (r_o + d_obstacle_min)`, where `c_o` and `r_o` are the obstacle's centre and radius. They come from `safety_params.yaml` (`obstacles:` list, generated from the world file) and are not tracked at runtime.
- Vertical keep-out: never descend below `z_t + h_min_above` while horizontally inside `2·d_min`.
- **Inflate for uncertainty:** the effective `d_min` for a track is `d_min + k·σ_pos`, using the position covariance from the tracker, so a poor track gets a larger margin. A track that goes stale keeps its last known keep-out for `coast_timeout`, then the filter holds position rather than forgetting it.
- Feasibility: if the constraints cannot all be satisfied (for example the drone is boxed in), command hover and set `MissionStatus.reason`. Never relax a distance to make a command feasible.
- Global limits: speed cap per mode, altitude floor and ceiling, geofence box, stale odometry or stale setpoint (> 0.3 s) → brake to hover.
- `/mission/estop` (std_msgs/Bool) latches HOLD.
- Publishes `MissionStatus` with `safety_active=true` whenever it modifies the command.
- INTERCEPT engagement gate (§4.4) is also enforced here, so a planner bug cannot bypass it.

### 4.4 INTERCEPT guidance
- Phases: `SEARCH` (loiter or scan) → `PURSUE` (APN) → `CAPTURE` → `DONE` / `ABORT`.
- PURSUE: augmented PN on the line of sight, using standard textbook forms (`a = N'·V_c·ω + (N'/2)·a_t⊥`), with N' ∈ [3, 5] and saturation at `max_accel`.
- CAPTURE: when `range < capture_start_range`, switch to relative-velocity matching so the drone arrives inside `capture_radius` (default 1.5 m) with relative speed below `capture_max_rel_speed` (default 3 m/s). This models a net-capture intercept, not an impact.
- **Engagement gate:** all of the following must hold, or the planner outputs HOLD:
  - The track's class is `uav` and it has been confirmed for ≥ 10 frames.
  - The target is ≥ `min_engage_altitude_agl` (default 10 m) above ground.
  - The target is inside the geofence.
  - The operator has armed via `SetMissionMode(armed=true)`.
- **Abort:** any person or vehicle track within `abort_ground_radius` (default 15 m, horizontal) of the predicted capture point, a static obstacle within `d_obstacle_min` of it, the target descending below the altitude floor, or the track becoming invalid.
- The keep-out barrier from §4.3 stays active in INTERCEPT mode for all people, vehicles and obstacles. The intercept path may be bent or stopped by it.

### 4.5 Trajectory controller (shared)
- Position PID → velocity command: `v = v_ff + kp·e + I + kd·(v_ff − v)`. The drone's onboard flight controller already closes the velocity loop (§4.6), so the controller stops at the velocity command; the setpoint acceleration is not used. With `position_valid = false` the setpoint velocity is tracked directly. Yaw: P on the wrapped error plus the yaw-rate feed-forward, rate limited. Zero velocity when no setpoint arrived for 0.3 s or `/odom` is stale.
- Anti-windup: the integral only accumulates within `integral_zone` of the setpoint (a long approach would otherwise overshoot), is clamped, and back-calculation unwinds it while the output is saturated. It is reset on a change of setpoint source, of `position_valid`, and of the mission mode.
- Output `/cmd_vel` (Twist: world-frame linear velocity and yaw rate). The plugin reads it in the heading frame by default, like teleop; `interceptor_full` launches the simulation with `command_frame:=world`.

### 4.6 Simulated drone (`quadrotor_dynamics` Gazebo plugin) ✅
- Force-based model instead of the planned kinematic bridge. Every 1 ms physics step, an onboard flight controller (velocity PI → attitude P → body-rate PI → mixer) turns `/cmd_vel` into rotor speeds. A motor and aerodynamic model (thrust k_f·ω², reaction torque, rotor and airframe drag, wind with gusts, ground effect) then applies force and torque to `base_link`, and Gazebo integrates the rigid body with collisions.
- Publishes `/odom`, TF `odom→base_link`, `/joint_states` (spinning props) and `/drone/status`. Arm with `/drone/arm`. Details in `docs/FLIGHT_DYNAMICS.md`.
- The physics and controller live in a ROS-free library (`interceptor_drone_flight`) with unit tests. PX4 SITL can still replace the onboard controller later behind the same `/cmd_vel` + `/odom` contract.

---

## 5. Phases (fastest path)

Estimates assume one developer. The dependency graph below shows what can run in parallel.

### Phase 0 – Build, fly, and decouple (blocking, ~3–4 days)
- **0.1** Get `colcon build` green in the container. Fix compile and link errors (ximgproc availability, include paths). Add `scripts/check.sh` that runs build and tests.
- **0.2** ✅ Interface changes from §3. Move all launch parameters into `config/*.yaml`, load them from launch files, and delete keys that aren't used. Add launch arg `mission_mode:=follow|intercept` and new files `follow_params.yaml`, `intercept_params.yaml`, `safety_params.yaml`.
- **0.3** ✅ Flyable drone + TF + `/odom`: done with the `quadrotor_dynamics` plugin and `drone_teleop_keyboard.py`. `hector_interface_node` deleted.
- **0.4** ✅ `groundtruth_target_node`: target poses from Gazebo (`/get_entity_state` for the `target_person` actor and `target_drone`) with configurable Gaussian noise and dropout. It publishes `TargetDetection` on `/target/detection_3d`. Selected by `perception_source`.
- **0.5** Re-enable gtest in CMake with one smoke test per library.

**Exit:** `make build-ws && make test` passes. With `teleop_twist_keyboard` publishing to `/cmd_vel`, the drone flies in Gazebo and RViz shows its TF and odometry.

### Phase 1 – Tracker (~2–3 days, after 0.2)
- **1.1** IMM-EKF library (`include/estimation/`, `src/estimation/`), pure C++ with no ROS dependency, plus unit tests on synthetic trajectories.
- **1.2** ✅ `target_tracker_node`: multi-track, publishes `/tracks` and the selected `/target/state`, plus RViz markers. Groups single-detection messages into frames by stamp.

**Exit:** on synthetic CV, CA and turn trajectories, position RMSE < 0.3 m at σ_meas = 0.3 m. The track survives a 2 s dropout.

### Phase 2 – FOLLOW mode (first demo, ~5 days)
- **2.1** `trajectory_controller_node` (shared).
- **2.2** `safety_filter_node` (keep-out CBF for tracks and static obstacles, covariance inflation, limits, e-stop), with unit tests for the projection.
- **2.3** `follow_planner_node` (presets, heading smoothing, lost-track handling).
- **2.4** World scenarios: walking person (1.4 m/s), jogger (4 m/s) with turns, occlusion behind trees, plus a second bystander actor. Add a `metrics_recorder` script that computes min distance (to every person, vehicle and obstacle), framing error and track retention from a rosbag.
- **2.5** Vision in the loop: `perception_source:=vision`, add COCO class 1 (`bicycle`), run YOLO on CUDA.

**Exit (100 randomized runs, ground truth):** minimum distance to any person or vehicle ≥ `d_min` and to any obstacle ≥ `d_obstacle_min` in **100 %** of runs, including runs where the planner is deliberately commanded straight at them. Target within ±15° of camera boresight ≥ 95 % of the time. Track reacquired after a 2 s occlusion. With vision: ≥ 90 % of that framing score.

### Phase 3 – INTERCEPT mode (~5–7 days, after Phase 1, 2.1, 2.2)
- **3.1** Target drone: a small quadrotor model in the world (`target_drone`) and `target_drone_behavior_node` with `hover`, `waypoints`, `straight` (CV) and `weave` behaviours. It moves via `/set_entity_state` like the bridge.
- **3.2** UAS detection. COCO has no drone class, so auto-label sim images: project the target drone's ground-truth bounding box into the left camera, generate a YOLO dataset, fine-tune `yolov8n` with class `uav`, and export to ONNX/TensorRT. Until this is done, use `perception_source:=groundtruth`.
- **3.3** `intercept_guidance_node` (phases, APN, capture logic, `GuidanceCommand` telemetry).
- **3.4** Engagement gate and abort logic in `safety_filter_node`, with tests. Include a test that a `person` track, however it's configured, can never be engaged.
- **3.5** Scenarios and metrics: hover target, 10 m/s straight target, weaving target, plus a "bystander" scenario where a person is placed near the target's path.

**Exit (ground truth):** capture envelope reached in ≥ 90 % of hover, ≥ 80 % of straight and ≥ 60 % of weave runs. **Zero** engagements of non-`uav` tracks. Abort fires in 100 % of bystander runs.

### Phase 4 – Integration & docs (~2 days)
- `follow.launch.py`, `intercept.launch.py`, a mode-switch service demo, RViz config with tracks, keep-out circles and setpoints, and README/CLAUDE.md updates.

### Dependency graph

```
0.1 ─► 0.2 ─┬─► 0.3 ─┬─► 2.1 ─► 2.2 ─► 2.3 ─► 2.4 ─► 2.5
            ├─► 0.4 ─┘                 │
            ├─► 1.1 ─► 1.2 ────────────┤
            └─► 0.5                    └─► 3.3 ─► 3.4 ─► 3.5
                         0.3 ─► 3.1 ─► 3.2 (parallel with Phase 2)
```

Parallel agents after 0.2: {0.3, 0.4, 1.1, 0.5}. After 0.3: {2.1, 3.1}. Wall clock with 3–4 parallel agents: about **2–2.5 weeks**. One developer, sequential: about **4 weeks**.

---

## 6. Default parameters

```yaml
# safety_params.yaml
safety:
  d_min_horizontal: 5.0        # m, to ANY person/vehicle track
  h_min_above: 3.0             # m, vertical clearance above target
  d_obstacle_min: 3.0          # m, to the surface of any static obstacle
  uncertainty_gain: 2.0        # k in d_eff = d_min + k * sigma_pos
  obstacles:                   # generated from the world file
    - {name: oak_tree_1, x: 0.0, y: 0.0, radius: 0.6}   # example entry; real values come from the world file
  cbf_alpha: 1.0               # 1/s
  altitude_floor: 2.0          # m AGL
  altitude_ceiling: 120.0      # m AGL
  geofence: {x_min: -100, x_max: 100, y_min: -100, y_max: 100}
  setpoint_timeout: 0.3        # s
  odom_timeout: 0.3            # s
  speed_cap: {follow: 15.0, intercept: 30.0}

# follow_params.yaml
follow:
  preset: BEHIND               # BEHIND | SIDE | LEAD | ORBIT
  offset: {behind: 8.0, side: 0.0, height: 5.0}
  orbit_rate: 0.2              # rad/s
  heading_tau: 0.8             # s, heading low-pass
  kp_pos: 0.8
  lost_hold_time: 2.0          # s -> HOLD
  lost_rtl_time: 10.0          # s -> RTL
  eligible_classes: [person, bicycle, car]

# intercept_params.yaml
intercept:
  nav_constant: 4.0
  max_accel: 15.0
  capture_start_range: 10.0
  capture_radius: 1.5
  capture_max_rel_speed: 3.0
  min_engage_altitude_agl: 10.0
  min_confirmed_frames: 10
  abort_ground_radius: 15.0
  eligible_classes: [uav]
```

---

## 7. Risks

| Risk | Mitigation |
|---|---|
| Workspace may not compile (never built) | Phase 0.1 first; every agent task must end with a green `make build-ws` |
| `opencv_ximgproc` missing from the image | Install `libopencv-contrib-dev`, or fall back to SGBM without WLS behind a CMake option |
| CPU SGBM and CPU YOLO too slow for vision-in-loop | Ground-truth source for control work. YOLO `device: cuda`. Lower stereo resolution or use `cv::cuda::StereoSGM` if built with CUDA |
| No drone class in COCO | Auto-labeled sim dataset (3.2); ground-truth source meanwhile |
| Simplified flight dynamics (no inflow or vortex-ring effects, uniform wind) | Fine for guidance and safety logic. PX4 SITL is a drop-in later behind the same `/cmd_vel` + `/odom` contract |
| Gazebo actors have no collision and a scripted path | Fine for FOLLOW. The ground-truth node reads actor pose via entity state |
| Safety distance eroded by tracking error or latency | Inflate `d_min` with track covariance, keep stale tracks as keep-out for `coast_timeout`, and test the filter against delayed and noisy tracks. Never fix a violation by lowering a threshold |
| Obstacle list drifts from the world file | Generate `obstacles:` from the `.world` file with a script and add a test that compares the two |
