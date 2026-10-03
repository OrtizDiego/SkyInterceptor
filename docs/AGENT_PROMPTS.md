# Agent Prompts

One prompt per task in `IMPLEMENTATION_PLAN.md` §5. Paste the **common preamble** first, then the task prompt.
Each prompt is self-contained. Run them in the order of the dependency graph (plan §5). Tasks in the same wave can run in parallel on separate branches.

| Wave | Prompts (parallel within a wave) |
|---|---|
| 1 | P0.1 |
| 2 | P0.2 |
| 3 | P0.3, P0.4, P0.5, P1.1 |
| 4 | P1.2, P2.1, P3.1 |
| 5 | P2.2, P3.2 |
| 6 | P2.3 |
| 7 | P2.4, P3.3 |
| 8 | P2.5, P3.4 |
| 9 | P3.5 |
| 10 | P4 |

---

## Common preamble (paste before every task)

```
You are working on SkyInterceptor, a ROS 2 Humble (C++17 + one Python node) drone project that runs in Gazebo Classic inside a Docker container (see CLAUDE.md for make targets). Read CLAUDE.md and IMPLEMENTATION_PLAN.md (v2) before you start; the plan is the source of truth for architecture, topics, message definitions and default parameters.

The project has two mission modes:
- FOLLOW: an aerial filming drone that follows a person, bicycle or car while ALWAYS keeping at least d_min distance from every person or vehicle.
- INTERCEPT: capture of an intruding small drone (class "uav" only) by reaching a capture envelope at low relative speed. Ground targets must never be engaged.

Rules:
- Stay within the task scope below. Don't implement other plan tasks; stub interfaces you depend on only if they don't exist yet, and say so.
- Follow the repo code style: PascalCase classes, camelCase methods, snake_case variables/members, UPPER_CASE constants, namespace `interceptor`. Put algorithms in ROS-free library code (include/<area>/, src/<area>/) linked into interceptor_drone_lib, and keep nodes thin.
- Parameters come from config/*.yaml loaded by the launch files. Never hard-code tunables in launch files.
- Add gtest unit tests for every algorithm you write. `make build-ws && make test` must pass before you finish. If you can't run Docker, say so explicitly and at least make sure the code is self-consistent.
- Commit with clear messages on your feature branch and summarize what you did, what you verified, and anything left open.
```

---

## Phase 0

### P0.1 – Make the workspace build

```
Task: get `colcon build` for interceptor_interfaces and interceptor_drone green inside the container, and add a one-command check script.

1. Run `make up && make build-ws`. Fix every compile and link error. Known risks: stereo_depth_processor needs opencv_ximgproc (OpenCV contrib). If it's missing, add `libopencv-contrib-dev` to the Dockerfile, or add a CMake option INTERCEPTOR_USE_WLS that compiles the WLS step out when ximgproc isn't found. message_filters headers must be the `.h` variants on Humble.
2. Fix warnings under -Wall -Wextra -Wpedantic in our code, such as the unused parameter `lambda_`/`sigma_` shadowing and the unused `pt_detection_camera_` member in target_3d_localizer.
3. Add scripts/check.sh that runs inside the container: build with `--symlink-install`, then `colcon test` and `colcon test-result --verbose`, exiting non-zero on any failure. Add a `make check` target that calls it.
4. Update CLAUDE.md "Commands" if anything changed.

Done when `make check` passes from a clean `make clean`.
```

### P0.2 – Interfaces, parameters, mission mode

```
Task: implement the interface and parameter changes in IMPLEMENTATION_PLAN.md §3 "Interface changes" and §6.

1. interceptor_interfaces:
   - Add class constants to TargetDetection.msg (PERSON=0, CAR=1, TRUCK=2, BICYCLE=3, UAV=4).
   - Add class_id, class_name and heading to TargetState.msg.
   - Add TargetStateArray.msg, FlightSetpoint.msg and MissionStatus.msg.
   - Replace SetInterceptMode.srv with SetMissionMode.srv.
   Update target_detector.py to use the new class ids.
2. Config:
   - Create config/safety_params.yaml, follow_params.yaml and intercept_params.yaml with the defaults from plan §6.
   - Clean ekf_params.yaml and controller_params.yaml so that every key has a consumer or is marked "reserved for <task>".
   - Put every node's YAML under the `<node_name>: ros__parameters:` structure.
3. Launch:
   - Make every launch file load YAML via `parameters=[yaml_path, {'use_sim_time': ...}]` instead of inline dicts.
   - Add launch args `mission_mode` (follow|intercept, default follow) and `perception_source` (vision|groundtruth, default groundtruth) to interceptor_full.launch.py and pass them down.
   - Create empty-but-valid follow.launch.py and intercept.launch.py that include interceptor_full with the mode set.
4. Remove the now-unused static `Parameters` class (include/common/parameters.hpp, src/common/parameters.cpp) or reduce it to typed structs filled by each node. No global mutable statics.

Done when everything builds, `ros2 interface show interceptor_interfaces/msg/FlightSetpoint` works, and `ros2 launch interceptor_drone interceptor_full.launch.py mission_mode:=follow` starts every node without parameter errors. Stub nodes are fine.
```

### P0.3 – Flyable simulated drone

> **Done differently:** the drone flies on a force-based model (the `quadrotor_dynamics` Gazebo plugin with an onboard flight controller) instead of the kinematic bridge below, and `drone_teleop_keyboard.py` flies it. See `docs/FLIGHT_DYNAMICS.md`. Only the removal of `hector_interface_node` is left from this prompt.

```
Task: replace hector_interface_node with sim_drone_bridge_node so the drone can fly in Gazebo (plan §4.6).

- Subscribe to /cmd_vel (geometry_msgs/Twist: linear = world-frame velocity, angular.z = yaw rate).
- Integrate a kinematic point-mass model at 100 Hz with a first-order velocity response (tau param), acceleration limit and jerk limit. Clamp the altitude floor at 0.3 m.
- Push the pose to Gazebo each tick via the /set_entity_state service from the gazebo_ros_state plugin already in worlds/intercept_scenario.world. Use an async client and never block the timer.
- Publish nav_msgs/Odometry on /odom and TF odom→base_link. Make sure the TF tree map→odom→base_link→stereo camera optical frames is complete, so target_3d_localizer can transform detections into map.
- Command timeout of 0.3 s: decelerate to hover.
- Remove hector_interface_node from CMake and launch files.
- Put the dynamics in a ROS-free class with gtests (step response, limits, timeout).

Done when, with the sim running, `ros2 run teleop_twist_keyboard teleop_twist_keyboard` flies the drone in Gazebo, and RViz (fixed frame map) shows the drone and its camera frames moving.
```

### P0.4 – Ground-truth target source

```
Task: create groundtruth_target_node, which publishes TargetDetection messages from Gazebo ground truth so tracking and control can be developed without vision.

- Params: entity_names (list, e.g. ["target_person"]), class per entity, rate_hz (default 30), pos_noise_std (default 0.2 m), dropout_prob, and an optional occlusion flag that drops detections when a straight line from the drone to the target intersects a list of configured cylinders (tree positions from the world file).
- Get poses via /get_entity_state (gazebo_ros_state). Publish on /target/detection_3d with position_world filled, world_position_valid=true, frame map and a stamp at sim time.
- Launch it only when perception_source:=groundtruth. In that case don't launch the vision perception nodes.
- gtests for the noise, dropout and occlusion helpers.

Done when `ros2 topic echo /target/detection_3d` shows the walking actor's position at the configured rate, with noise.
```

### P0.5 – Test infrastructure

```
Task: re-enable gtest in interceptor_drone/CMakeLists.txt.

Create test/ with one test file per library area (common, estimation, control, guidance, safety), each holding at least one smoke test. Existing math_utils functions need real tests: skewSymmetric, quatToRot/rotToQuat round trip (including the trace ≤ 0 branches), and saturateVector. Add a helper macro or function so later tasks can register tests in one line. Make sure `make test` runs them and reports results.
```

---

## Phase 1 – Tracker

### P1.1 – IMM-EKF library

```
Task: implement the IMM-EKF as a ROS-free library (plan §4.1): include/estimation/{ekf_models.hpp, imm_filter.hpp, track_manager.hpp} and src/estimation/*.cpp.

- Models: CV (6D), CA (9D) and CT (7D: p, v_xy+v_z, omega; EKF Jacobian). Use a common internal state layout so mixing works (augment to the largest state with zero-padding and document the approach).
- IMM: mixing, per-model predict/update, likelihoods, mode probability update, combined estimate and covariance. The model set and transition matrix come from parameters: CV+CA for FOLLOW, CV+CT for INTERCEPT.
- Gating: chi-square 3-DOF threshold. Measurement is a 3D position with covariance R.
- TrackManager: nearest-neighbour association within gates, M-of-N confirmation (3/5), deletion after max_missed or when the trace of the position covariance exceeds a limit, predict-only coasting, and is_valid=false after coast_timeout (2 s). Tracks carry a class id and a heading from velocity (hold when speed < 0.5 m/s).
- gtests with synthetic trajectories: straight CV, constant acceleration, coordinated turn, a 2 s dropout, and two crossing targets. Assert position RMSE < 0.3 m at 0.3 m measurement noise, and no track swaps in the crossing case.
```

### P1.2 – target_tracker_node

```
Task: implement target_tracker_node on top of the P1.1 library.

- Subscribe to /target/detection_3d (TargetDetection; use position_world only when world_position_valid).
- Timer at 50 Hz: predict, then publish TargetStateArray on /tracks and the selected TargetState on /target/state.
- Target selection: filter by the eligible_classes of the current mission mode (from follow_params / intercept_params, switched through the /mission/set_mode service or a mode topic defined in P0.2). Choose the operator-requested track_id if set, else the closest confirmed eligible track, with hysteresis so the selection doesn't flip.
- Publish RViz MarkerArray on /tracks/markers (sphere + velocity arrow + text label: id, class, speed).

Done when, with perception_source:=groundtruth, /target/state follows the walking actor smoothly in RViz.
```

---

## Phase 2 – FOLLOW mode

### P2.1 – Trajectory controller

```
Task: implement trajectory_controller_node (plan §4.5).

- Input: /setpoint/safe (FlightSetpoint) and /odom. Output: /cmd_vel (Twist, world-frame linear velocity + yaw rate).
- Cascade: position PID → velocity command (plus velocity feed-forward from the setpoint), then an optional velocity loop. If only velocity is valid (position_valid=false), track velocity directly.
- Yaw: P controller on wrapped yaw error, with a rate limit.
- Anti-windup (clamp + back-calculation), output saturation, reset integrators on source/mode change.
- Setpoint timeout 0.3 s → zero velocity.
- Gains and limits come from controller_params.yaml.
- Put the controller in a ROS-free class with gtests (step response settles, no overshoot beyond X %, integrator bounded under saturation).

Done when publishing a fixed FlightSetpoint moves the simulated drone to that point and holds it within 0.2 m.
```

### P2.2 – Safety filter

```
Task: implement safety_filter_node (plan §4.3). It sits between the planners and the controller, and nothing may bypass it.

- Inputs: /setpoint/raw (FlightSetpoint), /tracks (TargetStateArray), /odom, /mission/estop (std_msgs/Bool, latching), and the mission mode.
- Output: /setpoint/safe (FlightSetpoint) and /mission/status (MissionStatus).
- Keep-out CBF: for EVERY track whose class is person, bicycle, car or truck (not only the selected target), with h = horizontal distance − d_min, constrain the velocity so n·(v − v_t) ≥ −alpha·h. Enforce multiple constraints with sequential projection, iterating until all are satisfied or 10 iterations pass; then fall back to the minimum-norm velocity that satisfies them, or to hover. Also enforce vertical clearance h_min_above inside 2·d_min. Convert position setpoints to velocity first (use the controller's kp) so the filter always works on velocity.
- Limits: mode-dependent speed cap, altitude floor and ceiling, geofence (velocity pointing outward is zeroed at the boundary), and stale odometry or setpoint → hover.
- INTERCEPT engagement gate (plan §4.4) is enforced here as well. Leave a clear hook; P3.4 fills in the logic.
- Put all math in a ROS-free SafetyFilter class. gtests must include:
  - a drone commanded straight at a standing person stops at ≥ d_min;
  - a person walking toward a hovering drone makes the drone back off;
  - two bystanders plus the target, with no constraint violated;
  - e-stop latches.
  Also add a randomized property test: 10,000 random states and commands, and after a simulated step the distance never goes below d_min − 0.05 m.
```

### P2.3 – Follow planner

```
Task: implement follow_planner_node (plan §4.2).

- Input: /target/state and /odom. Output: /setpoint/raw (FlightSetpoint, source FOLLOW).
- Presets BEHIND, SIDE, LEAD and ORBIT with offsets from follow_params.yaml, switchable at runtime via a parameter callback.
- Heading smoothing (first-order low-pass with wrap handling, tau param). v_des = v_t + kp·(p_des − p), capped. Yaw points at the target.
- Lost target: after lost_hold_time, output HOLD at the current position; after lost_rtl_time, output RTL (return to launch position at safe altitude).
- The planner does NOT enforce distance itself; safety_filter_node does. Still pick offsets ≥ d_min so the filter rarely has to intervene.
- ROS-free FollowPlanner class with gtests for each preset's geometry, heading wrap around ±π, and lost-target transitions.

Done when, in the park world, the drone follows the walking actor in the BEHIND preset while keeping it in view.
```

### P2.4 – Follow scenarios and metrics

```
Task: scenario coverage and a metrics pipeline for FOLLOW mode.

- World variants (or actor scripts selected by launch arg): walker 1.4 m/s on the footpath, jogger 4 m/s with sharp turns, a path that passes behind the tree line (occlusion), and a second "bystander" actor walking across the follow path.
- scripts/run_follow_trials.py: launches N randomized trials headless (gzserver only) with a randomized start offset, preset and actor speed. Records a rosbag per trial and computes:
  - minimum distance to every person;
  - fraction of time the target is within ±15° of the camera boresight;
  - track-loss events and reacquisition time;
  - safety-filter intervention time.
  Writes a CSV plus a summary.
- Acceptance thresholds from plan §5 Phase 2. The script exits non-zero when they aren't met.

Run 100 trials with perception_source:=groundtruth and report the summary.
```

### P2.5 – Vision in the loop for FOLLOW

```
Task: close the loop with real perception in FOLLOW mode.

- target_detector.py: add COCO class 1 (bicycle → BICYCLE), default device 'cuda' with a CPU fallback, and optionally an ONNX/TensorRT engine path.
- Make sure detection_3d stamps and frames are consistent with the depth image (stereo_sync now preserves the capture stamp) and that TF to map works with the P0.3 bridge.
- Measure and log the end-to-end latency, from image stamp to /setpoint/safe publish.
- Re-run the P2.4 trials with perception_source:=vision (N=30). Report framing score versus ground truth and the minimum-distance statistics. The safety filter must still hold d_min in 100 % of runs. If it doesn't, the fix goes in tracking and safety (e.g. inflating d_min with track covariance), not in relaxing the threshold.
```

---

## Phase 3 – INTERCEPT mode

### P3.1 – Target drone and behaviours

```
Task: add a simulated intruder drone for INTERCEPT mode.

- Add a small quadrotor visual model `target_drone` to the world (or spawn it from launch), with a static collision box.
- Repurpose evasion_controller_node as target_drone_behavior_node. It moves target_drone kinematically via /set_entity_state and supports behaviours hover, waypoints, straight (constant velocity) and weave (sinusoidal lateral), chosen by parameter, with speed and acceleration limits. It publishes its ground-truth odometry on /target_drone/odom for metrics only; guidance must never subscribe to it.
- Keep altitude ≥ 15 m by default.
- Add target_drone to groundtruth_target_node's entities with class UAV.
- Rename files, CMake targets and launch references; delete the old evasion node.
```

### P3.2 – UAS detector (dataset + fine-tune)

```
Task: give the vision pipeline a "uav" class.

- scripts/gen_uav_dataset.py: while target_drone_behavior_node flies random trajectories and the interceptor flies random offsets, record left-camera images. Project the target_drone ground-truth 3D bounding box (known model size) into the image using camera_info and TF, and write YOLO-format labels. Include negatives (no drone in view) and backgrounds with trees and sky. Aim for about 5k images.
- scripts/train_uav_detector.sh: fine-tune yolov8n (Ultralytics) on the dataset, report mAP50, and export ONNX.
- target_detector.py: load an optional second model (or a merged model) for the UAV class and publish class UAV=4. Keep COCO classes for FOLLOW mode. Select the models by mission mode.
- Don't commit datasets or weights to git. Add paths to .gitignore and document where they go.
```

### P3.3 – Intercept guidance

```
Task: implement intercept_guidance_node (plan §4.4), replacing guidance_controller_node.

- Input: /target/state (the tracker only selects class UAV in INTERCEPT mode) and /odom. Output: /setpoint/raw (FlightSetpoint, source INTERCEPT) and /guidance/telemetry (GuidanceCommand: N', V_c, t_go, zero-effort miss).
- Phase machine SEARCH → PURSUE → CAPTURE → DONE/ABORT, with hysteresis on range thresholds.
- PURSUE: augmented PN (textbook form, plan §4.4) with saturation at max_accel. Output velocity = current velocity + a·dt, with the acceleration as feed-forward.
- CAPTURE: relative-velocity matching so that range < capture_radius is reached with relative speed < capture_max_rel_speed. DONE once held for 1 s.
- Engagement is only allowed when the mission is armed (SetMissionMode). Otherwise output HOLD. The gate checks live in safety_filter_node (P3.4); this node must also refuse non-UAV targets as defence in depth.
- ROS-free InterceptGuidance class with gtests: a closing geometry drives the zero-effort miss down, saturation is respected, the phase transitions happen, and a non-UAV target gives HOLD.
- Delete guidance_controller_node and update CMake and launch files.
```

### P3.4 – Engagement gate and abort

```
Task: implement the INTERCEPT safety hooks in safety_filter_node (plan §4.4). This logic can't be bypassed by the planner.

- In INTERCEPT mode, pass INTERCEPT setpoints only if all of these hold:
  - the mission is armed;
  - the selected track's class is UAV and it has been confirmed for ≥ min_confirmed_frames;
  - target altitude AGL ≥ min_engage_altitude_agl;
  - the target is inside the geofence.
  Otherwise replace the setpoint with HOLD and set MissionStatus.reason.
- Abort (latching until re-armed) when any person/bicycle/car/truck track is within abort_ground_radius horizontally of the predicted capture point (target position + target velocity · t_go), when the target drops below the altitude floor, or when the track becomes invalid.
- The keep-out CBF from P2.2 stays active in INTERCEPT mode for all ground tracks.
- gtests: every gate condition individually; a person track with a spoofed high altitude still can't be engaged (the class check); the bystander abort; and that re-arming is required after an abort.
```

### P3.5 – Intercept scenarios and metrics

```
Task: scenario runner and metrics for INTERCEPT mode.

- scripts/run_intercept_trials.py (headless, randomized): target behaviours hover, straight at 10 m/s and weave, with randomized start geometry. Plus a bystander scenario where the target's path passes above a person actor.
- Metrics per trial: capture achieved (range < capture_radius with relative speed below the threshold, held for 1 s), time to capture, min range, peak commanded acceleration, abort events and reasons, and whether a non-UAV track was ever engaged (must be 0).
- Acceptance from plan §5 Phase 3. The script exits non-zero if it isn't met.

Run 100 trials per behaviour with perception_source:=groundtruth, then 30 with vision if P3.2 is done. Report results.
```

---

## Phase 4 – Integration

### P4 – Integration and docs

```
Task: final integration pass.

- follow.launch.py and intercept.launch.py are complete. The mode-switch demo uses the SetMissionMode service at runtime.
- RViz config: tracks with labels, keep-out circles (d_min) around every person/vehicle, the current setpoint versus the filtered setpoint, capture radius, and a MissionStatus text overlay.
- Update README.md and CLAUDE.md: the two modes, node table, topics, how to run the trial scripts, and the current metric results.
- Remove dead code and stale config keys, then run `make check` from a clean build.
```
