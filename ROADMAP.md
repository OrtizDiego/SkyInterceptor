# Roadmap

The single source of truth for what is **done** and what is **still to do** in SkyInterceptor. It tracks the tasks of [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (§5); the plan says *how* each task works, this file says *where it stands*.

Any agent or person can read and edit this file: Claude Code, Codex, Cursor, a script or a human. If anything else (the progress board, a chat, a PR description) disagrees with it, this file wins.

## How to use it

- **Pick a task:** run `python3 scripts/roadmap.py ready` (tasks with status `todo` whose dependencies are all `done`), or read the list below. Tasks without a dependency between them can run in parallel on separate branches.
- **Update it in the same PR as the work.** A status describes the state *after your PR merges*, because `main` only ever shows merged work.
  - Finished the whole task: `Status: done`, `PR:` your PR number, `Branch:` your branch, and in `Notes` what you verified and what is still open.
  - Finished part of it: `Status: in_progress`, and say in `Notes` what remains.
  - Can't continue: `Status: blocked`, and say what blocks it in `Notes`.
  - Push the PR number as a follow-up commit once the PR exists.
- Don't mark a task `done` if its exit criteria or tests are not met. If a PR is closed without merging, whoever notices reverts its roadmap edit.
- Keep the layout below intact: one `###` section per task, fields as `- **Name:** value`, a continuation line indented by two spaces. `python3 scripts/roadmap.py check` validates it (CI runs it).
- To add a task, copy a section, give it a new code and add it to the right phase. Update `IMPLEMENTATION_PLAN.md` too if it changes the architecture.

**Status values:** `todo`, `in_progress` (partly merged, see Notes), `done` (merged to `main`), `blocked`. A branch may also use `review` while its PR is open.

**Fields:** `Status`, `Depends on` (task codes, comma-separated, or `—`), `PR` (number or `—`), `Branch` (or `—`), `Scope` (what the task delivers), `Notes` (verification, open items, obstacles).

## Phase 0 – Build, fly, and decouple

**Exit:** make build-ws && make test passes; the drone flies in Gazebo with keyboard teleop and RViz shows its TF and odometry.

### P0.1 – Make the workspace build

- **Status:** done
- **Depends on:** —
- **PR:** 23
- **Branch:** feat/p0-1-build-and-check
- **Scope:** Get colcon build green in the container, fix warnings, add scripts/check.sh and a make check target.
- **Notes:** Build is green: CI builds with -Werror and runs colcon test on every PR (PR #3, #4). `scripts/check.sh` and `make check` are the local mirror of CI: the static checks, then a `-Werror` build and the full test suite, run in a scratch workspace inside the container so the check tests the tree it is pointed at and leaves the shared build volumes alone. Verified: 392 tests, 0 failures, all linters green.

### P0.2 – Interfaces, parameters, mission mode

- **Status:** done
- **Depends on:** P0.1
- **PR:** 14
- **Branch:** feat/p0.2-interfaces-params-mission-mode
- **Scope:** Message/service changes from plan §3, YAML-driven launch files, mission_mode and perception_source args, mission_manager_node.
- **Notes:** mission_manager_node serves /mission/set_mode and latches /mission/mode (transient local). Starts disarmed; only INTERCEPT can be armed.

### P0.3 – Flyable simulated drone

- **Status:** done
- **Depends on:** P0.2
- **PR:** 7
- **Branch:** feat/busy-einstein-k0ao5o
- **Scope:** Drone that flies from /cmd_vel and publishes /odom and TF.
- **Notes:** Done with the quadrotor_dynamics Gazebo plugin (rotor/aero/wind model + onboard flight controller) and drone_teleop_keyboard.py instead of the planned kinematic bridge. See docs/FLIGHT_DYNAMICS.md.

### P0.4 – Ground-truth target source

- **Status:** done
- **Depends on:** P0.2
- **PR:** 17
- **Branch:** feat/p0-4-ground-truth-target-c4a66a
- **Scope:** groundtruth_target_node: Gazebo entity poses with noise, dropout and tree occlusion → /target/detection_3d, selected by perception_source.
- **Notes:** One /get_entity_state call per entity, each detection stamped with its own response. Only target_person is polled today.

### P0.5 – Test infrastructure

- **Status:** done
- **Depends on:** P0.2
- **PR:** 23
- **Branch:** feat/p0-1-build-and-check
- **Scope:** Re-enable gtest; one test file per library area with at least a smoke test.
- **Notes:** gtest is enabled for every library area: test_math_utils, test_mission, test_groundtruth_sensor, test_flight_dynamics, test_imm_filter, test_track_manager, test_frame_assembler, test_target_selector, test_trajectory_controller, test_guidance, test_safety_filter. 392 tests, 0 failures.

## Phase 1 – Tracker

**Exit:** On synthetic CV, CA and turn trajectories, position RMSE < 0.3 m at 0.3 m measurement noise. A track survives a 2 s dropout.

### P1.1 – IMM-EKF library

- **Status:** done
- **Depends on:** P0.2
- **PR:** 16
- **Branch:** feat/p1.1-imm-ekf-library
- **Scope:** ROS-free CV/CA/CT models, IMM filter and multi-target TrackManager with gtests on synthetic trajectories.
- **Notes:** RMSE 0.17–0.27 m on all synthetic trajectories at 0.3 m noise; 2 s dropout and crossing targets covered. docs/TRACKER.md.

### P1.2 – target_tracker_node

- **Status:** done
- **Depends on:** P1.1
- **PR:** 18
- **Branch:** feat/target-tracker-node-frames-178302
- **Scope:** Node on top of the P1.1 library: /tracks, selected /target/state, /tracks/markers; frames grouped by stamp; mode-aware target selection.
- **Notes:** FrameAssembler + TargetSelector, TargetState.confirmed added. 266 tests pass; checked live with a fake 3-target source. Not yet run in Gazebo with groundtruth_target_node (plan's RViz check). When target_drone joins (P3.1), groundtruth stamps per entity may be > 5 ms apart and split frames: raise frame_grouping.stamp_tolerance or stamp a poll cycle once.

## Phase 2 – FOLLOW mode

**Exit:** 100 randomized ground-truth runs: minimum distance to every person or vehicle >= d_min and to every obstacle >= d_obstacle_min in 100 % of runs; target within ±15° of boresight >= 95 % of the time; track reacquired after a 2 s occlusion. With vision: >= 90 % of that framing score.

### P2.1 – Trajectory controller

- **Status:** done
- **Depends on:** P0.3
- **PR:** 19
- **Branch:** feat/p2-1-trajectory-controller
- **Scope:** trajectory_controller_node: /setpoint/safe + /odom → /cmd_vel; position PID → velocity, yaw P with rate limit, anti-windup, 0.3 s setpoint timeout, ROS-free class with gtests.
- **Notes:** Verified: colcon build -Werror + colcon test (all linters), 308 tests, 0 failures from a clean build in the container; 30 new gtests incl. closed loop on the 6-DOF model (holds point within 0.2 m); headless Gazebo run settles within 0.07 m and cmd_vel goes to zero on setpoint loss. Open: not yet run with the safety filter or planner (not built yet); optional velocity loop left out (plant already has one); plugin speed limits (8 m/s) are tighter than the controller limits.

### P2.2 – Safety filter

- **Status:** done
- **Depends on:** P2.1, P1.2
- **PR:** 21
- **Branch:** feat/p2-2-safety-filter
- **Scope:** safety_filter_node: keep-out CBF for every person/vehicle track and static obstacle, covariance inflation, speed/altitude/geofence limits, e-stop, INTERCEPT gate hook; randomized property test.
- **Notes:** ROS-free `safety::SafetyFilter` in `include/safety/` with 24 gtests; the node is the thin ROS wrapper (`/setpoint/raw` -> `/setpoint/safe`, `/mission/status`, `/safety/keepouts` markers). Verified: clean `colcon build` with `-Werror` and `colcon test` in the container, 345 tests, 0 failures, all ament linters green. The plan's barrier had to be made relative degree 2: it constrains the *commanded* velocity, but the distance depends on the *achieved* one and the autopilot's velocity loop lags (measured 0.34 s on the X500 flight model), so the relative degree 1 form overshoots the keep-out by up to 6 m against a 5 m standoff. The filter now takes the stricter of the plan's bound and a lag correction, and a randomized property test closes the loop through the lag to check the distance never drops below d_min. Still open: P3.4 adds the altitude and geofence conditions to the engagement gate and the abort logic; the gate here only checks the mode, the arming and the class of the selected target. Not yet run in Gazebo.

### P2.3 – Follow planner

- **Status:** done
- **Depends on:** P2.2
- **PR:** 24
- **Branch:** feat/p2-3-follow-planner
- **Scope:** follow_planner_node: BEHIND/SIDE/LEAD/ORBIT presets in the target heading frame, heading smoothing, lost-track HOLD then RTL.
- **Notes:** ROS-free `follow::FollowPlanner` in `include/follow/` with 21 gtests; the node is the thin ROS wrapper (`/target/state` + `/mission/mode` + `/odom` -> `/setpoint/raw`, `position_valid = false`, a velocity setpoint, the safety filter keeps `/setpoint/safe`). Verified: clean `colcon build` with `-Werror` and `colcon test` in the container, 426 tests, 0 failures, all ament linters green; the node was also run live in the container: it publishes the expected setpoint (3.6, 0, 0) m/s with `source = SOURCE_FOLLOW` for the BEHIND preset, and it falls silent as soon as the mode switches to INTERCEPT. Design decisions: the presets are built in the target's *filtered* heading frame (a low pass with `heading_tau`), but the commanded yaw is the geometric bearing to the target, so a target that turns slides the framing point instead of whipping the camera around; ORBIT starts where BEHIND does and walks the framing point around the target at `orbit_rate`; the velocity is capped twice (the vertical component at 5 m/s, the whole command at 15 m/s, the plan's cap and the safety filter's FOLLOW cap). The lost-track machine is COAST (keep flying the last command) for `lost_hold_time`, then HOLD (hover) until `lost_rtl_time`, then RTL to the position the drone held on the first usable frame of the follow session; without that home the drone keeps holding instead of guessing. The planner publishes nothing at all when the mode is not FOLLOW, so it can never fight the intercept guidance on `/setpoint/raw` (the safety filter brakes to hover on a stale setpoint). The operator's track id is enforced twice: the tracker selects it and the planner refuses any other track on `/target/state`. The drone starts disarmed and the mission manager only arms an INTERCEPT mission, so the planner arms the motors itself (`arm_on_start`, on by default) once it has the odometry and the mode is FOLLOW; without this the setpoints would command a drone that never takes off. Still open: not yet run in Gazebo; the framing score and the minimum-distance score of the phase's exit criteria are P2.4's job, and the vision loop is P2.5.

### P2.4 – Follow scenarios and metrics

- **Status:** todo
- **Depends on:** P2.3, P0.4
- **PR:** —
- **Branch:** —
- **Scope:** World scenarios (walker, jogger with turns, occlusion behind trees, bystander) and a metrics_recorder for min distance, framing error and track retention.
- **Notes:** —

### P2.5 – Vision in the loop for FOLLOW

- **Status:** in_progress
- **Depends on:** P2.4
- **PR:** 26
- **Branch:** feat/p2-5-vision-in-the-loop
- **Scope:** perception_source:=vision, add COCO bicycle, YOLO on CUDA; reach >= 90 % of the ground-truth framing score.
- **Notes:** The vision path works end to end: the detector runs (it died on the NumPy 2 / cv_bridge ABI mismatch, so the image is now decoded with NumPy), YOLO detects the walker at 0.6-0.9 confidence, the localizer turns the detections into 3D positions and the tracker confirms a track. Four things were broken and are fixed: (1) the Python cv_bridge is unusable with the image's NumPy 2, so target_detector.py decodes the sensor_msgs/Image with NumPy; the Dockerfile also pins numpy<2, so a rebuilt image gets the binding back. (2) The stereo pair was mirrored: the URDF put the camera called "left" at -baseline/2 in a FLU body frame, so it was physically the right one and the disparity was negative. SGBM (minDisparity 0) could not match it and the depth of everything was the sky. The two joint origins are swapped now. (3) SGBM with 128 disparities dropped the target's disparity completely; 64 gives it back (measured: 100 % of the pixels of the person's box valid, disparity 7.9 px at 8.4 m). (4) The depth processor's fx was 535 instead of the camera's 554.4. Measured in Gazebo with the person at 8 m: 2D detections at 14.7 Hz on the GPU (1 Hz on the CPU), 3D detections at 5.4 Hz, the track confirmed and continuous (4370 of 4397 track samples confirmed, i.e. 99 %) on the GPU and confirmed but often lost on the CPU, and the 3D position against the ground truth at a median 0.35 m horizontal error (80 % within 2 m). The GPU needs a torch build the host driver can use: torch 2.6.0+cu124 sees the RTX 3060, the torch 2.14.1+cu130 in the image does not (the driver is 555.97, i.e. CUDA 12.5), so the Dockerfile now pins torch==2.6.0/torchvision==0.21.0. Still open: the framing score itself, which needs the P2.4 metrics harness - the numbers above are the vision-vs-ground-truth 3D proxy, not the framing score; and the image has to be rebuilt for the torch and numpy pins to take effect.

## Phase 3 – INTERCEPT mode

**Exit:** Ground truth: capture envelope reached in >= 90 % of hover, >= 80 % of straight and >= 60 % of weave runs. Zero engagements of non-uav tracks. Abort fires in 100 % of bystander runs.

### P3.1 – Target drone and behaviours

- **Status:** done
- **Depends on:** P0.3
- **PR:** 25
- **Branch:** feat/p3-1-target-behaviours
- **Scope:** target_drone model in the world and target_drone_behavior_node (hover, waypoints, straight, weave).
- **Notes:** The Crazyflie 2.1 model of PR 22 is kept, and the behaviour is now a selectable set: hover, waypoints, straight, weave and the original accelerating and curving profile (curve). The behaviour logic is a ROS-free library (include/guidance/target_drone_behaviour.hpp) with 17 gtests; the node is a thin wrapper and the parameters are in config/target_drone_params.yaml. The behaviour is selected with the `behaviour` parameter and can be switched while the node runs (ros2 param set /target_drone_behavior_node behaviour weave); the launch argument target_behaviour picks it at startup, default curve so the demo is unchanged. Both properties of PR 22 are kept: the published position is the exact integral of the published velocity (the finite difference of two published positions is exactly the published velocity), and the published velocity is continuous (the straight run eases in and out of its speed with a raised cosine, the corners of the waypoint loop are rounded off with a circular fillet, the weave oscillates). Verified: clean colcon build with -Werror in the container, 419 tests, 0 failures, all ament linters green; the node loads config/target_drone_params.yaml (4 waypoints from the file, the behaviour switch works at runtime and an unknown name is rejected); flown headless in Gazebo, the weave, hover, straight and waypoint behaviours all move the entity (the waypoint loop flies the rectangle of the config). Still open: the capture rates of the plan's exit criteria are P3.5, and the capture itself is still detected and not flown (P3.3).

### P3.2 – UAS detector

- **Status:** todo
- **Depends on:** P3.1
- **PR:** —
- **Branch:** —
- **Scope:** Auto-labelled sim dataset from target_drone, fine-tune yolov8n with class uav, export to ONNX/TensorRT.
- **Notes:** —

### P3.3 – Intercept guidance

- **Status:** done
- **Depends on:** P1.2, P2.1, P2.2
- **PR:** 27
- **Branch:** feat/p3-3-intercept-guidance
- **Scope:** intercept_guidance_node: phases, APN, the terminal capture law, capture logic, GuidanceCommand telemetry.
- **Notes:** The law lives in a ROS-free library (pure PN and augmented PN with the (N/2) * a_T_perp term, validated by a numerical sweep). The node is now called intercept_guidance_node (the name the plan asks for): the source file, the executable, the launch files, the config comment and the docs were renamed; the topics and the parameters are unchanged, and the node name in the ROS graph changed with them.
  The capture is now **flown**, not detected. The old setpoint commanded, along the line of sight, the *measured* closing speed, a term with no integral action: in the steady state the interceptor matched the target's along-LOS speed and the range stopped shrinking (measured in the sweep: against a target flying away at a constant 2-6 m/s the range never changed at all over 40 s, so the old law only closed the gap in the demo because the intruder accelerated from a standstill). The new terminal capture law commands the *relative* velocity: the target's velocity as a feedforward, a range-governed closure along the line of sight (the braking curve of a pursuer, corrected by the measured closing speed) and the unchanged PN lateral term, which is the only lateral authority - dropping it is what the earlier "relative-velocity match in the capture phase" experiment got wrong.
  The law was validated by a numerical sweep of 19 scenarios (the target's speed 0-10 m/s, its manoeuvre: hover, straight, head-on, crossing, weave and the demo's accelerating intruder, the initial range 5-30 m, the geometry and the lag of the vehicle's velocity loop) against a model of the X500 with the autopilot's velocity PI. All 19 scenarios reach the capture envelope with a relative speed of 1.5-3.3 m/s (the limit is 5 m/s) and the range shrinks monotonically into it; the same sweep on a vehicle with a lag of 0.3-1.0 s still captures, at 4.1 m/s at worst. The sweep is a gtest (test/test_guidance.cpp) and the numbers are asserted, not printed.
  In the simulator the capture is confirmed: with the drones clear of the bystander's patrol square the node logs "Target captured at 0.80 m, relative speed 4.72 m/s" (the old law never confirmed one: it reached the envelope at 10-12 m/s and overflew). Still open: the engagement is slow and messy in the demo scenario, because the safety filter's horizontal keep-out from the pedestrian (which has no altitude exemption) fights the intercept whenever the intruder circles over the patrol square, and the interceptor can only close on the intruder at 2 m/s (it flies at 8 m/s, the intruder at 6 m/s). The bystander scenario and the abort logic are P3.4/P3.5.

### P3.4 – Engagement gate and abort

- **Status:** todo
- **Depends on:** P3.3
- **PR:** —
- **Branch:** —
- **Scope:** Engagement gate and abort logic in safety_filter_node with tests; a person track can never be engaged.
- **Notes:** —

### P3.5 – Intercept scenarios and metrics

- **Status:** todo
- **Depends on:** P3.4, P3.1
- **PR:** —
- **Branch:** —
- **Scope:** Hover, 10 m/s straight, weaving target and bystander scenarios with capture/abort metrics.
- **Notes:** —

## Phase 4 – Integration & docs

**Exit:** follow/intercept launch files, mode-switch demo, RViz config with tracks, keep-out circles and setpoints, README and CLAUDE.md updated.

### P4 – Integration and docs

- **Status:** in_progress
- **Depends on:** P2.5, P3.5
- **PR:** 28
- **Branch:** feat/p4-docs-and-rviz
- **Scope:** follow/intercept launch files, mode-switch demo, RViz config with tracks, keep-out circles and setpoints, README/CLAUDE.md updates.
- **Notes:** The documentation and the RViz config are done: README.md (the two modes, the architecture, the launch files and their arguments, the config files, the tests), CLAUDE.md (the node table with the follow planner, the intercept guidance, the safety filter and the target drone behaviours, the ROS-free libraries, the config table, `make check`) and docs/ (INTERCEPT.md for the rename, the terminal capture law and the safety filter in series, SAFETY.md for the engagement gate and the keep-out markers, a new FOLLOW.md for the presets, the heading smoothing, the lost-track machine and how to measure a session, plus "Done" notes in AGENT_PROMPTS.md). The RViz config shows the two robot models, the tracks (/tracks/markers), the keep-out circles (/safety/keepouts) and the guidance vectors (/guidance/vectors). Still open, and it is the rest of P4: the follow/intercept launch files and the mode-switch demo, which wait for P2.4 (the metrics harness), P3.4 (the engagement gate and the abort) and P3.5 (the scenario runner and the capture metrics). One gap found while writing the docs and not fixed here because it is a source change: the safety filter publishes no marker for the setpoint it hands to the controller (/setpoint/safe is a custom message, which RViz cannot draw), so the filtered setpoint is not visible in RViz; adding a small marker to safety_filter_node would close it.
