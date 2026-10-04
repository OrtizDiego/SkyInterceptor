# Target tracker (IMM-EKF)

The tracker turns noisy 3D position detections of people, vehicles and drones into tracks with a position, velocity, acceleration, heading and covariance. Each track runs an Interacting Multiple Model (IMM) filter over extended Kalman filters, so it can follow a target that walks straight, accelerates or turns without retuning.

```
/target/detection_3d ──► FrameAssembler: one message per object → frames by stamp
                           │
                           ▼
                         TrackManager.processFrame(stamp, detections)          (per sensor frame)
                           │ predict every track to the frame time (IMM mixing + EKF predict)
                           │ chi-square gate, same class only
                           │ global nearest neighbour: confirmed tracks first, then tentative
                           │ IMM update (EKF update per model, mode probabilities, combination)
                           │ spawn / confirm (3 of 5) / delete
                           ▼
                 TrackManager.tracks(t) ──► /tracks, /tracks/markers           (50 Hz, extrapolated)
                           │
                 TargetSelector (mode whitelist, operator track, closest) ──► /target/state
```

| Piece | File | ROS dependency |
|---|---|---|
| Motion models (CV, CA, CT) and the Kalman update | `include/estimation/ekf_models.hpp`, `src/estimation/ekf_models.cpp` | none (Eigen only) |
| IMM filter | `include/estimation/imm_filter.hpp`, `src/estimation/imm_filter.cpp` | none |
| Multi-target track management | `include/estimation/track_manager.hpp`, `src/estimation/track_manager.cpp` | none |
| Frame grouping | `include/estimation/frame_assembler.hpp`, `src/estimation/frame_assembler.cpp` | none |
| Target selection | `include/estimation/target_selector.hpp`, `src/estimation/target_selector.cpp` | none |
| Unit tests (17 + 20 + 9 + 12) | `test/test_imm_filter.cpp`, `test/test_track_manager.cpp`, `test/test_frame_assembler.cpp`, `test/test_target_selector.cpp`, `test/synthetic_trajectory.hpp` | gtest |
| ROS node | `src/estimation/target_tracker_node.cpp` | rclcpp |
| Parameters | `config/ekf_params.yaml`, `eligible_classes` in `follow_params.yaml` / `intercept_params.yaml` | |

The library builds into `libinterceptor_drone_estimation.so`; `target_tracker_node` is a thin wrapper around it (see [The node](#the-node)).

## Motion models

All models share one 10-state layout, so the IMM can mix them with weighted sums:

```
x = [px py pz | vx vy vz | ax ay az | omega]      ENU world frame, omega = horizontal turn rate
```

| Model | Uses | Padded | Used for |
|---|---|---|---|
| CV, constant velocity | p, v | a = 0, omega = 0 | everything (the "nothing happening" model) |
| CA, constant acceleration | p, v, a | omega = 0 | ground targets: speeding up, braking |
| CT, coordinated turn | p, v, omega | a = 0 (the turn acceleration is omega x v) | drones: horizontal turns, CV in z |

A model sets its unused components to zero with zero covariance after every step: that is its hypothesis (a CV target does not accelerate). CT is nonlinear in omega. The EKF uses its analytic Jacobian, with series expansions for small `omega * dt`, where CT turns into CV. The measurement is always the 3D position, so `H = [I3 0]` for every model. The update uses the Joseph form.

Process noise is continuous white noise, discretised exactly: white acceleration for CV and CT, white jerk for CA, and a random walk on omega.

## IMM

One filter cycle per sensor frame: mixing, model-conditioned prediction, then on a detection the model-conditioned update, the mode probability update (in the log domain) and the moment-matched combination. Between frames, `extrapolate()` predicts the combined estimate without touching the filter, so the node can publish at 50 Hz on top of a 30 Hz sensor.

Two details matter, and the tests show why:

- **Mixing with unequal state dimensions.** Mixing the padded zeros of CV into CT would pull CT's turn rate (and CA's acceleration) towards zero by `mu_CV|j` every cycle. At 30 Hz the CT turn rate estimate ended up at 0.16 rad/s on a 0.5 rad/s turn. When mixing into model j, a source model that lacks a component instead lends model j's own estimate of it (Granström, Willett and Bar-Shalom, *Systematic approach to IMM mixing for unequal dimension states*, IEEE TAES 2015).
- **Time-scaled Markov chain.** The transition matrix holds switching probabilities per `transition_interval` (1 s). For a step dt the stay probability becomes `p_ii^(dt / interval)` and the off-diagonals keep their ratios. A per-frame matrix of 0.95 / 0.05 at 30 Hz switches modes every few hundred ms, keeps the mode probabilities stuck near the stationary distribution and makes CA overshoot. With the time-scaled chain the probabilities go to ~0.95 during a maneuver and ~0.05 after, and the behaviour does not depend on the detection rate. A 2 s gap also mixes as much as 2 s of frames would.

Model sets are chosen per track class: `uav` gets the aerial set (CV + CT), everything else the ground set (CV + CA).

## Track management

| Step | Rule |
|---|---|
| Gate | Squared Mahalanobis distance of the detection from the combined prediction, `S = P_pos + R`, below the chi-square quantile (3 DOF) of `gate_probability` (0.99 → 11.34). Only same-class pairs: a track's class never changes, so a `person` track can never become a `uav` track |
| Associate | Global nearest neighbour (Hungarian algorithm) on the normalised distance `d^2 + log det S`. One extra "miss" column per track lets it stay unassigned; the assignment maximises the number of associations first, then minimises the total cost. Confirmed tracks are assigned first, tentative tracks get the detections that are left |
| Spawn | An unassociated detection starts a tentative track, unless it lies within the wider `spawn_gate_probability` (0.9999) gate of a same-class track that got no detection this frame. The 99 % gate rejects 1 % of true detections, and without this rule each of those would start a duplicate of the track |
| Confirm | 3 hits in the last 5 frames. A tentative track is deleted once that is out of reach |
| Coast | A track without detections keeps predicting. `is_valid` turns false `coast_timeout` (2 s) after its last update |
| Delete | Confirmed: `max_missed_frames` consecutive misses **and** longer than `coast_timeout` without an update, so an occlusion shorter than 2 s never deletes a track, even while other targets keep the frames coming. Any track: `trace(P_pos) > max_position_variance` (50 m², reached ~4 s into a coast). `prune(t)` applies this while no frames arrive |
| Heading | `atan2(vy, vx)` while the horizontal speed is above `min_heading_speed` (0.5 m/s) **and** above 3 standard deviations of its own estimate; otherwise the last heading is held. The 3-sigma test stops the velocity noise of a person standing still from making the heading jump |

Frames older than the last processed one are dropped. A frame is all detections taken at one time; the node groups `TargetDetection` messages by stamp (next section).

## Frame grouping

Perception publishes one `TargetDetection` per object, all with the stamp of the image they came from. The tracker must see a sensor frame in one `processFrame()` call: a detection delivered on its own is a frame in which every other track missed, and a second call at the same stamp makes them miss again. Two walkers fed one message at a time end up with half of their M-of-N window empty (quality < 0.7 instead of 1.0, see `test_frame_assembler`).

`FrameAssembler` collects the messages first:

| Rule | Detail |
|---|---|
| Same frame | Stamps within `frame_grouping.stamp_tolerance` (5 ms) of a pending frame join it. The frame keeps the stamp of its first detection |
| Complete | When a detection of a newer frame arrives (sources publish in stamp order), or `frame_grouping.timeout` (10 ms, on the node clock) after the frame's first detection arrived. The second rule closes the last frame before a gap and frames with a single detection |
| Order | Complete frames go to the tracker oldest first, also if their messages arrived out of order |
| Late | A detection for a frame that already went to the tracker, or an older one, is dropped and counted (throttled warning) |

The node checks for complete frames on every detection and on every 50 Hz tick, so a frame reaches the filter within `timeout` + 20 ms of its first message, usually sooner because the next frame closes it. Detections without a valid `position_world` are ignored.

## Target selection

`TargetSelector` picks the track the planners work on and the node publishes it on `/target/state`:

- Only tracks of the current mode's `eligible_classes` can be selected, whatever else is asked: `follow.eligible_classes` (person, bicycle, car) in FOLLOW, `intercept.eligible_classes` (uav) in INTERCEPT, nothing in HOLD or before `/mission/mode` arrives. INTERCEPT can never select a person.
- **Operator track** (`MissionMode.track_id >= 0`): that track if it exists, is eligible and confirmed, otherwise nothing. It never falls back to another track.
- **Automatic**: candidates are confirmed, valid, eligible tracks. The closest one to the drone (`/odom`, transformed into `world_frame`) is selected. It is kept until another candidate is closer by more than `target_selection.switch_margin` (3 m), so two people at a similar distance don't make the camera flip between them. Once the selected track coasts past `coast_timeout` the closest candidate takes over; with no candidate it stays selected, with `is_valid = false`, until it is deleted. Without odometry, the oldest track (lowest id) wins.

With no selection the node still publishes `/target/state`, with `track_id = -1`, `class_id = -1` and `is_valid = false`, so a planner can tell "no target" from "tracker down".

## The node

`target_tracker_node` reads `ekf_params.yaml` plus `follow_params.yaml` and `intercept_params.yaml` (for the class whitelists); `guidance.launch.py` starts it in both modes.

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/target/detection_3d` | `TargetDetection` | in | `position_world` in `world_frame` when `world_position_valid`; `R = max(measurement_noise_pos², depth_variance) · I` |
| `/mission/mode` | `MissionMode` | in | Latched (transient local): mode and operator track |
| `/odom` | `nav_msgs/Odometry` | in | Drone position for the closest-track selection |
| `/tracks` | `TargetStateArray` | out, 50 Hz | Every track, tentative (`confirmed = false`) or confirmed, extrapolated to the publish time |
| `/target/state` | `TargetState` | out, 50 Hz | The selected track, or `track_id = -1` |
| `/tracks/markers` | `visualization_msgs/MarkerArray` | out, 50 Hz | Sphere, velocity arrow (1 m per m/s) and label (`#id class`, speed) per track. Selected orange, confirmed green, tentative or lost grey. "Tracks" display in `rviz/interceptor_config.rviz` |

Each tick runs on the node clock (simulation time with `use_sim_time`): close complete frames, `prune()`, extrapolate all tracks to now, select, publish. If the clock jumps back (Gazebo reset), the node clears its tracks and starts over.

## Performance

Synthetic trajectories at 30 Hz with 0.3 m measurement noise per axis (the plan's Phase 1 exit criterion is a position RMSE below 0.3 m). Worst of 5 seeds, RMSE after the first second:

| Trajectory | Class | Position RMSE |
|---|---|---|
| Walker, 1.4 m/s straight | person | 0.17 m |
| Car accelerating at 2 m/s² from 1.4 m/s | car | 0.22 m |
| Jogger, 4 m/s with two 90° turns in 2 s | person | 0.21 m |
| Car cruising, accelerating at 3 m/s², braking at 4 m/s² | car | 0.20 m |
| Drone, 10 m/s with a 0.4 rad/s turn | uav | 0.19 m |
| Drone weaving at 8 m/s, ±0.8 rad/s every 3 s | uav | 0.27 m |

For comparison, the raw detections have a 3D error of 0.52 m RMS. On a steady 0.5 rad/s, 10 m/s turn the turn rate is unbiased (0.15 rad/s RMS) and the acceleration estimate is within 1.5 m/s² RMS of the true 5 m/s².

Robustness over 200 seeds each: a 2 s sensor dropout never breaks a track, there is never a duplicate or missing confirmed track, and crossing targets keep their ids, with one exception. Two walkers that pass through the same point at the same moment swap ids in ~6 % of runs: position-only detections with 0.3 m noise cannot tell them apart for ~0.8 s, and both tracks bend towards each other. At a 1 m closest approach (two people cannot be in the same place) there are no swaps, and 4 m/s joggers or 10 m/s drones don't swap even through the same point. Coasting through ambiguous frames was tried and made it worse; fixing this properly needs appearance cues from the detector.

## Tuning

The defaults (`ProcessNoise`, `defaultGroundImmParams()`, `defaultAerialImmParams()`, `TrackManagerParams`) match `config/ekf_params.yaml`:

- The CV model is stiff (`process_noise_acc` 0.3) and leaves maneuvers to CA and CT, as an IMM should. A more agile CV absorbs the maneuvers itself, the IMM never switches, and the velocity noise of a target standing still doubles.
- `process_noise_turn_rate` trades turn-rate accuracy against weave tracking: 0.05 gives 0.10 rad/s RMS turn rate but 0.31 m on the weave, 0.5 gives 0.20 rad/s and 0.24 m. 0.2 sits in between.
- The aerial set starts new tracks with a 10 m/s velocity prior (ground: 5 m/s). With 5 m/s a 10 m/s drone often slips out of the gate of its own tentative track in the first frames.
