# Interceptor Drone System - Implementation Plan

## ROS2 Humble + Gazebo + Stereo Vision + Pursuit-Evasion

**Version:** 1.0  
**Last Updated:** 2026-02-05  
**Platform:** Simulation (Gazebo + Hector Quadrotor)

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Mathematical Foundations](#mathematical-foundations)
4. [Hardware Specifications](#hardware-specifications)
5. [Phase 1: Foundation & Simulation](#phase-1-foundation--simulation)
6. [Phase 2: Stereo Vision & Perception](#phase-2-stereo-vision--perception)
7. [Phase 3: State Estimation](#phase-3-state-estimation)
8. [Phase 4: Guidance System](#phase-4-guidance-system)
9. [Phase 5: Control System](#phase-5-control-system)
10. [Phase 6: Target Evasion](#phase-6-target-evasion)
11. [Phase 7: Integration & Testing](#phase-7-integration--testing)
12. [File Structure](#file-structure)
13. [Success Criteria](#success-criteria)

---

## Executive Summary

This project implements a sophisticated **active interceptor drone** using ROS2 Humble, Gazebo simulation, and stereo vision. The system employs **Proportional Navigation (PN) guidance** with **Augmented PN** for terminal collision precision, combined with an **Interacting Multiple Model (IMM) Extended Kalman Filter** for robust target tracking. The target actively employs evasion strategies including sprint, weave, and max-G turns, creating a realistic pursuit-evasion scenario.

### Key Features

- **Stereo Vision Depth Estimation**: Real-time depth from stereo pair (RTX 3060 GPU)
- **YOLOv8 Object Detection**: Person/car detection at 30+ FPS with TensorRT
- **IMM-EKF Tracking**: Multiple motion models for varying target dynamics
- **Augmented PN Guidance**: True PN with target acceleration feedforward
- **Terminal Collision Guidance**: Precision impact in final phase
- **Minimum-Time Trajectories**: Bang-bang control for intercept optimization
- **Active Evasion System**: Intelligent target that maneuvers to avoid capture

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         COMPLETE INTERCEPTOR SYSTEM                             │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│   INTERCEPTOR DRONE                      TARGET (EVADING)                       │
│   ┌──────────────────────┐              ┌──────────────────────┐               │
│   │  Stereo Vision       │              │  Evasion Controller  │               │
│   │  + YOLO Detection    │              │  - Threat detection  │               │
│   │  → 3D Position       │              │  - Evasive maneuvers │               │
│   └──────────┬───────────┘              └──────────┬───────────┘               │
│              │                                      │                           │
│              │ /target/detection                    │ /target/odometry          │
│              │ (drone sees target)                  │ (target sees drone)       │
│              ▼                                      ▼                           │
│   ┌──────────────────────┐              ┌──────────────────────┐               │
│   │  IMM-EKF Tracker     │◄─────────────│  Target state (for   │               │
│   │  - Estimate target   │   shared TF  │   evasion calc)      │               │
│   │    position/velocity │              │                      │               │
│   │  - Predict motion    │              │                      │               │
│   └──────────┬───────────┘              └──────────────────────┘               │
│              │                                                                  │
│              │ /target/state                                                    │
│              ▼                                                                  │
│   ┌──────────────────────┐                                                     │
│   │  PN Guidance (APN)   │◄────── Drone odometry                               │
│   │  - Collision course  │                                                     │
│   │  - Terminal guidance │                                                     │
│   └──────────┬───────────┘                                                     │
│              │                                                                  │
│              │ /guidance/command                                                │
│              ▼                                                                  │
│   ┌──────────────────────┐                                                     │
│   │  Trajectory Control  │                                                     │
│   │  - Min-time intercept│                                                     │
│   │  - Constraints: 150  │                                                     │
│   │    km/h, 200m alt    │                                                     │
│   └──────────┬───────────┘                                                     │
│              │                                                                  │
│              │ /cmd_vel                                                         │
│              ▼                                                                  │
│   ┌──────────────────────┐              ┌──────────────────────┐               │
│   │   HECTOR QUADROTOR   │              │   TARGET VEHICLE     │               │
│   │                      │   COLLISION  │   (Gazebo model)     │               │
│   │                      │◄────────────►│                      │               │
│   │                      │  (Impact!)   │                      │               │
│   └──────────────────────┘              └──────────────────────┘               │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## Mathematical Foundations

### 1. Stereo Depth Estimation

```cpp
// Depth from disparity: Z = (f * baseline) / d
// f: focal length in pixels
// baseline: distance between cameras (0.12m)
// d: disparity in pixels (x_left - x_right)

double computeDepth(int disparity, double focal_length, double baseline) {
    if (disparity <= 0) return -1;  // invalid
    return (focal_length * baseline) / disparity;
}
```

### 2. IMM-EKF State Estimation

**State Vector (9D):**
```
x = [px, py, pz, vx, vy, vz, ax, ay, az]^T
```

**Models:**
- **CV (Constant Velocity)**: States [p, v], zero acceleration
- **CA (Constant Acceleration)**: States [p, v, a], full 9D
- **CT (Coordinated Turn)**: States [p, v, ω], turn rate model

**Model Probability Update:**
```
μ_k^j = (1/c) * Λ_k^j * Σ_i (p_ij * μ_k-1^i)
```
Where:
- μ_k^j = probability of model j at time k
- Λ_k^j = likelihood of measurement under model j
- p_ij = transition probability from model i to j

### 3. Augmented Proportional Navigation

**Line-of-Sight (LOS) Rate:**
```
ω = (r × v_rel) / |r|²

where:
  r = p_target - p_interceptor
  v_rel = v_target - v_interceptor
```

**True PN Command:**
```
a_c = N' · V_c · ω

where:
  N' = navigation constant (3-5)
  V_c = closing velocity = -(r · v_rel) / |r|
```

**Augmented PN (includes target acceleration):**
```
a_c = N' · V_c · ω + (N'/2) · a_target⊥

where a_target⊥ is target acceleration perpendicular to LOS
```

### 4. Time-to-Go and Miss Distance

```cpp
double t_go = |r| / V_c;

// Zero-effort miss (ZEM) - where will we be closest if no more accel?
Vector3d zem = r + v_rel * t_go;
double miss_distance = zem.norm();
```

### 5. Minimum-Time Trajectory

For bang-bang control with acceleration constraint `a_max`:

```
a(t) = +a_max  for 0 ≤ t < t_go/2
a(t) = -a_max  for t_go/2 ≤ t < t_go
```

This produces the fastest intercept trajectory.

---

## Hardware Specifications

### Interceptor Drone
| Parameter | Value |
|-----------|-------|
| Platform | Hector Quadrotor (Gazebo) |
| Max Velocity | 150 km/h (41.7 m/s) |
| Max Acceleration | 20 m/s² |
| Max Altitude | 200 m |
| Min Altitude | 2 m |
| Sensors | Stereo camera pair (12cm baseline) |
| Camera Resolution | 640x480 @ 60 FPS |
| Computing | NVIDIA RTX 3060 |

### Target Vehicle
| Parameter | Value |
|-----------|-------|
| Type | Car or Person |
| Max Velocity | 80 km/h (22.2 m/s) |
| Max Acceleration | 3 m/s² (car), 2 m/s² (person) |
| Motion | Ground-constrained (z=0) |
| Behavior | Evasive (sprint, weave, max-G turn) |

---

## Phase 1: Foundation & Simulation

### Task 1.1: ROS2 Workspace Setup

**Duration:** 2-3 days  
**Dependencies:** None

**Subtasks:**

1. **Create package structure**
   ```bash
   mkdir -p ~/interceptor_ws/src
   cd ~/interceptor_ws/src
   
   ros2 pkg create interceptor_drone \
     --build-type ament_cmake \
     --dependencies rclcpp rclcpp_components std_msgs geometry_msgs \
     sensor_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport \
     OpenCV Eigen3
   
   ros2 pkg create interceptor_interfaces \
     --build-type ament_cmake \
     --dependencies std_msgs geometry_msgs rosidl_default_generators
   ```

2. **Install dependencies**
   - ROS2 Humble
   - OpenCV with CUDA support
   - Gazebo and Hector Quadrotor packages
   - TensorRT (for YOLO inference)
   - Eigen3

3. **Configure build system**
   - CMakeLists.txt for each package
   - package.xml with all dependencies
   - colcon build configuration

**Success Criteria:**
- [ ] `colcon build` completes with no errors
- [ ] All packages discoverable via `ros2 pkg list`
- [ ] Can run `ros2 run` on example nodes

---

### Task 1.2: Custom Message Definitions

**Duration:** 1-2 days  
**Dependencies:** Task 1.1

**Messages to Create:**

1. **TargetDetection.msg**
   - Header, detection_id, track_id
   - 2D bounding box (x, y, width, height)
   - Class ID, confidence
   - 3D position (camera frame and world frame)
   - Depth information

2. **TargetState.msg**
   - Header, track_id
   - Position, velocity, acceleration (with covariance)
   - Quality score, validity flag

3. **GuidanceCommand.msg**
   - Header, track_id
   - Acceleration command (3D)
   - Navigation parameters (N', V_c, t_go, miss_distance)
   - Guidance mode enum
   - Feasibility flag

4. **TargetTrajectory.msg**
   - Header, target_id
   - Waypoints and timestamps
   - Trajectory type enum

5. **SetInterceptMode.srv**
   - Mode request (AUTO, PN, APN, PURSUIT)
   - Success response

**Success Criteria:**
- [ ] All messages compile with rosidl
- [ ] Can echo and publish messages
- [ ] Services callable via ros2 service call

---

### Task 1.3: Hector Quadrotor with Stereo Camera

**Duration:** 2-3 days  
**Dependencies:** Task 1.1

**Subtasks:**

1. **Create custom URDF**
   - Import base hector_quadrotor
   - Add stereo camera mount link
   - Configure left and right camera frames (12cm baseline)
   - Add Gazebo camera plugins
   - Publish synchronized left/right images

2. **Camera calibration**
   - Intrinsic parameters (fx=535.4, fy=535.4, cx=320.5, cy=240.5)
   - Distortion coefficients (zero for simulation)
   - Stereo baseline configuration

3. **Test camera output**
   - Verify image topics publishing
   - Check camera_info messages
   - Visualize in RViz

**Success Criteria:**
- [ ] Left and right images publish at 60 FPS
- [ ] Camera calibration valid
- [ ] Images viewable in RViz

---

### Task 1.4: Gazebo World and Launch Files

**Duration:** 1-2 days  
**Dependencies:** Task 1.3

**Subtasks:**

1. **Create intercept scenario world**
   - Ground plane
   - Lighting
   - Spawn points for drone and target

2. **Create launch files**
   - simulation.launch.py (Gazebo + spawn drone)
   - perception.launch.py (stereo + detection)
   - guidance.launch.py (tracker + guidance)
   - interceptor_full.launch.py (everything)

3. **Configure RViz**
   - Robot model display
   - Camera images
   - Target detections and tracks
   - Trajectory visualization

**Success Criteria:**
- [ ] Single launch command starts simulation
- [ ] RViz shows drone and camera feeds
- [ ] Can teleop drone manually

---

## Phase 2: Stereo Vision & Perception

### Task 2.1: Stereo Synchronization

**Duration:** 2-3 days  
**Dependencies:** Task 1.3

**Subtasks:**

1. **Implement time synchronizer**
   - Subscribe to left/right images and camera_info
   - Use approximate time sync policy
   - Tolerance: 5ms

2. **Publish synchronized stereo pair**
   - Custom message with both images
   - Synchronized header timestamp

**Success Criteria:**
- [ ] Images synchronized within 5ms
- [ ] No dropped frames
- [ ] Latency < 10ms

---

### Task 2.2: GPU-Accelerated Depth Processing

**Duration:** 3-4 days  
**Dependencies:** Task 2.1

**Subtasks:**

1. **Rectification** (GPU)
   - Use cv::cuda::remap
   - Precompute undistortion maps

2. **Disparity calculation** (GPU)
   - Use cv::cuda::StereoBM or StereoSGBM
   - 128 disparities, 11x11 block size

3. **Depth conversion**
   - Z = (f * baseline) / disparity
   - Handle invalid disparities

4. **Filtering**
   - Bilateral filter for noise reduction
   - Hole filling (optional)

**Implementation:**
```cpp
// CUDA-accelerated pipeline
cv::cuda::GpuMat left_gpu, right_gpu;
cv::cuda::GpuMat disparity_gpu, depth_gpu;

// Rectify
cv::cuda::remap(left_gpu, left_rect, map1, map2, cv::INTER_LINEAR);

// Compute disparity
stereo_matcher_->compute(left_rect, right_rect, disparity_gpu);

// Convert to depth
cv::cuda::divide(fx_ * baseline_, disparity_gpu, depth_gpu);
```

**Success Criteria:**
- [ ] 60 FPS at 640x480 on RTX 3060
- [ ] Depth accuracy ± 0.05m at 5m
- [ ] Latency < 20ms end-to-end

---

### Task 2.3: YOLO Object Detection

**Duration:** 3-4 days  
**Dependencies:** Task 2.2

**Subtasks:**

1. **Export YOLOv8 to TensorRT**
   ```bash
   yolo export model=yolov8n.pt format=engine device=0 half=True
   ```

2. **Implement inference node**
   - Load TensorRT engine
   - Preprocess images (resize, normalize)
   - Run inference on GPU
   - Postprocess (NMS, bbox extraction)

3. **Filter target classes**
   - Person, car, truck
   - Confidence threshold: 0.5

4. **Publish detections**
   - TargetDetection message
   - Include 2D bbox

**Success Criteria:**
- [ ] 30+ FPS inference
- [ ] Person/car detection > 85% precision
- [ ] Bounding boxes accurate

---

### Task 2.4: 3D Target Localization

**Duration:** 2-3 days  
**Dependencies:** Task 2.2, Task 2.3

**Subtasks:**

1. **Extract depth at detection**
   - Calculate bbox center pixel
   - Get depth from aligned depth image
   - Handle invalid depths

2. **Back-project to 3D**
   ```cpp
   X = (u - cx) * Z / fx;
   Y = (v - cy) * Z / fy;
   ```

3. **Transform to world frame**
   - Use TF to lookup camera → map transform
   - Apply transformation
   - Handle transform failures gracefully

4. **Publish 3D detection**
   - Include both camera and world coordinates
   - Add uncertainty estimates

**Success Criteria:**
- [ ] 3D position accuracy ± 0.1m at 10m
- [ ] Successful transform to map frame
- [ ] Robust to missing depth

---

## Phase 3: State Estimation

### Task 3.1: EKF Models

**Duration:** 3-4 days  
**Dependencies:** Task 2.4

**Subtasks:**

1. **Constant Velocity (CV) Model**
   - State: [px, py, pz, vx, vy, vz]
   - Prediction: x_k = x_k-1 + v * dt
   - Simple, good for steady motion

2. **Constant Acceleration (CA) Model**
   - State: [px, py, pz, vx, vy, vz, ax, ay, az]
   - Prediction: full 9D with acceleration
   - Better for maneuvering targets

3. **Coordinated Turn (CT) Model**
   - State: [px, py, pz, vx, vy, vz, ω]
   - Models turning motion
   - Useful for evasive weaving

**Implementation:**
```cpp
void CAModel::predict(double dt) {
    Eigen::Matrix<double, 9, 9> F;
    F << 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
         0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
         0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
         0, 0, 0, 1, 0, 0, dt, 0, 0,
         0, 0, 0, 0, 1, 0, 0, dt, 0,
         0, 0, 0, 0, 0, 1, 0, 0, dt,
         0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q_;
}
```

---

### Task 3.2: IMM Filter Integration

**Duration:** 2-3 days  
**Dependencies:** Task 3.1

**Subtasks:**

1. **Model probability update**
   - Calculate likelihood for each model
   - Update probabilities using transition matrix
   - Mix state estimates

2. **Mode selection logic**
   - Blend state estimates weighted by probabilities
   - Output single best estimate

3. **Gating and validation**
   - Reject measurements > 3σ from prediction
   - Handle occlusion (prediction-only mode)

4. **Publish TargetState**
   - Position, velocity, acceleration
   - Covariance matrices
   - Quality metrics

**Success Criteria:**
- [ ] Smooth state estimates
- [ ] Handles target maneuvers
- [ ] 2-second occlusion tolerance
- [ ] 100 Hz update rate

---

## Phase 4: Guidance System

### Task 4.1: PN Guidance Implementation

**Duration:** 3-4 days  
**Dependencies:** Task 3.2

**Subtasks:**

1. **Calculate LOS geometry**
   - Relative position r
   - Relative velocity v_rel
   - LOS rate ω

2. **True PN command**
   ```cpp
   a_cmd = N_prime * closing_velocity * omega;
   ```

3. **Augmented PN**
   - Add target acceleration feedforward
   - Perpendicular to LOS component

4. **Saturate commands**
   - Max acceleration: 20 m/s²
   - Prevent command windup

---

### Task 4.2: Guidance Phase Management

**Duration:** 2-3 days  
**Dependencies:** Task 4.1

**Subtasks:**

1. **Phase detection**
   - FAR: range > 50m → Standard APN (N'=4)
   - MID: 10-50m → Enhanced APN (N'=5)
   - TERMINAL: < 10m → Pure Pursuit

2. **Mode switching**
   - Smooth transitions
   - Hysteresis to prevent oscillation

3. **Collision prediction**
   - Calculate time-to-go
   - Predict miss distance
   - Adjust for collision course

**Success Criteria:**
- [ ] Smooth phase transitions
- [ ] Miss distance < 0.5m at intercept
- [ ] Stable in terminal phase

---

### Task 4.3: Intercept Feasibility

**Duration:** 2 days  
**Dependencies:** Task 4.2

**Subtasks:**

1. **Calculate required acceleration**
   - Check if within vehicle limits

2. **Time-to-go estimation**
   - Account for target evasion

3. **Feasibility flag**
   - Mark intercepts as feasible/infeasible
   - Provide status messages

---

## Phase 5: Control System

### Task 5.1: Cascade PID Controller

**Duration:** 3-4 days  
**Dependencies:** Task 4.3

**Subtasks:**

1. **Position loop**
   - PID on position error
   - Generate velocity commands

2. **Velocity loop**
   - PID on velocity error
   - Generate acceleration commands

3. **Attitude generation**
   - Convert acceleration to roll/pitch
   - Yaw toward target

4. **Anti-windup**
   - Prevent integral windup
   - Smooth saturation

---

### Task 5.2: Minimum-Time Trajectory

**Duration:** 2-3 days  
**Dependencies:** Task 5.1

**Subtasks:**

1. **Bang-bang trajectory generation**
   - Accelerate at max for first half
   - Decelerate at max for second half

2. **Constraint enforcement**
   - Max velocity: 41.7 m/s
   - Altitude: 2-200m

3. **Publish to Hector**
   - Convert to Twist message
   - Map to Hector control inputs

---

### Task 5.3: Hector Interface

**Duration:** 2 days  
**Dependencies:** Task 5.2

**Subtasks:**

1. **Command conversion**
   - Twist → Hector velocity commands
   - Linear z → thrust (vertical velocity)

2. **Emergency stop**
   - Zero velocity command
   - Landing sequence

---

## Phase 6: Target Evasion System

### Task 6.1: Evasion Controller

**Duration:** 3-4 days  
**Dependencies:** Phase 1

**Subtasks:**

1. **Threat detection**
   - Calculate time-to-collision (TTC)
   - Detect when TTC < 5 seconds

2. **Evasion strategies**
   - **SPRINT**: Max acceleration away
   - **WEAVE**: Sinusoidal perpendicular motion
   - **MAX-G TURN**: Perpendicular to LOS
   - **RANDOM**: Unpredictable jinking

3. **Strategy selection**
   - Based on range and TTC
   - Smooth switching

**Implementation:**
```cpp
EvasionStrategy selectStrategy(double range, double ttc) {
    if (range > 50.0) return EvasionStrategy::SPRINT;
    else if (range > 20.0) return EvasionStrategy::WEAVE;
    else if (ttc < 2.0) return EvasionStrategy::MAX_G_TURN;
    else return EvasionStrategy::RANDOM;
}
```

---

### Task 6.2: Target Vehicle Model

**Duration:** 2-3 days  
**Dependencies:** Task 6.1

**Subtasks:**

1. **Create Gazebo model**
   - Car or person with velocity controller
   - Ground-constrained (z=0)

2. **Add contact sensors**
   - Detect collision with drone
   - Log impact velocity

3. **Respawn logic**
   - Reset after collision
   - Random initial position/velocity

---

## Phase 7: Integration & Testing

### Task 7.1: System Integration

**Duration:** 2-3 days  
**Dependencies:** All previous phases

**Subtasks:**

1. **Master launch file**
   - Start all nodes in correct order
   - Parameter loading

2. **Parameter configuration**
   - YAML files for all components
   - Tunable gains

3. **Visualization**
   - RViz config with all topics
   - RQT plots for debugging

---

### Task 7.2: Test Scenarios

**Duration:** 3-5 days  
**Dependencies:** Task 7.1

**Test Cases:**

1. **Static Target**
   - Target at fixed position
   - Expected: Collision within 10s

2. **Constant Velocity Target**
   - 20 m/s straight line
   - Expected: Interception with lead

3. **Evasive Target**
   - Active weave and sprint
   - Expected: 70%+ success rate

4. **Occlusion Test**
   - Temporary loss of sight
   - Expected: Reacquisition after 2s

5. **Multiple Runs**
   - 100 runs with statistics
   - Success rate, time-to-intercept, miss distance

---

## File Structure

```
interceptor_drone/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── perception/
│   │   ├── stereo_sync_node.hpp
│   │   ├── stereo_depth_processor.hpp
│   │   ├── yolo_detector.hpp
│   │   └── target_3d_localizer.hpp
│   ├── estimation/
│   │   ├── imm_ekf.hpp
│   │   ├── ekf_models.hpp
│   │   └── target_tracker_node.hpp
│   ├── guidance/
│   │   ├── pn_guidance.hpp
│   │   ├── guidance_controller_node.hpp
│   │   └── intercept_calculator.hpp
│   ├── control/
│   │   ├── cascade_controller.hpp
│   │   ├── trajectory_generator.hpp
│   │   └── hector_interface.hpp
│   └── evasion/
│       └── evasion_controller.hpp
├── src/
│   ├── perception/
│   ├── estimation/
│   ├── guidance/
│   ├── control/
│   └── evasion/
├── config/
│   ├── stereo_camera.yaml
│   ├── ekf_params.yaml
│   ├── guidance_params.yaml
│   └── controller_params.yaml
├── launch/
│   ├── simulation.launch.py
│   ├── perception.launch.py
│   ├── guidance.launch.py
│   ├── interceptor_full.launch.py
│   └── test_*.launch.py
├── urdf/
│   └── interceptor_quadrotor.urdf.xacro
├── worlds/
│   └── intercept_scenario.world
└── rviz/
    └── interceptor_config.rviz

interceptor_interfaces/
├── msg/
│   ├── TargetDetection.msg
│   ├── TargetState.msg
│   ├── GuidanceCommand.msg
│   └── TargetTrajectory.msg
├── srv/
│   └── SetInterceptMode.srv
├── CMakeLists.txt
└── package.xml
```

---

## Success Criteria

### Overall System
- [ ] **Collision Success Rate:** ≥ 70% against evading target
- [ ] **Average Time-to-Intercept:** < 30 seconds from detection
- [ ] **No False Collisions:** Drone only collides with intended target
- [ ] **Robustness:** Handles temporary occlusion, target acceleration changes

### Individual Components
| Component | Success Criteria |
|-----------|------------------|
| Stereo Depth | 60 FPS, ±0.1m accuracy at 10m |
| YOLO Detection | 30 FPS, 85% precision |
| EKF Tracker | Smooth estimates, handles 2s occlusion |
| Guidance | Stable commands, phase transitions smooth |
| Evasion | Increases intercept difficulty measurably |

### Performance Targets
| Metric | Target |
|--------|--------|
| End-to-end latency | < 200ms |
| CPU usage | < 70% |
| Detection range | 0.5m - 30m |
| Intercept velocity | Up to 150 km/h |

---

## Timeline Summary

| Phase | Duration | Weeks |
|-------|----------|-------|
| Phase 1: Foundation | 1-2 weeks | 1-2 |
| Phase 2: Perception | 2-3 weeks | 3-5 |
| Phase 3: Estimation | 1-2 weeks | 6-7 |
| Phase 4: Guidance | 1-2 weeks | 8-9 |
| Phase 5: Control | 1-2 weeks | 10-11 |
| Phase 6: Evasion | 1 week | 11-12 |
| Phase 7: Integration | 1-2 weeks | 12-14 |
| **Total** | **9-14 weeks** | |

---

## Next Steps

1. Exit plan mode and initialize workspace
2. Start with Phase 1.1 (ROS2 setup)
3. Proceed sequentially through phases
4. Test each component before integration
5. Document and tune parameters throughout

---

**Document Version:** 1.0  
**Created:** 2026-02-05  
**Status:** Ready for Implementation
