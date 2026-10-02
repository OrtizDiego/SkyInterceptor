# Flight dynamics and teleoperation

The simulated drone flies on physics: the rotor speeds produce thrust and torques, aerodynamic drag and wind act on the airframe, and Gazebo integrates the rigid body. An onboard flight controller (like PX4 on a real drone) turns velocity commands into rotor speeds, so you can fly it from the keyboard.

```
drone_teleop_keyboard.py ──/cmd_vel──►┐
                          ──/drone/arm─►│  quadrotor_dynamics plugin (Gazebo, 1 kHz)
                                        │   FlightController: velocity PI → attitude P → body-rate PI → mixer
                                        │        │ rotor speed commands
                                        │   QuadrotorModel: motor lag → thrust, reaction torque, rotor drag
                                        │        + airframe drag, wind and gusts (WindModel), ground effect
                                        │        │ force and torque on base_link
                                        └─► Gazebo ODE: gravity, inertia, collisions with the world
                                                 │
                                   /odom, TF odom→base_link, /joint_states (spinning props), /drone/status
```

| Piece | File | ROS / Gazebo dependency |
|---|---|---|
| Plant (rotors, aerodynamics) | `include/flight/quadrotor_model.hpp`, `src/flight/quadrotor_model.cpp` | none (Eigen only) |
| Wind and turbulence | `include/flight/wind_model.hpp`, `src/flight/wind_model.cpp` | none |
| Flight controller and mixer | `include/flight/flight_controller.hpp`, `src/flight/flight_controller.cpp` | none |
| Gazebo plugin | `src/simulation/quadrotor_dynamics_plugin.cpp` | Gazebo 11, gazebo_ros |
| Keyboard teleop node | `src/teleop/drone_teleop_keyboard.py` | rclpy |
| Unit tests (22) | `test/test_flight_dynamics.cpp` | gtest |

The three library parts build into `libinterceptor_drone_flight.so`. They have no ROS or Gazebo dependency, so the tests fly the drone in a standalone 6-DOF simulation.

## Flying it

```bash
make sim       # terminal 1: Gazebo with the park world, the walking person and the drone
make teleop    # terminal 2: keyboard teleop (needs an interactive terminal)
```

| Key | Action |
|---|---|
| `t` | Arm. The props idle until you climb |
| `w` / `s` | Climb / descend (thrust up / down). `w` while landed takes off |
| Arrow up / down (or `i` / `k`) | Forward / back |
| Arrow left / right (or `j` / `l`) | Left / right |
| `a` / `d` | Yaw left / right |
| Space or `h` | Stop: hover in place |
| `x` | Disarm. This also kills the motors in the air |
| `q` or Ctrl-C | Quit. The setpoint is zeroed, so the drone hovers |

Each key press changes the speed by one step (0.5 m/s horizontal, 0.25 m/s vertical, 0.2 rad/s yaw), and the speed is held until you change it. A terminal can't report key releases, so holding a key isn't a reliable way to fly. Forward, back, left and right are relative to the drone's heading. The status line shows the flight state (`DISARMED` / `LANDED` / `FLYING`), the setpoint, the altitude and the speed.

The plugin listens on the standard `/cmd_vel` (`geometry_msgs/Twist`: `linear.x` forward, `linear.y` left, `linear.z` up, `angular.z` yaw rate), so `teleop_twist_keyboard` or any other node works too. Arm with `ros2 topic pub --once /drone/arm std_msgs/msg/Bool "{data: true}"`. If no command arrives for 0.5 s, the drone holds position.

The drone spawns on its skids at (-3, -3), outside the person's 15 × 15 m patrol square and facing into it. Change this with `ros2 launch interceptor_drone simulation.launch.py x:=... y:=... yaw:=...`.

## Physics model

Frames follow REP-103: the world is ENU (z up) and the body frame is FLU (x forward, y left, z up). Rotor *i* sits at **r**ᵢ in the body frame (read from the prop joints of the URDF, ±0.174 m in x and y, 0.06 m up). It spins at ωᵢ in direction sᵢ (+1 CCW seen from above, −1 CW). The PX4 quad-X layout puts FR and RL CCW, FL and RR CW.

**Motors.** First-order lag towards the commanded speed, with τ_up = 12.5 ms and τ_down = 25 ms (props spin down slower than up). Speed is limited to 1000 rad/s.

**Thrust and reaction torque** of each rotor:

- Tᵢ = k_f · (ρ/ρ₀) · ωᵢ² · k_GE(hᵢ), along body +z
- Qᵢ = −sᵢ · k_m · Tᵢ, the drag torque of the propeller acting back on the body about z

k_f = 8.55·10⁻⁶ N/(rad/s)² and k_m = 0.016 m, the PX4 x500 SITL values. The 1.75 kg drone hovers at ω ≈ 708 rad/s (about 6800 RPM) with a thrust-to-weight ratio of about 2. Roll and pitch torque come from the thrust differences across the 0.25 m arms (**r**ᵢ × **F**ᵢ). Yaw comes from the reaction torques of the CW and CCW props.

**Fluid dynamics**

| Effect | Model | Default |
|---|---|---|
| Air density | Thrust and drag scale with ρ/ρ₀ | `air_density` 1.225 kg/m³ (sea level) |
| Airframe (parasitic) drag | **F** = −½ ρ (C_d A)ₖ \|vₖ\| vₖ per body axis, quadratic in the airspeed **v** = Rᵀ(**v**_drone − **v**_wind) | C_d·A = (0.04, 0.04, 0.08) m² |
| Rotor drag (blade flapping and induced drag) | **F**ᵢ = −\|ωᵢ\| · c_rd · **v**⊥ᵢ, linear in the in-plane airspeed at each hub (it includes rotation, **ω** × **r**ᵢ). Applied at the hub, so it also makes the drone weathercock | c_rd = 8.06·10⁻⁵ (PX4 x500) |
| Rotational damping | **τ** = −c_ω **ω** | c_ω = 0.002 N·m·s/rad |
| Ground effect | Cheeseman–Bennett, k_GE = 1 / (1 − (R / 4h)²), with h the rotor height above the ground plane. Capped at 4/3 below h = R/2 | R = 0.14 m (11" prop), ground at z = 0 |
| Wind | Uniform steady wind plus turbulence. Each gust component is a first-order Gauss–Markov process (a simplified Dryden model) with standard deviation σ and correlation time τ. The vertical gusts have half the intensity | off by default; τ = 2 s |

Drag is computed from the airspeed, not the ground speed. Hovering in a 6 m/s wind therefore gives the same forces as flying at 6 m/s through still air: the drone leans into the wind (about 6–7°) and the controller's integrator holds position.

Unmodelled: blade-element and momentum inflow effects (thrust loss with axial climb speed, translational lift), vortex ring state, rotor–rotor and rotor–body interaction, ground effect over obstacles (only the flat ground plane), and spatially varying wind.

## Flight controller

A PX4-style cascade runs every physics step (1 kHz):

1. **Velocity PI** (world frame) → desired acceleration. The horizontal command is limited to 8 m/s, the vertical to 3 m/s and the yaw rate to 1.5 rad/s. The integrators have anti-windup.
2. **Thrust vector** **F** = m(**a** + g**e**z), with a tilt limit of 35°. The desired attitude puts body z along **F** and body x towards the yaw setpoint, which integrates the yaw-rate command.
3. **Attitude P** on SO(3) (geometric error ½(R_dᵀR − RᵀR_d)^∨) → body-rate setpoint.
4. **Body-rate PI** → torque **τ** = J·α + **ω** × J**ω**.
5. **Mixer**: inverts the 4×4 allocation matrix [T, τx, τy, τz] = A·[T₁…T₄], then ωᵢ = √(Tᵢ / k_f). When the motors saturate, roll and pitch win over collective thrust, and yaw gets what's left.

Mass and inertia are read from the Gazebo model, so they always match the URDF. The camera moves the CoG about 1 cm forward, so the front rotors run a little faster than the rear ones in hover (about 731 vs 685 rad/s). The rate integrator takes care of that.

**Landed state.** After arming, the props idle (100 rad/s) until a climb is commanded (vz > 0.1 m/s). Touchdown is detected when the drone is commanded down or to hold, moves slower than 0.2 m/s and needs less than 75 % of its weight in thrust for 1 s. The rotors then return to idle.

## Wind

Set the wind at launch time:

```bash
ros2 launch interceptor_drone simulation.launch.py wind_x:=4.0 wind_y:=-2.0 wind_gust_stddev:=1.0
```

Or change the mean wind while the sim runs:

```bash
ros2 topic pub --once /drone/set_wind geometry_msgs/msg/Vector3 "{x: 6.0, y: 0.0, z: 0.0}"
```

`wind_x` blows towards +x (east) and `wind_y` towards +y (north). The actual wind, gusts included, is published on `/drone/wind`. In a 4/−2 m/s wind with σ = 1 m/s gusts, the drone holds hover within about 0.6 m horizontally and ±3 cm vertically.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in: velocity setpoint, heading frame (set `<command_frame>world</command_frame>` for the world frame) |
| `/drone/arm` | `std_msgs/Bool` | in: arm / disarm |
| `/drone/set_wind` | `geometry_msgs/Vector3` | in: mean wind [m/s], world frame |
| `/odom` | `nav_msgs/Odometry` | out, 100 Hz: ground-truth pose; twist in the body frame |
| TF `odom → base_link` | | out, 100 Hz |
| `/joint_states` | `sensor_msgs/JointState` | out, 30 Hz: prop angles and true rotor speeds [rad/s] |
| `/drone/status` | `std_msgs/String` (transient local) | out: `DISARMED`, `LANDED` or `FLYING` |
| `/drone/wind` | `geometry_msgs/Vector3Stamped` | out: current wind including gusts |

The rotor, aerodynamic, wind and controller-limit parameters are in the `<plugin>` block at the end of `urdf/interceptor_quadrotor.urdf.xacro`. The props in Gazebo spin at 1/10 of their true speed (`rotor_visual_slowdown`) so the rotation stays visible. `/joint_states` reports the true speeds.
