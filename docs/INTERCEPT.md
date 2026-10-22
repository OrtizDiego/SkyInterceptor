# INTERCEPT guidance (PN / augmented PN)

The INTERCEPT mode is a counter-UAS interception: the X500 has to reach a small
intruding drone and arrive inside the capture envelope with a low relative speed.
Ground targets are never eligible.

## The guidance law

The law lives in the ROS-free library `include/guidance/pn_guidance.hpp`, so it can be
unit tested on synthetic geometry.

**Pure PN** (Shneydor, *Missile Guidance and Pursuit*):

```
a_M = N * V_c * lambda_dot
```

`V_c` is the closing speed along the line of sight (positive while the range shrinks)
and `lambda_dot` is the LOS rate vector:

```
lambda_dot = (v_r - (u . v_r) u) / |r|
```

with `r = p_T - p_M`, `u = r / |r|` and `v_r = v_T - v_M`. The commanded acceleration
is therefore perpendicular to the LOS, and its job is to rotate the LOS onto the
target.

**Augmented PN** adds the target's acceleration, which the tracker estimates on
`/target/state`:

```
a_M = N * V_c * lambda_dot + (N / 2) * a_T_perp
```

The `(N/2)` factor is the textbook value (plan §4.4). `a_T_perp` is the target
acceleration projected perpendicular to the LOS: only the part of `a_T` that bends
the LOS away is worth compensating, the along-LOS part is already handled by the
closing-speed term of the pure PN law. A simulation sweep over random engagements
confirmed that `k = N/2` with the projection intercepts the large majority of the
cases where pure PN misses.

## The terminal capture law

PN is a pursuit law: it rotates the LOS onto the target but it never bleeds off the
closing speed, so the interceptor flies *through* the target. Before this law
existed the demo reached the capture envelope (1.5 m) at 10-12 m/s relative speed,
against a limit of 5 m/s: the capture was **detected, not flown**.

The law commands the *relative* velocity:

```
v_M = v_T + u * v_close(r) + a_pn * tau
```

- `v_T` is the target's velocity, a feedforward: the interceptor goes where the
  target goes, so what the law shapes is the relative velocity
  `v_r = v_T - v_M = -u * v_close(r) - a_pn * tau`. The interceptor flies *with* the
  target and closes on it instead of flying at it, and the relative speed at the
  capture is the closing speed, which the profile below governs.
- `u * v_close(r)` is the governed closure along the line of sight.
- `a_pn * tau` is the PN lateral term, scaled by `accel_to_velocity_gain` (the
  effective time constant of the velocity loop). It is the **only** lateral
  authority, and keeping it is what the earlier "relative-velocity match in the
  capture phase" experiment got wrong: dropping it loses the lateral correction and
  the range oscillates instead of shrinking.

### The closure profile

The commanded closing speed is the fastest speed from which the vehicle can still
slow down to `final_closing_speed` at the hold radius under a constant
deceleration `approach_decel`:

```
v_profile(r) = sqrt( v_final^2 + 2 * a * (r - r_hold) )   for r > r_hold
v_profile(r) = 0                                           for r <= r_hold
```

This is the braking curve of a pursuer: the deceleration it asks for is exactly `a`
everywhere, so it is a profile the vehicle can actually track, and the closing speed
at the hold radius is `v_final` by construction. The hold radius is *inside* the
capture radius on purpose: the range converges to it, so aiming at the envelope
boundary would approach the boundary asymptotically and never enter the envelope.

The profile on its own is not enough, because it is an open-loop schedule on the
range: the range closes at the speed the vehicle actually flies, so the profile
falls behind, the vehicle carries more speed into the envelope than the profile
asks for, and it arrives at the capture too fast. The measured closing speed is
therefore fed back against the profile:

```
v_close = v_profile - k * (V_c - v_profile)
```

The steady state is the profile itself, but on the way in the vehicle is slowed
down as soon as it is closing faster than the profile wants.

### Why the old along-LOS term was wrong

The setpoint used before this law commanded, along the LOS, the *measured* closing
speed `V_c`. That term has no integral action: in the steady state the
interceptor's along-LOS velocity converges to the target's, so the range stops
shrinking and the interceptor simply matches the target's speed. Measured in the
sweep: against a target flying away at a constant 2, 4 or 6 m/s the range never
changed at all (20 m at the start, 20 m after 40 s). The old law only closed the
gap in the demo because the intruder accelerated from a standstill.

### The sweep

The law was validated numerically against a model of the vehicle (the X500 flight
model with the autopilot's velocity PI, `include/flight/flight_controller.hpp`) over
the target's speed (0-10 m/s), its manoeuvre (hover, straight, head-on, crossing,
weave and the demo's accelerating intruder), the initial range (5-30 m), the
geometry and the lag of the vehicle's velocity loop. The sweep is a gtest
(`test/test_guidance.cpp`), and the results are asserted, not just printed:

| | |
|---|---|
| scenarios | 19 |
| captures (envelope reached, relative speed below 5 m/s) | 19/19 |
| the range at the capture | 1.50 m (the envelope) |
| relative speed at the capture | 1.5 - 3.3 m/s |
| the range shrinks monotonically into the envelope | 19/19 |
| the interceptor stays with the target after the capture | 19/19 |
| the same sweep on a vehicle with a lag of 0.3 - 1.0 s | the capture holds, 4.1 m/s at worst |

Without the closing-speed feedback the same sweep gives a relative speed at the
capture of 3.9 m/s and the interceptor overshoots the target by about a metre; with
it, the arrival is gentle and the interceptor stops overshooting.

The lag matters: the feedback goes through the lag of the vehicle's velocity loop,
so the gain has to stay well below 1. At a gain of 1 the loop rings on the real
airframe, and in the simulator the interceptor circles the target instead of
closing on it.

## The phases

`intercept_guidance_node` runs the phase machine of plan §4.4:

| Phase | What it does |
|---|---|
| `SEARCH` | No valid target: climb to `cruise_altitude` and hold. |
| `PURSUE` | The terminal capture law: PN or APN laterally, the governed closure along the line of sight. |
| `CAPTURE` | The same law, now inside `capture_start_range`: the capture envelope is armed. |
| `DONE` | Target captured. The law keeps running, so the interceptor holds the offset instead of flying through the target. |

The engagement gate (plan §4.4) must hold before `PURSUE` starts: the track must be
a `uav`, confirmed and valid, above `min_engage_altitude_agl`, and the operator must
have armed the mission.

The capture is **flown**: the terminal capture law above brings the interceptor
into the envelope with a relative speed of 1.5-2.4 m/s in the sweep, and the node
confirms the capture when the range and the relative speed are both inside their
limits at the same instant.

`capture_max_rel_speed` is 5 m/s rather than the 3 m/s of the plan: the X500 closes
on the target at several m/s, so 3 m/s is not reachable with this platform.

## The scenario

`target_drone_behavior_node` flies the intruder kinematically through
`/set_entity_state`. The profile is the point of the demo:

- the drone starts nearly stationary and accelerates to `final_speed` over
  `accel_time` (the tangential acceleration);
- it turns at a constant `turn_rate` the whole time (the lateral acceleration).

The position is the exact integral of that velocity, so the published velocity
really is d(position)/dt and the tracker's estimate of the target's velocity
and acceleration matches what the intruder does.

The intruder is moved by `/set_entity_state`, so no Gazebo plugin publishes its
pose. The behaviour node therefore broadcasts the `map -> target_base_link`
transform itself: without it `robot_state_publisher` has no root to hang the
propeller frames on and RViz reports the propeller transforms as missing.

The guidance never reads this node: it only ever sees the tracker's estimate on
`/target/state`, so the whole perception and estimation chain stays in the loop.

## The scenario

The interceptor spawns *behind* the intruder on the same line, both in the middle
of the park above the pedestrian's patrol square:

| | position | heading |
|---|---|---|
| interceptor | (2, 5) m | +x, towards the intruder |
| intruder | (10, 5) m | +x |

The line of sight therefore runs along the interceptor's nose from the start, so
the stereo cameras keep the intruder in frame through the approach. The
intruder still has to accelerate and curve, so the guidance law is what decides
the outcome: pure PN misses it by about 4 m, augmented PN reaches the capture
envelope.

Both spawn points are clear of the trees (the nearest canopy edge is 2.6 m away
at the intruder's start) and well above them (the tallest canopy is 6.5 m, the
drones fly at 12 m).

## The cameras

The stereo camera hangs off the front of the airframe, so the nose has to follow
the target for the cameras to see it. The guidance commands that heading: the
yaw setpoint is the bearing of the target in the world frame.

Getting the nose onto the target takes two things beyond the yaw setpoint:

- the trajectory controller's yaw gain (`kp_yaw`) and rate limit, and
- the flight controller's `max_yaw_error`, the window its internal heading
  setpoint may run ahead of the airframe. The mixer allocates the yaw torque
  last, so a tight limit starves the yaw exactly when the drone accelerates
  hardest and the nose lags the line of sight.

## Running it

```bash
# The demo scenario: the intruder accelerates and curves, the interceptor engages it
ros2 launch interceptor_drone intercept_demo.launch.py

# The same scenario with pure PN instead of augmented PN
ros2 launch interceptor_drone intercept_demo.launch.py guidance_law:=pn
```

Inside the container, source the workspace overlay first: the entrypoint does it
for the container's own shell, but a `docker exec` shell starts fresh and would
otherwise report `Package 'interceptor_drone' not found`.

```bash
docker exec -it interceptor bash
source /workspace/interceptor_ws/install/setup.bash
```

`intercept_demo.launch.py` arms the mission automatically (`auto_arm:=true`), so the
engagement starts on its own. In `interceptor_full.launch.py` the same argument
exists but defaults to `false`: the mission manager always starts disarmed, and
arming is an operator decision.

The Gazebo GUI starts on a raised three-quarter view of the middle of the park
(the world file's `<gui><camera>` block), so both drones, the pedestrian and the
camera feeds are in frame from the first second.

Telemetry:

| Topic | What it carries |
|---|---|
| `/guidance/command` | `GuidanceCommand`: the acceleration command, `V_c`, `t_go`, the zero-effort miss, the law and the phase. |
| `/guidance/vectors` | RViz markers: the line of sight and the commanded acceleration. |
| `/setpoint/raw`, `/setpoint/safe` | The velocity setpoint. |
| `/target_drone/odom`, `/target_drone/accel` | The intruder's true state, published by the behaviour node for reference only. The guidance never reads them: it only sees the tracker's estimate. |

Both setpoint topics carry the same message because `safety_filter_node` does not
exist yet (plan §2.2). When it lands it takes over `/setpoint/raw` and the guidance
node must stop publishing on `/setpoint/safe`.

To watch the engagement from the outside, independently of the guidance node's own
telemetry, run the recorder in a second shell:

```bash
ros2 run interceptor_drone record_intercept.py
```

It prints the range, the closing speed, the relative speed and the phase on every
sample.

### The first launch is slow

`gzserver` fetches the Gazebo model database from the internet the first time it
starts, and `/spawn_entity` only becomes available once the world has finished
loading. That can take well over a minute on a cold start, which is longer than
`spawn_entity`'s default 30 s timeout, and the drones then never appear. The
launch files pass `spawn_timeout` (180 s by default) for exactly this reason. If
the drones do not appear, check the Gazebo log for the model-database download
before suspecting anything else.

## The comparison

With the demo parameters (the intruder accelerates to 6 m/s over 4 s while turning
at 1.2 rad/s) the two laws differ in the closest approach:

| Law | Closest approach | |
|---|---|---|
| Pure PN | 4.0 m | misses: the LOS bends away faster than the law can correct it |
| Augmented PN | 1.1 m | captures |

The difference is that pure PN has no way to anticipate the turn: it only ever
corrects the LOS it sees, while augmented PN adds the target's own acceleration
estimate to the command.

The separation depends on the manoeuvre. With a gentle turn both laws intercept
(pure PN reaches 0.0 m), and with a very tight one the interceptor cannot keep up
with the target at all and neither law captures. The demo parameters sit in
between, where the guidance law is what decides the outcome.

Note that the comparison is sensitive to the tracker's velocity and acceleration
estimates: the guidance sees the *estimated* target state, not the truth, so a
laggy estimate eats into the margin that separates the two laws.

### What the demo does and does not achieve

In the simulator the interceptor reaches a closest approach of about 1.5 m and
enters the capture radius, but it does so at 10 m/s relative speed, so the capture
is *not* confirmed: the guidance is a pursuit law and it flies through the target
rather than arriving gently. The node says so explicitly ("overflying the
target") instead of claiming a capture it did not get.

Closing that gap needs a terminal law for the last few metres, which the plan
puts in the capture phase (§4.4). The version tried here — switching to a
relative-velocity match inside the capture radius — was measured to be worse: it
loses the lateral correction and the range then oscillates instead of shrinking.
That is why the capture is detected rather than flown.

