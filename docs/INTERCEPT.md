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

## From an acceleration to a velocity setpoint

The interceptor flies a velocity loop (the trajectory controller commands a velocity
and the onboard autopilot closes it), so the acceleration has to become a velocity
setpoint. `pnVelocitySetpoint()` splits it:

- along the LOS: the closing speed the interceptor sustains, capped at `max_speed`;
- perpendicular to the LOS: the guidance acceleration, scaled by
  `accel_to_velocity_gain` (the effective time constant of the velocity loop).

The guidance acceleration is the **only** lateral authority. Adding the target's
velocity to the setpoint instead would track the target directly and hide what the
guidance law contributes.

## The phases

`guidance_controller_node` runs the phase machine of plan §4.4:

| Phase | What it does |
|---|---|
| `SEARCH` | No valid target: climb to `cruise_altitude` and hold. |
| `PURSUE` | PN or APN on the line of sight. |
| `CAPTURE` | The same law, now inside `capture_start_range`. |
| `DONE` | Target captured. |

The engagement gate (plan §4.4) must hold before `PURSUE` starts: the track must be
a `uav`, confirmed and valid, above `min_engage_altitude_agl`, and the operator must
have armed the mission.

The capture is **detected, not flown**. Switching to a plain relative-velocity match
inside the capture radius loses the lateral correction, and the interceptor ends up
chasing the target's own turn instead of closing on it — the range then oscillates
around the capture start range instead of shrinking. The law therefore keeps running
to contact and the node reports the capture when it happens.

`capture_max_rel_speed` is 5 m/s rather than the 3 m/s of the plan: the X500 closes
on the target at several m/s, so 3 m/s is not reachable with this platform.

## The scenario

`target_drone_behavior_node` flies the intruder kinematically through
`/set_entity_state`. The profile is the point of the demo:

- the drone starts nearly stationary and accelerates to `final_speed` over
  `accel_time` (the tangential acceleration);
- it turns at a constant `turn_rate` the whole time (the lateral acceleration).

The guidance never reads this node: it only ever sees the tracker's estimate on
`/target/state`, so the whole perception and estimation chain stays in the loop.

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

