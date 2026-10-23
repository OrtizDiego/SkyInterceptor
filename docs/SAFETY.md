# The safety filter

The safety filter is the last word between the planners and the trajectory controller.
Whatever the planner commands, the drone never comes closer to a person, a vehicle or a
static obstacle than the configured standoff. Safety distance is the top-level rule: a
threshold is never lowered to make a command feasible.

The maths lives in the ROS-free library `include/safety/safety_filter.hpp`, so it can be
tested on synthetic geometry; `src/safety/safety_filter_node.cpp` is the thin ROS
wrapper around it.

```
planner ──► /setpoint/raw ──► safety_filter_node ──► /setpoint/safe ──► controller
                            ▲
              /tracks, /odom, /mission/mode, /mission/estop
```

## What it does

| | |
|---|---|
| **Keep-out** | Every person and vehicle track on `/tracks`, not only the selected one, and the static obstacles from `config/safety_params.yaml` |
| **Uncertainty** | The standoff of a track grows with its position covariance: `d_eff = d_min + k·σ_pos` |
| **Stale tracks** | A track that disappears from the tracker output keeps its keep-out for `coast_timeout`. When it is finally forgotten the filter holds for that cycle rather than flying on a command that assumes the keep-out is gone |
| **Limits** | Speed cap per mode, altitude floor and ceiling, geofence box |
| **Timeouts** | A stale setpoint or stale odometry brakes the drone to a hover |
| **E-stop** | `/mission/estop` latches: the filter holds until it is released |
| **Engagement gate** | In INTERCEPT mode the filter refuses an engagement that is not cleared, so a planner bug cannot bypass it |

It always works on velocity: a position setpoint is turned into one first, with the
controller's own `kp_pos`, and the output always carries `position_valid = false`, so the
controller tracks the *filtered* velocity instead of adding a position term of its own.

## The engagement gate

The gate and the abort live in the ROS-free `safety::EngagementGate`
(`include/safety/engagement_gate.hpp`), which `safety_filter_node` evaluates on
every cycle; the intercept guidance checks a subset of the same conditions itself,
so the two do not have to be trusted to agree. In INTERCEPT mode the interceptor
may close on the selected target only when **all** of the following hold:

- the mission is armed (`/mission/mode` carries the `armed` flag);
- the selected track (`/target/state`) is of class `uav`, in `/target/state` and in
  `/tracks` alike, and it has been confirmed for `min_confirmed_frames` cycles;
- the intruder is above `min_engage_altitude_agl` (10 m) and the interceptor is at
  most `engage_altitude_band` (5 m) below it;
- both are inside the geofence.

A person or a vehicle as the selected target latches the gate closed for the rest of
the mission. A condition that fails *during* an engagement aborts it, and the abort
latches as well; so does a person or a vehicle within `abort_ground_radius` (15 m,
in 3D) of the interceptor or of the capture point, and the e-stop. An abort brakes
the drone to a hover, and `MissionStatus` carries `abort_active` and the reason.

### Climbing is not engaging

The altitude band is a condition on the **engagement**, not on the take-off. The
interceptor starts on the ground, so a gate that held the drone completely until the
band was met could never open: the drone would never climb into the band. This is
the deadlock the first intercept runs hit (the guidance started the engagement, the
filter held every setpoint, and nothing moved).

So the authorisation to engage and the authorisation to climb are separate. While an
armed mission waits for a gate condition that may still clear (no target yet, not
confirmed yet, too far below the intruder), the filter lets the **vertical** part of
the planner's command through and drops the horizontal part ("engagement gate
closed: climb only"). The interceptor reaches the band without ever closing on the
intruder from below, which is the hazard the band is there for. The keep-out
barriers and the altitude limits still apply to the climb. An unarmed mission, a
latched gate and an abort hold the drone completely.

### The band has a hysteresis

The gate opens at the edge of the band, and the intruder's altitude is an estimate
with 0.2 m of noise, so without a margin the noise alone aborts an engagement that
began at the edge (the first straight-run scenario did exactly that). Once the
engagement is running, the interceptor may drop `engage_altitude_hysteresis` (2 m)
further before the engagement is aborted.

## The barrier

### The plan's form, and why it is not enough

Plan §4.3 gives the keep-out as a control barrier function on the margin

```
h = ‖p_xy − p_t,xy‖ − d_min        require  ḣ ≥ −α·h
```

which is a half-space in the *commanded* velocity:

```
n · (v − v_t) ≥ −α·h        (n points from the target to the drone)
```

The half-space is right. The problem is the **relative degree**.

`ḣ` depends on the closing speed the drone *actually has*, not the one it is told to
have, and the autopilot's velocity loop lags. Measured on the X500 flight model (a 4 m/s
step, `docs/FLIGHT_DYNAMICS.md`), the velocity loop has a time constant of **0.34 s**: it
reaches 63 % of the step after 0.60 s.

A barrier that only constrains the command is therefore satisfied instantly and then
coasted past. On the flight model the relative degree 1 form overshoots the keep-out
margin by **up to 6 m** against a 5 m standoff, and the overshoot grows with the
vehicle's lag. Nothing in a unit test that only inspects the returned command shows it;
it shows up in a closed loop.

### The relative degree 2 correction

The margin is relative degree 2 in the command, so the barrier is built on the achieved
motion:

```
ḧ + 2ω·ḣ + ω²·h ≥ 0
```

With `c` the achieved closing speed (positive while the range shrinks) and
`ċ = (c_cmd − c)/τ`:

```
c_cmd ≤ c·(1 − 2ω·τ) + ω²·τ·h
```

The correction bites exactly when the drone is closing too fast to stop in time: the term
`c·(1 − 2ω·τ)` is negative for `ω·τ > ½`, so the bound goes below zero and the drone is
told to open the distance.

The filter uses the strictest of three bounds:

```
c_cmd ≤ min( α·h ,  c·(1 − 2ω·τ) + ω²·τ·h ,  0  if the target is closing on the drone )
```

The first is the plan's own exponential approach, the second is the lag correction, and
the third makes the drone *yield*: when a person walks towards the drone, the drone does
not close any further, so the range and the margin are held.

### The sweep

The correction was not assumed. It was swept numerically over:

- the plant's time constant, 0.2 s to 1.0 s;
- a mismatch between the assumed and the real `τ` (the plant at 0.34 s, the filter
  assuming 0.2 s to 1.0 s);
- 0.3 m/s of noise on the measured closing speed;
- starting states from `h = 30 m` down to `h = 0.5 m`, and closing speeds from 0 to
  15 m/s.

With the plant at the measured 0.34 s and the filter assuming 0.5 s:

| | worst margin, all states | worst over the states from which stopping is possible |
|---|---|---|
| the plan's relative degree 1 law | **−3.11 m** | −0.11 m |
| the relative degree 2 correction | −1.00 m | **0.00 m** |

The correction holds the margin **exactly** (`h ≥ 0`) for every state from which stopping
is physically possible, where the relative degree 1 form overshoots in the same states.
From a state that is already unrecoverable no law can help, and the filter does the best
it can.

**The assumed `τ` must not be smaller than the real one.** Assuming 0.2 s for a plant
that runs at 0.34 s overshoots by up to 0.11 m even in the recoverable states; assuming
0.34 s or more holds exactly in all of them. The default of 0.5 s is therefore
deliberately on the safe side of the measured 0.34 s — it makes the drone approach a
little more gently than it has to.

The closed-loop property test in `test/test_safety_filter.cpp` reproduces this: it flies
the filter around a lagged plant for 10,000 steps and checks that the distance never drops
below `d_min − 0.05 m`.

## Parameters

All in `config/safety_params.yaml`:

| Key | Default | |
|---|---|---|
| `d_min_horizontal` | 5.0 m | to any person or vehicle track |
| `d_obstacle_min` | 3.0 m | to the surface of any static obstacle |
| `h_min_above` | 3.0 m | vertical clearance above a person or vehicle |
| `uncertainty_gain` | 2.0 | `k` in `d_eff = d_min + k·σ_pos` |
| `drone_radius` | 0.3 m | so the standoffs are surface to surface |
| `cbf_alpha` | 1.0 /s | the rate at which the barrier is enforced |
| `velocity_lag_tau` | 0.5 s | the assumed velocity-loop time constant (measured 0.34 s) |
| `barrier_omega` | 3.0 /s | the natural frequency of the lag correction |
| `coast_timeout` | 2.0 s | how long a disappeared track keeps its keep-out |
| `speed_cap` | 15 / 30 m/s | FOLLOW / INTERCEPT |
| `obstacles` | the trees and benches | flat `x, y, radius, height` quadruples |

The obstacle list is a flat list of quadruples because a parameter server cannot hold a
list of dictionaries. The values come from `worlds/intercept_scenario.world`, and the tree
canopies are the same cylinders `config/groundtruth_params.yaml` uses for its occlusion
test, so the two agree on where a tree is.

Each obstacle is a **vertical cylinder** from the ground to its height, and
`d_obstacle_min` is the 3D distance to its surface: beside the cylinder it is the
horizontal distance to its side, above it the height above its top, and beyond the
rim the diagonal distance to the rim (`safety::obstacleDistance`). The barrier pushes
along that direction. So a drone keeps 3 m from a tree's canopy whichever way it
passes: around it, over it, or over its edge. A low bench far below the drone does not
get in its way.

The first version was a 2D circle that stopped at the obstacle's height. The FOLLOW
drone flies 5 m above a person, at 6-7 m, which is the height of an oak's canopy. Above
6.5 m the tree did not exist for the filter, so the drone drifted over the canopy, and
when it sank below 6.5 m it was already inside the circle. The P2.4 scenario runs
measured it 1 m inside the canopy.

**If you change `velocity_lag_tau` or `barrier_omega`, re-run the sweep.** A value that is
too small lets the drone coast past the margin; a value that is too large makes it
approach so slowly that it never reaches the standoff. The defaults are deliberately
conservative: the filter assumes a slower vehicle than the X500 actually is.

## Precedence

When the limits and the keep-out disagree, **the keep-out wins** and the conflict is
reported in `MissionStatus.reason`. The speed cap is applied to the planner's request
before the barriers, and again afterwards, but the cap is only applied when the capped
velocity still satisfies every barrier: the keep-out is never traded away for speed.

The command is the **exact** projection of the planner's velocity onto the intersection
of all the barriers' half-spaces, together with the vertical speed cap and the altitude
floor and ceiling (`safety::projectOntoHalfSpaces`): the closest velocity that satisfies
every one. In 3D that point is fixed by at most three active half-spaces, so every set of
one, two and three is tried and the one that meets the KKT conditions is the answer.

Only when the intersection is empty — the drone really is boxed in — does the filter
command hover, and say so, rather than relax one of the distances.

The first version projected onto the half-spaces one at a time, ten rounds, and called it
a box when the rounds ran out. With several barriers pulling in different directions that
converges slowly: in the P2.4 jogger runs, 276 of 341 "conflicting keep-out constraints"
holds (81 %) had a velocity that satisfied every barrier, and the drone stopped dead while
the jogger ran at it.

## Seeing it in RViz

The filter publishes two things an operator can watch:

| Topic | What it carries |
|---|---|
| `/safety/keepouts` | A `MarkerArray` with one translucent cylinder per person or vehicle track and one per static obstacle. The radius of a track's circle is its *effective* standoff (`d_min + k·σ_pos`), so a badly tracked person gets a visibly bigger circle. The obstacle circles are the obstacle's own radius plus `d_obstacle_min`. |
| `/mission/status` | A `MissionStatus`: the mode, the phase, the selected track, the distance to it, whether the filter is active and — in `reason` — why it intervened ("keep-out from track 3", "engagement gate closed", "stale setpoint", "conflicting keep-out constraints"). |

The "Safety Keep-outs" display of `rviz/interceptor_config.rviz` shows the circles,
and the "Tracks" display next to them shows the tracks the circles come from.

## What is not here yet

- **P2.4** measures the result over randomized runs. The exit criteria are in the plan
  §5 Phase 2.
- The filter publishes the command it hands to the controller on `/safety/keepouts`,
  in the `setpoint_velocity` and `setpoint_target` namespaces: an arrow in the
  direction of the filtered velocity and the point it reaches in a second. The
  filtered velocity itself is on `/setpoint/safe`, a custom message that RViz cannot
  draw, so without that marker the one command that matters — the filtered one —
  would be invisible. The guidance's own `/guidance/vectors` (the line of sight and
  the commanded acceleration) is a different thing: it is what the *planner* asked
  for, before the filter had its say.
