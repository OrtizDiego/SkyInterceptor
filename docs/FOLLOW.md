# FOLLOW mode (the aerial filming planner)

The FOLLOW mode is a filming drone: it keeps a person, a bicycle or a car in the
frame while it flies. The constraint that dominates every other decision is the
safety distance — the drone never comes closer to a person or a vehicle than
`d_min`, and never closer to a static obstacle than `d_obstacle_min`. That is not
the planner's job: the planner asks for a framing, and the safety filter decides
whether the drone may have it.

## The chain

```
/target/state  ─┐
/odom          ─┼─► follow_planner_node ──► /setpoint/raw ──► safety_filter_node
/mission/mode  ─┘        (a velocity)                            │
                                                              ▼
                                                       /setpoint/safe
                                                              │
                                                              ▼
                                                  trajectory_controller_node
```

The planner publishes a **velocity** setpoint with `position_valid = false`. The
controller flies a velocity loop, and the safety filter always works on a velocity
too, so a position setpoint in the middle of the chain would have to be turned
into one twice, and the second conversion would be behind the filter's back. The
planner therefore asks for the velocity that reaches the framing point, and
nothing else.

## The framing presets

The framing point is expressed in the **target's heading frame**:

```
p_des = p_t + R_z(psi_t) · [dx, dy, dz]
v_des = v_t + K_p · (p_des − p)                capped at 15 m/s
yaw   = the bearing from the drone to the target
```

| Preset | The framing point | The offsets |
|---|---|---|
| `BEHIND` | behind the target, along its heading | `[-behind, 0, height]` |
| `SIDE` | beside the target, laterally | `[0, side, height]` |
| `LEAD` | in front of the target, looking back at it | `[behind, 0, height]` |
| `ORBIT` | circling the target at a fixed angular rate | `R_z(psi_t + theta + pi)` applied to `[behind, 0, height]` |

The offsets come from `config/follow_params.yaml`: 8 m behind, 6 m to the side and
5 m above the target by default. `ORBIT` starts where `BEHIND` does (its phase
`theta` is 0) and walks the framing point around the target at `orbit_rate`
(0.2 rad/s), **in the target's heading frame**, so the circle follows the target's
own turns instead of staying fixed in the world.

The preset is a parameter of the planner (`follow.preset`) and it can be changed
while the node runs: the node registers a parameter callback, so `ros2 param set` on
the preset takes effect immediately and an unknown name is refused with the reason
in the parameter service's answer. The offsets are far enough from the target that
the safety filter rarely has to intervene: the filter's own keep-out is 5 m.

## Three things the formula hides

**The heading is filtered.** The tracker's heading is the direction of the target's
velocity. It rotates fast when the target turns, and it jumps by 180 degrees when
a noisy velocity flips sign — the tracker holds it, rather than publishing noise,
when the target is barely moving. The preset frame is therefore built from a
**low-pass filtered** heading (time constant `heading_tau`, 0.8 s), and the
planner holds its own filter too when the target is slower than
`heading_min_speed` (0.3 m/s), because below that the direction of the velocity
is meaningless. The result is that the framing point slides around the target
instead of teleporting from one side to the other.

The *commanded yaw* does not use the filtered heading at all: it is the geometric
bearing to the target, so the camera keeps the subject in the middle of the frame
whatever the subject does. A target that turns makes the framing point slide and
the nose follow, instead of the camera whipping around.

**The target's velocity is a feed-forward.** Without the `v_t` term the drone lags
a fast target by `v_t / K_p` metres — 5 m of lag at 4 m/s with the default gain
(0.8 /s). With it, a target that moves at a constant velocity is followed exactly.
The cost is that the drone inherits the target's speed, which is why the whole
command is capped.

**The velocity is capped twice.** The vertical component at `max_vertical_speed`
(5 m/s, the safety filter's climb limit) and the whole vector at `max_speed`
(15 m/s, the plan's cap and the safety filter's FOLLOW cap). The filter caps it
again, so the two agree on the ceiling.

## The lost-track machine

A target is *usable* when a message arrived recently (`target_timeout`, 0.5 s),
the track is confirmed and valid, and it is the track the operator asked for.
Anything else counts as a lost track, and the time since the last usable target
drives the state machine:

| Phase | When | What the drone does |
|---|---|---|
| `COAST` | the first `lost_hold_time` (2 s) | keeps flying the last command |
| `HOLD` | until `lost_rtl_time` (10 s) | hovers: zero velocity, holds the position |
| `RTL` | after 10 s | flies back to the home position |

`COAST` is the grace period for a momentary dropout — one missed frame, a
glimpse of the target through the trees. The last command is the best guess
available and the drone keeps flying it. `HOLD` is the safe answer to a target
that is really gone: the drone stops where it is rather than chasing a stale
position. `RTL` is the last resort.

The home position is where the drone held on the **first usable frame of the
follow session**, i.e. where the follow started. It is captured when the mode
enters FOLLOW and thrown away when the mode leaves it, so every follow session
has its own home. If no home is known — the target was never seen, so the follow
never started — the drone keeps holding instead of guessing a position to fly to.

## The mode gate and the arming

The planner publishes **nothing at all** when the mode is not FOLLOW. In INTERCEPT
the guidance node owns `/setpoint/raw`, and a second publisher on that topic would
make the safety filter alternate between the two commands, which could abort an
engagement. Staying silent is safe: the safety filter brakes to hover on its own
when the setpoint goes stale (0.3 s).

The drone starts disarmed. The mission manager only arms an INTERCEPT mission —
that flag is the engagement authorization, not the motors — and the guidance node
arms the drone itself in INTERCEPT. In FOLLOW nobody else does it, so the planner
arms the motors the first time it has the odometry and the mode is FOLLOW
(`arm_on_start`, on by default). Without this the setpoints would command a drone
that never takes off.

The planner also stops publishing when the odometry is stale (`odom_timeout`,
0.3 s): it does not know where the drone is, so it has nothing sensible to say.

## The operator's track

The tracker's target selection and the planner enforce the same rule twice. The
operator can name a track on `/mission/set_mode`; the tracker then selects exactly
that track and never falls back to another one, and the planner treats any other
track on `/target/state` as a **lost** target rather than as a subject to follow.
A second person walking into the shot cannot steal the camera.

## The safety filter in series

The planner does not enforce the safety distance itself, and it must not: the
filter is the last word, and it applies to *every* person and vehicle on
`/tracks`, not only to the selected one. The filter's keep-out is inflated by the
position uncertainty of the track (`d_min + k·σ_pos`), so a poorly tracked person
gets a bigger margin, and it is applied to the static obstacles as well. The
distance rules are never lowered to make a command feasible: when the constraints
conflict, the filter commands hover and says so in `/mission/status`. See
[`SAFETY.md`](SAFETY.md).

## Running it

```bash
# The whole system in FOLLOW mode, with the ground-truth target source
ros2 launch interceptor_drone follow.launch.py

# The same, with the vision pipeline instead of the ground truth
ros2 launch interceptor_drone interceptor_full.launch.py mission_mode:=follow perception_source:=vision

# The tracker and the planner only, no simulation
ros2 launch interceptor_drone guidance.launch.py mission_mode:=follow
```

The preset is read from `config/follow_params.yaml` at startup, and it can be
switched while the node runs:

```bash
ros2 param get /follow_planner_node follow.preset
ros2 param set /follow_planner_node follow.preset ORBIT
```

The eligible classes are `follow.eligible_classes` in `config/follow_params.yaml`:
`person`, `bicycle` and `car`. The tracker refuses to select anything else, and the
planner follows only what the tracker selects.

## Watching a session

| What | Where |
|---|---|
| the selected target, its velocity, its heading and its covariance | `/target/state` |
| every track, with its class and its confirmation | `/tracks` and the "Tracks" display in RViz |
| the keep-out circles of the safety filter | `/safety/keepouts` and the "Safety Keep-outs" display in RViz |
| why the filter intervened, and the phase | `/mission/status` |
| the setpoint the planner asked for | `/setpoint/raw` (a `FlightSetpoint`) |
| the setpoint the drone actually flies | `/setpoint/safe` |

`/setpoint/raw` and `/setpoint/safe` are custom messages, so RViz cannot draw
them; the two are worth logging instead. The guidance's own `/guidance/vectors`
(the line of sight and the commanded acceleration) is drawn in RViz, but it belongs
to the INTERCEPT mode.

## What is not measured yet

The plan's Phase 2 exit criteria (plan §5) ask for 100 randomized runs and three
numbers: the minimum distance to every person and every obstacle, the fraction of
the time the target is within ±15° of the camera boresight, and the number of
track losses and how long the reacquisition took. The scenario harness and the
metrics recorder that produce them are the open **P2.4** task, so the numbers
above are the ones an operator can watch by hand, not a measured score.

The vision loop is P2.5: the detector, the depth and the 3D localizer work (see
the notes in [`ROADMAP.md`](../ROADMAP.md)), but the framing score it produces
still needs the P2.4 harness to be comparable.
