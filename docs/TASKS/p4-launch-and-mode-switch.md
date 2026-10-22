# Task: P4 (finish) — the launch files and the mode-switch demo

**Roadmap:** `ROADMAP.md` → `### P4 – Integration and docs` (currently `in_progress`).
**Branch:** `feat/p4-launch-and-mode-switch`.
**Depends on:** P2.4 (the FOLLOW scenario) — PR #31 — and P3.5 (the INTERCEPT
scenario runner) — PR #33.

## What the task is

The plan's Phase 4 exit criteria:

> follow/intercept launch files, mode-switch demo, RViz config with tracks,
> keep-out circles and setpoints, README and CLAUDE.md updated.

**Done already** (PR #28 and PR #30):

- the README, `CLAUDE.md` and `docs/` are up to date, and `docs/FOLLOW.md` is
  new;
- the RViz config shows the tracks, the keep-out circles, the guidance vectors
  and the filtered setpoint (the safety filter now publishes an arrow in the
  direction of the filtered velocity and the point it reaches in a second).

**Left:** the launch files and the mode-switch demo.

## The launch files

`launch/follow.launch.py` and `launch/intercept.launch.py` already exist and
are thin wrappers over `interceptor_full.launch.py`. What they need is to be
*complete* — the full argument set of the thing they launch, and the two new
scenarios:

- `follow_scenario.launch.py` (from P2.4) selects the FOLLOW scenario —
  `walker`, `jogger`, `occlusion`, `bystander` — with a seed;
- the INTERCEPT side has the target drone's behaviours (`hover`, `waypoints`,
  `straight`, `weave`, `curve`) through `target_behaviour`, and the seed
  through `groundtruth_seed`, both of which `intercept_demo.launch.py` already
  takes.

Check the argument list against the launch files themselves rather than against
this document: the arguments are `mission_mode`, `perception_source`,
`guidance_law`, `target_behaviour`, `auto_arm`, `groundtruth_seed`, the spawn
positions (`interceptor_x`, `interceptor_y`, `target_x`, `target_y`) and `gui`.

## The mode-switch demo

A demo that shows the **same drone** changing mission mode at runtime: follow
a person, then switch to INTERCEPT and engage the intruder, then back. The
mission mode is owned by `mission_manager_node`, which serves `SetMissionMode`
on `/mission/set_mode` and latches `/mission/mode` (transient local), so the
tracker, the planners and the safety filter all see the same mode.

The interesting part is not the service call, it is what has to be true for the
switch to be *safe*:

- **Only one planner may publish on `/setpoint/raw` at a time.** The follow
  planner publishes nothing outside FOLLOW and the intercept guidance publishes
  only in INTERCEPT — that is deliberate, because a second publisher on the
  topic would make the safety filter alternate between two commands, which
  could abort an engagement. A mode switch must not open that window: the
  planner that is leaving has to go quiet before the one that is arriving
  starts. Check how the current code sequences that, and make the demo
  demonstrate it rather than assume it.
- **The safety filter's gate and abort are reset by a new mission**, not by the
  mode alone. The abort latches: an e-stop abort is cleared by a new mission
  (a mode change or a re-arm), and outside INTERCEPT the e-stop is the filter's
  own latch. A demo that switches modes has to show what happens to a latched
  abort.
- **The engagement gate** (P3.4) decides whether an INTERCEPT may start at
  all: the mission has to be armed, the target has to be a confirmed `uav`,
  and the altitude and geofence conditions have to hold. A demo that switches
  into INTERCEPT without arming will simply be refused, with the reason on
  `/mission/status`.
- **The drone's arming is per-mode.** The mission manager only arms an
  INTERCEPT mission (that flag is the engagement authorisation, not the motors);
  the motors are armed by the guidance node in INTERCEPT and by the follow
  planner in FOLLOW, each on `/drone/arm`.

## What a good demo looks like

It should be runnable with one command and it should show, in RViz, the mode
changing and the behaviour changing with it: the tracks, the keep-out circles,
the filtered setpoint, and the phase and the reason on `/mission/status`. The
`/mission/status` topic is where the reason a mode change was refused (or an
abort fired) shows up — a demo that does not show it is hiding the interesting
part.

## The pitfalls

- The container's `src` is bind-mounted from a worktree, not from `main`, so
  build with `./scripts/check.sh <src-dir>` and never in
  `/workspace/interceptor_ws`.
- Run under your own `ROS_DOMAIN_ID` and `GAZEBO_MASTER_URI` port; the
  container carries stale `robot_state_publisher` processes from finished
  sessions that fight over the TF frames.
- `gzserver`'s model-database download can outlast a 30 s spawn timeout on a
  cold start.
- Every config file is loaded by the launch files and the parameters are never
  hard-coded in them.

## Done means

- The launch files complete and consistent with the arguments they pass.
- A mode-switch demo that runs with one command and shows the switch in RViz,
  including what happens to a latched abort and to an unarmed INTERCEPT.
- The roadmap section updated in the same PR, with what you verified and what
  is still open.
