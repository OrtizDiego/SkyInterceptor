# Task: P2.5 (finish) — the vision path's framing score

**Roadmap:** `ROADMAP.md` → `### P2.5 – Vision in the loop for FOLLOW` (currently `in_progress`).
**Branch:** `feat/p2-5-framing-score`.
**Depends on:** P2.4 (the metrics harness) — PR #31.

## What the task is

The plan's Phase 2 exit criteria end with:

> *With vision: >= 90 % of the ground-truth framing score.*

The vision path **works** — PR #26 fixed the four bugs that stopped it, and the
detector now reports the person's 3D position with a median error of 0.36 m
horizontally. What is missing is the *comparison*: the vision pipeline has to
frame the target as well as the ground-truth path does, and nobody has measured
that.

## The score being compared

"Framing" is how well the target sits in the camera's frame. The metric is
already defined by P2.4's metrics recorder:

- the **framing error** is the *azimuthal* angle between the drone's boresight
  and the bearing to the target (the camera is mounted level on the nose, the
  body frame is FLU, so the boresight is the body's +x);
- the **framing score** is the fraction of the scored time the target is within
  **±15°** of the boresight;
- the criteria for the ground-truth path are **≥ 95 %** of the time, and the
  vision path has to reach **≥ 90 %** of that.

The full 3D cone angle is reported too, but it is *not* the criterion: the
planner deliberately flies 5 m above and 8 m behind the target, which is a
32° cone offset, so the cone angle is dominated by the geometry the planner
asked for rather than by the framing.

## How to do it

1. **Measure the ground-truth baseline first.** Run P2.4's scenarios with
   `perception_source:=groundtruth` and take the framing score over the same
   runs. That is the denominator, and it has to come from the *same* scenarios
   and the same seeds, or the comparison is meaningless.
2. **Then the vision path**, with `perception_source:=vision`, over the same
   scenarios and seeds.
3. **Report the ratio** (vision score / ground-truth score) and the verdict
   against ≥ 90 %.

The vision path is slower than the ground-truth one — the depth map runs at
about 6 Hz (SGBM + WLS, ~160 ms a frame), which caps the rate of the 3D
detections, and the tracker can lose the target when the detections are sparse.
So the comparison is not only about the framing: it is about whether the vision
path can *keep* a track at all, at the frame rate the perception pipeline
delivers. A half-resolution depth map would fix the rate, but it changes the
localizer's back-projection, so it is a real change and not a free one.

## What already exists

- `scripts/check.sh` — the CI mirror, and the way to build a worktree.
- P2.4's metrics recorder and batch runner (PR #31) — the framing score, the
  verdicts and the batch are already there; this task is a *comparison*, not a
  new harness.
- `docs/FOLLOW.md` — the FOLLOW mode, what to watch and what is not measured
  yet.

## The pitfalls, all of which cost time in the previous session

- **The container's `src` is bind-mounted from wherever `docker-compose` was
  started**, which was a worktree, not `main`. A build in
  `/workspace/interceptor_ws` tests the wrong tree. Check with
  `docker exec interceptor findmnt -T /workspace/interceptor_ws/src -o TARGET,SOURCE`
  and use `./scripts/check.sh <src-dir>` instead.
- **Run under your own `ROS_DOMAIN_ID`.** The container carries ~80 stale
  `robot_state_publisher` processes from finished sessions; they publish the
  same TF frame names and silently override each other. A different domain
  insulates you. Do not kill processes that are not yours.
- **The image has NumPy 2 and a torch that the host driver cannot use.** The
  code-level fixes work anyway (the detector decodes the image with NumPy
  directly). For the GPU, `PYTHONPATH=/opt/torch124` gives a cu124 torch that
  this driver *can* use; the durable fix is the Dockerfile pin in PR #26, which
  needs an image rebuild.
- **The `gzserver` model-database download can outlast a 30 s spawn timeout** on
  a cold start.
- **ROS CLI across separate `docker exec` sessions** needs
  `FASTRTPS_DEFAULT_PROFILES_FILE=<ws>/src/fastdds_disable_shm.xml`.

## Done means

- The framing score of the ground-truth path, of the vision path, and the ratio
  between them, over the same scenarios and seeds.
- The verdict against ≥ 90 %.
- The roadmap section updated in the same PR, with what you measured and what is
  still open.
- **Report exactly how many runs you did.** Do not extrapolate.
