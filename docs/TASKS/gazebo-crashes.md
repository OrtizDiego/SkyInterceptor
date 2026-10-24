# Task: gzserver crashes in the scenario batches

**Roadmap:** infrastructure (it costs every batch 15–20 % of its attempts).
**Branch:** `fix/gazebo-crashes`. **Its own PR.** Do it after the data mount task.

## What is known

- In the batches of 2026-10-23/24, **15–20 % of the headless attempts produced no
  result**: 43 of ~143 in one 100-run follow batch, 40 retries for the final 100
  runs, 23 + 3 in the intercept batches. When it was visible in a log, `gzserver`
  had died with **exit code -11 (SIGSEGV)**, typically 15–20 s after the launch,
  around the time the drones are spawned and the scenario starts moving entities.
- The intercept runner keeps one log per attempt (`launch_attempt<N>.log`); the
  follow runner overwrites `launch.log` on a retry, so its failed attempts' logs
  are lost. **Make the follow runner keep per-attempt logs first** (copy what
  `run_intercept_batch.py` does).
- Both runners retry at once when the log says gzserver died (`simulator_died()`),
  so a crash costs ~20 s, not the 400 s wall-clock timeout it used to.
- The interceptor URDF carries **two stereo camera sensors** (rendered images),
  which the ground-truth runs never use. Headless rendering of camera sensors is a
  classic source of gzserver segfaults.
- The batches ran on WSL2 with Docker Desktop. The crash rate may differ on a
  native Linux host (see `HANDOFF.md`).

## What to do

1. **Look before changing anything.** Read the last gzserver lines before the crash
   in a few failed attempts (the intercept batches have them:
   `/tmp/p35batch/*/launch_attempt*.log` if the container still has them, otherwise
   produce new ones). Try to get **one backtrace**: run `gzserver` under `gdb -batch
   -ex run -ex bt` (or enable core dumps, `ulimit -c unlimited`, and read the core).
2. **Add a xacro argument** `enable_cameras` (default `true`) that leaves out the two
   stereo camera sensors from `urdf/interceptor_quadrotor.urdf.xacro`. The scenario
   launch files (`follow_scenario.launch.py`, `intercept_scenario.launch.py`) set it
   to `false` when `perception_source` is `groundtruth`; vision runs keep the
   cameras. Thread it through `simulation.launch.py` / `interceptor_full.launch.py`.
3. **A/B test**: about 20 headless runs with the cameras and about 20 without (same
   scenario, same seeds). Report the **crash rate** and the **wall-clock time per
   run** for both. Report exactly how many runs and crashes; do not extrapolate.
4. If the crashes **remain** without the cameras: make the scenario
   (`follow_scenario_node`), behaviour (`target_drone_behavior_node`) and
   ground-truth (`groundtruth_target_node`) nodes **wait until the entity exists and
   `/clock` is running** before they call `/set_entity_state` / `/get_entity_state`,
   and measure again.

## Done when

- the crash rate with and without the cameras is measured and reported;
- ground-truth scenario runs use the configuration that crashes less (if the
  cameras are the cause, without them);
- the follow runner keeps per-attempt logs;
- one backtrace (or a clear statement that none could be obtained) is in the PR.

## Pitfalls

- A changed URDF changes the airframe's inertia slightly (the camera links have
  mass): check that the flight model's parameters still come out the same, or that
  the difference is negligible, and re-run the flight dynamics gtests.
- Vision runs (P2.5, P3.2) need the cameras: the default must stay `true`.
