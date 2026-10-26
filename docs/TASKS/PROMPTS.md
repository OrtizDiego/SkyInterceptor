# Launching the next tasks — ready-to-paste prompts

The previous session (on the old WSL machine) queued these tasks as one-click chips in
the Claude desktop app. Those chips do not travel to another machine, so here they are
as prompts you paste into Claude Code. Each prompt is self-contained: it points the
agent at `HANDOFF.md` and at the task's spec in `docs/TASKS/`.

## How to launch one task

Do this **after** `docs/MOVE_TO_UBUNTU.md` is finished (`make doctor` passes).

1. Open a terminal in the main checkout and update it:

   ```bash
   cd ~/code/SkyInterceptor && git pull
   ```

2. Start Claude Code **in a new worktree** for the task (the name is the one given
   with each prompt below), so every task gets its own branch and folder:

   ```bash
   claude -w data-mount
   ```

3. Paste the task's prompt (below) as the first message.

Rules of thumb:

- **One task per session, in the order below.** The tasks share one container and
  one GPU, and the Gazebo batches load the CPU, so running two batch-heavy tasks at
  once would distort both. Tasks 1 and 2 must come first.
- The container keeps mounting the **main checkout** (`make up` from
  `~/code/SkyInterceptor`). Agents build and test their worktree in a scratch
  workspace with `./scripts/check.sh <src-dir>`; the prompts say so.
- When a task's PR is merged, `git pull` in the main checkout before starting the
  next one.
- To check where things stand at any time: `python3 scripts/roadmap.py summary`,
  `gh pr list`, and `HANDOFF.md`.

---

## 1. Persistent data mount — `claude -w data-mount`

```text
We are on native Ubuntu (the move from WSL is done; docs/MOVE_TO_UBUNTU.md). Read
HANDOFF.md and docs/TASKS/data-mount.md first; that spec is the task. Its own PR,
branch feat/data-mount from origin/main.

The mount itself is already in the compose files (~/skyinterceptor-data ->
/workspace/data, created by make up). What is left:
- Make /workspace/data/... the default output of every script that writes datasets,
  weights or batch results, instead of /tmp: src/tools/run_follow_batch.py (now
  /tmp/follow_batch) and src/tools/run_intercept_batch.py (now /tmp/intercept_batch)
  under interceptor_ws/src/interceptor_drone, e.g.
  /workspace/data/batches/<follow|intercept>/<timestamp>; say so in --help. The P3.2
  scripts (PR #32, scripts/uas_dataset/) follow when that PR is rebased.
  scripts/check.sh scratch builds may stay in /tmp (say so in a comment).
- Document it in DOCKER.md and HANDOFF.md. Rule: big artefacts never go in /tmp.
- Check that the container's src is the main checkout and that a file written to
  /workspace/data survives `make down && make up` (`make doctor` shows both mounts).
Build/test with ./scripts/check.sh <src-dir> inside the container (CI uses -Werror).
Update ROADMAP.md only if a roadmap task is touched (this is infrastructure).
```

## 2. Gazebo crashes — `claude -w gazebo-crashes`

```text
We are on native Ubuntu. Read HANDOFF.md and docs/TASKS/gazebo-crashes.md first;
that spec is the task. Its own PR, branch fix/gazebo-crashes from origin/main. Do it
after the data mount task.

On the old WSL machine 15-20 % of the headless scenario attempts produced no result;
when visible, gzserver died with exit code -11 about 15-20 s after launch.
1. FIRST measure the crash rate here, unchanged, with the cameras on: about 20
   headless runs (e.g. run_follow_batch --runs 20 --scenarios walker). It may already
   be gone without WSL's D3D12 rendering. Report runs and crashes exactly.
2. Make run_follow_batch keep per-attempt logs (launch_attempt<N>.log, like
   run_intercept_batch.py). Read the last gzserver lines before a crash and try to get
   one backtrace (gdb -batch or core dumps).
3. Add a xacro argument enable_cameras (default true) to
   urdf/interceptor_quadrotor.urdf.xacro that leaves out the two stereo camera
   sensors; the scenario launch files set it false when perception_source is
   groundtruth; vision runs keep the cameras.
4. A/B: about 20 headless runs with the cameras and about 20 without, same scenario
   and seeds. Report the crash rate and the wall-clock time per run for both.
5. If crashes remain without the cameras, make follow_scenario_node,
   target_drone_behavior_node and groundtruth_target_node wait until the entity exists
   and /clock is running before calling the entity-state services, and measure again.
Run batches detached (setsid nohup), results in /workspace/data, never /tmp.
Build/test with ./scripts/check.sh <src-dir> (CI uses -Werror).
```

## 3. Safety filter rate conflict — `claude -w safety-rate-conflict`

```text
We are on native Ubuntu. Read HANDOFF.md, docs/SAFETY.md and
docs/TASKS/safety-rate-conflict.md first; that spec is the task.

PR #40 (merged) already implements the core: when the exact projection (#38) is
infeasible, the rates of the least urgent barriers (largest margin h first) are
relaxed one at a time until feasible; only barriers with h >= safety.relax_min_margin
(2.0 m) are eligible; the reason names the relaxed barrier; with nothing eligible the
drone holds. Code: include/safety/safety_filter.hpp, src/safety/safety_filter.cpp,
test/test_safety_filter.cpp under interceptor_ws/src/interceptor_drone.

Not done yet:
1. The threshold: my request said h > 1 m; #40 uses 2 m. Justify or change it.
2. During a hold, keep the yaw tracking the planner's yaw setpoint (and the #41
   bearing-rate feed-forward) instead of freezing it (SafetyFilter::hold sets
   yaw = state.yaw, yaw_rate = 0). Decide for the e-stop and abort holds too.
3. A randomized property test that the 3D and horizontal distances never drop below
   their limits with the relaxation in play.
4. Then re-run the jogger and occlusion scenarios, 10 runs each, and report runs,
   retried gzserver crashes, framing, distance_ok, reacquire_ok, passed — exactly.
Either implement it as its own PR (branch fix/safety-rate-conflict) or, if the
hold-yaw change or the threshold raises a safety question, propose a design first and
wait for my go-ahead. Keep the documented limitation (the exactly-aligned
jogger/drone/tree case, 4.7 m against 5.0) visible.
```

## 4. P3.2 UAS detector — `claude -w p3-2-uas-detector`

```text
We are on native Ubuntu. Read HANDOFF.md and docs/TASKS/p3-2-uas-detector.md first;
that spec is the task. Do it after the data mount and Gazebo crash tasks.
Branch feat/p3-2-uas-detector, draft PR #32 (far behind main: rebase it first).
- Regenerate the dataset into /workspace/data/uas/ with a fixed seed
  (scripts/uas_dataset/generate_dataset.py). Commit only the config, the seed and
  manifest.csv — never images or weights.
- Train on the GPU and say which device was actually used (log
  torch.cuda.is_available() and the device name). The image's torch is the CUDA 12.4
  build; on this native host it should see the GPU directly (the old /opt/torch124
  workaround was for WSL). Fix the earlier DataLoader worker crash (workers or the
  container's shm_size).
- Evaluate on the held-out split (precision, recall, mAP50, mAP50-95), export ONNX
  (TensorRT if possible), wire the model and the uav class into the detector config.
- Prove it end to end with perception_source:=vision: detector -> 3D localizer ->
  confirmed uav track. The stereo cameras must be on (enable_cameras true).
- Update PR #32 and the P3.2 section of ROADMAP.md (runs, device, metrics, open items).
Build/test with ./scripts/check.sh <src-dir> (CI uses -Werror).
```

## 5. P2.4 jogger and corners — `claude -w p2-4-jogger-corners`

```text
We are on native Ubuntu. Read HANDOFF.md, the P2.4 section of ROADMAP.md and
docs/TASKS/p2-4-jogger-corners.md first; that spec is the task. Branch
feat/p2-4-jogger-corners from origin/main.

In the last P2.4 batch (main with #37-#41) the jogger failed 25/25: the 3D distance
held (worst margin +0.56 m) but the horizontal standoff broke in every run by
1.5-2.3 m, and framing was 65-88 % (criterion 95 %). The safety filter was not holding
when the framing was lost: the follow planner's BEHIND framing point cuts inside the
corners of the jogger's 4 m/s loop, so the jogger comes round the corner straight at
the drone and the reactive barrier cannot get the lagging drone out of the way.
Fix it in the follow planner (include/follow/, src/follow/), e.g. keep the framing
point clear of the target's predicted path, limit how fast it swings, or put it on
the outside of the turn. Evaluate in a ROS-free closed loop first (planner + lag
plant + safety filter), then in Gazebo. Report the jogger (25 runs) and re-run
walker, bystander and occlusion (25 each) to show they still pass. Report runs and
retried crashes exactly; no extrapolation. Any yaw change must be re-checked on the
intercept scenarios. Update the P2.4 roadmap section (done only if all four pass).
```

## 6. P2.5 framing score — `claude -w p2-5-framing-score`

```text
We are on native Ubuntu. Read HANDOFF.md and docs/TASKS/p2-5-framing-score.md first;
that spec is the task (branch feat/p2-5-framing-score). P2.4's harness is on main now.
Measure the vision path's framing score against the ground-truth baseline (>= 90 % of
it) over the same scenarios and seeds; report runs, crashes and both scores exactly.
```

## 7. P4 launch files and mode-switch demo — `claude -w p4-launch-and-mode-switch`

```text
We are on native Ubuntu. Read HANDOFF.md and docs/TASKS/p4-launch-and-mode-switch.md
first; that spec is the task (branch feat/p4-launch-and-mode-switch). P2.4's and
P3.5's runners are on main now.
```

---

## The old session's transcript (optional)

The archive from the old machine also carries the full transcript of the session that
did P3.5, P2.4 and the five safety/control fixes (`skyinterceptor-data/claude-session/`).
**You do not need it** — everything that matters is in `HANDOFF.md`, `ROADMAP.md`, the
PRs and `docs/TASKS/`, and a fresh session per task works better than resuming a very
long one. It is there to read, or to resume if you really want to: see the
`README.md` next to it.
