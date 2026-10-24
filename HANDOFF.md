# HANDOFF — SkyInterceptor, 2026-10-24

The plan is `IMPLEMENTATION_PLAN.md`; the source of truth for progress is `ROADMAP.md`
(`python3 scripts/roadmap.py ready` says what can start, `summary` gives the counts).
**14 of 18 tasks are done.** In progress: P2.4 (the jogger scenario), P2.5 (vision
framing score), P4 (launch files, mode-switch demo). To do: P3.2 (UAS detector).

**The rule that matters:** one task, one branch, one PR; the roadmap section is updated
in the same PR. Report exactly how many runs were done and how many were retried; never
extrapolate. Each remaining job has a spec in `docs/TASKS/`.

---

## 1. What landed in the last session (2026-10-23/24)

| PR | What |
|---|---|
| #33 | **P3.5 done.** Intercept scenarios + metrics; the take-off deadlock fixed (an armed mission may *climb* while the engagement gate is closed, not close on the intruder; 2 m band hysteresis). 125 Gazebo runs: hover 25/25, straight 24/25, weave 25/25 captures, bystander abort 25/25, zero non-uav engagements. |
| #37 | Safety: obstacles are 3D cylinders — `d_obstacle_min` holds above a canopy too. |
| #38 | Safety: the exact projection onto the barriers (was 10 rounds of one-at-a-time projection that gave up on feasible commands and held the drone). |
| #39 | Flight controller: anti-windup on the heading setpoint (a lead against the yaw-rate command is dropped). |
| #40 | Safety: when barrier rates conflict, relax the least urgent barrier's rate (margin ≥ 2 m) before holding. |
| #41 | Control: heading gain 3 → 1.5, rate limit 2.5 → 1.5 rad/s, and a bearing-rate yaw feed-forward from the follow planner and the intercept guidance. Intercept re-check 40/40. |
| #31 | **P2.4 harness** (scenarios, ROS-free recorder, batch runner). Walker 25/25, bystander 24/25, occlusion 24/25 (framing over the tracked time, by decision), **jogger 0/25** — P2.4 stays in progress. |

The details, numbers and decisions are in each PR and in the roadmap sections.

---

## 2. What to do next, in this order

| # | Task | Spec | Notes |
|---|---|---|---|
| 0 | *(optional)* Move to the native Ubuntu machine | §3 below | Before 1: the container is recreated there anyway. |
| 1 | **Persistent data mount** | `docs/TASKS/data-mount.md` | Own PR. Needs the container recreated, from the **main** checkout. Rule: big artefacts never go in `/tmp`. |
| 2 | **Gazebo crashes** | `docs/TASKS/gazebo-crashes.md` | Own PR. gzserver segfaults on 15–20 % of headless attempts. A/B with and without the stereo cameras. |
| 3 | **Safety filter rate conflict** (finish or propose) | `docs/TASKS/safety-rate-conflict.md` | Own PR, or a design proposal first. #40 already does most of it; what is left: the threshold (1 m vs 2 m), yaw tracking during a hold, a randomized property test with the relaxation, then 10 jogger + 10 occlusion runs. |
| 4 | **P3.2 UAS detector** | `docs/TASKS/p3-2-uas-detector.md` | After 1 and 2. PR #32 (draft, far behind main). Dataset into `/workspace/data/uas/`, train on the GPU, say which device, ONNX, end to end with vision. |
| 5 | **P2.4 jogger + corners** | `docs/TASKS/p2-4-jogger-corners.md` | The follow planner puts the drone in the jogger's path at the corners. Fix in the planner, not the filter. |
| 6 | P2.5 framing score | `docs/TASKS/p2-5-framing-score.md` | Now unblocked (P2.4's harness is on main). |
| 7 | P4 launch files + mode-switch demo | `docs/TASKS/p4-launch-and-mode-switch.md` | Needs P2.4 and P3.5's runners (both on main now). |

---

## 3. Moving to a native Ubuntu host

It helps, for four reasons seen in this project:

- **Docker on WSL2 was fragile**: Docker Desktop was not running at the start of the
  last session, and the WSL distro had no `docker` until it was.
- **The GPU stack**: the image's torch (cu130) needs a newer driver than the WSL host
  had, so P3.2 training needed a side-installed torch (`/opt/torch124`). A native host
  with a current NVIDIA driver and the container toolkit removes that workaround.
- **gzserver crashes** may be partly a WSL2/WSLg rendering effect — the camera sensors
  render through WSLg here. Measure the crash rate on the target machine, not on WSL.
- **Speed and file-system overhead** of the WSL2 VM and `/mnt` paths.

**When:** now, before task 1 — the data mount task recreates the container anyway,
and the Gazebo crash A/B (task 2) and the GPU training (task 4) should be measured on
the machine that will keep running them. Things to carry over: the repo (push all
branches first), `~/skyinterceptor-data` (empty for now — nothing big survived the
last recreate), the Claude memory directory, and the GitHub/`gh` login. On the new
host: NVIDIA driver + `nvidia-container-toolkit`, `docker compose`, then `make build`
and recreate the container from the main checkout. The WSL-specific mounts in
`docker-compose.yml` (`/usr/lib/wsl`, `/mnt/wslg`) have to be made conditional or
removed there.

---

## 4. Environment notes (learned the hard way)

- **Build and test a branch in a scratch workspace inside the container**, never in
  the shared `/workspace/interceptor_ws`: `./scripts/check.sh <src-dir>` (CI mirror:
  static checks, `-Werror` build, tests), or copy `interceptor_ws/src/.` into
  `/tmp/<ws>/src` and `colcon build` there. The container's `src` mount follows
  whichever directory `docker compose` was started from — often an old worktree.
- **CI builds with `-Werror`**; the linters are strict (uncrustify, cpplint include
  order and 100-char lines, flake8, pep257 — no backslashes in docstrings, yamllint,
  shellcheck).
- **Verify that parameters load**: run the node with its config and `ros2 param get`
  the value. A missing declaration fails silently.
- **ROS CLI across `docker exec` sessions** needs
  `FASTRTPS_DEFAULT_PROFILES_FILE=<ws>/src/fastdds_disable_shm.xml`.
- **Batches**: run them detached (`setsid nohup ... < /dev/null &` inside `docker
  exec`), watch with `until grep -q "the report is in" <log>`. Both runners resume a
  stopped batch and retry at once when gzserver dies. `run_follow_batch` is installed
  as a renamed copy: rebuild after editing it.
- **Streaks of failures inside a batch are not always the environment**: re-run one
  failing seed alone before blaming the box (that is how the yaw ringing was found).
- **Instrument in a scratch copy**, never in the repo, and restore/diff it before
  trusting results.
- **Guards**: a hook blocks writing into another worktree's files (work in the
  session's own worktree); `rm -rf` inside a `bash -c` script is blocked (build into
  a fresh directory instead).

---

## 5. Quick commands

```bash
make up                               # start the container
./scripts/check.sh                    # the CI mirror on the current checkout
./scripts/check.sh <src-dir>          # ... on any source tree
python3 scripts/roadmap.py summary    # the counts
python3 scripts/roadmap.py ready      # what can start
ros2 run interceptor_drone run_follow_batch --runs 4 --scenarios all          # in the container
ros2 run interceptor_drone run_intercept_batch.py --runs-per-scenario 2       # in the container
```
