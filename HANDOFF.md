# HANDOFF — SkyInterceptor, 2026-10-23

A previous session was executing the plan in `IMPLEMENTATION_PLAN.md`, tracking it in
`ROADMAP.md` (the source of truth; `python3 scripts/roadmap.py ready` says what can
start, `summary` gives the counts). **13 of 18 tasks are done and merged.** Three
tasks were in flight and were cut off mid-work by a provider-side API error (HTTP 402,
the model host's free plan ran out) — **not** by anything in the code. Their work has
since been committed, made to build and pass, pushed, and opened as **draft PRs**:

| PR | Task | Branch | State |
|---|---|---|---|
| [#31](https://github.com/OrtizDiego/SkyInterceptor/pull/31) | P2.4 follow scenarios + metrics | `feat/p2-4-follow-scenarios` | builds, 570 tests green, **the batch is not run** |
| [#32](https://github.com/OrtizDiego/SkyInterceptor/pull/32) | P3.2 UAS detector | `feat/p3-2-uas-detector` | builds, 465 tests green, **the training is not done** |
| [#33](https://github.com/OrtizDiego/SkyInterceptor/pull/33) | P3.5 intercept metrics | `feat/p3-5-intercept-scenarios` | builds, 531 tests green, **the runner and the batch are missing** |

Two tasks have no code yet and are specified in `docs/TASKS/`, each on its own branch:
`feat/p2-5-framing-score` and `feat/p4-launch-and-mode-switch`.

**Read this first, then pick up one of the three open tasks.** The rule that matters:
*one task, one branch, one PR; the roadmap section is updated in the same PR.*

---

## 1. What the project is

A ROS2 (Humble) C++17 autonomous drone in Gazebo Classic, with two mission modes.

- **FOLLOW** — an aerial filming drone that follows a person, bicycle or car while
  always keeping a minimum safety distance (`d_min`) from every person or vehicle and
  `d_obstacle_min` from every static obstacle.
- **INTERCEPT** — counter-UAS capture of an intruding small drone (class `uav` only).
  Ground targets are never eligible.

Pipeline: `Perception → Estimation → Guidance → Safety → Control → Platform`.
Planners publish a **velocity** setpoint on `/setpoint/raw`; the **safety filter**
owns `/setpoint/safe` and has the final say; the trajectory controller flies that.

Conventions: ROS2 C++ style (`PascalCase` classes, `camelCase` methods, `snake_case`
variables), `-Wall -Wextra -Wpedantic -O3`, and every algorithm lives in a **ROS-free**
class under `include/<area>/` with the node as a thin wrapper and a gtest under `test/`.
That is the pattern for all five libraries: `common/`, `flight/`, `estimation/`,
`control/`, `guidance/`, `safety/`, `follow/`.

---

## 2. Done and merged (13 tasks, 9 PRs)

`main` is at `0a20389`, builds clean with `-Werror`, **501 tests, 0 failures**.

| PR | Task | What landed |
|---|---|---|
| #23 | P0.1, P0.5 | `scripts/check.sh` + `make check`: the local mirror of CI (roadmap check, python syntax, shellcheck, yamllint, `-Werror` build, `colcon test`) |
| #14, #7, #17 | P0.2–P0.4 | interfaces/mission mode, flyable drone (Gazebo plugin), ground-truth target source |
| #16, #18 | P1.1, P1.2 | IMM-EKF library, `target_tracker_node` |
| #19 | P2.1 | `trajectory_controller_node` |
| #21 | P2.2 | `safety_filter_node` — keep-out barrier, global limits, e-stop, engagement gate |
| #24 | P2.3 | `follow_planner_node` — BEHIND/SIDE/LEAD/ORBIT presets, filtered heading, COAST→HOLD→RTL |
| #25 | P3.1 | five target-drone behaviours: `hover`, `waypoints`, `straight`, `weave`, `curve` |
| #26 | P2.5 | the vision path works (4 bugs fixed, see §5), COCO bicycle class |
| #27 | P3.3 | the terminal capture law; node renamed `guidance_controller_node` → `intercept_guidance_node` |
| #28 | P4a | README/CLAUDE.md/docs rewritten, new `docs/FOLLOW.md`, RViz shows tracks + keep-outs |
| #29 | P3.4 | engagement gate + abort (`safety::EngagementGate`), keep-out changed to a 3D rule |
| #30 | P4b | runtime framing-preset switch, and the filtered setpoint drawn in RViz |

### Two results worth knowing

- **The capture is now flown, not just detected.** The old pursuit law used the
  *measured* closing speed along the LOS, so it had no integral action: against a target
  flying at a constant 2/4/6 m/s the range never changed (20 m → 20 m over 40 s). It
  only worked in the demo because the intruder accelerated from a standstill. The new
  law is `v_M = v_T + u·v_close(r) + a_pn·τ`: the target's velocity as feedforward, a
  pursuer braking curve for the closure with feedback against the measured closing
  speed, and the PN term untouched as the only lateral authority. A gtest sweeps 19
  scenarios (target speed 0–10 m/s, hover/straight/head-on/crossing/weave, range
  5–30 m, 0.3–1.0 s vehicle lag): 19/19 capture, 1.5–3.3 m/s relative at the capture
  (limit 5 m/s). In Gazebo with `groundtruth_seed:=42`: *"Target captured at 1.44 m,
  relative speed 4.38 m/s"*, closest approach 0.36 m.
- **The keep-out is now 3D.** The hard rule is a 3D distance `d_min_3d` (5 m) to every
  person and vehicle; the plan's flat horizontal standoff is released with height above
  the person (`keepout_horizontal_altitude` 5 m, `keepout_horizontal_taper` 1 m per
  metre of climb). Both rules are enforced and the stricter wins. The safety argument:
  whenever the horizontal margin is released, the vertical separation alone already
  exceeds `d_min_3d`, so the distance never drops. This is what makes an INTERCEPT over
  a park with pedestrians possible at all.

### Still open at the roadmap level

`P2.5` is `in_progress` (the vision path works; the *framing score* against the
ground-truth baseline still has to be measured — that needs P2.4's harness).
`P4` is `in_progress` (docs and RViz are done; the launch files and the mode-switch
demo are not).

---

## 3. The three in-flight tasks

All three are **unpushed**. Each has a worktree under `.claude/worktrees/`. Resume in
the worktree, not in the main checkout.

### 3.1 P2.4 — Follow scenarios and metrics recorder

- **Worktree:** `.claude/worktrees/p2-4-follow-scenarios`, branch `feat/p2-4-follow-scenarios`
- **State:** committed and pushed, **PR #31** (draft). The branch is
  `3a71e57` (the scenarios, the recorder and the batch runner) plus `bc3b198` (a merge
  of `origin/main`) plus `b431dc8` (the 3D metric) plus `5ecf422` (the roadmap).
  Builds with `-Werror`, 570 tests, 0 failures.
- **Committed:** `include/follow/follow_scenario.hpp` + `src/follow/follow_scenario.cpp`
  + `follow_scenario_node.cpp` (the four scenarios: walker, jogger, occlusion,
  bystander — the target is a *computed* path, not a baked spline, so a run is
  reproducible from its seed, which 100 randomized runs need);
  `include/metrics/metrics_recorder.hpp` + `.cpp` + `metrics_recorder_node.cpp`;
  `src/tools/run_follow_batch.py`; `test/test_follow_scenario.cpp`;
  `test/test_metrics_recorder.cpp`; `config/follow_scenario_params.yaml`;
  `launch/follow_scenario.launch.py`; `worlds/follow_scenario.world`.
- **What was in flight:** the metrics recorder measured the **horizontal** distance to
  a track against `d_min_horizontal`. P3.4 changed the binding rule to a 3D distance
  (`d_min_3d`, released with height). The uncommitted 4 files are that rework:
  `include/metrics/metrics_recorder.hpp`, `src/metrics/metrics_recorder.cpp`,
  `src/metrics/metrics_recorder_node.cpp`, `test/test_metrics_recorder.cpp`.
  **The metric that decides Phase 2's exit criterion must be the 3D distance against
  the effective threshold** (which grows with height above the person) — otherwise the
  recorder reports margins the filter no longer guarantees. The effective threshold is
  read from `config/safety_params.yaml`: `d_min_3d`, `keepout_horizontal_altitude`,
  `keepout_horizontal_taper`.
- **Also note:** `config/safety_params.yaml` is a merge hotspot — P3.4 added
  parameters to it and P2.4 added obstacles. The merge is already done and **both**
  parameter sets are present; keep it that way.
- **Remaining work:** finish the 3D metric, rebuild, run the batch, update the roadmap
  section, push, open the PR.
- **Exit criteria (plan §5, Phase 2):** minimum distance to every person/vehicle
  `>= d_min` and to every obstacle `>= d_obstacle_min` in 100 % of runs; the target
  within ±15° of boresight ≥ 95 % of the time; the track reacquired after a 2 s
  occlusion. Report **how many runs you actually did** — do not extrapolate.

### 3.2 P3.2 — UAS detector (dataset, fine-tune, export)

- **Worktree:** `.claude/worktrees/p3-2-uas-detector`, branch `feat/p3-2-uas-detector`
- **State:** committed and pushed, **PR #32** (draft). Builds with `-Werror`,
  465 tests, 0 failures. The work is: `scripts/uas_dataset/`
  (`generate_dataset.py`, `train_uav_detector.py`, `export_uav_model.py`,
  `dataset_config.yaml`), `config/uav_detector_params.yaml`, and edits to
  `src/perception/target_detector.py`, `launch/perception.launch.py`,
  `launch/interceptor_full.launch.py`.
- **Artifacts on disk (not in git, and that is correct):**
  - `/tmp/uas_ds3/` — the generated dataset: **653 train + 177 val PNG images** with
    matching YOLO labels, `data.yaml` (single class `uav`) and a `manifest.csv` whose
    columns are `image,split,scene,mode,reason,distance_m,bearing_deg,elevation_deg,box_w,box_h`.
    The `reason` column is how background frames are handled — e.g. `outside_frame` —
    so the negatives are labelled *as negatives*, not as negative boxes. Keep that.
  - `/tmp/uas_train/runs/smoke/weights/best.pt` and `last.pt` — from a **smoke training
    run that crashed during validation** with
    `RuntimeError: DataLoader worker (pid(s) ...) exited unexpectedly`. The
    `results.csv` for that run shows 2 epochs with `precision/recall/mAP` all **0** —
    so the model is not trained, only started.
  - `/tmp/uas_train/yolov8n.pt` — the approved COCO-pretrained checkpoint (the user
    explicitly approved fetching it from Ultralytics' official GitHub release).
  - `/tmp/uas_ds2`, `/tmp/uas_test` — earlier dataset attempts; `uas_ds3` is the good one.
- **The GPU:** the image's torch is `2.14.1+cu130`, which needs a newer driver than the
  host has (555.97, CUDA 12.5), so `torch.cuda.is_available()` is False. A compatible
  build is side-installed at `/opt/torch124` — use it with `PYTHONPATH=/opt/torch124`
  (environment-only; the durable fix is the Dockerfile pin in PR #26). **Say which
  device your training actually used.**
- **Remaining work:** fix the DataLoader crash (workers dying — likely `num_workers`
  or memory; the dataset is small), train for real, evaluate on the held-out split,
  export ONNX (TensorRT if possible), wire the model path and the `uav` class into the
  detector and the tracker, and prove it end to end with `perception_source:=vision`
  (the detector publishing `uav`, the localizer making a 3D detection, the tracker
  confirming a `uav` track).
- **Do not commit the dataset or the weights** — they are build artefacts (`.gitignore`
  already ignores `*.pt`; add an entry for the dataset directory if needed). Commit the
  scripts, the config and the docs.

### 3.3 P3.5 — Intercept scenarios and engagement metrics

- **Worktree:** `.claude/worktrees/p3-5-intercept-scenarios`, branch `feat/p3-5-intercept-scenarios`
- **State:** committed and pushed, **PR #33** (draft). Builds with `-Werror`,
  531 tests, 0 failures. The library is `include/metrics/intercept_metrics.hpp`,
  `src/metrics/intercept_metrics.cpp`, `src/metrics/intercept_metrics_node.cpp`
  and `test/test_intercept_metrics.cpp`; the CMake entries were added by hand when
  the branch was picked up again. The scorer reproduces
  the guidance's own capture condition (range inside the capture radius **and** relative
  speed under the limit), reports the weaker *envelope* separately, latches the abort
  reason, and counts non-`uav` engagements (the criteria demand zero). Records are CSV,
  one row per run, so a batch can aggregate them.
- **The agent's last words, and the thing you must deal with first:**
  > *"I have a clear diagnosis. The engagement gate deadlocks take-off."*
  and
  > *"Now let me wire the mission mode into the launch"*
- **What that means.** P3.4's gate requires `min_engage_altitude_agl = 10.0` m AGL
  before an engagement is allowed. The interceptor starts on the ground. If the guidance
  only engages — and therefore only climbs — once the gate is open, the drone never
  takes off and the gate never opens: a deadlock. The gate's altitude condition was
  meant as a *capture* condition, not a *climb* condition. **Resolve this properly and
  say which way you resolved it and why.** The safety reading is that the interceptor
  must not be low and slow near an intruder it is about to chase; the wrong fix is to
  simply drop the altitude condition to make the runs work. Look at
  `include/safety/engagement_gate.hpp` and `src/guidance/intercept_guidance_node.cpp`
  and decide whether the condition belongs on the *engagement* (closing) rather than on
  the *gate* (authorisation), or whether the drone is allowed to climb before it is
  authorised to engage.
- **The other tension, stated in the original brief:** with a 15 m abort radius the
  abort also fires on *ordinary* capture runs whenever the intruder flies over the
  pedestrian's patrol square, which would collapse the capture rates. The criteria are
  "the capture still happens" and "the abort fires when it should" — **not** "the abort
  never fires". Do not weaken the abort to make the rates look good. Place the
  scenarios so the bystander is out of the way of the capture runs and in the way of
  the bystander run, and/or make the abort radius per-scenario configurable. If a
  layout makes both impossible at once, report the trade-off with numbers.
- **Run artifacts:** `/tmp/p35logs/` (`exp1.log`, `exp2.log`, `pilot_hover.log` — a
  2.7 MB run log) and the build workspace `/tmp/p35-1792675291`. `exp2.log` shows the
  mission arming, the guidance starting the engagement at 13.8 m range and the drone
  arming — and then nothing further, which is consistent with the deadlock above.
- **Remaining work:** resolve the deadlock, build the scenario runner and the batch
  aggregator, run the batch, report the numbers, update the roadmap section, push, PR.
- **Exit criteria (plan §5, Phase 3):** capture envelope reached in ≥ 90 % of hover,
  ≥ 80 % of straight and ≥ 60 % of weave runs; **zero** engagements of non-`uav`
  tracks; abort fires in 100 % of bystander runs. Report **how many runs you did**.

### 3.4 After those three

- **P2.5 (finish)** — measure the framing score of the vision path against the
  ground-truth baseline (≥ 90 % of it). Needs P2.4's metrics harness. The vision path
  itself already works.
- **P4 (finish)** — the launch files and the mode-switch demo. Needs P2.4's
  `follow_scenario.launch.py` and P3.5's runner.

---

## 4. The merge policy (please follow it)

- One task, one branch, one PR. Branches: `feat/p<N>-<short-name>`.
- **The user has approved merging green PRs to `main` yourself**, as they come in
  green. Merge with `--squash`. CI must be green first.
- The roadmap section is updated **in the same PR**: `Status: review` + `PR: <number>`
  + `Branch: <branch>` + `Notes:` saying what you verified and what is still open.
  The `PR` field must be a single number or `—` (the validator rejects `28, 30`).
  A status describes the state *after* the PR merges, so `done` is written before the
  merge.
- `python3 scripts/roadmap.py check` must pass (CI runs it).
- Merging a branch that was based on an older `main` will conflict in `ROADMAP.md` and
  sometimes in `config/safety_params.yaml` and the launch files. Resolve by keeping
  **both** sides' changes, and re-run the build and the tests after resolving.
- When two branches touch the same `include/metrics/` area (P2.4 and P3.5 both do),
  keep them self-contained and expect to rename on conflict — P3.5's note says the
  same.

---

## 5. Environment notes (learned the hard way — read before running anything)

- **The container's `src` is bind-mounted from wherever `docker-compose` was started**,
  which is a *worktree*, not `main`. A build in `/workspace/interceptor_ws` therefore
  tests the wrong tree. Check with
  `docker exec interceptor findmnt -T /workspace/interceptor_ws/src -o TARGET,SOURCE`.
  **Use `./scripts/check.sh <src-dir>`** instead: it copies the given source into a
  scratch workspace inside the container, builds with `-Werror` and runs the tests, and
  never touches the shared build volumes. It takes a few minutes.
- **CI builds with `-Werror`.** A green local build without it is not enough.
- **The linters are strict.** `ament_uncrustify --reformat` in the scratch workspace,
  then copy the files back. cpplint wants lines ≤ 100 chars. Python files are linted
  by `flake8`/`pep257` (complete docstrings). `yamllint --strict` over the tracked
  YAML (no aligned columns, no trailing spaces). `shellcheck --severity=warning` over
  the tracked `*.sh` — **a comment that starts with the word `shellcheck` is parsed as
  a shellcheck directive and fails the build** (that cost one CI cycle on PR #23).
- **Verify parameters actually load.** `has_parameter()` is false for a value that
  comes from a params file and was never declared, so a missing declaration silently
  yields an empty parameter list. Run the node with its config file and read the
  startup log.
- **ROS CLI across separate `docker exec` sessions** needs
  `FASTRTPS_DEFAULT_PROFILES_FILE=<ws>/src/fastdds_disable_shm.xml`, or you get
  phantom node-discovery failures.
- **Use your own `ROS_DOMAIN_ID` and `GAZEBO_MASTER_URI` port.** The container has ~80
  stale `robot_state_publisher` processes from previous sessions that publish the same
  TF frame names; a different domain insulates you. **Do not kill processes that are
  not yours** — the permission classifier blocks it and it is not your call.
- `gzserver`'s model-database download can outlast a 30 s spawn timeout on a cold
  start. Give it time.
- The `interceptor` container image has **not** been rebuilt since the Dockerfile pins
  in PR #26, so the image still has NumPy 2 and torch cu130. The code-level fixes mean
  the code works anyway.
- **A provider-side HTTP 402 killed the previous session's agents mid-task.** If an
  agent dies with that error, the worktree still holds its work — resume from there.

---

## 6. Quick commands

```bash
make up                       # start the container
./scripts/check.sh            # the CI mirror, on the current checkout
./scripts/check.sh <src-dir>  # ... on any other source tree (a worktree)
python3 scripts/roadmap.py summary   # the counts
python3 scripts/roadmap.py ready     # the tasks whose dependencies are all done
gh pr list --state open
```

Inside the container, a scratch build:

```bash
WS=/tmp/<uniq>-$(date +%s)
docker exec interceptor mkdir -p $WS/src
docker cp <worktree>/interceptor_ws/src/. interceptor:$WS/src/   # note the trailing /. — without it colcon
                                                                   # reports duplicate package names
docker exec interceptor bash -lc "cd $WS && source /opt/ros/humble/setup.bash && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror"
docker exec interceptor bash -lc "cd $WS && source /opt/ros/humble/setup.bash && \
  source install/setup.bash && colcon test && colcon test-result --verbose"
```
