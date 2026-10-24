# Task: P2.4 (finish) — the jogger and the corner problem

**Roadmap:** `ROADMAP.md` → `### P2.4 – Follow scenarios and metrics` (stays `in_progress` until this is done).
**Branch:** `feat/p2-4-jogger-corners`.
**Depends on:** the P2.4 harness (PR #31, merged) and the safety fixes #37–#41 (merged).

## What fails

In the last P2.4 batch (2026-10-24, main with #37–#41) the jogger scenario
failed **all 25 runs**:

- the **horizontal standoff** (`d_min_horizontal`, released with height) broke in
  every run, by 1.5–2.3 m. The 3D distance to the jogger held in every run
  (worst margin +0.56 m), so the hard rule was never broken;
- the obstacle margin went to −0.34 m in one run;
- framing 65–88 %, median 81 % (criterion ≥ 95 %). 21 % of the scored time had
  the target out of the ±15° frame, almost all of it with the target tracked and
  the safety filter **not** holding (12 of 5308 lost samples were during a hold).

The other three scenarios pass (walker 25/25, bystander 24/25, occlusion 24/25).

## Why (the diagnosis so far)

The jogger runs a rectangular loop at 4 m/s with 2.5 m corner radii (1.6 rad/s,
6.4 m/s² lateral). The follow planner's `BEHIND` preset puts the framing point
8 m behind the jogger along its **filtered heading**. At a corner the heading
swings 90° in about a second, the framing point swings with it, and the drone —
which has a velocity loop with a ~0.34 s lag and flies a velocity feed-forward of
the target's own velocity — ends up **inside the corner**, so the jogger comes
round it running straight at the drone. The safety filter's barrier is reactive:
it can only push the drone away once the jogger is already closing, and with the
lag the horizontal margin is eaten before the escape takes effect. The same turns
cost the framing.

Logged evidence (from the first P2.4 investigation): the horizontal margin falls
from +1.0 m to −2.1 m in 0.8 s while the filter reports `keep-out from track 1
(horizontal)` and is **not** holding — the filter is doing its job; the planner
put the drone in the way.

## What to do

The fix belongs in the **follow planner**, not in the safety filter.

1. Reproduce one jogger run with the time series (`run_follow_batch --runs 1
   --scenarios jogger --seed-base 2`) and plot the drone, the framing point and
   the jogger around a corner. Confirm the drone is inside the corner when the
   jogger comes round.
2. Candidate changes, to evaluate in a ROS-free closed loop first (the follow
   planner library + a lag plant + the safety filter, like the safety filter's
   closed-loop gtests), then in Gazebo:
   - **predict the target's path**: keep the framing point clear of where the
     target will be in the next ~1-2 s (its predicted position along the CT/CV
     estimate), not only where it is now;
   - **limit how fast the framing point may swing** with the heading, or place it
     on the outside of the turn (the turn rate's sign is in the track);
   - **add a planner-level standoff**: never command a framing point inside
     `d_min_horizontal` + margin of the target's predicted path, so the barrier is
     a backstop and not the steering.
3. Keep the walker, bystander and occlusion results: they pass now and must still
   pass.

## Done when

- jogger: distances OK and framing ≥ 95 % in the 25-run scenario, or a clear,
  measured statement of what blocks it;
- walker, bystander and occlusion re-run (25 each) and still passing;
- `ROADMAP.md` P2.4 updated (status `done` only if all four scenarios pass), and
  the number of runs and of retried gzserver crashes reported exactly.

## Pitfalls

- **Do not re-run whole batches on a degraded box**: gzserver segfaults on
  ~15–20 % of attempts (its own task: docs/TASKS/gazebo-crashes.md). The runners
  retry at once; count the retries and report them.
- The `run_follow_batch` script is installed as a renamed copy: rebuild (or copy
  it over) after editing it.
- The intercept guidance shares the controller's yaw gains and the bearing-rate
  feed-forward (#41): a planner change that touches the yaw must be re-checked on
  the intercept scenarios too.
