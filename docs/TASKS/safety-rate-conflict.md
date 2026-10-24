# Task: the safety filter's rate conflict — finish it, or propose a design first

**Roadmap:** P2.2 follow-up (safety filter). **Its own PR**, or a design proposal and a
go-ahead from the user before any code.
**Branch:** `fix/safety-rate-conflict`.

## Where it stands (read this first: most of it already exists)

PR **#40 (merged)** already implements the core of what was asked:

- when the exact projection (#38) is infeasible, the rate limits of the **least
  urgent barriers** — largest distance margin `h` first — are relaxed **one at a
  time** until the rest can be satisfied;
- only barriers with `h >= relax_min_margin` are eligible (**2.0 m** today,
  `safety.relax_min_margin` in `config/safety_params.yaml`);
- the reason names what was relaxed (`...; rate relaxed for obstacle oak`);
- with nothing eligible the drone holds, as before;
- gtests: the logged jogger-against-oak scene (one step), a close keep-out that is
  never relaxed, and a closed loop with the jogger 0.5-2 m off the line through the
  drone and the tree.

What the user's request adds, and is **not done yet**:

1. **The threshold**: the request says `h > 1 m`; #40 uses 2 m. Justify the choice
   (at 15 m/s and 50 Hz the drone covers 0.3 m per cycle; the next cycle enforces a
   relaxed barrier again) or change it, and say which.
2. **Yaw during a hold**: today a hold freezes the yaw at the current heading
   (`SafetyFilter::hold()` sets `yaw = state.yaw`, `yaw_rate = 0`). Keep it tracking
   the planner's yaw setpoint (and the #41 bearing-rate feed-forward) instead, so
   the camera keeps the target in frame while the drone holds. A hold is about the
   *velocity*; the heading does not move the drone. Check the e-stop and the abort:
   do those also keep the yaw, or should they freeze it?
3. **A randomized property test** that the 3D and the horizontal distances never
   drop below their limits *with the relaxation in play*: random scenes with
   several people (some moving at the drone), trees and benches, a lag plant, and
   a planner that commands into the keep-outs. The existing randomized property
   test predates the relaxation.
4. Then **re-run the jogger and occlusion scenarios, 10 runs each**, and report
   (runs, crash retries, framing, distance_ok, reacquire_ok, passed — exactly, no
   extrapolation).

## Known limitation to keep in mind

A barrier constrains the rate along its own normal, so it cannot see a sideways
escape. With the jogger, the drone and the tree's centre exactly in line, the drone
backs towards the tree until the tree is too close to relax, then holds, and the
jogger gets to 4.7 m (limit 5.0) in the closed-loop test (documented in
docs/SAFETY.md). Do not hide that case; if the design fixes it, show it.

## Option: propose a design first

The user allowed either implementing the above or **proposing a design for the
rate conflict and waiting for a go-ahead**. If the threshold or the hold-yaw
change raises a safety question (it changes what "hold" means), propose first:
what changes, the safety argument, the tests that prove it, and what is still not
covered.

## Pitfalls

- The jogger's main failure is the **follow planner** putting the drone in the
  jogger's path (docs/TASKS/p2-4-jogger-corners.md): the filter fixes here will not
  make the jogger pass on their own. Report the jogger numbers anyway.
- `ABoxedInDroneHoldsInsteadOfRelaxingADistance` and the other existing safety
  tests must still pass: a distance is never relaxed, only a rate.
