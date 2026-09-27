# H57 — continuous bounded pelvis and foot task

Apply H56's bounded pelvis residual in H55's continuous fourteen-axis preview.
Keep the six-distal-axis angular stage and its trigger/reserve unchanged. The
second stage minimizes eight foot-centre horizontal pose rates plus six pelvis
pose-rate excess components outside 0.01 m/s and 0.05 rad/s norm bounds.
Normalize linear components by 0.1 m/s and angular by 0.5 rad/s; normalize
controls by their original native torque caps. Keep eight iterations, ±1 Nm
probes, one-cap trust scaling, six backtracking fractions and improving feasible
probe retention. Reset additive bias to zero each step.

Start at initialization under the same H22 held-reference, quiet-arm, pressure-
contour and world-damping conditions. Require measured loaded support before
the combined task; retain exact 25-body copied/live matching on every eligible
step. No native state transfer, new actuator, physical setting or production
change is allowed. Record all candidate metrics/digests, motor parameters,
chosen live bodies and checkpoint/failure snapshots as in H55.

Evaluate 2 s settling plus 10 s observation at all three original headings.
Keep every official drift, speed, support and structural bound unchanged.
Advance to 2+30 s only after every short screen passes. Report misses, runtime,
maximum excursions and vertical ranges. A passed screen alone does not establish
interactive performance or production acceptance.

## Observations while the full screen is still running

The screen is already rejected at −π/4 tick 135 and heading zero tick 329.
The former retains a baseline forefoot peak of 1.378959 rad/s and 0.121146 m/s;
the latter reaches its sixteen angular-iteration ceiling at 0.571874 rad/s.
Both use the original independent acceptance limits. These are recorded
failures, not complete 720-step results. The runs continue unchanged to retain
their full requested observation windows; no long evaluation is authorized.

[H58](standing-h58-contract.md) replays each failure and establishes a local
full-step search correction within the original motor ceilings. That separate
copied-state experiment does not alter these live histories. The old H55 mode
also reproduces all first thirty native states, physical responses and chosen
prediction hashes at each heading under the new helper (ninety exact steps).

## Completed result — rejected at all headings

All three 720-step runs completed upright with two planted feet, zero steps
and no observed support loss. Endpoint foot drift is 9.562/7.482/8.989 mm;
pelvis drift is 4.375/3.432/2.823 mm. Peak linear speeds are
0.099982/0.143154/0.121146 m/s and angular speeds are
0.571874/0.608882/1.378959 rad/s. There are 1/51/1 observed steps above the
speed bounds after settling. No 30 s evaluation follows.

All 2,160 chosen 25-body predictions exactly match the live result; all
425,534 copied cases preserve the native motor caps. Native inspection checks
294 motor-axis configurations across 21 checkpoint snapshots. One serialized
JS zero loses the sign of native negative zero; this verifier-only discrepancy
is explicitly retained with the failed verification attempt and original native
receipt. All numerical parameters match at f32 precision, and no physical
acceptance assertion changes.

Average preview time is 1,473/1,378/1,570 ms per step in these concurrent runs,
excluding evidence writes. Maximum joint separation stays below 0.048 mm;
self penetration is zero; floor penetration maxima are 2.860/2.896/3.733 mm.
Full excursion and vertical measures, trial logs, snapshots and source are in
the [H57 archive](checkpoints/2026-09-27/h57-pelvis-preview/manifest.json).
All 67 production/package files remain identical to H51.
