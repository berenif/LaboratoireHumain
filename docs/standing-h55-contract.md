# H55 — continuous full-leg horizontal preview

H54 reaches the local horizontal deadband at all three retained states with
fourteen original leg motor axes. Extend H49's horizontal stage to those
axes using H54's dimensionless, guarded search and eight-iteration ceiling.
Preserve H49's separate six-distal-axis angular stage, including its original
trigger/reserve, probe radius and update rule. No actuator, physical setting
or acceptance threshold changes. Start additive bias from zero each tick.

Run the same H22 idle-reference, quiet-arm, pressure-contour and world-damping
conditions from initialization. Horizontal correction still requires measured
loaded support on both sides. Every chosen 25-body prediction must exactly
match the live step. Native motor ceilings stay unchanged, including all
three hip axes' documented aggregate vector bound. No body state is copied
into the live world. Base request telemetry identifies overrides separately.

First require all original bounds in a three-heading 2+10 s screen, then
advance to 2+30 s only after every short screen passes. Record maximum
excursions, vertical ranges, misses, saturation/probe decisions and runtime.
An infeasible stage retains its best available measured candidate and records
the miss; this never changes the independent acceptance threshold.

Use compact durable evidence: every step records all trial metrics, biases,
predicted-body digests, current motor configurations and the chosen/live
25-body result in a JSONL log. Keep native snapshots at tick 1 and each
120-tick checkpoint, plus any actual speed-bound failure. The initial source
and deterministic full run remain the route to reconstructing arbitrary
intermediate native states; do not imply every step has a standalone snapshot.
The report carries log row counts and digests. This replaces redundant
per-trial body copies and per-tick binaries, not verification coverage.

Earlier preview modes retain their physical arithmetic and evidence format.
Correct their held-reference diagnostic reason labels to match the already
applied behavior. No production integration or interactive-performance claim
follows merely from launching or passing this diagnostic.

## Result — rejected for whole-body speeds

All three 2+10 s screens fail. At headings 0, +π/3 and −π/4, peak linear
speeds are 0.327420/0.304006/0.217229 m/s and angular speeds are
1.396746/1.173186/1.572519 rad/s. Endpoint foot drift is
8.418/8.297/8.246 mm; pelvis drift is 3.665/2.752/6.777 mm. All finish
upright with two planted feet and zero steps. No 30 s evaluation follows.

Every one of 2,160 live steps matches its chosen 25-body prediction exactly.
The 142,848 copied queries respect the original motor ceilings. Average
preview calculation costs 424/387/456 ms per step, excluding evidence writes.
The feet-only objective permits pelvis motion, followed by excessive hand
speeds. Exact one-step prediction and local foot authority do not establish
stability of the coupled body.

The [1,511-artifact archive](checkpoints/2026-09-27/h55-full-leg-preview/manifest.json)
preserves all trial metrics/digests, chosen live bodies, native checkpoints,
speed-failure snapshots, excursion/vertical/structural measures and source.
The earlier six-axis mode reproduces all native snapshots, physical responses
and chosen body digests for the first 30 steps at each heading. The reason-label
correction changes no physical arithmetic in that compatibility check.
