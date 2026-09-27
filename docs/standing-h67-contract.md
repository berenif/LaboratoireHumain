# H67 — local reuse of the previous motor correction

H66's copied-world search is still too costly for an interactive controller.
While its unchanged continuous screens run, test whether the immediately
previous motor correction is a useful starting point at the three tick-240
native checkpoints. This is a local performance investigation, not a substitute
for the ongoing standing evaluation.

Replay the saved zero-bias baseline and chosen H66 correction exactly. From
the same pre-step state, compare the preceding tick's torque bias clipped to
the current native ceilings, and its preceding cap-normalized fraction mapped
to the current ceilings. Keep every original target, gain, ceiling, timestep,
collision rule and physical state. Apply candidates only to copied worlds.

Measure actual whole-body speeds, foot angular speeds, initial-reference
position error, the H66 horizontal tracking and bounded pelvis task, and the
time spent restoring, stepping and reading each world. Preserve native motor
inspection, both source rows, initialization references, all candidate body
states, commands and invocation source. A candidate that misses speed or task
bounds remains a miss. No controller change or continuous warm-start claim
follows from these three isolated snapshots.

## Result — bounded seeds, incomplete task correction

Both previous-command variants are identical here because the motor ceilings
did not change between ticks 239 and 240. They satisfy all physical speed
limits and the 0.4 rad/s foot search guard at all three headings. Their task
objectives are lower than the uncorrected baseline in every state, but improve
on the selected H66 command only at heading 0. No candidate reaches the full
tracking/pelvis deadband. Thus this is evidence for a starting point, not for
replacing the search with the previous command.

All three saved baselines and chosen 25-body states replay exactly; all 102
native motor-axis configurations match. Twelve complete copied queries are
retained. Measured warm-copy costs are roughly 9–16 ms while the three H66
histories run concurrently; this is a small local timing sample, not a frame-rate
benchmark. The first derived-metric comparison failed because addition and
subtraction were reassociated. Matching the original parentheses repairs that
comparison without relaxing its exact assertion; the failed source and receipt
are preserved in the [verified nineteen-artifact archive](checkpoints/2026-09-27/h67-previous-correction/manifest.json).
