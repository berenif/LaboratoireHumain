# H54 — guarded search and full-leg local authority

H53's optimizer can discard a feasible improving probe when its Newton line
search fails. Correct that search behavior, then compare the eight existing
single-axis controls with all fourteen leg motor axes (add both hips' X/Y/Z).
This tests proximal contribution rather than assuming the remaining motion
can be cancelled through distal joints alone.

Keep the same snapshots, H50 starting bias, dimensionless residual/control
scales, ±1 Nm probes, eight-iteration budget, native ceilings and physical
guards. Scale each Newton direction uniformly when its largest increment
exceeds one original torque-ceiling fraction, before backtracking and clipping
the accumulated bias. This preserves direction while bounding the initial
numerical proposal. Among already computed probes and the accepted line
candidate, retain the lowest feasible objective; stop only when none improves.
Preserve the earlier dimensional and dimensionless methods for exact replay.

Read added hip-axis targets, gains, models and ceilings from the original
snapshot, verifying handles, enabled axes and profile ceilings. Native
per-axis limits and their documented aggregate vector cap are unchanged.
Every baseline and retained 25-body reference must match exactly, and every
trial starts from the unchanged snapshot with original collision hooks.
Require the same 0.2 mm/s local horizontal deadband and actual speed guards.
This is still a one-state actuation/search comparison, not standing acceptance.

## Result — full-leg local authority verified

All fourteen leg axes reach the 0.2 mm/s deadband at all three headings in
one Newton iteration: maximum rates are 0.0138/0.0133/0.0819 mm/s. Each uses
31 copied queries including two exact 25-body replay checks. Speed guards
pass. The guarded eight-axis search reaches the deadband only at heading zero;
it retains 1.579/0.434 mm/s at the other headings.

The [15-artifact archive](checkpoints/2026-09-27/h54-full-leg-authority/manifest.json)
retains all 488 queries, native motor receipts and source. These results
justify a continuous full-leg preview test; they do not prove stability over
time or computational suitability for production.
