# H56 — simultaneous foot and pelvis motion task

H55's feet-only objective leaves pelvis motion uncontrolled. Test a fourteen-
component task through the same fourteen native leg motor axes: eight actual
foot-centre horizontal displacement rates and the pelvis's three displacement
rates plus three rotation-vector rates. Normalize linear components by the
unchanged 0.1 m/s acceptance bound and angular components by 0.5 rad/s. Derive
angular displacement from normalized quaternion differences using atan2, without
the production telemetry helper's small-angle deadband or speed clipping.

First compare this task with the feet-only objective on the same three H49
tick-350 native snapshots, starting from the identical H50 bias. Preserve
exact baseline and retained 25-body replay checks, native motor configuration
inspection, ±1 Nm central probes, eight iterations, dimensionless motor-cap
scaling, guarded line search and computed-probe retention. No physical settings,
actuator ceilings, production source or acceptance thresholds change.

Use task stopping bounds of 0.2 mm/s maximum foot horizontal rate, 0.01 m/s
pelvis displacement rate and 0.05 rad/s pelvis rotation rate. These are search
targets, not substitutions for official drift/speed acceptance. Keep all-body
actual linear ≤0.1 m/s and angular ≤0.5 rad/s plus foot angular ≤0.4 rad/s
as candidate guards. Retain all trials and report trade-offs when the task
cannot meet these targets. A local result alone authorizes no standing claim.

## First result and bounded-task comparison

The zero-target task reaches all stopping bounds only at −π/4. The other
headings retain 0.645/1.280 mm/s maximum foot rate while pelvis motion is
already well inside its stopping bounds. All candidates remain within the
actual speed guards. The [29-artifact archive](checkpoints/2026-09-27/h56-pelvis-authority/manifest.json)
retains 908 task queries and a byte-exact replay of all 488 H54 control queries.
All 67 production/package files remain identical to H51; four diagnostic
mathematical tests and targeted lint pass.

Compare a bounded pelvis residual next: zero inside the already-declared
0.01 m/s and 0.05 rad/s pelvis norm bounds; outside, subtract the closest
point on that bound's sphere. Keep the feet residual, all scales, search
limits and physical guards identical. This explicitly tests whether pursuing
zero pelvis motion after satisfying the intended bound unnecessarily competes
with the foot objective. Record both variants separately and retain the
initial failed result; no acceptance or stopping threshold changes.

The bounded variant meets all three declared task stopping bounds with fourteen
controls, using 125/31/91 queries. Maximum foot rates are
0.0132/0.0133/0.0383 mm/s; pelvis rates remain below 0.01 m/s and 0.05 rad/s.
The eight-control comparison still fails at all headings. All 719 queries are
retained in the [15-artifact archive](checkpoints/2026-09-27/h56-bounded-pelvis-authority/manifest.json).
Five diagnostic mathematical tests and targeted lint pass. This supports the
next continuous evaluation, not production adoption.
