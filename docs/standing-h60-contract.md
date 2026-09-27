# H60 — local arm-motor authority for hand-speed failures

H59's −π/4 tick 123 predicts left/right hand speeds of 0.112828/0.107047 m/s,
while its foot speeds are bounded. H57's late +π/3 tick 705 is a second distinct
hand-speed failure. Test whether the twenty already-existing arm motor axes
can reduce these motions without disturbing the accepted lower-body bounds.
No motor, physical range, torque ceiling or live-state replacement is added.

Read exact native f32 targets, gains, enabled axes, force-based models and caps
from each original snapshot. Verify joint handles/parents/children and profile
caps. Verify native payload bytes and island-set membership using the existing
reader; restore the original binary for each JS query. The original baseline
must reproduce all twenty-five saved bodies exactly before changing a motor.

Minimize the ten arm segments' sixty actual linear/angular velocity components,
normalized by the unchanged 0.1 m/s and 0.5 rad/s acceptance bounds. Use ±1 Nm
central probes, motor-cap normalization, one-cap trust scaling, six backtracking
fractions, improving feasible-probe retention and at most eight iterations.
Keep all actual body speed guards and foot angular ≤0.4 rad/s during candidate
selection. Stop when arm linear speeds are ≤0.08 m/s and angular speeds ≤0.4
rad/s. These stricter search reserves do not replace official acceptance.

Retain every trial, original native state, motor inspection and standalone
invocation source. The same predeclared method applies to both states. Results
remain copied-state authority evidence; they do not alter the ongoing H59
histories or prove continuous stability, drift bounds or interactive cost.

## Result — both retained arm failures have local authority

The same first Newton update succeeds in both states, using 42 queries each
including an exact 25-body baseline replay and forty central probes. Maximum
arm linear speeds become 0.021733/0.036118 m/s and angular speeds
0.101833/0.126990 rad/s. Whole-body maxima remain ≤0.039970 m/s and
≤0.275948 rad/s; maximum foot angular speed stays ≤0.096030 rad/s.

All twenty arm axes use exact original native f32 targets, gains and force
ceilings, with verified handles, profiles and serialized payload. The
[ten-artifact archive](checkpoints/2026-09-27/h60-arm-authority/manifest.json)
retains both native states, motor receipts, all eighty-four queries and the
standalone invocation source. Targeted lint passes. These local corrections
support a continuous arm-stage test but do not close any physical gate.
