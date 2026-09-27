# H64 — local recovery of simultaneous speed violations

H63's +π/3 history first violates speed bounds at tick 269. Its angular stage
retains the uncorrected baseline: foot angular speed is 1.002 rad/s and another
body reaches 0.670 rad/s. Separate stages reject every trial that leaves any
other speed violation. Test whether a coupled search can recover a feasible
command using the same 34 existing leg and arm motor axes.

Use the earliest post-settling failed native snapshot from each failed H63
heading, with the exact original baseline and retained prediction replayed.
For every body's linear/angular velocity vector, form the vector excess above
0.08 m/s or 0.4 rad/s, normalized by the independent acceptance bounds of
0.1 m/s and 0.5 rad/s. Minimize the squared residual across all 25 bodies.
These reserve targets do not alter any physical acceptance threshold.

Use eight iterations, ±1 Nm probes, native-cap column normalization and trust
scaling, the same six backtracking fractions, and best improving probe retention.
Intermediate copied trials may violate speed bounds; accept only finite trials
that reduce this violation objective. Never apply those trials to a live world.
Report whether the final candidate meets all original speed guards and whether
the stricter reserve target is reached; failure remains failure.

Preserve original snapshots, all trial body states and metrics, native motor
configuration checks, unchanged caps, source fingerprints and invocation source.
Do not modify the solver, timestep, collisions, joint limits, ownership or
production controller. This local comparison cannot establish standing or
position-feedback stability and authorizes no automatic production adoption.

## Result — local reserve search rejected

All three original baseline/retained states and chosen repeats replay exactly.
The 1,769 queries keep the original motor caps. None reaches the complete
declared speed guard or reserve target. Heading 0 reaches the independent
physical limits (0.0912 m/s, 0.4718 rad/s), but foot angular speed remains
0.400186 rad/s against the search guard of 0.4. The other two retain physical
failures: +π/3 angular speed 0.7224 rad/s and −π/4 linear speed 0.1162 m/s.

At +π/3 every retained iteration is the same −1 Nm ankle probe; eight such
updates reduce the objective monotonically, while the coupled calculated
updates are rejected. This identifies a concrete step-size limitation in the
retained coordinate direction, without establishing that a larger step succeeds.

The [fourteen-artifact archive](checkpoints/2026-09-27/h64-coupled-speed-authority/manifest.json)
is verified and retains all queries, native snapshots, motor configurations and
the exact invocation source.
