# H33 — isolate the optional native contact block solver

The pinned f32 engine is built without its optional `block-solver` feature.
The earlier precision comparison deliberately aligned f64 to that same feature
set. H33 isolates this implementation choice in the independent fixed-assembly
fixture; it does not change the application's frozen solver configuration.

Build two native f32 executables from identical Rapier 0.35.0 / Parry 0.30.2
source. Enable `block-solver` in exactly one. Verify resolved Cargo features
for every dependency: only the executable's forwarding feature and Rapier's
feature may differ. Record build commands, outputs and executable digests.

Replay the same heading-zero tick-one snapshot with unchanged integration
parameters, collision filtering and full state. Require the normal executable
to reproduce all prior native f32 samples exactly, and both initial measured
states and parameter records to match. Compare the twelve-second trajectories
and pre-collapse samples. A failure rules out this isolated implementation
choice as a repair for the rigid fixture. Even success would only justify
further diagnosis: no runtime, solver feature, actuator, collider, threshold
or production source is changed or accepted by this experiment.

The block solver is not a repair for the rigid fixture. Its pelvis reaches
0.560721 m height and −0.845153 m forward position at native tick 120, versus
0.781448/−0.632239 m for the normal build. Both topple. All 23 normal body-state
samples replay exactly, both initial states and parameter records match, and
the complete resolved dependency graph differs only in the declared feature.
The default native output was rebuilt without that feature and its digest
again matches the separately preserved normal executable.

Evidence: [verified H33 archive](checkpoints/2026-09-27/h33-block-calibration/manifest.json).
