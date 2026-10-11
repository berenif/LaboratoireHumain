# Native coupled control, 2026-10-11

Local branch `investigate/physical-support`, base and HEAD
`c597959f46f37715035349c8cd0a699b6822111a`. No push, merge or release.
Recovery remains disabled by default and unqualified. This is a test-only
research harness, not a production controller or a recovery acceptance result.

## Implemented and verified

`recovery/native_predictor.rs` copies the persistent Rapier world directly,
including contacts, islands and solver state, and recreates only the stateless
pipeline. Predictions integrate a separate world. No predicted body state is
transplanted into the real simulation. The archived loaded prefix agrees
bit-for-bit on position, rotation and both velocities of all 25 bodies;
fixed-command four-step predictions also agree at all four actual steps.

The archived opt-in test controller identifies 21 coupled motor response columns using
native four-step rollouts. A bounded active-set box QP proposes target-velocity
corrections; fresh native predictions must pass the original measurement guards
at every intermediate state. Search is limited to 28 evaluations and 112 native
integrated steps; the typical queries use23 evaluations /92 steps; final trajectory peaks at26/104.
Commands preserve force-model, gains, anatomical caps and the strict 760 Nm
aggregate budget. A downward-only f32 rounding repair enforces that budget
without widening any limit. Motor target positions are clamped at the original
anatomical command endpoints, not by relaxing measured-state limits.

`measurement::inspect_world` exposes the original guards for predicted worlds.
The real-state wrapper retains diagnostic updates even on refusal. The rejected controller and its test hook have been removed from live source.
The direct predictor, original guard extraction and read-only trace harness
remain. Archived source and binary pairs reproduce the rejected experiments.

All selected first predictions agree bit-for-bit with real integration of all
25 bodies. The four-step horizon assumes constant commands; replanning after
the first actual step does not imply that the remaining three predictions are
the realized continuation. Geometry/limit checks do not certify future capture
or balance. Original bilateral support-loss persistence stops the feet-only
trial after 0.2 s without measured support. Stopping a diagnostic rollout does
not stabilize the body and does not qualify recovery.

## Structural command-ownership experiment

Native corrections were initially placed on top of another controller's stiff
position targets. At 0.5 s that controller commanded hip about 0 and knee
0.244 rad while the low-pose plan requested 0.827 and 0.917 rad. The final
experiment gives the native planner ownership of both position and velocity
targets, preserving the previously allocated gains and caps.

At 0.5 s this changes measured pelvis Z from +0.1151 to -0.0738 m, pelvis Y
from 0.9471 to 0.9097 m, forward COM speed from 0.3724 to 0.3240 m/s and
forward COM acceleration from 3.4994 to 1.5293 m/s². Both hindfeet and forefeet
carry actual resolved load. Hip/knee move to approximately 0.949/0.910 rad,
but the capture point remains outside the actual support region. No stable
transfer or rise is demonstrated by these short trajectories.

The allocation still comes from the previous controller's requests. In the
owned-target trial at 0.5 s the hip spring request is about -106 Nm while its
cap is 69 Nm and delivered magnitude is about 68 Nm; knee cap is 76 Nm with
about 13 Nm delivered. These are distinct diagnostic quantities: spring
request and raw constraint-row impulse are not interchangeable with a
physical delivered generalized torque. Recomputing allocation consistently
with the new owner is an identified follow-up, not implemented or validated.
The observed failure does not prove feet-only recovery impossible or establish
that hand/knee support is necessary.

## Results and cost

The exact-prefix intervention brakes the forefoot over several real steps,
not merely in an inverse-dynamics model. The old one-step long attempt falls;
it was deliberately interrupted after 7.2 physical seconds and its report is
explicitly marked interrupted. The later four-step controller with concurrent
position targets stops at 1.3667 s on the unchanged 0.2 s bilateral support-loss
criterion. The final owned-target full trajectory stops after334 actual steps at1.3917 s
on the same support-loss criterion, with zero stable substeps and no rise.
Its120-step physical/command prefix is exactly identical to the independent
short run. Neither advances from preparation to rise.

Prepared native copy measured about 27 µs; one native prediction 3.54–7.19 ms,
four steps about 15.65 ms in the Sol release benchmark. Full coupled control
costs about 464 ms per 4.17 ms physical substep (median previous full run).
This is unsuitable for interactive 240 Hz execution. Bounded work and safe
refusal are verified; real-time performance is not. Caching would require a
validated state/contact/saturation trust region and fresh candidate validation.

## Evidence and reproducibility

`evidence/native-control-2026-10-11/` contains complete JSON/JSONL traces,
physical contacts, COM velocity/acceleration, motor commands and delivered
constraints, timings, rejected trials, source snapshots and release binaries.
`RUN-METADATA.md` and binary manifests associate experiments with exact source.
`native-command-ownership-500ms.png` compares the structural intervention;
`native-multistep-333ms.png` shows real toe braking;
`native-one-step-fall.png` labels the earlier physical failure.

The raw QP refusal matrix has max symmetry discrepancy 3.64e-12 and is positive
definite. Computing the upper triangle and mirroring fixes assembly without
changing the original strict symmetry/KKT tolerance; the independent backend
certifies the exact corrected matrix. NaN inputs and nonconvex matrices refuse.
Tiny positive-definite Newton steps are solved instead of rejected solely by
step length. Rejected matrices and controller versions remain in evidence.

Final retained-source checks:27 recovery tests passed (3 explicitly ignored),
4 measurement tests passed, targeted clippy and formatting passed. The archived
controller revision passed28 recovery tests including its QP regression test
(4 explicitly ignored). Raw logs distinguish these source revisions. They verify infrastructure and original guard
behavior, not physical recovery. The known baseline hybrid articulation and
four TypeScript failures have not been repaired or declared passing. No claim
is made that the full suite passes.
