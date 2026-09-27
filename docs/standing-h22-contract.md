# H22–H24 — idle forcing, stance tasks and controller-phase tracing

The existing arm pose requests opposite-phase 0.018 m hand oscillations at
1.7 rad/s even with no user disturbance (`pose.ts`, `composeArm`). H21 fixes
stance intent and uses measured patch moments but still creeps and fails one
angular-speed bound. H22 tests whether this periodic internal forcing drives
the remaining stance motion.

Compare H21's `held-pressure-contour-world-damping` against the identical
`quiet-held-pressure-contour-world-damping` mode from initialization. Recompose
only arm targets at animation phase zero while preserving actual simulation
time, balance feedback, torso/head/leg targets, native motors, gains, ceilings,
contacts, collision settings and solver. Assert every non-arm target is exactly
unchanged by this operation. Arms remain dynamic, supported and actively
controlled through their existing native joints; they are not locked.

Require the same 2 s settling + 10 s screen at 0, +π/3 and −π/4: foot drift
≤0.01 m, pelvis drift ≤0.03 m, all-segment linear/angular speeds ≤0.1 m/s and
≤0.5 rad/s, no step/support loss/non-upright state, and the existing structural
and collision limits. Require the H21 controls to replay exactly. Any failure
rejects H22; no amplitude or frequency sweep follows. No production adoption
before all screening and full official standing requirements pass.

H22 is rejected: all speed limits pass, but foot drift is
0.01501/0.01378/0.01777 m. Suppressing commanded arm oscillation removes the
last speed failure and does not remove the sustained foot-position error.

## H23: explicit foot-position task with fixed references and measured moments

H17's Cartesian stance task failed while its posture reference still followed
slip and its feedforward routed support through hindfeet alone. H22 removes
those two conflicts and all speed failures. Compare H22 to the same mode with
H17's explicit position/velocity task, using exactly its existing gains,
acceleration bound, measured mass share and Jacobian-transpose joint torques.
This isolates the interaction between compatible position and moment intents;
it is not a gain sweep. Every torque remains inside the existing native ceiling.
Require exact H22 control replay and the same complete three-heading 2+10 s
screen. Reject on any failed candidate heading.

H23 is rejected: the −π/4 assembly is fallen by tick 653. The initial six-run
screen stopped at that point because the observer incorrectly reconstructed
recovery commands from stale standing contributions. A focused repeat records
the mismatch explicitly: `state=fallen`, `commandIsStanding=false`. This is a
trace-boundary defect, not a standing controller fix. The observer now forwards
non-standing commands unchanged and labels them separately, while retaining the
same reconstruction assertion for every actual standing command. A focused
repeat then preserves the physical failure through the complete screen.

The corrected focused observer completes: the neck reaches 47.18023 rad/s at
tick 605 and the assembly ends recovering. H23 therefore fails independently
of the observer's earlier exception.

## H24: correct every non-leg tracking frame after the neck failure

Production expresses every non-leg target against its measured parent, but
the experimental world-damping correction has so far covered only arms. The
H23 neck oscillation exposes the same target/damping mismatch higher in the
trunk chain. Extend the existing exact parent-angular-velocity subtraction to
all non-leg joints (lumbar, torso, neck and head included), on the H23 candidate.
Keep leg damping, all gains, ceilings, commands, contacts and physical settings
unchanged. Compare all three headings against H23, including its failed −π/4
control. Require the same 2+10 s complete screen and exact replay of the saved
−π/4 control. Reject on any candidate failure, without tuning.

H24 is rejected. The 0/+π/3 foot drift is 0.01250/0.01070 m, the heading-zero
angular peak is 0.71859 rad/s, and −π/4 loses double support at tick 606 and
first leaves the accepted upright states at tick 616. Its final state is
recovering. The H23 −π/4 control replays exactly; its support loss and first
non-upright state are ticks 600 and 601.

That phase history also corrects the initial interpretation of the neck peak:
H23's 47.18 rad/s at tick 605 occurs **after** its first non-upright state.
It cannot establish a neck damping fault as the initiating cause of the fall.
The requested all-non-leg frame correction was tested and rejected, but the
post-fall peak must not be reused as evidence for a standing instability.

## Evidence and verification

The [archive](checkpoints/2026-09-27/h22-h24-evaluation/manifest.json) retains 32
compressed artifacts: complete bounded event windows and response reports,
failed/partial observer runs, the regenerated observer controls, and source
snapshots at archival time. All compressed and decompressed hashes verify.
The initial H23 screen did not save complete response histories before its
observer exception; that limitation is explicit in its failure record. H24
regenerates H23 at all three headings with the corrected observer.

The [post-H24 validation](checkpoints/2026-09-27/continuation-validation/manifest.json)
records targeted tests/lint and three plain/observed/repeated trajectories
matching the regenerated original exactly. Production/package source remains
unchanged. No experiment here establishes official 2+30 s standing acceptance.
