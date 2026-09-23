# Physical coherence acceptance contract

## 2026-09-23 publication checkpoint: work in progress

After the failed balance gate was disclosed, the user explicitly requested
"Commit push and merge to main". This later instruction authorizes preservation
and publication of the current merge; it does not establish physics acceptance.
The merge preserves both 60b9e010773560968fad3a18ddb27a8cb9a768e6 and
3ca4cc86bec4c33ab928c44f1c71692ab1ce56ae. Remote main was fetched and still
pointed to the latter commit before publication.

The final focused command runs balance-controller, landing-plan and
leg-target-dynamics tests: **34/36 pass, exit 1**. Balance is **12/14**;
slow pull falls at tick 169 and planted reversal at tick 130. Landing planning
is 13/13 and inertial leg-target mechanics 9/9. Both probe scenarios complete
two measured steps but subsequently fall; the probe exits 1. These failures
remain enabled. TypeScript passes after a type-only correction to a mutable
local pose-composition vector; the physical failures reproduce unchanged.

Current changes include explicit infeasible landing results, a bounded
two-dimensional joint-limited search, measured-pelvis planning/actuation
agreement, candidate-specific support/capture prediction, load diagnostics,
and swing-target inertial feedforward under the existing motor caps.
Retained-pressure delivery and second-swing height loss remain unresolved.
Experimental stance-height springs and alternate actuator/solver configurations
were confined to ignored diagnostics and were not adopted. No acceptance
assertion, absolute 52% retained-load gate, physical fall threshold or torque
budget was weakened; direct pelvis assistance remains zero by design.

The prescribed final fixture refresh, full automated suite, all 63 physics
scenarios, five-cycle protocol, two Pages configurations and complete browser
coverage have **not** been verified on this source. Publication is a WIP
checkpoint, not an accepted release. Older results below describe historical
sources and must not be read as current passing evidence.

Commands, native exit codes, source fingerprints, traces and preserved patches
are in ignored evidence/merge-execution-20260923-160052/, including
publication-balance.json, publication-typecheck-fixed.json,
publication-review.json and publication-probe.json.

## Historical records and acceptance contract

Frozen before integrated tuning. Dimensions remain 1.84 m nominal height and mass
72.2 kg. Existing justified limits and existing failing cases remain in the suite.
New focused tests use metres, seconds, radians, kilograms and newtons.

- Frame covariance: physical initial segment positions after common yaw agree
  within 0.1 mm; neutral hindfoot forward error < 2 degrees. Positive anatomical
  elbow/hip flexion moves a hanging distal segment forward; knee flexion backward.
  Test 0, 90 and 180 degree headings, mirrored girdles and both sides.
- Collision: initial non-excluded overlap <= 1 mm; deliberate slow/fast contact
  peak penetration <= 15 mm, penetration sustained for 12 frames <= 5 mm. The
  initial tolerance excludes contact skin; geometry is tested directly. Joint
  separation <= 10 mm in focused collisions. Existing 80 mm hard-failure gates
  are retained, not increased. No exclusion may disable upper arm versus trunk.
- Mass: actual Rapier masses sum to 72.2 kg within 0.0001 kg; all principal inertia
  components positive. Whole-body CoM agrees with mass-weighted worldCom to 1 µm.
  Passive unsupported CoM acceleration agrees with gravity to 0.1 m/s²; external
  support and ground-reaction feedforward disappear as soon as contacts disappear.
- Quiet stance, all existing pulls/reversals, support transfer, recovery and
  browser interaction must pass unchanged before merge. A passed build alone is
  not acceptance. New failures cannot be relabelled as inherited failures.

## Coordinate convention

Body +X is right, +Y is up, +Z is forward. Shoulder, elbow and hip flexion use
joint +X = body -X. Both sides of these joints share the same proper Y-half-turn
basis, leaving their zero relative orientation unchanged. Knee flexion uses
body +X. Ankle positive flexion lifts the toe; forefoot positive flexion lifts
the forefoot. Inverse kinematics must use these frames, not hardcoded world X.
Mirrored girdle elevation/protraction limits are mirrored, not copied.

## Surface and mass ownership

The upper-arm medial proximal wedge was embedded 9.02 mm in the ribcage at rest.
A closed convex clipping plane removes that wedge in the common render/pick/
collider geometry. It does not enlarge collision bounds, move anchors, change
length or change segment mass. Rapier recomputes CoM and inertia from that shape.
Each segment still has exactly one massive collider. Adjacent joints and the
existing overlapping ankle housing pairs are excluded; upper arm/ribcage is not.

Forced initial sleep and pose-derived runtime contact fallbacks are not accepted
as quiet standing. Actual positive floor contact loads determine support.

## Contact-state transitions frozen before their integration

Swing starts only after the moving sole carries no more than 25% of nominal
body weight and retained persistent environment contacts carry at least 52% of
**whole-body weight** for 0.10 consecutive seconds. A completed swing must
observe less than 3 N on its moving side for 0.05 consecutive seconds before a
new 0.10 s loaded touchdown within the existing 0.09 m horizontal target
tolerance. Planned or manipulated feet remain in the measured-support report
until they actually unload.
The controller's available-support subset is a separate field. Runtime force
anticipation uses the actual bounded grab impulse divided by dt, not its former
independent 620 N spring estimate. No external support is synthesized.

Native force-based motors now participate in Rapier's contact solve. Allowed axes
retain existing effort ceilings and gains; missing degrees of freedom are locked
by structural joint bits, not zero-width limit rows. Diagnostics distinguish the
bounded requested motor wrench from a solver impulse (not exposed by this API).

## Measured contact-tolerance defect

The installed Rapier 0.20 reports 5 mm allowed contact error and 20 mm prediction
distance. A slow passive thigh pair reached 12.36 mm geometric overlap while
retaining stale narrow-phase witnesses. Explicit 1 mm allowed error and 2 mm
prediction distance keep human-scale contacts tight without increasing the
existing iteration budget. Pair fixtures and production use the same settings.

Contact-impulse readback in this pinned release is not an exact absolute load
measurement: an independent 72.2 kg free body reads 743.70 N at 20 solver
iterations where vertical momentum balance gives 708.282 N. Internal additional
iterations can increase this discrepancy. Contact existence, normal, persistence
and positive impulse remain directly measured; load-sharing ratios/thresholds
still require validation against momentum. No fabricated replacement impulse or
arbitrary rescaling is used to claim that gate passed.

`stepCount` increments at completed, measured touchdown, not at a planning attempt.
The phase field exposes unsuccessful unloading attempts without calling them steps.

## Subsequent WIP merge authorization and concurrent-main reconciliation

After the failing saved-checkpoint results were disclosed, the user explicitly
requested `Merge main`. This authorizes a work-in-progress persistence merge,
not satisfaction of the acceptance contract above. All thresholds still apply
before an accepted physics release; known failures remain failing.

During publication, PR #5 independently advanced main to `d670e755303fbaead6112c3e5bf6c93592951212`.
The integration retains that ancestry and the exact saved commits. Its wider
shoulder-girdle geometry supersedes the clipping approach described above, so the
production arm uses the full original surface at the newer shared socket. Applying
both geometry corrections or reversing an already reversed joint axis is avoided.
See `body-coherence-integration.md` for the exact resolution and verification.
