# Physical coherence acceptance contract

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

Swing starts only after the moving sole drops below 5% nominal body weight and
retained persistent environment contacts carry at least 65%. A completed swing
must have observed less than 3 N on its moving side before a new 0.10 s loaded
touchdown within the existing 0.09 m horizontal target tolerance. Planned or
manipulated feet remain in the measured-support report until they actually unload.
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
