# Deterministic physical-humanoid verification

Run `npm run test:physics` with Node 22.13 or newer. The entry point is `scripts/physics-acceptance.ts`; immutable scenario inputs and numerical thresholds live in `scripts/physics-fixtures.ts`. Every scenario uses Rapier 0.20.0 at exactly 1/60 second per update.

The harness writes `evidence/physics-results.json` and `evidence/successor-trace.ndjson`. Report schema 4 records all 25 segment trajectories, joint-coordinate and contact diagnostics, motion-state changes, body-input gating, and the Rapier backend version. The trace samples every sixth update, while assertions inspect every update.

## Fixed acceptance contract

| Measurement | Required value |
| --- | --- |
| Segment count, total mass, standing stature | 25, 72.2 kg, 1.84 m |
| World-space separation of corresponding joint anchors | At most 0.08 m |
| Collider depth below the enabled floor | At most 0.08 m |
| Structural joint-limit error under sustained torque | At most 0.06 rad |
| Representative automatic recovery | At most 25 simulated seconds from committed fall |
| Stable standing observation after recovery | At least 1 consecutive second |
| Repeated complete fall/recovery cycles without Reset | 5 |
| Neutral settling before drift measurement | 2 simulated seconds |
| Neutral 30-second pelvis drift at headings 0, +pi/3, and -pi/4 | At most 0.03 m, with zero corrective steps |
| Neutral planted hindfoot/forefoot drift | At most 0.01 m |
| Neutral peak segment linear / angular speed | At most 0.1 m/s / 0.5 rad/s |
| Direct pelvis assistance force / torque | Exactly zero |
| Paired lockout replay position / velocity difference | At most 0.0000001 m / velocity units |
| Paired lockout replay rotation difference | At most 0.00001 degrees |

All positions, rotations, velocities, joint coordinates, limit errors, motor loads, saturations, and contact measurements must remain finite. Actual joint torque is checked against each joint profile's permitted-axis actuator budget. Contact diagnostics must report a consistent total count, load-bearing count, normal load, and supporting-segment list.

## Continuous physics ownership

The harness records every Rapier rigid-body and collider object immediately after initialization. The same 25 dynamic objects must remain alive through standing, dragging, fall commitment, landing, recovery, and the return to standing. Reset and explicit fixture seeding are the only operations that may establish a new assembly baseline.

Every body's translation, rotation, linear-velocity, angular-velocity, body-type, and next-kinematic-pose setters are instrumented for the whole active lifecycle. Invoking any of them after initialization or reset fails the scenario. Fall commitment and recovery completion are also sampled immediately before and after the state change; body identity, pose, and velocity must be unchanged.

With the floor disabled, load-bearing contact diagnostics must clear immediately. The mass-weighted center of mass must continue ballistic free fall while internal motors operate: internal equal-and-opposite torques may reorganize the body, but they cannot create an external upward impulse or arrest descent.

## Anatomy, geometry, and constraints

The anatomy gate independently verifies:

- the pelvis–lumbar–ribcage chain, bilateral shoulder girdles, split forearm/twist chains, and bilateral ankle–hindfoot–forefoot chains;
- exact mass and stature, connected anatomical anchors, and direct-pair collision exclusions;
- one canonical procedural convex surface per segment, used by the Rapier collider and exposed unchanged to both renderers and picking;
- seven selectable regions whose grouped segments retain an exact picked segment and local anchor; and
- identical physical poses when snapshots are requested for Canvas2D and WebGL.

The structural-limit gate creates isolated joints through the version-pinned Rapier adapter, disables posture motors, and applies sustained equal-and-opposite torque. It checks mirrored elbows and knees, both torque directions, rotated headings, forearm rotation, ankle flexion, and combined shoulder/hip rotations. Absent coordinates remain locked; declared coordinates remain inside their asymmetric ranges. Elbows and knees must move only in their positive flexion direction.

Floor penetration is measured with independent Rapier `ConvexPolyhedron` shapes made from the same canonical vertices and triangles. Negative contact distance is penetration; positive clearance, a disabled floor, or geometry outside the finite floor footprint is zero error. A known 0.035 m hindfoot penetration guards the measurement semantics.

## Contact and recovery evidence

`RECOVERY_ACCEPTANCE` freezes the contact and phase thresholds used by recovery:

| Condition | Threshold |
| --- | --- |
| Upward floor-contact normal | Y component at least 0.65 |
| Contact solver distance | At most 0.012 m |
| Measured contact load | At least 3 N |
| Load-bearing persistence for one segment | 0.05 consecutive seconds |
| Loaded non-foot landing persistence | 0.10 consecutive seconds |
| Settling persistence | 0.30 consecutive seconds |
| Settling mass-weighted RMS linear / angular speed | At most 0.65 m/s / 1.8 rad/s |
| Stable foot-supported standing persistence | 0.55 consecutive seconds |
| Stable mass-weighted RMS linear / angular speed | At most 0.22 m/s / 0.65 rad/s |
| Stable pelvis/ribcage up-vector Y component | At least 0.97 |
| Minimum phase duration | 0.20 seconds with required contact and pose evidence |
| Unsupported phase retry | 0.20 seconds without qualifying support |
| Supported phase stall retry | 3.0 seconds without sufficient progress |

Eligibility follows anatomical roles. Rolling may use the trunk, limbs, hands, shins, ankles, hindfeet, and forefeet; bracing and kneeling use measured distal-limb support; standing requires bilateral loaded foot support. Head and neck contact never authorize assistance. Direct pelvis assistance remains zero in every phase.

## Scenario coverage

Deterministic pulls cover both sides, four directions, three headings, reversal, held targets, and release during an actual corrective swing. Gentle pulls must remain recoverable, stronger pulls must provoke stepping, and overpowering pulls must create a physical fall without stretching joint anchors.

Recovery fixtures cover crouch, half-kneel, prone, supine, and both sides. Each phase advances only from integrated contact, pose, support-margin, and movement evidence. The suite also covers five consecutive cycles without Reset, floor loss and restoration, supported obstruction retries, pause/resume, Reset, lockout trajectory isolation, and the fresh-press requirement after recovery.

Browser visual replays provide the complementary presentation record in Canvas2D and WebGL, from side and three-quarter cameras at normal and slow playback.
