# Deterministic balance, falling, and recovery verification

Run `npm run test:physics` with Node 22.13 or newer. The Windows workspace's bundled Node 24 executable is available through `CODEX_MCP_NODE_PATH`; the default PATH currently resolves to unsupported Node 20. The entry point delegates to `scripts/physics-acceptance.ts`; fixture inputs and independent golden thresholds live in `scripts/physics-fixtures.ts`.

The harness writes `evidence/physics-results.json` and `evidence/successor-trace.ndjson`. JSON includes exact trajectories, command and release frame indices, initial root parameter and heading, backend version, numerical limits, every scenario outcome, and per-segment handoff measurements. The trace includes all segment poses at simulation time zero and every sixth fixed update. Assertions inspect **every** fixed update, including unrecorded trace frames and both sides of a transfer. All scenarios use Rapier 0.20.0 and exactly 1/60 second per update.

## Fixed acceptance contract

| Measurement | Required value |
| --- | --- |
| World-space separation of corresponding joint anchors | At most 0.08 m |
| Collider depth below the existing floor surface | At most 0.08 m |
| Each segment's translation discontinuity at either handoff | At most 0.025 m |
| Each segment's shortest relative rotation at either handoff | At most 3 degrees |
| Representative automatic recovery | At most 25 simulated seconds from committed fall |
| Stable upright observation after recovery | At least 1 consecutive second |
| Repeated complete fall/recovery cycles without Reset | 5 |
| Neutral 30-second root drift at headings 0, +pi/3, and -pi/4 | At most 0.001 m, with zero corrective steps |
| Neutral peak segment linear / angular speed | At most 0.1 m/s / 0.5 rad/s |
| Joint motor torque | At most 110 Nm |
| Pelvis assistance force / torque | At most 950 N / 300 Nm |
| Transfer initialization linear / angular speed | Clamp inherited values to 3 m/s / 6 rad/s |
| Inherited-velocity comparison tolerance | 0.00001 in velocity units |
| Paired lockout replay position / velocity difference | At most 0.0000001 m / velocity units |
| Paired lockout replay rotation difference | At most 0.00001 degrees |

All segment positions, rotations, linear velocities, and angular velocities must remain finite. Dynamic recovery must not invoke direct body translation, rotation, velocity, or body-type setters; the harness instruments these methods after fall initialization. Each fall clears all external grab vectors and stored force/torque values exactly to zero. Motors and supported assistance have separate diagnostics.

Floor error is actual collider penetration, not clearance. The harness independently constructs each segment's Rapier Ball, Capsule, or Cuboid from the anatomical definitions and queries its complete oriented shape against the finite, enabled floor collider. Negative contact distance is penetration; positive clearance, a disabled floor, and shapes outside the finite floor footprint yield zero. A geometric fixture asserts these cases and a known 0.035 m penetration. Joint error is independently recomputed from the parent and child anatomical anchors. Handoffs are instrumented at `activateRagdoll` and `restoreUpright`, without integration between the two pose samples. The JSON retains the result for every segment, rather than only the pelvis or the maximum.

## Contact and phase evidence

The independent `RECOVERY_ACCEPTANCE` object asserts that implementation thresholds remain equal to these values:

| Condition | Threshold |
| --- | --- |
| Upward floor-contact normal | Y component at least 0.65 |
| Contact solver distance | At most 0.012 m |
| Measured contact load | At least 3 N |
| Load-bearing persistence for one segment | 0.05 consecutive seconds |
| Loaded non-foot landing contact persistence | 0.10 consecutive seconds |
| Settling persistence | 0.30 consecutive seconds |
| Settling mass-weighted RMS linear / angular speed | At most 0.65 m/s / 1.8 rad/s |
| Stable foot-supported standing persistence before transfer | 0.55 consecutive seconds |
| Stable mass-weighted RMS linear / angular speed | At most 0.22 m/s / 0.65 rad/s |
| Stable torso/pelvis up-vector Y component | At least 0.97 |
| Minimum phase duration | 0.20 seconds, with required contact and pose evidence |
| Unsupported phase retry | 0.20 seconds without qualifying support |
| Supported phase stall retry | 3.0 seconds without sufficient progress |

Landing may use alternating loaded non-foot segments; it cannot be inferred from still-planted feet. Recovery assistance requires persistent load on an eligible segment. Rolling permits torso, pelvis, arms, hands, thighs, shins, and feet. Bracing permits hands, forearms, shins, and feet. Kneeling permits hands, shins, and feet. Standing permits only feet. Head and neck contacts never authorize assistance.

Phase assertions inspect the preceding integrated contact and pose evidence: rolling to bracing needs hands, shins, or feet plus torso-up greater than 0.25 and pelvis above 0.28 m; bracing to kneeling needs shins or feet, torso-up greater than 0.65, and pelvis above 0.38 m; kneeling to standing needs both feet, torso-up greater than 0.88, and pelvis above 0.55 m. Returning to the procedural motor additionally requires persistent stable feet, low motion, and upright torso/pelvis at the same instant.

## Scenario coverage

Recoverable fixtures exercise both hands and feet, forward/backward/lateral pulls, non-default headings, reversal, held targets, and release during an actual corrective swing. Their geometry must stay connected and the documented minimum number of corrective steps must occur. Strong five-tick pulls and sustained 180-tick pulls must fall, protect, land, settle, recover, and remain stably upright within the fixed limit.

Paired replay uses the same initial pose, build, backend, and timestep sequence. One character receives no new body grabs during lockout; the other receives repeated begin/move attempts. Lockout covers every dynamic frame and all 45 frames of the motor-owned recovered-pose seed, during which balance diagnostics remain null. Every segment's trajectory, body-input transition, balance-availability transition, and motion-state transition must match within the stated tolerances. A new press after stable recovery must be accepted.

Support loss is tested by disabling the floor before integration. Assistance must be zero immediately; a separate prolonged loss must cause a dynamic retry and must never force standing after a timeout. A supported obstruction fixture adds a fixed ceiling at Y=1.30 m, with half-extents (4, 0.05, 4), after contact-based settling. It verifies actual stalled progress with loaded support, repeated dynamic retry, and no forced standing for 25 seconds.

Pause/resume and Reset are exercised separately in falling, fallen, and recovering, followed by fresh body input. Runtime tests separately cover same-update pointer capture release, cancellation of queued commands before additional fixed substeps, held-pointer isolation, renderer switching, and cleanup without an additional physics step. Browser replays in both renderers provide the separate visual acceptance record.

## Preserved baseline

The pre-edit working tree is preserved under `evidence/baseline-20260906`: all 135 tracked/untracked source files in `source-snapshot.zip`, SHA-256 inventories, original porcelain status, HEAD, binary working-tree diff, and exact check logs. Typecheck and lint passed; the original physics harness passed 11 scenarios; the Node 24 build and 20 application tests passed. Default Node 20 `npm test` failed before tests because `node:fs/promises` lacks `glob`; running the unchanged source on Node 24 passed. Build warnings concerned chunk size and unknown static route classification.
