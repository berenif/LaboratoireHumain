# Body alignment, self-collision and physical mass repair

Status: feature repair verified locally; not a complete balance release. Keep the pull request unmerged while the two existing balance gates fail.

## Source and scope

Continues `berenif/LaboratoireHumain` from main `06c53a933c637c2425888558ae788f4f99d40f58` on `agent/body-alignment-self-collision`. The base was an explicitly merged work-in-progress checkpoint with two failed balance tests. No history was rewritten, no engine was replaced, and no runtime body transform or velocity override was introduced.

The request is to align the feet and arms, make limb/trunk collisions real, and make body weight calculations consistent with the physical assembly.

## Causes and changes

### Anatomical directions

Elbows used the same positive-X flexion frame as knees. A downward forearm therefore flexed backwards. Both elbow joint frames now rotate 180 degrees about local Y, so positive elbow flexion acts about anatomical -X while knee flexion remains +X. The neutral frame and the existing positive scalar elbow limit are preserved. The upright and recovery arm solvers now use that declared hinge axis, including native Rapier limits and recovery wrist orientation.

The shoulder's forward/backward and outward/inward ranges were also reversed. Their magnitudes are retained but assigned to the correct directions. Shoulder-girdle protraction and elevation ranges mirror correctly between sides. Recovery elbow poles are body-relative instead of assuming world-down.

### Self-collision

Both upper arms explicitly excluded the torso. Those exceptions are removed. The shoulder socket moves from 0.225 m to 0.250 m from the centre, with the shared girdle geometry and anchors extended consistently. This removes the approximately 9 mm initial upper-arm/torso overlap without shrinking the torso or disabling contact.

All nonadjacent upper-arm, forearm, forearm-twist and hand pairs against pelvis, lumbar and torso remain enabled. Direct anatomical joint neighbours and the existing compact ankle-housing exceptions remain excluded. CCD, native contacts and bounded actuation remain active. Physics still owns every rendered body transform.

### Physical mass and applied force

Balance and recovery previously weighted modeling origins rather than the volume centres used by Rapier. A shared signed-volume centroid calculation now feeds both controllers; recovery gravity lever arms use the actual rigid body's `worldCom()`. The 72.2 kg total mass and individual segment masses are unchanged. The initial whole-body discrepancy was approximately 7.97 mm before repair.

Standing balance now receives the same bounded force actually applied by the grab controller in that update. It no longer counteracts a separate, much larger spring estimate in production. Grab force, torque, speed and power limits are unchanged.

Recovery support also rejects a reported arm load above the floor and does not qualify a low-shoulder target as a usable floor brace. Joint and contact tolerance gates were not loosened.

## Verification

- The original four added defect checks failed on the base and passed after repair. The final `tests/body-physics.test.mjs` contains eight checks covering direction, outward reach, collision coverage, native hinge limits under torque, mass-centre integration and dynamic contact.
- The combined body, recovery-arm and initial-snapshot suite passes 23/23. Four inward-pull cases cover both arms at headings 0 and 1.1 radians for 180 ticks each. All produce sustained real contact impulses, with zero sampled arm/trunk penetration. Maximum mass-centre disagreement with Rapier is under 0.00000003 m. Largest sampled joint gap in those cases is 0.0192 m, below the unchanged 0.08 m structural gate. This is a finite test sample, not a claim of mathematically zero penetration in every possible interaction.
- The full locally executable domain suite passes 135/137 across 17 test files; only the two existing balance scenario categories fail. Built-page/UI tests are not part of this local run because full application dependencies are available in CI, not this offline runtime.
- The focused existing release suite passes 15/17. Connected strong-pull falling and input lockout pass. Slow pull/reversal still falls at tick 237, and planted reversal at tick 203. These are the two already-failing scenario categories on main; the changed trajectories are not claimed as improvements merely because their failure times moved.

Reproduce with:

```sh
npm ci
npm run typecheck
node --test tests/body-physics.test.mjs tests/recovery-support.test.mjs tests/native-fixed-streams.test.mjs
node --test tests/character-domain.test.mjs tests/physical-chain-integrity.test.mjs tests/balance-controller.test.mjs
```

The read-only body-physics CI workflow records its tested commit and uploads its test log. The existing full release workflows are retained; this focused job is not a replacement for them.

## Browser review and limits

The production `DemoRuntime`, character, picking, pointer interaction and renderer were bundled directly from this source for offline review. Native mouse input at 1280x900 and native touch input at 390x844 selected the right hand, moved it, released it cleanly and finished upright without runtime exceptions. The side view confirms that forearm flexion and toes share the forward direction.

This local browser cannot create WebGL2 and falls back to the actual Canvas2D renderer. This is not a full application build or a successful WebGL review. Full build, page, WebGL and wider physics checks remain the responsibility of the retained release workflows. No deployment or merge is implied by the local checks.

## Recorded-input preservation

The anatomy change invalidated the two stored initial assembly snapshots. Only these snapshots were refreshed, using `scripts/refresh-body-initial-snapshots.ts --write`. Their captured command sequences and recorded historical transfers are unchanged. A SHA-256 assertion freezes those arrays at `56236d783618a932b5a2777b450a6697e83992065866aa7edd56bfd488c2ccbd`.

The migration is labeled explicitly in the fixture. It is not a new browser capture, not a replay of the historical recoveries, and not evidence that those recoveries currently pass. The independent recovery reconstruction test now reads the declared hinge frame rather than hard-coding the defective +X elbow. Its numerical joint-limit thresholds are unchanged.
