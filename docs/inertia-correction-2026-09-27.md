# Geometric inertia correction — unaccepted candidate

This page retains the initial inertia candidate's results. [H40](standing-h40-contract.md)
subsequently repairs the chest-contact regression and passes the prior 49-test
selection. Its wider 63-test selection exposes two earlier anatomy failures.
Standing and complete release acceptance remain open; see [current status](status.md).

The [H38 audit](standing-h38-contract.md) confirms incorrectly oriented physical
inertias for 13 of the pinned runtime's automatically computed convex segment
mass properties. Explicit signed-tetrahedron integration agrees with the older
runtime's full tensors to below 1e-6 relative error on all 25 unchanged meshes.
The correction is mathematical; it does not establish standing acceptance.

`src/core/geometry-mass.ts` now computes mass properties from the float32
vertices actually passed to Rapier. `EmbodiedCharacter.ts` supplies those
properties when constructing each collider, using the segment's nominal mass.
Shapes, body count, transforms, collision filters, actuator ceilings, timestep,
solver settings and fall guards remain unchanged. This supersedes the earlier
claim that all 65 production/package files are unchanged: 64 still match H8,
the character factory has these additional edits, and the geometry-mass module
is new. Pre-existing working-tree changes remain preserved.

The H38 diagnostic instead supplied the already rounded body mass after
construction. Its traces and the new factory's traces are distinct captures;
do not infer exact trajectory replay between them.

## Verification and remaining failures

The [verified candidate archive](checkpoints/2026-09-27/inertia-production-candidate/manifest.json)
retains commands, logs, source fingerprints, the geometric calculation tests,
and the new plain-controller 2+30 s capture.

- Typecheck passes. Targeted ESLint has no errors and one pre-existing unused
  `RIGHT` constant warning in the character file.
- Initially 47/49 targeted tests pass. This includes four new geometric tests, checking
  analytic tetrahedron moments, translation/winding, rotated repeated/small
  inertias, and all production body tensors at two headings.
- The left-hand chest-drag test fails its actual-contact assertion: no torso
  contact is observed. Penetration and joint separation are within their limits.
- The tiny ankle-impulse prediction test fails unchanged tolerance: the first
  reported expected/actual values are −2.253565/−1.868033.
- Both failing tests pass with the exact archived pre-correction factory loaded
  process-locally; 64 other production/package dependencies are hash-verified
  unchanged. These are confirmed regressions, not waived historical failures.

A follow-up isolates the ankle fixture's zero-impulse background rate. At +π/3,
the right-foot Z background is 0.00010457944 rad/s; subtracting it changes the
measured response from −1.868033 to −1.972612. Both impulse signs at all three
headings, across all five measured coordinates, pass the existing 15%/0.15
tolerance when measured as `(driven rate − control rate) / impulse`.

The test now uses an exact snapshot copy as its zero-impulse control, asserts
all 25 initial body states/mass properties match, and covers both signs. Its
fixtures, impulse magnitude, headings, solver and numerical tolerance are
unchanged. The original raw-response test and failure are preserved in the
first validation's `test-source.json`; the measurement definition is explicitly
corrected rather than claiming that the old assertion now passes. This is
separate from the previously diagnosed contact-loaded small-impulse outlier.
The repeated selection passes **48/49** tests; targeted lint for the changed
test and new diagnostic scripts passes.

The chest regression remains. An unchanged-fixture trace exactly reproduces
both recorded root-travel results. With corrected inertias the hand contacts
the pelvis and lumbar segment, but the smallest queried arm-to-torso distance
is 0.018823 m and no torso manifold appears. At the final sample the hand is
0.3920 m from its requested point. This is not a disabled self-collision filter;
the target/contact behavior still needs repair without a force-cap increase.

The current plain controller fails the official-duration standing measurement:

| Heading | Foot drift (m) | Pelvis drift (m) | Peak linear (m/s) | Peak angular (rad/s) |
| --- | ---: | ---: | ---: | ---: |
| 0 | 0.058953 | 0.043094 | 0.644918 | 1.680792 |
| +π/3 | 0.044511 | 0.015380 | 0.714819 | 1.563424 |
| −π/4 | 0.050226 | 0.048371 | 0.618331 | 1.729441 |

All three remain upright in double support with zero steps. Maximum excursions
and segment vertical ranges are retained in the report. Full application,
recovery, physical, browser and release acceptance remain open. The next work
must retain the geometric correction's validity while resolving the changed
controller behavior and the remaining chest-contact regression; it must not widen tolerances
or refresh fixtures to hide these failures.
