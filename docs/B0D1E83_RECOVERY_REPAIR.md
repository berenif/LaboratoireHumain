# Recovery projection repair of b0d1e83

Status: PARTIAL FIX, NOT AN ACCEPTED PHYSICS RELEASE. Do not merge or deploy on the strength of this checkpoint.

Base commit: `b0d1e83f27c0fa6a70b26aa8c1ae7fb112d03185`.

## Implemented corrections

- Foot placement considers the measured/preferred endpoint before the coarse search grid. A legal flat crouch sole approximately 19 cm ahead of its hip was previously excluded by the grid's 10 cm forward bound. Full joint-limit and floor-clearance checks still qualify every candidate.
- Arm-brace planning now requires a converged geometric solution instead of treating the runtime 4 mm contact tolerance as exact reachability. The bounded iteration budget is 96, with an early convergence exit. Runtime contact tolerances are unchanged.
- The crouch and half-kneel initialization fixtures use the current anatomical joint frames. Crouch retains its 1.10 rad knee bend and level soles. Half-kneel solves the leading hip/knee angles so its actual sole and trailing shin surfaces share the floor; individual bodies are never repositioned independently. Shoulder abduction is outward in the new mirrored frames.

Three regression tests cover forward plants beyond the old grid, exact two-support fixture construction, and converged floor braces across both sides and rotated headings.

## Local verification, 19 September 2026

| Check | Result |
| --- | --- |
| Production build, as part of `npm test` | PASS |
| `npm run typecheck` | PASS |
| `npm run lint` | PASS, two unchanged unused-import warnings in `tests/leg-target-frame.test.mjs` |
| `npm test` | 165/171 PASS, six failures remain |
| `npm run test:recovery` | 81/83 PASS, two failures remain |
| Body-coherence and body-physics suites | 24/24 PASS |

Among the original tests, the result improved from 156/168 to 162/168; all three added tests pass. Original recovery tests improved from 72/80 to 78/80.

Existing test assertions, release thresholds, anatomical limits, collision exclusions, native motors, balance behavior, and grab limits were not changed. Historical native command streams and recorded transfers were not refreshed or relabeled as fresh evidence.

## Remaining original failures

| Test | Unresolved behavior |
| --- | --- |
| Slow pull, release and reversal | Unexpected physical fall; no accepted completed support transfer |
| Planted reversal | Unexpected physical fall |
| Fall/recovery input lockout | Strong-pull setup does not enter recovery within the existing test window |
| Five-tick strong hand pull | Does not overwhelm balance as required |
| Native fixed-stream initial assembly | Historical shoulder-girdle snapshot disagrees with current assembly |
| Sprawled prone support | A supposed unplaced arm is still classified as usable support |

These failures remain visible and keep release blocked. The two recovery-suite failures are input lockout and sprawled prone support.

A read-only standing trace also exposed up to roughly 0.62 m between desired and reconstructed leg commands when a virtual target pelvis is applied to the measured pelvis. Experimental measured-root and support-wrench corrections did not reliably pass the balance scenarios, so they were excluded from this patch. Fixing this requires a coupled balance/actuation correction, not simply hiding the diagnostic error.

The full physics and browser release gates are not passed by this work. The selected-physics run was stopped by its 120-second timeout (exit 124) without emitting a completed report. It supplies no scenario-pass evidence. Browser motion and deployment were not verified. No merge or deployment is authorized by these results.
