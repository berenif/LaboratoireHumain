# b0d1e83 recovery repair — partial, not an accepted physics release

Base: `b0d1e83f27c0fa6a70b26aa8c1ae7fb112d03185`.
Verification date: 2026-09-19. Main remains unchanged; merge is blocked by the
failures below. No existing test assertion, case, anatomical limit, collision
exclusion or deployment gate is removed or relaxed.

## Changes

- **Actual elbow kinematics.** A flexed/deviated wrist changes the direction and
  length of the effective forearm-plus-hand bone. The recovery solver previously
  applied the elbow limit to that effective bone angle. It now solves the actual
  elbow angle and aligns both bone vectors, retaining exact captured hand
  endpoints, local wrist rotation, bone lengths and legal elbow coordinates.
  Floor-target convergence has a bounded 96-iteration ceiling and an explicit
  quaternion-residual early exit. Exact floor targets remain distinct from the
  unchanged tolerances for live approximate targets.
- **Joint-frame-correct landed fixtures.** Shoulder, hip and ankle signs now agree
  with the shared half-turn anatomical frames. All 24 named fixtures, mirrors,
  headings and weak half-kneels remain. Prone is still fully horizontal; a new
  regression prevents tilting it to ease floor-brace placement. The supported
  crouch uses hip 0.34, knee 0.66 and ankle 0.32 radians, so the sole is level
  without requesting 43.5 degrees of dorsiflexion from a 20-degree ankle. It is
  consequently shallower than the invalid previous crouch. The half-kneel uses
  connected forward kinematics and a bounded leading-leg solve to align its
  actual sole with the trailing convex shin surface before integration. These
  are fixture initializations, never runtime body-transform corrections.
- **Independent mass observer.** The physics evidence compared aggregate runtime
  COM to nominally weighted transform origins even though Rapier supplies actual
  mass centres. The observer now independently aggregates the recorded per-body
  measured masses, centres and velocities. Pose-only legacy inputs retain their
  previous nominal-mass/origin interpretation. It does not call the production
  mass helper. The existing 1e-7 COM/projection gates remain, and added negative
  tests still reject origin-based or nominal-mass aggregate diagnostics.

## Local verification

Node 22.16.0, exact locked dependencies. The GitHub workflows pin Node 22.13.0;
local results are not represented as remote CI results.

| Check | Result |
| --- | --- |
| `npm test` | Build passes; 168/173 pass, 5 fail |
| `npm run test:recovery` | 84/85 pass, 1 fail |
| Body coherence + body physics | 24/24 pass |
| Added regressions on repaired source | 5/5 pass |
| Same added regressions on exact b0d1e83 | 0/5 pass, reproducing the defects |
| Typecheck and lint | Pass; 2 pre-existing unused-import warnings |
| `npm run test:pages` | Pass; static export and 12 local asset references verified |
| Selected physics harness | 3/3 pass; selection below, not a full-harness pass |

Seven of the original twelve failing unit cases are fixed. Five new tests are
added; passing-count increases must not be confused with fixing twelve cases.

Selected scenarios: `expanded-anatomy-shared-geometry-and-ownership`,
`solver-structural-joint-limits`, `floorless-center-of-mass-free-fall`.
A separate selection including `idle-30-seconds` was stopped after 199 seconds
of wall time while that scenario remained incomplete. Its two preceding checks
passed; idle is unverified, not passed. Full end-to-end recovery physics, browser
motion/input replay and an independent review are not verified for this repair.

## Remaining release blockers

1. Slow pulls/reversals do not reliably complete corrective stepping.
2. Planted reversal still enters a transient unsupported fall.
3. Strong-pull falling/input lockout remains wrong (two unit cases; one also in
   the recovery suite).
4. Historical native initial-assembly parity still fails after the merged body
   geometry/frame changes. The recordings are deliberately unchanged.

The native-recording file SHA-256 remains
`30599cb714c3f96f8d456e7a0466cfa5bc5bc7f296eb8b476df4bf9415915760`.
No standing-controller experiment is included: trials that destabilized quiet
standing were discarded. `BalanceController.ts`, `EmbodiedCharacter.ts` and
`pose.ts` are byte-identical to b0d1e83. No merge or deployment is authorized by
these partial results.
