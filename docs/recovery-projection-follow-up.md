# Recovery projection follow-up: partial, not a release

Date: 2026-09-19. Requested checkpoint: b0d1e83f27c0fa6a70b26aa8c1ae7fb112d03185.
Preserved concurrent parent: c146683c41ce8c766c01c2fc34a0357c51ed6fd5.
Branch: fix/b0d1e83-physical-recovery. Main remains unchanged.

## Incremental corrections

The recovery arm planner now solves the effective forearm/hand chain for each
fixed wrist rotation directly, then bisects the scalar hand-floor height. It
reconstructs every joint and retains the existing anatomical limits, actual
surface-clearance checks and shared measured/planned hand lever bound. The
near-straight two-bone calculation uses a factored triangle area and atan2,
removing heading-dependent numerical elbow bends and endpoint errors.

The recovery foot search retains the measured/preferred candidate over the
physical leg reach and searches forward as well as backward. A legal leading
half-kneel sole more than 30 cm ahead of the hip no longer disappears from the
candidate set. Existing joint-limit and floor checks still decide feasibility.

Two new regression tests cover near-extension endpoints over wrist flexions and
headings, and mirrored forward half-kneel plants. Both fail on the requested
checkpoint and on the concurrent parent, and pass on this correction. No existing
test assertion or threshold was changed.

The concurrent parent's corrected recovery fixtures, measured-mass oracle,
physical fall gate, four regression tests and provenance are preserved. This
follow-up does not take credit for replacing those earlier corrections. The
original historical native streams remain byte-for-byte unchanged.

## Verification of the integrated source

Linux, Node 22.16.0, npm 10.9.2; unchanged lockfile, offline locked dependencies.

| Check | Result |
| --- | --- |
| npm test, including production build | 169/174 pass; five fail |
| Original tests from b0d1e83 | 163/168 pass, up from 156/168 across the repair branch |
| Added tests across the repair branch | 6/6 pass, including these two regressions |
| npm run test:recovery | 81/82 pass; one fail |
| npm run typecheck | Pass |
| npm run lint | Pass; two pre-existing unused-import warnings |
| npm run test:pages | Pass; 12 local asset references verified |
| Selected physics contracts below | 6/6 pass; not the full acceptance suite |
| Full physics, browser motion, independent review, deployment | Not verified as passing |

Selected physics cases: expanded-anatomy-shared-geometry-and-ownership,
solver-structural-joint-limits, frozen-recovery-threshold-contract,
finite-floor-penetration-semantics, seven-region-picking, and
floorless-center-of-mass-free-fall.

The browser attempt returned net::ERR_BLOCKED_BY_ADMINISTRATOR; no browser
policy bypass or successful interaction review is claimed. Earlier full-physics
runs on the pre-integration tree timed out and failed balance scenarios; they
are not evidence of a passing integrated release.

## Remaining blockers and rejected experiments

The unchanged slow pull/release/reversal test falls at tick 354, and the planted
reversal test falls at tick 257. Strong pulls still fail to produce the required
physical fall and recovery-input lockout. The native fixed-stream provenance test
still reports a leftShoulderGirdle initial-rotation mismatch. These are the five
failing unit tests; the strong-pull case is also the remaining recovery failure.

Measured-root/support-frame retuning worsened the balance scenarios and was
removed. Sharing the filtered grab target with posture introduced a new chest-drag
joint-separation failure on the concurrent parent's fall gate; it and its test
were removed. EmbodiedCharacter, controller effort/power limits, collision
exclusions and physical step-completion rules are unchanged from the parent.

The next release blocker is actual support transfer, not counting a planned step
as completed or relaxing the strong-pull test. This branch remains a draft.
No merge, deployment, history rewrite, fake contact or root teleport is included.
