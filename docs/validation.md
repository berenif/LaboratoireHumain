# Validation record

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

## 2026-09-23 merge execution: mandatory balance gate failed

Current evidence is in `evidence/merge-execution-20260923-160052/` and is not a
release acceptance. The exact balance test command passes 8/10 (exit 1), with
slow-pull/reversal failures at ticks 122/139. The balance probe exits 1; direct
transition diagnostics identify capture instability in both cases. The six
new landing, frame, and load-diagnostic regressions pass (exit 0).

The controller correction remains unaccepted and regresses the preserved
physical baseline. Broader automated checks, fixture refresh, browser coverage,
and publication were withheld at the failed prerequisite gate. HEAD and
MERGE_HEAD are unchanged, no unmerged index entries exist, and the saved staged
patch hash matches the current index exactly. See [merge acceptance](merge-acceptance.md)
and the run's `causal-account.md` for the unresolved transfer behavior.

Generated evidence is the authority for a particular working tree. Run the commands below after controller changes; do not treat historical scenario counts or trajectories as current results.

```bash
npm run typecheck
npm run lint
npm test
npm run test:physics
```

Browser acceptance additionally runs the recovery and interaction replays in WebGL and Canvas2D from side and three-quarter cameras, at normal and slow playback.

## Current investigation (2026-09-19)

Physics acceptance remains unachieved. The current balance trace loses measured
stance support during the first weight transfer, before the second step is
released. A passive articulated-response correction passes the 16 cross-body
paths, but other pull and recovery scenarios still fail. Targeted passes do not
establish full acceptance. The running validation ledger, per-frame evidence,
rejected experiments and visual artifacts are indexed in
[the September 19 record](../evidence/limb-20260919/validation-summary.md).
Concurrent edits in the shared checkout require source-provenance checks before
any final-source claim; a changed controller or golden threshold invalidates it.

## Historical worktree verification (2026-09-13)

The anatomy and continuous-physics migration is integrated, but the complete
behavioral acceptance suite is not yet green. The latest verified partition is:

- `tsc --noEmit --pretty false`: pass;
- repository ESLint: pass;
- production `vinext build`: pass (with only the existing large-chunk warning);
- every non-balance application test: 123/123 pass;
- balance-controller tests: 3/5 pass; the forward/rotated slow-pull case and
  planted reversal still enter recovery instead of returning to upright;
- the focused anatomy, structural-limit, threshold, finite-floor, neutral
  30-second standing, seven-region picking, and floorless-free-fall physics
  scenarios: 7/7 pass; and
- joint-motor/recovery-motor regressions: 21/21 pass, including measured
  free-assembly and one-stance Rapier response comparisons.

The balance investigation at that checkpoint identified a single-support global-yaw mode: the physical
root rotates while the landing preview remains world-fixed. Isolated hip-yaw,
sole-weld, and swing-target counter-rotation experiments all destabilized at
least one canonical direction and were therefore reverted.

Prone recovery now establishes shoulder clearance, deliberately unloads and
replants one arm, and enters contact-gated `push-brace`. It does not yet unload
the pelvis/lumbar into a balanced brace-only support polygon, so the five-pose,
25-second, repeated-cycle, obstruction, and complete dual-renderer recovery
gates remain unaccepted. A browser smoke run exercised actual WebGL and Canvas2D
standing/drag/fall/recovery-state rendering, but was stopped during the same
incomplete recovery and is not visual-acceptance evidence.

## Migration baseline

The pre-upgrade working tree, HEAD, dirty-file inventory, and existing failures were recorded before the 25-segment migration in [physical-humanoid-baseline-20260913.md](../evidence/physical-humanoid-baseline-20260913.md). Existing local recovery changes were retained.

At that checkpoint, the focused application suite had 67 passes and one known character-domain recovery failure after 30 simulated seconds. The earlier physics probes could stand and accept picks and gentle pulls, while several fast-pull and prone/supine/side/half-kneel recoveries remained failing. Those observations are a baseline, not acceptance of the new implementation.

## New physical-integrity gates

The report schema is now version 4. Its scenarios and per-update measurements cover:

- 25 unique segments, 72.2 kg total mass, and 1.84 m canonical stature;
- connected anchors, explicit collision exclusions, and canonical convex collider data;
- identical Canvas2D/WebGL physical poses and grouped seven-region selection;
- asymmetric structural limits under sustained torque with posture motors absent, including mirrored hinges, rotated headings, forearm rotation, ankle flexion, and combined shoulder/hip loading;
- one continuously dynamic Rapier body/collider assembly across standing, fall, and recovery;
- zero runtime transform, velocity, or body-type setter calls between initialization/reset and disposal;
- zero pose, velocity, or ownership jump at fall commitment and recovery completion;
- joint-coordinate, limit-error, motor-torque, saturation, and contact diagnostics;
- no direct pelvis assistance; and
- mass-weighted center-of-mass free fall when the floor is disabled.

The focused anatomy, structural-limit, convex-floor, renderer-pose, continuous-ownership, and free-fall smoke gates passed during the migration. Their local report is [physics-migration-smoke-results.json](../evidence/physics-migration-smoke-results.json). Full behavioral acceptance still depends on the complete generated `physics-results.json` from the final working tree.

## Behavioral and lifecycle coverage

The complete physics run exercises 30-second standing at headings 0, +pi/3, and -pi/4; gentle, step-provoking, and overpowering pulls; unreachable targets; crouch, half-kneel, prone, supine, and side recovery fixtures; five consecutive fall/recovery cycles; floor loss; obstruction retry; lockout trajectory isolation; fresh-press behavior; pause/resume; Reset; renderer switching; and disposal.

Acceptance thresholds and measurement semantics are documented in [physics acceptance](physics-acceptance.md). Controller mechanics are documented in [balance](balance-controller.md) and [dynamic recovery](dynamic-recovery.md).
