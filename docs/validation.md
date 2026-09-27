# Historical validation record

[Current status](status.md) is the authoritative entry point for the latest recorded checkpoint. This file preserves earlier results and implementation history. Consult [evidence availability](evidence.md) before relying on local artifact references; paths marked unavailable are historical retrieval leads.

## Historical publication checkpoint

The September 23 WIP publication and its 34/36 focused result are preserved in
[the merge history](merge-acceptance.md#2026-09-23-publication-checkpoint-work-in-progress).
Publication did not establish acceptance. See [current status](status.md) for
subsequent checkpoints and [evidence availability](evidence.md) for artifact scope.

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

## Historical investigation (2026-09-19)

Physics acceptance remains unachieved. The current balance trace loses measured
stance support during the first weight transfer, before the second step is
released. A passive articulated-response correction passes the 16 cross-body
paths, but other pull and recovery scenarios still fail. Targeted passes do not
establish full acceptance. The running validation ledger, per-frame evidence,
rejected experiments and visual artifacts are indexed in
the September 19 record (`evidence/limb-20260919/validation-summary.md`; unavailable).
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

The pre-upgrade working tree, HEAD, dirty-file inventory, and existing failures were recorded before the 25-segment migration in physical-humanoid-baseline-20260913.md (`evidence/physical-humanoid-baseline-20260913.md`; unavailable). Existing local recovery changes were retained.

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

The focused anatomy, structural-limit, convex-floor, renderer-pose, continuous-ownership, and free-fall smoke gates passed during the migration. Their local report is physics-migration-smoke-results.json (`evidence/physics-migration-smoke-results.json`; unavailable). Full behavioral acceptance still depends on the complete generated `physics-results.json` from the final working tree.

## Behavioral and lifecycle coverage

The complete physics run exercises 30-second standing at headings 0, +pi/3, and -pi/4; gentle, step-provoking, and overpowering pulls; unreachable targets; crouch, half-kneel, prone, supine, and side recovery fixtures; five consecutive fall/recovery cycles; floor loss; obstruction retry; lockout trajectory isolation; fresh-press behavior; pause/resume; Reset; renderer switching; and disposal.

Acceptance thresholds and measurement semantics are documented in [physics acceptance](physics-acceptance.md). Controller mechanics are documented in [balance](balance-controller.md) and [dynamic recovery](dynamic-recovery.md).
