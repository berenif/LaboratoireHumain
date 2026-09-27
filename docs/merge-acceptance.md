# Merge acceptance history and gate map

For the latest recorded checkpoint, use [current status](status.md). Results below belong to the dated source scopes stated in each section; the gate tables are historical, not a current pass ledger. [Evidence availability](evidence.md) identifies recovered originals and missing artifacts. Paths marked unavailable are retained for provenance and cannot be used as live verification links.

## 2026-09-23 repair checkpoint: still not accepted

[The repair report](physics-repair-2026-09-23.md) records a continuous measured
contact-load allocator and increased internal solver convergence on top of
`44fc25437d841151c704895c1fb92de4c2e468d0`. Final focused checks pass **107/110**
(exit 1); balance remains **12/14**, with capture-instability falls at ticks
**177/229**. The balance probe and all three headings of the selected neutral
scenario still fail acceptance. TypeScript and lint pass. Full physics,
protocol, fixture refresh and publication remain gated. Prepared CI changes
and the original verification evidence are preserved. These uncommitted
results supersede earlier results only for the gates actually rerun.

## 2026-09-24 continuation: diagnosis only, still not accepted

The retained source was rechecked after a transfer/landing diagnosis. Focused
tests remain **107/110**, exit 1; the balance probe exits 1 with the same
capture-instability falls at ticks **177/229**. Runtime instrumentation found
repeated `capture-support` rejection of the active candidate while the
alternate is feasible only under the opposite measured sole load. The
candidate resets were therefore not removed: a rejected general debounce had
expired-transfer and balance regressions, and a temporary crossover landing
candidate failed two existing landing regressions while slow-pull remained at
tick 177 and reversal fell at tick 250.

Neutral evidence shows measured foot positions feeding the flat-floor support
target while the feet drift under loaded contacts. Fixed and fractional anchor
experiments did not meet drift acceptance. No production physics change was
retained; the landing planner was restored to its prior SHA-256. Detailed
commands, exits, source fingerprints, and traces are appended to
[the repair report](physics-repair-2026-09-23.md). Full physics, protocol,
browser gates, and native fixture refresh remain gated because focused
acceptance did not improve. No threshold, assertion, actuator ceiling, or
fixture was changed.

## 2026-09-23 post-merge verification: failed

The latest verification of `44fc25437d841151c704895c1fb92de4c2e468d0` is
documented in [the post-merge report](physics-verification-2026-09-23.md).
CI completed all 63 physics scenarios in 47 minutes 53 seconds: **6 pass,
57 fail, exit 1**. All 63 local worker results agree. Build and TypeScript
pass; the application suite passes **273/280**, focused checks **35/37**, and
the independent five-cycle protocol fails its first cycle. The balance failures
are at ticks **169/130**, superseding the earlier tick and test counts below.
Pages deployment was skipped. The report also records recovery of the local
trace aggregation after a disk-space error and the pending local CI workflow
improvements. Earlier checkpoint details below remain historical evidence.

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

## Execution checkpoint — 2026-09-23, not accepted

The merge remains in progress and publication is blocked by the balance gate.
Evidence: `evidence/merge-execution-20260923-160052/` (ignored). The original
objective was located in the saved attachment and copied into that directory.

| Check on the current controller | Result | Native exit |
| --- | --- | --- |
| Exact objective balance test command | 8/10; slow pull falls at tick 122, planted reversal at tick 139 | 1 |
| Balance transition probe | Both scenarios fail; neither reaches the required upright endpoint | 1 |
| New landing/load/frame diagnostic regressions | 6/6 | 0 |

The instrumented transition identifies **capture instability** in both current
failures, rather than the baseline pelvis-height guard. Slow pull fails at
2.050 s with pelvis height 0.9740 m; reversal at 2.333 s with pelvis height
0.8905 m. Both remain finite, connected, and Rapier-owned. The current changes
therefore do not establish successful physical balance, and regress earlier
than the preserved baseline (ticks 362/236).

This execution added explicit landing feasibility with a bounded 2-D search,
measured-pelvis moving-leg composition, contact-load diagnostic corrections,
and force/pressure consistency checks. These changes are **unaccepted work in
progress**. The remaining transfer problem is not solved: a selected moving
sole can remain almost fully loaded while the intended retained sole loses
load, despite a geometrically feasible landing. Joint-target consistency and
geometric feasibility alone do not establish an achievable support transfer.
Replanning between both sides during a stalled transfer also needs completion.

No fixture refresh, expanded automated acceptance, browser acceptance, commit,
push, PR, CI verification, or Pages deployment verification was performed in
this execution, because the prerequisite physical gate failed. No existing
acceptance assertion or physical threshold was weakened. Initial staged and
unstaged patches, source fingerprints, commands, exit codes, full traces, and
the evolving causal account are preserved under the evidence directory. The
original merge index was not staged or committed by this execution.

The historical checkpoint below describes the source **before** this run; its
passes do not apply to the current controller.

This ledger is for the in-progress merge of `3ca4cc86` into `main` at
`60b9e010`. It maps the documented production contract to the implementation
and evidence that must be checked on the **final, unchanged source tree**. A
source file, historical pass, or generated artifact is not itself an acceptance
result. Record the executed scenario count, exit code, source fingerprint, and
visual finding before changing any row to **Pass**.

Sources: [README](../README.md), [architecture](../ARCHITECTURE.md),
[physics acceptance](physics-acceptance.md),
[physical coherence](body-physical-coherence-acceptance.md),
[balance](balance-controller.md), [recovery](dynamic-recovery.md),
[playground](playground.md), [protocol visual replay](protocol-visual-replay.md),
`scripts/physics-fixtures.ts`, `scripts/physics-acceptance.ts`, and the tests
named below. Historical reports in `docs/validation.md` and related repair
notes supply leads, not final-source results.

**Pre-execution result: blocked.** On that source, focused balance passes 8/10;
slow pull enters `falling` at tick 362 and planted reversal at tick 236. The
balance transition probe also exits 1. Both transitions trip the unchanged
physical pelvis-height guard (< 0.56 m), with finite Rapier-owned bodies and
connected joints. The combined focused run passes 53/55; recovery support
passes 14/14, joint-motor and leg-frame regressions pass 16/16, and TypeScript
passes. All other rows below await final-source verification.
Do not publish or call the merge accepted while a required row is failed or
unverified.

The focused checkpoint used these SHA-256 source fingerprints:
`BalanceController.ts` `4E7565A22597A18B0521542715C822A312B1B5674B2EB128B9272DDDFD70083D`,
`EmbodiedCharacter.ts` `8BCB40E567ABBC4F835072EC51571C02B4C75E7DFD0078C1D6C898877E299699`,
`pose.ts` `02D63721808487BED9E430D8BDB73D00713EA448811BFEC26E8262892A465E4B`,
and `DynamicRecovery.ts` `23F1EFE3DFB81D97E971EAC2139D014A4EA6C300E4D113A1CF056E10DF53B526`.
These are diagnostic checkpoints, not final acceptance fingerprints.

### Open balance diagnosis

Fresh worker replay at `evidence/merge-acceptance-20260923/worker-final-baseline/`
reproduces 8/10 and probe exit 1 without changing the source. The slow pull
finishes three genuine steps, then starts a fourth with only about 203 N on
its intended retained sole, below the required absolute 52% load. Its transfer
lasts until the forward capture point has escaped the support patch. In the
reversal, the third step launches after the pelvis has pitched backward; the
rear hip reaches its −20° extension limit. The later-step feasibility search
returns the starting foot position when no point along its search ray is
feasible, even though the joint-limited pose misses that returned target by
84 mm. Off-ray reachable points do not catch the backward COM at that time.
The observed transitions are physical pelvis-height falls, not broken contact
or joint integrity. Step timing, reachable placement, and motor response need a
production correction before the remaining gates can run.

## Historical contract and source map

| ID | Required behavior and fixed limit | Production path | Automated evidence | Browser evidence | Result |
| --- | --- | --- | --- | --- | --- |
| P1 | 25 connected dynamic segments, 72.2 kg, 1.84 m; lumbar, girdles, split forearms, articulated soles; seven selectable regions. | `src/core/humanoid.ts`, `src/core/geometry.ts`, `src/character/EmbodiedCharacter.ts` | `anatomy`, `body-physics`, `body-coherence`, `physical-chain-integrity`, physics harness anatomy scenario | Pick all region types in both views. | Pending |
| P2 | Canonical convex surfaces agree across Rapier collision, picking, WebGL, and Canvas2D; exact picked segment and local anchor survive grouping. | `src/core/geometry.ts`, `src/interaction/`, `src/scene/` | `anatomy`, `character-domain`, `body-physics`, `limb-collisions`; harness geometry and seven-region scenarios | Interaction replay in both renderers. | Pending |
| P3 | Anatomical joint frames and mirrored limits remain connected and covariant at rotated headings (initial positions within 0.1 mm; hindfoot forward error < 2°); anchor gap ≤ 0.08 m, focused collision gap ≤ 0.01 m, and structural-limit error ≤ 0.06 rad. | `src/core/humanoid.ts`, `src/character/joint-motors.ts`, `src/character/native-joint-motors.ts`, `src/character/pose.ts` | `joint-coordinates`, `rapier-joint-adapter`, `joint-motors`, `leg-target-frame`, `body-coherence`; harness structural-limit scenario | Inspect knees, elbows, feet, and joint continuity in recovery replay. | Pending |
| P4 | Real nonadjacent self-collision, including upper arm against trunk; floor depth ≤ 0.08 m and general self-penetration ≤ 0.005 m. Added coherence cases require initial overlap ≤ 1 mm, slow/fast contact peak ≤ 15 mm, and 12-frame sustained penetration ≤ 5 mm. | `src/core/humanoid.ts`, `src/core/geometry.ts`, `src/character/physics-settings.ts`, `src/character/EmbodiedCharacter.ts` | `limb-collisions`, `body-physics`, `body-coherence`, `physical-chain-integrity`; harness floor and cross-body scenarios | Inspect pulls, floor contact, and body overlap from side and three-quarter views. | Pending |
| P5 | Measured Rapier mass sums to 72.2 kg within 0.0001 kg, positive inertia, COM agrees with `worldCom()` within 1 µm, passive free-fall acceleration agrees with gravity within 0.1 m/s²; finite diagnostics and bounded permitted-axis motors; no planned-only support. | `src/character/mass-state.ts`, `src/character/support-loads.ts`, `src/character/native-joint-motors.ts`, `src/character/EmbodiedCharacter.ts` | `body-physics`, `body-coherence`, `contact-loads`, `joint-motors`, `recovery-measured-mass`; full harness | Inspect diagnostic report for finite values and measured contacts. | Pending |
| P6 | Rapier owns the same bodies, colliders, pose, and momentum through stand, fall, and recovery; no runtime setter/reset; floor loss clears load and COM falls ballistically; direct pelvis force and torque stay exactly zero. | `src/character/EmbodiedCharacter.ts`, `src/character/DynamicRecovery.ts`, `src/character/BalanceController.ts` | `physical-chain-integrity`, `body-physics`, `recovery-controller-support`; harness ownership and free-fall scenarios | Inspect transition continuity in full interaction/recovery replays. | Pending |
| P7 | Native fixed streams keep historical commands, transfers, and integrity hashes; only declared initial assembly snapshots/migration metadata may change. | `scripts/fixtures/native-fixed-streams.json`, `scripts/refresh-body-initial-snapshots.ts` | `native-fixed-streams`; refresh script and complete fixture diff | — | Pre-refresh read-only comparison: both fixture IDs and all 1,235/1,194 historical updates plus both transfer arrays match `HEAD` exactly; refresh and final diff pending |
| B1 | Quiet standing at 0, +π/3, and −π/4 after 2 s settling: 30 s with no corrective steps, pelvis drift ≤ 0.03 m, planted sole drift ≤ 0.01 m, linear/angular speed ≤ 0.1 m/s / 0.5 rad/s. | `src/character/BalanceController.ts`, `src/character/EmbodiedCharacter.ts` | `balance-controller`, `balance-support`; harness `idle-30-seconds` | Standing frames in both renderers. | Pending |
| B2 | Gentle pulls at tested headings, held targets, reversal, and release during swing remain upright; stronger pulls make completed corrective steps. | `src/character/BalanceController.ts`, `src/character/pose.ts`, `src/character/leg-target-frame.ts` | `balance-controller`, `balance-support`; all `PULL_FIXTURES`, native regression fixtures, and fall-transition probe | Full interaction replay, including pull, step, release, and fresh press. | **Fail:** balance 8/10; slow tick 362 and reversal tick 236 |
| B3 | Step side and target use measured support, COM/velocity, and reachable terrain. Swing needs ≥ 52% of **whole-body weight** on the retained measured sole for 0.10 consecutive seconds; a completed step needs actual moving-side unloading followed by ≥ 0.10 s loaded touchdown within 0.09 m of target. A timer or planned load cannot complete it. | `src/character/BalanceController.ts`, `src/character/support-loads.ts`, `src/character/pose.ts` | `balance-controller`, `balance-support`, `leg-target-frame`; fall-transition probe with contact/COM/motor trace | Confirm actual lift, landing load, and no unfinished swing at probe endpoint. | Open with B2 |
| B4 | Overpowering pulls cause a physical protective fall under the existing support-margin, speed, posture, and pelvis-height guards; no masking or threshold relaxation. | `src/character/BalanceController.ts`, `src/character/EmbodiedCharacter.ts` | `balance-controller`, `physical-chain-integrity`; fast/sustained pull harness scenarios | Full fall and recovery replay. | Pending |
| R1 | Landing, settling, brace, kneel, and standing transitions use persistent measured solver contacts (normal Y ≥ 0.65, distance ≤ 0.012 m, load ≥ 3 N for 0.05 s), anatomical eligibility, pose, and speed. Head/neck cannot authorize progression; initial sprawled prone hand is rejected while valid raised support is allowed. | `src/character/DynamicRecovery.ts`, `src/character/recovery-support.ts`, `scripts/recovery-measurements.ts` | `recovery-support`, `recovery-controller-support`, `recovery-measurements`, `recovery-arm-kinematics`; full harness | Inspect phase and contact traces from side and three-quarter views. | Targeted 14/14 pass on current source; full harness pending |
| R2 | Deliberate hand/foot placement respects reach, joint limits, terrain, collision clearance, and actual support. A release requires COM projected 0.15 s forward inside the remaining loaded patch hull; material-patch drift > 0.05 m invalidates an anchor until unload/replant. Unsupported > 0.20 s or stalled > 3 s retries without pose replacement. | `src/character/DynamicRecovery.ts`, `src/character/recovery-support.ts`, `src/character/recovery-joints.ts` | `recovery-controller-support`, `recovery-foot-targets`, `recovery-forefoot-anchor`, `recovery-motors`; obstruction and floor-loss harness scenarios | Inspect foot slip, lift source, retries, and continuity in full recovery videos. | Pending |
| R3 | Crouch, half-kneel, prone, supine, and both sides, including mirrors/rotations and natural falls, recover on unobstructed floor within 25 simulated seconds and hold one second of stable bilateral standing (pelvis > 0.93 m; pelvis/ribcage up Y ≥ 0.97; low RMS motion). | `src/character/DynamicRecovery.ts`, `scripts/recovery-fixtures.ts` | `recovery-fixtures`, full harness pose/native-stream/natural-fall scenarios | `run-recovery-visual-replay.mjs` in both renderers, side/three-quarter, normal/slow. | Pending |
| R4 | Five consecutive fall/recovery cycles without Reset; floor removal/restoration, obstruction retries, pause/Reset, lockout trajectory isolation, and fresh press after recovery. | `src/character/EmbodiedCharacter.ts`, `src/demo/DemoRuntime.ts`, `src/interaction/` | harness five-cycle/lifecycle scenarios; `character-domain`, `demo-runtime`, `recovery-controller-support` | Full interaction replay; full recovery replay. | Pending |
| T1 | `/` is the protocol by default. Visible strike button and `P` shortcut each request one attempt; pause/busy actions are neither executed nor queued. Sound is optional. | `src/demo/EmbodiedDemo.tsx`, `src/ui/ProtocolPanel.tsx`, `src/character/PhysicsStriker.ts` | `protocol-striker`, `demo-runtime`, `rendered-html` | Open `/`, exercise button, shortcut, pause, sound, desktop/mobile controls. | Pending |
| T2 | Striker positioning and retraction clear real geometry; impacts come from measured collision, misses stay misses, and repeat attempts preserve body ownership. Counters, phase/status, camera controls, and renderer switch remain truthful. | `src/character/PhysicsStriker.ts`, `src/core/protocol.ts`, `src/ui/ProtocolPanel.tsx`, `src/scene/protocol-visuals.ts` | `protocol-striker`; `protocol-five-cycle-acceptance.mjs` | `verify-protocol-visual.mjs` and three-second impact replay; inspect actual contact and retraction. | Pending |
| T3 | Five complete strike/fall/recovery/one-second-stable-return cycles, within 25 s after impact, without Reset. | `src/character/PhysicsStriker.ts`, `src/character/DynamicRecovery.ts` | `protocol-five-cycle-acceptance.mjs` | Full recovery evidence; three-second protocol replay alone is insufficient. | Pending |
| G1 | `?mode=playground` exposes seven stations and Gentle/Challenging/Extreme, using physical terrain and shared render geometry. Feet spawn clear; support queries and destination heights follow terrain. | `src/core/playground.ts`, `src/character/PhysicsPlayground.ts`, `src/demo/DemoRuntime.ts`, `src/ui/PlaygroundPanel.tsx` | `playground`, `contact-loads`, `character-domain` | `verify-playground.mjs` at explicit playground URL, all stations, both views. | Pending |
| G2 | Deck moves on simulation time; pause freezes it, Reset restores it; station/difficulty changes start trials and preserve pause. Upright/best/falls/loaded-feet metrics and camera controls are accurate, controls stay usable on mobile. | `src/core/playground.ts`, `src/demo/DemoRuntime.ts`, `src/ui/PlaygroundPanel.tsx` | `playground`, `demo-runtime`, `fixed-step-budget` | Playground browser report and 390 × 844 visual review. | Pending |
| U1 | WebGL/Canvas2D show the same physical snapshot and support picking/dragging, orbit/pan/zoom, responsive controls, renderer switch, and public motion states (`upright`, `reacting`, `stepping`, `falling`, `fallen`, `recovering`) and recovery phases (`protect`, `settle`, `roll`, `brace`, `kneel`, `stand`). | `src/scene/`, `src/interaction/`, `src/demo/`, `src/ui/` | `character-domain`, `demo-runtime`, `browser-replay`, `ui-components` | Full interaction replay and recovery review in both renderers. | Pending |
| U2 | Fixed-step input cleanup, pause/resume/reset, visibility/focus handling, cancellation, and idempotent disposal preserve the single live runtime. | `src/demo/DemoRuntime.ts`, `src/demo/useDemoRuntime.ts`, `src/interaction/` | `demo-runtime`, `fixed-step-budget`, `character-domain` | Exercise pause, Reset, renderer switch, and lockout in browser. | Pending |
| D1 | Vinext build, complete tests, physics/protocol harnesses, typecheck, lint, and root/subpath Pages exports pass on final source; published `main` and CI/Pages reflect the accepted commit. | `package.json`, `scripts/verify-pages.mjs`, `.github/workflows/deploy-pages.yml` | Full gate ledger below | Open both exported routes and inspect images/reports. | Pending |

Historical **65%** transfer prose in the physical-coherence document has been
reconciled with the governing **52% of total body weight** retained-load
threshold. Actual unloading and loaded touchdown remain separate requirements.
Recovery on every Extreme obstacle, native-touch
certification, and GPU-performance certification are outside this acceptance
contract.

## Historical final-source gate ledger

Use `C:\Users\flori\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe`
for every Node entrypoint and child process. Capture each native command's exit
code immediately. Clear diagnostic scenario filters and list-only settings
before the full harness; the report must show every expected scenario executed.
Build Vinext before `tests/*.test.mjs`, because rendered-page tests import it.

| Gate | Required command or review | Count / exit / evidence |
| --- | --- | --- |
| Focused balance and recovery | `--test tests/balance-controller.test.mjs tests/recovery-support.test.mjs tests/balance-support.test.mjs tests/body-coherence.test.mjs tests/body-physics.test.mjs`; `scripts/probe-balance-fall-transition.mjs` | **Fail:** combined 53/55, exit 1; only balance fails (slow tick 362, reversal tick 236). Probe exit 1 with the same physical-height failures; see `evidence/merge-acceptance-20260923/measured-fourth-leg-frame/`. Separate `recovery-support` 14/14 and `joint-motors` + `leg-target-frame` 16/16 pass, all exit 0. Rerun after balance fix. |
| Native fixture | `--import tsx scripts/refresh-body-initial-snapshots.ts --write`; inspect complete diff, then `--test tests/native-fixed-streams.test.mjs` | Pending |
| Static checks | `node_modules/typescript/bin/tsc --noEmit --pretty false`; `scripts/run-tool.mjs eslint . --ignore-pattern dist --ignore-pattern .next` | TypeScript exit 0 on current source; lint pending. Rerun both on final source. |
| Application build and tests | `scripts/run-tool.mjs vinext build`; then `--test tests/*.test.mjs` | Pending |
| Full physics and protocol | `--import tsx scripts/run-physics-harness.ts`; `scripts/protocol-five-cycle-acceptance.mjs`; final balance probe | Pending |
| Pages root | Empty `PAGES_BASE_PATH`: `scripts/run-tool.mjs next build`; `scripts/verify-pages.mjs` | Pending |
| Pages project path | `PAGES_BASE_PATH=/LaboratoireHumain`: same build and verify commands; restore previous environment afterward | Pending |
| Playground browser | Explicit `?mode=playground` route; `scripts/verify-playground.mjs`; inspect desktop/mobile screenshots and result JSON | Runner route repaired; execution and visual review pending |
| Interaction and recovery browser | Full `scripts/run-visual-replay.mjs full` and `scripts/run-recovery-visual-replay.mjs`; inspect both renderers, side/three-quarter, normal/slow, reports and videos | Interaction runner route repaired; execution and visual review pending |
| Protocol browser | Default `/` route; `scripts/verify-protocol-visual.mjs` and `scripts/run-protocol-visual-replay.mjs`; inspect impact/retraction, desktop/mobile, report and screenshots | Pending; short replay is insufficient for recovery |
| Publication | Review full staged/unstaged merge and fixture diff, scan for secrets/conflict markers, `git diff --cached --check`, no unmerged entries; commit, fetch, push or protected PR, inspect CI and Pages | Pending |

Store browser evidence in an ignored run-specific directory. Record source
fingerprints before and after each run, command arguments, scenario counts,
exit codes, and visual findings in that directory and link them here. Any source
change invalidates affected results; rerun those gates before publication.

## 2026-09-24 focused physics repair — final retained-source ledger

| Gate | Result | Evidence |
| --- | --- | --- |
| Focused 110-test selection plus landing-capture, transfer-prediction, joint-coordinate suites | **Fail: 140/142, exit 1.** Slow-pull/release/reversal falls at tick 216; planted reversal falls at tick 343. All other selected tests pass, including the existing impulse assertion. Source fingerprints match before and after, including after candidate rollback. | `focused-restored-retained-source.json` (`evidence/physics-repair-20260924-completion/focused-restored-retained-source.json`; unavailable), log (`evidence/physics-repair-20260924-completion/focused-restored-retained-source.log`; unavailable) |
| Balance probe | **Fail: exit 1.** Slow-pull has an infeasible candidate and opposite requested/allocated horizontal force at its failure frame, with left thigh saturation 3.074×; planted reversal ends at the unchanged pelvis-height guard. Both runs retain finite connected Rapier-owned bodies. | Final check manifest (`evidence/physics-repair-20260924-completion/final-restored-source-checks.json`; unavailable), log (`evidence/physics-repair-20260924-completion/balance-probe-restored-final-source.log`; unavailable); raw traces are named in the manifest. |
| Official idle scenario | **Fail: exit 1, 0/1 scenarios accepted.** Required 2 s settling plus 30 s observation was run at 0, +π/3 and −π/4. All runs stayed upright with zero steps, but pelvis drift, foot drift, linear speed, and angular speed exceed limits at every heading. | `official-idle-final-source-rerun.json` (`evidence/physics-repair-20260924-completion/official-idle-final-source-rerun.json`; unavailable), log (`evidence/physics-repair-20260924-completion/official-idle-final-source-rerun.log`; unavailable) |
| Articulated impulse response | **Focused hip/ankle assertion passes with its original tolerance.** A 36-case paired heading/sign/magnitude sweep has one 15.23% right-thigh mismatch at the smallest 0.0005 Nm·s impulse; it remains recorded as an unresolved low-impulse diagnostic outlier. | `joint-response-paired-static-probe.json` (`evidence/physics-repair-20260924-completion/joint-response-paired-static-probe.json`; unavailable), trace (`evidence/physics-repair-20260924-completion/joint-response-paired-static-probe.jsonl`; unavailable) |
| Combined stance posture/reference candidate | **Rejected and reverted.** Focused balance/controller + body coherence was 27/30; it delayed the two falls but caused a 0.03035 m joint separation in left-hand chest drag. The diagnostic idle attempt failed all headings and recorded 0.074518 m self-penetration; output JSON was not saved because its temporary output directory was absent. | `stance-reference-transition-candidate.json` (`evidence/physics-repair-20260924-completion/stance-reference-transition-candidate.json`; unavailable), log (`evidence/physics-repair-20260924-completion/stance-reference-transition-candidate.log`; unavailable) |
| TypeScript | **Pass, exit 0.** | `typecheck-restored-final-source.log` (`evidence/physics-repair-20260924-completion/typecheck-restored-final-source.log`; unavailable) |
| ESLint | **Pass, exit 0; 0 errors and 6 warnings.** | `eslint-restored-final-source.log` (`evidence/physics-repair-20260924-completion/eslint-restored-final-source.log`; unavailable) |
| `git diff --check` | **Pass, exit 0.** The two appended reports also passed a trailing-whitespace scan. | Final check after both report edits. |

The source remained stable within each focused test, probe, and official idle
run. Focused-run SHA-256 fingerprints include `BalanceController.ts`
`329d2f40cfde8329cc0fbef42aa867ed8fe8b36170b687df1d2bc0377b3bad0a`,
`EmbodiedCharacter.ts`
`e76e52091d901e561936e27393532ea06d53c5328623f20385cca45126826f02`, and
`support-loads.ts`
`9637a126ffa827f8523cec06013552635f5e154ecf40927783ace4bc0f2d1a2a`; full
before/after maps are in the linked manifests. No stance-reference/transfer
controller correction passed the required regressions, so none was retained.
The focus repair is **not accepted**. Full application, physics, protocol,
build/browser, and Pages gates remain a separate phase. Existing continuous
allocator, solver settings, thresholds, actuator ceilings, fall guards,
Rapier ownership, and native fixtures were preserved. No commit, push, or
deployment was made.
