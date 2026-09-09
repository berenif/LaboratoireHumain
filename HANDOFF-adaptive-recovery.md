# Adaptive natural recovery — continuation handoff

Prepared 2026-09-09 for the next chat. Workspace: `O:\LaboratoireHumain`. Branch: `main`. Baseline HEAD: `8ee2b7573ebd2af44a2df58988eeb51db82eed5c`. All changes are local and uncommitted. No deployment, publishing, PR, or new Codex task was created.

## Read this first

**The requested recovery rewrite is not complete or accepted.** Significant controller, geometry, diagnostics, test, and replay work is implemented, but production prone, supine/side, and half-kneeling recovery still fail. Do not present geometry tests, a phase label, or the one fragile experimental stand-up as proof of success.

The latest user instruction was: “Finish work and write everything you did to pass it to another chat. don't create another subagent”. New tuning stopped, the three existing agents finished their in-flight checks and supplied their findings, and this handoff was written. No additional agent was created after that instruction. Existing agents are finished; no further tuning should be assumed to be running.

For the next chat, start with this document, then read the production controller and the three detailed supplements:

- `src/character/DynamicRecovery.ts`
- `evidence/prone-experiment-handoff.md`
- `evidence/recovery-20260908/half-kneel-integration.md` — **read the final appendix**, which corrects important scope differences in the earlier integration advice.
- The rolling findings in this handoff and `scripts/experimental-roll-recovery.ts` / `scripts/probe-roll-recovery.ts`.

The source tree contains retained experimental controllers and probes. They are useful diagnostic work, **not production integrations**. The application imports `src/character/DynamicRecovery.ts`, not the experimental copies. The evidence directory is gitignored, so it must be copied separately if moving this work to another machine or checkout.

## User's required result and unchanged boundaries

Implement adaptive support-first recovery after the original settling interval:

1. Balanced feet underneath: lean as needed and rise from crouching.
2. One foot plus opposite knee: transfer to the planted foot, extend it, bring the trailing foot underneath.
3. Prone with arms available: brace through hands/forearms, tuck a knee, establish a leading foot.
4. Supine/side: roll toward usable arm support, then brace and kneel.

Use established support before torso orientation. Prefer a planted foot, otherwise least reachable movement; choose roll side by existing arm support then floor proximity, with deterministic ties and retained choices until retry. Advancement must depend on actual loaded contact, joint motion, and balance, with repositioning/retries for weak support.

Keep Rapier in control of all dynamic transforms. Use the existing two-bone solver for reachable hand/foot geometry, then relative joint rotations and bounded equal-and-opposite torques. Capture established support targets and keep them fixed until deliberate release or support loss. Before deliberately releasing support, mass-weighted COM projected **0.15 s** forward must lie inside the actual remaining loaded contact hull. Kneel includes the leading-leg rise and trailing-foot placement; enter stand only with actual trailing-foot support and all original entry conditions, then finish extension and relax arms.

Remove the old upward rolling target and external 88% body-weight pelvis compensation. Maximum residual upward assistance is **20% BW = 141.6564 N** for the unchanged 72.2 kg character, zero during rolling or inadequate support. Residual pelvis torque maximum **60 Nm**. Joint ceilings remain: torso/thigh/shin110 Nm; upper arm30; forearm20; foot65; hand9; neck/head22.

Preserve proportions, joint topology, controls, 60 Hz integration, original public motion states and recovery phases, stable-standing handoff, retries, input lockout, pause/reset, missing-floor behavior, repeated cycles, continuity/geometry checks, and **25-second recovery acceptance on unobstructed floor**. No deployment changes. Numerical success alone is insufficient; review WebGL and Canvas2D, side and three-quarter, normal and slow playback.

## What is integrated into production

### Controller structure and diagnostics

`DynamicRecovery` now separates route selection (`chooseRoute`), support planning/transfer (`transfer`, `release`), reachable limb targets (`limbTargets`), joint actuation (`actuate`), and bounded external assistance (`assist`). Existing public states/phases remain.

`RecoveryDiagnostics` keeps old fields and adds route, leadingSide, rollSide, transferStage, supportMarginM, centerOfMass, projectedCenterOfMass, plantedTargets, releasedSupports, releaseMarginM, extension, progressError, and noSupportTimeS. Existing replay code now reads these public fields rather than private `bestError`, `rise`, or `targetOrigin` controller internals.

Route choices and selected sides remain fixed until retry. Reset clears support/trajectory state. Returning to settle clears deliberate-release and unload history; retry waits for the original settling evidence before recalculating a route.

The controller captures all relative entry rotations and pelvis rotation. It blends targets from those measurements, normalizes quaternion sign, chooses the legal YXZ decomposition, and preserves positive knee/negative elbow intent. No animation transform is assigned to a dynamic body.

### Contacts and deliberate releases

`observe` reads actual solved floor contact points and impulses, copying WASM-returned vectors before another query can reuse scratch storage. Contact normalY>=.65, solver distance<=.012 m, load>=3 N for .05 s establish support. Established plants capture position, rotation, and copied contact point. They remain fixed despite sliding; loss grace is .10 s.

`release` requires projected COM inside the remaining loaded support polygon when releasing existing/loaded support. Already released contacts cannot authorize later releases, even if their old manifold still has load. A released limb must be observed unloaded and then acquire three fresh loaded frames before it can be captured again. An adjacent forearm/shin release cannot erase the first limb's unload history. Regression tests cover all of these cases.

A late reporting correction retains the actual applied force/torque after `observe`, even if contact disappears during that integration. Previously, post-observe clearing could hide a real last-step impulse. `apply` resets force/torque at the next decision, so the next unsupported step stops assistance immediately. Explicit floor disable already invalidates contact before `apply` in `EmbodiedCharacter`.

### Reachable support geometry

`src/character/recovery-support.ts` contains mass-weighted COM/velocity, convex hull and signed support margin, contact-patch geometry, deterministic route/side selection, arm usability, and reachable arm planning.

- Prefer actual solver contact points. The fallback for a box uses only its low contacting edge/face, not an invented full horizontal sole or a padded disk.
- Degenerate point/line support has no area.
- `usableRecoveryArmSupport`: actual hand contact lever<=.36 m from shoulder, forearm<=.335 m, with shoulder/contact height checks.
- `reachableArmBraceTarget`: shared two-bone geometry, true oriented hand-box floor height, shoulder/elbow/wrist limits, measured-body/heading-aware bend; returned `floorReachable` is insufficient alone — also require negligible `jointLimitErrorRad`.
- Final arm candidate scoring uses actual lowest hand corner/edge with .32 m lever reserve. Earlier center-only scoring selected a far-side hand whose center was legal but floor contact was too far from the shoulder.
- Narrow bend uses heading-lateral .20 and measured torso-back .65, replacing the overly broad .85 lateral bend.
- `solveRecoveryArmTarget` treats forearm plus fixed local wrist/hand as an exact effective second bone, preserving wrist bounds along a raised clearance arc. It returns upper/forearm/hand world rotations and exact reached endpoint. Quaternion-sign wrapping prevents mirrored/yawed q/-q cases from choosing the opposite wrist bend.

`src/character/recovery-foot-targets.ts` searches reachable foot floor placements with shared two-bone geometry and legal hip/knee/ankle rotations. It returns feasibility and an explicit legal reachable fallback when the floor cannot be reached; no fictitious planted support is created. Captured feet are preferred before least-travel alternatives.

`pose.ts` only exports the existing `solveTwoBone`; proportions and topology were not changed.

### Bounded coupled motor solver

`src/character/recovery-motors.ts` replaces independent high-gain torque updates with a simultaneous implicit PD solve using copied world inverse-inertia tensors. Shared-body coupling is included. Each motor intent has id, parent, world error/relative velocity, kp/kd, feedforward, and its old cap. Equilibrium feedforward preserves static load. Bounded 3x3 block solves propagate saturation to neighboring joints rather than clamping afterward; maximum96 sweeps with a1e-8 convergence threshold.

The root pelvis orientation motor joins the same solve, with parent=null and60 Nm cap. All other torque impulses have equal-and-opposite parent impulses. Tests independently check simple closed-form systems, energy/angular-momentum behavior, anisotropic inertia, equilibrium feedforward, and cap-coupled stationarity.

Current active gains: torso kp4500; large leg/foot kp2800; neck/head1500; arms500. Damping: large85; foot35; neck/head12; arms16. Root kp4500/kd160. Passive gains are lower, except an already supported crouch/half-kneel holds its measured entry configuration. Preserve caps while tuning gains.

### Current route planning details

- Supported crouch can skip directly to stand after settling, but only with two persistent soles, sole-up>.85, torso-up>.88, pelvis>.55 m, and projected COM within actual sole hull.
- Initial prone arm placement runs within the existing roll phase while holding measured root/torso/leg targets. It plans each hand from measured entry through a .15 m clearance path over roughly .8–1.15 s, releases with the same COM guard, and captures actual recontact. Feedforward is disabled during this initial preparation. Rolling position assistance is zero.
- A planted trailing shin in kneel pivots around its fixed capsule contact instead of freezing its entire old orientation; pitch1.8 and ankle-.65 permit the trailing toe to reach the floor. Release the shin only after toe load and the guard.
- Kneel initially rises over the entry root XZ, then blends toward mass transfer over the leading ankle as pelvis height grows from .65 m. Torso relative lean relaxes during ascent.
- Current production kneel goal is **.80 m**, virtual root advance **.10 m**, active kneel internal ground-force budget approximately .8BW plus0–30 N; upward assistance gains **1500/150 during kneel only** and600/100 elsewhere. These differ from the one fragile experimental full recovery below.
- Trailing-foot release currently needs leading load>.55BW, vertical COM velocity>-.05, pelvis>.65, and the exact COM release guard. A .42 s/.12 m clearance arc and .22 s trailing-joint blend guide the step. Accidental rear toe recapture is deliberately re-released only if the guard still passes.
- **Known missing production action:** brace sets the plant-lead stage after trailing-shin support but does not actually perform the required guarded leading-foot release/reposition. The experimental prone copy attempts that action, but never achieves the preceding knee support reliably.
- **Known production roll problem:** target orientation still uses original heading with a fixed roll/pitch composition. This can turn the long body axis across the floor/end-for-end rather than roll around it. The improved measured-axis version remains experimental.

## Validation implemented and current results

New deterministic fixture file `scripts/recovery-fixtures.ts` defines24 landed cases: prone, supine, side, strong half-kneel, crouch, plus weak half-kneel; both sides and headings0/pi3. Fixture names are `landed-prone-left`, `landed-prone-left-heading`, etc. **There is no `-heading-0` suffix.** Seeded poses preserve anatomical anchors, use true oriented collider floor height, and have zero initial momentum. Setup-only seeding precedes integration; the controller is still forbidden to set dynamic transforms.

`scripts/recovery-measurements.ts` independently reads raw Rapier manifolds and measures COM, .15 s projection, loaded contacts, releases, leading load share, limb extension, knee/elbow reversal, actual planted material-point drift, and whether captured target positions move. It does not accept a phase label as physical proof.

`physics-acceptance.ts` retains old time, continuity, geometry, input, reset/pause, finite/missing-floor, obstruction, and repeated-cycle checks and adds the new cases/measurements. Route shortcuts are accepted only with equivalent measured support/pose conditions. An empty scenario regex now produces an explicit no-matching-scenarios error instead of querying uninitialized Rapier WASM.

A late harness sampling fix gates COM diagnostic equality on an actual recovery integration: integrated time plus previous authority==ragdoll. Dynamic activation occurs after the upright clock advanced, resets recovery metadata, and does not yet call `observe`; that same-instant handoff must not be mistaken for a recovery integration. Natural-fall failures in the earlier full intermediate report include this false-positive COM issue. The following dynamic step still checks COM exactly. This fix does not waive recovery or support requirements.

Verification completed during closeout:

- Whole-repository TypeScript check: PASS.
- Whole-repository ESLint: PASS. The retained temporary private-state probe has a file-local explicit-any exemption; production does not.
- `npm test` build phase: PASS; Vite reports existing bundle-size/classification warnings, no build failure.
- Final application-test run: **83 tests,82 passed,1 failed**. Log: `evidence/recovery-20260909/final-application-tests.log`. Failure is `tests/character-domain.test.mjs:131`, assertion at152: recovery remains ragdoll instead of returning to character-motor. Do not weaken that assertion.
- Focused assistance/support/measurement suite:19/19 passed after last force reporting changes.
- Support geometry12 tests, motor8 tests, reachable foot5 tests passed; fixture and controller support tests are included in the application run.
- Earlier production crouch run: all4 mirrored/yawed cases recover in2.0167–2.05 s, no retries/reversals/force violations; max measured plant drift2.16–3.37 cm. Evidence: `evidence/crouch-tuning-production.json`. This was an intermediate source revision; rerun after final route work.
- Final full physics harness result and selected follow-up are appended under Closeout below. Recovery acceptance is FAIL regardless of the sampling correction.

## Experimental half-kneel result — not integrated as accepted tuning

Files: `scripts/experimental-load-recovery.ts`, `scripts/probe-load-recovery.ts`, `scripts/probe-half-kneel-best.ts`. Full details: `evidence/recovery-20260908/half-kneel-integration.md`, especially Final session result.

Only one numerically complete variant:

```ts
{feedforwardScale:1, balancedRise:true, groundBudget:true,
 smoothTransfer:true, fastTrail:true, clearanceArc:true,
 retryToe:true, uprightRise:true, kneelHeight:.92, rootAdvance:.15}
```

Unlisted options are false/default. Swing duration .42 s and trailing blend .22 s. **Clone assistance1500/150, .15 m root advance, and .8BW+bounded30N ground demand continue through stand as well as kneel.** Earlier advice to apply these only in kneel did not reproduce the completed trace. Production still has the narrower kneel-only/.80/.10 integration.

`landed-half-kneel-left` returns to stable handoff at5.2167 s and holds character-motor for1 s. Max upward141.6564 N, root torque60, joint torque110, joint gap10.8 mm, penetration9.3 mm. Trailing shin releases at .7333 s after leading foot472 N/toe71 N. Toe release1.2667 s had only+0.000797 m remaining margin. Leading knee bend2.104→.491 rad.

**Other3 mirrored/yawed cases fail25 s.** Even the successful case has17.43 cm trailing-foot drift during weak stand contact at1.97–2.18 s (load11.2 N, centerY.112, soleUp.927). That is unacceptable physical/visual behavior.

Tighter release height>.85/margin>.015:0/4. Slower .75 s swing with stronger measured sole-entry checks:0/4. Latest .65 s swing +stableSole+worldTorso:0/4; worldTorso compensates measured pelvis rotation in the relative spine target, improves torso-up to .995 and gets both unrotated cases into stand, but they fall afterward. Rotated cases still fail. Actual-assistance-subtracted ground budget overshoots; predictive/continuous-lead/inertia variants did not solve it.

Evidence: `half-kneel-best-results.json` and `best-landed-half-kneel-*.json.gz`; `guarded`, `slow`, and `world-torso` counterparts; rich motor telemetry in `toe-ground-budget-experiment.json` index0. `probe-half-kneel-best.ts` currently selects the latest failed worldTorso variant, **not** the5.2167 s options.

Useful diagnosis: direct forward root shift at a deep leading knee demanded224 Nm versus110 Nm available. Holding initial root XZ, pivoting the knee onto its toe, rising, then transferring was much better. However swing/body reactions and weak trailing-foot replant/stand balance remain unresolved. Simply increasing height or delaying release is not a demonstrated solution.

## Experimental prone result — initial arms work; knee transition fails

Files: `scripts/experimental-recovery.ts`, `scripts/probe-experimental-recovery.ts`, `scripts/probe-prone-grid.ts`; supplement `evidence/prone-experiment-handoff.md`.

Both initial hands now become usable at1.8667 s in the isolated controller, about16/15 N load. Feet drift roughly7–12 mm, and rolling upward assistance is zero. This validates an initial arm placement, not a weight-bearing press or recovery.

The final12-case grid varied trailing hip-.45/-.75/-1.05, root pitch.8/1.2, arm load fraction.12/.24; spine-.3, root height.45, knee bend2.1, back shift.12. Each runs up to480 ticks or first retry/upright. Every case fails: trailing shin loaded time0; leading foot correctly never released; pelvis after brace only.129–.1675 m; hand drift.159–.292 m. Best scores were initial brace frames. No winning press variant exists.

The trailing heel rises with flexion, but shin capsule remains above the floor while the thigh is loaded. The next mechanical problem is actual contact/knee placement and load allocation across arms, torso/pelvis/thigh, not merely a phase condition. Whole-pose freezing of forearm/hand can also overconstrain the press. Shoulder lever .2 m at200 N would demand40 Nm beyond the unchanged30 Nm ceiling, so do not ask the arms to support all body weight before the knee/leg geometry can contribute.

The clone includes guarded trailing-foot/shin release and attempted guarded leading-foot/shin release once trailing knee really supports. It has an experimental6 mm downward hand target bias through joint torques (no direct hand force). Do not blindly copy its failed press settings.

Evidence: `evidence/prone-grid.json`, failed tied trace `prone-grid-best.json`, `experimental-landed-prone-left.json`. Root's production `narrow-brace-results.json`/trace are earlier failed diagnostic runs.

## Experimental rolling result — measured long axis is necessary

Files: `scripts/experimental-roll-recovery.ts`, `scripts/probe-roll-recovery.ts`. This copy was not integrated into production.

Original-heading roll targets redirect the body long axis across the floor. A supine body's head-to-pelvis projected direction is opposite initial forward; a side-landed body may point along initial right/left. The improved experiment derives a prone orientation from the measured torso-long floor axis, uses retained rollSide, and recomputes reachable hand targets from measured torso-right/heading after turning. Existing stable-standing handoff already captures actual pelvis-forward heading; there is no need to force original yaw before recovery.

Earlier measured-axis target reached prone at1.9 s versus12.4 s but slipped a supporting foot. Holding all measured joints and sequentially releasing safely reduced drift to2.5 mm but could not roll the full support width under60 Nm. Final compact cross-body arm/lateral-knee variant:

- Supine-left heading0: turns prone6.53 s, two usable hands/brace12.75 s, max planted drift7.87 cm, no measured violations at that intermediate goal.
- Supine-left heading pi3: turns6.42 s, brace7.90 s,14.26 cm right-hand drift during placement (FAIL).
- Both: zero upward assistance during roll and zero raised rolling pelvis target.

The roll stall metric still uses height/up error, which cannot adequately distinguish supine from prone; measured angular, joint, and support progress needs to replace it. Use a separate captured recoveryHeading in a production refactor; the experimental clone overwrites internal heading.

These are **only roll→prone→brace**, not completed recovery; neither demonstrates full acceptance. Earlier side-left reached brace8.47 s with24 cm drift/3 retries. Mirrors and other side cases are not robust.

Untested issue in the final clone: intermediate prone target local-yaw1.25 yields forwardY near-.315, so a turned threshold<-.5 requires overshoot. Derive consistent intermediate target/threshold rather than relying on overshoot. Already usable arm support should be allowed without demanding a newly computed arm target. Post-turn preparation should depend on actual pose/support, not route label (a retried half-kneel can also physically roll).

Final clone SHA256 `62A4139C2CD9DC9DC44145C4F1C09E1DE3FBFC0F6E661D29677839CA294F0D82`; probe `BED02B1589C6A952D1728B7E035ACA14160D36BDDA595CB473B5D3E7223307A1` at agent handoff. Root's later measurement sampling correction is outside these files.

## Replay implementation and actual visual review

`run-visual-replay.mjs` has been updated to public recovery diagnostics. New `run-recovery-visual-replay.mjs` simulates one immutable60 Hz Rapier trace, then replays exactly the same snapshots in four panels: WebGL side/three-quarter and Canvas2D side/three-quarter. It writes normal and .25x MP4, native WebM, trace JSON.gz, phase-entry images, source hashes, and measured summaries. A visible8px marker aligns the Playwright video clock. Source hashes include all recovery helper modules, including the foot solver added at closeout.

Actual completed **failure diagnostic** recording:

`evidence/recovery-20260909/diagnostic-half-kneel-2/`

Contains normal.mp4, slow.mp4, native.webm, trace.json.gz, phase images, source manifests, and first-transfer.jpg. Root inspected the phase image and a5fps filmstrip of the first2.4 s of the normal video. It shows an attempted rise followed by forward collapse; no credible completed recovery. Source changes during recording invalidated its fingerprint check, so it is not acceptance evidence. Full normal/slow visual acceptance is still outstanding.

The isolated experimental recorder `run-experimental-recovery-visual-replay.mjs` installs the fragile1/4 clone, but its latest recording failed: local Vite5174 crashed with Windows EBUSY while watching an evidence .webm. No successful authoritative all-route recordings exist. Obsolete tooling smoke/marker videos only test the recorder.

Before another recording, exclude `**/evidence/**` from Vite's watcher. No server was restarted after the user requested handoff; final process inspection found no remaining5174 preview process. The production replay script requires a live local Vite server and the bundled Playwright runtime. Do not publish or deploy.

## Tool/runtime details and commands

Use PowerShell in `O:\LaboratoireHumain`. `$env:CODEX_MCP_NODE_PATH` points to bundled Node24.19.0:

`C:\Users\flori\AppData\Local\OpenAI\Codex\runtimes\cua_node\b474a88d5d105afa\bin\node.exe`

Default system Node20 is too old for Vite. The project `scripts/run-tool.mjs` has no `tsc` alias. Commands:

```powershell
Set-Location O:\LaboratoireHumain
& $env:CODEX_MCP_NODE_PATH node_modules/typescript/bin/tsc --noEmit --pretty false
& $env:CODEX_MCP_NODE_PATH scripts/run-tool.mjs eslint . --ignore-pattern dist --ignore-pattern .next
$env:PATH=(Split-Path $env:CODEX_MCP_NODE_PATH)+';'+$env:PATH
npm.cmd test
& $env:CODEX_MCP_NODE_PATH --test tests/*.test.mjs

# Full harness. No scenario filter for acceptance.
Remove-Item Env:PHYSICS_SCENARIO_PATTERN -ErrorAction SilentlyContinue
$env:PHYSICS_OUTPUT_PREFIX='evidence/recovery-next/full'
# Create this exact output parent directory first if it does not exist.
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/run-physics-harness.ts

# Focused production fixture; omit the mistaken -heading-0 suffix.
$env:PHYSICS_SCENARIO_PATTERN='^landed-half-kneel-left$'
$env:PHYSICS_OUTPUT_PREFIX='evidence/recovery-next/half-left'
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/run-physics-harness.ts

# Retained experiments; they do not run the production controller.
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/probe-half-kneel-best.ts
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/probe-load-recovery.ts
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/probe-prone-grid.ts
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/probe-experimental-recovery.ts landed-prone-left 600
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/probe-roll-recovery.ts supine-left 900

# After starting a local Vite server with evidence excluded from its watcher:
$env:RECOVERY_REPLAY_URL='http://127.0.0.1:5174'
& $env:CODEX_MCP_NODE_PATH --import tsx scripts/run-recovery-visual-replay.mjs evidence/recovery-next/representative landed-half-kneel-left
```

Environment issue: default sandbox exec/apply_patch/view_image and node_repl failed before execution with a sandbox helper setup error. Narrow authorized workspace `exec_command` calls using `sandbox_permissions:'require_escalated'` worked and were approved. No automatic approval rejection occurred. Files were edited with native PowerShell/IO methods. Normalize CRLF to LF before exact multiline replacements; otherwise mixed endings silently miss substitutions. Avoid broad destructive file operations. No AGENTS.md or hosting configuration was found for this task.

To view a local QA image when view_image fails, read JPEG bytes with an approved filesystem command and pass the base64 data URL to functions.image; for PNG, System.Drawing can encode it to an in-memory JPEG. ffmpeg is available for extracting diagnostic video frames. Do not confuse these QA frame extractions with an image-generation task.

Root's temporary `scripts/probe-recovery.ts` and `probe-experimental-recovery.ts` create the character without heading options, so **do not use them to validate rotated headings**. Official harness and `probe-half-kneel-best`/`probe-roll-recovery` create it with fixture.heading. `inspect-probe.mjs` expects an evidence JSON filepath, not a fixture ID. Probes often overwrite fixed evidence filenames; copy important evidence or change the output prefix before reruns.

## Next work, in order

1. Keep all existing requirements/caps. Read actual production vs clone differences before integrating anything; several experiments depended on hidden stand-phase gains and fixed ground-force budget scope.
2. Resolve half-kneel trailing-foot replant and stand balance with actual loaded sole geometry, coordinated pelvis/spine/leg movement, and fixed plants. Preserve the successful crouch path. A phase transition alone is not success.
3. Resolve prone knee contact and load transfer before press, then implement the missing guarded leading-foot release/placement in production. Review fixed full-pose intermediate supports and body-contact load allocation.
4. Correct rolling to use the measured long axis and measured post-roll heading, with internally generated asymmetric mass movement and safe releases. Preserve deterministic chosen side until retry; avoid dragging planted limbs.
5. Rerun all24 landed cases and natural-fall routes including rotated/mirrored cases. Attribute slip by segment/contact/time; inspect knee/elbow reversals. Keep25 s acceptance and all existing entry/handoff conditions.
6. Run final typecheck, lint, app build/tests, and full physics harness on frozen source. Record representative recoveries across both renderers/views at normal andslow speed on the same frozen source. Review hoisting, sliding, symmetry, abrupt transitions, and real weight transfer.
7. Only after physical/visual success, simplify/format the experimental-looking controller code, remove unneeded diagnostic clones/probes from deliverable source, and update `docs/physics-acceptance.md`/verification status. Preserve evidence and useful regressions. No deployment work is authorized in this iteration.

## Closeout

The final check summary, evidence inventory, and source hashes are in `evidence/recovery-20260909/handoff-validation.json`, `handoff-evidence-inventory.json`, and `handoff-source-manifest.json`. This main handoff is intentionally tracked-ready; the detailed evidence supplements are gitignored and remain available in the current workspace.
### Final observed check results

The full intermediate harness completed all59 scenarios: **17 passed,42 failed**. All4 landed crouches passed. Remaining failures include actual recovery timeouts and excessive planted-limb drift. Some additional COM assertions in that report were the activation-sampling issue described above. The subsequently corrected `fast-forward` rerun still fails recovery/drift, but the false COM/projected-COM failures are gone. No full rerun after that validation-only correction was made during closeout.

- Full report: `evidence/recovery-20260909/full-intermediate-results.json`
- Full sampled trace: `evidence/recovery-20260909/full-intermediate-trace.ndjson`
- Corrected sampling follow-up: `evidence/recovery-20260909/final-fast-forward-results.json` and corresponding trace.
- Final application tests:83 total,82 pass,1 fail.
- Final TypeScript check:pass, exit0.
- Whole-repository ESLint:pass before the final sampling-only correction; final closeout run/log is recorded in the validation JSON.
- Build succeeded; complete recovery/visual acceptance did not.

The full harness ran with the source loaded at its start. It is explicitly an intermediate report, not a frozen-source acceptance bundle. Use the closeout source manifest for the exact files left in the workspace.