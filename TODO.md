# TODO

Reviewed 2026-09-26 against the local working tree at `44fc25437d841151c704895c1fb92de4c2e468d0`, including existing uncommitted changes. This is a work plan, not a fresh physics result. See [current status](docs/status.md) for recorded failures and evidence qualifications.

The protocol room, seven-station playground, both renderers, continuous 25-body physics, and diagnostic/test infrastructure already exist. The missing milestone is repeatable, accepted physical behavior with reproducible evidence. Work through the dependencies below; independent evidence/tooling work can proceed while physics remains open.

**2026-09-27 update: the first unfinished task is still standing still without
drifting, wobbling, or falling.** The experimental search sometimes threw away
a helpful move because it missed a preferred extra margin, then kept an old
move that caused much more wobbling. [H71](docs/standing-h71-contract.md#plain-language-explanation)
confirms this selection error. Keeping a tested correction within the actual
speed limits is an experimental repair, not proof that standing is solved.

Next, preserve the partial results from [H73's memory failures](docs/standing-h73-contract.md#first-attempt--memory-failures-no-complete-screen)
and repeat with **one simulation at a time** under its documented runtime bound.
Recheck disabled compatibility, then complete all three short screens; proceed
to the unchanged longer standing tests only if every short screen passes.
Keep the standing checkbox open until the full criteria below pass. Weight
shifting, stepping, recovery, and full application checks remain afterward.

## P0 — Restore reproducibility and establish quiet standing

- [x] **Make a fresh baseline possible without lost historical reports.** Added explicit `--fresh` capture in [capture-physics-baseline.mjs](scripts/capture-physics-baseline.mjs). The [new baseline](docs/checkpoints/2026-09-26/fresh-baseline.json) captures 252 source/configuration hashes, installed direct dependency versions, the Node executable digest, staged/unstaged/untracked work, planned scenario inputs, and four missing historical artifacts. It inherits no 140/142 result and runs no physics. Five CLI regression tests cover missing/empty/changed history, dirty worktrees, and immutable output; see [verification](docs/checkpoints/2026-09-26/fresh-baseline-validation.json) and [reproduction instructions](docs/evidence.md#fresh-baseline-capture). Original artifacts remain unchanged and missing originals remain unavailable.

- [x] **Restore or regenerate the allocator experiment input.** New capture `standing-regenerated-20260926-02` reproduces the +π/3 tick-176 spike (1.07393464 rad/s); plain, observed, and repeated histories match exactly over 240 ticks. Both recorded allocator outputs replay exactly on the matching production source. The nonempty trace, reports, production source snapshot, and digests are retained in the [verified archive](docs/checkpoints/2026-09-26/h8-evaluation/manifest.json). This is new evidence; the missing original remains unavailable. See [reproduction](docs/standing-h8-evaluation.md).

- [x] **Finish evaluating the H8 hypothesis before any production adoption.** Added controlled probe modes and [staged evaluation](scripts/evaluate-standing-h8.mjs). The static jump falls from 247.41781 N to 0.11198 N with force/moment/positivity checks passing. Local forefoot response falls 95.09%, with the next five ticks ≤0.25162 rad/s. All six 2+10 s startup screens fail stationary bounds, including the upper-limb damping combination. **H8 rejected; no production adoption.** Explicit initialization, pressure infeasibility, patch infeasibility, and support-membership counts and preserved failed runs are in the [report](docs/standing-h8-evaluation.md).

- [ ] **Pass official quiet standing at every heading.** Explain and repair the measured load-allocation discontinuity and remaining motion without assuming either alone explains all failures. Done when the unchanged 2 s settling + 30 s observation passes at 0, +π/3, and −π/4: pelvis drift ≤0.03 m, hindfoot/forefoot drift ≤0.01 m, all-segment peak speed ≤0.1 m/s, angular speed ≤0.5 rad/s, zero added steps, both feet planted, and the accepted upright state. Also report maximum excursions and vertical ranges. Preserve structural, collision, actuation, and ownership gates. H12–H35 did not establish a repair: short-screen improvements still leave drift or speed failures. H36 records a copied-world response model, H37 rejects a runtime downgrade, and [H38](docs/standing-h38-contract.md) confirms a geometric-inertia defect. Its [current production correction](docs/inertia-correction-2026-09-27.md) still fails all three 2+30 s captures and remains unaccepted. [H40](docs/standing-h40-contract.md) repairs the resulting chest-contact regression; [H41](docs/standing-h41-contract.md) repairs relaxed elbow clearance. [H44](docs/standing-h44-contract.md) strengthens finite-state checks for mass and center of mass. [H51](docs/standing-h51-contract.md) now passes the unchanged 69-test selection after correcting swing-ankle target utilization within the original anatomical range. Fresh H51 standing captures exactly reproduce all three failed H44/H41/H40 histories. [H42–H43](docs/standing-h43-contract.md) do not establish an admissible constraint repair. Independent [native constraint calibration](docs/rapier-constraint-calibration.md) has not established a repair. All three headings currently have recorded failures; see the [standing contract](docs/physics-standing-contract-2026-09-26.md#frozen-measurement-contracts).

## P1 — Transfer, prediction, and repeatable steps

These milestones depend on accepted quiet standing. Existing controller and prediction tests remain useful but do not establish the complete physical sequence below.

- [ ] **Demonstrate repeated measured weight transfer.** Add or complete executable coverage of three left–right–neutral cycles per heading in one uninterrupted run. Each target qualifies within 3 s; planted-foot slip stays ≤0.01 m per transfer and cumulatively per cycle. Neutral settles within 2 s, then holds stable double support for ≥1 s. Authorize swing only from measured retained load ≥52% of body weight and moving load ≤25% for 0.10 s. Done when every heading meets the [frozen transfer contract](docs/physics-standing-contract-2026-09-26.md#frozen-measurement-contracts) without resets or relaxed standing limits.

- [ ] **Validate prediction against the production simulation.** Compare matched 0.5 s trajectories at held-out signed command magnitudes of 75% and 125% of calibration, within existing caps, plus infeasible requests. Done when each foot-load-change error is ≤10% of body weight, horizontal COM-displacement error ≤0.02 m, and readiness-time error ≤0.05 s; record maxima and RMS errors. Infeasible requests must be labeled and cannot predict readiness. The recorded short hip-pulse check does not establish this transfer-prediction gate.

- [ ] **Complete individual steps, then sequences, release, and reversal.** Require qualified transfer, measured swing unloading <3 N for 0.05 s, touchdown within 0.09 m for 0.10 s, completed cooldown, and ≥1 s stable double support. Prove each direction at each heading before subsequent steps and release/reversal cases. Resolve the recorded slow-pull fall at tick 216 and planted-reversal fall at tick 343 in [balance-controller.test.mjs](tests/balance-controller.test.mjs), plus both balance probes. Done when the complete focused selection and new physical regressions pass on the same source; recover or explicitly re-enumerate the historical 142-test selection before comparing counts.

- [x] **Resolve the separate small-impulse diagnostic outlier.** [Controlled native precision comparison](docs/small-impulse-diagnostic.md#result--numerical-precision-sensitivity-isolated) identifies a precision-sensitive numerical residual in the tiny paired-impulse measurement. With matching solver features, identical engine source and exact serialized physical-state transfer, all twelve f64 cases are smooth and within 1.71% error; native f32 remains irregular (up to 15.35%). Correctly hooked WASM snapshots replay all 24 post-step body states exactly. Contacts and joint-limit regimes are unchanged. The application still reproduces its 15.23% diagnostic outlier; the original assertion and tolerance remain unchanged, with [nine original tests passing](docs/checkpoints/2026-09-27/impulse-precision-validation/manifest.json). [Evidence and failed attempts are archived](docs/checkpoints/2026-09-27/impulse-precision/manifest.json). The fixed-assembly fixture still collapses in f64, so this is not a standing repair.

## P2 — Complete integration and release evidence

- [ ] **Run the complete application and physics gates on one candidate.** After the earlier milestones pass, record `npm run typecheck`, `npm run lint`, `npm test`, `npm run test:recovery`, and the unfiltered `npm run test:physics`, including exits, scenario counts, settings, dependency versions, and before/after fingerprints. The harness currently discovers 63 physics scenarios; discovery is not execution. Done when all required assertions pass and warnings are documented. The older 273/280 application and 6/63 physics counts are historical failures, not current verification.

- [ ] **Establish end-to-end recovery and five-cycle protocol acceptance.** Run the required recovery fixtures and [protocol-five-cycle-acceptance.mjs](scripts/protocol-five-cycle-acceptance.mjs). Done when five uninterrupted cycles complete with the existing 25 s post-impact recovery limit, measured stable returns, continuous body ownership, and no automatic reset, while all structural/contact criteria hold. Capture the required visuals; a three-second protocol replay cannot close this task. See [physics acceptance](docs/physics-acceptance.md) and [browser setup](docs/browser-verification.md).

- [ ] **Refresh browser and terrain verification in both renderers.** Follow the full [browser verification selection](docs/browser-verification.md#run-the-required-selection), inspect PNGs/videos, and record the actual renderer used. Cover protocol controls, dragging/input lockout, pause/reset, renderer changes, playground stations/difficulty, and mobile viewport. Record terrain failures explicitly. Done when required behavior and visual checks pass on the candidate; retain the distinction between UI smoke coverage and physical terrain/recovery acceptance. Native touch and hardware GPU performance remain unverified unless tested on actual devices/hardware.

- [ ] **Verify both Pages configurations and connect the complete release gates.** Run `npm run test:pages` at the domain root and with `PAGES_BASE_PATH=/LaboratoireHumain`, following the [platform-specific commands](README.md#deploy-to-github-pages). The local deployment workflow blocks on its configured checks, but it does not run the full browser selection or explicit five-cycle script; the separate browser workflow runs only `quick`. Done when the release candidate has both build results and all required integration evidence, and deployment is gated by those results. Inspect remote CI/deployment separately before reporting remote status.

- [ ] **Publish durable evidence and update the status page.** Keep compact summaries/manifests with the docs and store large traces/videos at durable artifact locations with SHA-256 digests. Local CI retention is currently 7 days for Pages evidence and 14 days for physical-chain evidence; neither alone is a permanent archive. Done when a fresh checkout can locate and verify every claimed result, including failures, and [current status](docs/status.md) links to that candidate's evidence. Follow [evidence preservation](docs/evidence.md#preserving-future-results).

## Completion rules

Keep timestep, solver settings, actuator ceilings, collision rules, fall guards, continuous Rapier ownership, and acceptance thresholds fixed during this repair sequence. Do not refresh fixtures to turn failures green. Preserve rejected/partial runs and distinguish diagnosis from an accepted repair. Only mark a task complete with source-scoped evidence; changed dependent source invalidates earlier verification.

The initial review changed documentation only. The first P0 repair verified fresh baseline capture. The next P0 work regenerated the allocator input and rejected H8 after controlled static/local/startup evaluation; H9–H11 also failed their declared screens. Thirteen tooling/regression tests, targeted ESLint, and full typecheck pass on the [recorded source](docs/checkpoints/2026-09-26/standing-tooling-validation.json). No production physics change was adopted, and complete physics, browser, build, and deployment acceptance remain open. Individual fingers, individual vertebrae, and deformable tissue remain intentional model exclusions rather than missing release work.

The H25–H26 continuation preserves six exact screen-control replays and all 65 production source/package hashes. Its [55-artifact archive](docs/checkpoints/2026-09-27/h25-h26-calibration/manifest.json) includes independent native-engine calibration and rejected interventions. [Twenty-five targeted tests and targeted lint pass](docs/checkpoints/2026-09-27/h25-h26-validation-complete/manifest.json); physical and integration milestones remain open.

The [H27–H29 archive](docs/checkpoints/2026-09-27/h27-h29-evaluation/manifest.json) preserves 27 verified artifacts: rejected hybrid constraints, an isolated upstream runtime comparison, and the rejected centered balance target. Three H22 controls and the pinned runtime loader replay exactly; all 65 production/package hashes remain unchanged. Rapier 0.21.0 fails two of 21 original focused tests and all three standing speed screens; 0.20.0 passes all 21 focused tests. [Nine archive tests and targeted lint pass](docs/checkpoints/2026-09-27/h27-h29-validation/manifest.json). None of these diagnostic results closes quiet standing.

[H30](docs/standing-h30-contract.md) and [H31](docs/standing-h31-contract.md) reject a smooth startup transition and a centered initial posture, respectively. H31 preserves the original sole positions and constructs the requested COM placement before dynamic body creation, but still fails drift and speed bounds at every heading. Six additional H22 controls replay exactly. The standing task remains open.

The H32–H37 continuation preserves negative CCD/block-solver calibrations, rejected integral/interval feedback, a copied-world response model and a non-drop-in older runtime comparison. [Evidence is archived](docs/evidence.md#h32-onward). The H38 geometric audit identifies a separate physical mass-property error and retains a correction candidate with explicit failed gates. Production is no longer identical to H8; all prior pass claims retain their original source scope. No P0, P1 or P2 acceptance box is closed by these diagnostics.

H40 adds bounded active hand-reach refinement and a stronger pre-recovery contact assertion. Both chest-drag cases now pass the existing contact/penetration/separation gates; their later falls do not establish balance acceptance. The two wider anatomy failures reproduce under the archived compositor: inward relaxed elbow placement during lean, and a swing-foot heading assertion whose full-vector metric also includes pitch. Their inputs and assertions remain unchanged.

H41 constrains relaxed elbow placement on the existing two-bone circle without moving its wrist endpoint or stretching bones. The original lean/twist/heading anatomy case and four added geometric tests pass. The remaining swing-foot fixture and assertion are unchanged; no physical acceptance checkbox is closed. See the [H41 result](docs/standing-h41-contract.md).

H42 confirms that corrected inertias do not remove the independent fixed impulse-tree collapse. H43 rejects a single-axis multibody forest after loss of upright state at tick 109. H44 corrects the finite guard that overlooked its invalid centers of mass: the unchanged trajectory now fails at tick 133, with 68/69 targeted tests passing. These results and source-preservation checks are [archived](docs/checkpoints/2026-09-27/h44-finite-mass/manifest.json); full physical and release gates remain open.

H45 rejects the forefoot frame correction. H46 verifies exact copied-world
two-motor predictions but fails one local speed bound. H47 verifies six-motor
authority at all three retained peaks, including exact 25-body live pulses
and bounded following five-step windows. An earlier +π/3 peak still fails;
no checkbox closes. See [H47 evidence](docs/standing-h47-contract.md) and the
[H48 online preview result](docs/standing-h48-contract.md). H48 keeps all
observed speeds bounded with exact 25-body matches on 2,160 live steps, but
fails +π/3 foot drift (11.208 mm), so standing remains open. H49 tests the
remaining horizontal motion through the same capped motors.

H49 passes all three 2+10 s diagnostic screens, with foot drift
8.785/9.187/7.540 mm and exact predictions on 2,160 live steps. Its unchanged
2+30 s evaluation fails foot drift at every heading (26.035/29.916/23.336 mm),
despite exact 25-body predictions and bounded speeds on all 5,760 steps.
The 225–258 ms average short-screen preview cost is not an interactive
implementation; no production acceptance checkbox closes. See the
[H49 evaluation](docs/standing-h49-contract.md#long-evaluation--rejected-for-accumulated-foot-drift)
and [H50's local search comparison](docs/standing-h50-contract.md).

[H51](docs/standing-h51-contract.md) repairs the swing-foot anatomy failure
within unchanged anatomical limits. All 69 tests in the unchanged selection,
typecheck and modified-file lint pass. The other 66 production/package files
remain identical to H44. Three fresh 2+30 s plain histories replay H44 exactly;
standing, transfer, recovery and release acceptance remain open.

H52–H54 verify native motor parameters and local full-leg authority. The
[H55 continuous evaluation](docs/standing-h55-contract.md) rejects that extension:
all three short screens fail whole-body speeds, despite bounded foot drift and
2,160 exact live predictions. The H51 production candidate remains unchanged;
the standing prerequisite and all dependent acceptance work stay open.

[H56](docs/standing-h56-contract.md)'s bounded pelvis task passes all three
local comparisons. [H57](docs/standing-h57-contract.md)'s completed short screens
fail speeds at all three headings. [H58](docs/standing-h58-contract.md) finds
bounded local corrections for two earlier failures by fixing angular search
step size; later +π/3 peak states remain uncorrected. H59's continuous
follow-up passes two short screens but retains eight hand-speed failures at
−π/4. [H60](docs/standing-h60-contract.md) verifies local correction of two hand
failure states through the twenty existing arm motor axes. H61's completed
arm-stage follow-up bounds all observed speeds, but fails +π/3 foot drift
(10.095 mm against 10 mm); its 2,160 live predictions match exactly.
[H62](docs/standing-h62-contract.md) verifies local return toward the initial foot
positions with incomplete one-step task convergence. [H63](docs/standing-h63-contract.md)
rejects that position feedback after speed failures at every heading and a fall
at +π/3. All 1,906 chosen live predictions match; 254 later steps are skipped
by the original state guard. [H64](docs/standing-h64-contract.md)'s coupled local
search remains insufficient. [H65](docs/standing-h65-contract.md) reaches all
three local speed reserves by expanding improving motor-probe directions.
[H66](docs/standing-h66-contract.md) passes all three 2+10-second diagnostic
screens, with 2,160 exact live predictions and unchanged speed/drift limits.
Its original 2+30-second evaluation records a heading-0 speed failure at tick
978; a complete long result remains unreported in that checkpoint.
Production adoption and interactive performance remain unresolved; no
acceptance box closes.

[H67](docs/standing-h67-contract.md) and [H68](docs/standing-h68-contract.md)
investigate the remaining runtime cost on saved H66 states. Previous commands
and cached response columns give bounded seeds and some improvements, but miss
the complete local task and do not establish continuous performance. Their
complete result archives verify; the original H66 histories remain unchanged.

[H69](docs/standing-h69-contract.md)'s continuous warm-cache candidate fails two
short screens, including a fall at +π/3; only heading 0 passes. All 2,147 selected
live states match their predictions exactly, with thirteen guarded skips after
the fall. Complete nested data and injected source are archived and verified.
No physical acceptance box closes and no long H69 run follows.

[H70](docs/standing-h70-contract.md) passes the −π/4 short screen; its other
headings remain pending. [H71](docs/standing-h71-contract.md) confirms that the
search discarded corrections within the actual speed limits in two saved
states because they missed its stricter foot margin. [H72](docs/standing-h72-contract.md)
extends the search and retains valid fallbacks, finding admissible corrections
at all three examined states. These are local results only.
[H73](docs/standing-h73-contract.md) reproduces the local queries and disabled
baseline, but all three concurrent enabled attempts exhaust memory without
final reports. Preserve their partial outputs and repeat serially as specified
above. No completed serial screen or standing acceptance is claimed.
