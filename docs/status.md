# Current status

**Recovery performance investigation, 2026-09-27:** repeated foot-placement searches during rolling are skipped until a placement phase needs them. A 10-second strike replay fell from 146 seconds to 20–27 seconds on this host; it still does not run in real time or complete get-up. Lower solver counts and rolling motor trials were rejected. Per-tick diagnostics expose a transient joint-limit violation missed by coarse sampling. See the [measurement, retained changes and limitations](recovery-responsiveness-profile.md).

**Recovery launch correction, 2026-09-27:** the application now integrates recovery motors with native contact/joint constraints instead of applying frame-sized torque impulses first. A real strike reproduced the old floor-to-ceiling launch; the corrected 10-second replay remains near the floor (maximum pelvis height after landing 0.1222 m). The character still stalls while rolling toward a brace, so get-up acceptance remains open. See [cause, comparison and validation](recovery-native-actuation-fix.md). The standing investigations below retain their recorded source scope; this correction does not establish quiet standing.

Updated 2026-09-27 through H75-v1. **Quiet standing remains the first unfinished TODO.** H74's three references remain rejected. [H75](standing-h75-contract.md) executes all 342 frozen copied motor probes with exact controls/repeats, but full native row inspection remains incomplete after three failed builds. The central best probe lowers 0.501134 to 0.464467 rad/s, still above the reference reserve; no controller repair is justified. H74's 81 regressions, five candidate tests and typecheck retain their unchanged source scope. No physical acceptance checkbox closes.

One confirmed mistake was rejecting a helpful correction because it missed a preferred extra margin, then keeping an old command that caused much more wobbling. The [plain-language explanation and replay evidence](standing-h71-contract.md#plain-language-explanation) describe why. The experimental repair retains a tested correction within the actual speed limits when the preferred margin is unreachable; it does not establish lasting balance.

Some short tests improved, but H66's longer test has a recorded speed failure. H73's first three concurrent simulations exhausted memory without final reports. Any H73 retry must preserve their partial results and run **one simulation at a time**, following the [H73 restart procedure](standing-h73-contract.md#first-attempt--memory-failures-no-complete-screen). These remain experimental fixes. Weight transfer, stepping, recovery, interactive performance, and full application verification remain open. Dated reports retain their own evidence and limitations.

## Coordinated investigation — bounded negative result

The [coordinated stabilization plan](standing-stabilization-plan.md) now has an
explicit shared experimental controller and serial acceptance operation:
`node scripts/standing-acceptance.mjs`. [H74-v1](standing-h74-contract.md) freezes
numeric limits, preserves nine exactly replayed failure captures, exercises six
failure transitions and verifies unchanged default behavior against archived
source for thirty steps. The three reference trials fail at ticks 133, 120 and
123. Later physical stages are incomplete; the command exits 1. No validated
operating/backup region, MPC result or interactive-performance claim follows.
The H42 rigid comparison reproduces its representation difference. H75's new
frozen diagnostic verifies all 176 listed H74 artifacts and 315 retained source
hashes, then completes six exact controls and 168 independently repeated signed
motor commands. Zero native position stiffness excludes direct excitation by
the motor's arcsine position-coordinate formula in these states. Substantial
relative forefoot motion in locked directions remains unexplained; unchanged
contact counts do not isolate contact or constraint effects. Compiled raw/finalized
row reconstruction is incomplete after the three-build budget is exhausted.
Next freeze the [six-snapshot build-and-read diagnostic](standing-h75-contract.md#concrete-next-diagnostic),
using zero new physical steps. All failures and sources are retained.

H73 remains a **pending comparison candidate**, not an accepted baseline or a
prerequisite for this investigation. Its documented serial restart remains the
procedure for evaluating H73. H66's 1.72–1.88 s timing is average preview time
under concurrent diagnostic runs, not an isolated application measurement.
H74 changes neither physical thresholds nor historical results. It remains
opt-in; standing, transfer, stepping, recovery and release gates stay open.

## Source and evidence scope

The reviewed working tree starts at `44fc25437d841151c704895c1fb92de4c2e468d0` and includes uncommitted physics, test, and workflow changes. The [documentation checkpoint manifest](checkpoint-2026-09-26.json) records a partial source fingerprint and missing artifact inventory. It is not a physics test report or a complete dependency fingerprint.

The default physical controller retains [H51](standing-h51-contract.md); H74 adds an opt-in candidate and shared application/harness frame function. Its tests and thirty-step disabled comparison do not refresh H51's longer physical histories. [H44](standing-h44-contract.md), [H41](standing-h41-contract.md), [H40](standing-h40-contract.md) and the earlier [geometric-inertia report](inertia-correction-2026-09-27.md) retain their source-scoped results. Earlier results below retain their dated source scope. The [September 26 investigation](physics-standing-contract-2026-09-26.md) still has missing original reports; recovered artifacts and limitations are listed in [evidence availability](evidence.md). Missing reports cannot be treated as independently verified results merely because their counts appear below.

## Recorded gates

[H52](standing-h52-contract.md) verifies native motor inspection and added knee
authority. [H53](standing-h53-contract.md) exposes scaling and search limitations;
[H54](standing-h54-contract.md) reaches the local foot-motion deadband using all
fourteen existing leg axes. Continuous [H55](standing-h55-contract.md) then fails
all three short screens: foot drift stays below 1 cm, but hand and forefoot
speeds exceed the frozen limits. All 2,160 live predictions are exact. These
diagnostic changes do not modify the H51 production candidate or close any gate.

[H56](standing-h56-contract.md) adds measured pelvis pose rates to the local
objective. Its bounded variant meets the declared foot/pelvis task targets at
all three saved states; the zero-target variant does not. Five mathematical
tests and targeted lint pass. [H57](standing-h57-contract.md)'s completed short
screens fail speeds at all headings despite foot drift below 1 cm and 2,160 exact
predictions. [H58](standing-h58-contract.md) corrects two early isolated states
with full calculated updates under the original motor ceilings; later +π/3
peak states remain uncorrected. This is limited local search evidence;
[H59](standing-h59-contract.md)'s completed short screens pass at 0 and +π/3,
but retain eight hand-speed failures at −π/4. All 2,160 predictions are exact.
[H60](standing-h60-contract.md) uses the twenty existing arm motor axes to
correct two distinct retained hand-speed failures in one local iteration each.
[H61](standing-h61-contract.md)'s completed continuous arm-stage evaluation
bounds all observed speeds but fails +π/3 foot drift at 10.095 mm. All 2,160
live predictions match exactly; the full archive reconstructs all original files.
[H62](standing-h62-contract.md) reduces foot position error in three saved
states using the existing leg motors, but only one meets the complete local
tracking/pelvis deadband. [H63](standing-h63-contract.md)'s continuous position
feedback fails all three speed screens and falls at +π/3; its full verified
archive preserves 417,945 queries and 1,906 exact live predictions, with 254
guarded skips after the fall. [H64](standing-h64-contract.md)'s coupled local
speed search reduces the violations but does not meet all search guards.
[H65](standing-h65-contract.md) expands improving probe directions and reaches
the stricter speed reserve at all three first-failure states. Its 947 copied
queries and earlier comparator failures are archived. [H66](standing-h66-contract.md)
reproduces all 947 local queries through its extracted helper and passes all
three short standing screens. Its 489,616 queries produce 2,160 exact live matches
with bounded speeds, drift, support and state. The unchanged 2+30-second
diagnostic was launched and later recorded a speed failure at tick 978. Preview cost remains 1.72–1.88 seconds per step;
interactive performance and production adoption remain unresolved.
[H67](standing-h67-contract.md) finds physically bounded previous-command seeds
at three saved states. [H68](standing-h68-contract.md)'s cached response matrices
improve those seeds but beat the original selected correction at only one
heading; neither local comparison establishes continuous performance.
[H69](standing-h69-contract.md)'s continuous cache candidate passes heading 0
but fails speeds at −π/4 and falls at +π/3. Its 2,147 exact live predictions and
thirteen guarded skips are preserved in a complete verified recursive archive.
Reduced query counts do not establish an acceptable replacement.
[H70](standing-h70-contract.md) passes its completed −π/4 short screen; the
other two headings remain pending while H73 tests the separate fallback repair.
The H66 long history has a known heading-0 speed failure at tick 978;
[H71](standing-h71-contract.md) confirms that physically admissible commands were
discarded because they missed an internal search margin. Complete long results
remain unreported in that checkpoint; this known failure already prevents a pass.
[H72](standing-h72-contract.md) finds corrections within the actual speed limits
at all three saved failure states by extending the copied search and retaining
valid fallbacks. This is local evidence, not continuous standing success.
[H73](standing-h73-contract.md) verifies local replay and disabled compatibility,
but its first concurrent enabled runs end in memory errors without final reports.
Repeat one simulation at a time under the documented runtime bound, preserving
all partial results. No completed serial screen is established by this update.
All 67 production/package files remain unchanged. No standing or production
gate closes before complete independent acceptance results.

[H45](standing-h45-contract.md) rejects a forefoot target-frame correction.
[H46](standing-h46-contract.md) verifies exact two-motor predictions but leaves
one local angular-speed failure. [H47](standing-h47-contract.md) uses the six
existing distal motors to reduce all three sampled peaks and keep the next
five steps bounded. Every live output matches its copied prediction for all
25 bodies. Earlier peaks remain in the full histories; no standing or P1
prediction gate follows from these local results. [H48](standing-h48-contract.md)
bounds every observed speed with exact predictions on all 2,160 steps but
fails the +π/3 foot drift bound (11.208 mm in the short screen). No 30 s
acceptance run follows. [H49](standing-h49-contract.md) adds horizontal motion
correction through the same native motors and passes all three short screens,
then fails all three 2+30 s foot-drift bounds (26.035/29.916/23.336 mm).
All 5,760 long-run steps match their exact previews. All 67 production/package
files matched H44 during those evaluations; the results do not replace
production acceptance.

[H50](standing-h50-contract.md) checks search convergence on isolated saved
states while H49's longer evaluation ran unchanged. One full calculated
update improves the local residual more than four 1 Nm-limited updates at
each heading, within the existing motor ceilings and speed guards. This is
an opportunity to reduce calculation cost, not a measured online speedup.

| Gate | Latest recorded result | Scope and qualification |
| --- | --- | --- |
| Current geometric-inertia candidate | Mathematical defect corrected; candidate unaccepted | [H38 audit](standing-h38-contract.md) identifies 13 incorrect automatic segment tensors. Explicit geometric mass properties are now supplied at construction. Solver, geometry, collisions and actuator ceilings are unchanged. |
| Current focused validation | 69/69 selected tests pass; typecheck and modified-file lint pass | [H51](standing-h51-contract.md): original swing-foot assertion now passes, and all twelve test files remain unchanged. The earlier unused `RIGHT` warning in unchanged `EmbodiedCharacter.ts` is outside this lint selection. This count is not the physics harness scenario count. |
| Swing foot target | Original anatomy failure repaired within unchanged anatomical limits | [H51 evidence](checkpoints/2026-09-27/h51-swing-range-production/manifest.json): target utilization follows the existing lift envelope, releasing the planted reserve while raised and restoring it at touchdown. |
| Finite physical state | Mass and center of mass now checked | [H44 evidence](checkpoints/2026-09-27/h44-finite-mass/manifest.json): the identical failed multibody trajectory is rejected at its first non-finite COM, one tick before the previous inertia exception. |
| Relaxed elbow clearance | Original lean/twist/heading cases pass | [H41 evidence](checkpoints/2026-09-27/h41-elbow-clearance/manifest.json): 25 mm minimum lateral clearance, four geometric regression tests, original joint bounds and physical contact tests preserved. |
| Active hand reach | Chest contact repaired within unchanged limits | [H40 evidence](checkpoints/2026-09-27/h40-bounded-reach/manifest.json): both original contact fixtures pass. Left-hand contact occurs in `reacting` before falling; the test now requires this. The later falls remain unaccepted balance/recovery behavior. |
| Current 2+30 s standing | All three headings fail | [Fresh H51 capture](checkpoints/2026-09-27/h51-swing-range-production/manifest.json) exactly replays H44/H41/H40 and the inertia candidate: 0.04451–0.05895 m foot drift, 0.61833–0.71482 m/s peak linear speed. Upright, double support and zero steps do not establish acceptance. |
| Corrected-inertia constraint calibration | Rigid impulse trees still topple; hinge-forest experiment rejected | [H42](standing-h42-contract.md) preserves the rigid representation difference. [H43](standing-h43-contract.md) retains anatomical motion but leaves upright at tick 109 and becomes non-finite at tick 133. No controller/runtime change is adopted. |
| Fresh baseline tooling | Capture succeeded; five CLI regression tests passed | [New baseline and source-scoped verification](checkpoints/2026-09-26/fresh-baseline-validation.json). Four historical artifacts are recorded missing; no inherited focused result and no physics executed. |
| Regenerated allocator replay | Exact replay of both ticks; three identical control histories | [New capture and verified archive](standing-h8-evaluation.md); the missing original is not recovered. |
| H8 hypothesis | Rejected after all six 2+10 s screens failed | Static and local checks pass; prolonged standing does not. No production adoption. [Results and event counts](standing-h8-evaluation.md). |
| H12–H21 continuation | No accepted repair | [Declared experiments and outcomes](standing-h12-contract.md). H13 and H20 combinations pass short-screen speed bounds but fail foot drift; holding references still fails. Production physics was unchanged at that checkpoint. |
| H22–H24 continuation | Rejected | [Idle forcing/stance-task screens](standing-h22-contract.md): H22 passes speeds but fails 0.01378–0.01777 m foot drift. H23/H24 also fall at −π/4. A trace boundary fix now separates recovery commands from standing contributions; no production repair adopted. |
| H25–H26 continuation | Rejected | [Displacement feedback and fixed local leg targets](standing-h25-contract.md) still fail drift or stability. Six control histories replay exactly. |
| Native constraint calibration | Constraint representation isolated in the rigid fixture | [Fixed multibody comparison](rapier-constraint-calibration.md#fixed-multibody-comparison) keeps 25 separate dynamic bodies and limits foot endpoint drift to 0.45 mm, while fixed impulse joints collapse. Both f32/f64 impulse variants fail. Anatomical motion is locked in these fixtures; no articulated standing repair is established. All 65 production source/package files matched the regenerated capture at that checkpoint. |
| Focused physical suite | 140/142, exit 1 | September 24 retained-source report; slow-pull sequence falls at tick 216, planted reversal at tick 343. Original focused report/log unavailable at their documented paths. |
| Historical quiet standing | Failed at all three headings | Earlier 2 s settling plus 30 s observation. Bodies remain upright with zero steps, but drift and speed limits fail. An upright state is insufficient. |
| Balance probes | Both fail | Recorded capture/force feasibility and pelvis-height failures remain unresolved. |
| Small-impulse diagnostic | Precision-sensitive residual identified | [Matched native f32/f64 comparison](small-impulse-diagnostic.md#result--numerical-precision-sensitivity-isolated): all twelve f64 cases are smooth and within 1.71%; f32 remains irregular. Exact state transfer, matching solver features, unchanged contact/limit regimes and exact hooked WASM replay are verified. The application outlier still exists; no tolerance or runtime changed. |
| Historical diagnostic validation | 25 tests pass; lint passes without warnings | [Post-H26 verification](checkpoints/2026-09-27/h25-h26-validation-complete/manifest.json) covers original motor/continuity assertions, baseline capture and corruption detection for binary snapshot archives. The native calibration executable builds successfully. Full typecheck last passed in [prior validation](checkpoints/2026-09-26/standing-tooling-validation.json); no TypeScript production file had changed at that checkpoint. These checks do not establish a complete application/physics gate. |
| Complete application and physics suites | Unverified after the retained repairs | The older September 23 checkpoint recorded 273/280 application tests and 6/63 physics scenarios passing. Those counts do not describe a new run on this working tree. |
| Five-cycle protocol, recovery, browser review, both Pages configurations | Acceptance open | No later complete integration result establishes acceptance. |

The [physics acceptance contract](physics-acceptance.md) retains independent numerical limits. The [merge ledger](merge-acceptance.md) maps requirements to tests, but its older table results are historical. The local Pages workflow keeps required verification failures blocking deployment; this document does not establish remote CI or deployment status.

## What is currently missing

The [prioritized TODO](../TODO.md) is the actionable backlog. The protocol room, playground, renderers, and controller infrastructure exist; their presence does not close the physical gates.

| Missing milestone | Current gap | Completion evidence |
| --- | --- | --- |
| Quiet standing | Recorded failure at all three headings; no accepted repair | Unchanged official standing contract plus structural regressions |
| Transfer, prediction, and stepping | Complete repeated physical sequence is unaccepted; two focused failures and both probes remain open | Repeated measured transfers, held-out prediction comparisons, then steps/release/reversal |
| Recovery, terrain, and browser integration | No complete result on the retained candidate; short/synthetic replays cover only part of the requirements | Full recovery/five-cycle and inspected browser results in both renderers |
| Release verification and durable evidence | No complete candidate-wide gate bundle; CI artifacts expire | Full suites, both Pages configurations, durable manifests/artifacts, and verified deployment gating |

The small-impulse outlier remains a separate diagnostic task. See [fresh capture instructions](evidence.md#fresh-baseline-capture) and [missing original artifacts](evidence.md#still-unavailable). Native touch and hardware GPU performance have no certification from synthetic replay.

## Investigation and next milestone

The September 26 report records a local causal intervention at ticks 175–176: changing the previous lower-chain load request reduced a forefoot angular-speed spike. Its offline replay isolates sensitivity to measured hull membership. This is evidence for a specific allocation discontinuity, not an accepted stationary controller repair.

H1–H4 did not meet their declared repair predictions. H6 and H7 failed their static load-jump bounds. The [new H8 evaluation](standing-h8-evaluation.md) passed static/local checks but failed every prolonged screen, including the upper-limb damping combination. H8 is rejected and remains diagnostic-only. New traces, exact replay, matching controls, explicit reset/infeasibility counts, and failed screens are preserved with verified digests. A passing local intervention does not establish successful standing, transfer, stepping, or recovery.

The next milestone remains quiet standing under the unchanged official three-heading contract, followed by repeatable measured transfer, individual steps, sequences, and integration. Preserve failed runs and thresholds. Restore the original source-scoped evidence or generate a fresh complete run before claiming a gate passes. See [evidence preservation](evidence.md#preserving-future-results).

The [H9–H11 continuation](standing-h9-contract.md) also produced no accepted repair. Soft projection of prior references and its combination with held posture references fail prolonged standing; hindfoot-preferred allocation fails the static discontinuity bound. These rejected results and selected peak traces are archived. A remaining forefoot spike occurs with continuous commanded loads, so load continuity alone does not explain every failure. The production controller and frozen limits remain unchanged.

The [post-H11 baseline](checkpoints/2026-09-26/standing-post-h11-baseline.json) records 255 source/configuration files, installed dependencies, executable digest, and preserved uncommitted work. All 65 production/package files still match the regenerated capture. The baseline itself executes no physical scenarios; its full worktree snapshot remains local-only under the named ignored evidence directory.

The [H12–H21 continuation](standing-h12-contract.md) preserves 174 additional
compressed artifacts in two verified archives. H20 reduces the local forefoot
response by 92.05% and passes both short-screen speed bounds at all headings,
but foot drift remains 0.01310–0.02018 m. H21's held-reference combination is
also rejected. Contact calibration shows that this installed engine reports
zero tangent impulse even when momentum balance proves friction acted; raw
tangent fields therefore cannot diagnose friction absence or saturation.
All 65 production/package files still match the regenerated capture.

The [H22–H24 archive](checkpoints/2026-09-27/h22-h24-evaluation/manifest.json)
adds 32 verified artifacts, including rejected idle-forcing/stance-task runs
and the observer's controller-phase repair. H23's large neck peak is post-fall;
it does not establish the initiating standing fault. The next physical milestone
remains the unchanged official quiet-standing gate.

The [H25–H26 and calibration archive](checkpoints/2026-09-27/h25-h26-calibration/manifest.json)
adds 55 verified artifacts, including rejected screens, full bounded event
traces, native snapshots, upstream-source receipts and source snapshots.
The [native calibration](rapier-constraint-calibration.md) qualifies the
position/velocity discrepancy and the separately confirmed upstream contact-
impulse reporting defect. No source-backed engine finding yet closes standing.

The subsequent [precision archive](checkpoints/2026-09-27/impulse-precision/manifest.json)
closes the separate small-impulse diagnosis through a controlled precision
comparison. All 65 production/package hashes remain unchanged. The fixed
assembly still collapses in f64, so its instability and quiet standing remain
open. [Nine original joint-motor tests and targeted lint pass](checkpoints/2026-09-27/impulse-precision-validation/manifest.json);
the original impulse assertion is unchanged.

The [H27–H29 archive](checkpoints/2026-09-27/h27-h29-evaluation/manifest.json)
contains 27 verified artifacts. [Hybrid translational constraints](standing-h27-contract.md)
lose support; the [upstream runtime comparison](standing-h28-contract.md) fails
two unchanged motor/inertia assertions and the standing speed bounds; the
[centered neutral target](standing-h29-contract.md) falls at all three headings.
All three H22 controls replay exactly, as does the pinned loader's heading-zero
control. All 65 production/package hashes remain unchanged. The original
runtime passes the 21 focused adapter/coordinate/motor tests through the same
loader. [Nine archive tests and targeted lint pass](checkpoints/2026-09-27/h27-h29-validation/manifest.json).
These rejected interventions do not change the open physical gates.

The [H30 transition](standing-h30-contract.md) and
[H31 centered initial posture](standing-h31-contract.md) also fail. H31 verifies
the centered construction and keeps all three headings upright, but foot drift
is 0.02550–0.03240 m and all speed bounds fail. H30 and H31 preserve six further
exact H22 control replays; no production change follows.

## Documentation verification

The documentation review checked local links, implementation settings, and harness discovery, which returned 63 scenarios. Discovery executes no physical scenarios. The repair also verified all six recovered artifact digests and the Playwright module-resolution recipe with the installed 1.62.1 runtime; the documented tooling install uses the 1.63.0 version pinned in CI and was not installed during this repair. Documentation repairs do not change controller behavior, refresh fixtures, or establish physics/browser acceptance.

The subsequent TODO review confirmed that all 12 source/package digests in the partial documentation checkpoint still match the working tree. It inspected the baseline script's prerequisites, H8 call sites, and local workflow coverage. This does not extend that partial fingerprint or establish a fresh test, browser, or remote CI result.

The first P0 implementation subsequently added explicit `--fresh` capture and verified it against the local checkout. Its [portable ledger](checkpoints/2026-09-26/fresh-baseline.json) includes 252 source/configuration hashes, installed direct dependency versions, executable information, and uncommitted-work digests. Five CLI regression tests cover absent and invalid history, staged/unstaged/deleted/untracked files, dependency failures, and overwrite prevention. The [validation manifest](checkpoints/2026-09-26/fresh-baseline-validation.json) records this tooling-only scope. These checks do not change the recorded physical failures above.
