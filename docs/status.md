# Current status

**2026-10-03 migration continuation:** the v24 standing candidate increases
supported COM restoration from 3 to 5 rad/m without changing acceptance limits
or motor ceilings. Preliminary native standing passes all six public-mode
cases; the freshly built WASM artifact now passes all six with unchanged source
and artifact hashes. Selected pulls, terrain and striker checks also pass
natively. The
gain-4 trial is rejected for native drift and a terrain structural failure.
Fresh verification passes all **73 Rust tests**, native/WASM Clippy and
formatting. All **15 native stages pass**, with byte-identical repeated
foundation, six-case public-mode standing, terrain and striker reports and
unchanged source hashes. All **10 final browser stages pass**, including six standing
cases, 42 terrain cases, physical striker, worker lifecycle and both renderers
at root/subpath origins. The normal preview build is byte-identical to the
verified root artifact. Final evidence is under
`evidence/migration-remaining-final-{native,browser}`.

Native quiet diagnostics now select protocol, playground or both modes, and
foundation verification repeats the six-case public-mode matrix. The old v23
native playground +60° trial fails foot drift at tick 1871, exposing the gap
in the previous protocol-only native diagnostic. Scoped disturbance exit
codes now reflect their actual results while release admission stays false;
invalid profiles leave no evidence directory. Windows browser builds preserve
the inherited Cargo/Node paths even when the environment spells `Path`.

Fresh v24 strict stepping is **0/9**, crouch-left recovery records zero
recoveries in 25 seconds, and isolated-box contact stress passes **200/216**.
The full retained-app suite reports **375 pass, four fail, seven skip**: the
same slow-pull, planted-reversal and two strong-pull/fall failures remain.
Fresh idle browser timing fails on both renderers: WebGL2/WebGPU record
197/483 core tick deadline misses at simulation/wall ratios 1.000/0.986.
Source and artifact hashes remain unchanged; full performance admission stays
open (`evidence/migration-remaining-performance/report.json`).
Strict stepping, automatic recovery/cycles, contact stress, full disturbance
and terrain traversal coverage, the retained-app failures and hardware timing
still block full migration and production promotion. See [the continuation
record](rust-rework.md#migration-continuation--2026-10-03).

**2026-10-02 migration integration:** this source checkpoint includes the Rust
preview, audited engine, native/WASM tooling, renderer, UI, terrain and striker.
Production remains on the existing application while
physics and performance acceptance are open. Cargo output from the standalone
vendor build is now ignored along with workspace builds and local evidence.

The native acceptance CLI rejects unknown/misplaced diagnostics, duplicate or
missing options and nonfinite headings before creating evidence. Four new
argument tests bring the passing Rust suite to **71 tests**. An executable-level
typo check confirms exit 2 without an evidence directory. Native/WASM Clippy,
formatting, TypeScript, the application build, Pages export/asset verification
and four JavaScript tooling tests pass; JavaScript lint has no errors and eight
existing warnings. The full app
suite reports **374 pass, four fail, seven skip**: slow pull falls at tick 304,
planted reversal at tick 228, and two strong-pull/fall assertions fail. Evidence:
`evidence/migration-final/` and `evidence/migration-final-native/`.

The final native runner passes all 13 stages. Both foundation runs pass gates
A–C and all eight selected pulls; both 21-case terrain runs and both three-case
striker runs pass. Every repeated report is byte-identical, and source hashes
remain unchanged (`implementedChecksPassed: true`, `repeatable: true`,
`releaseAccepted: false`). Gate D and full release admission remain incomplete.

Fresh v23 WASM standing passes **five of six cases**. The +60° playground case
fails left-forefoot drift at tick 1902, substep 3: 0.010000321 m against the
unchanged 0.010 m limit. All three protocol headings and the other two playground
headings pass the full two-second warmup plus 30-second observation. Source and
artifact hashes remain unchanged. This supersedes the older all-six passing
v22 result for the current source. Evidence:
`evidence/migration-final-browser/quiet/report.json`.

The combined browser verifier completes all ten stages: **nine pass**, with
quiet standing the sole failed stage. Root/subpath builds, 39 UI checks at each
origin, both fault suites, worker lifecycle, all 42 terrain cases and all six
striker cases pass. Desktop and mobile screenshots were inspected. See
`evidence/migration-final-browser/execution.json` and its per-stage reports.

Native foundation verification now repeats terrain and striker cases as well
as gates A–C and selected pulls. Browser verification includes the existing
42 terrain cases and physical striker checks against the freshly built root
artifact, plus root/subpath UI, faults, worker lifecycle and quiet standing.
Neither command claims recovery, terrain traversal or hardware-performance
admission. The browser standing failure, strict stepping, automatic
recovery/cycles, contact stress coverage,
the four retained-app regressions and performance remain release blockers.

**2026-10-02 articulation feasibility and further motion diagnostics:** a new
native-only diagnostic preserves all 25 canonical bodies, colliders and joint
definitions while testing reduced-coordinate articulation. Full conversion
cannot integrate: Rapier's two-axis wrist branches are unimplemented, and its
spherical coordinates do not match the anatomical motor/limit coordinates.
A hinge-only hybrid keeps all anatomy and integrates. Read-only final
generalized velocities expose stale rigid-body velocity reporting at the last
solver substep: the floorless hybrid passes the unchanged momentum threshold
over 240 substeps when measured from those generalized velocities. This is
diagnostic evidence, not permission to ignore the ordinary body-state failure.
With real floor contact, the hybrid exceeds the unchanged right-thigh angular
limit tolerance. The new vector contact-momentum diagnostic also fails for
the impulse baseline, so that comparison is not a qualified contact gate.
No controller migration, vendor change or performance promotion follows.
See the detailed results in [the Rust record](rust-rework.md).

Further capture, contact-moment and stance-frame trials do not fix the four
application failures. The stance-frame variant passes the two overload/input
tests but makes the two gentle-pull falls earlier; it was rejected. Recovery
trials using a virtual upright pelvis, canonical resting targets or measured
gravity/support feedforward each still produce zero recoveries and stall in
bracing. All these runtime experiments were restored byte-for-byte to the
previous checkpoint. The app's 374 pass / four fail / seven skip result and
disabled recovery default remain current. Commit, push and merge are pending.

**2026-10-02 recovery and solver diagnostics:** Rust now has an opt-in,
contact-gated recovery prototype and 24 exported landed-pose seeds, constructed
before integration. Recovery changes bounded joint motor intent; it never moves
dynamic bodies directly. The default `recovery_enabled` is false. No dynamic
recovery trial has passed, so this is unfinished implementation, not admission.
Repeated crouch trials still tip forward; the foot-reserve trial
also trips the unchanged 5 mm self-contact limit at 5.113 mm
(`evidence/finish-rust-recovery-crouch-21`). A subsequent trial returns a failed
rise to passive falling and resumes bracing after settling, without a structural
failure, but remains in bracing at 25 seconds (`finish-rust-recovery-crouch-22`).
The full Rust suite passes 64 tests (`finish-rust-tests-26.log`), including six
recovery checks for seed validation, read-only planning, coupled interpolation,
absent support, torque ceilings, input lockout, impact interruption and passive
fallback after a failed rise. Native/WASM Clippy passes on the retained source
(`finish-rust-lint-29.log`), as do TypeScript and the changed scripts' lint
(`finish-recovery-typecheck-28.log`, `finish-recovery-scripts-lint-28.log`).
An arm-extension experiment did not advance beyond bracing and was reverted.
The final replay (`finish-rust-recovery-crouch-24`) preserves the earlier
initial/final snapshots and every sampled physical snapshot exactly, still with
zero recoveries and no structural failure.
The diagnostic now requires a further second of measured standing after motor
handoff, in addition to the second required before completion. Actual repeated
strike/recovery cycles and browser recovery remain unqualified.

The 10 outer/32 internal/64 falling-pass diagnostic passes all three native
quiet-standing cases, all three impacts, 21 terrain checks and eight selected
pulls. Reports are `evidence/finish-rust-{quiet,impact,terrain,pulls}-outer10-pgs32-21`.
It still fails actual browser timing: 278/1,819 charged update misses on WebGL2
and 192/1,804 on WebGPU, with maxima 50.7/27.5 ms against 16.667 ms. Round-trip
p99 is 130.1/48.5 ms against 20 ms. The run uses the frozen `rust/dist/striker-10`
artifact with an explicit worker profile override; served/local asset hashes,
the profile and observed source hashes remain unchanged. See
`evidence/finish-rust-performance-outer10-pgs32-21/report.json`. This candidate
is not promoted. The performance probe now records optional profile/asset
provenance; its targeted lint passes (`finish-profile-probe-lint-24.log`).

The four retained-app motion failures, automatic recovery, strict stepping,
terrain traversal and performance remain release blockers. The default physics
profile and production entrypoint remain unchanged. No commit, push or merge
has occurred.

**2026-10-02 retained-app impact repair:** the left-hand chest-drag regression
is fixed by increasing internal solver passes during recovery from 32 to 64.
Standing keeps its configured budget, restored immediately after each recovery
integration. The measured peak joint-anchor separation falls from 27.4 mm to
8.4 mm under the unchanged 10 mm assertion; both chest-drag cases retain their
contact, penetration and ownership checks. The contact/motor selection passes
28/28 (`evidence/finish-chest-regressions-11.log`). No test deadline, body pose,
motor ceiling or contact threshold was changed.

The complete build/test run in `evidence/finish-npm-test-11.log` confirms the
contact repair but reports 372 pass, five fail and seven skip. Four failures are
the existing slow-pull, planted-reversal and two physical-overload/lockout
cases. The fifth is a concurrent-test timing failure in the H77 selector test.
That selector now uses a controlled clock; a separate boundary regression
admits exactly 8 ms and requires a timeout transition at 8.001 ms. All four H77
tests pass in `evidence/finish-implicit-clock-12.log`. The final complete
application test replay reports **374/385 passing, four failures and seven
skips** in `evidence/finish-node-test-12.log`; only the four existing motion
failures remain. Real runtime/browser timing limits remain intact. Typecheck
passes; lint has no errors and eight unchanged warnings
(`finish-typecheck-11.log`, `finish-lint-11.log`); the final test edit also passes
targeted lint (`finish-implicit-lint-12.log`).

Two native solver-budget diagnostics remain unpromoted. Four outer/32 internal
passes fail all three quiet-standing cases; eight outer/32 internal passes
pass the three 30-second quiet cases and all three impacts, but fail gentle
rubble at 5.188 mm pelvis/hand overlap (5 mm limit) and the rotated right-hand
pull at 0.55958 m pelvis height (0.56 m limit). Evidence is in
`evidence/finish-rust-{quiet,impact,terrain,pulls}-outer8-pgs32-12` and
`evidence/finish-rust-quiet-outer4-pgs32-12`. The v23 source/default profile and
browser artifact remain unchanged; these diagnostics do not qualify timing.

The balance geometry, trigger-margin, recentering and pelvis-gain experiments
did not repair the motion failures and were reverted; their failing logs remain
under `evidence/finish-balance-*-11.*`. `BalanceController.ts` and the balance
diagnostic script retain their pre-experiment hashes. Automatic Rust recovery,
strict steps, terrain traversal and performance remain open; no commit, push,
merge or deployment has occurred.

**2026-10-02 physical striker integration:** Rust now owns the protocol room
and its kinematic impact head. Positioning checks the complete machine against
current and predicted body geometry; retraction checks clearance and refuses
new/deeper overlap. Only a measured solver impulse increments the strike count.
The button and P shortcut reject paused/busy requests, snapshots expose the
integrated apparatus, and falls cancel body input without replacing bodies.
The separate protocol geometry export preserves the frozen anatomy/course data.

The current candidate is `rust-physics-v23-passive-fall`. Falling uses canonical
passive damping and cubic resistance near joint limits, under the existing
individual/aggregate torque ceilings. Standing retains 20 internal solver
passes; falling uses 32, explicitly recorded in the profile. The three native
heading/impact cases and all 21 native terrain cases pass with unchanged
structural thresholds. All 58 Rust tests and native/WASM Clippy pass. Evidence:
`evidence/finish-rust-striker-passive-impact-10`,
`evidence/finish-rust-striker-passive-terrain-10`,
`evidence/finish-rust-striker-tests-10.log`, and
`evidence/finish-rust-striker-lint-10.log`.
The frozen final browser artifact passes six impact cases and a second strike
in each backend, 42 terrain cases, and 39 existing interaction checks across
WebGPU/WebGL2. The mobile machine framing is corrected and visually checked.
See the reports in `finish-rust-striker-browser-10`,
`finish-rust-striker-terrain-10`, and `finish-rust-striker-ui-10` under `evidence/`;
`finish-rust-striker-frozen-10` preserves the source and verification index.

The earlier `finish-rust-striker-foundation-06` passes the complete scoped
foundation twice with identical reports, but predates v23 passive fall changes;
it is not full v23 admission. The five retained-app failures, automatic recovery,
strict stepping, terrain traversal and performance remain open. No commit,
push, merge or production entrypoint change has been made.

**2026-10-02 terrain integration:** the Rust playground now creates the canonical
course solids, spawns the connected anatomy at the selected station, measures
upward support from actual terrain contacts, and integrates the wobble decks.
Station/difficulty controls create a fresh generation while preserving pause;
the renderer uses the same canonical triangles and integrated environment poses.
All 21 native two-second structural/contact cases and all 42 actual WASM cases
(21 each on WebGL2/WebGPU) pass. Browser checks also verify deck pause/resume,
station changes while paused, and removal of course geometry on returning to
the protocol. Evidence: `evidence/finish-rust-terrain-native-04/terrain.json`
and `evidence/finish-rust-terrain-browser-06/report.json`.
All 50 Rust tests pass; the native foundation and eight selected pulls repeat
with byte-identical output. Native/WASM formatting and Clippy pass after a
test-module ordering repair, whose unchanged implementation and test content
are checked in `evidence/finish-rust-terrain-checkpoint-06.json`. The existing
browser interaction matrix also passes 39 checks across both backends
(`evidence/finish-rust-terrain-ui-06/report.json`).

This is scoped terrain integration evidence. Five slope/rubble setups fall
within the two-second native observation; successful balance, terrain traversal
and recovery are not qualified. Flat and hurdles start on the base floor, so
their idle cases do not claim obstacle impact coverage. The five retained-app
failures below remain open. A longer unchanged-input diagnostic observes the
two strong-pull falls at ticks 193 and 250, after their existing 180-tick test
deadline (`evidence/finish-overload-observation-03.log`); the tests and thresholds
have not been relaxed. Striker, recovery, strict stepping and performance still
block release, commit/push/merge remain pending, and production is unchanged.

**2026-10-02 regression repair in progress:** six of the eleven recorded
retained-app failures have been repaired in focused checks. Recovery foot
planning now requires an exact, level floor plant and includes the anatomical
hip line in its candidate search. Full-extension arm reconstruction is stable
across headings, mutable rest poses no longer alias canonical anatomy, and the
balanced half-kneel fixture carries its projected mass inside actual loaded
contacts. Native stream initial snapshots were refreshed without changing
historical input commands or recorded transfers. The forward-plant and arm
tests now use the actual fixture geometry and legal elbow reach.

The current focused recovery/geometry matrix passes 63/63. The dynamic matrix
still fails five tests: slow pull/reversal, planted reversal, left-hand chest
drag joint separation, and the two overpowering-pull fall/lockout cases.
Two experimental step changes were rejected and removed after broader checks
exposed transfer regressions. The final full suite records **372 passes,
5 failures, 7 skips** (384 tests, including two added regressions) in
`evidence/finish-npm-test-02.log`. Build and typecheck pass; lint reports no
errors and the same eight warnings. Focused results are preserved in
`evidence/finish-recovery-geometry-05.log` and
`evidence/finish-dynamic-independent-poses.log`. The previous full run's two
additional controller-support failures are resolved: exact foot search now
includes the anatomical hip line, and movement fixtures supply a reachable
crouch instead of relying on shared rest-pose mutation.
Rust striker/terrain, automatic recovery, strict stepping and performance
qualification remain open. Work is on `codex/finish-rust-rework`; no commit,
push, merge or deployment has been performed in this continuation.

**2026-10-02 migration continuation:** Rust now observes physical falls,
updates the fall counter and cancels active/future body input. Reset opens a
fresh trial; automatic recovery is still absent. Pinned tool bootstrap and
root/subpath browser verification are reproducible package commands.
The unchanged v22 Coulomb-contact profile passes native gates A–C and eight
selected pull probes twice with byte-identical results. All 46 Rust tests and
all six WASM standing trials pass. Root/subpath browser checks pass 78 assertions
across WebGPU/WebGL2, plus 35 worker lifecycle checks and 24 fault checks.
The final preview matches the tested root artifact byte for byte.
The last nine-fixture step diagnostic had zero passes and was not rerun in this
continuation. Physical
striker/stations, recovery and full performance/release qualification remain
incomplete; production stays on TypeScript.
The current browser performance probe also fails update/IPC timing in both
backends (worker round-trip p99 73.8/94.1 ms; required maximum 20 ms).

The full retained application suite records **364 passes, 11 failures,
7 skips**. These 11 failures also occurred in the preceding run; its H77 selector
failure did not reproduce. Build and typecheck pass; lint has eight existing
warnings and no errors. These failures remain release blockers.
See the [current verification record](rust-rework.md#migration-continuation--2026-10-02)
for scope, commands and local evidence. Earlier checkpoints below are historical.

**2026-10-01 Rust checkpoint:** the isolated [Rust rework](rust-rework.md)
passes its implemented native gates A–C (geometry/contact/joints, loaded feet and
chains, and 2 s settle + 30 s quiet standing at three headings).
`rust-migration-frozen-01` records two identical native executions on preserved
source and passes all eight selected pull probes, repairing the three v13
failures. Native lifecycle controls are tested. Gate D remains incomplete because
corrective-step counts, swing-release and full coverage are not qualified;
E–I and full browser/release qualification remain incomplete. Production still
uses the legacy TypeScript entrypoint. Native quiet-standing success does not
resolve the browser candidates' failures below.

The opt-in corrective-step follow-up records measured liftoff and touchdown in
four fixtures, but none of the nine strict recoverable fixtures pass. Post-step
falls, final foot orientation, lateral transfer, reversal and swing-release
remain unresolved. The candidate stays disabled; the narrower native pull
passes above do not admit completed steps. See the [step evidence](rust-rework.md#corrective-step-follow-up).

**2026-10-01 renderer checkpoint:** both browser modes now expose Auto/Low/High
quality, shared presentation metrics, and redraws on demand while paused.
The local `evidence/sandbox-acceptance/report.json` records 16 before/after replay
combinations, four UI combinations, and four 60 s live trials without reported
errors. Its 18 recorded scene/runtime/core hashes still match the reviewed tree;
this is a partial source fingerprint. Live simulation/wall-time ratios are
0.416–0.746, so these trials do not establish real-time physics performance.
The report does not qualify reference hardware or GPU execution. See
[rendering results and reproduction](rendering.md) and [local evidence](evidence.md#rust-and-renderer-local-evidence).

**H78–H80 bounded result, 2026-09-30:** the [coordinated contact-force
investigation](standing-h78-h80-contract.md) has exhausted all three frozen,
opt-in candidates. H78 and H79 fail the 8 ms controller deadline. H80 fails two
references on that deadline (8.0155/8.468 ms); the -5 mm reference reaches tick
120 but fails entry/hold with 74.49 cm pelvis error, 11.96 cm foot error,
2.972 m/s linear speed and 2.774 rad/s angular speed. At its first admissible
allocation, requested horizontal restoration already changes direction; native
command readback remains within tolerance. This identifies the earliest observed
discrepancy, not the complete physical cause. All native first-failure replays
match 25 bodies exactly. No standing or transfer operating range is validated.

The implementation includes constrained allocation under combined motor ceilings,
one serializable support state shared with a non-mutating forecast, complete
controller-state snapshots, explicit H78–H80 selectors, and the previously
unfinished two-renderer browser performance stage. Forecast admission stays
disabled. Local-return, prediction validation, slow-pull/reversal acceptance,
short screens, official standing, sustained/held-out and actual browser gates
remain incomplete because feasibility failed. There is no fourth attempt.

Frozen checks: typecheck, lint and build exit 0 (lint: seven warnings); regressions are
80/82 pass, one failure and one skip. The strong-pull physical-fall failure is
reproduced on the exact saved pre-change source. Archive compatibility passes
inside acceptance (and skips without its archive in the standalone suite).
Source and manifest fingerprints remain unchanged during each completed run.
Subsequent shared-workspace renderer/runtime edits are preserved and are outside
this result's source scope. A verified `evidence/standing-h80-v1/frozen-workspace`
matches all 354 frozen source/config fingerprints for reproduction.
The dirty starting tree and historical evidence are preserved; normal startup
stays legacy. See [evidence storage and run paths](evidence.md#h78h80-local-evidence)
before copying the workspace: these new evidence directories use verified
junctions to available storage. Recovery, terrain and release acceptance remain
open.

**H76/H77 standing result, final verification 2026-09-28:** [H76](standing-h76-contract.md) now
freezes a successful six-snapshot scalar native build/read diagnostic with zero
physics steps; its rows appear coherent but do not explain within-step forefoot
motion. [H77](standing-h77-contract.md) implements opt-in implicit native posture
feedback and passes current-state torque/readback equivalence, but improves only
one of three preserved references. The official acceptance operation fails all
three reference entries (ticks 132/120/120), before local return or later gates.
Output-neutral actual-substep traces show responsive, uncapped joint rows and
select the contact-wrench/load-distribution evidence branch. H77 is not promoted;
normal startup remains legacy, and H74/H75 remain immutable failed experiments.
Final checks: typecheck and production build pass; lint exits 0 with 6 warnings;
the complete unit run is 325/339 pass with 13 failures and 1 skip; the focused
balance/controller run is 25/28 pass, with both active-step commitment tests now
passing and the separate slow-pull/reversal tests still failing at ticks 304 and
228. The full physics harness is 6/63 pass. The H77 standing operation itself
fails feasibility and leaves every later acceptance stage incomplete.

**Recovery performance investigation, 2026-09-27:** repeated foot-placement searches during rolling are skipped until a placement phase needs them. A 10-second strike replay fell from 146 seconds to 20–27 seconds on this host; it still does not run in real time or complete get-up. Lower solver counts and rolling motor trials were rejected. Per-tick diagnostics expose a transient joint-limit violation missed by coarse sampling. See the [measurement, retained changes and limitations](recovery-responsiveness-profile.md).

**Recovery launch correction, 2026-09-27:** the application now integrates recovery motors with native contact/joint constraints instead of applying frame-sized torque impulses first. A real strike reproduced the old floor-to-ceiling launch; the corrected 10-second replay remains near the floor (maximum pelvis height after landing 0.1222 m). The character still stalls while rolling toward a brace, so get-up acceptance remains open. See [cause, comparison and validation](recovery-native-actuation-fix.md). The standing investigations below retain their recorded source scope; this correction does not establish quiet standing.

Updated 2026-09-27 through H77-v1. **Quiet standing remains the first unfinished TODO.** H74/H75 remain rejected and frozen. H76 closes only the static native build/read gap. H77's native-PD request identity passes, but its reference feasibility and promotion gates fail. No physical acceptance checkbox closes.

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
| Complete application and physics suites | Failed on the final H77 source | The complete unit suite is 325/339 pass (13 fail, 1 skip); all 63 physics scenarios ran, with 6 pass and 57 fail. Typecheck/build pass and lint has 0 errors/6 warnings. These checks do not override failed H77 feasibility. |
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

The 2026-10-01 refresh aligns the overview, architecture, index, standing
selection, browser/rendering guides, TODO and evidence summary with the existing
working tree and local reports. Link/anchor checks cover 374 local references in
ten updated documents; 123 historical checkpoint links remain unavailable as
documented in the evidence guide. All other checked references resolve, and the
documentation diff has no whitespace errors. No physics or browser scenarios
were rerun for this documentation-only refresh.

The documentation review checked local links, implementation settings, and harness discovery, which returned 63 scenarios. Discovery executes no physical scenarios. The repair also verified all six recovered artifact digests and the Playwright module-resolution recipe with the installed 1.62.1 runtime; the documented tooling install uses the 1.63.0 version pinned in CI and was not installed during this repair. Documentation repairs do not change controller behavior, refresh fixtures, or establish physics/browser acceptance.

The subsequent TODO review confirmed that all 12 source/package digests in the partial documentation checkpoint still match the working tree. It inspected the baseline script's prerequisites, H8 call sites, and local workflow coverage. This does not extend that partial fingerprint or establish a fresh test, browser, or remote CI result.

The first P0 implementation subsequently added explicit `--fresh` capture and verified it against the local checkout. Its [portable ledger](checkpoints/2026-09-26/fresh-baseline.json) includes 252 source/configuration hashes, installed direct dependency versions, executable information, and uncommitted-work digests. Five CLI regression tests cover absent and invalid history, staged/unstaged/deleted/untracked files, dependency failures, and overwrite prevention. The [validation manifest](checkpoints/2026-09-26/fresh-baseline-validation.json) records this tooling-only scope. These checks do not change the recorded physical failures above.
