# Rust rework implementation and verification record

**Incomplete. Production remains on the legacy entrypoint.** The current Rust
candidate is `rust-physics-v24-com-restoration`. Its freshly built WASM worker
passes all six standing cases, superseding the v23 +60° playground drift
failure. All 15 native foundation stages pass with byte-identical repeated
reports, including the six-case public-mode matrix. All ten browser integration
stages also pass. Earlier simplified-contact, v22 and v23 results remain historical.
All nine strict stepping fixtures still fail. Recovery, repeated strike/recovery cycles,
terrain balance/traversal, full actuator/input qualification and performance remain open.
Builds and UI checks do not establish complete physical or release acceptance.

## Migration continuation — 2026-10-03

The supported center-of-mass restoration gain changes from 3 to 5 rad/m.
Motor ceilings, anatomy, contact settings, fixed stepping and acceptance limits
are unchanged. A gain-4 trial passed the six preliminary WASM cases but failed
native +60° playground standing and the gentle rubble structural check, and
was rejected. Gain 5 passes the preliminary six-case native/WASM matrices,
eight selected native pulls, 21 terrain checks and three striker cases.
The preliminary browser receipts are not final verification: their source
snapshot changed while independent CLI tests were being edited. Final checks
use freshly built default artifacts and frozen runtime sources. The final
six-case WASM standing run passes with `sourceUnchanged: true` and
`artifactsUnchanged: true` in
`evidence/migration-remaining-final-browser/quiet/report.json`.
All ten stages of `evidence/migration-remaining-final-browser/execution.json`
pass, including root/subpath builds, UI/fault checks, worker lifecycle,
42 terrain cases and physical striker checks. Formatting, native/WASM Clippy,
all 73 Rust tests, five JavaScript tooling tests, TypeScript and the application
build pass. JavaScript lint has no errors and eight existing warnings.
The final native runner passes all 15 stages, with byte-identical repeated
foundation, public-mode standing, terrain and striker reports and unchanged
source hashes (`evidence/migration-remaining-final-native/execution.json`).
The normal `rust/dist/app` output is byte-identical to the verified root
artifact (`evidence/migration-remaining-default-artifact.json`).

The native quiet CLI now accepts `--mode protocol|playground|all`; omitting it
preserves the earlier protocol-only diagnostic. Foundation verification
repeats `--mode all` so it observes both actual public-mode assemblies at all
three headings. This reproduces a previously missed v23 native playground
drift failure at tick 1871 rather than reporting protocol success as coverage
of both modes. Invalid profiles are rejected before evidence creation.

Selected disturbance diagnostics and their wrapper now return success when
their requested checks pass, while retaining `releaseAccepted: false` and the
explicit partial scope. Executable-level regressions distinguish a passing
physical pull from a failed strict-step request and reject invalid input
without creating evidence. Browser build environments also preserve inherited
Windows tool paths when the variable is spelled `Path`, avoiding a duplicate
`PATH` that hid Cargo from child processes.

Fresh v24 strict stepping remains 0/9, and the crouch-left recovery trial remains
in bracing after 25 seconds with zero recoveries. Full isolated-box contact
stress passes 200/216. Those diagnostics do not qualify the new default for
stepping, recovery or full contact stress. Full
disturbance and input coverage, recovery/cycles, terrain traversal and actual
hardware performance remain open. Production is not switched to Rust.
The retained-app suite reports 375 passes, four failures and seven skips;
the same slow-pull/reversal and two strong-pull/fall assertions remain failed.
Fresh idle timing against the exact default preview also fails on WebGL2 and
WebGPU, with 197/483 core tick deadline misses despite simulation/wall ratios
of 1.000/0.986. Source and artifact hashes remain unchanged. This is an idle
diagnostic, not full hardware, input latency, process memory or soak admission
(`evidence/migration-remaining-performance/report.json`).
Local trial reports and logs are under `evidence/migration-remaining-*`.

## Migration integration — 2026-10-02

This integration checkpoint includes the Rust source preview and pinned tools.
The current verification record is at the top of [current status](status.md).
The native runner now repeats the implemented foundation, terrain and striker
cases, and the combined browser runner includes both public-mode integrations.
The native CLI rejects invalid diagnostic requests before evidence creation.
Rust CI runs on affected pushes to `main` as well as pull requests, preserving
verification evidence without publishing the preview. Generated Cargo output,
including the independent vendor build, stays local.

This source integration leaves production on its existing entrypoint. Release
acceptance remains incomplete; the historical results below retain their
original scope and are superseded only by explicit newer checks.

## Articulation feasibility — 2026-10-02

`lh-acceptance <fresh-output> --diagnose-articulation` is a native-only,
explicitly unaccepted diagnostic. It assembles all 25 canonical dynamic bodies
with exact collider and GenericJoint data, preserving the free root, total mass
and all 46 angular DOFs. Full conversion reaches Rapier's unimplemented
two-axis wrist Jacobian on its first step; isolated wrist integration also
panics. A separate closed spherical-coordinate path returns zero accumulated
coordinates while anatomical quaternion-derived coordinates differ by
0.00607967 rad. The default impulse-joint controller is unchanged. Evidence:
`evidence/articulation-feasibility-20261002/articulation.json`.

The companion `--diagnose-hybrid-articulation` command converts only the 12
single-axis joints. Four multibodies contain 16 links, with nine standalone
bodies and the 12 existing multi-axis impulse joints; 60 bilateral locking rows
are removed without changing anatomy. This remains an opt-in passive diagnostic.
It does not run the ordinary controller or claim quiet-standing admission.

The first floorless step reveals stale rigid-body velocity writeback, rather
than demonstrated generalized momentum loss: its stored-body residual is
0.049699325 N·s while the read-only body Jacobian times final generalized
velocity gives 2.38428e-7 N·s. A separate damping-disabled control has the same
reporting discrepancy. Over 240 substeps the hybrid's generalized momentum
residual peaks at 0.00110197 N·s, below the unchanged 0.002 N·s gate, with anchors
and angular limits passing. The stored-body gate remains failed. Rapier's final
multibody velocity writeback does not refresh the rigid-body velocities; no
vendor or body-state correction is applied by the diagnostic.

With floor contact the hybrid violates the right-thigh angular tolerance at
substep 143 (0.0626842 rad against 0.06). Generalized momentum residual also
fails. The impulse baseline itself fails this new vector contact-momentum
diagnostic, so those residuals do not establish a qualified contact comparison.
Rapier's generalized damping additionally differs from the configured body
angular damping, which it skips for multibody links. Those semantics remain
unresolved. Passive early-stop timings are not accepted performance evidence.
The reports in `evidence/hybrid-articulation-feasibility-20261002` and
`evidence/hybrid-articulation-240-20261002` retain these failures explicitly.
The final report and verification logs are in
`evidence/hybrid-articulation-final-20261002`. All three scoped articulation
tests, native/WASM Clippy and formatting pass. The earlier full 55-test sim run
predates the hybrid extension; it is not reported as a fresh full-suite result.

Further legacy-app trials (capture checks, explicit centroidal moment,
post-release fall observation and stance-frame changes) remain rejected. The
stance-frame variants pass the two overload/lockout tests but make gentle pulls
fall earlier. Native crouch trials 35, 36 and 39 respectively test virtual root
orientation, canonical-rest targets and gravity/contact support feedforward;
all end with zero recoveries and stalled bracing, with no structural failure.
The runtime sources are restored byte-for-byte to the pre-experiment checkpoint.
See `evidence/finish-*-3*-tests.log`, `finish-stance-*-4*-tests.log`, and
`finish-rust-recovery-crouch-{35,36,39}/recovery.json`. No acceptance limit,
production entrypoint or default profile changes.
`evidence/finish-articulation-frozen-43/checkpoint.json` records the restored
runtime hashes and the diagnostic source delta from recovery checkpoint 29.

## Recovery prototype and solver diagnostics — 2026-10-02

An opt-in `recovery_enabled` profile field now exposes a native recovery
prototype. It uses persistent measured support to start bounded motor intent;
a failed standing rise returns to passive falling before another settled
attempt. The 24 exported landed poses initialize bodies before integration.
Seed validation, motor planning and input/interruption checks do not establish
dynamic recovery. The current crouch case returns to bracing and remains there
at 25 seconds; no recovery fixture or actual strike/recovery cycle is admitted.
The default remains disabled. The native diagnostic requires one second of
measured standing before handoff and a further second under ordinary standing
control. See the latest evidence and counts in [current status](status.md).

The 10 outer/32 internal/64 falling-pass diagnostic passes the 35 scoped native
quiet/impact/terrain/pull cases. Actual browser idle timing still fails the
frozen update and round-trip deadlines, so this profile is not promoted.
The performance probe accepts optional profile and exact artifact arguments:

```sh
node scripts/probe-rust-browser-performance.mjs <fresh-output> <origin> all <profile.json> <artifact-directory>
```

The override is sent through the worker's existing initialization profile
input. The report preserves its digest, initialization receipt, and matching
served/local artifact digests. It remains an idle diagnostic, not release
performance admission. Production and the default physics profile are unchanged.

## Physical striker integration — 2026-10-02

The protocol owns fixed room boundaries and a kinematic impact head. A separate
`protocol-v1.json` export supplies the canonical machine pieces, stow height,
room dimensions and collision head; `canonical-v1.json` remains unchanged.
The complete visible assembly is checked during positioning and disabled
retraction, including predicted limb motion during descent. Impossible paths
remain unavailable rather than moving through anatomy. Room-constrained misses
do not increment the counter. A strike counts only after a current solver
impulse exceeds 0.25 Ns, once per stroke. The 4 m/s stroke and retained
position/retraction timings are preserved; no force, velocity or anatomical
pose setter substitutes for contact.

The Strike button and P shortcut are connected through the worker command
boundary. Paused, hidden, busy and future-dated strikes cannot queue delayed
attempts. A measured impact changes state and cancels current/future body input;
the same dynamic bodies continue falling. The renderer uses the integrated
head transform for all four canonical machine pieces and draws the inner room
bounds. Camera projection preserves horizontal framing on narrow viewports.

V23 adds the retained passive joint behavior during falls: canonical damping,
a cubic restoring spring inside each anatomical soft limit zone, and the
existing native per-axis and aggregate torque ceilings. It supplies no standing
or get-up posture while fallen. Disabling all motors had exposed a native slope
ankle-limit failure; reducing passive gains by the active-posture scaling also
left an impact failure. Both failed diagnostics are retained. Canonical passive
gains plus 32 internal solver passes during falls pass the final three-heading
impact and 21-case terrain diagnostics. Quiet standing keeps its prior 20-pass
budget. No structural tolerance or performance budget was relaxed.

Current native evidence:

- `evidence/finish-rust-striker-passive-impact-10`: one measured impact,
  physical descent below 0.56 m, unchanged dynamic ownership, and completed
  retraction at 0°, +60° and −45°.
- `evidence/finish-rust-striker-passive-terrain-10`: all 21 short terrain
  structural/contact cases pass after the passive-fall correction.
- `evidence/finish-rust-striker-tests-10.log`: all 58 Rust tests pass, including
  room misses/bounds, blocked apparatus paths, lifecycle rejection, passive
  resistance bounds, and portrait/landscape machine projection.
- `evidence/finish-rust-striker-lint-10.log`: formatting and native/WASM Clippy pass.
- `evidence/finish-rust-striker-build-manifest-10.json`: source hashes are
  unchanged across the final browser build and include the native executable
  and delivered artifact fingerprints.

The exact final artifact passes six browser impact cases (three headings per
WebGPU/WebGL2) in `evidence/finish-rust-striker-browser-10/report.json`, including
button/keyboard input, paused and busy rejection, apparatus/body pause, a
second impact in the same trial, and mobile touch. All 42 terrain browser cases
also pass (`finish-rust-striker-terrain-10`), as do the existing 39 browser
interaction checks (`finish-rust-striker-ui-10`). The desktop and mobile images
were inspected; the complete stowed machine fits the revised narrow viewport,
and heading selection uses the actual UI options. These are physical/rendering
checks, not automatic recovery or timing admission. Final sources and the
verification index are preserved in `evidence/finish-rust-striker-frozen-10`.

The two byte-identical scoped foundation runs in
`evidence/finish-rust-striker-foundation-06` passed before passive fall changes;
their evidence is retained without promoting it to full v23 admission.
Automatic get-up, five complete strike/fall/recovery cycles, strict steps and
the frozen performance matrix remain release blockers.

## Terrain integration — 2026-10-02

The playground owns canonical fixed course colliders and kinematic wobble decks.
All seven stations and three difficulties use the retained application's course
definitions, tiled triangular prisms and footprint-based spawn height. Anatomy
remains dynamic; scene changes replace the trial atomically, cancel old input,
preserve pause, and increment generation. Upward solver impulses from enabled
terrain surfaces participate in support measurement. Fall height is relative
to the supporting surface; the inherited settling grace does not postpone a
floor-disabled fall. The renderer draws canonical course triangles with the
integrated piece poses; presentation never writes body transforms.

`lh-acceptance <fresh-directory> --diagnose-terrain` records initial/final states
and actual terrain loads for 21 two-second cases. All pass structural/contact
checks in `evidence/finish-rust-terrain-native-04/terrain.json`. Five slope/rubble
cases end falling. Flat and hurdles begin on the base floor, so their stationary
cases do not claim hurdle collision coverage or successful traversal.

`node scripts/verify-rust-terrain.mjs <fresh-directory> <origin> <artifact>` checks
all 21 setups through each real browser renderer and WASM worker. The 42-case
`evidence/finish-rust-terrain-browser-06/report.json` passes with no browser
errors and matching served/local artifact hashes. It also verifies integrated
deck motion, pause/resume, fresh station generations with pause retained, and
course removal on switching back to the protocol. Source hashes and six
screenshots are preserved there. These results do not admit stepping, recovery,
performance, or a production release.

The 50-test Rust workspace passes. The two native foundation runs in
`evidence/finish-rust-terrain-foundation-05` pass A–C and the eight selected
pull probes with byte-identical reports. That runner's aggregate result remains
failed because Clippy rejected the placement of the new test module. Moving
that unchanged module to the end fixes native/WASM Clippy; the exact equality
of both implementation and test content before/after is checked in
`evidence/finish-rust-terrain-checkpoint-06.json`, with the separate successful
lint log `evidence/finish-rust-terrain-lint-06.log`. The established browser
interaction regression matrix passes another 39 checks across both backends
in `evidence/finish-rust-terrain-ui-06/report.json`.

## Migration continuation — 2026-10-02

The later [regression repair checkpoint](status.md) supersedes the eleven-failure
application count below. It repairs recovery geometry and fixture independence;
it does not qualify the remaining Rust release stages or promote production.

The continuation adds measured fall state, once-per-fall counters and input
lockout to the Rust simulation. The inherited flat-floor criteria are observed
after each integration: pelvis below 0.56 m, torso lean above 1.25 radians, or
foot-support loss beyond 0.14 s (0.36 s during a corrective swing). Falling
cancels both the current grab and future pointer commands. New grabs remain
locked until Reset; there is no implemented get-up controller. A settled,
non-foot-supported body is reported as fallen after 0.3 s below the inherited
linear/angular settling limits. These observations never set a dynamic pose.
The browser displays the state and fall count while retaining camera controls.

`setup:rust` bootstraps the pinned tools and both Cargo lockfiles. Builders use
the same tool discovery, version checks, explicit bindgen PATH and validated
workspace outputs. Playwright 1.57.0 is now a repository development dependency;
all previously locked dependencies retain their versions. Browser checks no
longer depend on a Codex installation and support an explicit browser channel.
`verify:rust:browser` builds and checks root/subpath artifacts with private
ephemeral preview servers, and records command failures without admitting a
release. `verify:rust:foundation` gives CI a scoped native result; the ordinary
`verify:rust` remains the full, currently failing release gate. Omitting an
evidence path now creates a fresh timestamped directory instead of failing
before any checks run. CI also builds both browser deployment paths; its
execution on GitHub remains unverified.

Two performance experiments remain rejected/unpromoted. Reducing internal PGS
iterations from 20 to 10 passes A but fails the −45° loaded chain at tick 111,
with 3.52477 m measured floor penetration (`migration-finish-pgs10-native`).
The unchanged solver compiled with WebAssembly SIMD records 36 WebGL2 and
417 WebGPU update deadline misses
(`migration-finish-simd-performance`). Neither change is in the default profile
or build. Solver settings, physical limits and performance thresholds remain
unchanged. These experiments do not establish broader physics acceptance.

Final continuation verification:

| Check | Result / local evidence |
| --- | --- |
| Rust formatting, native/WASM Clippy, engine Jacobian and workspace tests | Pass; 46 unit tests in `evidence/migration-finish-native-02` |
| Native gates A–C and eight selected pull probes | Two byte-identical passing runs; unchanged source; foundation command exits 0. D and E–I remain incomplete |
| Built root and Pages-subpath WebGPU/WebGL2 UI | 39 assertions per origin, including measured fall display and input lockout |
| Browser faults / actual worker lifecycle | 12 fault checks per origin and 35 worker checks pass |
| Actual-worker standing | All six mode/heading cases pass the full 480 settling + 7,200 quiet substeps; source and artifacts unchanged |
| Browser orchestration | All eight build/check commands pass in `evidence/migration-finish-browser-02` |
| Final preview artifact | All eight files in `rust/dist/app` match the tested root artifact byte for byte; `evidence/migration-finish-final-artifacts.json` |
| Retained application | Build/typecheck pass; lint has no errors and eight existing warnings. `npm test`: 364 pass, 11 fail, 7 skip (`evidence/migration-finish-npm-test.log`) |
| Full migration/release | Incomplete: physical striker/terrain, qualified stepping/recovery, actuator/input coverage, timing/soak and retained-app failures remain open. Production entrypoints are unchanged |

The 11 retained-app failures all appeared in the preceding 12-failure run.
The H77 selector failure did not reproduce; no H77 repair is claimed. The four
new JavaScript tooling tests pass. The first continuation native run exposed
a fall-observer assumption about a torso in the intentionally reduced loaded-leg
fixture; that run is preserved in `migration-finish-native`. The correction
excludes partial assemblies from whole-body classification and adds a regression
covering all three loaded-chain fixtures. Final native and browser results above
use that corrected source. All these evidence directories are ignored local
artifacts; a fresh checkout reproduces them using the documented commands.
The earlier strict stepping failures remain unresolved; that opt-in diagnostic
was not rerun in this continuation.

## Migration verification — 2026-10-02

`evidence/rust-migration-verification-02` preserves the current physical source,
43 passing native workspace tests, native/core-WASM Clippy and formatting logs,
the engine Jacobian check, and two identical native acceptance runs. Each passes
A–C and all eight selected no-fall/final-standing pull probes. D and E–I remain
incomplete, and the wrapper correctly exits 1. The source was unchanged during
the run. The later `migration-rust-lint-all.log` additionally verifies **every**
native/WASM crate and target, including browser and worker code.

`rust-migration-quiet-default-02` runs the shipped worker's ordinary default,
without a profile override: all six 2+30-second standing trials pass, with
7,200 observed quiet substeps per case and unchanged source/assets. Its maximum
foot drift is 0.009950511 m against the unchanged 0.01 m limit.
The independent bounded tangential-contact calibration passes 216 native and
216 actual-worker cases. Stress cases still fail; this does not admit general
character angular/tangential contact measurement or a friction-based controller.

Current browser checks are `rust-migration-ui-root-02` and
`rust-migration-ui-subpath-02` (37 checks each),
`rust-migration-faults-root-02` (12 checks), and `rust-migration-worker-02`
(35 lifecycle checks). These run the actual v22 artifacts. They do not qualify
the missing physical environments, hardware performance or native touchscreen.
`npm run build:rust` then rebuilt `rust/dist/app`; all eight output files are
byte-identical to the tested root artifact, recorded in
`evidence/rust-migration-final-artifacts.json`.

`rust-migration-performance-02` runs the actual browser with the frozen timing
contract. WebGL2/WebGPU respectively advance at 0.9892/0.9802 simulation seconds
per wall second, but miss 346/980 update deadlines. Worker round-trip p99 is
73.8/94.1 ms against the 20 ms limit. Neither backend records a console error;
both fail the partial timing checks. Source is unchanged, and full hardware,
input latency, process memory and soak qualification remain open.

`rust-migration-coulomb-steps-01` tests the current contact profile with only
corrective stepping enabled. All nine strict fixtures fail: failures include
post-step falls, final foot orientation, transfer and minimum completed steps.
This diagnostic stays opt-in; thresholds and the default step setting are
unchanged.

The retained application builds and typechecks, but the complete `npm test`
run (`evidence/migration-npm-test.log`) records **359 passes, 12 failures and
7 skips**. Failures cover slow-pull/reversal balance, cross-body collision,
fall/input lockout, H77 standing, a native fixture mismatch, and recovery
geometry/support. They are not skipped or reclassified as passing. JavaScript
lint now excludes generated Rust artifacts and passes with eight existing
warnings. The migration cannot be promoted on these results.

The root package exposes `build:rust`, `preview:rust`, `test:rust`, `lint:rust`
and `verify:rust`. The verification runner includes all WASM crates, rejects
unsuccessful native repeats in `completedRequestedChecks`, and separately
reports `implementedChecksPassed`; full `releaseAccepted` remains false.
Clippy uses release artifacts to avoid duplicating the large debug build cache.
All evidence paths in this section are ignored local artifacts, not files
available from a fresh clone.

## Checklist and requirement-to-test mapping

| Requirement | Actual implementation and verification | Status |
| --- | --- | --- |
| Preserve tracked/untracked work | Baseline source, manifest, tracked/index patches, fingerprints; final audit compares all 466 original source/configuration files | Preserved; no legacy entrypoint/dependency changes |
| Reproduce failures before tuning | Legacy idle at three headings, fixed-assembly diagnostic, unmodified Rapier 0.35.0 native replay | Selected runs reproduce instability; not full legacy acceptance |
| Pinned native boundaries | `rust/`: model, contracts, simulation, acceptance, worker, renderer and browser crates; Rust 1.89.0, exact crates and lockfiles | Native/WASM boundaries built; presentation has no simulation/Rapier dependency; original dependency versions retained |
| Canonical 25 segments / 72.2 kg / 1.84 m | Actual canonical f32 surfaces/poses/frames/axes/limits/exclusions; recomputed principal mass properties; indexed triangles drive rendering and ray picking | Model tests pass against independent integrals; renderer geometry/local-anchor/occlusion tests pass; all-region physical input coverage remains pending |
| Typed commands/observations | Schema 4, bounded tick queue, generation isolation, acknowledgements, partial-tick timestamps; native `Runtime` owns pause/resume/reset/init/visibility/floor/teardown | Native, Edge worker and scoped UI integration checks pass; strike/station/difficulty remain unsupported and explicitly rejected |
| Four fixed 1/240 s substeps at 60 Hz | Substep motors/contact measurements and independent shape/frame/state inspection | Implemented; observed failures halt and cancel grab |
| Contact signal before support control | Audited reporting backport plus selected-solver-point measurement and independent momentum calibration | 12 primitive + 144 canonical cases pass; no rescaling |
| A: geometry/mass/measurement/isolated joints | Once-counted mass, principal inertia, 216 isolated stress cases and deliberate observer violations | Implemented native checks pass; later controller/ownership qualification remains required |
| B: feet/loaded chains | 12 loaded-foot cases; three 11-body leg chains with upright/support evidence after settling | Implemented native checks pass; historical structural-only chain results are not standing evidence |
| C: neutral standing | 2 s settle + 30 s observation, headings 0, +pi/3, -pi/4, every integration inspected | v22 native and six-case WASM matrix pass; zero corrective steps/direct pelvis assistance; public physical modes remain unqualified |
| D: disturbances | Actual tick-indexed repository pull schedules, physical S1 grab impulses, bounded anatomical arm motor targets and support-gated angular-velocity feedback | All eight selected no-fall/final-standing probes pass; fixture minimum-step counts, swing-release and full D coverage remain incomplete |
| E–I: transfer/stepping/falls/recovery/five cycles/terrain | Original behavioral thresholds retained; opt-in contact-gated transfer, liftoff and touchdown diagnostics | Incomplete; four fixtures complete a measured step, but none of the nine strict recoverable fixtures pass. Contact-gated recovery absent |
| Dynamic ownership/actuators | Generic joints, private dynamic bodies, physical grab impulses, motor targets | Ballistic/negative/grab-energy tests pass; delivered motor impulse/vector actuator qualification pending |
| Protocol/seven stations/all difficulties | Actual room/striker/stations/course settings inventory exported; Leptos mode controls and explicit migration notices | Physical striker, terrain/object collisions, complete metrics/routes and public-mode behavior remain absent |
| Worker/wgpu/WebGPU/WebGL2/Leptos/Trunk | Dedicated WASM physics worker; Rust canonical geometry/picking/interpolation and wgpu; Leptos controls; pinned Trunk/bindgen builds | Forced WebGPU/WebGL2 Edge UI smoke passes at root and subpath; full browser physics/performance pending |
| Interaction/lifecycle | Native cancellation/hiding/reset/teardown and measured fall lockout; actual Edge worker and UI pointer, keyboard, emulated touch, pause/reset/floor/backend controls | Scoped checks and actual WebGL context-loss recovery pass; recovery, physical touchscreen, WebGPU device-loss and full interaction matrix pending |
| Performance | `rust-rework-performance-v1.json`, frozen before tuning; bounded per-tick/phase browser telemetry | Actual idle timings expose deadline/round-trip failures; full physics/input/memory/soak qualification pending |
| Build/lint/tests | Trunk root/subpath release builds; 46 workspace unit tests; native/WASM Clippy and fmt; JavaScript tooling and Edge UI/worker checks | Scoped checks pass; the retained app still has 11 regression failures; full admission and CI execution remain pending |
| CI/deployment/promotion | Non-publishing foundation and browser-build jobs; immutable evidence runner; existing deployment unchanged | Local root/subpath checks pass and final artifact matches; GitHub CI execution, Pages publication and release admission remain pending |

The complete contract remains `docs/physics-acceptance.md` and its existing
fixtures. This table maps ported checks, not acceptance of omitted fixtures or
every later actuator/ownership requirement.

## Earlier simplified-contact worker standing observation — 2026-10-02

This section preserves the pre-v22 failure and rejected numeric candidates.
The later Coulomb-contact results above supersede its statement about the
current default; the old failed artifacts are retained.

`Runtime` now owns a read-only quiet-standing observer, enabled only on an
untouched visible flat-floor trial. It records all 480 settling and 7,200 quiet
integrations, takes the reference after tick 120, and freezes the trial at tick
1,920 or the first failure. Valid commands and hiding are rejected during this
qualification. The ordinary worker protocol remains unchanged. An optional
typed profile permits a separately frozen numeric experiment without changing
the default. Failed qualification initialization frees its temporary runtime
and leaves initialization available on the same worker.

Observed failures also cancel queued future pointer intent and acknowledge its
cancellation. A test verifies the partial-substep stop, active/future grab
cancellation and frozen terminal state. A complete native trial compares the
owner against independent direct simulation and the existing `QuietMetrics`;
the initial, settled, final and measured values match exactly, including with
the optional drift trace. The affected crates pass 37 release tests. The
acceptance CLI now forwards its supplied profile to the eight pull scenarios;
previously its normal execution used the default profile for those pulls.

The full actual-worker matrix is recorded in
`evidence/rust-worker-quiet-browser-03/report.json`. Both mode tags pass at 0 and
-45 degrees. Both fail at +60 degrees at tick 1,765, substep 1:
`leftForefoot` drift is 0.010007268749177456 m against the unchanged f32 0.01 m
limit. Native +60 completes with a 0.009430001 m maximum. Packed poses match
every physical reply, rejected valid Reset commands return `Unsupported`, and
terminal measurements stay frozen. Source and fetched artifact hashes remain
unchanged. These mode tags exercise a flat floor; they do not qualify the
missing public environments or rendering coupled to the full standing window.

Four frozen opt-in profiles remain rejected. Stiffness 25 passes the targeted
WASM +60 trial but fails native +60. Stiffness 30 fails that trial on both
architectures and adds a native -45 angular-speed failure. Damping 50 passes
targeted WASM +60 but fails native +60. COM displacement gain 6 also fails
native +60 and introduces a native 0-degree angular-speed failure. Plans,
profiles, results and rejection decisions are retained under
`evidence/rust-worker-quiet-v18-*` through `rust-worker-quiet-v21-*`. None changes
the default, thresholds, geometry, iteration counts or motor ceilings.

The optional `quiet-drift` qualification adds bounded once-per-second body,
COM, leg-coordinate, pre-solve motor-target and selected-contact samples, plus
the failing quiet substep. Full traced native/WASM physical observations and
metrics match their original untraced reports exactly. Contact normal impulse
reporting retains its existing calibration. **Tangential reporting is not
admitted:** Rapier's default `Simplified` friction solver writes normal and
warm-start data but does not populate per-point `tangent_impulse`. Raw zeros
cannot establish zero friction or unsaturated contacts. The trace labels that
limitation; causal contact-slip diagnosis and tangential momentum calibration
remain required before choosing a friction or stance-control repair.

The final artifacts pass 35 worker lifecycle checks, paired profiling/plain and
previous-default comparisons at three headings over 80 ticks, 37 UI checks at
each root/subpath origin, and 12 crash/hang/bootstrap checks across forced
WebGL2/WebGPU. Native/WASM Clippy, formatting and focused JavaScript lint pass.
The source, built artifacts, passing and failing reports are frozen in
`evidence/rust-worker-quiet-checkpoint-01/manifest.json`. This closes the missing
full-window observation coverage and preserves the browser failure; it does
not close Gate C for WASM, D–I, public physical modes, hardware/performance,
CI/deployment or migration acceptance.

## Earlier migration continuation — 2026-10-01

`evidence/rust-migration-frozen-01` freezes profile
`rust-physics-v17-supported-angular-feedback`, source fingerprint
`e888444ce1bfc263e36f2be7eb410b84457271589765e4da885178b4649d7787`
and executable `4c4712a61346166039e60f6a2087c6b0430f3cc485992a8cddb86d6a9a27d3ee`.
All nine requested executions return zero, both native result directories match
byte for byte, and the captured source remains unchanged. The evidence wrapper
returns 1 because full release admission remains incomplete.

The repair adds measured pelvis angular velocity to supported ankle feedback,
with tilt/COM/COM-velocity/angular-velocity gains 2/3/4/2. Motor ceilings, 760 N m
aggregate request cap, grab force/torque/power caps, input schedules and all
physical thresholds retain their previous values. Bodies and poses remain owned
by Rapier. There are zero direct pelvis forces or torques.

All eight selected pull probes now complete with no integrity or no-fall failure
and bilateral standing evidence at every substep of their final second. This
does **not** satisfy their inherited minimum corrective-step counts: the default
still has no qualified step controller, and Gate D remains incomplete.
Quiet-standing maxima over three headings are 0.004225 m pelvis drift,
0.009430 m foot drift, 0.015046 m/s linear speed and 0.238770 rad/s angular speed,
all within the unchanged 0.03/0.01/0.1/0.5 limits.

Schema 4's native lifecycle owner processes commands while paused without
integrating, cancels active and queued grabs on suspension/hiding, requires a
fresh press, isolates reset generations, clears floor support immediately,
limits advance requests to four ticks, and rejects execution after shutdown.
Previously accepted future commands apply once at their requested tick, even
when a later-received command has an earlier tick. Strike, station and difficulty
commands return `Unsupported`; they cannot acknowledge nonexistent behavior.
These are native tests, not browser lifecycle qualification.

### Dedicated worker follow-up

`lh-worker` now compiles to an executable browser WASM module. Its JavaScript
adapter owns one simulation and exactly three 1,300-byte transferable pose
buffers. It rejects an advance before integration when no buffer is available,
limits requests to four ticks, and explicitly frees/closes on shutdown.
Command packets enqueue atomically: a bad command cannot leave earlier commands
from the rejected packet queued. Six native lifecycle tests pass, with native
and worker-target WASM Clippy and focused JavaScript lint returning zero.

`evidence/rust-migration-worker-browser-05/report.json` passes 35 recorded checks
in a real Microsoft Edge 154.0.4258.48 dedicated worker. Checks cover finite
25-body transfers, physical grab commands, cancellation of queued and active
pointer intent, fresh presses, pause/hide freezes, generation isolation,
unsupported strikes, advance bounds, buffer recycling and explicit teardown.
Browser-fetched asset hashes equal the local captured hashes and the listed
source files remain unchanged. Worker WASM SHA-256:
`fe5863d75a3a987b15cca687b5a232359c0490c46b15f21ff1e1813226b50954`.
The first worker smoke failed stale-generation rejection priority; the native
owner now checks schema/generation before suspension/tick checks. Its follow-up
tests and browser attempts 02–05 pass. The final follow-up workspace run passes
all 24 native unit tests (`rust-migration-worker-final-unit-02.log`). This later worker source differs from the
earlier native frozen source; it does not refresh full native/browser acceptance.

Build with `node scripts/build-rust-worker.mjs`, using Rust 1.89.0 and
wasm-bindgen CLI 0.2.108. The CLI may be supplied with `WASM_BINDGEN_BIN` or in the
workspace tool directory. Start the worker test origin with
`node scripts/serve-rust-preview.mjs rust/dist/worker 5184`, configure Playwright
as in [browser verification](browser-verification.md), then run
`node scripts/verify-rust-worker.mjs <fresh-output-directory> http://127.0.0.1:5184`.
The root is a test harness, not the replacement UI. No rendering, browser physics
matrix, timing/soak or release admission follows from this smoke.

### Browser application follow-up

`lh-renderer` uploads the canonical indexed triangles for all 25 segments and
the same flat-floor dimensions as the native simulation. It validates each
transferred pose against the immutable JSON observation, interpolates only
presentation transforms, and uses those displayed triangles for surface-local
grab anchors. Region filtering cannot pick through a nearer surface. The
renderer and `lh-browser` dependency tree contain no simulation or Rapier crate.
Leptos owns the responsive controls; the JavaScript adapter handles browser
events, bounded command transport and buffer recycling. One dedicated worker
continues to own all physical bodies and the fixed clock.

`evidence/rust-browser-ui-06/report.json` passes 37 recorded checks in Edge
154.0.4258.48 across forced WebGL2 and WebGPU. Both tagged modes cover pause,
exact four-tick resume, reset generations, native floor removal and gravity,
60-degree initialization, camera/quality redraw while paused, keyboard input,
graphics-backend changes, bounded advances, narrow layout and teardown. A
trusted pointer picks head triangle 117 and causes 0.034917 m additional head
motion versus the same undisturbed clock. Trusted Chromium touch emulation
tests single-finger picking and cancellation before two-finger pinch/orbit;
the resulting native poses exactly match the undisturbed control. Actual
`WEBGL_lose_context` loss/retry preserves paused physical state and its worker.
Keyboard camera movement also cancels an active grab and prevents page scrolling.

`evidence/rust-browser-subpath-ui-04/report.json` repeats those 37 checks on a
separate build served from `/LaboratoireHumain/`. Every browser-fetched asset
matches its captured local SHA-256, both reports confirm unchanged source and
artifacts, and neither records console errors or failed HTTP responses. Desktop
and narrow-layout PNGs were visually inspected. These are local static-artifact
checks, not a GitHub Pages deployment or full final-artifact qualification.

`rust-migration-browser-default-probe-01` replays the eight selected native pull
diagnostics after the frontend dependencies, optional region metadata and shared
floor constants were integrated. All eight retain passing physical checks and
their complete final snapshots and command schedules exactly match the earlier
`rust-migration-frozen-01` controls. The comparison is recorded in
`evidence/rust-browser-default-compatibility.json`. The diagnostic wrapper still
returns 1 because this scope does not admit Gate D or a release.

The first WebGPU trial failed WGSL derivative-uniformity validation. Derivatives
now execute outside the floor-only branch, and graphics initialization awaits
its validation scope. The first UI trial found the ground checkbox reverting
before native acknowledgement; requested and confirmed ground states are now
separate. Failed attempts remain in `rust-browser-first-webgpu-01` and
`rust-browser-ui-01`. The successful release workspace run passes 31 tests;
native/WASM Clippy, fmt and focused JavaScript lint return zero. The initial
unoptimized workspace test run was stopped after an expensive calibration
fixture; its partial log is retained, and the complete release run passes.
WebGPU's linear canvas format and WebGL's sRGB attachment also produced different
colors. Both now use an sRGB render-attachment view, with matching fixed-pose
screenshots inspected on both backends. Final root build 09 and subpath build 03 include keyboard
grab cancellation. Their reports capture 257 source files, including canonical
geometry/scenario JSON and the modified vendored engine. The physical worker
and default controller are unchanged.

The worker rebuild at that earlier checkpoint also passes all 35 lifecycle checks in
`evidence/rust-migration-worker-browser-06/report.json`, including matching
browser/local asset hashes and unchanged captured source. Worker WASM SHA-256:
`55293f879037bd9eaf2f6800de7b7d8fefc72a41b55945ce076ac854548c4ade`.

Reproduce the scoped browser checks with the pinned workspace Trunk 0.21.14
and wasm-bindgen 0.2.108 tools. For external tools, provide `TRUNK_BIN` and place
the exact wasm-bindgen version on `PATH`:

```powershell
node scripts/build-rust-browser.mjs
node scripts/serve-rust-preview.mjs rust/dist/app 5185
node scripts/verify-rust-browser.mjs evidence/rust-browser-reproduction-01 http://127.0.0.1:5185/

node scripts/build-rust-browser.mjs rust/dist/app-subpath /LaboratoireHumain/
node scripts/serve-rust-preview.mjs rust/dist/app-subpath 5186 /LaboratoireHumain/
node scripts/verify-rust-browser.mjs evidence/rust-browser-subpath-reproduction-01 http://127.0.0.1:5186/LaboratoireHumain/ rust/dist/app-subpath
```

Run each preview server in a separate terminal and configure Playwright as in
[browser verification](browser-verification.md). The actual browser trials use
headless Edge with `--enable-unsafe-webgpu`; they do not establish hardware GPU,
physical touchscreen, WebGPU device-loss, full physics, latency, memory or soak
admission. Cold timing samples are slow and remain unqualified. The browser
marks the physical striker and terrain stations as still being migrated, and
the production entrypoint remains on the original application.

The sampled timing diagnostic `evidence/rust-browser-timing-probe-01/report.json`
uses the earlier root build 07, before the sRGB-view correction. After a 2-second
wall warmup, its 30-second windows record simulation/wall ratios 1.0095 for
WebGL2 and 0.9026 for WebGPU. WebGL2 includes backlog catch-up; WebGPU accumulates
3.0447 seconds of backlog. Maximum sampled worker batches are 123.1/107.0 ms.
Both have no recorded integrity failure or console error, but this sampled
diagnostic omits individual tick/controller timings, adapter identity, latency,
memory and soak. It cannot admit the frozen zero-miss performance requirements.

### Per-tick timing and worker-fault follow-up

The optional native trace now measures all four controller, solver, contact,
integrity and observer phases of each completed tick. The worker reports its
output/JSON overhead separately; the browser keeps bounded packet/frame telemetry
with explicit overflow counts, round-trip times and WASM linear-memory sizes.
The timing probe charges shared worker output cost to the last tick of its batch.
It measures IPC separately and does not equate a four-tick batch with one update.
Native tests and paired actual WASM workers show identical physical replies
after removing only the two optional timing fields; packed pose bits also match
at headings 0, +60 and −45 degrees in the 80-tick grab/pause/resume/reset probes.

The first profiled browser run, `rust-profiling-browser-01`, identifies the solver
as the dominant cost. Integrity checks repeatedly transformed each convex shape
for pairwise bounding-box rejection. The observer now caches each collider's
bounds once per substep, retaining every eligible pair, exact penetration query,
threshold and error priority. A contact-query oracle test covers overlaps,
separation and rotated shapes. Integrity time drops from about 1.33/1.50 ms to
0.20/0.24 ms per tick in WebGL2/WebGPU in `rust-profiling-browser-02`. Both still
miss update deadlines. The final native eight-pull comparison in
`rust-profiling-default-compatibility-02.json` preserves every final snapshot and
replayed command exactly against `rust-migration-frozen-01`; full D remains open.

An isolated `rust/dist/app-simd` build uses
`CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUSTFLAGS=-C target-feature=+simd128`.
It retains the same physics configuration and iteration counts. The 30-second
idle windows in `rust-profiling-browser-simd-01` record:

| Backend | Simulation/wall ratio | Full charged tick maximum | Update deadline misses | Mean solver time |
| --- | ---: | ---: | ---: | ---: |
| WebGL2 | 0.99979 | 21.0 ms | 17 / 1,805 | 12.08 ms |
| WebGPU | 1.00001 | 31.7 ms | 179 / 1,807 | 14.12 ms |

The exact sample counts are recorded in the report. These are idle diagnostics,
not release admission. WebGL identifies the local NVIDIA GTX 1050 Ti Max-Q via
ANGLE; WebGPU's adapter fields are redacted, and the report retains the browser's
system GPU metadata. Actual displayed snapshot age, input-visible latency, OS
process memory, the 120-second soak, other physics/mode fixtures and independent
hardware admission remain unqualified. SIMD remains experimental.

Premature physical controls now stay disabled and keyboard actions cannot alter
desired state while the worker is loading. A crash, undecodable packet or request
timeout rejects pending advances, cancels input and freezes the trial. Reset
creates a fresh worker generation with the selected heading, mode and ground
setting; late events from the previous worker are ignored. The actual browser
fault harness passes delayed-bootstrap, crash, timeout and pending-teardown
checks in both backends. The simulator provides no running-body pose transfer.

The final default root/subpath builds also retain a graphics-loss message and
visible retry button when physics is reset while the view is lost. A subpath
trial exposed an invalid-viewport halt during transient layout collapse. The
bridge now retains its last valid surface, cancels any active grab, skips drawing
while the canvas is zero-sized and resumes on the next drawable frame. The held
press cannot reactivate its grab. `rust-profiling-ui-root-03` and
`rust-profiling-ui-subpath-03` pass 37 checks each; their source/artifact hashes
remain unchanged. `rust-profiling-faults-root-03` and
`rust-profiling-faults-subpath-03` pass 12 actual browser recovery checks each.
The default worker still passes 35 lifecycle checks and the three paired timing
comparisons; all three shipped worker assets are unchanged after the view fixes.
The final idle timing run, `rust-profiling-browser-default-03`, records
simulation/wall ratios 0.99995/1.00003, charged update maxima 26.8/30.6 ms and
75/625 deadline misses in WebGL2/WebGPU. Round-trip p99 is 53.1/73.3 ms, above
the frozen 20 ms requirement. Neither trial records a physics or console error,
and both confirm unchanged source. These failures remain explicit.

`evidence/rust-profiling-default-checkpoint-01` freezes the final default root
and subpath artifacts, their source and the matching 74 UI/24 recovery checks,
35 worker checks, paired timing comparison, 36 native tests, native/WASM lint
logs, native eight-pull comparison and the final raw timing data. Goal completion
and release acceptance are false in the manifest. Production remains unchanged.

`evidence/rust-profiling-simd-checkpoint-01` freezes 264 source files, the exact
SIMD artifact, current 37 UI/35 worker checks, paired-worker and eight fault
checks, timing data and the native comparison. Its manifest records all copied
hashes. The 36-test native release suite and native/WASM Clippy checks pass.
An initial build ran out of disk space; the recorded cleanup removed only
generated unoptimized Cargo caches. Frozen sources and evidence were retained.

### SIMD8 and angular-motor experiments

The worker-only SIMD8 candidate uses `rapier3d/simd8` and the same
`-C target-feature=+simd128` compiler flag; its renderer retains the default
build. `rust-simd8-native-01` passes A–C, preserves the complete quiet-standing
report byte for byte, and retains the current eight-pull report exactly.
`rust-simd8-checkpoint-01` preserves its source, artifact, flags and measurements.
Its 30-second idle windows still miss the frozen zero-miss update requirement.

The engine normally sends 3D angular motors through scalar assembly/solving.
The new, disabled-by-default `experimental-wide-3d-motors` feature retains that
scalar assembly and packs its rows into body-disjoint vector groups. Row
signatures include active motor axes and models. Coupled and linear motors
retain scalar fallback; fixed arrays add no heap ownership or unsafe operations.
The experimental small-scene color threshold is two, while the default remains
64. The first full native run used the original threshold and consequently did
not exercise this path on the canonical body. Later small-scene native runs
also preserve A–C and the eight current pull reports exactly.

The first browser candidate repeated scalar assembly for padded copies of lane
zero and regressed. Reusing fixed scratch and assembling those copies once
improves the recorded follow-up, but does not beat the earlier SIMD candidates:

| Worker candidate / backend | Simulation/wall ratio | Mean solver | Charged tick maximum | Deadline misses | Round-trip p99 |
| --- | ---: | ---: | ---: | ---: | ---: |
| SIMD8 / WebGL2 | 0.99930 | 11.24 ms | 21.1 ms | 9 / 1,807 | 24.5 ms |
| SIMD8 / WebGPU | 0.99979 | 12.73 ms | 25.0 ms | 45 / 1,805 | 26.9 ms |
| Padded-assembly fix / WebGL2 | 1.00058 | 13.81 ms | 20.4 ms | 42 / 1,802 | 29.8 ms |
| Padded-assembly fix / WebGPU | 0.99892 | 15.82 ms | 32.7 ms | 586 / 1,808 | 93.7 ms |

These are separate local idle windows, not complete performance admission.
Reports `rust-simd8-performance-01` and `rust-wide-motors-performance-01/02`
retain the raw data, unchanged-source receipts and explicit failing timing
checks. The latter probe now fingerprints the relevant vendored engine files.
No thresholds, motor caps, solver iteration counts or default build flags changed.

`rust-wide-motors-native-03/comparison.json` confirms that removing duplicate
work preserves every complete native report from the preceding candidate.
The affected-crate default suite passes 33 tests, and native/WASM application
Clippy checks pass. Vendored tests confirm exact scalar/vector row solves,
warmstarts, motor/lock/limit impulse bounds, row signatures, real small-scene
vector use, recycled builders and absence of drop ownership. Four test-only
conversion warnings were repaired while retaining their numeric values.

The direct WASM comparison found a small pose difference after four ticks
against the default worker compiled without SIMD flags. With matching flags,
`rust-wide-motors-worker-profiling-04` matches every normalized physics reply
and packed pose bit at three headings in the 80-tick grab/lifecycle probe; the
19 shared model/contracts/simulation/worker source files match that scalar
reference. This narrower evidence does not establish full browser standing,
disturbance or actuator acceptance. Comparison failures now record the first
different field without constructing an enormous assertion diff. The aborted
PowerShell run and the failing default-flag comparison remain preserved.

The optimized candidate also passes 35 worker lifecycle checks and 37 actual
Edge UI checks across forced WebGL2/WebGPU, with matching fetched artifact
hashes. `rust/vendor/wide-motors-provenance.json` records the patch and evidence
chain. Both motor candidates remain experimental and unpromoted. Strict D still
passes zero of nine recoverable step fixtures; E–I, public physical modes,
complete browser/hardware qualification, CI and deployment remain unfinished.

Run the new scoped probes with a fresh output directory:

```powershell
node scripts/verify-rust-browser-faults.mjs <fresh-output> http://127.0.0.1:5185/
node scripts/verify-rust-worker-profiling.mjs <fresh-output> http://127.0.0.1:5184/
node scripts/probe-rust-browser-performance.mjs <fresh-output> http://127.0.0.1:5185/
```

### Corrective-step follow-up

The optional step candidate stays disabled. Transfer now waits for the measured
capture point to lie inside the retained sole's convex hull and for retained
load to persist for 0.05 s. A completed step requires measured liftoff (at least
0.02 m center lift, less than 3 N foot load, and both convex foot shapes clear of
the floor by more than the two contact skins plus 0.002 m, for 0.05 s), at least 0.04 m actual
horizontal travel, and loaded touchdown for 0.10 s. A continuously loaded slide
cannot increment the counter. Floor loss cancels the plan. All inverse poses
remain temporary calculations; anatomical motors own actuation.

The swing candidate reserves more of the existing 760 Nm sum of native axis
ceilings for the retained leg. It retains nonzero ceilings for every active
axis and the previous stiffness/damping. This is not delivered vector-torque
qualification. Default standing uses the existing uniform allocation.

`--require-steps` now checks the exported fixture's unchanged minimum completed
steps and includes `release-during-corrective-step`. That fixture releases only
after measured liftoff in the swing phase. Reports preserve per-substep phase
events, actual commands, required counts, missed behavior and physical failures.
The ordinary pull diagnostic retains its narrower scope.

`evidence/rust-migration-step-15/16` captures two executions of the current
opt-in source/profile and all nine recoverable fixtures. The results match byte
for byte, with source unchanged. None pass the strict behavioral diagnostic.
The forward hand pull completes one step and subsequently falls; the backward
pull falls without verified full-foot liftoff. Both foot
pulls and the held target complete one step but fail the unchanged final
upright-foot threshold. Lateral hand pulls, reversal and swing-release do not
complete the required steps. No thresholds, fixture inputs or motor hard limits
were relaxed, and no release admission follows from a completed step alone.

Five focused stepping tests pass, including canonical inverse geometry,
tapered support, rejection of loaded slides and toe contact, floor-loss
cancellation, and native motor ceiling/gain checks. Current-source workspace
tests pass all 28 cases, with native/WASM Clippy and formatting returning zero
(`rust-migration-step-*-03.log`).

The disabled-step negative control (`rust-migration-step-negative-02`) completes
zero steps and correctly fails all nine strict step requirements. Its eight
ordinary pull final snapshots equal the prior passing native frozen snapshots
exactly. Source fingerprint:
`75eac1e26a25e7194af4684e711a407324935829475083b8afd8faec286d1bb4`.
Executable SHA-256:
`a293ad614be2417fbd005c25935bf93150c256cb54212bca8f5dba927f91350d`.
The combined record is `evidence/rust-migration-step-sole-summary.json`.

Earlier backward-pull attempts `rust-migration-step-01/02` failed at tick 173
with zero completed steps. Trials 13/14 counted five center-lift events; the
subsequent whole-foot clearance check rejects the backward-pull event because
the toe did not remain clear. Trial 04's transfer adjustment fell later; trial 12's
post-step hip balance worsened the physical failures and was removed. Demand
budgets v14/v15 and earlier hip feedback also failed. Failed sources, profiles,
commands and results remain under `evidence/`.

Reproduce the stricter partial diagnostic (its wrapper always exits nonzero):

```sh
node scripts/run-rust-disturbance-probe.mjs <fresh-output-directory> --profile evidence/rust-migration-profiles/step-v18.json --require-steps
```

The unchanged legacy application builds and typechecks; lint returns zero with
seven warnings. Its 378-test run has 359 passes, 12 failures and seven skipped
local-evidence cases. Five of those failures also reproduce in the focused JUnit
record `evidence/rust-migration-legacy-focused.xml`. Legacy balance, recovery and
saved-stream failures are not resolved by native Rust success. The browser
replacement, full physics matrix and release qualification remain required.

## Historical v13 evidence

## Source and evidence

Baseline revision: `8c32e8f5b0dd7e41a6b4950e5497b4bebe51921a`, including 33 dirty
tracked files and original untracked source. Baseline fingerprint:
`89e4c61e3440612f3e4764ddeb31e4f83e4bc828082f97b443a680e4fa3f1d55`.
Rollback source: `evidence/rust-rework-20261001/baseline/source` (466 files), with
manifest, configuration/dependency/toolchain hashes and separate working-tree/
index patches. Evidence links/caches/external H78–H80 junctions were inventoried
without recursive archiving.

Final implemented sequence: `evidence/rust-rework-20261001/frozen-07`:

- Source: `7a7a3c3e764fed29ee1e073f2f90fc764bd43059d846b70d6a8c1a3a42609008`.
- Profile: `rust-physics-v13-arm-motor-targets`; profile JSON SHA-256:
  `c5f68c4b3bad2431d338ef1dcbeadc19602f64015ae49bc66194120af792ac13`.
- Executable: `3890ca487ae5cc2f05d3ae320d3be9039cdf8ce54c0bc32da3310bae614d5e22`.
- All eight result files match across two executions, captured source unchanged.
  Both native executions return 1 because D fails; release acceptance is false.

`arm-reaching-01` captures the arm-repair source, executable, exact commands,
profile, replayed inputs and first failures. It is a **partial disturbance
diagnostic**, not promotion. `frozen-07` subsequently runs the complete implemented
sequence twice on the arm-target revision, with identical failures. The earlier
`frozen-06` completes A–C with five D failures; arm targets reduce these to three
without changing inputs, force caps, scenarios or behavioral thresholds.

Quiet-standing maxima across headings in `frozen-07` (unchanged from `frozen-06`):

| Measurement | Observed maximum | Unchanged ceiling |
| --- | ---: | ---: |
| Pelvis drift | 0.004342 m | 0.03 m |
| Hindfoot/forefoot drift | 0.008966 m | 0.01 m |
| Segment linear speed | 0.014404 m/s | 0.1 m/s |
| Segment angular speed | 0.178619 rad/s | 0.5 rad/s |

These native substep values do not establish browser repeatability, real-time
performance or full acceptance. Both final native runs fail:

| Existing recoverable fixture | Tick / substep | Pelvis height | Lower bound |
| --- | --- | ---: | ---: |
| `slow-hand-backward` | 264 / 2 | 0.555879 m | 0.56 m |
| `slow-hand-right-heading` | 227 / 3 | 0.555648 m | 0.56 m |
| `direction-reversal` | 216 / 1 | 0.557429 m | 0.56 m |

Forward, left, held-target and two foot probes pass the selected native suite. The
controller lacks deliberate weight transfer and capture-point stepping needed
for the remaining pulls. Release-during-actual-swing, overpower, cross-body,
recovery/lockout/cycle and terrain scenarios remain unqualified. Foot probes use
the physical raw-target path; legacy collision-aware reachable foot routing has
not been ported.

## Contact and joint repairs

The unmodified 0.35.0 calibration workspace, cache package and native replay
remain available. Registry archive SHA-256:
`c56bf7b5596ef716abcf578636db53b3780162043fada35717d46f79d4022fa4`.
The reporting [upstream correction](https://github.com/dimforge/rapier/commit/37eeac7dec6addd6a94a237b4874ca0b676d70da)
is retained as an audited patch with before/after hashes under `rust/vendor/`.
The backport alone still reports roughly 407–467 N on real feet under a declared
354.141 N load; `frozen-02` preserves that failed calibration.

Rapier also retains impulses on cached manifold points not selected for the
current solver; the pair-wide sum includes them. The independent measurement
path uses current solver manifolds/contact point indices and sums only selected
points. It validates indices and finite nonnegative impulses, without scaling
or changing solver/warm-start state. Reports retain old cached sums beside the
validated measurement. Rest/changing/unloading/contact-loss cases cover all four
foot surfaces, three headings and warm-start coefficients 0, 0.5, 1. Momentum
balance residual must stay below 0.002 N s; loaded-foot peak is about 0.000101 N s.
Observation force is the four-substep impulse sum divided by four actual 1/240 s
intervals. Slope/terrain contact normals remain unqualified.

Contact-loss testing exposed the default 400 m/s numerical solver velocity cap,
which alters momentum after prolonged accelerated free flight without a physical
force. Profile v5 explicitly disables it with the documented `Real::MAX`
setting. Duration, acceleration, timestep, speed observation and calibration
threshold remain unchanged; this decision is fingerprinted.

Measurement schema 2 fixes old archive labels `*nm_s` to `*n_s` for linear
contact impulses (N s); values/calculations are unchanged. Delivered angular
motor impulse (N m s) remains unavailable. Some earlier reports said
`partial-gates-pass` despite D failing; their gate fields/failures are
authoritative. Current report schema 2 marks a failed D report `fail`.

The experimental angular patch derives uncoupled 3D motor/limit rows from
`2 atan2(q_i,q_w)`, with matching motor feedback. Locked/coupled rows and
integration retain upstream code. Finite differences against independent world
rotation perturbations and 216 isolated stresses pass. This does not qualify
the complete modified engine. `rust/vendor/README.md` and the five-file audited
diff describe the dependency changes.

Negative tests seed floor/self penetration, limits, ownership conversion and
non-finite state. Upright/support checks reject lying chains. Floor-disabled
ballistic tests check mass, gravity/momentum/horizontal invariants at all three
headings. Kinematics tests compare temporary FK against actual canonical world
transforms, both arms/headings, anatomical limits and unchanged live state. Grab
power tests independently measure body kinetic-energy change. Lost grab-body
ownership is tested to halt/cancel before integration.

## Decisions and pending requirements

- One agent; preserve model/reasoning. Isolated Cargo workspace; original bytes
  untouched. Rust 1.89.0/edition 2024 and exact/locked crates. This host uses the
  GNU toolchain because MSVC SDK is absent. Dynamics remain f32; f64 geometry
  integration occurs only at initialization, then builds f32 principal moments.
- Explicit prescribed collider mass properties once; no additional mass double
  counting. Exclusions are an undirected pair union. Generic impulse joints
  retain declared axes and asymmetric limits; no hybrid multibody conversion.
- Contact-gated tilt/COM feedback and virtual arm reaching change native motor
  targets only. Temporary kinematic poses never write live bodies. No direct
  pelvis assistance, body-type conversion, teleportation or hidden reset.
- Individual generalized motor ceilings are uniformly scaled into a 760 N m
  aggregate request cap. Requested effort is labelled as requested; delivered
  impulse is `None`. Vector actuator limits/readback and full diagnostics still
  need independent qualification. A ceiling alone is not delivered-effort proof.
- Physical grabbing retains 180 N / 12 N m / 36 W and original target speed/
  acceleration limits. Commands apply before substep 0; updates occur each
  substep. Native and scoped UI pause/reset/hide cancellation and fresh presses
  are implemented and tested. The continuation also implements measured
  fall lockout; automatic recovery and the full interaction matrix remain pending.
- Observation schema 4 separates integrated time and last complete contact
  timestamp. Interrupted ticks cannot claim unperformed integration. Unmeasured
  update/controller timing is `None`, not zero. Replacement consumers operate
  in the isolated Rust preview; the production browser entrypoint is unchanged.
- Performance is frozen in the JSON on the named i7-8750H/Windows/Edge host.
  8 ms controller and 16.67 ms complete-update requirements remain applicable
  references, with zero allowed misses; frame/input/round-trip/age/memory/soak
  limits are retained. No actual replacement browser path is qualified.
- Required skipped/unexecuted stages stay incomplete. The evidence runner
  blocks release; new CI preserves failures and publishes nothing. Existing
  legacy deployment remains operational. The new job pins Ubuntu 24.04, Node
  22.13.0, Rust 1.89.0 and action commits verified from their primary releases:
  [checkout 7.0.0](https://github.com/actions/checkout/releases/tag/v7.0.0),
  [setup-node 7.0.0](https://github.com/actions/setup-node/releases/tag/v7.0.0),
  [upload-artifact 4.6.2](https://github.com/actions/upload-artifact/releases/tag/v4.6.2).

Source-specific profile history: v1 contact-only fails isolated constraints;
v3 explicit-mass/load fixtures fails cached contact measurement; v4 selects
solver points; v5 disables numerical speed cap; v6–v8 refine posture/feedback
frames; v9 adds exact grab replay; v10 lateral feedback; v11 unavailable timing/
quaternion-sign invariance; v12 impulse unit/report labels; v13 virtual arm motor
targets. Frozen directories contain full actual profiles. Exploratory
`solver-contact-*`, `posture-*`, `arm-reaching-*` records are partial and cannot
be combined across different source/configuration into release evidence.

## Reproduction and rollback

From repository root with Node 22.13+ and rustup available:

```powershell
npm ci
npm run setup:rust
node scripts/setup-rust.mjs --check
npm run verify:rust:foundation
npm run verify:rust:browser
```

Setup installs Rust 1.89.0, the WASM target, Clippy/rustfmt, Trunk 0.21.14 and
wasm-bindgen CLI 0.2.108, and fetches both locked Rust dependency graphs.
It reuses correctly versioned tools from PATH, explicit `TRUNK_BIN` /
`WASM_BINDGEN_BIN` overrides or the workspace cache. Browser tests require
Edge on Windows or `npx playwright install chromium` elsewhere; set
`RUST_BROWSER_CHANNEL=chromium` to select that browser on Windows too.
The foundation/browser commands are partial qualification only.

For the release gate, use a fresh output directory or omit it to generate one.
Exit 1 is expected while physics/release acceptance fails or remains incomplete:

```powershell
node scripts/run-rust-gates.mjs evidence/rust-rework-reproduction-01
```

The wrapper snapshots source, exact commands/tools, all implemented checks and
two runs each of the native foundation, 21 terrain-contact cases and three
striker cases; it verifies unchanged source and every result file. It uses the
existing calibration cache; otherwise fetch both lockfiles as in the new CI.
The combined browser command also includes the terrain and striker checks
against the exact root artifact, alongside root/subpath UI and fault checks.
These remain scoped integration checks, not traversal or recovery admission.
Native diagnostic flags and options are validated before creating evidence;
unknown, duplicated, missing or inapplicable options exit with status 2.
A partial disturbance reproduction with source/log capture is:

```powershell
node scripts/run-rust-disturbance-probe.mjs evidence/rust-disturbance-reproduction-01
```

Representative legacy reproduction, selected scenario only:

```powershell
$env:PHYSICS_SCENARIO_PATTERN='^idle-30-seconds$'
$env:PHYSICS_OUTPUT_PREFIX='evidence/legacy-idle-reproduction'
$env:PHYSICS_PROGRESS='1'
node --import tsx scripts/run-physics-harness.ts
```

Restore baseline files into a **fresh** checkout at the recorded revision, honor
deletion records and verify manifest hashes. Do not overwrite a live working
tree. The original application already remains in place. `final-audit` applies
to the historical `frozen-02` source only; `final-audit-02` separately records
original-source preservation, final frozen-source equality, dependency diffs
and ancillary source/document hashes.
