# Rust migration implementation checkpoint — 2026-10-03

Release status: blocked. No accepted artifact, GitHub publication, production
entrypoint change, or TypeScript retirement has occurred. The default Rust
profile still disables corrective stepping and recovery. Historical source,
failed evidence and frozen contracts remain intact.

2026-10-05 later-transfer continuation: a transfer after verified landing now
blends the existing motor allocation toward persistent measured sole-load
shares over its 0.6-second clock. Legs receive priority
`0.55 + 0.45 * load_share`, upper body 0.12, normalized within the unchanged
760 N m aggregate ceiling. Native gains and per-axis ceilings remain unchanged.
First transfers receive no new reserve, unsupported state clears it, and a
committed swing immediately uses its original priority.

Right-foot-heading admits a second swing at tick 180 with the original measured
readiness proof, verifies liftoff at 229 and falls at 260 without a second
landing. Liftoff follows the nominal trajectory endpoint and does not qualify
stable repeated stepping. Held-target stays upright for the complete 601-tick
fixture. The three previously upright nonlanding fixtures and direction reversal
are unchanged. Strict stepping remains 1/9; physical upright cases remain 4/9.
The other one-step failures are forward 235, backward 214 and left-foot 228.

All 85 workspace tests, formatting and native/WASM Clippy pass. Guards and native
installation are covered by persistent-load/phase and motor-allocation tests.
Two fresh dedicated native reports repeat identically and match the retained
probe. Verification sources are fixed and match the tested snapshots. The
authoritative receipt is
`evidence/migration-transfer-stance-2026-10-05/final-qualified/execution.json`;
it binds same-source passing code checks from `final-verified/` to the fresh
native build/replays. Earlier compile-failure and binary-reuse rejection receipts
remain explicit. Direct stance/reference coordination trials, alone and with
the reserve, regressed held-target and were removed; their reports have verified
lossless gzip archives. Next work is requested swing-path reach and delayed
physical lift, followed by transfer/stance dynamics. Full migration qualification
and production stepping/recovery remain unchanged and release admission is false.

2026-10-05 post-landing continuation: the stance blend now differentiates its
complete bounded anatomical trajectory, including the changing virtual root.
Native position/rate ordering remains consistent with subsequent swing
overrides. Verified landing keeps the previous support leg's allocation and
fades it to ordinary allocation over 0.6 seconds, removing the immediate hip
capacity drop. The budget query is separate from standing-feedback selection;
unsupported updates clear it, and a new committed swing overrides it. Native
gains, anatomical ceilings, the 760 N m aggregate budget, full-swing endpoint
weights and all physical step criteria remain unchanged.

Held-target now passes the strict fixture: landing at tick 201, one counted
step, no fall through tick 601, minimum pelvis height 0.942 m. Strict stepping
is 1/9 versus 0/9; four fixtures remain physically upright versus three. All
five previously stepping fixtures still land, but the other four still fall
(forward 234, backward 213, left-foot 221, right-foot-heading 255). The three
prior upright reports are unchanged and still have zero steps. Direction
reversal still fails with zero steps; release during a corrective step remains
unexercised because its fixture never admits a swing.

All 84 workspace tests, formatting and native/WASM Clippy pass. Two complete
strict reports repeat identically, sources remain fixed during verification,
and core files match their tested snapshots. Two regressions cover full blend
rates/native installation, allocation handoff and swing precedence, gains and
ceilings, support-loss clearing, one counted landing and immutable body state.
Evidence is in `evidence/migration-stance-tracking-2026-10-05/final/`.
Next work is coherent post-landing stance/transfer references and feasible
lateral unloading. Continuous stepping, recovery and the full migration remain
unqualified; release admission is false. Earlier entries retain their source scope.

2026-10-05 swing tracking continuation: the admitted trajectory now supplies
native canonical angular target rates from fixed-pelvis IK samples one substep
apart. The rate is added to existing pelvis-torque feedforward, and effort
accounting uses the same desired-minus-measured coordinate rate. Position/rate
references retain the original sinusoidal 4 cm lift and 0.45-second duration;
gains, ceilings and physical step requirements remain unchanged. A regression
covers velocity-predicted FK, native motor installation and support-loss clearing
at two headings, without changing body snapshots. All 82 workspace tests,
formatting and native/WASM Clippy pass.

The complete strict reports repeat identically and match the retained candidate.
Five fixtures now complete a verified step, versus three before; backward and
held-target gain a landing, then fall at ticks 208 and 474 respectively (previous
failure ticks 227 and 540). All five completed-step fixtures still fall. Three
upright reports are byte-equivalent in data to the prior controller, still with
zero steps; strict qualification remains 0/9. Source-stable verification and
comparison are in `evidence/migration-support-reserve-2026-10-05/swing-final/`.
Five rejected transfer/coordination trials and a rejected squared-sine envelope
are preserved with decisions; five replay reports have verified lossless gzip
archives. A virtual-pose diagnostic preserves flat soles for tested stance and
short forward swing goals, while a 0.28 m forward plus 0.02 m lifted fixed-root
goal misses reach by 16.4 mm. Full pose/trajectory feasibility is still open.
Next physical work is post-landing stance tracking and feasible lateral unloading.
Production stepping/recovery remains disabled and release admission is false.

2026-10-05 landing continuation: the landing posture blend now captures only
its eight leg joints, clips declared coordinates to anatomical target limits
and zeros locked axes. A quaternion-based regression exercises both boundary
overshoots at two headings, verifies actual landing capture and preserves live
body snapshots. Another regression preserves the existing abort policy without
enabling permanent stance control or incrementing a step. All 81 workspace
tests, formatting and native/WASM Clippy pass.

The two strict replays are fully identical and remain 0/9, with three upright
and three one-step-then-fall fixtures. Measured physics, contact/motor telemetry
and failures match the previous verified source exactly; only 49 requested
locked-axis target values become zero. Source-stable verification and the
baseline comparison are retained in
`evidence/migration-handoff-2026-10-05/clamp-final/`. Two measured-posture abort
handoff policies regressed upright fixtures from three to zero and one; both
were rejected and removed, with receipts and reasons in `decisions.json`.
Feasible unloading and abort posture continuity remain unresolved. Release
admission is false, and production defaults still disable stepping/recovery.

2026-10-05 phase continuation: the two-second controller timeout now applies
separately to transfer and admitted swing. A late swing can finish its trajectory
without relaxing readiness, measured liftoff, touchdown or fixture deadlines.
The preserved central-reference diagnostic moves right-foot landing from an
early abort at tick 151 to physical touchdown at 169; it still falls later.
The new regression admits late swing time, rejects a loaded slide, enforces the
swing timeout and confirms unchanged live bodies. All 79 workspace tests,
formatting and native/WASM Clippy pass.

The final strict reports repeat identically and remain 0/9, with three upright
and three one-step-then-fall fixtures. Reversal falls earlier at tick 224 versus
233 in the prior constrained-IK source. No aggregate physical gate is qualified.
All experimental transfer/stance/load/damping policies were removed; `lib.rs`
is restored byte for byte. The build guard refreshed the restored inputs before
compiling. Source-stable verification is retained in
`evidence/migration-step-coordination-2026-10-05/phase-final/execution.json`,
and rejected trials in `phase-decisions.json`. Feasible unloading and continuous
posture intent across transfer abortion remain unresolved. Production defaults
still disable stepping/recovery, and release admission is false.

2026-10-05 build continuation: native checks/probes and worker/Trunk builds now
use a shared local-input content ledger before compilation. Changed or restored
files, including the vendored engine, receive fresh timestamps; valid unchanged
inputs preserve caches. Missing/invalid history refreshes all inputs. Native
receipts record the input fingerprint and refreshed paths. All 13 tooling/release
checks pass, including an actual pinned-Cargo A/B/restored-A executable regression
that reproduces the stale binary without the guard and rebuilds A with it.
The guarded repository probe in `evidence/migration-build-freshness-2026-10-05/`
rebuilt successfully and preserved the entire verified lateral physical report.
Direct Cargo invocations after checkpoint restoration must use the guard too.

2026-10-05 stepping continuation: anatomical clamping invalidated the analytic
foot endpoint for shifted stance and swing goals. A bounded virtual Jacobian
refinement now improves that endpoint using the existing iteration/increment
budgets and anatomical limits, without changing physics directly. Independent
forward kinematics passes all 36 new cases at less than 0.1 mm error; the
original first swing case missed by 1.28 mm. All 78 workspace tests, formatting
and native/WASM Clippy pass. Read-only disturbance traces additionally record
leg coordinates, actual configured axis ceilings and solved motor impulses.

Strict stepping remains 0/9, with three upright fixtures and three one-step
fixtures that subsequently fall. Bilateral stance, hip-transfer and provisional
unloading experiments were rejected and removed; controller policy, thresholds,
motor budgets and default disabled stepping/recovery flags remain as before.
The fresh source-bound receipt is
`evidence/migration-step-coordination-2026-10-05/final-fresh/execution.json`.
All physical replay data match the previous constrained-IK run after excluding
the added trace fields. The earlier `final/` receipt is invalid because
timestamp-preserving source restoration left a newer experimental executable
cached; touching the restored inputs forced the successful fresh rebuild.
The subsequent build continuation above supplies the restored-source guard.
This continuation does not qualify physical stepping or release admission.

2026-10-05 continuation: the native actuator delivery failure described below
is repaired. An isolated torque-free spherical-body test reproduced spin drift
from gyroscopic frame round trips. The engine now skips that calculation for
exactly equal principal inertias, which have zero gyroscopic torque. The
unchanged 432-case matrix passes all 1,728 substeps twice with byte-identical
reports; maximum readback error is 1.2158e-7 N m s against the original 1e-6
limit. All 77 workspace tests, three anisotropic gyroscopic regressions, three
cached-row equivalence tests, native/WASM Clippy, formatting, contact stress
and striker checks pass. Evidence is retained in
`evidence/migration-actuator-gyro-2026-10-04/verification.json`. This scoped
native repair does not qualify the remaining migration release gates.

Implemented changes:

- Compensated f32 velocity accumulation in Rapier external-force, joint and
  contact solves, with residuals preserved by scalar/SIMD gather/scatter and
  cached joint rows. Solved contact impulses, row order, friction bounds,
  schedules, timesteps and acceptance thresholds are unchanged.
- The unchanged 216-case contact stress matrix and 36 floor-disabled controls
  now require passes. Both passed twice in `rust-implementation-final-01`.
- Actual solved scalar and world angular motor-row impulses are measured
  separately from requested effort, with compensated observational totals.
  Measurements are unavailable before integration; the per-joint observation
  vector is reused. Independent body momentum probes cover all 24 joints,
  three headings, three coordinate postures, both signs and four substeps:
  432 cases. The unchanged allocated axis ceilings now bound every corner of
  the oblique angular impulse box inside their Euclidean vector ceiling.
  `rust-implementation-actuators-11` passes every sampled vector bound and
  opposite-row check, but only 280/432 complete probes. Body readback residuals
  reach 3.7956e-6 N m s against 1e-6; actuator acceptance remains failed.
- Step readiness requires moving-foot load at most 25% of body weight,
  persistent retained load at least 52%, and 0.10 consecutive seconds. Real
  liftoff and the frozen 0.09 m horizontal touchdown tolerance remain required.
  Landing captures both measured soles and blends standing intent. Committed
  steps request bounded equal-and-opposite hip restoration through native
  motors. Emergency follow-up transfers retain the same readiness requirement.
  Transfer bias is projected into the measured tapered support hull and holds
  its last admitted goal if support degenerates. Repeated held-demand transfers
  are admitted. Standing IK coordinates the measured soles and limits its
  requested root height to their two-link reach; these intents remain unqualified.
- Recovery exposes measured progress, phase time, unsupported time and retry
  counts. Stalled phases refresh measured geometry; unsupported attempts hand
  back to passive descent. These changes have not qualified physical get-up.
- Four legacy behavioral fixture ports retain the original schedules and
  assertions. Their explicit reset is limited to the legacy input-lockout
  test; recovery/cycle tests do not reset bodies or generations.
- Disturbance execution includes overpower fixtures and records cancelled
  press rejections instead of aborting the entire fixture matrix.
- Terrain trials include standing, support, traversal and settling observations
  for all 21 configurations, rather than only two-second chain checks.
- Full verification executes and aggregates all native stages twice, rejects
  absent coverage, failures, timeouts and source changes, and rejects zero
  selected engine tests. Foundation verification remains explicitly scoped.
- Browser instrumentation records displayed snapshot age, submitted input
  response, OS process memory and queue/buffer bounds. Hardware admission and
  the full 120-second soak are required. Physical display/touch and full timing
  admission remain unqualified.
- Candidate construction retains immutable root/subpath builds. Accepted bundle
  export and promotion validate source, asset and receipt identities without
  rebuilding. Dispatch-only qualification/promotion workflows require the
  frozen Windows runner and include post-publication checks and restoration of
  a prior accepted Rust artifact.

Diagnostic iterations before the final motor-vector enforcement:

| Fixture suite | Passed | Evidence directory |
| --- | ---: | --- |
| Strict stepping after stance reach admission | 0/9 | `evidence/rust-implementation-transfer-08` |
| Held-target experiment before stance reach admission | 1/9 | `evidence/rust-implementation-transfer-07` |
| Legacy behavioral ports before stance reach admission | 1/4 | `evidence/rust-implementation-legacy-07` |
| Complete exported disturbances | 5/14 | `evidence/rust-implementation-disturbances-03` |

Earlier complete native runs used the source snapshot retained inside
`evidence/rust-implementation-final-01/source`. They agreed byte for byte on
all emitted physical reports. The disturbance command error omitted its final
report, so complete repeatability coverage was correctly rejected. Scripts
changed during that run; its overall source identity was also rejected. Its
physical results are diagnostic evidence, not current-source acceptance:

- Recovery: 0/24 within 25 seconds plus stable standing; prone fixtures stalled
  in Brace, supine fixtures stalled in Roll, and some seeds failed collision
  integrity.
- Five uninterrupted cycles: 0 completed; the first striker fall descended
  physically but did not recover.
- Terrain: 0/21 full balance/traversal trials.
- Quiet standing: 60-degree native protocol foot drift reached 0.0100047 m
  against the frozen 0.010 m limit.
- 74 Rust workspace tests passed, along with formatting and native/WASM Clippy.
  The corrected native cached-row filter independently ran and passed all three
  bit-equivalence tests; the first full run's incorrect filter selected zero.
- Retained application build/typecheck passed. Its tests reported 378 passes,
  four failures (the four legacy behavioral tests), and seven skips out of 389.

`evidence/rust-implementation-browser-01` built and exercised both mounts,
both modes, WebGPU and forced WebGL2. UI, worker, lifecycle, graphics-loss,
standing and striker checks passed. Terrain UI/physics passed 40/42 cases;
gentle rubble failed both renderers with left-hand joint-limit error 0.073994
rad against 0.06. These builds predate the last controller changes.

The final current-source execution is retained separately in
`evidence/rust-implementation-frozen-14/execution.json`. It executes the native
stages twice, formatting, native/WASM Clippy, 76 workspace tests, the actual
three cached-row tests, tooling and retained application checks. Its source
snapshot and hashes identify precisely what was tested; inspect its stage
results instead of substituting the diagnostic results above. Interrupted
`frozen-04`, `browser-04`, `frozen-12` and `browser-12` runs have no acceptance
standing. The latter interruption corrected unavailable motor delivery before
the first actual solve. `frozen-13` stopped with ENOSPC and its receipt was
truncated. Disposable old test binaries and the standalone vendored build cache
were reclaimed; historical evidence was preserved. Receipt writes now use an
atomic replacement, and failed accepted-bundle export rejects promotion. None
of those interrupted executions is a release receipt.

Required remaining work:

1. Qualify all nine steps, all four legacy behaviors and quiet standing while
   preserving gentle recoverability and integrated overpower descent.
2. Complete all 24 physical recoveries and five uninterrupted cycles.
3. Qualify balance and complete authored course traversal, including obstacle
   contacts and final standing, across all stations and difficulties.
4. Complete native/WASM scalar/cached equivalence; the independent native
   delivered motor/body momentum probes now pass in the 2026-10-05 continuation.
5. Qualify current production worker/browser bytes at both mounts, including
   actual touchscreen behavior, every frozen timing/memory/buffer limit and soak.
   The default installed Edge reports 154.0.4258.53; the contract requires
   154.0.4258.37. A matching runtime has not been admitted.
6. Qualify the self-hosted Windows workflow and import/verify the currently
   deployed TypeScript artifact as the first migration's rollback artifact.
   The current rollback path supports prior accepted Rust bundles only.
7. Only after complete frozen-source acceptance: enable production stepping and
   recovery, switch normal commands, finish preview controls, remove migration
   notices and retire the TypeScript runtime/dependency. Requalify the final
   sources and assets before promoting those exact bytes.

The full runner intentionally leaves unsupported coverage failed or missing;
no acceptance flag is used to bypass an unimplemented contract.
