# H75-v1 — native angular representation and signed motor probes

2026-09-27. **Partial diagnostic executed; full row inspection incomplete and
standing feasibility still unestablished.** All 342 frozen copied steps completed.
Six untouched controls reproduce the original 25-body outputs exactly, and all
168 perturbed commands repeat exactly. No physical controller repair is justified.
Three failed native builds exhaust the separate build budget. All failures remain.

## Frozen scope and preserved inputs

The [versioned manifest](experiments/standing-h75-v1.json) was saved before the
builds or physical trials. It names H74's three saved pre-step states (ticks
133, 120, 123), fourteen existing leg axes, biases −0.5, −0.25, +0.25, +0.5 Nm,
two independent copies per command, two untouched controls per state, and one
step per copy: **342 physical steps total**. Limits are 600 s for the experiment,
three native build attempts / 300 s build time, 384 MiB Node old space and a
768 MiB sampled process RSS ceiling. No fourth build or additional physical
query was run. H74-v1's reference trials remain exhausted and unchanged.

Before and after the probes, all **176 listed H74 artifacts** verify across
`attempt-01`, `analysis-01` and `validation-01`; all **315 retained source/config
hashes**, installed dependency hashes and the Node executable match. Every
member of H74's source snapshot is checked. The native build verifies all 146
engine source files against the retained precision investigation. The four
H42/H43 native-source archive members also match. The executed H75 snapshot
preserves 319 source/config files as base64, the initial Git diff/status, exact
commands, executable hashes and runtime details. New diagnostic sources are
additional to the unchanged H74 runtime.

Each copy retains its original 25 dynamic bodies, original fixed floor,
colliders, joint frames, limits, caps, stiffness/damping, solver/timestep and
collision hooks. Body state setters are forbidden. The sole intervention is
`targetVelocity += biasNm / damping` on one motor axis through the existing f32
ABI. Requests crossing an original cap would fail the diagnostic; none does.
The largest effective-bias rounding error is 0.00000190735 Nm. Native readback
verifies the sole changed target velocity for every distinct command. Original
commands, controller state, references and contacts remain in the copied failure
record; every trial retains pre/post native snapshots and all output bodies.

These are offline copies, never an alternative startup or a continued standing
history. No settling interval, acceptance threshold, fall guard or H73 input is
changed. The same runtime remains opt-in through `?standingCandidate=h74-v1`.

## Executed results

| Saved reference | Original peak angular speed | Best single-axis probe | Resulting peak | Axes meeting both signed-response tests |
| --- | ---: | --- | ---: | ---: |
| 0 mm, tick 133 | 0.501134 rad/s | Right forefoot X, −0.5 Nm | 0.464467 rad/s | 14 / 14 |
| −5 mm, tick 120 | 0.056990 rad/s | Right forefoot X, +0.25 Nm | 0.048757 rad/s | 12 / 14 |
| +5 mm, tick 123 | 0.459446 rad/s | Left forefoot X, +0.5 Nm | 0.430944 rad/s | 14 / 14 |

The central right-forefoot −0.25 Nm probe reaches 0.483549 rad/s; the +0.25 and
+0.5 Nm probes worsen it to 0.515240 and 0.535589 rad/s. At +5 mm the left
forefoot +0.25 Nm probe reaches 0.445226 rad/s, while −0.25 and −0.5 Nm worsen
it to 0.473565 and 0.488369 rad/s. The signed effect therefore repeats at the
second predeclared magnitude. Neither state's best probe meets H74's 0.4 rad/s
reference reserve. The −5 mm original failure concerns 5.154 mm reference foot
error, not an official speed failure; its low speeds do not repair that gate.

At −5 mm, left hindfoot Z and left forefoot X fail the declared simple sign
test. Both negative hindfoot probes produce small positive relative-rate
changes (+0.000399 / +0.000730 rad/s). Both positive forefoot probes produce
negative changes (−0.013763 / −0.001798 rad/s). These repeat exactly and remain
unresolved coupled responses, not a motor-sign correction to apply globally.

All 171 command pairs (three controls plus 168 perturbations) reproduce complete
body states **and contact records** exactly. Of 342 output samples, 40 central
samples exceed an official speed bound (including the two unchanged failures);
they are retained. Contact pair, manifold contact and solver-contact counts
remain unchanged relative to each state's control throughout this set. This
does not exclude changes in contact force, friction regime or internal substeps.

## What the representation check establishes

The retained native source uses `2*asin(q_i)` for the motor position coordinate,
centered `2*atan2(q_i,q_w)` for angular limits, parent-frame basis vectors for
motor/limit rows, and quaternion-derivative rows for angular locks. The compiled
native helper also orthogonalizes rows. These are distinct representations.

**All 46 native motor position stiffnesses are zero in all three saved states.**
H74 computes posture feedback in the application and encodes its bounded torque
through target velocity plus native damping. Thus the native arcsine position
coordinate contributes exactly zero positional motor request here. Changing
that native position formula cannot directly correct these commands.

The fallback inspection reads actual joint frames and limits through WASM, then
evaluates the retained coordinate formulas and raw motor basis in JavaScript.
Its limit-coordinate difference from the application is at most 5.56e−17 rad;
this is **not independent native f32 coordinate evaluation**. The application's
analytic coordinate Jacobians agree with algebraic ±1e−6 rad finite differences
to at most 1.54e−10. Across all enabled axes the largest motor/limit coordinate
difference is 0.001970 rad, but its native positional request effect is zero.
At the central failing right forefoot that coordinate difference is only
4.37e−12 rad and the permitted-axis Jacobian difference has norm 2.44e−5.

More substantial motion lies in constrained directions. At the central output,
the right forefoot's child-minus-parent angular velocity projected into its
**pre-step native parent joint frame** has permitted X rate +0.351222 rad/s
and locked Y/Z norm **0.212204 rad/s**, despite a maximum locked coordinate of
0.0000320 rad. At +5 mm, the left forefoot's locked Y/Z norm is **0.489605 rad/s**,
despite a maximum locked coordinate of 0.0001181 rad. The central pose-derived
versus final-native angular-vector difference remains 0.150542 rad/s.

Small coordinate errors with these endpoint rates warrant examination of the
twenty native substeps. They do not prove a constraint defect or remove controller
and contact coupling as explanations. Endpoint rate, pose-averaged rate, a saved
impulse and a delivered whole-step torque are different measurements.

## Missing row evidence and preserved build failures

The read-only inspector source calls the retained native row helpers and
finalization through visibility-only changes in an isolated engine copy. It
would reconstruct rows at saved poses; even a successful build would not itself
trace the changing rows inside WASM integration.

1. `native-build-01`: the old isolated GNU toolchain is absent; rustup's channel
   request fails under restricted network access.
2. `native-build-02`: the installed MSVC compiler cannot find `link.exe` in its
   environment; inspection also finds its Windows SDK libraries unavailable.
3. `native-build-03`: after installing the pinned GNU toolchain, the library
   compiles but the inspector fails with Rust E0034, ambiguous scalar/SIMD
   `JointConstraint::update`. The source now explicitly names `<f32, 1>`;
   **that correction has not been compiled**, because the three-build budget is
   exhausted. The original failed source remains in the build directory.

The three recorded build times sum to 281.763 s. An intervening GNU installation
on O: fails with disk-space error 112 and rolls back; its downloads and failure
record remain. Installation succeeds in this chat's writable C: staging area.
No failed evidence was deleted to free space. Build and installation evidence
are under `evidence/standing-h75-v1/`; C: holds the successful toolchain and
native build cache. No production dependency or runtime binary was replaced.

The existing H74-verified `snapshot_motors.exe` was used for independent native
configuration readback. **Full raw/finalized angular rows, effective row masses,
RHS/CFM and substep impulse attribution remain incomplete.** Files ending in
`.frames.json` explicitly identify the JS formula reconstruction and missing
finalized rows. No overall diagnostic pass or candidate repair follows.

## Bounds, verification and reproduction

The serial physical operation completes in **37.705 s** and exits **1** because
the complete diagnostic/feasibility gates remain incomplete. The largest sampled
Node RSS is **155.53 MiB**, JS heap 28.58 MiB and WASM linear memory 2.625 MiB.
External allocations and array buffers are recorded separately and overlap RSS.
The retained native inspector has no process-memory measurement; this is not a
complete process-memory acceptance result or a browser benchmark.

All copied outputs remain finite, with unchanged ownership and body masses.
Maximum anchor separation is 0.02958 mm, coordinate-limit error 0.001981 rad,
native contact-distance floor penetration 1.170 mm and measured non-excluded
self-penetration zero. These short copied checks do not replace the independent
geometry, full structural, continuous-standing or application gates.

The executed command, with immutable output protection, was:

```powershell
node --max-old-space-size=384 --import tsx scripts/standing-angular-diagnostic.mjs evidence/standing-h75-v1/attempt-01 retained-inspector
node scripts/analyze-standing-angular.mjs evidence/standing-h75-v1/attempt-01 evidence/standing-h75-v1/analysis-01
```

The first command consumes all 342 declared physical queries. Do not extend or
retune H75-v1; a further experiment needs a new frozen version. The analysis
command can be repeated into a **fresh analysis directory** without simulation.
It verifies the 1,923 attempt artifacts, restores all 319 captured source members,
checks matching current source, and verifies every repeated contact/body record.
Syntax checks and targeted ESLint pass. H74's 81 regressions, five candidate tests
and typecheck retain their unchanged source scope; they were not rerun or claimed
as new H75 physical acceptance.

Evidence:
[attempt report](../evidence/standing-h75-v1/attempt-01/report.json),
[analysis](../evidence/standing-h75-v1/analysis-01/report.json),
[run fingerprint](../evidence/standing-h75-v1/attempt-01/run-manifest.json),
[artifact inventory](../evidence/standing-h75-v1/attempt-01/artifacts.json),
[failed native build](../evidence/standing-h75-v1/native-build-03/build-report.json),
[validation](../evidence/standing-h75-v1/validation-01/report.json).

## Concrete next diagnostic

Freeze a separate bounded **build-and-read** version: one cached build of the
explicit scalar inspector (at most 90 s), then six reads (three original
pre-step snapshots and three exact control post-step snapshots), **zero new
physical steps**, and at most 120 s inspection wall time. Reuse the preserved
native-build-03 library without altering it. Verify source/runtime identities,
row count/order, motor and limit coordinates, raw/finalized Jacobians, effective
row masses, RHS/CFM and stored impulses. A failed build/read remains incomplete.

If endpoint row inspection still does not isolate the mechanism, a subsequent
separately frozen experiment must instrument the actual WASM substeps and require
unchanged 25-body control outputs while recording contact and constraint impulses
and pose/velocity updates. Do not infer their causal split from the present
endpoint probes. Quiet standing, local return, backup validity, sustained and
disturbance tests, browser acceptance and H73's separate comparison remain open.
