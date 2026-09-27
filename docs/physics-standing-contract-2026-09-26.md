# Stationary standing investigation — 2026-09-26

See [current status](status.md) for the latest recorded gate summary and [evidence availability](evidence.md) for recovered originals and missing files. This investigation records its original observations; the documentation repair is not a new physics run.

Updated implementation note: H8 now has controlled [probe](../scripts/probe-standing-causality.mjs) modes and a [staged evaluator](../scripts/evaluate-standing-h8.mjs). The [new evaluation](standing-h8-evaluation.md) regenerated the input with matching controls, passed static/local checks, then rejected H8 after all six prolonged screens failed. No production change was adopted; the contract below is unchanged. The following original investigation remains historical.

Acceptance remains open. This contract implements the revised sequence: explain
stationary standing, then preserve it while adding transfers, single steps,
sequences, and integration. One agent; no prescribed controller rewrite.

**Recorded checkpoint: baseline and diagnostic tooling delivered; no behavioral
repair accepted.** The investigation reported that all 25 saved focused hashes
and all 75 prior completion-evidence files matched at that time, and that 57 new
diagnostic runs completed. Those claims were not fully reverified during the
documentation repair; the focused report is now unavailable at its original path.
The recorded 140/142 result applies only to its recorded source scope. A controlled allocator
failure is localized, but stationary standing and every later milestone remain
open; no integration gate is presented as passed.

The original `evidence/physics-standing-20260926/index.json` and
`evidence/physics-standing-20260926/production-ticks-175-176.jsonl.gz` are
unavailable. A byte-identical surviving [baseline ledger](checkpoints/2026-09-26/baseline.json)
and five other original reports were recovered into the documentation; their
locations and checksums are in the [recovery manifest](checkpoints/2026-09-26/recovery-manifest.json).
The investigation stored further artifacts at
`C:\Users\flori\AppData\Local\Temp\physics-standing-20260926` after O: ran out
of space. That directory survives, but some trace files are empty. Do not assume
every referenced run remains recoverable. Earlier reported results retain their
original source scope.

## Baseline and invariants

The saved `evidence/physics-repair-20260924-completion/focused-restored-retained-source.json`
is immutable. Verify all 25 before/after hashes before reusing its 140/142 result.
`scripts/capture-physics-baseline.mjs` records the pinned executable and hash,
installed critical dependency versions, lockfile, complete source/test/config
hashes, physics settings, scenario inputs, existing diff and untracked work,
and hashes of the previous completion evidence in a fresh evidence directory.
The historical 25 hashes alone do not fingerprint every transitive dependency;
retain that limitation rather than treating them as a complete release gate.

Keep timestep, solver settings, all actuator ceilings, acceptance thresholds,
collision rules, physical fall guards, and continuous Rapier ownership fixed.
Do not refresh fixtures to make a failed behavior pass. Preserve rejected runs;
revert only each experiment's changes. A delayed fall does not demonstrate an
improvement when coherence, motion limits, or a previously passing check fails.

| Gate | Baseline status | Evidence / next requirement |
| --- | --- | --- |
| Focused suite | 140/142, exit 1 | Saved restored-source manifest; two balance failures: slow pull tick 216, planted reversal tick 343 |
| Focused coherence, collisions, actuation, ownership | Passing within the 140 | Reuse only while dependent source/runtime remain unchanged |
| Official idle, headings 0, +π/3, −π/4 | Failed at all headings | `official-idle-final-source-rerun.json`; see metrics below |
| Both balance probes | Failed | `final-restored-source-checks.json`; capture/height failures |
| TypeScript / ESLint | Last recorded pass / pass with six warnings | Same manifest; rerun after relevant edits |
| Small impulse paired diagnostic | Separate open outlier | 15.23% right-thigh mismatch at 0.0005 Nm·s; no shared cause established |
| Application build/tests; full physics | Unverified on final repaired source | Run after preceding physical gates pass |
| Five-cycle protocol; recovery; terrain | Unverified on final repaired source | Preserve all existing stricter requirements |
| Browser, canvas2d and Three; Pages, both configurations | Unverified on final repaired source | Record visual findings and command/configuration hashes |

| Heading | Endpoint pelvis drift m | Endpoint foot drift m | Peak speed m/s | Peak angular speed rad/s |
| --- | ---: | ---: | ---: | ---: |
| 0 | 0.051197 | 0.0353 | 0.6763 | 1.5335 |
| +π/3 | 0.0407 | 0.0396 | 0.6789 | 1.5429 |
| −π/4 | 0.056833 | 0.091682 | 0.6870 | 2.6904 |

## Frozen measurement contracts

Stationary acceptance uses the unchanged official scenario: 2 s settling and
30 s observation at all three headings; endpoint horizontal pelvis drift ≤0.03 m,
hindfoot/forefoot drift ≤0.01 m, all-segment peak speed ≤0.1 m/s and angular
speed ≤0.5 rad/s, zero additional steps, both feet planted, accepted upright
state (`upright` or `reacting` per the official definition). Also report maximum
horizontal excursions from the observation start and vertical ranges, so endpoint
drift cannot conceal oscillation. These extra reports do not replace official metrics.

Repeatable transfer means three left–right–neutral cycles per heading within
one uninterrupted run (no reset, teleport, velocity clearing, or hidden settling
restart). A target must qualify within 3 s. Planted hindfoot and forefoot slip
must remain ≤0.01 m from the start of each transfer and ≤0.01 m cumulatively
through a cycle. Return to neutral must settle within 2 s, then sustain at least
1 s of double support, pelvis excursion ≤0.03 m and foot excursion ≤0.01 m
from the neutral checkpoint, and the stationary all-segment speed limits.
Retain stricter existing limits. Before swing: retained measured load ≥52% of
total body weight and moving measured load ≤25% for 0.10 consecutive seconds.
Predicted load never authorizes readiness.

Prediction is frozen before tuning: for matched initial state and command
history over 0.5 s, each foot-load-change error ≤10% of total body weight,
horizontal COM-displacement error ≤0.02 m, readiness-time error ≤0.05 s.
Report maxima and RMS errors, never averages alone. Evaluate held-out signed
command magnitudes at 75% and 125% of the calibration magnitude (within existing
caps) and requests exceeding available support/friction/moment capacity.
Infeasible requests must be labelled explicitly; a readiness prediction for an
infeasible request is a failure. The production simulation remains the oracle.

A complete step requires qualified transfer, measured swing unloading <3 N
for 0.05 consecutive seconds, measured touchdown within 0.09 m for 0.10 s,
completed touchdown/cooldown, then ≥1 s stable double support under the
stationary motion limits relative to the endpoint checkpoint. Prove each
direction at each heading, then subsequent steps, release in transfer/swing,
and reversal without resets. All 142 focused tests, new physical regressions,
both balance probes, official idle, and structural acceptance must pass.

Final integration must use one executable source/configuration fingerprint.
Record commands, exits, scenario counts, numerical results and visuals.
Any later behavior change invalidates dependent evidence. No stale or unverified
row counts as acceptance.

## First experiment contract: command construction and response

Hypothesis H1: a specific posture, gravity/load-compensation, or pelvis-correction
request drives the first measurable stationary divergence. An alternative is that
the first error is upstream reference motion or downstream contact/motor tracking.
Do not assume reference holding is a repair: the prior combined reference
experiment failed coherence (0.03035 m joint separation against 0.01 m).

Use production controller, allocator, native motors and contacts on flat ground.
Run unchanged control, an exact replay, instrumented unchanged control, ±0.5 Nm
parent-joint-X thigh command pulses, and separate posture, gravity/load and pelvis
correction ablations. Pulse/ablation starts at tick 121 (after 2 s) for 6 ticks;
observe the first response tick and 0.5 s thereafter, then continue to 4 s.
No reset within a run. Each pair starts from identical construction/options and
must match all physical pre-intervention states exactly.

Predictions: instrumented and uninstrumented physical trajectories must match
exactly. A signed pulse must produce an opposite signed first-tick relative
angular response in the named parent-frame axis when not curtailed by a cap or
constraint; otherwise report the constraint and do not assert a sign conclusion.
A culpable contribution ablation must reduce the corresponding tracking error or
all-segment motion by ≥20% in the 0.5 s window, with no worsening of separation,
penetration, ownership, or measured support. Report each rather than cherry-pick.
No short experiment is an accepted behavioral checkpoint.

Capture command tick k with pre-step physical/contact state (response to k−1),
post-step body/contact state (response to k), references before/after with update
reasons, force feasibility/residuals, per-axis posture/damping/gravity/pelvis/
trajectory requests and final capped requests, tracking and body motion.
Native request is not delivered motor torque: the pinned Rapier JS API does not
expose individual motor impulse readback. Measured angular/contact changes are
coupled physical responses and cannot be assigned wholly to one motor.

Exit only on a repeatable earliest divergence plus a controlled intervention;
revise rejected hypotheses before further tuning. Test the resulting explanation
on an independent command magnitude before accepting the standing milestone.

## H1 result and next falsifiable hypothesis

The 24 four-second runs (three headings × eight modes) reproduce control exactly
with instrumentation and on replay. Every pair is identical before tick 121.
±0.5 Nm left-hip pulses produce signed first-tick relative-speed changes of
approximately +0.0068 to +0.0072 / −0.0066 to −0.0073 rad/s. None of the broad
ablations meets the declared improvement. Removing gravity/load compensation
increases peak linear motion 1.61–1.73× and angular motion 2.96–3.09× in the
window. H1 does not identify any of those contributions as removable.

H2: non-leg targets are expressed against measured parent orientation each tick,
so their positional feedback tracks a world orientation, while native damping
uses child-minus-parent angular velocity. This mismatch adds a parent-velocity
drive to an otherwise world-referenced target. Observed upper-limb oscillations
occur without actuator clipping or an infeasible allocator at the sampled ticks.
This is a specific tracking-frame hypothesis, not a gain sweep.

Intervention: in a runtime-only wrapper, subtract `damping * parent angular
velocity` from upper-limb feedforward, in world coordinates, while leaving
position targets, gains, budgets, gravity compensation, and all leg control
unchanged. Apply from tick 121 through 240. The request algebra must show the
expected subtraction exactly. Prediction: within 0.5 s, upper-limb tracking/motion
declines ≥20%, with no degradation of structural limits or measured support;
by 4 s report all segment peaks and excursions. Reject if absent at any heading.
The control and pre-intervention trajectory remain bit-identical. Even success
would require full stationary and existing physical gates before retention.

H2 is rejected under its 0.5 s contract: linear peaks improve only 8–9%, while
angular peaks worsen 33–35%. In the final second the hand-dominated linear peaks
fall 70–75%, but that later observation does not rescue the original prediction.
No production change is retained. A separate initialization experiment H3 will
test the frame correction from tick 1, allowing the unchanged official 2 s
settling interval. Predict ≥20% improvement in both post-settling speed peaks
over a 10 s screen, feet remaining within 0.01 m, no structural regression.
Reject immediately if the screen misses those predictions; run the unchanged
32 s official gate only if the screen passes. This tests startup behavior, not
a longer waiting period after an accepted settling interval.

H3 fails its slip bound (0.01218/0.01860/0.02116 m at the three headings).
It reduces linear peaks to 0.09261/0.09894/0.07955 m/s, but forefoot angular
peaks remain 0.50570/0.83671/0.84353 rad/s. It is not an accepted repair.

H4 tests the two demonstrated mechanisms together, without changing gains:
upper-limb tracking-frame damping plus holding initial posture feet/rotations
and neutral COM reference during uninterrupted idle. Measured contact positions,
loads and hulls remain untouched. Use runtime property interception only, not
synthetic measurements. Include held-reference-only and tracking-only controls
to separate the effects. Prediction: after the same 2 s settling, the 10 s
screen meets all stationary motion/drift bounds and structural limits at every
heading. Otherwise reject and do not promote to the official idle gate. Holding
alone was previously rejected; this factorial experiment tests whether removing
the demonstrated upper-limb excitation changes that outcome, not whether a later
fall can be accepted. No transition/rebase policy is implemented before this
restricted idle hypothesis passes.

H4 fails: combined foot excursions are 0.01410/0.01236/0.01978 m; angular
peaks are 0.88903/0.43444/0.90548 rad/s. No reference or damping change is retained.

H5 targets an allocator-to-motor discontinuity in unchanged production control.
At +π/3, tick 175→176, measured left-hindfoot load changes 370.75→356.93 N,
while its planned load changes 312.98→116.87 N. Left-forefoot manifold count
changes 2→4; right-forefoot planned load drops 86.33→0 N despite its measured
load rising 36.03→42.88 N. Hindfoot gravity/load requests reverse sign and the
left forefoot reaches 1.07393 rad/s in the next physical response.

Prediction: replacing only the lower-chain gravity/load request at tick 176 with
its tick-175 value will reduce the tick-176 left-forefoot angular speed ≥20%,
with identical history through tick 175, unchanged current contact measurement,
and no structural/ownership violation. Observe five following ticks, including
any delayed spike; do not classify a postponed peak as a repair. Compare a
whole lower-chain intervention with a foot-only intervention. This is a causal
probe, not permission to reuse obsolete contacts in production or to smooth
infeasible allocations. Follow it with static replay of the recorded allocator
inputs to isolate which manifold/ownership change creates the redistribution.

H5 passes its local causal prediction: lower-chain one-tick hold reduces the
forefoot spike 1.07393→0.04309 rad/s; the next five samples stay ≤0.16746 rad/s.
Foot-only intervention gives 0.65340 rad/s. Prehistory matches exactly through
tick 175. Offline production allocator replay is exact (JSON signed-zero semantics).
Holding only the old requested force or COM leaves the redistribution; replacing
the old right-hindfoot manifold restores left-hindfoot planned load to 321.68 N.
The hull still contains measured points. The defect is unstable force sharing as
nearly collinear measured points enter/leave the hull's vertex set, not demonstrated
force infeasibility. Current mean-value coordinates are continuous for a fixed hull
but do not preserve load sharing when vertex membership changes.

H6 experiment: a constrained weighted least-squares projection of measured patch
shares over all current measured points, preserving the existing total force and
horizontal pressure moment exactly. No new contacts, gains, caps, thresholds,
fixtures, or force-feasibility policy. Rank-deficient/infeasible cases explicitly
fall back and are counted. This remains in an experimental script; production
source is unchanged. Predict the tick-175/176 static load jump falls below 20 N,
force/moment residuals ≤1e-8 in normalized coordinates, no negative shares, and
the tick-176 response falls ≥20% under a matched intervention. If that succeeds,
screen projection alone and projection plus the upper-limb frame correction
from initialization for 10 s; require all stationary limits and existing structural
limits at all headings before considering a production candidate.

H6 is rejected at the static screen: largest planned-load jump is 24.50882 N,
above the frozen 20 N bound (left hindfoot 14.93952 N, left forefoot 17.10267 N).
No runtime sweep or production change follows that failed prediction. The next
repair hypothesis must address manifold-dependent force sharing while retaining
force/moment feasibility and independently fixing upper-limb tracking; neither
an unverified allocator nor reference holding alone is accepted.

Independent command-response check: calibrate the first-tick left-hip relative
speed slope at each heading from the signed ±0.5 Nm pair. Before running a held-out
±0.625 Nm pulse at tick 121, predict the signed first-tick response by that slope;
require ≤15% relative error and correct sign at every heading. This tests only
the local actuator response, not transfer prediction or the standing milestone.

The held-out check passes all six signed heading cases, with 2.26–7.04% error.
Final targeted ESLint and `git diff --check` pass. No production source or test
has changed during this investigation. A repeat of the unchanged 2+30 s idle
timeline reproduces the saved official endpoint/speed values exactly and fills
the previously missing excursion coverage:

| Heading | Max pelvis horizontal excursion m | Max foot horizontal excursion m | Pelvis vertical range m |
| --- | ---: | ---: | ---: |
| 0 | 0.05119714 | 0.03545270 | 0.01639152 |
| +π/3 | 0.04568137 | 0.03979506 | 0.01675725 |
| −π/4 | 0.05683257 | 0.09168228 | 0.00974935 |

All three runs retain double support and zero steps, yet all fail stationary
motion limits. This supplementary diagnostic does not replace the official
harness or promote its structural observations to a full structural acceptance.

The following historical command requires the original two-tick input fixture.
That fixture is currently unavailable; see [evidence recovery](evidence.md#still-unavailable)
for its checksum and recovery requirements. The surviving [original replay report](checkpoints/2026-09-26/allocator-replay-verification.json)
can be inspected, but it cannot replace the missing input. Once the original
fixture is restored, run against the matching source using the active Node:

```powershell
node scripts/replay-standing-allocation.mjs evidence/physics-standing-20260926/production-ticks-175-176.jsonl.gz 176 "$env:TEMP\standing-allocation-replay.json"
```

Use a fresh output filename; the tools refuse to overwrite completed evidence.
`probe-standing-causality.mjs` defaults to unchanged/replay/signed pulses and the
three independent ablations at all headings. The reference, tracking and load
projection modes are explicitly named rejected experiments, not runtime features.

## Continued repair: H7

H6 also fed instantaneous measured force amplitudes into its allocation objective.
In the causal pair, right-hindfoot measured load rises by 28.77 N, so that objective
can recreate a rapid commanded-load change even after removing hull-vertex
dependence. H7 separates support eligibility from the force-sharing objective:
use equal reference weight per currently eligible measured patch, divided over
its real manifold points, then solve the same constrained projection. Measured
loads still authorize support/readiness; they are not a desired force split.
No temporal filtering, stale contacts, added support, budget changes, or gains.

Before implementation, retain H6's ≤20 N maximum jump on the recorded pair and
force/moment/positivity checks. If it passes, predict ≥20% reduction of the
tick-176 forefoot response from an identical prehistory, with no postponed spike
within five ticks. Then screen 10 s from initialization (2 s settling) with and
without the independently demonstrated upper-limb frame correction. Require all
stationary motion and structural bounds before the full official idle gate.

H7 fails: maximum jump 22.84871 N. Removing measured amplitude feedback alone
does not remove the manifold-count-dependent redistribution.

H8 instead holds each still-qualified patch's previous commanded share and
pressure reference, projecting those references onto its **current** measured
manifold. Solve for the new total requested pressure with those patch shares;
if the shares are infeasible or support membership changes, report that condition
and use the existing current-support allocation. Old points never become contacts.
This tests continuity of command references rather than smoothing measurements.
Keep the same ≤20 N static jump, exact force/horizontal-moment and nonnegativity
requirements, ≥20% tick-176 response reduction, five-tick no-postponement check,
and all-heading 10 s screen. Count every infeasible/reset event. Reject before
promotion if any contract fails; no parameter sweep is authorized by a failure.
