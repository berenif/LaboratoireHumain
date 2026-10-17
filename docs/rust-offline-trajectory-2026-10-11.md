# Offline native whole-trajectory search — 2026-10-11

Research on local branch `investigate/physical-support`, base/HEAD
`c597959f46f37715035349c8cd0a699b6822111a`. No push or merge. Recovery remains
unqualified and default-off. This document separates the trajectory family,
native evaluations, independent replay and physical qualification.

## Why offline

The previous native local correction faithfully reproduced all25 bodies but
failed after1.392s and required about459ms per4.17ms native substep. It remains
archived and was removed from live control. The new search evaluates whole
motor-command programs offline; execution does only interpolation and capped
allocation. No native clone, finite-difference search or QP runs at each step.

The unchanged-controller witness binary matches its original SHA-256 manifest.
Fresh2s traces at0° and60° exactly reproduce all480 original native steps.
At50ms only the forefeet have resolved load; at100ms the hindfeet also carry
load but capture remains outside. The witness subsequently falls and ends in
Brace. Acquiring a heel contact alone is therefore insufficient.

## Family and interpretation

The family is sagittal and symmetric, with two discrete pivot strategies.
The coupled strategy changes all joints toward the low crouch reference;
the foot-first strategy keeps initial hip/knee/torso coordinates while the
ankles/toes acquire heel support, then changes to the low recenter waypoint.
This sequencing comparison follows the measured witness: heels are already
loaded at100ms while the original controller has entered Stand.
It starts from the actual crouch fixture,
commands a smooth toe/heel pivot toward the feasible low static joint pose,
changes a low-pose recenter waypoint, waits for measured support and capture,
then follows41 static-plan coordinate rows toward upright and requests native
ordinary-controller handoff. Static geometry gives reference joint coordinates;
it neither prescribes body poses nor proves dynamic support.

| Variable | Range | Intended mechanical effect |
|---|---:|---|
| Pivot duration |0.15–0.75s| Time for loaded toe rotation and heel acquisition |
| Recenter duration |0.25–1.5s| Time for the low supported joint-waypoint change |
| Rise duration |1–4s| Extension timing after the physical gate |
| Hip waypoint offset |±0.28rad| Alter the sagittal pelvis/upper-leg relationship |
| Knee waypoint offset |±0.28rad| Alter crouch depth and leg geometry |
| Ankle waypoint offset |±0.22rad| Alter the pivot/foot-leg relationship |
| Torso waypoint offset |±0.22rad| Alter upper-body counter-motion |
| Forefoot waypoint offset |±0.24rad| Alter the loaded toe hinge during pivot |

Offsets are command-waypoint parameters, not gains. Final targets remain
inside original anatomical endpoints; saturated target coordinates have zero
outward feedforward. Gains, ForceBased model, mass, geometry, contacts and
physical time step remain the pinned canonical values. Individual torque caps
and760Nm aggregate stay strict. A downward f32 correction only removes rounding
excess. No external pelvis force, body write/reset, velocity transplant or
contact creation is allowed.

The rise gate requires both actually loaded hindfeet, original upwardness,
all25 bodies below original velocity bounds, geometric balance AND capture in
the hull of persistent actually resolved foot contacts for0.55s. Handoff is
requested only after the planned rise is finished and original standing checks
pass. Physical success must finish within25s and sustain the original1s hold.
Thirty-second ordinary standing, other headings and repeated cycles are
additional original qualification gates; this family cannot close them merely
by a diagnostic test exiting successfully.

## Allocation and evaluation

First compare the same baseline sequence with inherited prepared caps and
caps computed from the native command owner's measured spring request. The
owned allocation uses the existing canonical0.10 reserve +1.25 demand policy,
anatomical clipping and proportional760Nm scaling. This is a command ownership
change, not a gain sweep. Desired requests, installed caps, native impulses,
COM acceleration and resolved contacts are distinct reported quantities.

Each evaluation integrates the actual full native program until its original
integrity/support refusal, physical deadline/hold, or explicitly labelled
search wall interruption. An early refusal is a complete failed attempt at
that program; it is not a simulated continuation through the unexecuted rise.
A feet-only program is additionally refused when an actual non-foot floor load
reaches the original 3 N load threshold for the original 12-substep persistence.
This conservative family consistency check does not relax qualification.
The objective ranks original success, integrity, known outcome, absence and
severity of unexpected support, then gated progress, support/capture and stability.
Its score is an optimization aid, never a replacement acceptance threshold.

The deterministic search uses ten mechanically interpreted initial sequences (eight coupled and two foot-first)
and bounded coordinate changes of the trajectory variables. Duplicate
parameter definitions reuse a versioned cache. Parameters belonging only to
unreached phases must be reported as untested, not optimized successfully.
The budget is at most48 native evaluations and600s search wall time, with a
separate fresh best-case replay for independent validation. Original guards
stop a failed integration; cached reports are immutable evidence. Timings of
command generation are measured separately from full native integration and
telemetry. No interactive success is inferred solely from cheap interpolation.

## Evidence

`evidence/offline-trajectory-2026-10-11/PLAN.md` defines the scope before search.
`baseline-repeat-verification.json` verifies the independent witness replay.
`baseline-reproduced.png` labels toe loading, heel loading and the subsequent
failure. Results are recorded below and in `family-analysis.json`.


## Completed bounded result

Campaign A completed four programs and was deliberately stopped during a fifth.
The inherited baseline failed at 1.075 s; three owned-allocation programs ran
through the full 26 s horizon without ever triggering rise. Inspection found
that the torso/head had fallen onto the floor while both forefeet stayed loaded.
This exposed a missing contact-mode check in the research objective, not a
physical recovery. Exact A source, executable and raw reports are preserved.

Campaign B adds the conservative unexpected-support refusal described above.
Its scheduled ten seeds and two coordinate rounds completed: 38 queries,
25 distinct native evaluations (23 coupled, two foot-first), 13 local cache
hits, 48.772 s search time. Every program was refused for unintended persistent
head/torso support, between 0.929 and 1.867 s. All have zero stable substeps,
zero rise initiation and zero standing rows reached. Rise duration was correctly
reported inactive; it was not optimized. Coordinate refinement selected the
coupled seed; foot-first has only two sampled seeds, not a continuous exhaustive
optimization. This bounded family search cannot prove mechanical impossibility.

Including A's interrupted evaluation, the main research consumed 30 evaluations
(29 completed plus one interrupted) and approximately 167.398 s of the original
48 / 600 s limit. Compilation, setup/preflights, cache I/O and independent
validation are separate costs, not concealed inside this search figure.

The score-selected B program uses pivot 0.15 s, recenter 0.60 s, rise 2 s and
zero waypoint offsets. It fails at 1.50833 s after a persistent head load of
103.74 N, with COM height 0.20755 m and forward velocity 1.19072 m/s. Its lower
unexpected-impact penalty is not a successful balance improvement. A fresh
uncached repeat reproduces commands, every saved physical state, final state
and failure exactly (1.840 s extra validation). A separate cache check reuses
all 38 queries with zero new search evaluations and independently repeats the
same failure again (3.145 s cache search, 2.339 s fresh validation).

All B runs preserve installed cap sums <=760 Nm and individual anatomical
ratios <=1. The owner-allocation A/B comparison is justified by actual clipping:
at 50 ms inherited left forefoot demand is -104.49 Nm under a 4.75 Nm cap,
while unused proximal capacity remains; targets have changed owner but prepared
allocation still reflects ordinary control. Same program with owner allocation
moves initial COM velocity farther backward (-0.066 vs -0.025 m/s at 50 ms),
yet forward velocity at 500 ms is worse (+0.339 vs +0.202 m/s). Contacts,
acceleration, requests/caps and delivered native impulses are separately saved
in `allocation-contact-comparison.json`. Desired demand is not delivered torque.
The final f32 downward cap repair is represented by `capSumNm`; per-axis request
telemetry is emitted before that <=1e-3 Nm rounding repair.

Interpolation and allocation are lightweight: for the selected program command
cost is median 31.179 microseconds, p95 137.688 microseconds, max 507.884
microseconds, across 362 substeps. This excludes full simulation, observation,
report serialization and OS scheduling guarantees. The entire case costs
1.811 s wall time for 1.508 s physical time. Cheap command generation has been
shown; successful interactive recovery has not.

## Retention and next physical blocker

The offline bench is retained under cfg(test) only. Its context is absent in
ordinary tests and production; no rejected program becomes a recovery recipe.
Failed candidates live solely in evidence/cache files. No new production control
hook or qualification/default setting is enabled. Lint-only cleanup after B is
separately archived; B evidence always refers to its exact archived source and
binary, never silently to the newer source fingerprint.

The tested foot-only commands do not sustain pivot/recentering. They acquire
feet contacts without reducing whole-body forward motion enough to reach the
original all-body stability/capture gate. A stronger trajectory representation
with certified dynamic support, or a deliberately planned intermediate contact,
is needed to make further progress. Neither a necessary hand/knee support nor a
feasible feet-only route has been established. Do not substitute accidental
head/torso impact for planned hand/knee support or launch another blind gain sweep.

The two trajectory/allocation unit tests, 29 recovery tests and four measurement
tests pass on evaluated B. Full repository tests were not claimed: the known
hybrid articulation and four TypeScript failures on the witness remain outside
this work. Final lint/format checks and their precise source are recorded in
`HANDOFF.md` and the portable archive.

## Reproduction and provenance limits

Use Rust 1.89.0 with the pinned Cargo.lock, canonical model and patched Rapier.
The portable archive includes complete retained Rust source (excluding target),
A/B source snapshots, exact native executables and manifests, static-plan input,
witness executable/manifest and deterministic raw reports. `RUN-METADATA.json`
records commands and costs. Use absolute `LH_OFFLINE_PLAN` and `LH_OFFLINE_OUT`.
Cache reuse requires the same source fingerprint AND externally verified binary;
the FNV key alone is not a cryptographic whole-build provenance guarantee.

The old technical-B preflight source/executable were overwritten before archival;
its exact trace repeat is not a reproducible archived source/binary pair.
Technical-A had a path error before physical evaluation. Neither is promoted to
main campaign evidence. The witness is verified from its original binary/hash
manifest; its exact build-source provenance is not newly asserted here.
