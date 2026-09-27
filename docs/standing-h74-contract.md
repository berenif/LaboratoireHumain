# H74-v1 — bounded coordinated standing investigation

2026-09-27. **Rejected at feasibility; quiet standing remains unaccepted.**
The three predeclared references failed. No local-return, short-screen,
official-standing, sustained-standing or disturbance pass follows. No MPC or
validated backup region was introduced. This is a finite negative experiment,
not evidence that standing is impossible.

## Implementation and frozen scope

The [numeric manifest](experiments/standing-h74-v1.json) was saved before the
trials. It declares three reference trials at heading zero, pelvis forward
offsets 0 / −5 / +5 mm, unchanged 2 s entry and 4 s reference hold, four local
impulses, six H42 fixtures, 7,320 maximum feasibility steps and 900 s wall time.
Later gates declare 120 s sustained observations, two repetitions, held-out
headings/inputs, an 8 ms feedback deadline, 1,600 work units, a 16.667 ms full
update budget, zero permitted deadline misses, 768 MiB process RSS and 64 MiB
post-warm-up growth. Thresholds were not tuned after the results.

[`CoordinatedStandingController.ts`](../src/character/CoordinatedStandingController.ts)
owns a fixed reference and combines anatomical posture feedback, fixed-foot
virtual work, pelvis posture/position feedback, all-body motion damping and
motor-effort regularization in one command. It is local feedback, not a
constrained optimizer or a predictive model. The contact-load feedforward and
native ForceBased motors remain in the existing physical path. There are no
world copies or searches in the runtime controller, no new actuators and no
body-state setters. References are serialized and checked for change each tick.

The candidate is explicitly selected through `coordinatedStandingOptions` by
both the harness and the application. The browser option is
`?standingCandidate=h74-v1`, using the declared flat-floor world. Room and
playground solver variants are excluded from this experiment. Ordinary startup
is unchanged. `characterFrame` shares the fixed-update/snapshot order, and the
candidate runs before native motors, Rapier integration, pose/contact observation
and the original fall guards. Default behavior retains the existing controller.

Timeout, infeasibility, invalid contacts/model, invalid backup and region exit
latch a transition to existing balance/fall/recovery behavior in the same update.
There is no command cache or validated backup to reuse. The transition policy
is not claimed to preserve quiet standing. Its native actuation and continued
integration are tested. Missing an optional headroom reserve does not reject a
capped command; reference-quality checks remain separate from command selection.

## Reproduce and inspect

Run from the repository root with the installed Node runtime:

```powershell
node scripts/standing-acceptance.mjs
```

This creates a fresh timestamped evidence directory, freezes source/runtime/
dependency hashes, verifies and preserves the H71 archive's original H66/H69
inputs, runs policy checks and injected failures, then investigates feasibility.
Physical simulations run serially in fresh processes with a 384 MiB Node
old-space setting. That setting is not a total-memory bound. An optional sole
argument selects a fresh output directory; existing attempts are never replaced.
`test:standing` is the package-script alias.

Dependent stages are saved regressions, three 2+10 s screens, three unchanged
2+30 s tests, structural/integration regressions, and the frozen sustained,
held-out, latency and memory evaluations. A failed prerequisite leaves later
stages `incomplete` and the operation exits nonzero. The actual browser/process-
memory gate also remains explicitly incomplete; Node measurements and source
parity cannot close it. The runner cannot declare overall success without that
application evidence.

The recorded operation used `evidence/standing-h74-v1/attempt-01`, exited **1**
in 43.366 s, and verified unchanged source during execution. The subsequent
native analysis verifies that every `src/` file and the dependency lockfile
still match the attempt. Original H66/H69 evidence was preserved and checked,
not rerun as a new passing candidate stage. H73 was not evaluated or modified.

First-failure native replay:

```powershell
node --import tsx scripts/standing-acceptance.mjs --replay evidence/standing-h74-v1/attempt-01/reference-0
```

The replay checks the artifact inventory and installed dependency hashes,
restores the snapshot taken **after motor configuration, immediately before
world.step**, restores collision exclusions/hooks, and compares all 25 complete
output body states exactly. The same verification runs automatically for each
retained failure. Each capture includes controller inputs and reference state,
native motor calls, contacts, external inputs, scenario/tick, source/runtime
fingerprints, commands, output states and a streamed reconstructing history.
Logs and completed artifacts survive an aborted child; an absent final capture
or report is incomplete, never a pass. Diagnostics keep one native step in
memory and stream history rather than accumulating it.

## Recorded results

| Reference offset | First failed tick / time | Failed measurement | Other measurements at failure |
| --- | --- | --- | --- |
| 0 mm | 133 / 2.217 s | Right forefoot angular speed **0.501134 rad/s**, exceeding both the 0.4 reference reserve and the official 0.5 limit | Peak linear 0.052179 m/s; reference foot error 4.407 mm; requested headroom 73.79% |
| −5 mm | 120 / 2.000 s | Reference foot error **5.154 mm**, against 5 mm | Peak linear 0.020043 m/s; angular 0.056990 rad/s; requested headroom 71.46% |
| +5 mm | 123 / 2.050 s | Left forefoot angular speed **0.459446 rad/s**, against the 0.4 reference reserve | Peak linear 0.024764 m/s; reference foot error 4.495 mm; requested headroom 68.56% |

The latter two are failures of the predeclared reference gate, not claimed
official speed/drift failures. All runs still have two planted feet, zero steps
and upright state at their stopping point. They do not complete the reference
hold. Headroom describes requested native motor effort, not measured delivered
torque. Native inspection independently verifies all **46** anatomical motor
axis configurations in each pre-step failure snapshot.

The three histories integrate 376 physical steps; the separate H42 fixtures
integrate 4,320. The runner allocates 5,400 of the maximum 7,320 feasibility
steps because early failures terminate reference trials. All three reference
trials are consumed, so there is no further tuning in v1. Local trials are
blocked by the failed reference. Six injected failures each run three steps
as separate policy regressions. Their first failed command steps, and all
three reference failures, replay exactly for 25 bodies: **nine captures**.

Invariant monitoring is active throughout startup and observation. Across the
three reference histories, maximum anchor separation is 0.03938 mm, independent
floor penetration 2.16434 mm, self-penetration zero and joint-limit error
0.002409 rad. No runtime state setter or body/collider replacement is observed;
requested motor vector caps and measured/planned contact checks pass. This
does not replace the later complete structural/recovery gates.

For the central reference's **13 observed steps only**, pelvis maximum
excursion is 7.349 mm; the largest foot excursion is 1.104 mm. Pelvis vertical
range is 0.150 mm, and maximum foot vertical range is 0.620 mm. Full per-foot
endpoint/excursion/vertical measurements are retained in each report. The −5 mm
trial has no post-settling observation, so its vertical ranges are null, not a
successful zero-motion interval.

## Mechanism check and next diagnostic

H42's six unchanged rigid fixtures reproduce the recorded representation
difference. Fixed impulse assemblies end near pelvis height 0.1031 m, with
maximum foot drift 0.511238 / 0.268614 / 0.521225 m. Fixed multibody assemblies
end near 0.9820 m, with drift 0.000471 / 0.000513 / 0.000349 m. These fixtures
lock anatomical motion and cannot pass articulated acceptance.

At the central articulated failure, right-forefoot native angular speed changes
from 0.126119 to 0.501134 rad/s. Its pose-derived angular rate is 0.514597 rad/s;
the vector difference from the final native rate is 0.150542 rad/s. Bilateral
sole contact remains measured, and the native forefoot motor retains its 35 Nm
ceiling. These observations do not isolate controller, contact or constraint
effects, and do not show actuator saturation as the explanation.

The next concrete diagnostic is to inspect the native angular constraint rows
on the saved pre-step states: compare their coordinates and Jacobians with the
application's quaternion coordinates, then test signed capped single-axis
perturbations under a **new frozen manifest**. H42 supports investigating
representation; it does not establish the cause of the articulated failure or
justify changing DOFs/limits. MPC remains blocked by the unestablished reference.

## Runtime, memory and verification

The subsequent [H75-v1 diagnostic](standing-h75-contract.md) preserves these
results and executes a separately frozen set of 342 copied motor probes. Its
native row reconstruction remains incomplete; H74-v1 is neither extended nor
retuned, and no operating/backup region or controller repair is established.

Target: Intel Core i7-8750H, Windows 10.0.26200 x64, Node v24.19.0, Rapier 0.20.0.
Each run records PID, command flags, executable/dependency hashes and timing
percentiles. The local feedback used 160 work units per update, against 1,600.

| Reference | Controller p50 / p95 / p99 / max, ms | Instrumented complete update p50 / p95 / p99 / max, ms | Sampled peak RSS |
| --- | --- | --- | --- |
| 0 mm | 0.455 / 1.300 / 1.765 / 2.054 | 14.435 / 26.975 / 32.428 / 77.869 | 113.87 MiB |
| −5 mm | 0.446 / 1.696 / 2.353 / 2.832 | 14.941 / 31.941 / 46.757 / 72.580 | 112.79 MiB |
| +5 mm | 0.438 / 1.483 / 2.167 / 2.499 | 13.739 / 31.259 / 47.412 / 86.636 | 112.88 MiB |

The central trial misses the 16.667 ms complete-update deadline on 6 of its 13
post-settling updates. Controller deadline misses are zero in these short
histories. Capturing native snapshots and body states adds diagnostic overhead;
these are neither isolated browser benchmarks nor interactive-performance
acceptance. WASM linear memory is measured separately at 2.25 MiB; final JS heap
is 28.98–32.23 MiB and external allocations 13.39–14.05 MiB. These allocations
overlap RSS and are not added to it. The short histories cannot validate the
120 s soak or the allowed growth bound.

- **Pass:** five candidate tests, including 30-step exact disabled compatibility
  against archived original production source; six injected transition cases;
  nine exact native failure replays; 16 historical archive originals verified.
- **Pass:** all 69 unchanged H51 focused tests plus 12 DemoRuntime tests (81),
  full typecheck, modified-file lint with only the pre-existing unused `RIGHT`
  warning. These checks are separate from staged physical acceptance.
- **Fail:** reference feasibility, and the central short observed interval's
  official angular-speed bound.
- **Incomplete:** local return, current-candidate saved-failure stage, three
  screens, official standing, complete structural/integration acceptance,
  sustained standing, held-out disturbances, soak and browser performance.

The operating envelope is **unvalidated**. Quiet standing, transfer, stepping,
recovery and release checkboxes remain open.

Local evidence (intentionally excluded from Git by the existing policy):
[attempt and stage report](../evidence/standing-h74-v1/attempt-01/report.json),
[frozen run manifest](../evidence/standing-h74-v1/attempt-01/run-manifest.json),
[artifact inventory](../evidence/standing-h74-v1/attempt-01/artifacts.json),
[native mechanism analysis](../evidence/standing-h74-v1/analysis-01/report.json),
[regression log](../evidence/standing-h74-v1/validation-01/regressions.log).
