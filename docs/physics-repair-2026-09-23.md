# Physics repair checkpoint — 2026-09-23

Historical report with September 24 appendices. See [current status](status.md) for precedence and [evidence availability](evidence.md) for recovered originals. Missing report/log links are retained as paths marked unavailable; recorded counts have not been promoted to newly verified results.

**Acceptance still fails.** This uncommitted repair starts from
`44fc25437d841151c704895c1fb92de4c2e468d0`. The prepared Pages workflow changes,
acceptance documentation, and original verification evidence remain preserved.
No assertion, acceptance threshold, actuator ceiling, or fixture was changed.
No commit, push, or deployment was performed.

## Retained correction

`src/character/support-loads.ts` now uses mean-value coordinates for continuous,
rotation-invariant allocation over the measured convex contact hull. Previously,
it averaged the barycentric representations of every triangle containing the
requested pressure. Crossing a diagonal changed the number of representations
abruptly. A two-micrometre pressure change in an asymmetric five-point patch
changed individual load shares by several percentage points. This introduces
finite joint-torque jumps for arbitrarily small changes in control intent.

The replacement preserves nonnegative normalized weights, measured contact
owners and heights, and the requested pressure moment. Vertex and edge cases
use their limiting point/linear weights. It also replaces the cubic enumeration
of triangles with a linear calculation after hull construction. Two new
regressions check continuity and force/moment conservation through rotated and
translated patches, including boundaries.

`src/character/physics-settings.ts` increases internal PGS iterations from 4 to
20; timestep, outer iterations, collision tolerances, motor capacities and
physical fall guards remain unchanged. The allocator alone introduced failures
in both cross-body chest-contact regressions. The combined correction passes
both, as well as the other body-coherence tests. Neutral experiments also show
reduced sole chatter and substantially less heading-dependent drift. Increased
iterations do not establish complete convergence or physics acceptance.

The absolute **52% of whole-body weight** retained-load gate, genuine unloading,
loaded touchdown, continuous Rapier ownership and zero direct pelvis assistance
remain unchanged. Planned force caps and pressure/force consistency are retained.

## Causal findings and remaining failures

The original traces were inspected before production changes. They show knee
targets following the descending measured pelvis until ankle targets reach
their limits. Flat-floor stance targets also adopt measured foot displacement.
These feedback choices permit settling errors to accumulate; however, freezing
stance targets by itself caused earlier neutral falls and was rejected.

Separate runtime ablations tried a standing-height reference, consistent local
arm targets, fixed stance targets, additional solver convergence, and bounded
contact-moment intent. Some substantially reduced neutral motion or delayed a
fall. None passed the full relevant physical contract. These experiments are
kept under the ignored evidence directory; none of their posture, gain,
step-trigger, knee-plane or contact-moment changes remains in production code.

On the retained source, both balance failures are now capture-instability:

| Scenario | Fall tick | Pelvis height | Completed steps |
| --- | ---: | ---: | ---: |
| Slow pull | 177 | 0.920572 m | 1 |
| Planted reversal | 229 | 0.920956 m | 2 |

Both remain finite, connected and Rapier-owned. Immediately before the slow
fall, requested horizontal X force is -248.65 N but the compatible allocation
is +148.97 N. Before reversal falls, requested Z force is +218.05 N but the
compatible allocation is -141.78 N. COM has escaped the measured support
region; the zero-centroidal-moment allocation cannot supply the requested
braking, and the landing forecast rejects the next candidate for
`capture-support`. These are physical control failures, not reasons to relax
the fall guards or claim a completed step. The new traces preserve the lead-up
needed to investigate transfer and landing timing.

## Verification on retained source

All commands use the pinned Node executable specified in the acceptance
ledger; children use `process.execPath`. Source fingerprints are unchanged
before/after each final command.

| Gate | Result | Exit |
| --- | --- | ---: |
| Focused balance, diagnostics, landing, leg dynamics/frame, recovery/support, body physics/coherence, contacts, allocation and joint motors | **107/110 pass** | 1 |
| Balance subset within focused run | **12/14 pass**, failures at 177/229 | 1 |
| Balance transition probe | Both scenarios fall | 1 |
| Official `idle-30-seconds` physics scenario | **0/1 scenario passes**; all three headings fail limits | 1 |
| TypeScript | Pass | 0 |
| ESLint | Pass; six existing unused-variable warnings, no errors | 0 |
| `git diff --check` | Pass | 0 |

The third focused failure is the existing support-conditioned impulse
prediction: predicted response 13.469373 versus measured 10.892522. The two
new allocation regressions pass. All body-coherence tests, including both
cross-body contact regressions, pass in the final focused run.

Neutral results use the unchanged 2-second settling and 30-second observation:

| Heading | Pelvis drift | Planted-foot drift | Peak linear speed | Peak angular speed | End state / steps |
| --- | ---: | ---: | ---: | ---: | --- |
| 0 | 0.051197 m | 0.035273 m | 0.676300 m/s | 1.533507 rad/s | upright / 0 |
| +pi/3 | 0.040666 m | 0.039638 m | 0.678940 m/s | 1.542940 rad/s | upright / 0 |
| -pi/4 | 0.056833 m | 0.091682 m | 0.687004 m/s | 2.690401 rad/s | upright / 0 |
| Required maximum | 0.030000 m | 0.010000 m | 0.100000 m/s | 0.500000 rad/s | upright/reacting / 0 |

The full application suite, all 63 physics scenarios, five-cycle protocol,
build, browser and Pages gates were **not rerun** because focused acceptance
still fails. Earlier results remain evidence for their original source only.
Native fixtures were not refreshed. Next work must resolve neutral stance and
measured transfer/landing failures before those expensive gates are reopened.

## Evidence and source identity

Evidence is in `evidence/physics-repair-20260923/`:

- `summary.json`: exact command counts/exits, neutral metrics, fall reasons,
  source manifest and archive verification;
- `final-*.json` and `final-*.log`: commands, arguments, source fingerprints,
  timings and native exits;
- `final-idle-results.json`, `final-idle-trace.ndjson.gz`, and
  `final-balance-trace/*.jsonl.gz`: retained-source physical evidence;
- `existing-analysis.log`, ablation logs and experimental preload/diagnostic
  scripts: investigation, not acceptance evidence;
- `retained-working-tree.patch`: working-tree diff captured after verification.

The tested-source manifest in `summary.json` contains SHA-256 values for source,
scripts, tests and package files. Its own digest is
`0360b023dc8c867961c74ff492bc664e3af6736c27bb1eb884823ea6414d6bb6`.
After verification, two line endings in the reverted `pose.ts` experiment were
restored to its exact original verification hash; normalized source content
is identical. `postVerificationEolRestoration` records both hashes and this
check. The final byte-level source manifest digest is
`d53905ea64ac75596ca1a792f5d5865244c51f492addeb6e23ed5d28e148f2ce`.
The two changed physics files have these exact SHA-256 values:

- `support-loads.ts`: `9637a126ffa827f8523cec06013552635f5e154ecf40927783ace4bc0f2d1a2a`
- `physics-settings.ts`: `be8e2ae2c49525460a5e1f80fd9f76a53a1612723066cd1845cbea810764bd07`

New compressed traces passed decompressed SHA-256 round-trip checks. Their raw
copies are also retained. The original verification archive and its deliberately
incomplete execution record were not altered. Free space was checked before
runs (about 762 MB initially and 687 MB before final compression); no full
trace aggregation or deletion of previous evidence was attempted.

## 2026-09-24 continuation — diagnosis, still not accepted

No production correction was retained in this continuation. The transfer
trace was instrumented without changing production source. In both balance
scenarios, the active landing forecast repeatedly returns `capture-support`
while the alternate candidate is feasible with the opposite, currently loaded
sole. The runtime study recorded nine candidate replacements before the first
uninterrupted swing in each scenario. At slow-pull 0.183 s, for example, the
right-foot candidate forecasts capture outside its landing support while the
left sole carries only 46 N and the right carries 642 N; switching to a
left-foot step is the only feasible candidate. By 0.250 s the loads have
shifted to 483 N left / 153 N right and the feasible candidate switches back.
Each replacement correctly starts fresh transfer and readiness ages. This is
a measured support-transfer problem, not evidence for reusing the rejected
general transfer debounce. After touchdown, the next forecast again falls
outside reachable support. The trace and source manifest are in
`evidence/physics-repair-20260923/continued-transfer-diagnosis-retry.jsonl`
and `continued-transfer-diagnosis-retry.json` (pinned Node command, exit 0).

The neutral trace shows a separate stance feedback loop. On flat floor, every
loaded foot's measured position is copied into `this.feet`; with no active
step, the support target and root target are then based on those same moving
foot targets in `src/character/BalanceController.ts`. At heading 0, from t=2.0
to 31.9 s, the pelvis moves about 0.051 m while both heel/toe pairs translate
about 0.034 m in the same horizontal direction; their measured normal contacts
remain loaded. The retained trace has the per-frame contact and pose data.
The earlier fixed-anchor experiment reduced root drift at two headings but
still had 0.0465 / 0.0393 m planted-foot drift and fell at -pi/4 after frame
1326. A completed runtime-only 25%/50% target-follow sweep also failed the
drift limits at all three headings; its six results and hashes are in
`continued-stance-anchor-sweep.json` and `.jsonl` (exit 0). The damping-only
ablation still does not establish acceptance.

A temporary landing-planner experiment allowed a crossover only when the
landing sole contained the forecast capture point. Slow-pull still fell at
tick 177; reversal fell at tick 250 instead of 229, and two landing-planner
regressions appeared. Its focused run was 105/110, exit 1. The changed
`landing-plan.ts` hash was
`dafb9f89c414c5ec70ba356f56d62033dc87b666967c16bdfa445bb97b1e6dd4`; the
experiment was reverted, and the retained hash is again
`b289a3cec79e34ffbbff049751f6e8253acb2e73988ea5764bf14100aaac1b6f`.
The run record is `continued-focused-20260924.json`.

### Retained-source verification

The focused command was run with the pinned Node executable and
`--test-concurrency=2` over the 12 focused files listed in
`final-focused-20260924.json`. It passes **107/110**, exit **1**: the two
balance scenarios still fail at ticks **177/229**, and the support-conditioned
joint impulse prediction remains **13.469373 versus 10.892522**. Source hashes
before and after the command match in that JSON record.

The balance probe was rerun with a new trace directory so the retained raw and
compressed traces were not overwritten:

```text
BALANCE_TRACE_DIR=evidence/physics-repair-20260923/continued-balance-trace-20260924 C:\Users\flori\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe scripts/probe-balance-fall-transition.mjs
```

It exits **1** with the same capture-instability falls at ticks **177/229**;
the measured states remain finite, connected, and Rapier-owned. Its native
log, command, exit, and before/after source hashes are in
`final-probe-20260924.log` and `final-probe-20260924.json`.

The retained `BalanceController.ts` SHA-256 is still
`3727b0a85c2da2aade785b2cfb89903d6376116e31d2f993afa8dc7d933cdb20`.
The coherent allocator and 20-iteration solver settings remain unchanged. No
acceptance threshold, assertion, actuator ceiling, fall guard, native fixture,
or physical ownership rule was changed. The focused gate still fails, so the
full physics/protocol/browser gates and fixture refresh remain gated.

## 2026-09-24 completion attempt — unresolved balance acceptance

The final retained-source focused command expanded the original 110-test
selection with the requested landing-capture, transfer-prediction, and
joint-coordinate suites. It completed **140/142**, exit **1**. The only failures
are still the full slow-pull/release/reversal sequence (falls at tick **216**)
and planted reversal (falls at tick **343**). The test log and manifest record
the pinned Node command, exit code, and matching before/after source hashes:
`focused-completion-retained-source.log` (`evidence/physics-repair-20260924-completion/focused-completion-retained-source.log`; unavailable),
`focused-completion-retained-source.json` (`evidence/physics-repair-20260924-completion/focused-completion-retained-source.json`; unavailable).
The source was stable during that run; the recorded SHA-256 values include
`BalanceController.ts` `329d2f40cfde8329cc0fbef42aa867ed8fe8b36170b687df1d2bc0377b3bad0a`,
`EmbodiedCharacter.ts` `e76e52091d901e561936e27393532ea06d53c5328623f20385cca45126826f02`,
and `support-loads.ts` `9637a126ffa827f8523cec06013552635f5e154ecf40927783ace4bc0f2d1a2a`.

The fresh final balance probe exits **1** with the same failures. At the
slow-pull failure frame, the candidate is invalid (`travel:83,reach:161,
leg-separation:1`), the forecast is pressure-infeasible, the measured left
sole is the only available support, and the allocator's horizontal force is
opposite the requested force (requested +161.9 N, allocated −161.9 N; reported
pressure-force residual 450.4 Nm). The left thigh is saturated at 3.074 times
its command limit. The reversal trace ends in the existing physical
`pelvis-height` fall guard at 0.5522 m; ownership remains `rapier-dynamic` and
the assembly remains finite and connected. Probe command, exit, source
fingerprints, and fresh trace paths are recorded in
`balance-probe-final-source.log` (`evidence/physics-repair-20260924-completion/balance-probe-final-source.log`; unavailable)
and `balance-probe-final-source.json` (`evidence/physics-repair-20260924-completion/balance-probe-final-source.json`; unavailable).
The full measured-load, commanded-distribution, posture, requested-versus-
observed motor response, saturation, candidate, readiness, and forecast trace
is `C:\Users\flori\AppData\Local\Temp\physics-repair-20260924-retained-final\completion-retained-source-final.jsonl`;
its manifest is beside it. The separately captured per-frame failure probe
traces are under
`C:\Users\flori\AppData\Local\Temp\physics-repair-20260924-balance-probe-final`.

The official `idle-30-seconds` scenario ran the required 2-second settling
and 30-second observation at all three headings, with 1,921 samples per run.
It exits **1** (0/1 scenarios accepted). Every run stayed upright with zero
steps, but every run exceeded all four stationary thresholds:

| Heading | Pelvis drift (limit 0.03 m) | Foot drift (limit 0.01 m) | Max linear speed (limit 0.10 m/s) | Max angular speed (limit 0.50 rad/s) |
| --- | ---: | ---: | ---: | ---: |
| 0 | 0.0512 m | 0.0353 m | 0.6763 m/s | 1.5335 rad/s |
| +π/3 | 0.0407 m | 0.0396 m | 0.6789 m/s | 1.5429 rad/s |
| −π/4 | 0.0568 m | 0.0917 m | 0.6870 m/s | 2.6904 rad/s |

The exact command, heading metrics, exit and stable before/after source
fingerprints are in
`official-idle-final-source-rerun.log` (`evidence/physics-repair-20260924-completion/official-idle-final-source-rerun.log`; unavailable)
and `official-idle-final-source-rerun.json` (`evidence/physics-repair-20260924-completion/official-idle-final-source-rerun.json`; unavailable).
Its raw scenario output is under
`C:\Users\flori\AppData\Local\Temp\physics-repair-20260924-official-idle-final-rerun`.

The original asserted hip/ankle impulse tolerance is unchanged and the focused
joint-motor tests pass after the paired fixtures are velocity-cleared together
after pose preparation and measured loaded left hindfoot/forefoot patches are
used for support conditioning. The expanded 36-case paired sweep records one
15.23% right-thigh response mismatch at the smallest 0.0005 Nm·s impulse; this
remains a diagnostic outlier, not a tolerance adjustment. The paired sweep
record and trace are
`joint-response-paired-static-probe.json` (`evidence/physics-repair-20260924-completion/joint-response-paired-static-probe.json`; unavailable)
and `joint-response-paired-static-probe.jsonl` (`evidence/physics-repair-20260924-completion/joint-response-paired-static-probe.jsonl`; unavailable).

TypeScript passes (exit **0**). ESLint exits **0** with six warnings and no
errors; the captured output is
`typecheck-final-source.log` (`evidence/physics-repair-20260924-completion/typecheck-final-source.log`; unavailable)
and `eslint-final-source.log` (`evidence/physics-repair-20260924-completion/eslint-final-source.log`; unavailable).
`git diff --check` exits **0** after both report updates. No correction to the
stance-reference/transfer controller was retained because the tested posture
target experiment failed balance tests, the two complete balance sequences,
and idle acceptance. The retained allocator, solver settings, thresholds,
torque ceilings, physical fall guards, Rapier ownership, and native fixtures
remain unchanged. The focused repair is **not accepted**; full application,
physics, protocol, build/browser, and Pages gates remain a separate phase and
were not run. No commit, push, or deployment was made.

A final combined posture/reference candidate explicitly held both foot pose
targets and the neutral COM target between qualified support transitions.
It was rejected: the focused balance/controller plus body-coherence run passed
27/30 and exited **1** (slow-pull fell at tick 259, reversal at tick 455, and
left-hand chest drag reached 0.03035 m joint separation against the unchanged
0.01 m limit). The candidate idle harness also failed all three headings and
measured 0.074518 m self-penetration; its result writer could not save its JSON
because its fresh output directory was missing, so that run is diagnostic only.
Commands, exits, source fingerprints, and this rejection are captured in
`stance-reference-transition-candidate.json` (`evidence/physics-repair-20260924-completion/stance-reference-transition-candidate.json`; unavailable)
and log (`evidence/physics-repair-20260924-completion/stance-reference-transition-candidate.log`; unavailable).
The candidate was reverted. The retained `BalanceController.ts` again matches
HEAD's Git blob `38b6c20604cb6578b88dddf42dcdd94723bded9c` and the final focused
run fingerprint above.

After that revert, the expanded focused selection was rerun on the restored
source and again passed **140/142**, exit **1**, with only the slow-pull tick
216 and planted-reversal tick 343 falls. The final balance probe again exits
**1** on the same source; TypeScript exits **0**, ESLint exits **0** with six
warnings, and `git diff --check` exits **0** after these report edits. The
combined final command and source fingerprints, including the fresh balance
probe trace directory, are in
`final-restored-source-checks.json` (`evidence/physics-repair-20260924-completion/final-restored-source-checks.json`; unavailable).
