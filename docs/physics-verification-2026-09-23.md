# Physics verification — 2026-09-23

Historical source-scoped verification. See [current status](status.md) for subsequent checkpoints and [evidence availability](evidence.md) for local artifact limitations. This document does not report the current remote CI state.

**Physics acceptance fails** on commit
`44fc25437d841151c704895c1fb92de4c2e468d0`. The earlier 8/10 balance result at
ticks 122/139 belongs to an older checkpoint. The committed controller now has
14 balance tests and fails at ticks 169/130.

## Verified results

| Gate | Result | Exit |
| --- | --- | --- |
| Vinext production build | Pass; 32.9 seconds | 0 |
| TypeScript | Pass | 0 |
| Balance controller | 12/14 pass; slow pull falls at tick 169, reversal at tick 130 | 1 |
| Focused balance, diagnostics, landing and leg dynamics | 35/37 pass; only the same two balance tests fail | 1 |
| Balance probe | Both scenarios enter a physical fall | 1 |
| Complete application test suite | 273/280 pass | 1 |
| Full physics harness in GitHub Actions | 6/63 pass; all 63 scenarios executed in 47 minutes 53 seconds | 1 |
| Local parallel physics harness | 6/63 pass; all case outcomes match CI; see aggregation recovery below | 1 |
| Five-cycle protocol | First cycle times out; zero stable returns | 1 |

The application suite's other five failures are:

- `anatomy.test.mjs`: the relaxed right elbow enters the torso and a planted
  foot does not preserve its yaw.
- `joint-motors.test.mjs`: the support-conditioned impulse prediction differs
  from the measured response.
- `native-fixed-streams.test.mjs`: the stored left shoulder-girdle rotation
  differs from the current assembly.
- `protocol-striker.test.mjs`: the next accepted click does not produce its
  own contact.

## Physical failures

The slow-pull probe commits a fall for `capture-instability` at tick 169. The
reversal probe commits a `pelvis-height` fall at tick 130, with pelvis height
0.5465 m. Both retain finite state, Rapier ownership, and connected joints.

Neutral standing fails at all three tested headings. At heading zero the
pelvis drifts 0.0922 m against a 0.03 m limit, and the planted feet drift
0.1170 m against a 0.01 m limit. At heading +pi/3 the character enters recovery.
Several pull and landed-pose scenarios also exceed the self-penetration limit
or fail to recover within 25 simulated seconds.

The independent five-cycle protocol stops after its first attempted cycle.
It records no stable return within 25 seconds after the last impact,
0.05349 m maximum self-penetration against a 0.005 m limit, and 1.25926 rad
maximum joint-limit error against a 0.06 rad limit. This does not constitute
five completed cycles.

## CI diagnosis and workflow change

[GitHub run 35905730974](https://github.com/berenif/LaboratoireHumain/actions/runs/35905730974)
passed the static build, static-site verification, TypeScript, and recovery
regressions. Its physics step ran from 18:55:52 to 19:43:45 UTC, finishing with
6 passing and 57 failing scenarios, exit 1. Pages upload and deployment were
skipped. The completed log accounts for all 63 expected scenarios exactly once.
The original workflow selects the default sequential harness,
does not enable within-scenario progress, and sets no explicit step or job
timeout. Local recovery scenarios take several minutes each; their progress
advances through the simulation. The completed CI log confirms a full run
ending with unmet acceptance assertions.

The local Pages workflow change adds an early focused balance gate, enables
two physics workers and progress every five simulated seconds, limits the
physics step to 45 minutes and the build job to 60 minutes, and uploads logs,
source identity and completed scenario reports even on failure. Bash pipefail
preserves test failures through `tee`. Failed gates still block Pages upload
and deployment. The YAML parses successfully and passes `git diff --check`.

These workflow changes are local and have not been pushed. The completed
GitHub job evaluated the original committed workflow. No physics
implementation, threshold, test assertion, or fixture
was changed for this verification.

## Reproduction and evidence

Local commands used the pinned runtime
`C:\Users\flori\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe`
(Node 24.19.0 on Windows); the inspected GitHub job uses Node 22.13.0 on Linux.
Children use the same executable as their parent. The complete unit suite ran
with `--test-concurrency=2`. The full physics run uses four scenario workers,
`PHYSICS_PROGRESS=1`, and no scenario filter or list-only setting. Worker
isolation changes scheduling, not scenario inputs or acceptance thresholds.

Run-specific logs, command records and source fingerprints are preserved in
the ignored local directory `evidence/physics-verification-20260923/`:

- `build.log`, `typecheck.log`, `focused.log`, `tests.log`, `probe.log` and
  `protocol.log`, with matching command/fingerprint JSON records;
- `balance-trace/`, including the measured fall reasons;
- `physics-workers/`, including each completed case's report and log;
- `physics-results.json` and `physics-trace.ndjson.gz`, containing the
  reconstructed aggregate report and all 14,727 trace rows;
- `physics-execution.json`, recording the discovered scenario list and source
  fingerprints;
- `summary.json`, `ci-status-initial.json`, `ci-physics-results.json` and
  `ci-physics.log`. The CI result JSON is extracted from the completed job log.

All 63 local case reports completed and match CI's pass/fail outcomes exactly.
The original aggregation then exhausted the workspace disk while duplicating
the worker traces, and its command record could not be saved. The incomplete
aggregate trace was removed. The aggregate report was reconstructed from the
63 retained worker reports, checking unique coverage, consistent metadata and
unchanged physics sources. Their traces were compressed to a 157 MB gzip file;
decompressing it reproduced the SHA-256 digest of the original concatenated
rows before the redundant uncompressed files were removed.

`physics-finalization.json` records these checks, and `trace-storage.json`
records the cleanup. The original `physics-execution.json` is preserved with
`completed: false` to distinguish the interrupted aggregation from this
explicit recovery. The disk error does not affect the complete CI result or
the agreement of all 63 independently saved local case results. Only this
run's incomplete and redundant generated traces were removed.
