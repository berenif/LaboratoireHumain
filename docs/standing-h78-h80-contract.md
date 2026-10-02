# H78–H80 bounded contact-force investigation

All three candidates are opt-in. The original startup, default controller, 25 bodies,
geometry, collision rules, solver settings, actuator limits and fall guards remain
unchanged. No standing, support-transfer, terrain, recovery-cycle or release
acceptance follows from the numerical regression tests.

## Frozen candidates

| Candidate | Change | Frozen numerical policy |
| --- | --- | --- |
| H78-v1 | H77 native posture plus measured-contact force allocation under combined motor ceilings | 64 gradient iterations, at most 24 convex-projection sweeps, normalized constraint tolerance 1e-5, convergence tolerance 1e-6, previous-force and motor-effort weights 0.02 |
| H79-v1 | Certified direct solution of the identical quadratic; original constrained fallback | Original constraints, objective, iteration bound and shared 8 ms absolute deadline |
| H80-v1 | Reuse contact mapping columns across axes; sparse constrained operations and reusable buffers | Same mathematical operations, limits, objective and deadline; final authorized candidate |

The manifests and mandatory source locks are
[`standing-h78-v1.json`](experiments/standing-h78-v1.json),
[`standing-h79-v1.json`](experiments/standing-h79-v1.json), and
[`standing-h80-v1.json`](experiments/standing-h80-v1.json), with sibling
`.lock.json` files. Locks include all source/dependency-configuration fingerprints
and the inherited manifest chain. Acceptance captures installed Node/Rapier/WASM
hashes, exit codes, logs and a compressed source snapshot. H78 and H79 source
snapshots preserve their exact implementations even as successors add opt-in paths.

## Implementation boundary

The solver minimizes normalized force/moment error, change from surviving
allocations, and 0.02 motor utilization. It uses measured sole resultants with
nonnegative normal force, friction 1.2, the original total/horizontal force limits,
and posture-plus-contact individual motor ceilings. Shared `contactJointTorque`
maps both allocation and execution. No contact earns a prescribed load merely by
touching down. Lost contacts remove their allocation immediately.

The shared serializable support state carries target, contact revision, previous
allocation, retained side, transfer age, measured readiness and stance revision.
Reversal resets readiness; touchdown/cancellation updates the current target.
Only retained measured load >=52% bodyweight and moving measured load <=25% for
0.10 s can satisfy launch readiness. Inferred sleeping loads cannot qualify.
Forecasting copies this state and cannot advance runtime timers or authorize
launch. Committed swing destinations and touchdown qualification are retained.

The initial forecast is deliberately a held-load model. It assumes no future
increase to the launch threshold. Transfer admission remains disabled until
matched production trajectory validation passes. The validation child reports
0.5 s errors at signed 75%/125% commands and refuses a vacuous readiness-time pass.
It does not establish a validated forecast merely by existing.

Diagnostics distinguish requested force, allocated force, normal-impulse
measurement, moment residual, motor headroom/readback, contact revisions and
transition reasons. Horizontal contact impulse is unavailable through the pinned
API and is reported as unknown. Forecast errors stay explicitly unvalidated until
a matched trajectory produces evidence. Complete serialized controller data
(including Maps and sentinel timers) accompanies native first-failure snapshots.

## Recorded outcome

- Fresh H77 verification reproduces reference failures at ticks 132/120/120 with
  complete controller state. Retained slow-pull/reversal files were hashed and
  inspected; they end falling at 5.083 s and 3.817 s and do not pass their assertions.
- H78 fails all three references at tick 4 on its first measured-contact allocation:
  controller times 8.0923/8.1724/8.0434 ms. Native replay matches all 25 bodies exactly.
- H79 fails at ticks 4/8/8, with controller times 8.4586/8.0949/8.0769 ms. Before the
  latter failures, accepted commands match native readback within 8.60e-7 Nm and
  4.10e-7 Nm. The earliest demonstrated defect remains allocation runtime.
- H80 fails central/+5 mm references at ticks 4/8 on controller times 8.0155/8.468
  ms. The -5 mm reference reaches tick 120, then fails entry/hold with 0.74494 m
  pelvis error, 0.11960 m foot error, 2.9720 m/s linear speed and 2.7743 rad/s
  angular speed. Its final native readback error is 1.55e-6 Nm, within the inherited
  1.90735e-6 Nm tolerance. All three native first-failure replays match 25 bodies
  exactly. There is no validated standing operating range.

In H80's -5 mm trajectory, the first admissible allocation at tick 4 already
differs from the requested horizontal restoration: requested x/z force is
0.6073/10.6546 N; allocated force is -4.5611/-4.4739 N. The moment residual is
(14.9117, 0.2210, -2.3418) Nm. Native commands agree within 2.55e-7 Nm; all four
contacts are still present and motor headroom is 77.36%. This locates the earliest
observed discrepancy at allocation, before native execution. It does not isolate
the complete physical cause or prove which objective term should change. See
`evidence/standing-h80-v1/acceptance-01/earliest-force-divergence.json`.

The final frozen source passes typecheck, lint and production build (exit 0;
lint reports seven warnings).
The applicable serial regression run is 80/82 pass, one failure and one skip
(exit 1). All new candidate, support-state, solver and runner tests pass. The
strong-pull physical-fall assertion also fails on the exact pre-change source
(243 restored files verified against the baseline); it is a retained legacy
failure. The archived-source compatibility check skipped in the standalone suite
passes in acceptance with its archive supplied. Source fingerprints remain
unchanged through testing. No fourth candidate or further tuning is authorized.

Local-return, prediction, transfer, short-screen, official standing, sustained,
held-out and browser acceptance require their preceding physical gates. An
incomplete gate is not a pass. The browser implementation executes real
`DemoRuntime` with Canvas2D and Three/WebGL through installed Edge/CDP, including
the inherited timing, process RSS, WASM and soak checks; its unit tests are not
an actual browser acceptance run.

## Reproduction and evidence

Renderer/runtime files in the shared workspace changed after the checks completed.
Those edits are preserved. `evidence/standing-h80-v1/frozen-workspace` reconstructs
all 354 frozen source/config files and matches H80's lock exactly; its provenance
is in `frozen-workspace-record.json`. The source fingerprint is
`e4b96f48364250872d2ddc3a481e77e78cd3e0a99290871802f80c612b752d2b`.
Run from that verified copy, using a fresh output directory each time:

```powershell
Push-Location O:/LaboratoireHumain/evidence/standing-h80-v1/frozen-workspace
node --preserve-symlinks-main --max-old-space-size=384 scripts/standing-acceptance.mjs O:/LaboratoireHumain/evidence/standing-h80-v1/reproduction-01 --controller=h80-v1
node --preserve-symlinks-main --max-old-space-size=384 scripts/standing-acceptance.mjs --replay O:/LaboratoireHumain/evidence/standing-h80-v1/acceptance-01/reference-1
node scripts/standing-frozen-checks.mjs O:/LaboratoireHumain/evidence/standing-h80-v1/checks-reproduction-01
Pop-Location
```

The main-entry flag is required when invoking this copy through a junction.
The documented replay was executed successfully (exit 0, all 25 bodies exact)
and is retained in `frozen-replay-report-02.json`. An initial invocation without
that flag produced no output and is preserved as incomplete evidence.

Physics is serial. Each candidate retains 900 s feasibility, 3,600 s acceptance,
300 s child timeout and 384 MiB Node old space. Compiler/lint/build checks use the
ordinary project toolchain configuration; a separate compiler attempt at 384 MiB
exhausted its heap and is retained as a tooling limitation. Physics children keep
their original 384 MiB limit.

Evidence roots are `evidence/standing-h78-v1`, `standing-h79-v1`, and
`standing-h80-v1`. H78's first attempt is incomplete from workspace ENOSPC; the
second is incomplete from relocated archive dependency resolution. Both remain
preserved. The third attempt is the first complete physical failure. No gains,
source or budgets changed between those H78 attempts; combined attempt wall time
was 35.214 s. H79 acceptance took 18.546 s; H80 took 31.593 s. All are below the
per-candidate wall budgets; a controller deadline miss still fails the candidate.

The task's 515 original evidence files were copied, individually SHA256-verified,
and relocated to available storage under
`C:/Users/flori/.codex/visualizations/2026/09/30/01a0f41d-4e3d-7c02-924f-c28ead55b9e4/`.
Workspace junctions preserve the original evidence paths. A parent dependency
junction points to the unchanged repository `node_modules`. Historical experiments
outside this task were not moved or deleted. Preserve the junction targets when
moving or backing up the workspace.
