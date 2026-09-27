# H8 evaluation — 2026-09-26

H8 is rejected as a standing repair. The regenerated allocator fixture is usable,
and H8 resolves the selected local discontinuity, but every prolonged screen
fails the unchanged stationary motion contract. Production physics is unchanged.

The [tooling validation manifest](checkpoints/2026-09-26/standing-tooling-validation.json)
records 13 passing regression tests, passing targeted ESLint, and full TypeScript
typecheck with identical before/after source fingerprints. Tests cover physical
allocation invariants, explicit fallback reasons, archive digests, tampered-trace
rejection, immutable output, and the existing baseline/continuity regressions.
Later rejected investigations are documented in [H9–H11](standing-h9-contract.md).

The [archive manifest](checkpoints/2026-09-26/h8-evaluation/manifest.json) identifies
29 compressed artifacts with compressed and uncompressed SHA-256 digests. It
includes complete response histories, the recorded tick windows, failed screens,
and a snapshot of production source and package manifests. This archive is new
evidence, not recovery of the missing original fixture. The initial heading-zero
capture is also preserved; the historical discontinuity occurs at +π/3.

`standing-regenerated-20260926-02` runs 240 ticks at +π/3 with plain, observed,
and repeated controls. All three physical histories and initial states match
exactly. The 175–181 trace window is nonempty. Replaying ticks 175–176 reproduces
both production allocator plans exactly under JSON signed-zero semantics. The
observed tick-176 left-forefoot speed is 1.07393464297 rad/s, matching the original
reported event. All production/package hashes match the recorded replay source.

| Stage | Result |
| --- | --- |
| Static maximum patch load jump | 247.41780685 → 0.11198060 N; ≤20 N passes |
| Static force residual | 4.44e-16 N |
| Normalized horizontal moment residual | 5.83e-17 |
| Minimum resulting patch share | 0.08658510; nonnegative |
| Matched one-tick intervention | 1.07393464 → 0.05274181 rad/s; 95.09% reduction |
| Following five forefoot responses | 0.06210, 0.10620, 0.06864, 0.11612, 0.25161 rad/s |
| Startup screen, 2 s settling + 10 s observation | 0/6 pass; H8 rejected |

The local pair first differs at tick 176. Its full control response matches the
regenerated capture. There is no delayed return of the spike over the declared
five-tick window. These facts establish a local response only.

| Heading | Mode | Peak m/s | Peak rad/s | Endpoint foot drift m | Patch infeasibility | Support changes |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 0 | H8 | 0.52674 | 1.94794 | 0.01027 | 35 | 6 |
| 0 | H8 + upper-limb world damping | 0.14089 | 0.79073 | 0.01276 | 31 | 4 |
| +π/3 | H8 | 0.58317 | 1.26283 | 0.01854 | 13 | 14 |
| +π/3 | H8 + upper-limb world damping | 0.18639 | 1.03956 | 0.01532 | 16 | 0 |
| −π/4 | H8 | 0.67045 | 1.61357 | 0.02049 | 62 | 0 |
| −π/4 | H8 + upper-limb world damping | 0.23923 | 0.65487 | 0.02153 | 34 | 18 |

Each run additionally records one initialization and three initial pressure-
infeasibility events. All retain finite state, persistent body/collider identity,
double support, zero steps, and upright state. Observed anchor separation is
below 0.000048 m, self-penetration is zero, and floor penetration is below 0.0044 m.
These diagnostics do not substitute for the full structural/actuation suite.
The first screen combined initialization/support/pressure reasons; a second
screen separated the reasons and reproduced all trajectories. Both are retained.

## Reproduce and verify

Verify the committed archive without needing the ignored evidence directory:

```powershell
node scripts/archive-standing-evidence.mjs verify docs/checkpoints/2026-09-26/h8-evaluation/manifest.json
```

Files are ordinary gzip streams. Decompress each `archive` entry under a fresh
directory, restoring its original run-directory and filename. Replay an extracted
`standing-regenerated-20260926-02/1-observe.jsonl` with:

```powershell
node scripts/replay-standing-allocation.mjs <trace.jsonl> 176 <fresh-replay.json>
node scripts/evaluate-standing-h8.mjs <capture-report.json> 176 <fresh-evaluation.json> <local-report.json> <screen-report.json>
```

The evaluator verifies trace hashes and production source/package fingerprints,
and returns exit 1 for the recorded H8 rejection. It does not relabel this as an
execution error or promote the experiment into production.

For a fresh capture, use `scripts/probe-standing-causality.mjs <fresh-directory>`.
Set `STANDING_HEADINGS=[1.0471975511965976]`, `STANDING_FRAMES=240`,
`STANDING_START_TICK=176`, `STANDING_PULSE_TICKS=1`, and
`STANDING_MODES=plain,observe,observe-repeat`. Optional
`STANDING_TRACE_START_TICK=175` and `STANDING_TRACE_END_TICK=181` bound trace size
without shortening physical execution or the recorded response history. The
local intervention uses `plain,hold-patch-loads`. The screen uses all three
headings, 720 frames, start tick 1, duration 720, and modes
`hold-patch-loads,held-patch-world-damping`. Use the same production source for all
stages, evaluate each before proceeding, and retain failed outputs.
