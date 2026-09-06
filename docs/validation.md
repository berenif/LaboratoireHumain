# Validation record

All required checks pass on Node 24.19.0: **35/35 physics scenarios, 30/30 application tests, typecheck, lint, and four complete native browser fall/recovery cycles across WebGL and Canvas 2D.** The original uncommitted refactor and unrelated changes were preserved before editing.

## Baseline comparison

| Check | Untouched baseline | Final result and evidence |
| --- | --- | --- |
| Typecheck | Passed | Passed; [log](../evidence/final-typecheck.log) |
| Lint | Passed | Passed; [log](../evidence/final-lint.log) |
| Build and application tests on supported Node 24 | Build passed; 20/20 tests passed | Build passed; 30/30 tests passed; [log](../evidence/final-application-tests.log) |
| Physics | Original 11/11 scenarios passed | Expanded 35/35 scenarios passed; [complete results](../evidence/physics-results.json), [log](../evidence/acceptance-final.log), [compact metrics](../evidence/final-physics-summary.json) |
| Native visual replay | Not included in the baseline | Four recoveries completed in both renderers; [report](../evidence/visual-20260906/final-prone/REPORT.md), [video](../evidence/visual-20260906/final-prone/full-both-renderers.webm), [per-step metrics](../evidence/visual-20260906/final-prone/per-step-summary.json) |

There are no remaining new failures in these checks. The default PATH Node 20.17.0 remains a verified baseline environment limitation: its build fails because node:fs/promises does not export glob. The unchanged baseline builds and passes its 20 tests on bundled Node 24.19.0. The project requires Node 22.13 or newer. Existing non-failing build messages concern large chunks and static route classification.

The [baseline summary](../evidence/baseline-20260906/summary.json) links exact check logs. Its folder also preserves original working-tree status and HEAD, the binary tracked diff, SHA-256 inventories, and all 135 tracked/untracked source files in source-snapshot.zip, including all 14 original untracked refactor files. Snapshot hashes were verified before archival. The generated snapshot directory was removed afterward so recursive TypeScript includes could not accidentally check baseline copies.

## Final numerical results

All scenarios use Rapier 0.20.0 at exactly 60 Hz. Assertions inspect every fixed update and every segment at both authority transfers; the trace samples every sixth update for inspection.

| Measurement | Headless maximum | Native replay maximum | Fixed limit |
| --- | --- | --- | --- |
| World-space joint-anchor separation | 0.021801 m | 0.018663 m | 0.08 m |
| Actual collider penetration into the finite floor | 0.024266 m | 0.024924 m | 0.08 m |
| Same-instant per-segment handoff translation | 0.0000000752 m | Before/after snapshots retained | 0.025 m |
| Same-instant shortest handoff rotation | 0.000003416 degrees | Before/after snapshots retained | 3 degrees |
| Completed fall/recovery duration | 20.70 s | 11.55 s | 25 s |

The headless suite observed 26 completed falls, including all five consecutive fall/recovery cycles without Reset. Paired lockout replay injected commands during 312 locked frames: position and velocity differences were exactly zero, with quaternion calculation roundoff of 0.000004183 degrees, below the 0.00001-degree tolerance.

The final native replay recorded 3,831 fixed updates, all finite. All 2,233 locked updates had exactly zero external grab force and no active grab. Four fresh presses after recovery were accepted. Native recovery times were 9.867, 5.850, 11.550, and 9.950 seconds. [Source hashes before](../evidence/visual-20260906/final-prone/source-before.json) and [after](../evidence/visual-20260906/final-prone/source-after.json) the run are retained.

Floor semantics are explicitly tested: clearance, a disabled floor, and shapes outside the floor footprint produce zero penetration; a known 0.035 m penetration measures 0.0349999964 m. Removing support stops assistance before the next integration and never forces standing after a timeout. A supported ceiling obstruction reaches the unchanged 3-second stall threshold, produces five dynamic retries, and records 60 near-stall frames with qualifying support.

## Regression coverage and retained failures

The final suite includes slow hand/foot pulls, corrective steps, held targets, reversals, release during a swing, strong and sustained pulls, non-default headings, seven surface-anchor/idle-timing regressions, and two archived exact native command streams. [The captured fixtures](../scripts/fixtures/native-fixed-streams.json) retain initial segment poses, every timestep and command, and source hashes. Timed presses are accepted only when body input is available at that recorded step; a press arriving during lockout is correctly rejected. Every observed fall must still complete within 25 seconds.

Earlier failures remain available for review:

- [Initial native replay failures](../evidence/visual-20260906/full/REPORT.md) motivated the preserved oblique-pull regressions and contact-dependent rolling corrections.
- [The 33-case candidate](../evidence/checkpoints/physics-native-candidate-33.json) detected a 0.108690 m transient ankle separation. [Controlled experiments](../evidence/constraint-solver-experiment.json) isolated the collision substep issue. Four CCD substeps remove the spike while retaining the 60 Hz simulation, 16 solver iterations, and existing torque caps.
- [The first four-CCD candidate](../evidence/checkpoints/physics-ccd4-candidate-35.json) exposed supported-prone stalls. Contact-gated roll-height progress corrected them; the final complete suite passes.

No geometric, continuity, settling, contact, motor-torque, or recovery-time threshold was weakened to obtain these results. The joint motor ceiling remains 110 Nm. See [physics acceptance](physics-acceptance.md) for the independent golden thresholds and [dynamic recovery](dynamic-recovery.md) for the implementation.
