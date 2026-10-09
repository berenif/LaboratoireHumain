# Contact-gated corrective stepping repair

This continuation repairs the strict stepping failures recorded in the
[October 5 swing diagnosis](rust-swing-diagnosis-2026-10-05.md). The canonical
anatomy, scenario definitions, physical measurements and frozen v24 input
profile are preserved.

The controller rebases both takeoff and landing intent onto the measured foot
at admission. Active stance reserves motor effort according to measured native
PD requests while retaining ordinary gains, every anatomical torque ceiling and
the 760 N m aggregate ceiling. Persistent sole load shares the pelvis-restoring
hip torque through the native joints' equal-and-opposite response.

An unadmitted lateral transfer can redirect when the pointer reverses. A raw
pointer displacement guard distinguishes reversal from filter overshoot at a
held target. The
moving foot stays anchored through IK using the measured pelvis, while native
sole motors prepare the loaded support and a small physical toe-off prepares
unloading. A short queued continuation uses that same support side; its retained
leg follows a gradual pelvis intent, and its mover continues using measured
pelvis IK. Commanded movement never increments the step counter.

During qualified touchdown, the COM reference shifts toward the measured
two-foot mean over the existing 0.10-second confirmation. The following stance
blends from measured joint coordinates over 0.6 seconds with matching native
target rates. Releasing an ordinary unadmitted request cancels it only when both
soles are loaded and their measured hull contains the capture point; committed
swings and queued continuations retain their physical completion requirements.

Admission still requires the capture point inside the measured retained sole
with a 3 mm inset for 0.10 seconds, persistent retained load of at least 52% of
body weight and moving load of at most 25%. Verified liftoff still requires at
least 20 mm of measured rise, less than 3 N of normal load and more than 10 mm
of actual sole clearance for 0.05 seconds. Landing still requires at least 40 mm
of measured horizontal travel, target error at most 90 mm and loaded touchdown
for 0.10 seconds. Final standing, clinical bounds and self-collision checks are
unchanged.

## Verified results

All **nine strict fixtures pass**, compared with one before this repair. Every
fixture finishes physically stable; the four previously stable fixtures are
preserved. Direction reversal completes two measured steps and stays stable
through tick 510. The other eight fixtures each complete one measured step.

| Fixture | Verified steps | Landing ticks |
| --- | ---: | --- |
| slow-hand-forward | 1 | 103 |
| slow-hand-backward | 1 | 130 |
| slow-hand-left | 1 | 147 |
| slow-hand-right-heading | 1 | 147 |
| slow-left-foot | 1 | 123 |
| slow-right-foot-heading | 1 | 115 |
| direction-reversal | 2 | 237, 339 |
| held-target | 1 | 156 |
| release-during-corrective-step | 1 | 144 |

All **88 Rust workspace tests**, formatting and native/WASM Clippy pass. The
13 tooling/release checks and four engine Jacobian/cache-equivalence tests pass;
their results are reused with verified unchanged inputs. The two new regressions
cover raw-pointer reversal versus held-target overshoot and release cancellation
versus unsafe, queued or admitted transfers. Motor regressions retain body-state
immutability, ordinary gains, anatomical ceilings and the aggregate cap.

The final strict reports are byte-identical. Their SHA-256 is
`4c78ea07a7690de3800849d8c9cdc08a08eedfeeae5051fdc128bb393121afef`.
The unchanged input profile SHA-256 is
`971661c88ae7258f157669d8e8a873161de40990274a9f2fd7d159f50006e00e`.
Local source snapshots, final receipts and comparison are retained under
`evidence/strict-stepping-2026-10-09/`: `code-checks/`, `verified-a/`,
`verified-b/` and `comparison.json`. Generated evidence is excluded from Git.
Failed trials are retained; large October 8 probe reports are losslessly
archived in `evidence/strict-stepping-2026-10-08.zip`. A disk-full foundation run,
a superseded partial foundation run and rejected stale-binary reuse are recorded
separately from the successful final code checks and dedicated strict replays.

Production stepping and recovery remain opt-in. Qualifying these frozen strict
fixtures does not complete recovery, cycles, terrain, browser or hardware
performance qualification, and does not authorize Pages promotion.
