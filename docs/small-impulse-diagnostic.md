# Small-impulse diagnostic — regenerated evidence

The original 36-case report remains unavailable. The new
`small-impulse-regenerated-20260926-01` run reproduces its reported failure:
right-thigh X, heading zero, +0.0005 Nm·s, predicted 12.93702329 versus measured
15.26150033 rad/(N·m·s), 15.230986% relative error. The other 35 heading/sign/
magnitude cases pass the unchanged 10% assertion. Every driven/control pair
has exactly equal initial physical states. The existing focused test is unedited.

The one-tick control develops 0.39963448 rad/s background hip rate despite
fixture velocity clearing; the driven rate is 0.40726523 rad/s. The predicted
increment is 0.00646851 rad/s and measured increment 0.00763075 rad/s, leaving
an absolute residual of 0.00116224 rad/s. Holding the measurement axis at its
initial orientation still gives 15.25147, so the coordinate-frame update does
not explain the discrepancy. No joint lies within 0.005 rad of a limit. Both
retained contact patches stay present in the driven and control histories.
Alternative free/normal-only/hindfoot-only predictions do not explain it.

## Declared origin-invariance check

Before changing any assertion or production code, repeat the matched probe
after a common ±1 m X translation of the complete already-prepared assembly
and floor. Keep orientations, velocities, gravity, solver, timestep, collision
rules and impulse unchanged. This is an isolated fixture diagnostic, not a
production or acceptance action. Record the actual f32 translation roundoff,
paired physical states, analytical prediction, one-tick rates, and complete
contact/limit observations. Repeat the unshifted case to check determinism.
If predictions remain effectively invariant while response differences change
materially, classify that as numerical origin sensitivity, with no claim about
the exact internal solver operation until further evidence supports it.

The first translated-fixture attempt correctly stopped before measurement:
Rapier's cached solver witnesses stay at their prior world coordinates until
the next step, so immediate native reobservation found no matched support.
The corrected diagnostic observes contacts before translation and maps those
real measured points by the common rigid translation for the analytical
comparison only. It injects no contact or force into Rapier. Post-step contacts
are observed natively after the engine updates its manifold.

All 18 translated/sign/magnitude combinations reproduce exactly on repetition.
At +0.0005 Nm·s, measured responses at origins −1/0/+1 m are
14.89725/15.26150/15.01199 while analytical values vary by less than 1.5e-6.
Translation roundoff is at most 5.96e-8 m. This demonstrates numerical origin
sensitivity but does not by itself explain the full outlier: the positive
small-impulse error persists at all three origins (13.16–15.23%). Keep the task
open. A further resolution check samples adjacent impulse magnitudes
0.00045/0.000475/0.00050/0.000525/0.00055 Nm·s at the original origin, both
signs. It tests response smoothness, not selection of a passing command or a
change to the original assertion.

The resolution check is irregular: positive responses at the five declared
magnitudes are 14.09578/13.27483/15.26150/14.54025/13.92187 rad/(N·m·s).
Increasing the impulse from 0.00045 to 0.000475 actually reduces the measured
rate increment from 0.00634310 to 0.00630555 rad/s; increasing it from 0.0005
to 0.000525 changes the increment by only 0.00000288 rad/s versus the
analytical 0.00032343. The measured response is therefore not a smooth local
derivative at this resolution. The precise internal source of the numerical
residual is still unisolated; no production change, fixture replacement or
tolerance relaxation follows. The original focused assertion passes unchanged.

All 82 paired numerical cases and the failed initial translated-fixture attempt
are retained in the [verified archive](checkpoints/2026-09-26/h21-impulse-evaluation/manifest.json).
The [24-test validation](checkpoints/2026-09-26/continuation-validation-02/manifest.json)
includes the unedited focused hip/ankle assertion. The broader prediction and
physical acceptance gates remain open.

## Declared precision isolation — 2026-09-27

Use the independent native replay to compare a single unchanged physics step
in f32 and f64. Export the complete prepared control/driven worlds after the
original impulse, including contact/joint caches and collision exclusions.
Transfer f32 values exactly into the f64 representation through Rapier's
serialization, and assert that every initial body value matches. Keep timestep,
solver parameters, contacts, mass, gains and the analytic prediction unchanged.

At heading zero/right thigh, cover both signs of the previously declared five
adjacent magnitudes, plus the original passing 0.001 Nm·s case. Require the
exported WASM histories to reproduce the previous cases. Compare native f32
and WASM explicitly; native floating-point execution is not assumed identical.
Attribute the irregular residual to numerical precision only if the f64
responses become smooth at these magnitudes and the original 10% response
bound passes without a changed contact/limit regime. Otherwise leave the cause
open. This is a diagnostic precision change, never an application runtime
upgrade, fixture replacement or acceptance-threshold change.

## Result — numerical precision sensitivity isolated

The separate diagnostic item is resolved as a finite-precision limitation of
the tiny paired-impulse measurement. The application still reproduces the
15.23% outlier; this conclusion does not convert that result into a pass or
repair the application. The original hip/ankle assertion and its 10% bound
remain unchanged and pass in the [nine-test verification](checkpoints/2026-09-27/impulse-precision-validation/manifest.json).

The [verified archive](checkpoints/2026-09-27/impulse-precision/manifest.json)
contains the twelve exported control/driven pairs, native runs, failed
attempts, restoration checks, engine-feature audit and exact source snapshots.
The canonical native batch is `impulse-native-precision-20260927-06`.

| Check | Result |
| --- | --- |
| Snapshot export versus previous WASM histories | All twelve physical cases match exactly |
| Correctly hooked WASM snapshot restoration | All 24 post-step worlds, each with 25 bodies, match exactly |
| Replace only the WASM transient physics pipeline | All twelve original responses remain exactly unchanged |
| Native engine source | 146 Rapier and 337 Parry files match across f32/f64 |
| Enabled solver options | Same source feature branches except precision; block solver disabled in both |
| Serialized f32-to-f64 state transfer | Every physical scalar/cache field verified exactly in all 24 f64 worlds; only zero-valued Pose3A alignment padding is omitted |
| Contact and joint-limit regime | Both left supports remain, with four hindfoot and one forefoot solver points; no self-contact or active joint limit in any of the 48 native results |

The feature audit also checks common dependency versions. Rapier's f32-only
`wide` dependency implements its four-lane data transpose; f64 uses plain
arrays. Parry's f32-only `static_assertions` dependency checks SIMD layout at
compile time. These differences belong to the precision implementations;
the comparison does not add a contact solver or change world parameters.

| Runtime | Maximum relative error | Cases outside original 10% bound | Adjacent positive-increment slopes |
| --- | ---: | ---: | --- |
| Original WASM f32 | 15.23099% | 2/12 | −1.50226, 53.00814, 0.11532, 0.93586 |
| Native f32 | 15.34779% | 7/12 | 5.76773, 20.02616, 15.74774, 30.39272 |
| Native f64 | 1.70319% | 0/12 | 12.72053, 12.72044, 12.72029, 12.72021 |

Slopes compare consecutive rate increments divided by the 0.000025 Nm·s
spacing, rather than selecting a convenient passing magnitude. The analytical
response is 12.93702. The smooth f64 response and the irregular f32 response
from identical initial physical fields isolate the precision-sensitive
numerical residual in the iterative engine calculation. Native f32 and WASM
are not assumed bit-identical. This experiment identifies the precision
implementation as the cause of the irregular excess response, not one specific
arithmetic instruction, nor a shared cause for every standing failure.

Three safeguards corrected preliminary diagnostic attempts. The first f64
batch accidentally enabled its crate's default block solver; it is retained
as confounded evidence and excluded from the conclusion. Two strict transfer
checks stopped before physics on a representation-only difference: f32 poses
serialize an explicit zero alignment field absent in f64 poses. The final
check permits only that documented padding omission and compares all other
fields. The first WASM restoration script omitted the event queue, causing
Rapier's wrapper to ignore collision hooks. Corrected replays supply the queue
and match exactly; the earlier replay difference is not a serialization or
transient-workspace defect. The corresponding warning in native batches 03–05
is superseded by the verified batch 06 qualification.

Reproduction uses the unchanged impulse magnitudes and original fixture:

```text
IMPULSE_HEADINGS=[0]
IMPULSE_SOURCES=rightThigh
IMPULSE_MAGNITUDES=[0.00045,0.000475,0.0005,0.000525,0.00055,0.001]
IMPULSE_EXPORT_SNAPSHOTS=1
node scripts/probe-small-impulse.mjs fresh-input-directory
node scripts/verify-impulse-snapshot-replay.mjs fresh-input-directory/report.json fresh-replay-directory
node scripts/run-native-impulse.mjs fresh-input-directory/report.json fresh-native-directory
node scripts/analyze-native-impulse.mjs fresh-native-directory fresh-replay-directory fresh-analysis.json
```

Set the environment variables using the host shell. Build the two native
executables in `evidence/rapier-calibration-f32-target` and
`evidence/rapier-calibration-f64-target`; use the pinned build command in the
[native calibration instructions](rapier-constraint-calibration.md#reproduction)
and add `--features f64` only for the second. The checked-in Cargo manifest
explicitly disables the f64 crate's default features. Run
`scripts/verify-native-engine-features.mjs fresh-feature-audit.json` with the
same isolated Cargo/Rustup environment before interpreting a new comparison.
`IMPULSE_FRESH_PIPELINE=1` reproduces the separate workspace check.

The [fixed-assembly comparison](rapier-constraint-calibration.md) still collapses
in f64. Quiet standing, measured transfers and prediction acceptance remain
open; no engine upgrade, actuator change or tolerance relaxation follows.
