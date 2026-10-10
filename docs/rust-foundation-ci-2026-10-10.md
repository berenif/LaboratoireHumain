# Rust foundation CI repair — 2026-10-10

The failed [CI run 37942668428](https://github.com/berenif/LaboratoireHumain/actions/runs/37942668428)
repeated the same Protocol +60° quiet-standing failure: left forefoot drift
reached 0.010001016780734062 m at tick 1838, substep 0. Primitive contacts,
canonical contacts, isolated joints and loaded-foot calibrations passed.
The receipt's `missing: ["contact-stress", ...]` lists full release stages;
foundation mode intentionally does not complete those stages.

The exact failure was reproduced with Rust 1.89.0 and Ubuntu 24.04's glibc
2.39 math runtime. Ubuntu 26.04's default math library produces a different
trajectory, so it was not sufficient to validate the CI repair by itself.
The reproduction uses an isolated extracted runtime, without replacing WSL's
system libc. Original failed artifacts and rejected probes remain in local
`evidence/` directories.

## Implementation

The candidate is `rust-physics-v25-relative-com-feedback`. Supported COM
feedback subtracts its world reference before accumulating mass-weighted
coordinates in double precision. It captures the initial COM rounding residual
once and removes it from displacement measurements against that captured
physical target. Independent stepping and recovery goals retain their literal
world coordinates. Calibration and recovery resets capture a new residual.
Quiet observation does not reset the target after settling.
COM velocity damping increases from 4 to 5 s·rad/m; COM position gain remains
5 rad/m. Fixed stepping, anatomy, solver iterations, motor caps, contact
settings and all acceptance limits retain their previous values.

The acceptance CLI now has an explicit `--foundation` scope containing the
documented eight recoverable pull probes. The shared disturbance runner had
expanded to fourteen fixtures, including six overpower falls, and the native
foundation call inherited that expansion. The full disturbance diagnostic still
contains fourteen fixtures, and strict stepping still contains nine. Unit
coverage checks those three scopes and preserves all foundation fixture IDs.

Each acceptance run's compact JSON summary is retained in `execution.json`
and printed in the CI log, including the first failed assertion, mode and
heading. Native `passed` also accounts for failed disturbance probes.
Foundation success now requires the cached-engine regression and tooling
checks in addition to the existing checks. Full release admission and
publication remain gated on all required release stages.

## Verification

The translated-spawn regression verifies zero initial COM error, a measured
1/2/−3 mm displacement, and an unchanged captured reference. It passes on
the pinned Windows GNU and Linux compilers.

Targeted frozen-profile probes of the selected candidate pass both modes at
+60° for all 1920 ticks, including 480 settling and 7200 observed substeps:

| Native runtime | Protocol maximum foot drift | Playground maximum foot drift |
| --- | ---: | ---: |
| Linux with glibc 2.39 | 9.502 mm | 8.764 mm |
| Windows GNU | 9.048 mm | 9.126 mm |

Final validation with fresh default artifacts passes the full foundation
runner on Linux with the CI math runtime and Windows GNU, using the original
timeouts. Each platform passes formatting, native and WASM release Clippy,
90 workspace tests, the Jacobian and cached-engine regressions, tooling checks,
and two runs each of native foundation, worker-runtime quiet standing and
striker acceptance. Reports repeat byte for byte and source hashes stay
unchanged during each run.

All six actual dedicated-worker quiet-standing cases also pass, with unchanged
source and loaded artifact fingerprints. Maximum quiet foot drift is 9.502 mm
on Linux, 9.126 mm on Windows GNU and 9.393 mm in WASM, against the unchanged
10 mm limit. The verification notes were finalized after those receipts; the
verified Rust and script source is unchanged.

Run the normal scoped gate with:

```sh
node scripts/run-rust-gates.mjs --foundation evidence/rust-foundation-verification
```

Local final evidence paths are `evidence/rust-ci-foundation-windows-relative-warm-2026-10-10`,
`evidence/rust-ci-foundation-linux-relative-verified-2026-10-10`, and
`evidence/rust-ci-wasm-relative-verified-2026-10-10`. Inspect their receipts for
completion, first failures and source/artifact identities. Earlier cold runs
retain their compile timeouts and are not reported as successful verification.
Foundation success does not imply full release acceptance.
