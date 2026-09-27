# Rapier constraint calibration — 2026-09-27

The standing milestone remains open. These are independent calibration
fixtures and source checks, not accepted controller changes. The application
still uses its pinned Rapier package and unchanged physical settings.
The [55-file verified archive](checkpoints/2026-09-27/h25-h26-calibration/manifest.json)
retains the reports, snapshots and source receipts. The
[verification manifest](checkpoints/2026-09-27/h25-h26-validation-complete/manifest.json)
records successful native compilation, 25 targeted tests and clean targeted lint.

## Position and velocity readback

`analyze-standing-kinematics.mjs` verifies the full H13 contact-trace digest and
compares measured COM displacement with integrated post-step COM velocity.
Over ticks 121–720, the endpoint differences reach 0.02160 m for a forefoot;
the pelvis differences remain below 0.00076 m. This is a measured discrepancy,
not evidence that friction is absent.

The official [0.20 binding manifest](https://github.com/dimforge/rapier/blob/v0.35.0/typescript/builds/prepare_builds/templates/Cargo.toml.tera)
maps to Rust Rapier 0.35.0, matching the installed runtime's version readback.
Its [solver worker](https://github.com/dimforge/rapier/blob/v0.35.0/src/dynamics/solver/staged_island_solver/worker.rs)
integrates positions after the biased pass, then runs the unbiased pass before
writing velocities. By default,
[friction is solved in the unbiased pass](https://github.com/dimforge/rapier/blob/v0.35.0/src/dynamics/solver/staged_island_solver/solve.rs).
Thus a post-step velocity is not the velocity used for every position update.
This order alone does not explain the humanoid's centimetre-scale drift.

The 18-case `probe-jointed-friction.mjs` calibration compares a 10.8 kg rigid
compound, a two-body fixed joint and a finite force-based motor, at both lean
signs and all three headings. It preserves the humanoid world settings and
per-body additional iterations. All remain near stationary: the maximum
endpoint foot drift is 0.0000238 m over the 2+10 s fixture. The simple joint
therefore does not reproduce the full assembly's instability.

## Fixed assembly versus compound

`probe-rigid-assembly-drift.mjs` independently replaces the humanoid's
articulated joints with native fixed joints at the initial measured poses.
All 25 bodies remain dynamic. A second mode combines the same collider
geometry and masses into one dynamic body. Initial marker positions,
velocities and quaternion components compare exactly; total mass matches
within 0.0001 kg and COM within 0.000001 m. The initial 72.2 kg assembly's COM
projects inside the geometric sole polygon.

The fixed-joint fixtures topple at all three headings. The compound fixtures
remain standing, with maximum endpoint foot drift 0.000251 m in the 2+10 s
window. Fixed-joint controls replay exactly in the repeated JS captures.
This difference concerns the constraint representation; neither fixture is an
admissible replacement for the required articulated character.

Two exporter assertions initially failed: class prototypes differed between
native vectors and plain marker objects, then a quaternion-composition helper
normalized the native f32 components. The exporter now compares plain values
and uses an unnormalized quaternion product for marker readback. No numerical
initial-state equality check was loosened. The second failed run and its
completed results are preserved; the first prototype-only error is recorded
here from its terminal output and has no saved full log.

## Native replay and controlled interventions

`scripts/rapier-calibration` reads the unmodified JS bincode snapshot using
Rapier **0.35.0**, Parry **0.30.2** and the same collision exclusions. Its
Cargo lockfile is retained. Rust 1.89.0 GNU was installed only in the ignored
calibration directory because the existing MSVC environment lacks the Windows
SDK libraries. The application package and lockfile were not changed.

Native and WASM initial states match exactly. At tick 120, their pelvis
positions differ by 0.0000660 m and both have begun the same backward fall.
They diverge substantially after impact; this is not a bit-identical native
replay, and post-impact trajectories cannot establish the source of an earlier
standing event.

The final native batch uses one executable and records its digest, every input
digest, exact commands, all exit codes and complete before/after source hashes.
Each following intervention changes only the independent rigid fixture:

| Intervention | Result |
| --- | --- |
| Default snapshot parameters | Topples |
| Solve friction in both passes | Topples |
| Warm-start joint impulses | Topples |
| Remove all internal body contacts | Exactly the same recorded states as the native control; no self-contact contributes |
| Near-rigid fixed-contact compliance | Unstable; rejected |
| Ten times the biased PGS iterations | Topples; increasing this count is not a demonstrated repair |
| Match unbiased iterations to the existing 20 biased iterations | Topples |

These are mechanism checks, not settings selected for production. The
instability's full cause remains open. In particular, the rigid-fixture result
does not establish that the live controller's drift has the same cause.

The next isolation checks the existing heading-zero fixed assembly in f64,
with the same serialized initial state, normal solver options and twelve-second
horizon. Its purpose is to test whether the small-impulse precision sensitivity
also explains this collapse. If collapse remains, precision alone is not a
repair for this fixture and cannot justify a standing-runtime change.

The initial transfer guard rejected the unstepped fixture because its pending
joint-creation `to_join` hash set was reordered by serialization. That run
performed no physics. Export the unchanged fixture after its first raw step,
when pending creation has been processed, and verify that this read-only
snapshot export leaves all three original WASM histories unchanged. Compare
both native precisions from that same tick-one snapshot; do not weaken the
serialized-state equality guard to accept a different pending mutation order.

The tick-one export leaves all three original WASM trajectories exactly
unchanged. Both native precisions start with exactly equal body fields, and
the f64 executable verifies every serialized physical/cache field. Both
collapse: at native tick 120 the pelvis heights are 0.781448 m (f32) and
0.781628 m (f64), with backward displacements about 0.632 m; at tick 180 their
heights are 0.333382 m and 0.262300 m. Precision alone therefore does not repair
this fixture. This comparison and the rejected pre-step transfer are retained
in the [precision archive](checkpoints/2026-09-27/impulse-precision/manifest.json).

## Fixed multibody comparison

Keeping the same 25 dynamic bodies and collider geometry, the new
`fixed-multibody` calibration replaces the 24 fixed impulse joints with 24
native fixed multibody links. Initial body markers and total mass remain
identical; all 25 original handles remain dynamic throughout the run. This
tests the constraint representation while retaining separate body ownership.
Like the other rigid calibrations, it locks all relative anatomical motion
and therefore cannot count as quiet-standing acceptance.

| Heading | Fixed impulse-joint maximum foot endpoint drift | Fixed multibody maximum foot endpoint drift | Compound maximum foot endpoint drift |
| --- | ---: | ---: | ---: |
| 0 | 0.487775 m, collapsed | 0.000339 m | 0.000246 m |
| +π/3 | 0.537499 m, collapsed | 0.000416 m | 0.000250 m |
| −π/4 | 0.514200 m, collapsed | 0.000449 m | 0.000045 m |

These are displacements between seconds 2 and 12, not maxima over the official
30-second observation. The multibody pelvis stays near 0.982 m height at both
endpoints. All nine physical histories in the combined run exactly reproduce
their earlier controls/repeats. The
[five-artifact archive](checkpoints/2026-09-27/fixed-multibody-calibration/manifest.json)
retains both multibody runs, the comparison/lint receipt and source snapshots.
No production source or physical setting changed.
The analysis receipt's `bodyCount` counts retained original body handles;
its zero for the compound means the originals were replaced by one compound
body, not that the compound scene has no body.

This narrows the independent rigid-fixture instability to its impulse-joint
constraint representation; it does not yet identify a specific faulty solver
operation or prove that the articulated controller's drift has the same cause.
A useful next candidate must preserve every anatomical degree of freedom,
joint limit, finite motor ceiling and ownership invariant. Replacing the real
character with this rigid assembly is not such a candidate. The pinned JS
multibody wrapper also lacks the per-axis motor setters used by the current
controller, so adopting this representation requires further investigation.

Reproduce this comparison with `RIGID_MODES=fixed-joints,fixed-multibody,compound`
and `node scripts/probe-rigid-assembly-drift.mjs fresh-output.json`.

## Separate contact-impulse reporting defect

The upstream [0.35.2 fix](https://github.com/dimforge/rapier/commit/37eeac7dec6addd6a94a237b4874ca0b676d70da)
removes an extra prior-step warm-start impulse from the reported total. The
official [release history](https://github.com/dimforge/rapier/blob/v0.36.0/CHANGELOG.md)
places this fix in 0.35.2, not 0.36.0. It explains the isolated resting-box
normal-impulse overcount already observed with the pinned 0.35.0 runtime.
A uniform scaling factor is not a general correction during changing contact
loads: the wrongly included term is the prior warm-start impulse. No reporting
rescale or runtime upgrade was adopted. This is also separate from the now
identified [small-impulse precision sensitivity](small-impulse-diagnostic.md).

## Reproduction

Use the pinned Node executable recorded in the evidence manifest. The JS
calibrations accept a fresh output JSON path. Set
`RIGID_EXPORT_SNAPSHOTS=1` for binary snapshots and collision/body metadata.

For the native executable, set `CARGO_HOME`, `CARGO_TARGET_DIR` and
`RUSTUP_HOME` to the corresponding `evidence/rapier-calibration-*` directories,
then build with:

```text
cargo +1.89.0-x86_64-pc-windows-gnu build --offline --release --manifest-path scripts/rapier-calibration/Cargo.toml
node scripts/run-rapier-calibration.mjs snapshot.bin metadata.json fresh-output-directory
```

The offline command requires the already downloaded crates and toolchain.
The wrapper checks the snapshot digest, refuses existing output, records
partial failures and verifies that neither source nor executable changed.
The portable archive includes binary snapshots and compressed source; Cargo
caches, toolchain files and the compiled executable remain local-only.
