# Audited Rapier experiment

This directory contains the checksum-verified `rapier3d 0.35.0` registry package
(`c56bf7b5596ef716abcf578636db53b3780162043fada35717d46f79d4022fa4`).
The original package remains in the baseline cache and the existing native
calibration workspace. This fork is an experimental dependency, not release
acceptance.

1. `contact-reporting.patch` backports the accumulator initialization from
   [upstream commit 37eeac7](https://github.com/dimforge/rapier/commit/37eeac7dec6addd6a94a237b4874ca0b676d70da).
   Version 0.35.0 already banks impulse before warm-start scaling. Its source
   and patch fingerprints are in `contact-provenance.json`.
2. The coordinate experiment changes the uncoupled 3D limit/motor row Jacobian
   from a plain basis axis to the derivative of `theta_i = 2 atan2(q_i,q_w)`,
   and makes motor position measurement use that same coordinate. Locked rows,
   coupled rows, collision geometry, and body integration retain upstream code.
   A finite-difference test uses actual world quaternion perturbations at three
   headings and three combined rotations. Contact calibration and all 216
   isolated-joint stress cases must pass again after every relevant change.
3. `wide-motors.patch` and `wide-motors-provenance.json` audit the optional
   `experimental-wide-3d-motors` layer against the frozen scalar source. It
   retains scalar 3D row assembly, packs independent angular-motor rows, and
   avoids repeating padded-lane assembly. Unsupported coupled/linear motors
   retain scalar fallback. Default features, caps and iteration counts are
   unchanged. Native A–C and eight existing pulls retain exact results, and an
   80-tick actual-WASM probe matches the scalar worker with identical SIMD
   flags at three headings. Browser timings still fail; this path stays off by
   default and does not establish full browser physics or release acceptance.
   The layer also repairs four test-only conversion warnings without changing
   their numeric values. Standalone vendor tests use their own pinned lockfile;
   workspace acceptance and browser checks use the workspace lockfile.
4. `tangent-reporting.patch` and `tangent-provenance.json` add a read-only
   world-space manifold tangent total. The Simplified solver reports its patch
   impulse; Coulomb reports the sum of its selected-point impulses. Existing
   warm-start, solver impulse and integration calculations remain unchanged.
   The ordered floor adapter returns the impulse on the measured foot. Native
   and actual-WASM bounded linear-box matrices pass all 216 cases, with a
   0.00019845 N s maximum error against the unchanged 0.002 N s threshold.
   The original high-force matrix remains 200/216: its 16 high-speed failures
   are retained, including the 0.019085 N s maximum. Six floor-disabled controls
   also fail momentum balance, demonstrating f32 integration error contributes
   without fully explaining the largest contact error. Unit tests verify both
   bounded calibration and rejection of the stress experiment; they do not
   turn its failed reports into passes. Angular, canonical-foot and generic
   multibody tangent reporting remain unqualified and the signal does not feed
   character control. The engine passes 292 standalone tests with serde enabled.

The Rust rework's v22 default explicitly selects Coulomb friction after native
and WASM foundation checks and all six worker quiet-standing cases pass. Legacy
profile JSON without the new field preserves the previous Simplified model.
This is a scoped standing repair; performance and full migration acceptance
remain open.

For `q=(v,w)`, left angular perturbations give `qdot = omega*q/2`; therefore
`grad(theta_i) = (w² e_i + w(v cross e_i) + v_i v)/(w²+v_i²)` in the parent
joint frame. The denominator guard keeps the arithmetic finite at the coordinate
singularity; it does not make that singular pose physically accepted. Body and
structural acceptance still inspect every integration substep.

The failed contact-only v1 source is preserved under
`evidence/rust-rework-20261001/native-01/source`, with a manifest and report.
Unmodified 0.35.0 baseline replay is under `baseline/native-fixed-assembly`.
