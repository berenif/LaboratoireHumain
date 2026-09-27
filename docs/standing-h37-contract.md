# H37 — check the preceding stable JavaScript runtime

The official [binding release history](https://github.com/dimforge/rapier.js/blob/0fd32c1cbbc7018af36f09b190c16ce72fbb9301/CHANGELOG.md)
identifies the 0.19 series before the current package's 0.20 rewrite, with the
0.19.1 update selecting Rapier 0.30.0. The official 0.19.3 package is preserved
with registry metadata and verified SHA-512 integrity. Do not infer that it
fixes the observed constraint drift from its age or release notes.

First run the unchanged 21-test adapter, joint-coordinate and joint-motor
selection through the existing process-local runtime loader. Adapt package
entry-point lookup to the older package layout and override only the adapter's
expected-version literal. Preserve every ABI presence check, frame/limit
readback, force ceiling and assertion. Missing APIs must fail visibly; never
replace a required force-cap setter with a no-op or run unbounded motors.

This compatibility check is separate from a physical regression comparison.
If required raw APIs are absent, the published older package is not a drop-in
production candidate. A separate unactuated rigid fixture can still compare
constraint behavior, but must preserve and verify its geometry, masses,
initial poses, collision filtering and explicit numerical settings without
pretending to establish articulated standing acceptance.

## Result

The exact 0.19.3 binding commit's [Cargo template](https://github.com/dimforge/rapier.js/blob/0fd32c1cbbc7018af36f09b190c16ce72fbb9301/builds/prepare_builds/templates/Cargo.toml.tera)
selects Rapier 0.30.1 and Parry ^0.25.3. Its unchanged focused selection passes
14/21: four failures lack `jointSetMotorMaxForce`, one lacks `setLocalFrame1`,
and two lack `revoluteWithAxes`. No guard was bypassed. The revised loader with
installed 0.20.0 passes all 21 original tests.

Common initial meshes, masses, poses, velocities and seven explicitly configured
public solver settings produce collapsing fixed-joint assemblies in both versions
at every heading. At tick 720 every pelvis is near 0.10 m height. This is not an
isolated solver comparison: observed contact ERP differs (0.239057 vs 0.135755),
and reconstructing full body-local tensors reveals substantial inertia differences.
COM differences are below 1e-8 m. The [H38 geometric audit](standing-h38-contract.md)
investigates those inertias independently. The older published package is not a
drop-in repair, and no runtime downgrade is adopted.

Evidence: [verified H37 archive](checkpoints/2026-09-27/h37-runtime-comparison/manifest.json).
