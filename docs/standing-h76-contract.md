# H76-v1 — six-snapshot scalar native row read

H76 is a frozen, read-only diagnostic. It compiles the scalar
`JointConstraint<f32, 1>` inspector against the verified Rapier 0.35.0 source
used by the pinned 0.20.0 JavaScript package, then reads the pre/post control
snapshots for all three H75 reference states. The executable performs **zero
physics steps**.

The successful build is retained at
`evidence/standing-h76-v1/native-build-05`; four earlier preflight/link attempts
remain preserved. The successful executable SHA-256 is
`272a7c97b511891a166d9e61ed72e0c505162fe850a5ed496ba4f8cdb52ce1c0`.

Run the frozen read with:

```text
node scripts/standing-native-row-read.mjs evidence/standing-h76-v1/read-01 evidence/standing-h76-v1/native-build-05/standing-angular-native.exe
```

Each result archives native motor configuration and stored impulse, angular
limits and stored DOF impulses, raw and finalized motor/limit/lock rows,
Jacobians, inverse effective row mass, RHS with and without bias, and CFM. The
checks require exact snapshot deserialization, 24 joints, 46 motor rows, the
structural lock rows, finite scalar fields, positive inverse effective masses,
and the declared 1/1200 s row timestep.

The static rows appear coherent. This is not an observation of warm-starting,
solved impulses, contacts, or velocities inside the original WASM step, and it
does **not** explain the retained within-step forefoot motion. It closes no
standing or controller-repair gate.
