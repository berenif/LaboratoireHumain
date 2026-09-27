# H38 — verify geometric inertia before another controller change

H37's common-mesh comparison reveals different physical inertia tensors, beyond
principal-axis permutation. Signed tetrahedron integration of the exact float32
mesh vertices agrees with 0.19.3 to below 1e-6 relative maximum tensor-entry error
for all 25 segments. In 0.20.0, thirteen segments exceed 0.1 relative error,
including both hindfeet (0.42865) and upper arms (0.81621). This is a measured
mass-property defect, not evidence that it explains all standing failures.

The independent integrator uses closed oriented triangles, exact polynomial
tetrahedron moments, the parallel-axis theorem and a relative-tolerance Jacobi
diagonalization. Analytic tetrahedron, translation/winding and rotated repeated
eigenvalue tests cover its numerical conventions and small scales. Compare
full reconstructed tensors, not eigenvalue order or quaternion components.

At initialization only, explicitly supply these geometric mass properties to
each existing collider. Preserve its current mass, shape, collision rules and
body/collider identity. Verify unchanged body origins, orientations and velocities;
record the sub-1e-7 m COM roundoff. Require reconstructed inertia readback within
1e-6 relative error and stable principal properties after every physics step.
Keep timestep, solver, motor ceilings and all acceptance bounds unchanged.

Evaluate paired unchanged/plain and H22 controls with their respective corrected
inertia variants at all three headings for the 2+10 s screen. Require prior
controls to replay exactly. Passing screens must then pass the official 2+30 s
standing and all structural/actuation/ownership gates before claiming a repair.
Retain a verified mass-property defect even if its correction alone fails the
standing gate; do not treat that as evidence that the geometric calculation is wrong.

## Paired screen result

The independent correction reduces maximum tensor-entry error from 0.0074290
to 9.31e-9 kg m²; all 25 bodies retain their corrected properties throughout
the runs. All three H22 controls replay exactly. The corrected plain controller
still exceeds speed bounds at every heading. Corrected H22 gives:

| Heading | Foot drift (m) | Peak linear (m/s) | Peak angular (rad/s) |
| --- | ---: | ---: | ---: |
| 0 | 0.008783 | 0.040205 | 0.460633 |
| +π/3 | 0.010137 | 0.052059 | 0.617906 |
| −π/4 | 0.009007 | 0.038280 | 0.540488 |

All remain upright in double support with zero steps. Heading zero passes the
short kinematic bounds; the other headings fail, so the complete screen remains
unaccepted. The geometric mass-property defect is confirmed and warrants a
separate numerical correction; it does not by itself repair standing.

Evidence: [verified H38 archive](checkpoints/2026-09-27/h38-inertia-evaluation/manifest.json).
