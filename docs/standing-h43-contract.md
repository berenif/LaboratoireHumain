# H43 — single-axis multibody forests

H42 confirms that corrected inertias do not remove the independent fixed-tree
collapse. A complete multibody conversion cannot preserve the current joint
contracts directly: pinned Rapier has unimplemented two-angular-axis branches,
and its three-axis position coordinates integrate angular-velocity components
rather than measuring the application's quaternion coordinates.

Test a smaller representation change: move only the five locked axes of each
single-angular-axis joint to a native reduced-coordinate link. Retain its exact
frames, degree of freedom and initial angle. Keep limits and finite motors on
the original impulse joint, with duplicate locked axes removed. All two/three
axis joints retain their original constraints. Preserve all 25 body/collider
objects, initial fields, contacts and world parameters.

Require byte-identical native body/collider sets through initialization and
forward-kinematics agreement within 1e-5 m/rad. Begin with paired plain and
candidate heading-zero 2+10 s screens; stop promotion on any failed structural
or standing bound. This is diagnostic only: the native multibody damping vector
and skipped rigid-body damping path differ and must be resolved before any
production adoption, even if a screen passes. No solver, gain or ceiling search
is included. No body pose correction, reset or kinematic body is allowed.

## Result: rejected

The converter builds successfully and transfers 12 hinge links into four
multibody trees (3, 3, 5 and 5 bodies). Native body/collider bytes and the live
JS objects remain unchanged. Initial forward-kinematics error is at most
1.20e-7 m, with no measured rotation error. Original impulse limit/motor data
are byte-identical apart from removal of duplicate locked axes. The receipt
records every tree's native generalized damping coefficients.

The character leaves the accepted upright states at tick 109, before the
observation window begins. It has no planted feet at tick 121. At tick 133,
16 multibody-linked bodies have non-finite centers of mass and inverse inertia;
the existing inertia guard stops the run at tick 134. Thus the first heading
fails and no other headings or production adoption follow. The post-fall
non-finite state is not established as the cause of the earlier standing loss.

The first failure capture did not retain the completed plain run or the failed
run's full partial history. Subsequent read-only capture retains these, and
copies native matrix components immediately to avoid reusable-buffer aliasing.
The intermediate matrix fields are explicitly untrusted. The two detailed
captures have exactly equal physical history hashes through tick 134. The
plain control's initial hash and all 720 responses equal H41's matching prefix.

The published finite flag nevertheless remained true because it omitted mass
and center of mass. [H44](standing-h44-contract.md) strengthens that check and
rejects the identical trajectory at tick 133 without changing its dynamics.

Evidence: [10 initial artifacts](checkpoints/2026-09-27/h42-h43-initial/manifest.json)
and [19 follow-up artifacts](checkpoints/2026-09-27/h42-h43-followup/manifest.json).
Native sources, build receipt, initial binary snapshots, partial traces and
failure reports are retained. The executable and Cargo caches remain local;
the archive records the executable digest and offline reproduction inputs.
