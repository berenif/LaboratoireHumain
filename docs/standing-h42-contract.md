# H42 — constraint calibration after geometric inertia correction

Before investigating a different joint representation, repeat the independent
fixed-assembly comparison on the current corrected-inertia factory. H25/H27
predate that production change. Use the existing `fixed-joints` and
`fixed-multibody` fixtures at all three headings, with the same initial 25
dynamic bodies, contacts, world settings and 2+10 s window. The standing
controller is absent; all relative anatomical motion is locked in both modes.

This is a mechanism check only. It cannot pass the articulated standing gate,
and a representation change would still need every degree of freedom, joint
limit, finite motor ceiling, collision rule and ownership invariant. Preserve
both successful and failed runs. Check native capabilities before proposing
an implementation; the pinned JS multibody wrapper exposes no motor setters.

## Result

Corrected-inertia fixed impulse assemblies still topple at all three headings,
ending with pelvis height about 0.1031 m. Fixed multibody assemblies end near
0.9820 m and stay nearly stationary over the recorded 2+10 s window.

| Heading | Fixed impulse maximum foot endpoint drift | Fixed multibody maximum foot endpoint drift |
| --- | ---: | ---: |
| 0 | 0.511238 m | 0.000471 m |
| +π/3 | 0.268614 m | 0.000513 m |
| −π/4 | 0.521225 m | 0.000349 m |

These are rigid calibrations, not articulated standing results. The geometric
inertia correction therefore does not remove this representation-dependent
fixture failure. It does not prove the live controller has the same cause.

The pinned native source also shows unimplemented two-angular-axis branches,
three-axis coordinates accumulated from angular velocities rather than the
application's quaternion coordinates, and a different multibody damping path.
The full native files and wrapper declarations are retained with their hashes.
The narrower single-axis experiment is recorded in [H43](standing-h43-contract.md).

Evidence: [initial capture and build archive](checkpoints/2026-09-27/h42-h43-initial/manifest.json),
[analysis and native-source archive](checkpoints/2026-09-27/h42-h43-followup/manifest.json).
