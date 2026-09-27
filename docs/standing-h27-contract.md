# H27 — native translational closure

Evidence: [verified H27–H29 archive](checkpoints/2026-09-27/h27-h29-evaluation/manifest.json).

The rigid calibration isolates a large difference between impulse-joint and
multibody constraint representations. It does not validate a rigid character
as a standing repair. This diagnostic preserves the articulated character.

At initialization, move only the three translational constraints of each
anatomical joint to a native spherical multibody link. Keep the original
impulse-joint handles, angular frames, locked angular axes, limits, finite
force-based motor configuration and motor controller. Verify that every other
serialized impulse-joint field is unchanged. The native converter must not
change any body or collider field, and the JS installation must retain the
original World and all 25 body/collider objects. Native multibody self-contacts
remain enabled; the original hooks and joint contact exclusions remain active.

This split avoids the pinned multibody engine's unimplemented two-angular-axis
branch and the JS wrapper's missing per-axis motor setters. The relative
orientation of each new spherical link is initialized from the original body
rotations; its forward kinematics must agree within 1e-5 m/rad without moving
the bodies. This is initialization-only, with zero initial velocities. No
runtime pose correction, reset, kinematic body or changed actuator ceiling is
permitted. The native joint sets are transferred into the existing JS world;
body ownership does not move to the independently restored diagnostic world.

Begin with a paired heading-zero 2+10 s screen. Stop interpretation on a
structural, finite-state or initialization failure. A passing screen must
subsequently cover the other headings and the unchanged 2+30 s standing gate,
all angular limits, collision behavior, delivered actuation and continuous
ownership. Multibody damping/integration semantics must also be audited before
any production adoption. The current artifact remains diagnostic and depends
on a local native converter; it is not a browser runtime implementation.

The heading-zero screen rejects H27. Initialization preserves every body and
collider field, with internal kinematic disagreement below 1.23e-7 m and
4.85e-8 rad. Nevertheless, at the first observation tick (121) the character
has already lost double support and the accepted standing state. It ends
fallen, with peak segment speeds 4.07135 m/s and 18.61518 rad/s, foot endpoint
drift 0.05924 m, joint separation 0.02371 m and floor penetration 0.10035 m.
The plain control reproduces the recorded upright-but-moving failure. No
production adoption follows. The retained input/output snapshots and native
receipt make the initialization conversion reviewable.
