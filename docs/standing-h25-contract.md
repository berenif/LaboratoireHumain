# H25 — distinguish interval displacement from post-step velocity

The H13 contact capture contains every input/output body state for ticks
121–720. Integrating the reported end-of-step foot COM velocities does not
reproduce the measured COM displacement: discrepancies reach centimetres
over ten seconds, while pelvis discrepancy is below a millimetre. This does
not identify the internal cause: within-step velocity changes and positional
constraint stabilization are not separated by the available readback.

H23's explicit stance task uses post-step COM velocity to damp a body-origin
position error. H25 tests interval-matched feedback: use the measured hindfoot
origin displacement between successive command samples divided by the same
fixed timestep, in that Cartesian task only. The initial zero-motion sample
has zero velocity. Leave all native motor damping, balance COM velocity,
gains (34 and 8.5), acceleration cap, measured side-mass weighting, force
allocation, torque ceilings, solver and contacts unchanged.

Compare H25 against H23 at all three headings for 2 s settling + 10 s
observation. Require H23's physical trajectories to replay exactly. Every
unchanged standing/structural bound must pass before continuing to the full
official gate: ≤0.01 m foot drift, ≤0.03 m pelvis drift, ≤0.1 m/s and
≤0.5 rad/s all-segment peaks, both feet planted, no step or non-upright state.
Reject on any failed candidate heading. No velocity-filter or gain sweep.

The new optional `STANDING_TRACE_KINEMATICS=1` records foot positions and
velocities for every response frame without requiring full large event traces.
It is read-only and must preserve the control trajectory hashes.

H25 is rejected. Foot drift at 0/+π/3 is 0.01508/0.01595 m; −π/4 loses support
at tick 574 and leaves the accepted state at tick 578. All H23 controls replay
exactly. The endpoint/velocity distinction is measured, but changing this one
Cartesian feedback sample does not repair standing.

## H26: isolate the continually recomputed local leg posture

H22 keeps world stance references fixed, but still recomputes every local leg
angle against measured pelvis motion. That moving command is another feedback
path even during undisturbed idle; native leg damping targets zero relative
velocity. H16's command-rate compensation failed with a different allocation,
and H25 shows that the explicit Cartesian task remains unsuccessful.

Compare H22 to the same mode holding the first bounded local leg command
rotations throughout uninterrupted idle. Recompute gravity/support/pelvis
feedforward from current physical state exactly as before. Keep the existing
native PD gains and every force ceiling. No body/velocity is fixed, cleared or
teleported, and no commanded foot is substituted for measured support. This
tests internal posture-reference feedback, not a production transition policy.

Require exact H22 control replay and the same complete three-heading 2+10 s
screen. Any failure rejects the hypothesis without trying alternative latch
times or postures. A passing screen would still require full 2+30 s acceptance
and a defined release/transfer policy before production adoption.

H26 is rejected. The three endpoint foot drifts are 0.01017, 0.01858 and
0.01293 m. At −π/4, the left forefoot also reaches 0.88175 rad/s at tick 272.
All three runs remain upright, but none passes the unchanged screen. Every
H22 control trajectory replays exactly; all three H25 controls likewise match
the earlier H23 controls. No controller intervention was adopted.

The [subsequent native-engine calibration](rapier-constraint-calibration.md)
investigates the position/velocity distinction independently. Its altered
rigid fixtures and solver switches cannot establish humanoid acceptance.
