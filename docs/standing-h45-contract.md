# H45 — forefoot target and damping in the sole's world frame

The corrected-inertia H22 screen passes linear speed but exceeds angular speed
at the right forefoot in two headings. At +π/3 tick 536 the measured forefoot
coordinate is 0.01443 rad; the local zero target requests −0.93827 Nm of posture
torque while gravity/load compensation contributes only +0.14299 Nm. This is
the requested motor wrench, not proof of its delivered impulse or the cause
of the ensuing speed peak.

The leg compositor solves a hindfoot endpoint. Preserving that leg chain's
local targets is necessary for its endpoint, but the forefoot is distal to it.
Test expressing only the forefoot's existing planned world orientation against
its measured hindfoot, with the matching parent-angular-velocity subtraction
for world-frame damping. Keep every proximal leg target, original world sole
target, force/torque cap, stiffness, damping coefficient and physical setting.
Apply only during idle before any step or grab.

Compare the corrected-inertia H22 control and this one frame correction at
0/+π/3/−π/4 for 2 s settling plus 10 s observation. Preserve the H22 quiet-arm,
held-reference and measured-pressure interventions identically in both modes.
Require exact control replay and all original standing/structural bounds before
advancing to 2+30 s. No gain sweep or production adoption follows a failed
screen. H18 changed distal damping alone; it did not change the forefoot target
frame. All prior rejected comparisons remain retained.

## Result — rejected

All three controls exactly reproduce the retained H22 physical histories.
The frame correction fails the angular bound at every heading: 0.665823,
0.536563 and 0.589537 rad/s at 0/+π/3/−π/4 respectively. The +π/3 foot
endpoint drift also exceeds 10 mm (11.109 mm). All runs remain upright in
double support with zero steps; linear and structural bounds pass. No
production change follows this failed screen.

The [10-artifact archive](checkpoints/2026-09-27/h45-forefoot-frame/manifest.json)
retains reports, analysis, source snapshots and bounded peak trace extracts.
Complete traces remain local at the manifest paths with their full digests.
