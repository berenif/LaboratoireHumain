# H29 — center the neutral balance target in the planned support hull

Evidence: [verified H27–H29 archive](checkpoints/2026-09-27/h27-h29-evaluation/manifest.json).

The initial planned center of mass is 0.007872 m ahead of the ankle midpoint,
while the unchanged sole geometry spans approximately −0.04 to +0.23 m. H22
removes idle forcing and the speed failures but retains sustained foot drift.
Test whether its heel-biased neutral target leaves an unnecessarily marginal
support/load distribution.

Compare H22 with one geometry-derived target: the area centroid of the convex
hull of the four initial sole support patches. Express it as the existing
neutral-COM offset from the held ankle/support target. Reuse the existing
balance feedback, root/leg IK, native motors and finite force ceilings to reach
that target during the two-second startup allowance. Do not move a body, apply
an external wrench, invent a contact or shift the measured foot anchors.

This is a single declared geometric choice, not an offset/gain sweep. Preserve
the original Rapier 0.20.0 runtime, all physics settings, anatomical degrees of
freedom, limits, collisions and acceptance bounds. Hold all other H22 changes
identical. Require exact H22 control replay and the complete three-heading
2+10 s screen. Any candidate failure rejects the hypothesis; a pass would still
need the full 2+30 s standing and structural/actuation gates before adoption.

H29 is rejected. The planned centroid is 0.095378 m ahead of the ankle
midpoint. All three candidates lose support and enter recovery; their first
support-loss/non-upright ticks are 392/438, 254/336 and 551/569. Foot endpoint
drifts are 1.25630, 2.53230 and 0.76711 m. The large later speed peaks occur
after instability and must not be presented as its initiating cause.

H22 controls exactly reproduce their previous trajectories and retain their
0.01501/0.01378/0.01777 m foot drifts. The candidate has many infeasible or
rank-deficient support-projection frames, unlike the controls' three startup
empty-contact frames. This does not establish whether the final target or its
direct startup application is responsible; do not silently reinterpret this
failed run as a passing equilibrium experiment. No production adoption or
parameter/offset sweep follows.
