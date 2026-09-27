# H30 — shape the existing centered target over the startup allowance

H29 applies an 0.087505 m change of neutral COM target in one controller sample.
It fails all three headings. That result leaves the commanded transition and
the terminal target confounded. H30 retains exactly the same initial planned
support-hull centroid and all H22 interventions, but reaches it using the
quintic profile `10u^3 - 15u^4 + 6u^5`, where `u` spans the unchanged two-second
startup allowance and is then held at one. Position, velocity and acceleration
are continuous at both ends of this command profile. No measured body state,
contact, foot anchor or solver parameter is modified.

This is one declared transition, not a duration, offset or gain sweep. Keep
Rapier 0.20.0, balance feedback, native motor ceilings, anatomical constraints,
collision rules, ownership and acceptance bounds unchanged. The target must
equal the H29 terminal target by the first observed sample after tick 120.
Retain the actual commanded offsets in the trace through tick 121.

Compare against H22 at all three headings over two seconds of startup and ten
seconds of observation. Require exact H22 trajectory replay and every standing
and structural bound. Reject on any candidate failure; passing would still
require the full official two-plus-thirty-second gate before production use.
Failure rejects this transition and does not prove that every possible path to
the centroid would fail.

H30 is rejected. At 0 and +π/3 it remains upright but foot drift is
0.01953/0.03061 m, linear peaks are 0.12764/0.18560 m/s and angular peaks are
1.04971/1.33620 rad/s. At −π/4 it loses double support at tick 538 and leaves
the accepted upright states at tick 556. Its large neck/hand peaks occur later
and cannot identify the initiating fault. All three H22 histories replay
exactly. The saved command trace verifies zero initial profile progress and
the exact H29 terminal target at tick 121. No production adoption follows.

Evidence: [verified H30 archive](checkpoints/2026-09-27/h30-evaluation/manifest.json).
