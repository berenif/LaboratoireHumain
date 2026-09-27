# H31 — construct a centered initial posture

H29 and H30 both require moving the body from the original heel-biased pose.
H30 avoids two of H29's falls but still fails drift and speed bounds. H31 tests
a distinct initial posture: build the same dynamic anatomy with its whole-body
COM already above the area centroid of the original planned sole hull.

Before any dynamic body is created, solve the pelvis's forward displacement
using the existing bounded pose compositor. Preserve the original four sole
positions to 0.00001 m, the neutral knee-flexion input, zero reaction offset,
arm phase zero, masses, shapes, joint frames, anatomical limits and all physics
settings. The compositor may use its existing anatomical height reserve. A
single bisection in the existing maximum-step-reach interval locates the same
geometry-derived centroid; it is not a simulation, gain or offset search.
Require both planned and constructed COM residuals below 0.00001 m. Abort if
the bounded pose cannot satisfy these requirements. Record all solve samples.

Construct the assembly once at zero velocity, then run continuously. There is
no physics warmup, reset, dynamic pose write or velocity write. Apply H22's
held stance, quiet arms, measured pressure projection and world arm damping,
with the centered neutral target from the first update. The normal balance
reset derives the root/COM offset and height from the newly constructed pose.

Compare with exactly replayed H22 controls at all three headings over two
seconds of startup and ten seconds of observation. Require every unchanged
standing, structural, collision, actuation and ownership bound. Reject on any
failure. Passing would require the full official two-plus-thirty-second gate,
and a reviewed production initialization change, before acceptance. This
experiment does not establish recovery into the same posture.

H31 is rejected. All three runs remain in the accepted upright states and keep
double support, but endpoint foot drift is 0.03240/0.02926/0.02550 m. Linear
peaks are 0.52624/0.15839/0.29662 m/s and angular peaks are
6.07413/1.26719/3.17315 rad/s. These failures occur during standing. The
constructed horizontal COM residual is below 6e-9 m at every heading, with a
0.105206 m planned forward root shift; the original sole positions are
preserved to numerical precision. The existing compositor lowers the pelvis
to 0.973139 m using its anatomical reserve. Thus construction satisfies the
declared geometry, but that posture does not satisfy physical acceptance.
All three H22 control histories replay exactly. No production adoption follows.

Evidence: [verified H31 archive](checkpoints/2026-09-27/h31-evaluation/manifest.json).
