# H40 — bounded hand-reach targets

The corrected-inertia left chest-drag fixture retains real pelvis/lumbar contacts
but never reaches the torso. Added read-only arm telemetry reproduces its exact
root travel. At the final sample, the requested hand height is 1.38070 m while
the motor's geometric hand target is only 1.05430 m. The shoulder target has
reached its extension and lateral lower bounds. The unconstrained two-bone
solution followed by independent angle clipping no longer reaches its endpoint.

Refine only active hand-grab kinematic targets with bounded damped least squares
over the three shoulder coordinates and elbow flexion, preserving the existing
girdle, wrist/twist intent and utilization limits. Reconstruct the entire chain
from the original anchors. Accept a refinement step only when it reduces hand
position error. Native motors, physical poses, solver, collisions, actuator caps,
grab limits, falling and recovery policies are unchanged. Idle never invokes
this refinement.

Require the existing left and right chest-drag contact/penetration/separation
assertions and relevant anatomy/kinematics tests to pass. Verify bounded static
targets, anchor closure and rigid-world covariance. This is not a standing fix:
all pending standing, transfer, recovery and release gates remain required.

## Result

The final solver uses the measured shoulder/elbow coordinates as the live seed
and removes outward-pointing coordinates from its active least-squares system
at a limit. All accepted iterations reduce endpoint error. Four new tests cover
active limits, reachable endpoints, all joint/anchor bounds including infeasible
requests, and world-yaw/translation covariance. Frozen measured pose inputs are
accepted without mutation. Static requests without measured poses keep the
geometric seed.

The original left and right contact fixtures pass, with peak joint separation
0.00005135/0.00003546 m and no positive penetration. Left-hand torso contact
first occurs at tick 123 in `reacting`; falling begins at tick 207. The contact
test is strengthened to reject a left-hand contact obtained only after recovery
begins. Both runs later lose balance, so these results establish contact and
structural bounds only, not balance or recovery acceptance.

One intermediate solver revision failed the left separation bound at 0.020321 m;
its logs and fingerprints are retained. The final measured seed resolves that
failure. The wider final selection passes **61/63 targeted tests**: all 49 from
the previous candidate pass, as do the new reach tests. Typecheck and targeted
lint pass. These are unit/regression tests, not the 63 physics-harness scenarios.

The two remaining failures reproduce with the exact archived pre-H40 compositor:
an idle right elbow moves 0.98–3.48 mm inward relative to its shoulder under the
specified lean, and a swing-foot full-forward-vector comparison fails. The
latter's planted heading passes; its swing forward vector pitches 9.8° while
its horizontal yaw differs by only about 0.63°. The original assertions are
unchanged. The required geometric behavior and measurement scope need resolution.

An idle H22 trace at +π/3 reproduces every H39 physical history hash exactly and
retains the 0.714825 rad/s forefoot spike. Hand reach does not repair quiet standing.

Evidence: [verified H40 archive](checkpoints/2026-09-27/h40-bounded-reach/manifest.json).

The final source also replays all three complete 2+30 s plain-controller captures
exactly: every physical history hash and response equals the prior inertia
candidate. Those standing failures remain unchanged. The [verified idle archive](checkpoints/2026-09-27/h40-idle-preservation/manifest.json)
retains the new reports, comparison and final production/diagnostic source.
