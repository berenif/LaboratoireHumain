# Upright balance controller

The procedural controller retains the 60 Hz architecture. `BalanceController.update` reads solved segment positions and velocities, returns a bounded root displacement, stance intent, a reachable corrective step, coordinated pose inputs, and a fall decision. It never owns Rapier bodies or recovery. `reset` seeds feet and momentum from the current world poses, including after a completed get-up. `composeUprightPose` preserves heading and places each foot from its solved ankle, so an unreachable request cannot disconnect the shin.

## Fixed thresholds

`BALANCE_LIMITS` in `src/character/BalanceController.ts` is the executable source. The limits below are fixed before acceptance validation.

| Quantity | Value / rule |
| --- | --- |
| Mass state | Sum segment position and velocity multiplied by segment mass, divided by total participating mass |
| Velocity filtering | Exponential response 14/s; capture point = horizontal COM + filtered horizontal velocity / sqrt(9.81 / COM height) |
| Stance intent | Exclude the active swing foot and any grabbed foot displaced over 5 cm from its press target, even beside the floor |
| Eligible solved foot | Sole clearance -2.5 to +3.5 cm, foot-up dot world-up > 0.8, solved horizontal center within 4.5 cm of its planted target |
| Stance persistence | Eligible for at least 50 ms |
| Support polygon | Convex hull of actual solved, eligible foot rectangles; local half-width 9.5 cm and half-depth 15.5 cm |
| Pull | Spring 620 N/m plus target-relative damping 12 Ns/m, total vector capped at 620 N |
| Stance actuator | Horizontal acceleration capped at 2.6 m/s²; integrated root speed capped at 2.5 m/s |
| Step anticipation | Capture point plus external horizontal acceleration × 0.09 s² |
| Step initiation | Anticipated support margin below 5.5 cm; no absolute drag-distance trigger |
| Step reach | Foot target at most 36 cm horizontally from pelvis reference, at most 43 cm from its start; two-bone solver enforces exact anatomical reach |
| Step timing | 340 ms swing, 65 ms cooldown; new steps depend on current balance, including after release or reversal |
| Immediate failure | No eligible support, support margin below -43 cm, or COM horizontal speed above 1.85 m/s while support margin is below -12 cm |
| Marginal failure | Margin below -24 cm and no recoverable landing footprint; or margin below -10 cm, projected landing margin below -4 cm, and speed above 0.75 m/s |
| Marginal persistence | 120 ms continuous marginal instability; decays at twice elapsed time while stable |
| Swing recovery capacity | Margin of projected touchdown capture point inside the hull of current stance plus the reachable landing footprint; projection adds velocity × half remaining swing time |
| Pelvis/knees | Base flexion 0.12, plus bounded balance and step flexion; at most 10 cm pelvis lowering, with connected two-bone legs |

During a swing, current support can temporarily be narrow. Marginal falling additionally requires the planned landing footprint to fail its momentum test (margin below -2 cm for the -24 cm current-margin case). An already unrecoverable current state still falls immediately. A released manipulated foot first lands from its current solved position through a step, even if the player held it close to the floor.

The default root motor starts at 0.977 m, its grounded collider height, with flexion 0.12. Starting above that height caused artificial knee motion while the motor settled, which could falsely initiate a step without input.

## Deterministic checks

Focused tests cover exact joint connectivity with unreachable foot grabs and steps; rotational equivalence of every segment at non-default headings; mass-weighted velocity; exclusion of manipulated near-floor feet; and slow hand pulls with reversal and release at headings 0, 1.1, and -1.7 radians.

Smoke trajectories use grounded default poses and 1/60 s updates. Hand lateral (+0.60, 0, +0.08), hand forward (0, 0, +0.70), hand backward (0, 0, -0.60), and foot (+0.22, +0.03, +0.02) targets ramp over 90 ticks, hold 60 ticks, then release for 240 ticks. Each remains connected, completes corrective stepping, and returns upright. A hand pull (+1.25, +0.15, +0.20) over 5 ticks and a sustained +1.50 m lateral pull over 180 ticks overpower balance. These are behavioral fixtures; fall decisions use support, applied force, momentum, and reachable landing capacity rather than those target distances.

External force is reported separately from balance acceleration. Dynamic contact, protective motors, recovery assistance, input lockout, and both transfer bounds are verified by the full physics harness.
