# Upright balance controller

The procedural controller retains the 60 Hz architecture. `BalanceController.update` reads solved segment positions and velocities, returns a bounded root displacement, stance intent, a reachable corrective step, coordinated pose inputs, and a fall decision. It never owns Rapier bodies or recovery. `reset` seeds feet, root height, and momentum from the current world poses while retaining a neutral COM-to-ankle offset derived from the same knee-flexed canonical stance used in production. `composeUprightPose` preserves heading and places each foot from its solved ankle, so an unreachable request cannot disconnect the shin.

## Fixed thresholds

`BALANCE_LIMITS` in `src/character/BalanceController.ts` is the executable source. The limits below are fixed before acceptance validation.

| Quantity | Value / rule |
| --- | --- |
| Mass state | Sum segment position and velocity multiplied by segment mass, divided by total participating mass |
| Velocity filtering | Exponential response 14/s; capture point = horizontal COM + filtered horizontal velocity / sqrt(9.81 / COM height) |
| Stance intent | Exclude the active swing foot and any grabbed foot displaced over 5 cm from its press target, even beside the floor |
| Eligible solved foot | Sole clearance -2.5 to +3.5 cm, foot-up dot world-up > 0.8, solved horizontal center within 4.5 cm of its planted target |
| Stance persistence | Eligible for at least 50 ms |
| Support polygon | Convex hull of actual solved, eligible foot rectangles; local half-width 5 cm and half-depth 13.5 cm |
| Pull | Spring 620 N/m plus target-relative damping 12 Ns/m, total vector capped at 620 N |
| Stance actuator | Horizontal acceleration capped at 3.6 m/s²; integrated root speed capped at 2.5 m/s |
| Step anticipation | Capture point plus external horizontal acceleration × 0.09 s² |
| Step initiation | Anticipated support margin below 2.5 cm (half the foot half-width, capped at 3 cm); no absolute drag-distance trigger |
| Step reach | Foot target at most 36 cm horizontally from pelvis reference, at most 43 cm from its start; two-bone solver enforces exact anatomical reach |
| Step timing | Adaptive 180–340 ms swing based on travel and urgency, then 180 ms double-support cooldown; new steps depend on current balance, including after release or reversal |
| Immediate failure | No eligible support; support margin below -43 cm without a recoverable active landing footprint; or COM horizontal speed above 1.85 m/s while support margin is below -12 cm |
| Marginal failure | Margin below -24 cm and no recoverable landing footprint; or margin below -10 cm, projected landing margin below -4 cm, and speed above 0.75 m/s |
| Marginal persistence | 120 ms continuous marginal instability; decays at twice elapsed time while stable |
| Swing recovery capacity | Margin of projected touchdown capture point inside the hull of current stance plus the reachable landing footprint; projection adds velocity × half remaining swing time |
| Pelvis/knees | Base flexion 0.12, plus bounded balance and step flexion; at most 10 cm pelvis lowering, with connected two-bone legs |

During a swing, current support can temporarily be narrow. Both the -43 cm immediate-margin case and the -24 cm marginal case therefore consult the planned landing footprint; the projected touchdown margin must be below -2 cm before that recoverable swing is abandoned. The high-speed and no-support cases still fall immediately. A completed procedural touchdown contributes support immediately when its measured floor, tilt, and target checks pass. The first lateral correction widens toward the disturbance, then feet alternate so the trailing leg follows. A released manipulated foot first lands from its current solved position through a step, even if the player held it close to the floor.

The default root motor starts at the declared 0.990 m pelvis-center height, with flexion 0.12. Its height-derived capsule retains the controller's 1.2 cm floor clearance. The static floor is inserted into Rapier's broad phase before the first controller query, preventing an initial 3.5 cm sink and false velocity spike. Once anatomical foot support is established on the flat floor, horizontal collision movement is retained while snap-to-ground's incidental vertical correction is suppressed; vertical descent resumes when support is genuinely lost. Starting above the declared root height caused artificial knee motion while the motor settled, which could falsely initiate a step without input.

## Deterministic checks

Focused tests cover exact joint connectivity with unreachable foot grabs and steps; rotational equivalence of every segment at non-default headings; mass-weighted velocity; exclusion of manipulated near-floor feet; and slow hand pulls with reversal and release at headings 0, 1.1, and -1.7 radians.

Smoke trajectories use grounded default poses and 1/60 s updates. Slow forward, backward, and lateral hand targets include a +0.03 m vertical offset; slow foot targets use (+/-0.22, +0.03, +/-0.02) m. They ramp over 90 ticks, hold 60 ticks, then release for 270 observed ticks. Each remains connected, completes corrective stepping, and returns upright. Abrupt hand pulls reach 1.25 m horizontally with a +0.12 m vertical offset over 5 ticks. Sustained forward and backward pulls reach 1.50 m with a +0.10 m vertical offset over 180 ticks. Both overpower balance. These are behavioral fixtures; fall decisions use support, applied force, momentum, and reachable landing capacity rather than those target distances.

External force is reported separately from balance acceleration. Dynamic contact, protective motors, recovery assistance, input lockout, and both transfer bounds are verified by the full physics harness.
