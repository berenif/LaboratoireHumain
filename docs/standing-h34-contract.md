# H34 — bounded integral compensation of stance posture error

H22 passes the short-screen speed bounds but retains 0.01378–0.01777 m foot
drift. Its held world stance references generate bounded local joint targets,
tracked by finite PD motors and feedforward. A persistent tracking error is
not removed by that proportional feedback alone. H34 tests integral rejection
of this residual, without claiming that all measured drift has that cause.

Retain H22 and add one integral feedforward term on each existing free leg
joint axis. Integrate the same bounded-target coordinate error already used
by the native posture motor: `I += stiffness * error * dt / 2 s`. The two-second
integration time is declared once, using the existing startup allowance; do
not sweep it. Keep original proportional/damping gains and all actuator
ceilings. Bound the unscaled accumulator by the original axis torque ceiling.
Freeze an update if the complete proposed PD + feedforward + integral request
would exceed that ceiling and the update would increase saturation. Allow
updates that unwind saturation. The final request still uses the same capped
native force-based motor, with no additional body wrench.

Enable the accumulator only during uninterrupted idle with qualified measured
sole contact on both sides. Clear it on a step, grab, support loss, inactive
intervention or non-standing controller phase. Record active ticks, saturation
events and the maximum strength-scaled integral torque. This is a diagnostic
policy; passing standing would not prove its transfer/release behavior.

Compare to exact H22 controls at 0, +π/3 and −π/4 over two seconds of startup
and ten seconds of observation. Require all unchanged drift, speed, support,
state, step, structural, collision and ownership bounds; reject on any failure.
Any passing candidate must then pass the full official two-plus-thirty-second
gate before a production implementation is considered. No physics setting,
geometry, body state, target anchor or acceptance threshold changes.

H34 is rejected. Foot drift falls to 0.01010/0.01219/0.01207 m but still exceeds
the limit at every heading. The 0/+π/3 angular peaks rise to 0.80975/0.76701
rad/s; +π/3 also reaches 0.11466 m/s and 0.03427 m pelvis drift. All three runs
remain upright with double support. Integral compensation is active for 717
ticks; its largest strength-scaled axis terms are 17.65765/17.81798/12.07346 Nm.
No accumulator update hits the saturation guard, so these runs do not verify
that branch. All three H22 control histories replay exactly. No production
adoption or integration-time sweep follows.

Evidence: [verified H34 archive](checkpoints/2026-09-27/h34-integral-evaluation/manifest.json).
