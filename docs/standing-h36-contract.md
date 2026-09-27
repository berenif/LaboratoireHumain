# H36 — measure the local constrained plant response

The H34 integral and H35 interval-damping candidates fail the unchanged gates.
Before choosing another feedback correction, measure how the native constrained
system moves under small changes to the existing motor commands.

Use H22 at each heading and take a snapshot at tick 121 after its normal motor
configuration, before stepping. In independent restored worlds, hold those
native settings for thirty ticks. Keep joints, body state, contacts, collision
hooks, event queues, solver parameters, damping and torque ceilings unchanged.
Change only one native motor's velocity bias to add ±1 Nm feedforward on each
of the fourteen existing leg axes. Record all 25 body states at horizons 1, 6
and 30 ticks. Calibrate a central finite-difference response using only ±1 Nm.

Use ±0.75 and ±1.25 Nm as held-out inputs on each axis, plus the same four
amplitudes applied simultaneously with opposite signs on the two sides. Keep
these cases out of calibration. Record absolute and relative response scales;
small baseline motion can make relative errors misleading.

Before interpreting any response, require the unperturbed restored world's
first step to match every measured field of all 25 live bodies exactly. Verify
the live H22 trajectory still matches its earlier control over the full 720
ticks. Keep every original body/collider object and never transfer a predicted
state or applied perturbation into the live world.

This is plant identification with held native motor commands. Beyond the first
step it does not reproduce the controller's future command updates. It cannot
establish TODO transfer-prediction or standing acceptance. The full controlled
half-second trajectory and measured-load/readiness gates remain open.

The observer passes its fidelity checks: all 25 live body states match the
copied baseline exactly at every heading, and the three full H22 trajectories
replay exactly. The capture retains 267 copied-world cases, with fourteen
calibrated axes and sixty held-out cases per heading.

Across all headings and held-out cases, maximum horizontal foot prediction
errors are 0.00004793, 0.00030741 and 0.00139947 m at horizons 1, 6 and 30 ticks.
At heading zero and 30 ticks, the combined-input maximum error (0.00139947 m)
exceeds that group's largest actual foot change (0.00088754 m). Thus small
absolute errors do not establish a sufficiently accurate inverse model for
foot control. The largest horizontal COM error at 30 ticks is 0.00117657 m,
but the frozen-command qualification still excludes the production prediction
gate. No feedback controller is adopted from this local linear model.

Evidence: [verified H36 archive](checkpoints/2026-09-27/h36-response-model/manifest.json).
