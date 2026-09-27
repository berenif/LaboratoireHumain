# H46 — exact one-step forefoot actuation authority

H45 fails the angular-speed screen. Before choosing another controller, test
whether the two existing forefoot motors can remove the H22 forefoot peaks at
all three headings without violating another segment's speed bound.

At retained peak ticks 530/536/688, capture the native world after the normal
motor configuration and before the next step. Independent copies preserve all
25 body fields, motor caps, settings, contacts and hooks. Vary only the two
forefoot motor velocity biases, expressed as additional feedforward torque.
The unperturbed copy must exactly match every field of the 25 live post-step
bodies. No copied body state is ever transferred into the live world.

Minimize the sum of squared world angular speeds of both forefeet using up to
eight Gauss–Newton iterations. Central probes are ±1 Nm; each proposed update
has a 1 Nm maximum component and line search uses 1, 1/2, 1/4, 1/8. A 1e-8
diagonal regularization makes the two-variable normal equation nonsingular.
Accept only an actual copied-world improvement with all linear speeds ≤0.1 m/s
and every other segment's angular speed ≤0.5 rad/s. Bound the accumulated bias
by the unchanged native torque ceiling; all forces still pass through the
original finite native motor. Retain every probe and rejected line-search case.

If a useful bias exists, replay it for exactly one live tick in a fresh run.
Require the snapshot digest, current motor configurations and initial bodies
to match the source capture, then require all 25 live outputs to equal the
copied prediction exactly. Check the original bounds through the next five
ticks and retain the complete 2+10 s report. The trace must distinguish this
native override from its published base motor request.

This establishes local authority only. It does not establish sustained standing,
long-horizon model accuracy, production transfer prediction or an online
controller. No gain, acceptance-threshold or physical-setting changes occur.

## Result — exact local model, insufficient two-axis authority

All 122 copied cases preserve their initial 25-body fields. The three
unperturbed full histories exactly reproduce H22; all three live pulses
exactly match the chosen 25-body predictions. The pulse-plus-five-step
angular maxima are 0.425977, 0.563734 and 0.481360 rad/s at 0/+π/3/−π/4.
The +π/3 pulse remains above the independent bound. Earlier uncorrected peaks
also remain in the full histories. No online controller or standing gate is
accepted from this two-motor result.

The [20-artifact archive](checkpoints/2026-09-27/h46-forefoot-authority/manifest.json)
includes every copied case, native peak snapshot, live pulse result, full
response history and source. Model traces captured only tick 1, so their
archived peak extracts contain zero rows; copied peak states provide the
local evidence. Pulse traces have bounded retained peak windows.
