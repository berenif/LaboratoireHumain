# H12–H21 — current patch geometry, pressure and stance references

H9 showed a remaining forefoot spike with continuous patch loads. H11's attempt
to avoid unnecessary forefoot torque failed statically because it reused the
discontinuous hull-vertex weighting inside the hindfoot subset. H12 tests the
combination: use the current hindfoot subset only when it can represent the
existing requested pressure, then use H7's constrained equal-patch projection
over **all** its measured points. If hindfeet cannot represent pressure, retain
the existing all-contact allocation and count that fallback. Keep eligible
forefoot entries with zero requested force. No contact or force budget changes.

Apply the same mapping to both regenerated ticks 175–176: require maximum
patch-load jump ≤20 N, normalized force/moment residual ≤1e-8 and nonnegativity.
If it passes, apply for one tick at 176 after exactly matched unchanged history;
require ≥20% forefoot response reduction and no postponed peak over five ticks.
Then run 2+10 s at all three headings, alone and with upper-limb world damping,
requiring all standing and structural bounds before any production adoption.
Reject at the first failed stage. Gains, ceilings, solver, collisions, thresholds,
physical ownership, and measured readiness remain unchanged.

H12 is rejected statically: 22.02054 N exceeds the 20 N limit. There is no
runtime promotion. The input inspection identifies a concrete remaining
cardinality defect: the right-hindfoot four-point manifold contains a doubled
corner, and the doubled corner switches between ticks. Its physical three-
vertex patch contour remains nearly unchanged. Equal weight per witness thus
changes the objective even without a material support-geometry change.

## H13: use each patch's own measured contour for its objective

Replace only H12's per-witness objective by the measured vertices of each
individual patch hull, using the existing hull helper and recovering each
vertex's real measured height/owner. This preserves the represented support
region, introduces no contact point, and removes the changing redundant corner
weight. Use the same static/local/startup sequence and numerical thresholds
above. Record loss of hindfoot pressure feasibility explicitly; no parameter
sweep follows failure.

H13 passes static and local gates: 2.33605 N jump, 92.65% forefoot-response
reduction, next five ticks ≤0.47743 rad/s. Its combined arm-damping mode passes
both speed bounds at every heading (linear ≤0.04222 m/s, angular ≤0.30866 rad/s)
but fails foot drift at all headings: 0.02101/0.03374/0.03493 m. H13 is rejected
as a complete repair. All current-hindfoot projections succeed after the three
initial no-pressure ticks. The left foot migrates backward/medially while the
controller adopts that slip into its posture and neutral-support references.

## H14: hold stance intent after removing rapid motion

Compare the H13-plus-damping control to the same mode with the existing idle
posture-reference hold from initialization. This tests reference creep after
the rapid-motion gate has been independently met, unlike earlier H4/H10.
Actual contacts, support, and readiness remain measured. Require the held
candidate to meet all unchanged 2+10 s standing/structural bounds at every
heading, and confirm the control reproduces H13. Reject otherwise; no release
or step reference policy is authorized by a failed idle screen.

H14 is rejected. The control reproduces H13 exactly, but holding stance intent
still leaves 0.02161/0.03264/0.02608 m foot drift and produces angular peaks
0.60742/0.68745/0.63538 rad/s. Reference adoption is therefore not a sufficient
explanation of the remaining slip.

## H15: preserve measured force sharing on current patch contours

H13 routes all support through hindfeet even while measured forefoot loads
remain nonzero. That force-sharing mismatch can leave the positional motors
correcting a sustained load error. Test H6's measured-load objective after
removing the demonstrated redundant-witness weighting: use every qualified
patch's own current contour, retain measured patch shares as the objective,
and enforce the unchanged total force/moment/nonnegativity constraints.
This is instantaneous allocation; no measurement smoothing or obsolete support.
Keep the same ≤20 N static pair bound and subsequent local/startup contracts.

H15 is rejected locally. Static jump is 9.86756 N and the first response falls
71.36%, but the second following tick reaches 0.87698 rad/s, exceeding the
frozen 80%-of-control bound (0.85915). No startup screen follows.

## H16: track the moving leg command's velocity

H14 holds foot posture references but still updates leg IK against measured
pelvis motion. Native damping targets zero relative joint velocity, opposing
the changing angles required to keep a sole stationary. Swing already supplies
a command-velocity bias; standing does not. Test that specific mismatch by
adding the same bounded target-coordinate derivative term to standing leg
commands in the H14 candidate, from initialization. Keep the existing stiffness,
damping, torque ceilings, allocation, measured contacts, and fixed timestep.
The first sample has no derivative and injects no startup impulse. Compare
against the identical H14 mode; require unchanged stationary and structural
bounds at all headings after 2 s settling over the 10 s screen. Reject on failure.

H16 is rejected: foot drift remains 0.02230/0.03187/0.02731 m and angular peaks
0.54234/0.58559/0.56472 rad/s. Compensating command rate does not remove the
sustained foot-position error. All variants remain diagnostic-only.

## H17: supply the missing planted-foot position task through joint torques

Holding an IK reference and correcting its rate both leave centimetres of
measured slip. H17 adds an explicit horizontal planted-hindfoot position/velocity
task to the speed-stable H13-plus-damping mode. The task uses the existing COM
feedback law (34 s^-2 and 8.5 s^-1), its existing 3.6 m/s² bound, and the measured
side's share of body mass. Map its requested force to each ancestor joint with
the measured lever arm (Jacobian transpose), then combine it inside the same
native joint torque ceilings. No force is applied directly to a body or floor.
The intent is the initial foot posture; contacts/eligibility remain measured.
Enable only in uninterrupted idle with qualified current sole contact, no step
and no grab. This is not a production transition policy.

Compare to H13-plus-damping at all three headings. Require the same 2+10 s
speed, drift, support, ownership and structural bounds, and verify the control
matches its previous trajectory. No gain or cap sweep follows a failure.

H17 is rejected: foot drift is 0.02108/0.03256/0.02565 m. The final two headings
also fail angular speed (0.51216/0.76808 rad/s), and the −π/4 run needs 33
rank-boundary and four all-contact fallbacks. Before another controller change,
inspect native normal/tangent impulses and world velocities at actual solver
contacts under the reproducible H13 speed-stable control. The new read-only
`STANDING_TRACE_CONTACTS=1` option must leave its physical trajectory unchanged.

The contact observer reproduces every H13 control trajectory exactly. Loaded
hindfoot point-speed paths are typically 0.002–0.012 m versus centre paths of
0.013–0.057 m over the screen; body rotation/rocking therefore needs to be
distinguished from contact-point slip. Per-contact tangent readback is zero
throughout, but an independent 1 kg box held against a constant 5 N horizontal
force also reports zero while momentum balance requires −5 N·s. Thus those
fields cannot establish friction saturation or absent friction. The box also
reports 10.30050 N·s normal impulse over 1 s versus 9.81 N·s from momentum
balance; the normal-readback discrepancy remains separately uncorrected.

## H18: damping in the distal world-orientation frame

H16 differentiated the planned leg chain, which cannot correct measured parent
tracking errors. The sole/ankle/forefoot orientation targets are world-level
stance targets, while their native damping still follows parent motion. Test
the exact parent-angular-velocity subtraction already used for arms, now on
ankle/hindfoot/forefoot commands, in the H13-plus-arm-damping control. Use current
measured parent velocity; no derivative filter, stiffness change, added force,
or torque-cap change. Require the same all-heading 2+10 s standing/structural
screen before any production adoption, and preserve the paired control.

H18 is rejected: the +π/3 angular peak becomes 0.89455 rad/s. Distal world damping
alone does not cure the remaining physical behavior.

## H19: equal geometric patch references on current contours

H13's hindfoot-only objective leaves eligible forefeet carrying physical load
without a commanded share; H15's instantaneous measured amplitudes reintroduce
transient load feedback. H19 applies H7's equal-patch reference to all current
patch contours, removing only the now-demonstrated redundant-witness weighting.
This gives each real eligible patch a reference while preserving the same
force/moment/nonnegativity constraints. Test the same ≤20 N two-tick static
bound, ≥20% matched one-tick response reduction with five-tick no-postponement,
then the same 2+10 s screen. No production change or tuning follows failure.

H19 is rejected. Static jump is 7.59030 N and the local response falls 89.94%,
with the following five ticks at most 0.19080 rad/s. Its combined screen still
fails foot drift; the +π/3 angular peak is 0.69760 rad/s. No production adoption.

## H20: preserve measured pressure as well as measured patch amplitude

H15's reference distributes each measured patch load uniformly across its
contour, discarding the measured within-patch pressure. This can command a
different joint moment even if patch amplitude is unchanged. H20 maps each
current measured patch pressure onto its own measured contour and weights that
reference by its current measured normal force. Use the equal-patch geometric
metric to avoid zero reference weights excluding feasible vertices. Project
onto the same global force/moment/nonnegativity constraints, with no previous
contact, smoothing, gain change or additional actuation. The existing static
≤20 N, local ≥20% reduction/five-tick bound, and three-heading 2+10 s screen
remain mandatory. Reject at the first failing stage.

H20 passes static/local checks (16.43129 N jump, 92.05% first-response reduction,
following five ticks ≤0.10579 rad/s). Its arm-damping combination passes both
speed bounds at every heading but fails foot drift at 0.01310/0.02018/0.01965 m.
It is rejected as a complete repair. The first static implementation run caught
a missing independent-metric branch via the unchanged force/moment assertion;
that diagnostic implementation error was corrected before any runtime run.

## H21: test stance-reference retention with measured patch moments

H20 reduces drift and meets the speed limits while still adopting each small
foot displacement into its neutral stance reference. H14 rejected reference
retention with hindfoot-only support; test the same interaction now that every
patch's measured moment is represented. Compare H20 plus arm damping with the
identical candidate holding idle stance references from initialization. Keep
all contacts and eligibility measured. Require the same 2+10 s limits at all
three headings, exact replay of the H20 control, and unchanged structural gates.
No gain changes or tuning; reject on any failed candidate heading.

H21 is rejected. Every H20 control trajectory replays exactly. Holding intent
leaves foot drift 0.01462/0.01467/0.01987 m, and the −π/4 angular peak is
0.54511 rad/s. Every physical model, setting, threshold and production source
remains unchanged by H12–H21; all modes are diagnostic wrappers only.

## Evidence

The [H12–H20 manifest](checkpoints/2026-09-26/h12-h20-evaluation/manifest.json)
contains 78 verified compressed artifacts, including reports, peak trace
windows, contact-readback calibration and source snapshots. Full long event
traces remain local-only at their recorded paths and hashes; full response
histories remain in the reports. H20/H21 event capture was bounded to ticks
175–181 to limit disk use, while every frame remains represented in the
response report. The archive marks its extracted trace scope explicitly.

The [H21 and impulse manifest](checkpoints/2026-09-26/h21-impulse-evaluation/manifest.json)
contains 96 further verified artifacts. See [evidence qualifications](evidence.md#h12h21-continuation-and-small-impulses)
for intermediate-script and full-trace availability. The
[targeted validation](checkpoints/2026-09-26/continuation-validation-02/manifest.json)
passes all 24 selected tests and targeted lint, including force/moment/ownership,
duplicate-witness invariance, and preservation of an already-feasible measured
patch wrench. It does not establish physical standing acceptance.
