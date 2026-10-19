# Five-reviewed gait correction implementation

Status: experimental and opt-in. The requested final result is not achieved;
no stable preset, original-contact qualification or deployment is changed.

## Five actual reviews incorporated

1. Calibrate forward-positive thorax/pelvis axes and distinguish visual
   preferences from anatomical limits. Measure the mobile knee and respect the
   rigid two-capsule foot. Preserve opposition in the instantaneous thorax frame.
2. Keep the strict measured-arm gates. Prepare one immutable command for actual
   and prospective-unload QPs; admit changed references with current support,
   motor and ankle reserve rather than naive angle clamps.
3. Audit all native samples, global slip and same-world continuity. Distinguish
   own-foot advances from COM/body travel. Compare matching loaded-landings and
   actual-speed front/profile pixels.
4. Implement shared-reference parity, continuous COM/load progression, coupled
   posture, coordinated knee/foot motion, then cadence. Isolate causal changes;
   do not hide a six-step failure by stacking unqualified factors.
5. Preserve measured midpoint+35 mm loaded-settle and 80-sample readiness,
   distinct unload/airborne gates, idempotent histories and a separately declared
   audit profile for changed gait references.

## Unchanged physical and arm contracts

- Explicit impratio=8, friction=1, original geometry/strength; 800 Hz physics,
  200 Hz torque commands, motor bounds ±0.95 and global loaded slip ≤10 mm/foot.
- All original joint, collision, ground-ray, contact, support, capture and phase
  deadlines remain. No actual state installation after initialization, root
  assistance, slip reset or playback timewarp.
- Unload: swing <0.5 N, stance ≥0.8 BW, 80 consecutive native samples. Advance:
  whole-foot clearance ≥25 mm, swing <3 N, stance ≥0.8 BW, 80 samples.
- Robust support: ≥100 ms/21 causal commands, pressure/clearance agreement ≥0.8,
  three unique actual surviving contacts, original bounded correction and 3 mm
  measured reserve. Changed tasks acquire a fresh window; old admission is only
  provisional geometry.
- Loaded settle: actual received-feet midpoint+35 mm heading, unchanged height,
  1.52 s planned phase and original 80-sample measured readiness. No silent switch
  to loaded-landing handoff. Final quiet requires 4,000 consecutive samples.
- Delayed measured ipsilateral thigh drives near-body opposed arms at gain −1.
  Keep persistent origins/history, ≤0.5° maximum error, excursion ratio ±5%,
  correlation ≤−0.98, exact target/FK checks, no clipping and original map reserve.
- Freeze total COM speed/acceleration at 0.19260552331163003 m/s and
  0.3901818436690813 m/s²; do not widen them to admit a return path.

The delivered video shows four ≥180 mm own-foot advances, not ten, at about
6.66 s per loaded-landing interval. Ten advances never imply 1.8 m body travel.

## Implemented and measured checkpoints

Shared immutable references now prepare measured arms/contact-seek once per
command; both QP consumers retain subclass tasks and use that same context.
Native payloads, controls, histories, reference identities and sources are bound.

Continuous COM and earlier state-valid support selection pass independent
one-/two-step plus five-second quiet diagnostics. Earlier selection reduces the
first landing by 0.9975 s and alternating intervals from 7.63375 to 6.870 s,
still slower than the delivered video. The seven-step continuation stops at
48.215 s during seventh landing: six completed advances, global slip
10.00760/8.61442 mm, no final quiet. Endpoint projection and soft load-retention
trials are rejected; prior tangential damping/native-acceleration fixes are not
repeated. Stage-one tuning is closed.

The approved sequence revision allows isolated posture/knee pilots before that
seven-step ordering gate, with final acceptance unchanged. The first posture
handoff is rejected before application because its initial COM is outside the
selected triangle. The next completes a loaded step but rejects its old settle
quintic on the frozen derivative caps. A single analytical seven-cubic return
preserves the same measured target and 1.52 s duration; its independent whole
current-hull/capture certificate has 25.271 mm reserve.

The v3 posture pilot independently passes 12,402 native rows, 3,101 commands,
42 exact native witness replays and five quiet seconds. Loaded advance is
181.484 mm at 8.98 s; final after-quiet advance is 181.369 mm; slip is
1.418/1.180 mm. The complete settle owner absorbs the finished correction once;
a separate all-command proof verifies its diagnostic clock semantics. The first
clock-expectation failure and all immutable subproofs are retained.

This is only a physical checkpoint. Mid-advance pelvis pitch improves from
−30.53° to −17.37°, while torso tilt remains about 9.3° and landing is 1.85 s
slower. Quiet pelvis is −18.24° versus baseline −2.64°. The quiet task explicitly
requests −20.32°: the return mixes landing lumbar angles with persistent knee
goals and initial root orientation, rather than one foot/COM-compatible pose.
No overall visual, cadence or ten-step pass is claimed.

## Approved common-pose reference revision

Two finite moving-root/both-leg endpoint families fail; neither is installed.
The inherited weak-joint target is also off the simultaneous task-FK manifold
by 3.469 mm COM and 2.646 mm foot position. A declared C2 compatibility slack
must preserve the actual prepared command at activation, not project a pose or
zero velocities.

One stance-realized whole-body path now owns lower joints, both feet, COM,
thorax and pelvis jets. Nine independent lumbar/stance-leg coordinates drive an
implicit exact rigid swing-foot continuation. COM is free inside the unchanged
actual support/capture and derivative bounds. Root is derived by stance FK and
is never physically applied. Offline future-arm geometry is a prediction;
each actual command recomputes complete COM with the exact already-prepared
measured arm jets. Strong arms/gains/history, weak arm-tail preferences and
safety headers retain their owners.

The implemented flight horizon has 27 knot coordinates, original 0.8/1.2/1.0 s
clocks, stationary waits and exact held x/y through lift. Its fixed soft body1
and weak-posture preferences do not make 10° knee gain or neutral pelvis hard
anatomy. The first proposal has an explicitly recorded 0.6 mm landing-center
reference difference; it is not silently labeled identical to the old path.

At actual landing, a separate causal same-foot endpoint retains both current
feet and the controller-owned measured COM target. It derives root from stance
and keeps body preferences soft. The first endpoint matches geometry to
7.95e−14; two counter-zero native static rechecks require only 0.3057 motor
fraction and retain 76.57 mm clean gap. It misses the soft body preferences.
Its C2 return from the old posture landing is rejected at 0.2375 s: acceleration
0.43010 exceeds the unchanged 0.39018 m/s² cap. Endpoint feasibility is not
path/native admission. First-command lag of 0–3 native samples is handled with
current identity and only the remaining original 1.52 s deadline, not backdating.

An executable count-one opt-in wrapper now requires a bound passing proposal
before constructing a native world. It rechecks current finite analytic/static
admission, then acquires its own 21-command window. Missing endpoint/path,
non-rest unsupported handoff, changed seek ownership or live bound failures
reject; there is no hybrid rest fallback. Counts two/ten remain blocked until
explicit C2 cross-step ownership is reviewed.

## Current computational gate and next actions

One fixed 80-iteration flight solve was interrupted and verified stopped at its
20-minute compute ceiling, without a candidate. This is a numerical planning
blocker, not a physical/global infeasibility proof. A zero-optimizer profile
measures about 0.90 s per evaluation, dominated by repeated inner Newton IK;
27-coordinate finite differences cost about 25 s before line searches. Huge
initial acceleration violations cross an unconverged IK branch, not a proven
physical acceleration demand. Converged-point derivatives agree to 1.7e−8.

The timeout checkpoint is retained. Immutable entry reuse, phase-local caches
and a bounded exact-float64 candidate cache preserve all 33 measured objective/
832-constraint vectors bit-for-bit, including the invalid inner-IK branch. With
zero optimizer calls, a full 27-coordinate objective/constraint stencil falls
from 37.07 to 13.10 s (2.83×); the repeated constraint sweep takes 2 ms. Fixed
seeds, tolerances, targets, bounds and arithmetic order are unchanged. Detached
iteration packets retain unadmitted knots, residuals and cache counters; cache
hits still check the cooperative deadline/cancellation.

One identical-problem recovery stops cleanly at 180.002 s with 33 retained
iterations. It restores every prescreen foot-pose residual to about 4e−13 and
reduces final major-iterate slack to −4.89e−7, but does not converge or admit a
proposal. A zero-optimizer diagnostic of that exact final major path confirms
lift-75% analytic acceleration exceeds the cap by 5.47e−7 m/s², and the translate
endpoint misses the same reserved map boundary by 39.2 picoradians. Neither is
relaxed. A single approved minimum-norm feasibility polish uses analytic jets
and all original constraints around this exact retained path, then the original
strict 123 samples. Its explicit two-stage profile binds both failed receipts;
missing/downgraded provenance is rejected. The requested shape is visible only
in unadmitted math: 20.1° extra early knee bend followed by extension, with
38.6 mm sampled COM progression for the 182 mm foot command. The polish
converges in two iterations but rejects 12/123 strict samples: acceleration
peaks at 0.403917 m/s² between the sparse nodes and the support/capture minimum
is 2.89164 mm. No runtime binding or native world is created.

One approved bounded refinement retains the same 27 coordinates and exact
iteration-33 minimum-norm anchor. Three source-worst brackets supply current-candidate
scalar envelope constraints for lift acceleration and translate/land support;
the final certificate also checks the complete 200 Hz command grid and bounded
numerical extrema. An explicit envelope Jacobian reuses current minimizing
times for its 27 columns; rejected synthetic baselines cannot produce gradients.
Original clocks are hash-bound, and numeric evidence cannot
be replaced by pass flags. The first envelope refinement passes all 123 samples,
but three of 603 command-grid samples and global extrema reject a newly active
translate capture face: 2.999091 mm minimum reserve. It is not admitted. The
shared producer now proceeds to bounded face-aware exchange over the whole
translate phase, with deterministic continuation and the original objective.
Repeated FK-jet object validation is optimized separately; the original strict
FK owner remains the independent final certificate. The fast analytic COM
evaluator matches 15 archived/perturbed/arm/seek cases bit-for-bit and measures
7.09× faster with zero optimizers or native worlds. Whole-phase exchange covers
all 56 translate COM/capture face-quarter envelopes (890 total rows), reusing
the same profile and hash-linked deterministic history. Exchange one converges
in two iterations and passes the original 123 samples, all 603 command samples
and 219 strict numerical-extrema evaluations. Runtime preflight verifies the
bound proposal. The first native trial stops safely at counter 3,200 (4 s),
before applying any whole-body command: actual-state lift capture reserve is
2.95678 mm against the unchanged 3 mm requirement. Its final native payload is
identical to the retained pre-admission payload; all 32 frozen sources and an
independent targeted negative proof are retained. The offline first-lift witness
is not the actual stationary-transfer activation input. The same 27-coordinate
family and original objective now retarget that exact retained current state,
with all nine support faces covered across all three flight phases (216 closed
quarter envelopes; 1,102 total rows). Historical input certificates remain
historical. The next pilot must reproduce the bound actual state exactly and
pass current analytic/static/dynamic admission. Prior failures remain immutable.
The bounded actual-input retarget retains two complete iterates at its 300 s
solve cap; the 180 s certificate is incomplete and records a 7.56e−8 m/s²
acceleration excess and 4.64e−9 m reserve deficit. It is not admitted. Exact
duplicate-FK memoization precedes a deterministic same-input continuation, with
the original objective and bounds. The causal settle receipt now retains its
solved endpoint geometry and binds that geometry into path identity; no numeric
reference is changed by this evidence correction. No seed, goal or weight grid is permitted. Only a completely admitted
proposal proceeds to one native step plus quiet,
independent audit and actual-speed visual review, then alternating diagnostics
and the full ten-step test.

## Final acceptance

Require ten alternating own-foot advances ≥180 mm in one uninterrupted world,
then 4,000 quiet native samples; independently pass every unchanged physical,
arm, source/history and no-assistance gate. Also require faster loaded-landing
intervals and improved full actual-speed front/profile pixels: less waiting and
lateral sway, earlier knee flexion with controlled extension, coordinated pelvis
and slight forward bust, and visible near-body opposed arms. Report all phase
intervals, both sides, foot separation, COM progression/body travel and missed
preferences. A slower, physical-only or visually partial result is not final.
