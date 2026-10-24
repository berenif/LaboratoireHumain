# Local arm-pull first-step experiment (24 October 2026)

Opt-in only: `?pull-step=local` on loopback, then select Balance. Main/site and
ordinary Balance, passive and GetUp remain separate. This is not continuous
walking. The preserved old foot-cycle is not invoked.

Causal hypothesis: the former timer/angle cycle had no horizontal foot destination
and did not coordinate the joint shape with the measured support-relative COM.
A native-Jacobian reference for both tasks may transfer load and permit a first
step toward a real arm pull. Failure to unload, track the sole clearance or hold
the retained foot would refute this candidate; pure FK is not its validation.

Implementation uses existing native `mj_jacBody`, `mj_jacSubtreeCom` and Cholesky
routines, on separate MjData. The relative Jacobian removes common rigid root
motion, including support-frame rotation. No physical pose, velocity, contact,
model, motor limit or friction is changed. Fifteen existing trunk/leg hinges
coordinate COM XY, swing-foot position/orientation and torso orientation.
COM height is a secondary posture objective within30mm; it is not an obligatory
constant-height constraint. Iterative FK, active-ROM columns and monotone Hermite
interpolation verify a finite path. Quintic phase progress starts/ends smoothly;
intermediate FK nodes are not stops. Reference speed is measured, not claimed
as physical actuator authority.

The measured-shape trajectory qgeom is distinct from compliant servo reference
`qservo = nominalBalance + qgeom - qentry`. This retains Balance's existing PD
preload instead of cancelling it by capturing qactual. Servo ROM projection and
its residual are exposed. The COM feedback retains the initial equilibrium
offset and follows the same relative displacement in the measured stance frame.
The existing gains/caps and single torque owner remain. Total requests transition
from current nominal Balance over25ms, before final saturation. Ankle feedback
share follows measured contact load with50ms filtering, not a 1/1→2/0 phase jump.
This approximate preload does NOT establish that qgeom will be realized.

## Frozen single-run protocol

- One reset, Balance10s (8000 ticks); no other physical preparation.
- Existing `beginGrab/moveGrab/release` on left forearm body15 at local
  `[.09,-.09,.09]`; spring120N/m, damper12Ns/m, original180N force cap.
- Move the target WORLD+X by250mm using a smooth ramp over0.5s; keep its initial
  Y/Z. Hold that same world target, release3.8s after grab. No other force/push.
  This is a reproducible pointer-target profile through the actual grab force
  path, not an independently prescribed force or a mouse-input/mobile QA test.
- Forward traction>8N, dominant over lateral, AND capture proxy advances8mm
  from quiet COM or actual vx>0.02m/s triggers the plan. The capture proxy is
  indicative, not a viability proof. Right support/left swing for this first
  forward-only case; no alternating-step claim.
- Destination advance is clamped40–90mm from measured COM/vx/traction at trigger.
- Transfer nominal1.8s, unload≤2.3s. Left load<15N and right>100N continuously75ms
  before lift. Actual native contacts are never removed.
- Lift aims20mm whole-sole clearance; require actual minimum≥8mm and left load<1N
  for50ms. Lift deadline0.7s; transport0.6s; lower0.5s/reception deadline0.8s.
- Loaded reception>30N for50ms, return0.6s. Release before lift cancels and returns;
  in-flight release finishes landing. No new step on release.
- Success: observed unload, entire-sole clearance, ≥30mm advance DURING flight,
  loaded reception, both feet>30N, tilt<0.2rad and COM speed<0.03m/s continuously
  for the final1s after release. Sliding cannot substitute for flight advance.
- Retained foot drift>25mm, retained load<1N for25ms, nonfoot support,
  tilt>20°, existing measured fall, nonfinite/cap anomaly or phase deadline stop.
  Whole run≤16s/12800 actual ticks, forecasts0; interaction/calming≤6s.
  One attempt; no post-result tuning or physical retry.

Loads are from the last resolved substep; geometry is refreshed separately for
measurement and labelled. Joint torque actually transmitted is checked against
gear×ctrl and physical caps. Root generalized actuation and applied generalized
forces must remain zero. The only external body force is the disclosed grab.

## Validation scope

`node scripts/verify-mujoco-pull-step-pure.mjs` performs zero integrations,
including baseline command equality, relative-Jacobian finite differences,
finite path/interpolation/ROM, entry continuity, release and pre-integration
failure. It does not establish single-support or contact feasibility.

`node scripts/run-mujoco-pull-step.mjs <new-proof-directory>` is the exclusive
single-run runner; never invoke it without a separately authorized runtime.
It saves counters/poses progressively and a complete final native state. Proof
budget2MB; no media transfer or publication. Historical FAIL results remain.

## Observed single-run result — FAIL before transfer

Source40d62457dcaf22f96b3e9908f6ee7b978f944282. The only physical run stopped
at10.24375s /8195 ticks (8000 standing +195 with grab). The trigger did react to
real forward traction12.5995N and COM vx0.02057m/s, but the finite-path ROM check
rejected the measured initial shape before a transfer command was applied.

The native soft joint limits had allowed right/left knees to2.05042° /2.05698°
against the nominal2° bound. The input pose at path parameter0 is required to be
exactly the measured pose; the strict nominal-ROM assertion rejects that input.
This is a demonstrable initialization/validation mismatch, not evidence that
single support is physically impossible. The pure tests used a nominal-ROM seed
and missed this observed entry. No guard, gain, controller or cap was changed
following the result, and no retry was made.

At stop: COM+1.218mm forward since grab, torso tilt8.081°, right/left loads
195.840/203.680N; minimum whole left sole−0.748mm (native contact penetration).
No unload, lift, flight, placement or reception occurred; retained-foot drift
was0.038mm. Requested/applied motors stayed within the original caps; initial
reset landing had reached caps, while the largest at stop was60.42% of its cap.
No nonfoot contact or physical fall. The arm force remained an explicit user
grab, with no root/generalized assistance. The FK trajectory remains unvalidated.

Local proof directory `web/proofs/pull-step-001/`: progressive compressed trace,
ledger, exact native final checkpoint (pure state restore verified), diagnostic
and one inspected contact sheet. The sheet shows the full mannequin and floor,
with actual positions at0,10,10.05,10.125,10.2,10.24375s; it is labelled replay,
not a new simulation. No video/export/publication was produced.

MuJoCo confirmed total228054 PLUS the preserved prior unknown run; Rapier45735
unchanged. Main, remote branches and the private deployed site are unchanged.
Only this local experimental branch contains the candidate and its failure.

## Prospective correction and restored continuation 002

Only the entry validator changes. A measured geometric anchor may contain the
exact nominal-ROM excess already observed; the first Hermite segment must reduce
that excess monotonically, reach strict ROM at its first solved node, and never
introduce an excess elsewhere. The full cubic extrema and derivative extrema
are checked analytically on every segment, including between sampled FK checks.
The explicit anchor must equal the first configuration. All subsequent paths
remain strict. This exception does not apply to motor references, which remain
ROM-admissible from entry. Physical q/dq and native limits are never projected.
The desired shape starts with zero time derivative via the existing quintic;
this does not erase the outward physical knee velocities used by PD damping.

Exact001 checkpoint SHA256
`bdc3c9bd7167a8bc23ea583b55628d0c8d5ee878dd7a6fe3336a03bac41c743c`:
233 native integration values restored and compared exactly. The entry path
residual is0.327260mm, global speed bound1.365378rad/s (dense observed1.214929),
piecewise acceleration bound57.726326rad/s². These finite reference bounds are
not physical actuator authority or a claim of exact following. Intermediate
points remain C1 continuous; no physical velocity cap or gain is increased.
The complete entry command equals current Balance at the same restored state;
its largest difference from the preceding physically applied command is0.062205Nm
(the state has advanced since that preceding command). Physical knee damping
contributions remain−0.656052/−0.679578Nm. Six adverse ROM/nonfinite fixtures
are rejected. Future-reference command snapshots on the unchanged observed
state are finite and capped; their largest request69.770Nm is not a prediction
of future physical torque demand.

The old native snapshot does NOT include grab.previous. Continuation002 explicitly
starts a new grab point history on the same body15/local[.09,−.09,.09]. During
20 ticks (25ms), its external arm force is the convex quintic blend of the last
recorded force[12.599545569839282,0.69783056692452,0.12339019719056488]N and the
unchanged current grab law. The first newly computed point velocity uses native
Jacobian×physical qvel; later values use successive real point samples as before.
At tick0 the transmitted force is exactly the recorded force, at tick20 it is
entirely current grab. Both endpoints and the convex blend obey180N. No force is
applied to the root or another body. This is an authorized different interaction
entry, NOT a reconstruction of missing history or bitwise historical continuation.
Target timing, point, gains and caps remain unchanged; release at absolute13.8s.

`verify-pull-entry-pure.mjs` performs no integration. After it and the prior pure
checks pass, `continue-mujoco-pull-step.mjs web/proofs/pull-step-002` is authorized
once: at most4605 new native ticks, zero forecasts, absolute end16s. No new10s
preparation. Transfer entry/deadline remain10.24375/12.54375s; all physical stops
and observable step criteria above remain unchanged. The failed001 evidence is
immutable. The new checkpoint also retains full grab/bridge metadata. Proofs
are progressive and limited to2MB, including a geometry replay contact sheet.

## Observed continuation002 — FAIL, tilt stop before unloading

Tested source `5ad172d`. Exactly740 additional native ticks, zero forecasts:
10.24375→11.16875s (0.925s). Scenario counter8935; confirmed MuJoCo total228794
plus the preserved unknown historical run. Rapier45735 remains frozen. No retry.
The233-value native restoration passed; the declared25ms arm-force entry was
used. The original001 checkpoint and FAIL remain unchanged.

The corrected geometric reference was admitted and transfer executed. The
measured torso tilt grew8.081→20.009°, triggering the unchanged20° stop before
release or unloading deadline. The body's ground-fall flag remained false:
this is loss of acceptable balance, not an observed impact on the ground.
COM moved+162.225mm forward,34.861mm toward the retained right side, and29.658mm
down; final forward velocity0.387154m/s. The left load first increased to a
sampled226.739N and never reached the<15N unload criterion (minimum187.162N).
Final right/left loads179.129/187.162N. The entire left sole never cleared the
floor (best−0.748mm, final−0.986mm); retained-foot drift≤2.132mm. No flight,
placement, loaded reception, release or final calm were reached.

Both sagittal ankles saturated under the unchanged20Nm caps. Final total
requests146.020/154.493Nm were applied as20/20Nm. Saturation is visible by the
10.4s trace sample; its exact first tick/duration was not recorded. Sampled grab
force maximum23.959N, only on the forearm; no generalized/root support force.
All21 actual torques were checked every executed tick for finitude, cap and
gear×ctrl equality. This is an applied-cap PASS, not exact following of requests.

Observed cause of termination: forward lean/speed grew with feet planted while
the sagittal motor requests exceeded capacity. Hypothesis, not established
uniquely: this transfer does not unload soon enough for the imposed pull before
fixed-foot balance authority is exhausted. Neither a global inability to step
nor a gain adjustment is justified by this one result. The validator correction
is verified; mechanical step success is still absent.

`web/proofs/pull-step-002/` contains the progressive ledger and40 trace rows,
compact review, exact final native checkpoint and one inspected six-frame replay
sheet. Full body/ground remain visible from entry through increasing forward
lean; both feet remain planted. No additional simulation, video, media export,
push, deployment or controller adjustment after the result.
Final gzip SHA256:
`e00b3428d6ec07351f1f869641ff7990bf11a109086578ba9b73c04a9d30c9fd`.

## Load-feedback candidate — rejected by the pure dynamic gate, no trial003

This implementation remains opt-in (`loadFeedback:false` by default), local and
unpublished. Stable Balance and SLOW commands are unchanged. It records solved
contact positions, normals and full forces, and proposes a feedback correction
through the existing PD reference, not an external force. For ground reaction
changes δf, δτ=−Jcontactᵀδf and δq=Kp⁻¹δτ on the21 actuated axes. Root components
are reported and never applied. Existing ROM and final motor caps remain active;
the reference offset is filtered on the already declared25ms entry timescale.
There is no integral accumulator. Existing PD gains, friction and motor limits
are unchanged; this candidate is explicitly a new load-feedback law.

The approximate lateral goal uses the current COM height's natural frequency
ω=√(g/h), acceleration ω²(yGoal−y)−2ω·vy, and the measured arm force/moment.
A zero roll angular-momentum-rate target gives a desired CoP using the ground
lateral force required by that acceleration. Normal loads are projected to the
present two-foot resultant segment while reserving100N on the retained foot.
The implied acceleration is recalculated after projection and reported apart
from the unrestricted goal. This is an approximation, not a physical guarantee.

**The native pure check rejects this normal-only command formulation.** On the
exact001 q/dq at10.24375s, using recorded COM velocity and frozen recorded grab
force in both compared commands, the projected goal is ay=−0.529679m/s² (right).
The desired right/left loads are100/299.498N; full correction predicts native
94.667/304.810N, so it DOES affect load distribution with the intended sign.
But predicted COM ay is+0.099248m/s² (left), versus+0.014576 without correction,
and roll angular-momentum rate grows0.363299→22.741934Nm rather than approaching
its requested zero. The first filtered update also shifts the wrong way:
ay+0.018705m/s² and roll moment1.454718Nm. These are fixed-state evaluations,
not a forecast or an observation of a future fall/25ms transient.

The target's required tangential reaction is not commanded by the normal-only
correction. Opposite vertical-force changes sum to zero but create18.437Nm of
roll moment. Dropping the six unactuated components does not make that moment
vanish. Native MuJoCo resolves a physically consistent response that differs
from the requested task. The six independently recomputed base-equation
residuals are below8.2e−13; no generalized/root force was applied. Virtual-work
sign/unit error is below1.4e−17. All21 native evaluated motor efforts remain finite/capped;
contact friction ratio≤0.117 and servo ROM checks pass. This therefore is a
command/task failure, not solver nonconvergence or proof that walking is impossible.

002 is independently rejected by the existing20° physical stop. A preview beyond
that guard would expose nominal Balance commands which the runtime would NOT
integrate; these are not reused as a valid continuation. No trial is launched
from either checkpoint. The placement-retarget implementation is left pending
because its required support-exchange primitive failed this gate; updating a
foot destination cannot repair this demonstrated roll/force mismatch.

`node scripts/verify-pull-load-pure.mjs` tests the two saved states, normal-only
virtual work, projected moment/force demand, complete native response, friction,
ROM/caps, absence of a load command on missing contact, and physical-state
immutability. `web/proofs/pull-step-load-pure/result.json` is the compact result.
The historical relative-FK/entry/Balance/policy/package pure checks still pass.
No integration or forecast was performed; MuJoCo stays228794 confirmed plus the
preserved unknown historical run, Rapier45735. No gain sweep, tangential solver,
new physical trial, media export, push or deployment follows this rejected gate.

## Coupled native-response witness — 44 evaluations, zero integration

`scripts/verify-pull-coupled-pure.mjs` is a standalone diagnostic, **not wired
into the controller**. It evaluates one baseline, 42 central motor perturbations
and one candidate on independent copies of the exact001 native233 state.
Each copy includes the same warmstart; the current recorded arm force is applied
at body15's same material point. No root/generalized force is added. `mj_forward`
computes the native normal/tangential reactions and all27 accelerations; no
`mj_step`, transition finite differences, forecast or physical integration occurs.
The native saved ctrl is a later unintegrated proposal, not the last applied
command. After exact restoration, the diagnostic explicitly installs the last
applied command from the ledger; the maximum difference is0.032659Nm.

The objective was fixed before evaluation. Outputs are COM acceleration xyz,
centroidal angular-momentum rate xyz and right/left normal loads. Scales are
g, mgh and mg (h above the current contact plane); task weights are1 for the
six centroidal components and0.25 for the two loads. Lateral acceleration and
load goals retain the previous declared demand, roll moment targets zero, and
the other centroidal outputs retain their baseline values. Motor variables are
gear-normalized controls. Regularization is1e-4 times squared control change;
central perturbations are0.001. One Cholesky solve supplies one direction,
scaled analytically by0.473909 to respect a fixed0.1 control trust radius and
the unchanged caps. This is a local limited least-squares direction, not a claim
to solve a general bounded optimization or a new feedback controller.

The sole candidate passes all declared local gates. Native COM ay changes
+0.014596→−0.242923m/s² toward the right support, against a−0.529679 goal
(47.314% of the requested change). Centroidal roll-moment rate decreases
0.350796→0.184064Nm. Ground lateral force changes−0.101661→−10.619803N;
right/left normal loads change195.817/203.668→150.437/249.048N. Thus this is
an initial propulsive redistribution, **not left-foot unloading already achieved**.
The largest changes are left/right hip-x−4.000/+3.998Nm and abdomen-x−1.744Nm;
sagittal ankle changes are only+0.00539/−0.00318Nm. The full21 commands and
actual native efforts are saved in the result, with maximum effort60.405% of cap.

The independently assembled27 dynamic residuals are≤1.231e-13, including the
six free-base equations. Friction utilization is≤0.19914. All eight contacts
remain active, with the same native constraint regime for all44 evaluations.
Normalized nonlinear prediction error is6.122e-15, below the predeclared0.009577
limit. Task cost decreases0.0102285→0.00283592; regularization cost is2.230e-6.
Terminal002 is still rejected at20.009° by the unchanged controller guard,
using kinematics only and zero direct dynamics evaluations.

**This PASS is only a fixed-state command witness.** The native right-contact
normal accelerations are+0.175…+0.601m/s² and left-contact values
−0.592…−0.180m/s². These compliant-contact accelerations are not a prediction
that support persists. Root local angular acceleration x increases
0.0802→1.2763rad/s² despite the smaller whole-body roll-moment rate; centroidal
moment control does not by itself establish upright torso control. Neither a
future motor transition, unloading, flight, placement nor reception was tested.
The candidate must not be described as a completed support-transfer controller.

Evidence is `web/proofs/pull-step-coupled-pure/{protocol.json,evaluations.ndjson,
result.json}` (49,535bytes). All44 begin/end records are durable. Native solve
time totals11.146ms (maximum3.268ms), whole diagnostic386.937ms on this cloud;
this is not a browser/phone control-rate benchmark. Existing evidence refuses
a duplicate execution. MuJoCo remains228794 confirmed plus preserved unknown
history, Rapier45735. No runtime controller, main, site or dependency changed.

## Constrained witness — algebra feasible, native regime guard rejects

The next local diagnostic (`verify-pull-constrained-native-pure.mjs`) reuses
the complete saved baseline, eight-output Jacobian, scales, weights, lambda1e-4
and0.1 control trust radius. Exactly21 one-sided perturbations recover the
missing torso/contact sensitivities; every common output matches its former
sample exactly. There is no new baseline forward solve. The only additional
candidate brings the total to22 completed native forward evaluations, zero
physical ticks and zero forecasts.

The small active-set Cholesky adapter passes19 algebra tests (including a
contradictory system and an independent2D active-face enumeration). It finds a
solution in23 of64 permitted pivots. All92 linear inequalities pass individually:
two torso constraints, eight nonnegative normals,32 native pyramidal-friction
facets, retained load>=100N, four endpoint-gap guards,42 motor/trust bounds,
lateral progress and two global-roll bounds. Caps and trust are in the solve;
there is no final command clipping or scaling. The torso conditions are
thetaDD<=0 for the currently growing tilt and upDot·upDD<=0. The latter bounds
squared orientation speed, not physical kinetic energy. These are constraints,
not higher objective weights.

Gap jets use the actual capsule endpoint centres minus radius, including their
Jdot*qvel, rather than a permanently fixed contact material point. Their
1.25ms Taylor margins are a conservative local diagnostic only. Positive normal
acceleration is not rejected in isolation and no physical contact is welded.

**The candidate is NOT natively admitted.** The22nd forward calculation finishes,
then the full `efc_state` comparison fails, before candidate telemetry is saved.
There are22 durable begin records and21 end records; the catch result independently
records22 completed evaluations. This is a known completed call, not an uncertain
or retryable runtime. The exception text says `Native contact regime changed`,
but that array contains joint-limit rows as well as contacts.

A position-stage-only metadata read (no acceleration/force solve, qacc and all233
integration values unchanged) identifies row0 as the **right-knee soft limit**,
joint7. It changes QUADRATIC(1)→SATISFIED(0). The34 rows comprise two knee limits
and32 contact-pyramid rows. This invalidates the assumed constant constraint
regime; it does not establish foot support loss. The geometry signature comparison
had passed. The full final constraint-state array and force/acceleration telemetry
were not preserved, so no claim is made about all remaining native constraints.

Only the linear prediction is available: ay−0.286565m/s², tilt acceleration
−0.339359rad/s², right load135.895N and endpoint gaps−0.427…−0.725mm. These are
NOT measured native candidate results. The constrained algebra is feasible;
physical feasibility after the regime change remains undetermined. No other
candidate, parameter adjustment, forward retry or integration follows this stop.

`web/proofs/pull-step-constrained-pure/` preserves protocol, evaluation journal,
result and a saved-data audit, including the missing-telemetry limitation.
Native time totals7.393ms, complete diagnostic382.740ms on this cloud. MuJoCo
stays228794 confirmed plus unknown history; Rapier45735. Active controllers,
stable main/site, all previous verdicts and dependencies remain unchanged.

## Exact candidate telemetry repair — one native recheck, still rejected

The previous Node process had exited; its missing response was not recoverable
in memory. The saved baseline controls and 21 IEEE754 increments do preserve
the exact candidate: the same `u0[i] + saved.solve.x[i]` expression is evaluated,
without optimization, clipping or new perturbations. The input checkpoint,
model, prior result and sensitivity journal are hash-checked. All 233 native
integration values are checked immediately after restoration, before installing
the same command and recorded arm force at the same material point.

`verify-pull-constrained-native-pure.mjs --recheck-saved-candidate` performs exactly
one `mj_forward`, with a separate exclusive proof directory. It writes all native
forces, accelerations, actuator efforts, contacts, constraint types/IDs/states
and constraint residual inputs **before** the first comparison. No old proof is
overwritten. The old nonqualified result is preserved; this is an explicitly
authorized additional diagnostic, not a retroactive PASS or a simulation retry.

The eight contacts and all 32 ground-contact constraint states are unchanged.
Only the right-knee joint-limit row changes QUADRATIC → SATISFIED; its reaction
is zero. The left-knee limit remains active. This is not a foot losing contact.
The following are now native results, not predictions:

| Existing condition | Native result | Outcome |
|---|---:|---|
| Growing tilt: thetaDD <= 0 | -0.333019963 rad/s² | Pass |
| `upDot·upDD <= 0` | +0.000285776081 s⁻³ | **Fail**, tolerance 1e-10 |
| Eight normal forces >= 0 | minimum 26.335201 N | 8/8 pass |
| 32 pyramidal friction facets | worst residual -17.734491 N | 32/32 pass |
| Retained right load >= 100 N | 135.924908 N | Pass |
| Four current right endpoint gap jets | -0.724521 to -0.427486 mm | 4/4 pass |
| 42 motor/trust inequalities | largest violation 9.47e-15 control units | 42/42 pass |
| Lateral progress | COM ay -0.286637694 m/s² | Pass |
| Two non-worsening global-roll bounds | HdotX +0.345334055 Nm, baseline +0.350796094 | 2/2 pass |

Thus **91/92 inequalities pass; the candidate remains rejected**. The failing
quantity is half the derivative of squared upright-vector speed, not rotational
kinetic energy or full angular speed including yaw. It falls from the baseline
0.027054679 to 0.000285776 s⁻³, but stays positive. Its three dot-product
contributions are -0.014300040, +0.014863300 and -0.000277483 s⁻³. Decreasing the
scalar tilt acceleration therefore does not satisfy the separate orientation
speed condition. No tolerance is widened to accept this result.

The linear model put this condition on its boundary (2.64e-15 s⁻³). The native
error 0.000285776 is within the previously declared prediction-error allowance
0.005410936, while the native inequality itself fails. These are different tests.
All other feature prediction checks pass; normalized common-output error is
0.000481402 versus its 0.012236428 bound. The one observed joint-limit regime
change invalidates exact affine equivalence. Lower task cost (0.010228501 →
0.001624616) likewise does not qualify the command.

COM acceleration xyz is [0.248276773, -0.286637694, -0.025535670] m/s²;
right/left loads are 135.924908/263.588570 N. Largest motor utilization is
70.4211% (left ankle-y, 14.084219/20 Nm); no root actuator effort is present.
All 27 independently assembled dynamics residuals, including six free-base
equations, are <= 3.91e-14. Current right material-contact normal accelerations
are +0.2245 to +1.0600 m/s² with penetration and small signed normal velocities.
This is compatible with the soft-contact response at this instant; the local
gap jets do not establish persistent support, unloading or a step. Root local
angular accelerations are [5.444564, -0.283786, -26.123648] rad/s², also retained
explicitly rather than conflated with centroidal moment or upright-vector speed.

Evidence: `web/proofs/pull-step-constrained-recheck/`, 61,612 bytes. Native raw
SHA256 `1c537202749fce9115edd93620a078b1052e253eaf67eb7cb66a94fd753a4d3e`;
result SHA256 `e3d7b13b84cc503d2cedb7ebbe28ab3efdb8915b8b7c70981a1ae6837ae3d02a`.
`node scripts/verify-pull-recheck-recording.mjs` audits the saved response,
reconstructs the contact/cap/gap conditions and verifies that rejection did not
discard telemetry. It imports no engine and performs no native evaluation.

Cost: one completed forward call, 1.973 ms native / 214.707 ms whole diagnostic
on this cloud. Zero optimizer calls, perturbations, integration ticks or forecasts.
MuJoCo remains 228794 confirmed plus unknown history; Rapier remains 45735.
There is no activation in the controller, dynamic-readiness claim, push or site
change. This result neither proves physical impossibility nor supplies a safe
feedback controller; it identifies the remaining failed local condition.
