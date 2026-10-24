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
