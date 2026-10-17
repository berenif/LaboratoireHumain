# Contact-aware landing MVP

Goal: GetUp followed by an unassisted physical step with at least15 mm actual
whole-foot capsule clearance, at least50 mm completed unload–land placement,
verified supported landing, then at least5 s native anchored standing. Existing
GetUp weights, body, joint limits and default runtime stay unchanged. This is an
experimental model-based hybrid, not a newly learned policy.

## Current causal test

The new opt-in `unilateral-soft` contact mode replaces the old hard normal
acceleration equalities with bounded contact active/release enumeration. At the
25 ms decision horizon, each active point may retain existing penetration but
not worsen it: min(currentGap,0) <= predictedGap <=0. Inactive points have all
three planned force components exactly zero and predictedGap >=0. Original
normal/tangent velocity stabilization is a soft task. Floating dynamics,
friction, joint/represented-action/motor limits and native physical gates are
unchanged. No mathematical contact constraint is applied to the engine.

This is a constant-acceleration receding-horizon surrogate. Native Rapier has a
soft biased pass used for position integration and a rigid bias-free final
velocity pass. Its one-pass CFM fixed point is not an exact equation on final
reported velocity or previous-frame total impulse. Consequently, no ERP-only
upward velocity target is added to the already stable, compliant quiet pose.

New read-only diagnostics resolve paired foot/floor anchors at current poses,
separate live gap from cached narrow-phase distance, and expose frozen solver
lever velocity and native frame/internal-substep parameters. On the retained
rolling failure, true gap at the opening point is−84.2 micrometres, rather than
its approximately−199 micrometre world height. Its free crossing time is about
0.77 ms; this is shorter than a1.25 ms native frame. This observation alone
never forces release: acceleration and predicted-gap feasibility select modes.
Normal impulse divided by native frame dt reproduces actual measured load;
internal solver dt must not be used for converting impulse to force.

## Preregistered sequence

1. Pure quiet and rolling-state replay: bounded motor/residual, exact original
   equalities after rank reduction, friction/gap constraints, zero inactive
   force leakage, valid reaction signs. Preserve old near-stop approximation.
2. Zero-shift12 s from the retained quiet checkpoint, per-process60 s physical
   wall cap. Require at least5 s anchored native standing in the full-model
   1–6 s window, no flight/non-foot support, loaded slip <=10 mm and maximum
   COM drift <=20 mm. Recovery under fallback does not pass the model hold.
3. If zero qualifies, repeat the same measured-transfer/swing/landing schedule
   and seed as the previous partial-step trial, cap90 s physical wall. Keep the
   complete landing/clearance/displacement/5 s final-standing requirements.
   A solver or support rejection is a failed/incomplete trial, not success.

Raw protocols/checkpoints and the aggregate ledger remain local under
`outputs/contact-landing-mvp/session-20261017T164129Z/`; compact audited fixtures
are in `docs/fixtures/getup-contact-modes.json`. Never republish the prior large
raw archives. First working-batch resource review is due17:41:29 UTC; further
work follows a fresh bounded hypothesis and aggregate accounting.

Pure checks:

```
OPENBLAS_NUM_THREADS=1 python3 scripts/getup-contact-dynamics-test.py
OPENBLAS_NUM_THREADS=1 python3 scripts/getup-contact-step-test.py
```

A fresh physical run requires an explicit future deadline and a separately
bounded process budget; old configs and fixtures are evidence, not permission.

## 2026-10-17 first working block: measured outcomes

- Unilateral mode zero trial: the predeclared1–6s native hold gate failed at one
  800Hz sample. The same complete12s run later held8.2475s continuously, with
  no flight/non-foot support and1.28/1.25mm loaded slip. The fixed gate remains
  failed; a separately declared exact-state readiness chain used the later hold.
- Unilateral transfer from that state stopped at9.5s without qualified unload.
  Capture stayed11.2mm from the right support line. No foot-lift success.
- A separate composite candidate retained hard-normal/soft-tangent contacts and
  added sole-localY→worldY angular tasks. Its final3–8s contained4000 native
  standing samples. Both feet really acquired two capsule tracks and their own
  noncollinear support hulls. Slip was1.57/1.94mm. This is a flat-support proof.
- The exact flat physical state, contact cache and clock were preserved for a
  fresh declared step trial; its COM target came from the new actual right
  hull (19.7mm minimum edge margin), not the old outer-track target. It lifted
  and moved59.6mm with24.6mm true capsule clearance, but never landed: one
  native flight sample and22.69mm stance loaded slip reject the trial.
- Exact capsule distance checks exclude foot overlap in that trial: minimum
  actual surface separation28.57mm; projected footprint separation27.72mm.
  The left/right feet are not an excluded self-collision pair.

The rejected flat step exposes both hard-normal contact-mode mismatch (planned
load at positive live gaps) and a held-PD actuation mismatch. The next isolated
opt-in ablation keeps that candidate's QP/contact/geometry/tasks unchanged and
advances its inner q/dq reference at800Hz along the solved acceleration:
qref=q0+t*dq0+0.5*t²*qdd; dqref=dq0+t*qdd. Torque is feedforward plus the original
unit position/.2 velocity impedance, clamped through the existing actuator path
at the original gear caps. There are no physical pose/velocity writes. The
experimental profile is reestablished by each explicit25ms command and is not
promoted to production checkpoint/controller state. A pure Rust20-substep
feedforward/cap test and18 Python dynamics/phase tests pass; native zero-budget
API validation admits zero integrations. Physical profile-zero passed: all6400 samples/8s native anchored standing,
no flight/non-foot support,0.205/0.313mm total loaded slip,1.072mm maximum COM
drift. Both per-foot two-track hulls persisted. Across320 decisions, first
native torque matched feedforward within2.4e-6Nm and final native torque matched
the full predicted-reference formula within6.3e-6Nm. The same exact flat seed
and step schedule are now tested with only this actuation change.

The profile-only step readiness trial stopped without qualified unload at9.5s.
All7600 native samples remained anchored standing, with no flight/non-foot
support and3.38/0.64mm loaded slip. Actual COM moved99.39mm and capture was
inside the measured right hull, but the left still carried24.46N (right93.9%).
The QP predicted25.07N left load, agreeing with that remaining physical support.
No lift was attempted. This rejects the complete step, while demonstrating a
cleaner transfer than the earlier held-reference trial. Through six trials the
ledger is153.14664s physical wall/45,140 native integrations; first-block resource
review remains17:41:29UTC. Next hypothesis is gap-aware release on the now-flat,
profile-driven controller, pending exact-state replay and a fresh bounded batch.

## Measured unloading and contact-set follow-up

A0.5s bounded left-load ceiling, activated only after measured right-hull
COM/capture support and sustained right load, achieved actual zero-left support.
The first lift reached21.47mm capsule clearance but drove the swing knee toward
extension without braking reserve. An opt-in position/velocity stopping-distance
guard now uses the existing50rad/s² model bound; this is not proof that the
plant always has that braking capability. Original native joint/torque limits
are unchanged. A local equality-RHS shadowing bug in its first offline residual
check was fixed and covered by a full-solve test before any guard rollout.

The guard-only rollout stopped on solver status during lift. Its candidate was
hard-feasible, but remains rejected; no physical acceptance was promoted. A
frozen-pose static calculation independently showed the old stance target would
need128% of the original ankle cap. This does not prove dynamic impossibility.

A new explicitly composite reference solves full-body planted-foot IK and real
static force/motor balance, selecting an attainable heelward COM point inside
the measured stance hull. Its terminal static motor requirement is79.5%; all310
synthetic reference states admit the unchanged hard-normal QP below80.7%, with
positive model braking reserve. PCHIP geometry is checked on the actual runtime
curve. Prescribed-path inverse dynamics is labeled an approximate planning
capacity diagnostic, not an exact hard-contact trajectory certificate.

The22s diagnostic horizon is bounded separately (default20,max40) and checkpoint
schema2 preserves it while schema1 remains readable. This changes no physical
qualification predicate. The new reference rollout stayed standing10.725s with
0.73/0.40mm slip but never qualified unloading: the stance sole rolled1.44°,
losing a support corner and putting actual COM/capture15mm outside its current
right-only triangle. No capacity readiness calculation or ceiling was triggered.
Hard-normal acceleration rows prevented the angular task from correcting that
steady tilt. This is a measured contact-mode obstruction, not successful stepping.

The next declared numerical/contact ablation uses the same references and caps
with exhaustive unilateral contact modes. Official Clarabel0.11.1 is an opt-in
research QP backend; SLSQP remains default. Its frozen-state objectives agree
within3.3e-7 but near-degenerate controls are not identical. Full original
unreduced equalities, gaps, friction and motor/box bounds are still checked.
An8s native zero hold is required before another full step. No browser controller
port or default GetUp change is claimed here.

The Clarabel/unilateral zero case (B10) remained anchored for all 8 seconds,
with no flight or non-foot support and 0.23/0.26 mm loaded slip. Its original
both-foot flatness prerequisite FAILED: the left foot ended on one capsule
track. The right foot retained two tracks and a noncollinear support hull for
its final 1.35 seconds. A subsequent deliberately one-sided stepping protocol
may use that right-foot prerequisite; it must still qualify actual right-only
COM/capture support, measured left unloading, capsule clearance, landing and
five seconds of native anchored balance. This does not retroactively pass B10's
both-foot geometry gate. B10 used 21.0844 physics-wall seconds and 85.56 total
process-wall seconds. Cumulative physics after B10 is 252.0252 seconds.

The optional research backend is reproducible with
`python -m pip install -r scripts/requirements-balance-mvp.txt` (Clarabel 0.11.1).
The production runtime and default GetUp do not require Python or this package.

B11's one-sided replay retained the right four-point hull and reached sustained
actual left unloading. It stopped before applying a lift command because two
independent readiness timers were out of order: actual-pose motor capacity first
fell below 0.9 at 9.500 s (0.8214), after the real stance hip reached its eligible
upper stop, but only 25 ms of its required 100 ms had accumulated when the older
unload timer requested lift. No flight/non-foot support occurred; loaded slip
was 1.77/1.09 mm. This does not qualify a step.

B12 changes only phase sequencing: transfer continues with the existing measured
unloaded-foot hold until both the original unload gate and the full current
support/capacity readiness gate have qualified. The bounded timeout and every
physical acceptance threshold remain unchanged. Eleven pure phase tests cover
waiting and timeout behavior. Six native balance pure tests pass, including the
bounded diagnostic horizon and checkpoint-compatible default.

B12 synchronized readiness and physically cleared the left foot by 30.49 mm,
with no flight/non-foot support and 1.77/1.86 mm loaded slip. It entered early
translation but stopped at 10.050 s before placement or landing. Elbow 17 was
30.24 microradians above its lower stop and closing at 0.0639 rad/s, outside
the old low-speed stop approximation. The preceding QP had planned only
2.8e-10 rad future stopping reserve; a 10.65 microradian native position error
exhausted it. Native internal joint-limit impulses have no state writeback,
and the native limit law activates only after crossing the bound. No existing
zero-valued impulse field is claimed to measure this impact.

The next one-variable robustness test adds an opt-in 100 microradian buffer to
FUTURE free-joint stopping positions only. Current actual-state checks, original
limits, the model's 50 rad/s² bound, near-stop eligibility and all motor/contact
constraints remain unchanged. The default buffer is zero. At the exact prior
elbow decision, the buffered original-cap QP is feasible and changes elbow
braking acceleration from 0.4909 to 0.7827 rad/s², leaving at least 100 microradians
predicted reserve. The buffer is an engineering margin, not a certified bound
on all native prediction error. The already-unviable stopped state remains
rejected; there is no retroactive state correction or pose reset.

B13 was externally interrupted before a result. Its durable trace measured
4.3075 physics-wall seconds; accounting reserves another 5 seconds / 20 native
substeps for a potentially unrecorded final command. The single same-seed retry
used only the remainder of that original allowance. It stopped safely at
4.750 s on a representation edge before any lift: ankle 7 was exactly one
native f32 neighbor below its lower limit and moving back inside. No flight or
non-foot support occurred; slip was approximately 0.56 mm per foot.

The numerical correction matches all 21 analysis bounds to the original native
f32 limits. A one-neighbor current observation is accepted only when separating
back inside, with no stop-force reaction. Zero/closing velocity and larger
violations remain rejected. Predicted motion and its future braking reserve
still use the unchanged native range. No physical bound, actual pose or velocity
is modified. Pure native assembly verifies every installed bound and its JSON
round-trip; frozen replay verifies the separating ankle has no stop reaction.

B14 resolved the numeric ankle rejection but stopped at 10.725 s without
qualified unload. The right support hull lost a corner, leaving measured COM
and capture approximately 23 mm outside the remaining hull. The capacity gate
and terminal force ceiling never activated. No flight/non-foot support occurred;
loaded slip was 1.41/1.08 mm. The earlier favorable unilateral trajectory is
therefore sensitive to small numerical changes and is not a robust step MVP.
The next investigation compares actual native state/torque/contact evolution to
the controller prediction; no additional target or gain change is admitted by
this result.

B15 compares two independent 25 ms forks of the exact saved B14 physical state,
including native contact caches. Only the mathematical contact-force Jacobian
changes: current geometric contact points versus native frozen solver levers.
Live material-point Jacobians still drive gaps, velocities and motion tasks.
Measured right-foot angular acceleration X/Z changes from −0.0315/−0.0137 to
−0.8755/−0.3386 rad/s², versus planned −0.7744/−0.3013. Tilt falls from 1.2147°
to 1.1810° rather than 1.2070°. Neither fork has flight/non-foot support; total
physics wall time is 0.1517 s for 40 native frames. Per-point tangent impulses
sum to the native aggregate within 5.9e−10 N·s. Yaw and pose-transport versus
final relaxed velocity discrepancies remain unresolved. This is a causal
local-model correction, not a completed step. B16 therefore changes only this
force map in the same-seed full trial, retaining all physical success gates.
