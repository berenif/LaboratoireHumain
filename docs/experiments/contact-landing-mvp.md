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
