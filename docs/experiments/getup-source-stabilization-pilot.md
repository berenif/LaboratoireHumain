# Source stabilization pilot, 18 October 2026

This is a bounded failed initializer experiment and a pure prototype. There is
no new physical forward-step or final-five-second standing proof. The source
MuJoCo engine is used only for experimental diagnosis/learning preparation;
the application's default engine and behavior are unchanged.

## Fixed data and first candidate

The prior original-source SLOW run was upright but failed strict quietness by
20.59 mm foot drift. Its final pose is a disclosed training-only starting point.
The proposed direct-torque teacher used200 causal pairs from its final5s:
first-substep motor control at IDs2401+20n, PRE-action physical observation at
2400+20n, with one fixed measured-entry target. The first120 pairs train and last
80 validate. Source residual radians are never used as normalized torque labels.
All labels are checked against actual actuator generalized force/original gear.
No saturated labels occurred; maximum normalized torque was0.468239.

The first100 supervised Adam steps at1e-5 failed the fixed heldout gate:

- Training normalized-action RMS0.074738; heldout RMS0.082517
- Heldout maximum absolute error0.211244, reaching18.64 Nm under original gear
- Required heldout RMS≤0.02 and maximum≤0.05
- Both hidden layers and the mean head were trainable; logstd output rows were
  fixed, with actual latent standard deviation0.02

The original fit did not retain Adam/minibatch RNG state. Rather than claim an
exact resume, the explicitly admitted extension reconstructed100 updates from
the original seed/data and checked EVERY actor tensor against the saved100-step
weights. They matched bit-exactly. All100 reconstruction calls were charged.
Full actor, Adam, NumPy minibatch RNG and Torch RNG are now saved every step.
A synthetic next-Adam-update restore test is also bit-exact.

The fixed500-trajectory endpoint failed too:

- Training RMS0.048018; heldout RMS0.074283
- Heldout maximum absolute error0.186944
- Final minibatch raw-mean MSE0.00220184
- Actual optimizer accounting:100 original +100 reconstruction +400 new =600
- No source physics, task collection, critic warmup, SAC actor update or physical
  held-control benchmark was executed in this stage

The full500-step loss trace is retained; its first100 entries are the deterministic
reconstruction, with the100 endpoint verified exactly. The original failure is
preserved. There was no best-heldout checkpoint selection, threshold change,
optimizer reset disguised as continuation, or automatic initializer fallback.
This result does not establish that the architecture or task is infeasible.

## Why a composed analytic base is worth reviewing

A zero-learning calculation of the original deterministic SLOW holding policy
and source PD composition reproduces all200 actual motor labels to about
7.4e-8 maximum normalized error using the single-frame source implementation.
Thus the labels and action order are sound, while the bounded new fit was not
accurate enough. This motivates a separately reviewed architecture proposal:

1. Frozen source SLOW actor sees its ORIGINAL standing joint targets, terminal
   actor-height sentinel1.4 and torso-up vector, at power1.
2. Its deterministic residual is composed with the original holding law
   q_standing + residual − q −0.2*dq, clipped and reordered into actuator torque.
3. A finite latent base is atanh(clamp(u_base,−1+1e-6,1−1e-6)).
4. A trainable mean correction is added BEFORE tanh; its final mean head starts
   exactly zero. The new task's references enter only this correction network.
5. Latent Gaussian std starts0.02 via the exact inverse source logstd map.

The frozen state-dependent latent shift needs no additional action-density
Jacobian when conditioning on state. The source epsilon-regularized tanh logprob
is retained, rather than claiming an exact normalized density near saturation.
The baseline is detached from correction-parameter gradients. A future optimizer
must include only correction parameters. Power below1 is deliberately unsupported
in this prototype; WEAK curriculum needs a separate design.

Unsaturated initial means reproduce the source holding torque pointwise, up to
float32 arithmetic. Saturated bases are interiorized by epsilon, so exact equality
is not claimed there. Source800Hz recomputes PD inside a held residual command;
a future200Hz teacher would hold the torque itself for25ms. Pointwise algebra
therefore cannot establish dynamic equivalence or stance stability.

The pure two-network prototype matches the200 recorded labels within1.61e-6,
with initial mean versus its own base≤2.99e-8 and std error0. It has not been
activated as a fallback or given any physical trial in this stopped stage.
The existing single-MLP native binary exporter cannot represent it. Fresh math,
interface/held-control, checkpoint, resource and export reviews are required
before any future activation, optimizer release or application integration.

## Retained implementation and evidence

- `scripts/getup-step/stabilization_init.py`: causal labels, source mean fit
- `fit_stabilization.py`, `extend_stabilization_fit.py`: bounded charged fits
- `training.py`: critic-only warmup with explicit target Polyak refresh
- `stabilization_ledger.py`: shared lock, per-call accounting and reserves
- `analytic_actor.py`: pure inactive composed-mean prototype
- `benchmark_stabilization.py`: blocked prospective benchmark helper; no run
- `docs/evidence/getup-source-stabilization-2026-10-18/`: compact own receipts,
  both failures, full loss trace, exact reconstruction and pure algebra evidence

All actor/optimizer weights and raw source caches remain local. No new upstream
or derived weights were redistributed. Current source task config still refuses
collection and learning. The failed-fit branch is stopped; another candidate or
stage requires an explicit reviewed admission rather than reuse of unused budget.

Independent source/math review cleared the inactive analytic prototype and four
pure fixtures at source SHA256
`2e05f5f1da7981d7965f8ab4c4b588ec3fe423d1e56a7912dc8ed6c30d959309`.
This clearance covers the conditional latent shift, original hold semantics,
saturation and correction gradients only. It gives no engine/optimizer or
application export admission.


## Separate analytic held-control diagnostic (closed)

A fresh, separately admitted diagnostic ran after the fit branch closed. It had
600 s total setup/review/execution wall time, a tighter120 s execution deadline,
1 s aggregate physics, and64 total40 Hz intervals/320 source200 Hz steps. It
loaded no failed-fit weights and enabled no optimizer or task-data collection.
The actual admitted window was00:56:09–01:06:09 UTC on2026-10-18.

The frozen SLOW actor plus original holding PD was evaluated at each policy
boundary, then its normalized actuator torque was held for25 ms. This preserved
pointwise source holding torque semantics but intentionally tested the different
closed-loop timescale. The initial source200 Hz cache was exact; it had previously
received the actual800 Hz SLOW endpoint q/qvel with a disclosed cold-cache/model
transition. No solver cache was transferred between different models/timesteps.

The first command was run twice from the same complete model-bound cache and
controller/RNG sidecar. ALL five post-integration packets were bit-exact. Both
branches were charged. The retained branch then failed the unchanged10 mm loaded
slip gate at source step28, t=0.140 s:

- Charged33 integrations in7 intervals:5 repeated-prefix steps +28 retained
- Loaded slip L4.8391 mm / R10.2146 mm; first crossing at the final actual step
- Actual original-gear torque equation residual exactly0 Nm
- No warning, flight or nonfoot sample before the stop
- Maximum hinge speed0.445 rad/s at0.100 s →3.204 at0.125 s →17.214 at0.140 s
- Only about1.68 mm total COM displacement, despite the rapid joint response
- One final base saturation was interiorized by about1e-6 normalized control;
  this epsilon effect is far smaller than the observed response
- Aggregate physics0.00272655 s, zero optimizer calls, zero replay collection

This rejects this particular200 Hz/held25 ms torque initializer at the recorded
entry. Algebraic initialization fidelity and exact cache continuation did not
establish dynamic equivalence to SLOW's800 Hz inner PD. The direct-torque analytic
branch is stopped. There is no automatic longer hold, SAC release or reuse of its
unused allowance. Retaining source SLOW's original800 Hz inner PD is the next
causal design direction, subject to a separate bounded proposal and admission.

Implementation: `scripts/getup-step/analytic_diagnostic.py`, explicit
`analytic-diagnostic-config-v1.json`, and a separate diagnostic ledger profile.
Compact own evidence is in `docs/evidence/getup-analytic-diagnostic-2026-10-18/`.
The107 Python tests pass. Raw journals/caches and original actor assets remain
local; no new upstream or derived weights were published.
