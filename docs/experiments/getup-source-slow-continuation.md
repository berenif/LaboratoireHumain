# Exact source-SLOW online continuation

Status: closed on 2026-10-18 at the fixed cumulative5,000-transition endpoint.
The deterministic candidate fails the original anchor gate at0.07125s; the
immutable original lasts2.865s. Baseline retained. No quiet5s or forward step.

## Prospective change and exact starting state

Section15 of the Python GetUp plan continues the retained TRAINING checkpoint at
transition1,580/proposal139/accepted11, including its populated critic/Adam/target,
all four random streams, actual solver cache, controller anchors/slip/clock and
seven unfinished episode48 rows. No validation cache, reseeding or rewarm occurs.
The zero-step native/controller/cache and next observation/action/replay-index/
proposal140 tape tests pass exactly.

Only original-policy distance and historical/midpoint action-drift rejection are
made report-only. Incremental full Gaussian KL(previous accepted||candidate)≤.05,
same-noise actor-loss decrease≥1e-6, all physical criteria and original motor
bounds remain hard. Both output heads learn; original hidden layers and source
alpha0.000612579835321518 remain fixed. The118-input source hold target, height
sentinel1.4, residual-radian actions and20×800Hz kd=.2 native PD remain unchanged.
The new rejection streak starts0; old128 anchor-only rejections remain evidence.

## Counted execution

The fresh window starts03:22:14.894121UTC and has a fixed04:07:14.894121 deadline.
Setup/review count within it. Limits are45min wall,15physics sec (11training,
4protected pair),3420new training plus640pair intervals,81200new native steps,
1710new optimizer calls and300wall seconds protected for validation. Prior costs
stay charged in their separate closed ledger. There is no repeated benchmark.

Actual result:

-3,420 NEW transitions, reaching cumulative5,000;64,096new training integrations
-855 new critic updates plus855 both-head proposals,1,710new calls
-All855 proposals accepted at scale1; cumulative994attempts/866accepted
-Maximum incremental KL0.01370553; minimum same-noise loss decrease0.00406933
-462new physical terminals:137loaded-slip,324fixed-anchor and1both-feet-unloaded
-435partial and27full20-sample terminals; ALLmask0 with durable reset proofs
-64,096training +2,292frozen +57candidate =66,445consecutive new-stage samples
-3,538new-stage policy intervals;4.190003physics seconds;585.085run wall seconds
-No unresolved native, optimizer or proposal transaction at close

Periodic full states still use cumulative completed-transition IDs. The committed
TRAINING cache contains all5,000replay entries,95,206cumulative training samples
(31,110prior+64,096new), and the restored learner/RNG state. Final validation
caches are separate. No unused allowance or charged work is refunded.

## Fixed final physical pair

Both branches start with bit-identical118 observations, the same entry packet,
model/config and exact800Hz cache. Independent reduction finds every saved
PRE-state→source residual-PD→original-gear equation exactly0Nm in error.
The complete2,292-packet frozen branch matches the previous frozen run exactly.

-Original: first LEFT loaded-slip crossing at2.865s/2,292samples;
 L10.001635mm/R9.685628mm, terminal12sample interval/mask0
-Candidate: first RIGHT fixed-anchor crossing at0.07125s/57samples;
 previous9.985335mm→10.198153mm, terminal17sample interval/mask0
-No earlier candidate slip crossing, warning, both-feet-unloaded or nonfoot support

The initial states match, while the very first candidate residual differs from
the original by0.459979rad RMS/0.809189rad maximum. Together with exact torque and
metric equations, this supports deterministic policy divergence rather than a
reset/measurement mismatch. It does not identify the learning objective's cause.

The historical mean drift reaches0.456650rad RMS/0.814372rad maximum; cumulative
full Gaussian KL to the original reaches394.258. Local incremental KL did not
bound cumulative policy change. Final latent std over201×21probes spans
0.04435–1.38949, median0.11903. Passing local actor-loss/KL checks did not predict
physical advantage. No threshold is relaxed and no intermediate physical-best
checkpoint is selected.

## Reproducibility

Lean evidence is in `docs/evidence/getup-source-continuation-2026-10-18/`.
Run the zero-engine/zero-optimizer reducer with retained local journals:

```sh
.venv/mvp/bin/python scripts/getup-step/reduce_continuation_stage.py \
  --stage outputs/getup-source-continuation/session-20261018T032214Z \
  --source outputs/getup-source-engine/session-20261017T233404Z \
  --output /tmp/continuation-reduction.json
```

The literal runtime admission binds all106 code/config/model/asset/proof hashes.
Raw journals/caches and source/derived weights remain local; upstream distribution
permission is unverified. No default application policy or engine is changed.

## Closed observation/state audit

A separate read-only audit checks all5,000 checkpoint replay rows against the
closed old/new streams and reconstructs510 completed episode returns. One final
live row is excluded. Physical67 plus the two constant hold targets/power omit
cumulative slip, fixed-world displacement from anchors, elapsed/quiet/persistence
history and previous-control terms. Those omitted quantities affect task rewards
or termination, so source118 is a partially observed task contract.

This does NOT establish the cause of the failed learner. Across10,034,740
non-reset cross-episode pairs, no strict/tight observation AND residual-action
matches occur under the declared physical-unit normalization. The closest joint
neighbor still differs90.75mrad RMS in action. Six coarse exploratory pairs are
insufficient to isolate hidden-history effects. The mathematical history
counterexamples are conditional contract examples, without proved reachable
identical-state histories in this replay. The empirical audit cannot justify an
observation change or explain the critic error by itself.

The endpoint critic overestimates complete logged soft behaviour returns on
average, particularly near failure. Such multi-step changing-policy comparisons
are off-policy diagnostics. Physical mask0 transitions offer the stronger
one-step check: their Bellman targets are exactly the recorded reward, with no
bootstrap. Actor learning must not be resumed merely because local actor loss
improved; direct critic target fitting is the next diagnostic gate.

## Direct terminal-target fitting and actor attribution

On510 actual physical mask0 transitions, the final minimum twin-Q prediction
averages−1.160 against a mean exact reward target−9.619; terminal RMSE is9.554.
In the latest256 replay rows,75 terminals average Q+.564 versus target−9.620,
RMSE10.575. Unlike multi-step behaviour returns, these one-step targets do not
bootstrap or depend on the future policy. They show serious fitting error on
recorded terminal pairs; they do not by themselves distinguish implementation,
sampling, optimization and approximation causes.

The final proposal's saved common-noise loss reconstructs exactly. About99.97%
of its local loss improvement comes from the Q term. Entropy/Q gradient norm
ratios are0.001414 for the mean head and0.03626 for the std head, so entropy alone
is not supported as the driver. However the final critic at the entry state ranks
the ORIGINAL deterministic action10.503 above the final candidate9.027. A simple
claim that the final critic prefers the bad candidate at that state is therefore
false. Moving off-policy value miscalibration and cumulative distribution drift
are supported observations, not a fully isolated causal mechanism.

The next proposed diagnostic is frozen-actor, replay-only critic target fitting
with episode-separated holdout and exact mask0 target checks. It requires a fresh
bounded review/admission. No further actor update or physics is authorized by
this failed result or by the diagnosis.
