# Source SLOW online stabilization pilot

Status at 2026-10-18 03:10 UTC: real online learning and the fixed final pair
completed. The candidate failed the slip gate50ms earlier than the immutable
source baseline. Baseline retained; no quiet-standing or forward-step success.

## Mechanism

Python retains the original SLOW118 observation, standing target with height
sentinel1.4, residual-angle action and800Hz inner PD. Both original hidden layers
and learned temperature0.000612579835321518 stay fixed. Both mean/logstd output
heads learn under the reviewed full Gaussian KL, original-scaled mean-distance,
historical action-drift and common-noise loss-decrease guards. Collection,
critic-target, proposal and replay random streams are independent.

The proposed endpoint is5,000 genuinely new actual transitions:1,024 frozen-actor
collection,1,000 critic-only Polyak updates, then one critic and one both-head
proposal per four new transitions, at most994 proposals. All finite physical
terminals, including partial intervals, retain mask0. There is no old replay,
physical-best checkpoint selection or automatic noise fallback.

## Counted optimized benchmark

The 02:31:39→03:16:39 stage closed with:

-64 policy intervals and1,280 actual source integrations, including a charged
 20-sample full-cache continuation repeat. All repeated packets/results match.
-0Nm error in the original native PD/gear equation
-16 critic and2 forced-reject,11-scale both-head updates, all discarded
-0.074873835 physics seconds;9.163847 wall seconds
-5.225955 seconds for the64-interval loop, approximately12.25 intervals/second
-Worst critic update0.060065s;2,560-witness11-scale proposal0.375862s
-Maximum-shaped5,000-row full checkpoint0.436375s;actual-cache reset0.026344s

The initial every-terminal/every-proposal full-snapshot cost model projected
6,125s of training and correctly blocked candidate launch. Candidate transitions
and optimizer calls in that stage remain zero. Its unused allowance was closed.

## Explicit checkpoint-cadence amendment

The parent-authorized overlay `online-checkpoint-cadence-v1.json` changes only
checkpoint scheduling. Full rolling states are committed every64 completed,
verified transitions, after warmup, at fixed milestones, before validation and
at the final complete boundary. Every terminal's raw packets, action/reward/mask,
behaviour witness and exact actual-reset-cache hash/ordinal are durable before
reset. Every proposal retains an immutable compact head/Adam/RNG/trust receipt.
Unresolved native/optimizer/proposal transactions cannot replace a checkpoint.

If execution stops between full commits, the retained checkpoint is explicitly
older than the charged ledger. Later work is neither refunded nor represented as
an exact current-state resume. Physical sampling and failure gates are unchanged.

The original benchmark is reused only after an explicit source-hash/AST migration
check; no new benchmark integrations or optimizer calls are invented. A separate
fresh45-minute execution window starts03:01:12.654562 UTC, with20 physics seconds,
5,000 training intervals and a protected640-interval/300-wall-second final pair.
The measured revised whole-training projection is approximately1,632s plus the
protected tail. Live capacity checks remain binding before actual collection,
actor release and the complete final pair.

## Scope and reproducibility

The preserved actual source8s endpoint is training-only and does not already pass
quiet-slip qualification. Final evaluation still requires the final4,000 actual
800Hz samples with the original quiet predicates and unchanged10mm limits.
A successful stabilization would still not qualify lifting/forward placement.

Code uses the pinned source equations and local source assets documented in the
main Python GetUp plan. Raw cache/trace/model weights remain local; upstream
redistribution permission is unverified. The compact benchmark evidence is under
`docs/evidence/getup-source-online-2026-10-18/`. No application default changed.


## Actual online outcome

The fresh stage ran267.624s wall and2.210328s actual physics. It collected1,580
NEW transitions before the prospectively defined128-consecutive-rejection stop:
1,000 critic warmups,139 additional critic updates and139 both-head proposals,
1,278 counted calls in total. Eleven proposals were accepted. Every rejected
proposal/physical reset retained all prior charges and independent RNG progress.

Training contains31,110 actual native samples across49 episodes, with42 loaded
slip and6 fixed-anchor terminals. Forty-seven terminals are partial, and one
physical terminal occupies all20 samples; all48 have mask0 and durable reset
proofs. There were no warning, both-feet-unloaded or nonfoot-support samples.
Independent saved-data checks reproduce every motor-control/torque equation at
exact0Nm error. All35,654 training plus validation native IDs are consecutive.

The final deterministic pair, from the same original full800Hz cache, gives:

-Original: first left-slip10mm crossing at2.865s (2,292 samples),
 L10.001635mm/R9.685628mm
-Candidate: first left-slip10mm crossing at2.815s (2,252 samples),
 L10.002413mm/R9.281825mm
-Both stop on that first crossing; both terminal intervals have12 samples and
 mask0. Neither supplies the final4,000 quiet samples. The candidate is50ms worse
 on this failure time, so no physical improvement is supported.

Historical mean drift remained within the preset: RMS0.009403rad,
maximum0.010444rad. Cumulative full Gaussian KL to the original reaches0.06228;
this is distinct from the incremental previous-policy KL cap. All128 rejected
proposals fail the original-scaled MEAN-distance preservation bound; their1,408
scale trials still satisfy loss decrease and incremental KL. The preservation
heuristic has stopped learning, including coupled std updates; it is not a
physical impossibility certificate. A prospective learning-scope change requires
new reviewed admission, and does not qualify this candidate.

The committed training checkpoint covers all1,580 transitions and31,110 training
integrations. Validation caches/controllers are separate; they are never relabelled
as the training state. The ledger closes at1,808 total policy intervals and35,654
native integrations with no outstanding native/optimizer/proposal transaction.
The earlier benchmark's1,280 integrations and18 discarded calls remain in their
separate closed ledger; no unused allowance transfers.

Reproduce the zero-engine/zero-optimizer journal algebra with:

```sh
.venv/mvp/bin/python scripts/getup-step/reduce_online_stage.py \
  --stage outputs/getup-source-online/session-20261018T030112Z \
  --source outputs/getup-source-engine/session-20261017T233404Z \
  --output /tmp/online-stage-reduction.json
```

The compact reduction/summary and input hashes are in the evidence directory.
Raw local journals/checkpoints are required by this reducer; no source or derived
weights have been redistributed.


## Forward-only causal audit

A separate zero-engine/zero-optimizer audit verifies all139 receipt hashes and48
complete behaviour-return recurrences. Attempt139's replay batch,2560 witnesses,
noise and all11 recorded loss/KL/distance trials reconstruct with exactly zero
closure error. All rejected scale trials satisfy incremental KL and loss decrease;
the smallest-scale original-mean-distance excess is only0.86–2.24×10^-6 above0.05.

Using that same saved full-Adam parameter delta, a std-only forward substitution
would lower the local actor loss by0.000980616 while passing the local incremental
KL/mean-distance tests and leaving deterministic means unchanged. This demonstrates
coupling in the42-output projection, not a tested physical improvement. Across the
actual11 accepted steps, the median pointwise std ratio rises2.590% (range−2.852%
to+3.569%); variance did not collapse.

The final critic's first48 episode-start predictions average3.072 versus recorded
soft behaviour return15.049, RMSE12.247. Same-first31 RMSE changes12.628→12.319
from the1000-update warmup to final. These off-policy realized-return comparisons
are report-only, not an unbiased current-policy value estimate. They do not prove
the50ms physical regression's cause or qualify the candidate. The next scope is a
prospective reviewed continuation; no source critic is replaced midrun.
