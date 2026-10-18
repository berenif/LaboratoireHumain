# Source SLOW online stabilization pilot

Status at 2026-10-18 03:03 UTC: implementation and original-distribution benchmark
passed. Candidate collection/learning awaits the final fresh-window admission.
No quiet-standing improvement or forward step has been established.

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
