# Corrected learning loop: source-only, experiment blocked

The separately approved corrected learning-loop experiment **has not run**.
The previous rejected pilot remains closed. Its compact evidence and physical
results are unchanged.

The workspace was replaced after that pilot. The full published source was
restored at commit `0776a3ba7d1ff32195b8d1eb9501c19678b8a233`, tree
`c85b1870a790aa1766e17085891a8893405478b4`. The replay, raw transition log,
learning/provenance audits, original/nearby/BELLY checkpoints, learner state and
compiled WASM were local ignored artifacts and were not recovered. Source and
compact receipts cannot recreate their physical experience.

## Implemented contracts

`train-getup-learning-loop.py` reuses the existing correction actor, critics,
source-transition builder and actual-WASM JSONL bridge. The frozen 118-input
actor, original reward, discount `exp(-0.025/25)`, correction scale 0.05, controller,
deadline and physical gates are unchanged.

- Zero correction mean, fresh critics, 256 critic-only warmup updates
- At most 16 blocks: 256 fresh head-controlled decisions with fixed collecting
  weights, followed by at most 64 joint attempts
- Exactly 64/32/32 items per 128-item batch: original BACK pre-holding, original
  BACK holding, verified complete baseline-success preservation; episode-uniform
  then transition-uniform within each category
- Verified successful exploratory BACK episodes remain in their BACK strata.
  High reward alone never creates a baseline-success label
- All verified retained rows plus 4,096 fresh rows are retained without eviction
- Ongoing physical/controller state, episode identity and impact/reference clocks
  survive learning pauses; a source-to-balance tail retains its real terminal
  credit even on the 256th source action
- An unfinished final collection episode is an administrative truncation, with
  nonterminal bootstrap. It is not relabeled as a deadline failure
- Each block uses a fixed collecting-policy witness: all 256 recent states plus
  64 historical pre-holding and 64 preservation states. Gaussian KL is
  collecting-to-candidate, summed over 21 dimensions then averaged over states,
  at most 0.1. Maximum deterministic correction change is at most 0.05
- Rejected attempts restore actor, critics, targets, all optimizers, alpha,
  gradients, modes, learning counters and CPU/Python/NumPy/replay RNG states.
  Exposure/attempt counters do not roll back. A block with no accepted actor
  update stops the experiment

The existing bridge gains an opt-in exact retained-input contract. It refuses
new fixture/heldout generation, validates every file hash before importing WASM,
and offers read-only pause/epoch receipts. The first 20 native source samples
remain original/uncorrected on every branch, as in the closed pilot; these are
charged as initialization rather than fresh head-controlled replay decisions.

## Hard budget and final evaluation

The prospective run has one immutable 60-minute wall deadline, 4,096 fresh
decisions, 1,024 joint attempts and 300,000 charged native integrations. Initialization,
forecasts, collection, evaluation and durable-backup work inside the run share
that deadline. A separate process watchdog and deadline-aware transport stop
in-flight work; calls are conservatively charged 5,020 integrations before entry.
Unknown completion cannot trigger a retry. Final capacity is protected during
both physical collection and learning, with at least 147,000 integrations and a
throughput-derived wall reserve.

One deterministic final head receives pure-WASM and applied-correction parity
checks with the existing 2e-6 tolerance. Original BACK is evaluated first. Zero
qualifying diagnostic interval, failure of the original 800-native standing gate,
or any integrity failure stops it. Nearby BACK and BELLY preserve qualification.
On first success, the original anchored 30-second hold starts immediately. The
diagnostic interval freezes at exactly impact+20,000 native samples within that
same hold. Extra hold samples never inflate the fixed-25-second score. Integer
standing-counter deltas and unchanged original anchors prove the hold; 10 mm
foot-origin drift and 20 mm maximum anchored material-point excursion stay
separate.

## Recovery gate

The runner integrates `experiment_backup.py`. Before a costly run it requires a
fresh private remote download/restore proof bound to the exact input manifest
and every prerequisite hash. Coherent generations after warmup, collection and
updates include learner/optimizer/RNG state, replay and episode mappings,
pending-transition state, exact Runtime checkpoint, JavaScript reward/metric
state, reports, configuration, raw logs, absolute deadline and monotonic ledger.
Uncommitted source and unresolved retention rights are refused. A failed or
uncertain retention operation stops collection and suppresses automatic remote
retry. See [durable backup tooling](durable-experiment-backups.md).

The official private-Library helper currently fails authentication with HTTP403.
No remote archive upload or real durable recovery proof succeeded. No alternate
route was used.

## What the tests establish

Pure synthetic fixtures test phase interleaving, episode-aware sampling, immutable
witness limits, complete rollback contracts, source-tail credit, deadline waits,
inclusive ledger admission, and fail-closed input/recovery gates. They construct
no neural model or WorkerRuntime and never enter real replay. Actual PyTorch
optimizer rollback, recovered audit-schema compatibility, compiled-WASM replay,
durable real-data restore and physical improvement remain untested.

The runner intentionally contains an unconditional `require_execution_ready()`
gate. `--run` cannot override it. Enabling a run requires recovered exact inputs,
an independently verified adapter for the actual lost audit schemas, a real
durable recovery proof and renewed review of any changed data-collection design.
The canonical metadata reader is a proposed strict mapping contract, not a claim
that the lost audit format has been tested. No replacement samples were generated.

See [source-only evidence](../evidence/getup-corrected-learning-loop-2026-10-20.json).
