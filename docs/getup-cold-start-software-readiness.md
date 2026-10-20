# Cold-start software readiness — 2026-10-20

**Software rehearsal only. No new physical trial, forecast, experimental optimizer,
clock, upload, candidate, merge, or deployment is authorized by this checkpoint.**
The closed attempt remains 2,940 charged integrations, zero training decisions and
zero learning updates. Its files and original deadline are unchanged.

## One production path

`train-getup-cold-start.py` now delegates to `ColdPilot` for admission, next-transition
parity, bootstrap, preoptimizer restoration, critic warmup, interleaved collection
and guarded updates, export/evaluation handling, and terminal retention. The old
historical-input runner remains unconditionally closed. The consumed one-operation
qualification amendment cannot be restarted.

Only clock, physics bridge, and external backup transfer are scripted in the
rehearsal. Mapper, replay, CPU Torch actor/critics/optimizers, rollback, serialization,
ZIP creation/verification/extraction, `DriveExchange`, `DrivePhaseTransport`,
request/receipt validation, and application-before-prune ordering are production
implementations. Synthetic records, archives, requests, receipts, and application
markers are explicit; real recovery rejects synthetic receipts.

Independent restoration constructs new learner, replay, RNG, pending mapper,
phase, witness, and controller objects. It binds source/artifacts, policy, snapshot
nonce, raw source features/actions/rewards, pending folded tails, replay strata,
ledger and deadline. Active and runtime-free boundaries are distinct. A fresh
Python process must match sampled replay indices and the next synthetic guarded
update of the **uninterrupted paused live objects**, not another restoration.

## Verified software scope

Commands use the existing CPU-only environment:

```sh
work/cold-start/venv/bin/python scripts/getup-cold-start-engine-test.py
work/cold-start/venv/bin/python scripts/getup-cold-start-test.py
work/cold-start/venv/bin/python scripts/getup-learning-loop-test.py
work/cold-start/venv/bin/python scripts/experiment-backup-test.py
npm test
```

Readiness baseline results: **12/12 engine**, **16/16 cold contracts**, **23/23 historical
loop**, **23/23 backup**, and **129/129 JavaScript tests** passed. The final engine
suite took 56.49 seconds. JavaScript syntax and `git diff --check` also passed.

The subsequent absolute-stage-cutoff amendment passed **13/13 engine** tests
(53.838 seconds) and **16/16 cold contracts** (2.380 seconds), without native
physics, network transfers, or changes to the closed attempt.

The engine suite covers 256 critic warmups and two complete 256-source-decision /
64-attempt blocks. The second collection uses an accepted updated actor. It covers
seven operational archive restores, export/evaluation handling, and terminal
retention, plus rejected-block complete rollback, malformed/uncertain/dead bridges,
empty strata, pending-tail continuation, failed/delayed backups, mixed identity /
ledger / deadline rejection, protected reserves, rapid-terminal forecast charges,
final-iteration wall overrun, and missing-admission / closed-output rejection.

These assertions do **not** establish real native transition/reward/mapping parity,
actual forecast-heavy throughput, a trainable physical replay, a trained policy,
or the unchanged physical acceptance tests. Native/WASM execution and broad MVP
physical regressions were not rerun for this software checkpoint.

## Fixed envelopes, not throughput promises

A future admitted attempt retains 3,600 seconds, 300,000 total integrations,
4,096 decisions (bootstrap at most 1,024, iterative at most 3,072), and 147,000
protected evaluation integrations. Absolute work/evaluation/retention cutoffs are
start + 2,100 / 3,300 / 3,600 seconds. Evaluation and retention reserve 1,200 and
300 seconds respectively. Qualification's 60 seconds / 1,000 integrations are
included **before** admitting the complete meaningful suffix.

The reviewed narrow deadline amendment fixes **exactly two blocks**, even when
unused time/integrations/storage could fit more. Input restoration must complete
by start +120 seconds; qualification +180; transition parity +240; bootstrap plus
verified backup +600; warmup plus backup +900; block one plus both backups +1,500;
block two plus both backups +2,100. These are executable absolute stage cutoffs,
not waits or timing assumptions. Full per-phase allowance must fit its cutoff.
The original ≤20 mm material metric remains maximum anchored material-point
excursion, not cumulative path travel; no physical acceptance criterion changed.

The conservative default admits two blocks: 1,920 seconds of work-phase envelopes
including the six in-work backups, plus terminal retention. Every reset/forecast
counts inside finite hard phase integration envelopes; these are not predictions
that an episode will finish. Phase/command failure stops without retry or refund.
Controlled, mixed forecast, bridge, learning, serialization, and transport/restore
costs are recorded separately; mixed forecast timing is never labeled isolated
forecast duration.

Storage admission includes the entire cadence, not just the next upload: seven
backups for two blocks, each with an 18 MiB payload and 20 MiB compressed archive
hard envelope. The actual transport route also remains limited to 32 MiB per
archive; protocol maxima remain 32 MiB shared / 64 MiB generation / 160 MiB total.
Cumulative uploaded bytes include Trash and the existing 16,167,158 bytes, with no
quota refund. Current + previous remote generations remain; replacement must pass
application restore before scoped retirement. Closed local evidence is protected
and counted, never deleted by a later runner. All rehearsal archives are temporary
local files and removed when tests finish.

Seven measured synthetic archives are approximately 4.90–6.16 MB (about 40.80 MB
combined), projecting 56,970,714 bytes cumulative including existing remote bytes.
Observed temporary rehearsal storage peaked at 56,308,040 bytes with one-second
sampling; end-of-run retained test files were 28,187,259 bytes, then deleted by
the temporary-directory fixture. Sampling is not an assertion of exact peak;
pre-copy/pre-transfer staging admission independently enforces the byte cap.
This is a synthetic compression measurement, **not** a promise for real replay.
The hard cadence envelopes project 162,967,798 bytes, below 167,772,160 bytes.
Oversize actual payload/archive or local staging rejects before transfer.

## Three separate gates

1. **Software:** passing tests and independent review, bound to exact source,
   dependencies, worker and protocol hashes. This grants no trial authority.
2. **Operational trial:** a future root resource decision and explicit admission
   are required before a new clock. Existing hash-bound immutable archives are
   reused without unchanged uploads; application loads occur inside that clock.
   Prior private roundtrip reuse is disclosed, not relabeled a new remote transfer.
   Real next-transition/reward/terminal/mapping parity and coherent operational
   learner/replay/runtime restoration must precede any experimental optimizer.
3. **Physical acceptance:** unchanged 25-second recovery after impact, 800 stable
   native samples, 30-second hold, nearby BACK/BELLY preservation and full release
   regressions. No software/synthetic flag can satisfy these.

A future admission file binds an explicit resource-decision reference, new run ID,
exact reviewed commit, software receipt/source inventory, independent review,
immutable request/receipt/application hashes, seed paths, and complete owner
storage inventory including current/previous generations and protected local roots.
The admission also binds one output path and an announced `started_at_unix`;
launch validation/notification elapsed time remains spent, never refunded by a
new timestamp. A future or ≥120-second-old start is rejected before runtime load.
The tests never generate an authorized admission. No generic resume or automatic
new trial is enabled.

## Runtime input identities (read-only)

- WASM SHA256: `ff3da0d06c22d444972ecb7cde04b66304b6b65ae88f07ca73d876b1f3a1bdc1`
- Glue SHA256: `073bd2ec869551c2dddd3f5e0710d8fd9c75b6733a3b430bbf6c9c5609af3aa5`
- Protocol SHA256: `84a6ea3561be5ea2bcd95fc7f5b0717f1d0655ae0c7417f23047541020006cfd`

Source commit/tree identity and exact test/size receipts accompany this checkpoint.
No private Drive IDs, model binaries, runtime checkpoints, replay, credentials,
old Git evidence, cache, venv, or build output are included in source publication.
