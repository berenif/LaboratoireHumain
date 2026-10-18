# Source observer optimization

Status: pure implementation and saved-data parity passed. No new engine world,
physical integration, policy inference or optimizer operation was used for this
observer experiment. Live end-to-end throughput remains a separate admission.

## Measured bottleneck

The prior 5.29 policy-interval/s benchmark included initialization, cache repeat,
journaling and 16 discarded critic updates. It was not a steady-state loop rate.
The earlier 81.1 teacher rate used five 200 Hz frames per policy decision; the
SLOW controller uses twenty 800 Hz frames. Its closer existing original-source
SLOW comparison was 23.71 commands/s, with different task-audit work.

A saved 115-interval baseline isolated 12.92 s of observer work: preview decode
4.06 s, consume with duplicate decode/backup 6.35 s and preview deepcopy 2.13 s.
Both deepcopies repeatedly copied the immutable 507 KB model. Journal work was
only 0.283 s. Scratch measurements put the unchanged durable ledger's 41 atomic
fsync/replace writes at roughly 21–31 ms per full policy interval; storage timing
varies and those guarantees were retained.

## Implementation

- An owned recursively read-only model is SHA256-bound. Static kinematic frames,
  mass and motor gears are cached by model hash, never by physical state or time.
  NumPy arrays use immutable bytes owners, preventing write-flag re-enabling.
- Each native packet is validated and decoded once. A process-local token,
  context identity, canonical packet SHA and native ID bind immutable metrics.
  Consume rechecks the original raw packet; stale, mutated, swapped or foreign
  samples cannot skip validation.
- Preview shares immutable geometry and copies only its mutable slip/foot/quiet
  state. Consume backs up an explicit nine-field transaction schema, including
  history and clock. A post-history-mutation failure restores every mutable field.
- Per-native preview and consume safety decisions, actual q/dq PD refresh,
  stopping order, exact partial terminals and all ledger/fsync writes remain.
- The new observer module is included in future runtime fingerprint admission.

## Exact saved-packet result

The harness loads the published pre-optimization baseline commit
`54957e7843f5aeab507d8bff314e670611ddf260`. Legacy and optimized fake-world paths
replay both completed physical branches: 4,522 recorded packets and 227 policy
intervals. State, action, next observation, reward, bootstrap mask and every
native safety/quiet field equal the historical receipts exactly. First loaded-slip
terminals remain source IDs 8692/8630, partial counts 12/10 and bootstrap zero.

Source/model/config fingerprints are captured before timing and checked after.
Observer-only timings are recorded in the accompanying receipt. These measurements
exclude physics, actual telemetry extraction, actor inference, durable ledger and
learning updates; they are not a promised online training rate.

Four immutable-observer tests and five existing measured-interval tests pass.
The broader source suite also passes 43 tests. Checks cover caller-model aliasing,
read-only arrays, token/packet/model mixing, error rollback, actual inner-velocity
refresh and next-command restore. The old failed physical policies remain failed;
this software optimization does not qualify quiet standing or a forward step.

Code: `source_observer.py`, `source_task.py`, `slow_task.py` and the pure
`replay_observer_parity.py` harness in `scripts/getup-step/`. Compact receipts:
`docs/evidence/getup-source-observer-optimization-2026-10-18/`. Raw journals and
checkpoints remain local; the replay harness requires those hash-bound inputs.
