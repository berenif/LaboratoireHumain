# Cold-start recovery, not historical reconstruction

The source at `1819996baaee8c2945a7ce77766e84ea2393faad` and the two Git-backed
model binaries (9,511,296 bytes total) were restored in the existing checkout.
The lost pilot replay, learner, raw audits, checkpoints and compiled WASM were
**not recovered**. The historical `train-getup-learning-loop.py` remains closed
by its unconditional `require_execution_ready()` gate.

The separately authorized cold-start path is `train-getup-cold-start.py`, with
the immutable `getup-cold-start-protocol.json` contract. It constructs new
experience and does not relabel it as retained historical experience.

## Narrow same-attempt qualification amendment

The optional `--continue-after-qualification` path is **not generic resume**.
It accepts only the exact certain boundary after preparation and qualification:
2,380 consumed real integrations, zero forecasts/decisions/updates, no bootstrap
artifacts, and the original start/deadline. An immutable amendment binds the
original stop report, ledger, configuration, triggered seeds, operational Drive
receipts, original source, audited amended source, unchanged learner constructor
sources/dependencies and seed 20261020. It restores cumulative Drive accounting
without re-uploading shared inputs. The initial stop report remains preserved.

This amendment admits **one useful BELLY bootstrap initialization only**:
at most one native call, conservatively precharged 5,020 integrations and a hard
120-second response deadline inside the original hour. Ordinary output reuse
and ledger reuse remain forbidden. Single-use claims exist in both Python and
the bridge. Preparation, qualification, steps, holds and optimizer work are
forbidden on this path. It stops unconditionally after the initialization and
retains nonresumable evidence and timing. A timeout is not retried.

The old formulas that extrapolated fixed connector waits per integration are
removed. The complete learning/evaluation phase remains closed pending separate
controlled-physics, forecast, learning, serialization/restore and sized future
backup projections. A qualification rate or one initialization does not establish
full evaluation capacity. Repair and publication wall time is never refunded.

## Admission order

1. Rebuild the pinned project worker, locally, CPU only. Reuse one build output
   and existing Rust caches. Pure software tests do not run physical simulation.
2. Start the single 3,600-second absolute pilot deadline, before its input backup.
3. Construct actual BACK/BELLY Runtime checkpoints at native and forecast zero.
   A private Drive upload/download, exact archive/payload hash verification,
   isolated extraction and actual runtime loading must all pass before physics.
4. Prepare fresh original BACK (480 native), nearby BACK (560 native) and BELLY,
   once, charging every real and forecast integration. Save triggered seeds once
   in a second immutable shared bundle. Verify their private operational restore.
5. Charge zero-correction trace parity and restored-next-trace qualification.
   Bootstrap complete zero-correction BELLY/nearby success preservation and
   original BACK pre-holding/holding strata, within 1,024 source decisions.
   Incomplete original BACK is an administrative truncation, not a fake terminal.
   Missing strata, failed preservation or exhausted capacity stop the experiment.
6. Before any optimizer update, privately retain and operationally reload a
   coherent learner/replay/metadata/RNG/cursor/runtime/ledger bundle. Real
   next-transition state, full native-trace hash, reward and terminal parity are
   checked in the physical budget; both executions are charged and the duplicate
   raw line is explicitly excluded from replay.
7. Freeze a schedule of at most 12 iterative blocks from remaining capacity,
   never extend it. Each block collects 256 fixed-policy source decisions, then
   at most 64 guarded attempts. All inputs and pending source-tail chronology are
   retained. The fixed witness's historical subset comes only from bootstrap.

All preparation, qualification, forecasts, collection, evaluation and in-run
backups share the **300,000 integration / 4,096 source-decision / 60-minute**
limits. Bootstrap and iteration have separate ceilings of 1,024 and 3,072
decisions. At least 147,000 integrations and a measured-throughput wall reserve
remain protected for the single final original BACK, nearby BACK and BELLY
evaluation. The original 25-second impact deadline, 800 consecutive native
standing samples and anchored 30-second hold are unchanged. There is no paid
compute, automatic restart, cap renewal, merge, deployment or release claim.

## Exact bytes versus exact runtime state

Uploaded and downloaded **archive and payload bytes must match SHA-256 exactly**.
Rapier checkpoint reserialization is not canonical: internal map order may
change after deserialization. Therefore runtime equivalence requires equality
of joint state, handles, physical snapshot, controller and compensated root
integrator state with zero steps; the admitted run additionally compares the
actual next full native trace and Python/JavaScript transition semantics.
Reserialized checkpoint byte equality is not falsely claimed.

Replay maps every source transition to its exact raw line range, episode,
behavior policy, impact/native chronology, applied correction, terminal
discount and folded-tail reward. Canonical float32 source/next feature hashes
come from the actual runtime, preventing arbitrary feature arrays from passing
an action/reward-only audit. Downloaded mapping, pending transition/origin,
cached/authoritative state and replay RNG are compared and rehydrated.

## Private Drive and bounded storage

Official Library helper access remains blocked; it is not retried or bypassed.
Private Drive is a separately authorized connector route. An operator transfer
owner services exact `*.request.json` files. The runner waits without advancing
physics and admits a receipt only after exact downloaded-byte verification and
an application-level restore callback. Remote identities remain in ignored
private run directories, not in published source/evidence.

Hard caps are admission gates, not targets:

- 32 MiB combined immutable input/triggered-seed payload, stored once.
- 64 MiB per live-generation payload.
- 160 MiB cumulative remote archive bytes **including recoverable Trash**,
  plus a conservative payload-peak check including a pending third generation.
- At most current and previous active verified generations. Replacement
  application restore occurs **before** a scoped recoverable-Trash request.
- Local archive/download/extraction peak is reserved before copying. Only exact
  directories created by this run are cleaned. Temporary connector downloads
  are removed by their owner after the application-verified marker.

No repository history, old Git evidence, toolchain, venv, build intermediates,
duplicate model binaries or unrelated user files enter backups. The 394-byte
prior Drive drill is conservatively counted against the cumulative cap. If
recoverable Trash fills the cap, the pilot stops; it cannot silently treat those
bytes as freed, permanently delete files, or purchase storage.

Partial safe-stop evidence is explicitly application-resume-ineligible in the
backup manifest. Complete checkpoint state still does not authorize automatic
restart or renewed time/integration budgets.

Qualification uses temporary runtimes and can leave no active runtime. A stop
at that boundary retains the report, ledger, configuration and available parity
evidence without creating a runtime or invoking physics. Optional live runtime
capture failure likewise cannot discard terminal documents. Their restore
validator checks decoded documents and ledger agreement, not runtime-resume
readiness; an unavailable runtime is explicitly recorded rather than fabricated.

## Software verification (separate from physical acceptance)

- Real CPU Torch 2.9.1: populated Adam states for all optimizers, model/target
  weights, gradients, modes, alpha, counters and Python/NumPy/Torch/replay RNG
  restore exactly after a forced guard rejection and serialization roundtrip.
- Sixteen cold-start tests pass, including feature corruption, cap exhaustion,
  historical-runner closure, restore-before-prune ordering and nonphysics
  terminal retention with an absent or unavailable active runtime; exact-boundary
  negative tests and a no-physics one-reset dispatch test cover the amendment.
- Existing corrected-loop and backup suites: 23 + 23 tests pass.
- Full JavaScript test suite: 129 tests pass.
- Rebuilt actual flagged WASM: both unstepped runtime checkpoints load and
  preserve all reported state with zero native/forecast integrations.

These software checks do not establish physical improvement, a successful real
remote backup, or broad MVP readiness. Those results must be reported separately
from the actual single-use pilot ledger.
