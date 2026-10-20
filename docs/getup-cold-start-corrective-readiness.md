# Corrected worker and bounded recovery readiness

This is software/build evidence, **not a new physical trial, a trained policy,
or physical acceptance**. Both closed attempts remain immutable. The wrong-worker
records in the [variant audit](getup-cold-start-variant-audit.md) are diagnostic
only and cannot supply corrected-build training or qualification.

## Semantic identity before runtime loading

`scripts/getup-cold-worker-contract.json` specifies these exact Cargo roots:

```text
lh-worker/root-translation-carry-experiment
lh-sim/getup-terminal-balance-experiment
lh-sim/standing-dynamics-experiment
lh-sim/unit-root-rotation-experiment
lh-sim/getup-repeat-cycle-experiment
lh-worker/getup-terminal-tail-observer-experiment
lh-worker/getup-policy-adaptation-experiment
```

`build-getup-cold-worker.py` resolves the locked offline dependency closure,
builds into the existing shared target, and records selected Cargo artifacts and
fingerprints. Rust 1.89.0 and wasm-bindgen 0.2.108, the raw WASM, generated web
WASM/glue, lockfile, source-file inventory, recipe files, frozen profile and both
policy binaries are hash-bound. A dirty build is explicitly recorded as dirty;
its exact source files are bound rather than claiming an unpublished commit
already contains them.

The static `lh-sim` capability export reports the actual compiled dependency
features, not just the worker's forwarding flags. Its probe constructs no
runtime and integrates no samples. Python preflight and each cold runtime entry
point require the exact seven effective simulation features. Physical roll,
missing standing dynamics and all unreviewed variants are rejected.

The newly built WASM is
`7a14f61e25ca1c30c4eb6352cbf7c20ecf5afcd2e0aef8e5c95911689526cee0`
(14,932,472 bytes). It is a **new artifact**, not the lost historical binary and
not evidence of historical performance. New BACK/BELLY native-zero checkpoints
roundtrip without stepping. Their bytes can coincide with old native-zero
checkpoints because feature-dependent state has not yet been materialized;
their provenance is the new compiled worker, not reuse of old triggered seeds.

The profile, WEAK/SLOW reference policy, 32 reference solver passes and 20
post-handoff passes are unchanged. No mechanics, reward, controller tuning,
policy weights or physical acceptance thresholds are changed.

## Refusal is diagnostic evidence, never a replay transition

The production step path distinguishes a legitimate zero-integration refusal
from malformed lookalikes. It requires an explicit failure, terminal true,
success false, unchanged native and impact indices, null source epochs/previews,
no submitted action, an empty native trace, and no pending mapper tail. A valid
refusal preserves the original physical failure. Malformed observations fail
the diagnostic contract. Both stop without replay insertion or retry.

The mapper still requires its ordinary strict 20-integration chronology.
Diagnostic zero correction/residual arrays do not imply a submitted action.
No sample, decision, or charge is invented or refunded.

## Two owned slots instead of unsupported Trash rotation

The supported Drive update operation can replace an existing file's bytes.
There is **no exposed conditional ETag/version/CAS operation**. A single transfer
owner must verify the exact older slot before writing; a pre-read is not an
atomic conditional write.

`TwoSlotDriveExchange` uses two explicitly owned, previously application-verified
slots. The current slot stays untouched while the older slot is replaced:

1. Persist a nonce/hash-bound intent and conservatively charge its full archive
   bytes before exposing the request. Unknown completion never refunds them.
2. Replace only the older ID; verify exact downloaded archive, manifest and
   payload identities, privacy, nonce, and expected old bytes.
3. Independently restore the application state.
4. Atomically switch the current pointer only after that proof.

Only the current generation is guaranteed during replacement. Revision history
is not relied upon. Interrupted writes, corrupt downloads, failed application
restoration and interruption after pointer persistence all stop. Read-only
reconciliation distinguishes exact old bytes, exact new bytes and unknown bytes,
and distinguishes an unchanged pointer from an already application-verified new
pointer. It does not authorize another external write or an automatic restart.

The existing small 19,332-byte diagnostic archive remains outside the two-slot
inventory. Old wrong-variant immutable archives remain evidence, not compatible
inputs. Neither is deleted or silently omitted from accounting. Tests use the
actual production archive/request/receipt adapter with scripted transfers;
**no real slot overwrite has been exercised by this software work**.

## Measured storage and hard admission limits

All uploaded bytes remain cumulatively charged, including replacements,
potential revisions and Trash. Overwriting is not a quota credit.

| Component | Bytes / bound |
|---|---:|
| Already uploaded, including both closed attempts | 22,056,886 |
| New corrected shared archive, measured locally | 10,620,053 |
| Corrected shared + future metered seed archive allowance | 12 MiB |
| Qualification active output / terminal payload | 32 MiB each |
| One mutable terminal archive (not seven) | 18 MiB |
| Maximum additional corrected inputs + terminal archive | 30 MiB |
| Cumulative envelope including all old uploads | 53,514,166 |
| Hard cumulative ceiling | 167,772,160 |

The measured shared archive leaves 1,962,859 bytes for fresh, **metered** seed
archives; their actual size is not yet known. Shared and seed uploads, like slot
replacements, are conservatively charged before their request becomes visible.
Unknown completion is not a refund or permission to retry.

One terminal generation is allowed. It can serve both terminal retention and
independent populated diagnostic-state restoration; it cannot authorize training
or automatic resume. The previous current slot remains intact during replacement.
The 32 MiB connector download limit remains an additional route constraint.

For the measured protected old recovery files (57,041,539 bytes), corrected
shared working files (16,218,147 bytes), shared ZIP (10,620,053 bytes), at most
2 MiB seed payload and 1,962,859-byte seed ZIP, the conservative local staging
peak is 163,440,950 bytes including the shared bundle's small manifest sidecar.
Preflight remeasures this inventory; the 4 MiB metadata
margin is not spendable headroom. The executable stage maximum is the larger of
creation (two payload copies), old-slot pre-read (new plus old archive), and
transfer/restore (two new archives plus extracted payload). Old slot archives
are already inside protected files and are not counted a second time.

The **160 MiB local limit is recovery staging**, not total workspace usage.
The active original qualification output is separately bounded to **32 MiB**
and included in the reported total physical recovery footprint. Thus the maximum
reported recovery peak can be staging plus active output; no total-workspace
160 MiB claim is made. Repository, CPU environment, toolchain and shared build
output are separately measured, pre-existing files are retained, and no caches
or repository history enter backups.

The full software rehearsal uses the production phase methods, actual mapper,
replay, Torch learner, serialization, independent fresh-object restoration and
fresh-process continuation. Its seven archives measured approximately 4.90–6.16
MB; the conservative projected cumulative total was about 75.44 MB. It added
zero remote bytes. Synthetic compression is **not** a guarantee for real replay
or raw traces: actual payload/archive size must pass before every request.

Read-only sizing of the retained real rows rejects treating that full rehearsal
as a storage-feasible two-block admission. The post-optimizer synthetic learner
alone has 13,739,102 raw bytes. Scaling the observed real replay and source-row
sizes to 1,536 records projects roughly 38.1 MB of payload before extra tails
or candidate evaluation; additional preview-bearing evaluation rows can also
exceed the 18 MiB archive envelope. These are sizing scenarios, not corrected
worker performance or compression guarantees. The 17 MiB payload assumption is
therefore not an adequate full-training plan. No traces are discarded, limits
inflated, or actual full-training admission granted to work around that result.
The next resource plan is restricted to qualification-only sizing and evidence.

## Exact qualification-only operation and byte budget

The production CLI now admits only the hash-bound qualification plan, not a
full two-block training run. Its normal integration envelopes are:

| Operation, in order | Normal integrations | Wall allowance |
|---|---:|---:|
| Corrected immutable upload and unstepped restore | 0 | 300 s |
| Fresh BACK480, nearby560, BELLY at most2000 | 3,040 | 120 s |
| Fresh seed backup and unstepped restore | 0 | 180 s |
| Original BACK off/zero/restored parity (two forecasts +1000real) | 11,000 | 120 s |
| BELLY initialization + two actual next-transition executions | 5,060 | 60 s |
| Remaining BELLY recovery +30 s hold | 43,960 | 360 s |
| Nearby BACK recovery +30 s hold | 49,000 | 360 s |
| Optional fresh original BACK recovery, **no additional hold** | 25,000 | 240 s |
| Coherent terminal application restore | 0 | 180 s |
| Final retention outer allowance | 0 | 300 s |

Normal total is **137,060**, plus **one shared 5,020** uncertainty/overrun
reserve: **142,080**. The separate **147,000 candidate-evaluation reserve stays
untouched** (combined289,080 <300,000). Every native call still reserves5,020,
including forecast exposure; there is no optimistic20-only classifier. Each
phase stops when its normal allowance is consumed, so the shared reserve cannot
be repeatedly spent. A hold command is admitted with29,020, then reconciles its
individual calls. BELLY preparation checks its2,000 limit before dispatch.
Unexpected forecast, source rearm, failure or uncertain completion stops physics.

The **1,024 source-decision cap includes qualification, reset-generated first
SLOW decisions, both parity executions and preservation**. Counts are actual
controller deltas with one decision precharged before each call; WEAK teacher
work is separate. The global4,096 ceiling is not a substitute. The original
impact epoch, WEAK/reference-to-SLOW route, no physical roll, and32 active /20
terminal solver passes are checked in the budgeted calls. The first BACK call
requires32; later valid terminal handoff uses20 rather than being falsely
required to remain on32 for the entire qualification window.

The hour begins before the first upload/load. Each full stage allowance must fit
before dispatch; physics stops atT+2,100 and all retention atT+3,600. Work advances
immediately, not at scheduled waits. Unused capacity never grants a second trial,
optimizer operation, candidate or extra block.

### Measured current checkpoint plus bounded next-call growth

Before **every native call**, the bridge measures all tracked current runtime
checkpoints and current active files, then reserves complete terminal learner,
replay, metadata, state and evidence capture, plus a **20 MiB next-call growth**
bound and64 KiB of next raw/audit output. It does not assume an8 MiB whole runtime
or discard old rows. The frozen, hash-checked source/cardinality argument is:

- **12 MiB controller:** two JSON-string escaping layers (`runtime.rs310–345`),
  up to250 TeacherFrames (`getup/controller.rs30–46,770`), each at most44,335bytes
  with32-character numeric/u64 fields,25 poses before/after and25 aggregated
  contacts. Total11,083,750bytes, plus at most501 reference frames at speed0.5
  (`getup/reference.rs`), teacher initial snapshot and one StudentDecision.
- **6 MiB world/contact:**25 pose slots,19 sphere/capsule body primitives in the
  frozen model; floor, striker and **five room walls** (`striker.rs100–150`) give
  26 primitives and at most325 pairs. Allow16 contacts/pair,256bytes combined
  tracked/solver point,1024bytes manifold/metadata, two original/solver copies:
  `325 ×2 ×(16 ×256 +1024) =3,328,000bytes`. Add2 MiB fixed25-link/27-DoF matrices,
  collider trees, graph/workspace/arena metadata =5,425,152bytes; round to6 MiB.
  No geometry insertion is available in this frozen path; nonserialized
  pipelines/CCD/retired-pair state are not counted as serialized growth.
- **2 MiB auxiliary/head:** maximum21×1024 float head at32-character width is
  below0.7 MiB; standing/pending/terminal fields and one student record fit the
  remainder. The source hash inventory and locked compiled dependency closure
  bind this argument; different geometry or serialization fails preflight.

A software-only initial proof measured the untouched learner archive bound as
5,516,172bytes and corrected native-zero BACK checkpoint as585,694bytes. Including
2 MiB seed-output allowance,2 MiB terminal metadata,20 MiB growth and64 KiB next
record gives **31,333,226bytes**, leaving **2,221,206bytes** below32 MiB. This is an
**initial preparation sizing proof**, not a guarantee that the later multiple
runtimes, full trajectories or holds fit. Those states are measured before each
dispatch; an administrative byte stop can preserve useful partial qualification.

Complete source/provenance rows, failures, duplicates, mapping, runtime state and
learner/RNG are retained. If preparation stops before immutable seed verification,
any already-created seed checkpoint is included once as unverified, nonresumable
terminal evidence. Exceeding the2 MiB immutable-seed allowance closes seed
upload/readiness, but already-created bytes remain diagnostic evidence under
the unchanged32 MiB terminal and18 MiB archive caps. Their exact output and
complete terminal capture are reserved before each checkpoint write. Verified
immutable seeds are referenced rather than duplicated. Successful holds check all24,000 native samples each,
retain full-sample extrema/stability and first800/final checkpoints; they do
**not archive all48,000 raw hold nativeTrace samples**. A failed hold retains its
checked/passed sample counts, observed maxima, original anchors and first error.

Diagnostic replay may have incomplete strata. Its exact case/outcome/phase
membership, approved-stratum counts, samples and RNG are independently restored;
unfinished BELLY/nearby rows do not become BACK training coverage. Diagnostic
batching and all optimizer entry points remain closed. Tests exercise a partial
byte stop, failed reset/hold capture, uncertainty, one-slot transaction and fresh
application restore using the production path with scripted I/O only.

## Gates still distinct

Software tests and nonstepping build receipts do not open a trial by themselves.
A separately admitted corrected-worker prefix must meter new fixtures and
forecasts, prove BACK enters the intended WEAK/SLOW path without resetting the
impact clock, establish actual off/zero/restored transition/reward/mapping parity,
and preserve nearby BACK and BELLY before any optimizer update. It must obtain
a coherent populated replay/learner/optimizer/RNG/controller/cursor/ledger
operational restore. Old wrong-variant parity supplies none of those proofs.

Original BACK may remain the learning target. Extra baseline preservation holds
must have their own explicit allowance; they cannot borrow the protected
147,000 integrations for candidate evaluation. The original acceptance remains
25 seconds after impact, 800 continuous standing samples, 30-second holds,
10 mm foot-origin anchors, the existing 20 mm maximum anchored material-point
excursion metric (not cumulative path travel), and unchanged contact, motion,
solver, torque and joint limits.

No automatic new attempt, resource extension, candidate promotion or broad MVP
completion follows from this readiness checkpoint.

## Software verification at this checkpoint

109 focused checks passed:14 production-engine rehearsal,10 corrective/slot
fault-path,8 qualification end-to-end,16 cold-contract,23 learning-loop,
23 archive/restore,9 existing JS contracts and6 qualification-budget checks.
The qualification rehearsal uses real mapper/replay/learner serialization and
independent restoration with scripted physics/transport; its sole terminal ZIP
was about4.90 MB (5.49 MB raw). It performs zero optimizer attempts and produces
no physical candidate. The broader engine rehearsal uses disposable synthetic
optimizer data only. Temporary rehearsal archives are removed afterward.

The corrected compiled-capability probe and exact build-receipt validation also
passed without constructing a runtime or integrating physics. Real semantic,
transition parity, preservation, remote two-slot overwrite and populated real
application-restore gates remain untested by this software-only checkpoint.
