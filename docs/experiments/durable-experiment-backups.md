# Durable experiment recovery points

Git preserves committed source. Ignored replay, optimizer/RNG state, physical
checkpoints, and budget ledgers require separate private storage. A directory on
the same executor is staging, not a durable backup.

`scripts/experiment_backup.py` provides a stdlib-only snapshot/restore mechanism
and a fail-closed private Library backend. It does not start physics or training,
load a checkpoint, reset a deadline, or recover already-lost files.

## What is saved

The paused writer supplies an explicit list of regular, nonsymlink files, with a
unique role, artifact schema, provenance, and `user_generated` or `project_source`
classification. No workspace globbing is used. Credential/configuration paths
such as `.env`, `.aws`, `.codex`, private keys, and Git internals are rejected.
Classification remains the caller's responsibility; the tool is not a content
classifier.

Each immutable ZIP contains `manifest.json` plus precisely its listed artifacts.
The manifest records SHA256 and byte length for every file, source commit and
dirty-tree description, frozen config, seed, provenance, stable run ID,
generation ID, previous manifest hash, required roles, and excluded dependencies.
For a learning-loop generation, include the exact physical checkpoint, all
replay arrays and row metadata, learner/target/optimizer state, learner and replay
RNG, reports, budget/deadline ledger, frozen config, and input manifest. Producers
must be paused before copying. A file changing during copying aborts staging.

Upstream GetUp weights, XML, or binaries embedding them are not implicitly
included. Unverified redistribution rights mean reference-only entries with
exact hashes, source/reacquisition instructions, and an explicit resume blocker.
A reference is not retained data. Any required excluded dependency makes
`full_resume_ready` false. No claim is made that unavailable upstream inputs
can be reacquired.

## Durability and interrupted writes

Snapshots use a private staging directory, file/directory fsync, verification,
and an atomic generation rename. Previous generations are never automatically
deleted or replaced. Interrupted staging directories are not recovery points.
Restore validates the archive and all manifest hashes before extracting into a
new isolated directory, then verifies the extracted files and atomically installs
the result. It refuses path traversal, duplicates, symlinks, unexpected members,
and overwrite of an existing run. An explicit expansion limit defaults to 8 GiB.

`LibraryBackend` uses current official Library helpers supplied privately at
runtime, outside the repository. Prepared upload supports the larger-file route
(including files around 50 MiB and above); server limits still apply. It sends no
base64 archive or credentials through configuration. A failed or uncertain write
is not automatically retried. A failed remote operation latches the recovery
guard closed, including terminal-save paths, until Library is inspected. Upload
is acknowledged only after actual Library
materialization, independent restore, and archive/payload SHA256 verification.
The remote round trip has an original-deadline timeout and cannot authorize a
budget reset.

Archive names contain `LaboratoireHumain-backup-<stable-run-id>-<generation>.zip`.
Search Library by that run ID after executor loss, inspect the generations, and
download through the current Library workflow. The complete manifest and parent
hash are inside every archive; a lost local locator/index does not hide the
archives. Rebuild a recovery proof from the retained archive's confirmed Library
identity and exact hash. Local proof files are caches, never standalone evidence
of current remote availability.

## Run admission and checkpoint hooks

Before constructing a model, runtime, or new experiment deadline, call
`require_recovery_point(receipt_path, input_manifest, required_roles, backend,
run_id=stable_run_id)`. It performs a **new** remote round trip, rejects synthetic
or incomplete points, binds the exact input-manifest SHA256, and matches every
required input role against that manifest's file SHA256. The existing retained
input hash checks and source-only execution blocker remain mandatory.

`RecoveryGuard(root, run_id, backend, source, synthetic=False).checkpoint(...)`
takes `artifacts`, `config`, `seed`, `provenance`, and keyword-only
`required_roles`, `excluded_dependencies`, and the original absolute `deadline`.
It returns a proof path only after successful retention and restore verification.
Real incomplete points throw even if the partial archive was retained. Call while
the physical episode is paused after each 256-decision collection block, after
its update block, and at a clean final stop. Preserve the exact physical state
and cumulative ledger through those pauses. Any failure keeps the run paused;
there is no local-only fallback or permission to continue accumulating costly
work. The absolute watchdog can still terminate the process before a final
save, so the most recent completed durable generation is the recovery boundary.

Standalone commands:

```sh
python3 scripts/experiment_backup.py snapshot --spec /private/spec.json --root work/experiment-backups
python3 scripts/experiment_backup.py retain /private/generation.zip --library-helpers /private/current-library-helpers --proof-directory work/recovery-proofs
python3 scripts/experiment_backup.py restore /private/downloaded-generation.zip --destination work/isolated-restore --sha256 EXPECTED_ARCHIVE_SHA256
python3 scripts/experiment-backup-test.py
```

The spec uses the manifest fields above plus `artifacts`, whose entries contain
`role`, absolute `path`, `schema`, `provenance`, and `retention_class`. A
`required_roles` list cannot omit a file required to resume the actual caller.
Keep staging, proof files and archives under ignored `work/`; do not publish raw
weights, replay, checkpoints, Library identities, signed URLs, or credentials.

## Verification on 2026-10-20

- Twenty-three synthetic stdlib contract tests pass without network, physics, or
  training. They cover interrupted copies, hash mismatch/corruption, safe atomic
  restore, missing prerequisites, exact role hash binding, synthetic admission
  rejection, upload failure/no acknowledgment, deadlines, and redacted errors.
- Two synthetic generations with nine artifacts each were locally restored and
  hash-verified. They contain fake replay/checkpoint/ledger values plus explicitly
  selected current project source/configuration. They are not historical data.
- **Durable upload and remote restore have not passed.** The current official
  Library helper failed, and a read-only diagnostic of its authenticated tool
  discovery returned HTTP 403. Direct Library inventory was readable but showed
  no newly retained drill bundles. No alternate upload route was used and no
  durable-success receipt was issued. The cause of the helper denial is unknown;
  no new credentials, paid storage, or user reconnection is prescribed.
- Consequently the real experiment remains blocked. Resolve the supported
  Library-helper access problem and successfully run a real retained-input
  round trip before claiming restart safety or admitting another costly run.
  Application-level resumption still requires its separately reviewed adapter;
  byte restoration alone is not permission to restart or replace missing data.
