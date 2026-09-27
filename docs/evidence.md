# Evidence availability and preservation

Checked during the 2026-09-26 documentation repair. The original review found 31 broken evidence-link occurrences. Their paths remain recorded in the [documentation checkpoint inventory](checkpoint-2026-09-26.json), including when a surviving copy was recovered. Missing files are explicitly labeled in the historical reports. A path or a reported test count alone does not establish acceptance.

## Recovered portable artifacts

The temporary directories named by the investigations still contained some original JSON reports. The following copies now live with the documentation, outside the ignored `evidence/` directory. The [recovery manifest](checkpoints/2026-09-26/recovery-manifest.json) records original locations, SHA-256 digests, stored/original byte counts, and compression provenance.

| Artifact | Availability and meaning |
| --- | --- |
| [Standing baseline](checkpoints/2026-09-26/baseline.json) | Byte-identical surviving baseline with executable/dependency information, source hashes, and historical evidence hashes. Its assertions describe capture time. |
| [H5 intervention report](checkpoints/2026-09-26/allocator-jump-h5-report.json.gz) | Original report preserved with lossless gzip compression; decompress to inspect commands, comparisons, and runs. |
| [Original allocator replay report](checkpoints/2026-09-26/allocator-replay-verification.json) | Surviving offline replay results and the original input checksum. The input fixture itself is unavailable. |
| [H6 static experiment](checkpoints/2026-09-26/allocator-projection-h6-static.json) | Original rejected static-screen results. |
| [H7 static experiment](checkpoints/2026-09-26/allocator-projection-h7-static.json) | Original rejected static-screen results. |
| [Official idle scenario output](checkpoints/2026-09-26/official-idle-results.json) | Original scenario result; separate from the missing command/before-after source manifest cited in the repair ledger. |

These six artifacts were copied or compressed from surviving files, not recreated by rerunning physics. They retain their original source scope. The SHA-256 of decompressed H5 bytes matches the original report. For example, decompress without changing the archive:

```sh
node --input-type=module -e "import {readFileSync} from 'node:fs'; import {gunzipSync} from 'node:zlib'; process.stdout.write(gunzipSync(readFileSync('docs/checkpoints/2026-09-26/allocator-jump-h5-report.json.gz')));"
```

## Still unavailable

The original standing `index.json`, the two-tick `production-ticks-175-176.jsonl.gz` input, and the focused 140/142 test report/log are unavailable at their documented paths. The named H5 `allocator-jump-h5/0-plain.jsonl` is empty, so it cannot restore the two-tick fixture. The surviving replay report records the missing compressed fixture's SHA-256 as `f4ae9784f98fc88bcf72cda036944af220704683de88d878c16890bca8b8daa8`; a replacement must match this digest before being called the original artifact.

The originally broken Markdown targets are enumerated in the checkpoint inventory; this is not an exhaustive inventory of every path mentioned in historical prose. Counts such as 140/142 and the original 57 diagnostic runs remain attributed to the written investigation, with the available supporting files identified separately. The documentation repair did not reverify every historical run or all previously cited evidence hashes. Some large trace paths still exist as empty files; filesystem existence alone is not sufficient evidence availability.

To recover a missing artifact, obtain the original bundle for the named run from its author or retained CI artifact, verify its recorded checksum and source manifest, and add its actual durable location here. No complete bundle download URL is currently recorded. Do not substitute a newly generated result under an old run's filename or infer a pass from a different source selection.

### Fresh baseline capture

[capture-physics-baseline.mjs](../scripts/capture-physics-baseline.mjs) now supports an explicit fresh capture without historical inputs. From the repository root, with the locked dependencies installed, choose a run directory that does not exist:

```sh
node scripts/capture-physics-baseline.mjs --fresh evidence/physics-standing-fresh-001/baseline
node --test tests/capture-physics-baseline.test.mjs
```

The output records the executable version/digest, installed direct dependency versions and package metadata digests, source/configuration hashes before and after capture, physics settings, and frozen planned standing inputs. `scenarios.execution` is `not-run`: this command records a baseline and executes no physical scenarios. Source hashing also covers hooks, shared libraries, workers, assets, vendored files, and root tool configuration; documentation is represented in the uncommitted-work snapshot when changed.

`--fresh` always sets `savedFocused` to `null`, even if historical reports are present. The evidence inventory distinguishes missing, empty, and available files and reports digest matches when historical checksums are known; availability alone does not establish valid evidence. Without `--fresh`, capture still requires the original focused report's digest and matching source subset. A missing or changed report fails with a message directing the caller to the fresh mode.

Uncommitted files are copied into `preserved-worktree/` with digests, deletions are listed, and `index.patch` and `worktree.patch` preserve staged and unstaged changes separately. Every capture requires a new output directory, including after a partial failure. Keep runs outside source, configuration, dependency, and Git directories. Existing reports are never overwritten.

The first fresh run succeeded at `evidence/physics-baseline-20260926-fresh-01`. Its [portable baseline](checkpoints/2026-09-26/fresh-baseline.json) is a byte-identical copy of the new run's ledger, separate from the recovered historical baseline. It records 252 source/configuration hashes and all four named historical inputs as missing. The [validation manifest](checkpoints/2026-09-26/fresh-baseline-validation.json), [five-test CLI log](checkpoints/2026-09-26/fresh-baseline-tests.tap), and [passing targeted ESLint check](checkpoints/2026-09-26/fresh-baseline-lint.json) identify the tested source. The full preserved worktree and patches remain local-only in the ignored run directory, with their digests in the portable ledger; there is no durable bundle URL yet. Documentation status updates after capture are not part of that worktree snapshot.

### Regenerated allocator input and H8 evaluation

[The new capture and H8 evaluation](standing-h8-evaluation.md) provide a nonempty 175–181 trace window and matching control/replay histories at +π/3. The production allocator reproduces ticks 175–176 exactly. H8 passes its static/local checks and fails all six prolonged screens; it is rejected. The [archive manifest](checkpoints/2026-09-26/h8-evaluation/manifest.json) retains 29 compressed files with verified compressed/uncompressed digests, including production source, histories, and failed runs. These small archives live with the docs and require no temporary directory. They are new evidence and do not replace or recover the missing historical original. See the report for replay and archive-verification commands.

### H12–H21 continuation and small impulses

The [H12–H20 archive](checkpoints/2026-09-26/h12-h20-evaluation/manifest.json)
contains 78 verified compressed artifacts. The
[H21 and small-impulse archive](checkpoints/2026-09-26/h21-impulse-evaluation/manifest.json)
contains 96. Verify each with `node scripts/archive-standing-evidence.mjs verify`
followed by its manifest path. Both compressed and decompressed digests are checked.
The archives retain failed results, production snapshots and diagnostic source
at archival time. Compare source hashes with each run: diagnostic scripts evolved
between experiments, so archival snapshots are not claimed to be every historical
intermediate script. Production/package hashes match throughout.

Long standing event traces are reduced to explicitly marked peak windows, with
their original full-file digests and local-only paths retained. H20/H21 capture
only ticks 175–181 as detailed events; their reports retain every frame's response.
All small-impulse paired JSON snapshots are retained. The first failed translated
fixture attempt has a failure record and log, with its missing intermediate script
explicitly qualified. These new runs do not recover the missing older artifacts
or close the release-wide durable-evidence requirement.

The [standing report](standing-h12-contract.md) records every rejected hypothesis.
The [impulse report](small-impulse-diagnostic.md) reproduces the original outlier,
then records separate numerical sensitivity checks without changing its assertion.
[Validation](checkpoints/2026-09-26/continuation-validation-02/manifest.json)
records 24 passing targeted tests and clean targeted lint. Its preceding failed
validation remains preserved: a new test helper initially misclassified a tiny
two-point segment by using the production margin helper's edge-length floor;
exact convex-segment membership fixes the test without changing physics limits.

The [H22–H24 archive](checkpoints/2026-09-27/h22-h24-evaluation/manifest.json)
adds 32 verified artifacts, including the partial failed H23 capture and later
complete regeneration. Its bounded traces are compressed in full, with no peak
extraction. [Post-H24 verification](checkpoints/2026-09-27/continuation-validation/manifest.json)
records 24 targeted tests, clean targeted lint and exact replay of all three
plain/observed/repeated original standing histories. The phase-aware trace fix
does not change physical commands or turn any failed standing result into a pass.

### H27–H29 constraint, runtime and balance-target comparisons

The [27-artifact archive](checkpoints/2026-09-27/h27-h29-evaluation/manifest.json)
retains the three rejected candidates, complete screen responses, conversion
snapshots, exact control comparisons and production/diagnostic source snapshots.
It also preserves the official Rapier 0.21.0 package tarball, registry metadata,
SHA-512 integrity receipt, upstream binding sources and failed original test
output. Package integrity is verified against the captured metadata; no
provenance-signature verification is claimed. The package was used only by a
process-local diagnostic loader. Production remains on Rapier 0.20.0.

Run `node scripts/archive-standing-evidence.mjs verify docs/checkpoints/2026-09-27/h27-h29-evaluation/manifest.json`
to verify the retained bytes. Read [H27](standing-h27-contract.md),
[H28](standing-h28-contract.md) and [H29](standing-h29-contract.md) for scope and
rejections. [Nine archive tests and targeted lint pass](checkpoints/2026-09-27/h27-h29-validation/manifest.json).

The [H30 archive](checkpoints/2026-09-27/h30-evaluation/manifest.json) retains ten
artifacts, including all command samples through completion of the two-second
centroid transition. The [H31 archive](checkpoints/2026-09-27/h31-evaluation/manifest.json)
retains the centered initial-posture solve, construction measurements and failed
physical screens. Both use the original runtime and preserve exact H22 controls;
neither establishes an accepted repair.

### Small-impulse precision isolation

The [precision archive](checkpoints/2026-09-27/impulse-precision/manifest.json)
retains exported binary worlds, paired metadata, complete native f32/f64
one-step results, exact WASM restoration and fresh-pipeline checks, matching
engine-source/feature receipts, failed/confounded attempts, and the independent
fixed-assembly comparison. Read the [diagnostic conclusion](small-impulse-diagnostic.md#result--numerical-precision-sensitivity-isolated)
before interpreting preliminary batches: the first f64 default enabled a
different solver, and the first WASM replay omitted the event queue needed
for collision hooks. Corrected comparisons preserve those controls.

The canonical native batch is `impulse-native-precision-20260927-06`; corrected
WASM replays are `impulse-snapshot-replay-20260927-02` and `-03`. Native batches
03/04 and the first fixed-assembly f64 transfer stop before physics on strict
transfer assertions. The final transfer check excludes only documented zero
alignment padding; pending joint mutations are instead processed by the
unchanged WASM fixture before exporting a new snapshot. All 65 application
source/package hashes still match the regenerated capture.

[Nine original motor tests and targeted lint pass](checkpoints/2026-09-27/impulse-precision-validation/manifest.json).
The earlier 25-test archive checks remain valid for their unchanged files.
This resolves the precision diagnosis, not quiet standing or release acceptance.

### H25–H26 and native constraint calibration

The [verified archive](checkpoints/2026-09-27/h25-h26-calibration/manifest.json)
contains 55 compressed artifacts: both rejected six-run screens, complete
bounded event traces, kinematic analysis, simple-joint and rigid/compound
calibration results, failed exporter evidence, native binary snapshots and
metadata, the seven-mode native batch, upstream-source receipts, and current
production/diagnostic source snapshots. Six prior screen controls match exactly;
65 production source/package digests remain unchanged. Historical intermediate
script and executable builds are not all retained; the final native batch uses
one hashed executable and a complete before/after source fingerprint.

`archive-standing-evidence.mjs` now retains `.bin` snapshots byte-for-byte under
gzip as well as JSON/JSONL. [Targeted verification](checkpoints/2026-09-27/h25-h26-validation-complete/manifest.json)
records 25 passing tests, including deliberate binary-archive corruption, and
clean targeted ESLint. The two continuity tests ran separately after a misspelled
filename in the first command was discovered; both batches have identical
source fingerprints. The native Rust calibration builds successfully; its
compiler/crate caches and executable remain local-only and are reproducible
from the recorded lockfile and commands.

See [H25–H26](standing-h25-contract.md) and
[native calibration qualifications](rapier-constraint-calibration.md). Altered
calibration fixtures and solver flags are not application settings or acceptance
results. The missing original evidence remains missing.

## H32 onward

All archives below are gzip bundles with recorded original and compressed
SHA-256 digests, verified by `archive-standing-evidence.mjs verify`.

| Investigation | Durable manifest | Scope |
| --- | --- | --- |
| H32 CCD | [8 artifacts](checkpoints/2026-09-27/h32-ccd-calibration/manifest.json) | Native rigid collapse already present in identical retained samples before post-impact divergence. |
| H33 block solver | [13 artifacts](checkpoints/2026-09-27/h33-block-calibration/manifest.json) | Matched source/feature graphs; both native variants collapse; normal executable restored. |
| H34 integral feedback | [10 artifacts](checkpoints/2026-09-27/h34-integral-evaluation/manifest.json) | Three rejected screens; controls replay exactly. |
| H35 interval damping | [11 artifacts](checkpoints/2026-09-27/h35-interval-evaluation/manifest.json) | Three rejected screens plus angular interval analysis. |
| H36 copied-world response | [13 artifacts](checkpoints/2026-09-27/h36-response-model/manifest.json) | 89 cases per heading; exact 25-body first-step clone/live match; no transfer or standing acceptance. |
| H37 runtime comparison | [15 artifacts](checkpoints/2026-09-27/h37-runtime-comparison/manifest.json) | Verified npm package, exact upstream binding receipt, 14/21 old-runtime tests, 21/21 pinned-loader tests, common-mesh rigid collapse and measured inertia differences. |
| H38 geometric inertia | [20 artifacts](checkpoints/2026-09-27/h38-inertia-evaluation/manifest.json) | Independent integrals, corrected tensor readback, all paired screens and pre-production source snapshot. |
| Production inertia candidate | [Candidate evidence](checkpoints/2026-09-27/inertia-production-candidate/manifest.json) | Initial 47/49 and subsequent 48/49 tests, preserved original/updated tests, controlled impulse-response measurement correction, unresolved chest contact, typecheck, and three failed 2+30 s captures. |
| H39 interval feedback on corrected inertia | [Verified comparison](checkpoints/2026-09-27/h39-interval-evaluation/manifest.json) | All three angular-speed screens fail; no controller adoption. |
| H40 bounded hand reach | [24 artifacts](checkpoints/2026-09-27/h40-bounded-reach/manifest.json) | Original/failed/final contact traces, 61/63 final targeted tests, old-compositor anatomy replay, typecheck, lint, and exact idle peak control replay. |
| H40 full idle preservation | [4 artifacts](checkpoints/2026-09-27/h40-idle-preservation/manifest.json) | All three 2+30 s captures exactly replay the prior inertia candidate's physical histories and responses; standing remains failed. |
| H41 relaxed elbow clearance | [12 artifacts](checkpoints/2026-09-27/h41-elbow-clearance/manifest.json) | 66/67 selected tests, unchanged swing-foot failure, geometric tests, typecheck, final clean targeted lint, and exact three-heading 2+30 s H40 replay. |
| H42–H43 initial constraint checks | [10 artifacts](checkpoints/2026-09-27/h42-h43-initial/manifest.json) | Corrected-inertia rigid comparison, successful native hinge converter build, preserved initialization, first failed standing screen and source snapshots. |
| H42–H43 failure isolation | [19 artifacts](checkpoints/2026-09-27/h42-h43-followup/manifest.json) | Native capability sources, exact plain control and partial candidate repeats, copied native non-finite fields, rejected hinge representation, and clean targeted diagnostic lint. Intermediate reusable-buffer matrix fields are explicitly superseded. |
| H44 finite mass/COM guard | [18 artifacts](checkpoints/2026-09-27/h44-finite-mass/manifest.json) | Two red/green regressions, 68/69 targeted tests, typecheck, lint qualification, exact three-heading 2+30 s replay, and tick-133 rejection of the same non-finite physical trajectory. |
| H45 forefoot frame | [10 artifacts](checkpoints/2026-09-27/h45-forefoot-frame/manifest.json) | Three exact H22 controls and three rejected frame-correction screens; source snapshots and bounded trace extracts. |
| H46 two-axis authority | [20 artifacts](checkpoints/2026-09-27/h46-forefoot-authority/manifest.json) | 122 copied cases and exact live 25-body pulses; +π/3 remains above the local angular bound. |
| H47 six-axis distal authority | [21 artifacts](checkpoints/2026-09-27/h47-distal-authority/manifest.json) | 627 copied cases, exact live pulses and three bounded five-step windows; earlier uncorrected +π/3 peak remains. Two numerical tests and diagnostic lint pass. |
| H48 online distal preview | [121 artifacts](checkpoints/2026-09-27/h48-distal-preview/manifest.json) | Exact 25-body prediction on 2,160 live steps; all post-settle speeds pass, but +π/3 foot drift rejects the short screen. Every triggered snapshot and trial is retained; runtime remains unaccepted. |
| H49 horizontal preview short screen | [4,314 artifacts](checkpoints/2026-09-27/h49-horizontal-preview-screen/manifest.json) | All three 2+10 s screens pass; exact live prediction on 2,160 steps; full snapshots/trials; 67 unchanged production/package files. Short raw files were [relocated and verified](checkpoints/2026-09-27/h49-local-relocation-verification.json). |
| H49 horizontal preview long evaluation | [11,513 artifacts](checkpoints/2026-09-27/h49-horizontal-preview-long/manifest.json) | All 5,760 live steps match exact previews and speed bounds pass, but all three 2+30 s foot-drift bounds fail. Full data and source retained; no standing repair accepted. |
| H50 horizontal-search convergence | [13 artifacts](checkpoints/2026-09-27/h50-horizontal-convergence/manifest.json) | 324 matched copied queries; one full calculated update beats four limited updates at each retained state. Original motor caps and speed guards hold. No online runtime or standing acceptance follows. |
| H51 isolated swing-range candidate | [5 artifacts](checkpoints/2026-09-27/h51-swing-range-candidate/manifest.json) | Four unchanged anatomy tests pass on a temporary candidate; 48 swing poses remain connected/bounded and 21 idle/boundary comparisons are exact. Production verification is separate. |
| H51 production swing-range correction | [6 artifacts](checkpoints/2026-09-27/h51-swing-range-production/manifest.json) | 69/69 unchanged selected tests, typecheck and modified-file lint pass; three exact 2+30 s H44 plain-history replays; only declared pose changes among 67 production/package files. Standing and later gates remain open. |

The [current candidate report](inertia-correction-2026-09-27.md) separates the
verified mass-property defect from unaccepted controller behavior. It changes
the production factory and adds a geometry module; earlier unchanged-source
statements are historical. Native executables/caches remain local-only; their
sources, lockfile, build receipts and digests are preserved. No artifact in this
section is a complete release gate.

The H52–H55 continuation retains [native-inspector preflight](checkpoints/2026-09-27/h52-native-inspector-preflight/manifest.json),
[knee authority](checkpoints/2026-09-27/h52-knee-authority/manifest.json),
[dimensionless search](checkpoints/2026-09-27/h53-dimensionless-authority/manifest.json),
[guarded full-leg search](checkpoints/2026-09-27/h54-full-leg-authority/manifest.json)
and [continuous full-leg preview](checkpoints/2026-09-27/h55-full-leg-preview/manifest.json).
Their respective archive counts are 5, 16, 15, 15 and 1,511. Native inspection
verifies exact joint payload bytes and unchanged set membership; reserialization
only reorders the island wake-up hash set. Original snapshots feed every JS
query. H55 fails all three short speed screens and is not adopted. Its compact
logs retain every trial's metrics, motor bias and body digest, all chosen live
bodies, and checkpoint/failure binaries. See [H55](standing-h55-contract.md).

H56 preserves the [zero-target pelvis task and exact H54 control replay](checkpoints/2026-09-27/h56-pelvis-authority/manifest.json)
in 29 artifacts, and the [bounded pelvis task](checkpoints/2026-09-27/h56-bounded-pelvis-authority/manifest.json)
in 15 artifacts. Both archives verify; the bounded fourteen-axis variant meets
all three local task targets, while eight-axis variants fail. This remains
local actuation evidence; continuous H57 and production acceptance are separate.

H58 retains [the first failed H57 state](checkpoints/2026-09-27/h58-angular-full-step/manifest.json)
and [the second failed state](checkpoints/2026-09-27/h58-angular-full-step-additional/manifest.json)
in two verified five-artifact archives. All 17 and 209 original angular queries
replay exactly; the unchanged full-step rule meets the angular reserve in both
states within original caps. Each report embeds its standalone invocation source
and fingerprint. These local queries did not modify the H57 histories.

The [later H58 queries](checkpoints/2026-09-27/h58-angular-full-step-late/manifest.json)
retain eight verified artifacts and fail to correct the late +π/3 hand and
forefoot peaks. The [completed H57 archive](checkpoints/2026-09-27/h57-pelvis-preview/manifest.json)
retains 202 verified artifacts with all 425,534 trial metrics and digests, 2,160 exact live matches, the
ninety-step compatibility check, and native motor inspections. All three short
screens fail speeds. The verifier's signed-zero serialization discrepancy and
its explicit repair are retained separately from the unchanged physical runs.

The original H45 raw directory has been [relocated and verified](checkpoints/2026-09-27/h45-local-relocation-verification.json)
to preserve workspace capacity for new archives. All eight files and
428,034,244 bytes match their recorded original hashes. The compressed H45
archive is unchanged; complete raw traces remain local-only at the path in
the [relocation receipt](checkpoints/2026-09-27/h45-local-relocation.json).

The [H59 archive](checkpoints/2026-09-27/h59-full-angular-preview/manifest.json)
retains 114 verified artifacts with 383,383 copied trials and 2,160 exact live matches. Two short screens
pass; −π/4 fails eight hand-speed samples, so the standing prerequisite remains
open. [H60's ten verified artifacts](checkpoints/2026-09-27/h60-arm-authority/manifest.json)
retain exact native arm configurations and all eighty-four copied queries for
two different hand failures. Both local corrections meet unchanged speed guards;
the continuous arm-stage evaluation and production adoption remain separate.

[H61's thirteen initial-verification artifacts](checkpoints/2026-09-27/h61-initial-native-verification/manifest.json)
verify the original native configuration of all thirty-four selected axes at
each heading, preserving exact first snapshots, original motor receipts and
seventy-five chosen/live body matches. Signed-zero serialization differences
are recorded. This initial verification does not establish a standing result.

The [completed H61 archive](checkpoints/2026-09-27/h61-arm-preview/manifest.json)
contains 150 verified compressed parts reconstructing 70 original files. It
retains all 415,092 queries and 2,160 exact live matches; 714 native motor-axis
checks preserve three explicitly recorded JSON signed-zero differences. All
speed screens pass, but +π/3 foot drift fails at 10.095 mm. Average copied-world
search cost is 1.30–1.55 seconds per live step, so this is not an interactive
production controller.

[H62's fifteen verified artifacts](checkpoints/2026-09-27/h62-foot-position-authority/manifest.json)
retain all 1,202 copied queries, original native states, initial foot references
and invocation source. Position feedback reduces reference error at all three
saved states; only +π/3 reaches the complete local task deadband. The continuous
H63 evaluation is separate; its [historical launch checkpoint](checkpoints/2026-09-27/h63-launch-checkpoint.json)
records the exact thirty-step H61 compatibility replay and unchanged production
source.

The [completed H63 archive](checkpoints/2026-09-27/h63-foot-position-preview/manifest.json)
contains 243 verified parts reconstructing 155 originals. All three speed screens
fail, including a +π/3 fall. All 1,906 chosen 25-body predictions are exact; the
original state guard skips 254 later steps. The archive retains all 417,945
queries, full responses and verification of 612 native motor-axis configurations.
Independent kinematic checks explicitly distinguish complete 25-body histories
from the fallen history's post-preview response-only interval.

[H64's fourteen verified artifacts](checkpoints/2026-09-27/h64-coupled-speed-authority/manifest.json)
retain 1,769 copied queries; the coupled search reduces violations but misses
the declared guards. [H65's 29 verified artifacts](checkpoints/2026-09-27/h65-expanded-probe-authority/manifest.json)
retain 947 completed queries and three earlier comparator failures. Expanding
an improving probe direction reaches the local speed reserve at all three
states. All 210 initial H64 control queries replay exactly except explicitly
recorded residual zero signs; body fields and every other value remain strict.
These isolated-state results do not establish continuous or production standing.

[H66's six verified helper-replay artifacts](checkpoints/2026-09-27/h66-coupled-helper-replay/manifest.json)
preserve exact reproduction of all 947 H65 queries, including 23,675 individual
body comparisons. The [launch checkpoint](checkpoints/2026-09-27/h66-launch-checkpoint.json)
records the exact thirty-step H63 compatibility replay, unchanged production
files and three live continuous screens. It is not a completed standing result.
The [eleven-artifact H66 preflight archive](checkpoints/2026-09-27/h66-preflight/manifest.json)
is also verified and retains the complete compatibility output and three
first-step records, including the bounded +π/3 startup correction.

The [completed H66 short-screen archive](checkpoints/2026-09-27/h66-coupled-speed-preview/manifest.json)
contains 174 verified parts reconstructing 59 originals. All three short screens
pass, with 2,160 exact live predictions, 489,616 copied queries and 714 native
motor-axis checks. Every velocity, endpoint drift, maximum excursion and vertical
range is independently recomputed from the stored 25-body states. Ten coupled
fallbacks reach their reserve targets. The [long-evaluation launch checkpoint](checkpoints/2026-09-27/h66-long-launch-checkpoint.json)
records three active 1,920-step histories under the same source; short success
does not establish their result or production acceptance.

[H67's nineteen verified artifacts](checkpoints/2026-09-27/h67-previous-correction/manifest.json)
retain twelve local queries, 102 native motor-axis checks and an earlier exact
derived-metric comparison failure. Previous-command seeds satisfy speed guards
but miss the task at all three tick-240 states. [H68's six verified artifacts](checkpoints/2026-09-27/h68-cached-task-matrix/manifest.json)
retain 47 new queries using response columns from the preceding step. Guarded
improvements exist, but only heading 0 beats H66's original selected task score.
The old tick-239 native states were not saved, so their cached columns remain
derived log evidence; complete source rows and tick-240 states are retained in
H67. Neither comparison establishes continuous warm starting or frame rate.

[H69's complete preflight archive](checkpoints/2026-09-27/h69-preflight-complete/manifest.json)
has fourteen verified parts reconstructing thirteen originals, including nested
simulation data and loader logs omitted by the earlier shallow archive. Its
[complete continuous archive](checkpoints/2026-09-27/h69-warm-cache-preview/manifest.json)
has 242 verified parts reconstructing 198 originals. It retains 144,343 queries,
2,147 exact live predictions, thirteen guarded skips, 680 native motor-axis
checks, independent analysis and all injected source/provenance. Heading 0
passes; −π/4 fails speeds and +π/3 falls. No long H69 evaluation is authorized
by these failed screens. `evidence/archive-preview-evidence-recursive.mjs`
extends the format-2 archiver to every file in nested input directories; the
exact archiver source is embedded in each archive.

[H70's preflight](checkpoints/2026-09-27/h70-preflight/manifest.json) reconstructs
twelve originals from thirteen verified parts. The disabled candidate reproduces
thirty H66 steps exactly; the [enabled launch](checkpoints/2026-09-27/h70-launch-checkpoint.json)
is an ongoing −π/4 short screen, not an acceptance result.
[H71's sixteen verified artifacts](checkpoints/2026-09-27/h71-physical-fallback/manifest.json)
retain ten exact local copied checks and 102 native motor-axis inspections.
Physically admissible fallback candidates replay at H66 long tick 978 and H69
−π/4 tick 497; none exists in H69's recorded +π/3 tick-621 trials. The H66 input
is a hashed completed prefix from a still-running history, not its final report.

[H72's nineteen verified parts](checkpoints/2026-09-27/h72-extended-fallback/manifest.json)
reconstruct thirteen originals. All 1,893 original coupled queries replay exactly;
extended search finds physically admissible corrections at the three saved
states. Of 3,704 copied queries, all restore the native pre-step bodies exactly;
new search outputs are not claimed to have independent recorded references.
Three selected outputs repeat exactly. See the [precise comparison scope](standing-h72-contract.md).

The inactive compiler toolchain was [relocated and verified](checkpoints/2026-09-27/inactive-rustup-relocation-verification.json)
to preserve archive capacity: 180 files and 807,363,104 bytes retain their hashes.
The [receipt](checkpoints/2026-09-27/inactive-rustup-relocation.json) records its
new temporary path, which future native builds must use as `RUSTUP_HOME`.
No compiler was active. The active f32 inspector remains at its original path
with the same digest, and running physical histories retain their source.

Two completed older native build targets were [relocated with all hashes
verified](checkpoints/2026-09-27/older-native-targets-relocation-verification.json)
to preserve capacity for the complete long histories. The [receipt](checkpoints/2026-09-27/older-native-targets-relocation.json)
records 1,376 files and 786,773,955 bytes at their new local-only temporary paths.
Reusing those old build outputs requires the recorded new target directory.
The active f32 native inspector stays at its original path, with its
[digest verified unchanged](checkpoints/2026-09-27/older-native-targets-active-f32-verification.json).

Large complete logs now use `evidence/archive-preview-evidence.mjs`: its format-2
manifest records ordered 16 MiB raw-byte chunks, individual compressed and raw
digests, contiguous offsets and whole-original digests. Verify with
`node evidence/archive-preview-evidence.mjs verify <manifest-path>`. Reconstruct
by concatenating the gunzipped raw bytes in manifest order; chunks can split a
JSON row or UTF-8 character. The archive embeds the exact archiver source.
[Six validation checks](checkpoints/2026-09-27/preview-chunk-archive-validation/manifest.json)
cover byte-exact reconstruction across a UTF-8 boundary, empty files, legacy
per-part verification, corrupt-part rejection and wrong-original-hash rejection.
Artificial archive fixtures are separate from physical evidence.

Completed H9/H10 local raw data were [relocated with all hashes verified](checkpoints/2026-09-27/h9-h10-local-relocation-verification.json):
15 files and 679,092,404 bytes are preserved at the temporary-directory paths in
the [receipt](checkpoints/2026-09-27/h9-h10-local-relocation.json). Compressed
archives are unchanged; local-only data still require separate preservation
before a checkout can be considered self-contained.

The completed H13/H14/H16/H17/H18/H19 raw directories were also [relocated and
verified](checkpoints/2026-09-27/h13-h19-local-relocation-verification.json).
All 44 files and 2,078,090,101 bytes retain their original hashes. The
[receipt](checkpoints/2026-09-27/h13-h19-local-relocation.json) records each
old/new path; the H12–H20 compressed archive remains unchanged.

## Preserving future results

1. Use a fresh run directory and record the exact command, environment overrides, native exit code, Node/engine versions, scenario selection/count, and source/configuration hashes before and after execution. Include uncommitted changes and the lockfile; a HEAD commit alone cannot identify this working tree.
2. Store a compact summary and manifest beside the documentation. Include failed, interrupted, and partial outcomes with their scope. A filtered run or list-only discovery is not a full-suite pass.
3. Give large traces/videos a durable artifact URL and SHA-256 digest. If only a local copy exists, label it local-only and record how it can be obtained; temporary directories and ignored files do not travel with a checkout.
4. Preserve failures and originals. For compression or extraction, retain the original digest and verify the decompressed content. For recovered summaries, distinguish byte-identical originals from derived summaries.
5. Update [current status](status.md) only with the gates actually executed against the recorded source. The worker harness can record source fingerprints, but that does not replace recording all relevant configuration, browser settings, and dependencies.
