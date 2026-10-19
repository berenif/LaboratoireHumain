# Ten-step video correction evidence, 2026-10-19

The plan received five distinct review passes before implementation. See
`docs/experiments/getup-ten-step-video-corrections-2026-10-19.md` for their
changes, the bounded correction and the remaining visual defects.

`summary.json` separates the independent physical profile verdict from the
stricter all-frame 3 mm quality target and the partially met posture target.
The measured quality JSON files retain all nine landing intervals, all phase
lengths, phase tilt RMS/p95/max and the complete final 4,000-sample statistics.
`final-limb-quality.json` additionally reports knee flexion, measured arm/thigh
antiphase and lateral COM motion. All these readers perform zero integrations.

The two fresh ten-step runs share every common journal field exactly through
native sample 56,409, when the stand-only pelvis preference activates. The
checkpoint comparison explicitly certifies the native payload before the
unused MuJoCo scratch stack, not whole-file identity. No bytes were normalized
or overwritten. The final checkpoint, summary, receipt and journal agree on
63,612 integrations. No continuation or controller reconstruction was needed.

## Reproduction after restoring the pinned local native dependencies

From the repository root, with its established `.venv`:

```sh
export OPENBLAS_NUM_THREADS=1
.venv/bin/python scripts/getup-step/source_reviewed_walk.py \
  --output outputs/ten-step-video-corrections-20261019/cadence-only \
  --inspector "$PWD/outputs/getup-source-contact-inspection/recovered-20261019/source_contact_inspection.so" \
  --mode continuous-com-causal --count 10 --wall-cap-s 3600
.venv/bin/python scripts/getup-step/source_terminal_pelvis.py \
  --output outputs/ten-step-video-corrections-20261019/final-terminal-ten \
  --inspector "$PWD/outputs/getup-source-contact-inspection/recovered-20261019/source_contact_inspection.so" \
  --count 10 --wall-cap-s 3600
.venv/bin/python scripts/getup-step/audit_source_terminal_pelvis.py \
  outputs/ten-step-video-corrections-20261019/final-terminal-ten \
  --output outputs/ten-step-video-corrections-20261019/final-terminal-ten-audit.json
```

Use a new output directory rather than overwrite recorded evidence. Read-only
native QP witnesses require the exact pinned engine and model; their replays
are not new gait integrations. Source runtime hashes are stored at both ends
of each run and verified unchanged. The audit-clock binding repair changes
only the independent reader: accumulated native landing time is tied to its
actual row, while counter-based summary time is checked against step × DT.
The 1e-12 comparisons and the native sequence guards were not widened.

Native source XML, original GetUp weights and third-party native libraries are
not redistributed here or in derived evidence archives. Recovery uses the
pinned official sources in `restore_source_step_assets.py`; provenance is
retained in the output evidence. These results qualify the named experimental
profile on the selected modified model, not the unchanged GetUp model or the
application's default behavior. No main or deployed site changed.
