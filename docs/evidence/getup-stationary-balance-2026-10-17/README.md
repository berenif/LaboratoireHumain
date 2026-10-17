# Stationary-balance experiment smoke

Publication availability: this branch keeps text results, configurations, ledgers and checksums. Full compressed traces, checkpoints and video/image payloads are retained in the original local investigation history, rather than published here. See [publication record](../publication-2026-10-17.md). Historical archive paths below describe those retained local files.

This verifies the external-action environment, not a newly learned balance skill. No training or public-runtime replacement occurred.

The single run admitted 800 native integrations in total, including both checkpoint-replay branches: 1.0 simulated second and 2.530828570 physics wall seconds. It stayed within its 5-second process limit. The aggregate physical-compute allowance decreased from 46.914406751 to 44.383578181 seconds. Reset/load preserve the process counters; a separate launch still requires the caller to carry that aggregate remainder forward.

Coverage: six malformed/invalid protocol requests rejected before integration; observe/action-query immutability; 120-value observation layout; full physical+experiment checkpoint round-trip; exact next-transition and snapshot equality after a nonzero torso-force pulse; process budget preserved across load/reset; inherited teacher forecast count unchanged at 680 (no new forecast). An independent zero-budget protocol check also passed with zero native integrations and zero physics wall time.

This short plumbing run used the original expanded SLOW initialization. The new anchor inputs still have zero initial weights. Maximum anchor drift was 1.50 mm, episode quiet duration 0.975 s, and no unloaded foot reposition occurred. These values do not demonstrate perturbation recovery, reduced shuffling, natural movement or generalization.

`summary.json` contains exact run/binary/input identities and metrics. `protocol.ndjson.gz` contains all requests/responses and sampled snapshots. `seed-getup-checkpoint.bin.gz` is the existing five-second standing continuation checkpoint, archived here to make reset reproducible without repeating physical computation. `archives.json` records compressed and decoded identities.

To materialize the seed without physics:

```sh
mkdir -p outputs/getup-standing/baseline-final-5s
gzip -dc docs/evidence/getup-stationary-balance-2026-10-17/seed-getup-checkpoint.bin.gz \
  > outputs/getup-standing/baseline-final-5s/checkpoint-final.bin
```

See [the experiment interface and limits](../../experiments/getup-stationary-balance.md). Re-running physics consumes additional compute and needs an available allowance; it is not necessary to inspect these archives.

Verification: native experiment release build; three zero-integration focused Rust tests; 61 Node tests; default-feature WASM worker check; focused Rust formatting and whitespace checks all passed. The broad physical qualification and full lint suite were not run. Existing dead-code compiler warnings remain.
