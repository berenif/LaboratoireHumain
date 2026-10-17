# Bounded SAC pilot: candidate rejected

Publication availability: this branch keeps text results, configurations, ledgers and checksums. Full compressed traces, checkpoints and video/image payloads are retained in the original local investigation history, rather than published here. See [publication record](../publication-2026-10-17.md). Historical archive paths below describe those retained local files.

A real CPU SAC fine-tuning run completed, but its candidate is worse than the inherited GetUp initialization on **all four** frozen held-out comparisons. The application controller was not replaced or enabled. No retuning, second candidate or additional rollout followed this result.

## What actually ran

The inspected upstream SAC implementation is pinned byte-for-byte. Official CPU PyTorch 2.9.1+cpu and NumPy 2.5.2 were installed locally. The actor inherited the validated 120-input GetUp expansion; critics and optimizers started fresh. Collection used small 0.02-radian noise, stochastic update sampling was capped to standard deviation 0.05, and the first 200 updates trained critics only. Actor learning rate was 1e-6, batch size 128, gradients clipped to norm 1 after finite checks. This is a small full-policy fine-tuning pilot, not a replica of upstream's full curriculum.

The fixed cap was reached: 3,000 training transitions, 1,000 critic updates, 800 actor updates and 800 temperature updates. All 2,600 optimizer pre-step gradient checks were finite. The first 100 critic updates took 3.6594 seconds. New anchor-column weight norm rose from zero to 0.02145. Export/reload was exact; native exported-actor inference differed from PyTorch by at most 2.20e-6 on the checked observation.

Training used diagonal horizontal pulses, 5–15 N for 0.2 seconds. Four cardinal held-out pulses, 20 N for 0.2 seconds, were frozen before baseline collection. Initial physical checkpoint, pulse timing and deterministic actor mode were paired. Baseline outcomes did not alter the training settings. All four outcomes are retained, including failures.

## Paired result

| Pulse | Baseline maximum drift | Candidate maximum drift | Candidate stopping time | Exact trigger |
| --- | ---: | ---: | ---: | --- |
| +X | 2.72 cm | 51.35 cm | 1.56875 s | Head height below 1 m |
| −X | 1.68 cm | 41.12 cm | 2.41250 s | Head height below 1 m |
| +Z | 1.85 cm | 60.03 cm | 2.03500 s | Anchor drift above 0.6 m |
| −Z | 2.92 cm | 60.01 cm | 2.33125 s | Anchor drift above 0.6 m |

All baseline cases completed 4 seconds. All candidate cases ended with zero continuous qualified quiet time. These are four task failures; only two crossed the head-height condition, so the data do not establish four falls. No integrity, non-foot-support or low-CoM stop triggered in these candidate evaluations.

`paired-heldout.mp4` shows every predeclared comparison. A stopped candidate panel is marked and cleared; no terminal pose is repeated as evidence of continued balance. Rendering consumed zero physics integrations. `acceptance.json` is the explicit rejection decision; `summary.json`'s `status: completed` means the experiment finished, not that the policy passed.

## Budgets and preservation

The new allowance was 360 seconds of aggregate native-physics wall time, separate from the earlier smoke allowance, under a 30-minute total compute deadline ending 2026-10-17 10:44:20 UTC. One native process handled baseline, training and candidate evaluation. It consumed **253.935439046 seconds and 79,313 native integrations**; 106.064560954 seconds remained unused. The run finished at 10:26:35.924501 UTC. Every physical integration, including partial terminal decisions, is charged. The audited trace contains 3,976 step commands; reset/read/checkpoint operations integrate nothing. There were no uncertain outstanding requests, retry runs or hidden teacher forecasts.

The learner archive retained locally contains model/target/optimizer/replay/Python–NumPy–Torch RNG state and a synchronized physical experiment checkpoint. The full learner is intentionally outside Git; identities are in `local-resume-artifacts.json`. A full training resume was **not executed or tested**, and the driver has no automatic resume path. Its checkpoint payload was saved before evaluation: any future authorized continuation must use the **final** budget ledger, not that older payload's 232.9265-second consumption. Nothing in these files authorizes restarting or spending the unused allowance.

The candidate actor is archived under this experimental evidence directory solely for reproducibility. It is not referenced by default runtime assets. `archives.json` records compressed and decoded identities; `config.json` pins the native binary, source, trainer, seed and held-out protocol. Full RPC, training and evaluation traces are compressed alongside the results.

## Interpretation and next hypothesis

Finite gradients and changed weights verify that learning occurred. They do not verify a useful skill. Offline mean residuals on the 640 baseline evaluation observations moved by RMS 0.218 radians, with a maximum absolute change of 0.594 radians. This supports substantial policy drift as an observed symptom. Fresh-critic error, insufficient experience and inadequate preservation of the inherited behavior are plausible explanations, not proven causes.

A separate future experiment could preserve the GetUp backbone and learn a small bounded corrective residual with a conservative behavior-preservation objective. It would need its own explicit budget, frozen evaluation and acceptance criteria. No such follow-up was run here.
