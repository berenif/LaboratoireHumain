# Offline SLOW mean-head trust-region pilot

Status: closed. The policy changes stayed inside the declared trust and action
guards, but the paired physical evaluation failed the unchanged 10 mm slip gate.
The original SLOW baseline remains selected. There is no quiet-five-second or
forward-step qualification.

## One prospectively fixed experiment

The parent-admitted stage ran from 2026-10-18 01:40:21 UTC with a fixed 01:55:21
deadline. Implementation, fixtures and review counted against the 15-minute wall
limit. All 3 physics seconds, 640 policy intervals and 12,800 inner integrations
were reserved for one deterministic frozen/candidate pair. Every offline action,
including line-search forwards and checkpoint writes, protected 300 wall seconds.

Only the already collected checkpoint-002 replay was used: 256 actual transitions,
including both finite physical failures and their zero bootstrap masks. There was
no new physical training data. These data came from the earlier latent-std 0.02
collector and have limited coverage of the restored broader original distribution.

The actor started from the complete original learned SLOW distribution. Both
hidden layers and the original state-dependent logstd rows stayed frozen. The
pinned original alpha, 0.000612579835321518, was fixed with no alpha optimizer.
Only the 21 mean-output rows and biases were trainable. The source 118 inputs,
holding target-height sentinel 1.4, power 1 and 800 Hz inner PD were unchanged.

Fresh critics received 100 critic-only updates with a target Polyak update after
each. Five subsequent attempts each used one sampled replay batch, one critic
update and one mean-head Adam proposal. A fixed Gaussian draw was reused for all
forward-only candidate scales. The accept test required a same-noise actor-loss
decrease and summed latent KL at most 0.05 on every immutable replay witness.
The historical action-drift guard and a preregistered midpoint audit retained
0.02 rad RMS / 0.05 rad maximum limits. These finite witnesses do not certify
global behavior or critic accuracy.

## Offline result

All five proposals accepted their first scale, 1.0. Runtime backtracking was not
needed; the accepted-half-scale and populated-Adam rejection paths were exercised
in throwaway pure fixtures. The total actual candidate cost was exactly 110
optimizer calls: 100 warmup critic, five proposal critic and five actor proposals.

- Maximum replay KL after proposal 5: 0.03266843
- Historical residual drift: 0.00821179 rad RMS, 0.00856634 rad maximum
- Midpoint audit: 0.00821229 rad RMS, 0.00856575 rad maximum
- Original hidden/std tensors and fixed temperature were preserved
- No optimizer proposal was selected using physical results

Every attempt retains an immutable postcritic/preactor learner/batch/RNG
checkpoint, postproposal checkpoint and batch/noise hashes. Charged calls are not
refunded by rejection or restoration. Eleven focused pure tests passed, including
same-noise loss equality to the original source Actor/critic, original-distribution
parity, a genuine accepted half-scale with full Adam moments, populated-Adam/RNG
rejection and next-proposal continuation, plus six resource-ledger fixtures.

## Actual deterministic pair

Both actors started from the same complete original 800 Hz source cache. Their
means, not stochastic samples, supplied qpos-order residual radians at 40 Hz.
Reference and residual stayed held for 20 inner samples; actual q/dq refreshed on
every sample with source holding damping 0.2 and original motor gears.

- Frozen original: first left loaded-slip failure at 2.865 s; L 10.001635 mm,
  R 9.685628 mm; 115 policy intervals and 2,292 actual integrations
- Candidate: first left loaded-slip failure at 2.7875 s; L 10.002189 mm,
  R 9.587705 mm; 112 intervals and 2,230 actual integrations

Both finite partial terminal transitions have bootstrap zero. Each stopper fired
on its first native threshold crossing. Independent reduction found no warning,
flight or nonfoot support and exactly zero motor-torque equation error. The
frozen branch's first 1,260 packets equal the earlier retained SLOW benchmark
bit-for-bit. The candidate failed 77.5 ms earlier; at the common endpoint it had
more slip on both feet, more COM drift and more tilt. No physical improvement is
supported, despite all offline trust checks passing.

Aggregate cost was 227 actual policy intervals, 4,522 integrations and
0.33643741 physics seconds. The executed optimizer/pair run took 49.44 wall
seconds; setup and review were separately included in the unchanged stage clock.
The stage closed at approximately 01:49:03 UTC with no pending request or
optimizer. Unused allowance did not release a retry or another candidate.

## Consequence

Freezing the sensitive shared trunk and using the original distribution avoided
the previous immediate action-drift veto. It did not make the fixed replay's Q
gradient physically beneficial. A future online-data or throughput proposal is a
separate experiment; this result does not justify repeating tiny offline fits or
loosening quiet-standing criteria.

Implementation is in `scripts/getup-step/slow_trust.py` and
`slow_trust_run.py`. Compact own receipts are in
`docs/evidence/getup-source-slow-trust-region-2026-10-18/`. Full checkpoints,
original/derived weights and raw physical journals remain local. No source
weights or default application policy were overwritten.
