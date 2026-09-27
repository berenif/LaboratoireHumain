# H59 — full calculated angular updates in continuous preview

H58 corrects two distinct H57 failures with the same full calculated angular
update. Apply that search only to H57's six-distal-axis angular stage; retain
the bounded fourteen-axis pelvis/foot task afterward. Normalize angular
residuals by 0.5 rad/s and control columns by their unchanged native caps.
Scale the initial update only if it exceeds one cap fraction, then backtrack
through 1, 1/2, 1/4, 1/8, 1/16 and 1/32. Retain the best feasible improving
probe when better than the line candidate. Keep ±1 Nm probes, sixteen angular
iterations, 0.4 rad/s trigger, 0.3 rad/s reserve and all actual speed guards.
Use H58's 1e-14 minimum numerical objective improvement; this is a search
stopping criterion and changes no physical acceptance bound.

Start at initialization with zero additive bias each tick. Keep every H57
reference, allocation, physical setting, collision, anatomical range and native
motor ceiling unchanged. No body state transfer. The independent actual 25-body
step must exactly match its chosen copy. Preserve compact full trial logs,
checkpoint/failure snapshots, all original structural measures and source.

Require the unchanged 2+10 s screen at 0, +π/3 and −π/4 before any 2+30 s
evaluation. This diagnostic must establish stability across time; the two
isolated H58 corrections alone cannot close standing or production gates.

H58 does not correct H57's later +π/3 peak states. This experiment tests whether
the improved angular search avoids reaching them from initialization; it must
retain and reject any new failures. No claim of a universal local repair is made.

The three 720-step runs have been launched with the source frozen. Targeted lint
passes. A separate 30-step heading-zero H57 replay checks compatibility of the
previous mode. These are pending executions, not successful results; their
source/launch checkpoint is [recorded separately](checkpoints/2026-09-27/h59-launch-checkpoint.json).

## Completed result — two short screens pass; one fails hand speeds

Headings 0 and +π/3 satisfy the original short drift/speed/support criteria.
Heading −π/4 has eight post-settle hand-speed failures, with maximum linear
speed 0.112828 m/s. Maximum angular speeds are 0.474392/0.484208/0.498652 rad/s;
foot drifts are 6.145/5.672/5.227 mm and pelvis drifts 4.656/6.058/5.437 mm.
All finish upright with both feet planted, zero steps and no support loss.
The three-heading gate therefore fails; no 30 s evaluation follows.

All 2,160 live steps exactly match their chosen 25-body prediction, and all
383,383 copied cases preserve original native caps. Checkpoint inspection
verifies 294 motor-axis configurations; one signed zero lost by JSON serialization
is recorded exactly as in H57. The previous H57 mode reproduces its first thirty
heading-zero states, responses and prediction hashes exactly. All 67 production/
package files remain identical to H51.

Average preview times are 1,365/1,293/1,391 ms per step in concurrent execution,
excluding evidence writes. Maximum joint separation is below 0.049 mm, self
penetration is zero and floor penetration maxima are 2.715/2.481/4.174 mm.
Full excursions, vertical ranges, native receipts, trial logs and source are in
the [H59 archive](checkpoints/2026-09-27/h59-full-angular-preview/manifest.json).
[H60](standing-h60-contract.md) establishes a local arm-motor correction for the
remaining type of speed failure; its continuous evaluation is separate.
