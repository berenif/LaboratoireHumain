# H58 — local angular search at H57's first observed failure

H57's −π/4 tick-135 baseline predicts a forefoot at 0.121146 m/s and
1.378959 rad/s. Its first 1 Nm-limited angular proposal improves both but
remains above the actual speed guards, so it is rejected. No tested candidate
is feasible and the live step retains the failing baseline. This establishes
a search failure; it does not establish that the original motors lack authority.

Replay that exact native state, including original motor targets, gains, caps
and collision hooks. Verify the saved initial fields and all existing angular
trial body digests before testing a full calculated six-distal-axis update.
Normalize angular residuals by 0.5 rad/s and controls by their original native
torque ceilings. Scale a proposal only when it exceeds one ceiling fraction;
use the existing six backtracking fractions, sixteen-iteration budget, ±1 Nm
probes, and retain the best already-computed feasible probe. All candidate
acceptance speed guards and the 0.3 rad/s angular reserve stay unchanged.

This is an isolated copied-state search comparison. Record its standalone
invocation source and fingerprint alongside all trials. It does not modify the
ongoing H57 runs or permit a live body-state transfer. Any continuous follow-up
requires separate declaration and complete source-scoped measurement.

## Result — original caps provide local authority

All seventeen original angular queries replay byte-exact predicted body digests.
One full calculated update then reaches the angular reserve: maximum foot
angular speed falls from 1.378959 to 0.241318 rad/s, maximum body linear speed
from 0.121146 to 0.045961 m/s, and other angular speed is 0.286419 rad/s.
All original motor ceilings and candidate speed guards hold. Thirty total
queries include the seventeen replays and thirteen new trials.

The [five-artifact archive](checkpoints/2026-09-27/h58-angular-full-step/manifest.json)
retains the exact source state/row, standalone invocation source, all trial
bodies and current production/diagnostic source. This corrects the local search
limitation; continuous stability remains unverified.

The unchanged search is also evaluated at heading zero's subsequently observed
tick-329 failure. H57 uses all sixteen limited iterations there yet retains a
0.571874 rad/s foot peak. Preserve every original angular query replay and
evaluate the same full-step rule without tuning its scales, guards or budget.

All 209 original queries at tick 329 replay exactly. The same full-step rule
reaches 0.189510 rad/s maximum foot speed, 0.047207 m/s maximum body linear
speed and 0.430992 rad/s other angular speed. The [additional five-artifact
archive](checkpoints/2026-09-27/h58-angular-full-step-additional/manifest.json)
retains 250 queries including those replays. Both observed failures therefore
have a bounded local correction through the original distal actuators.

## Later +π/3 failures — not corrected locally

H57's completed +π/3 run develops 51 speed failures after tick 600. Apply the
same unchanged search to its peak-linear tick 705 and peak-angular tick 696.
Neither reaches a feasible correction: their retained maxima are respectively
0.143154 m/s at the hand and 0.608882 rad/s at the forefoot (with maximum body
linear speed 0.100879 m/s). The [eight-artifact archive](checkpoints/2026-09-27/h58-angular-full-step-late/manifest.json)
retains 54 queries, including eighteen exact baseline/original angular replays.
The two earlier successes therefore do not establish complete local authority
or continuous stability. H59 can test prevention from initialization, but cannot
assume this late state has been repaired.
