# H49 — previewed horizontal foot-motion correction

H48 bounds all speeds but fails one short endpoint-drift gate. Its objective
uses post-step angular velocity, whereas the solver's integrated pose can
include motion absent from the published post-step velocity. Use the exact
one-step predictor to minimize actual horizontal motion of all four foot
segment centers through the same six existing distal motors.

Preserve H48's angular stage and all physical settings. After that stage,
when measured floor contacts are load-bearing on both sides, compute eight
residuals: predicted minus current X/Z center position divided by the fixed
timestep. Trigger the extra stage if any center's horizontal rate exceeds
0.0002 m/s. This controller deadband is not an acceptance threshold and does
not depend on the test clock, settle anchor or orientation. Use up to four
additional finite-difference/line-search iterations with H47's 1 Nm radius,
regularization and native ceilings. Accept a positional improvement only
when all other-body speeds pass and all four foot angular speeds are at most
0.4 rad/s. Stop when the deadband is reached; explicitly retain residual
motion when it is not. No position or velocity is assigned to a live body.

Save every triggered case and exact 25-body match. Report angular versus
horizontal triggers, residual rates and computation cost separately. Screen
2+10 s at the three headings; require every original standing/structural
bound before a 2+30 s evaluation. The source and test contract stay fixed
during a run. A failed stage or screen must not be relabelled as acceptance.

## Short-screen result

All three 2+10 s screens pass their declared physical bounds. Foot endpoint
drift is 8.785/9.187/7.540 mm, pelvis drift 6.321/6.298/2.656 mm, angular
maxima 0.358384/0.390751/0.337831 rad/s and linear maxima
0.035769/0.033977/0.034082 m/s at 0/+π/3/−π/4. The report also preserves
maximum excursions and every segment's vertical range. All runs remain
upright in double support, with zero steps and passing structural bounds.

Every one of the 2,160 live steps matches all 25 predicted bodies exactly.
There are 115,313 copied steps, 2,127 applied corrections and no speed-bound
infeasibility after settling. The horizontal deadband remains unmet on
2,148 samples: reducing motion is not the same as eliminating it. The
1 Nm update radius truncates some much larger calculated corrections; the
retained convergence analysis is a linear estimate, not an applied test.

Preview cost averages 225–258 ms per step and reaches 938 ms on this host.
This is an offline physical feasibility experiment, not an interactive
controller candidate. Two solver tests and diagnostic lint pass; all 67
production/package files remain byte-identical to H44. The
[4,314-artifact archive](checkpoints/2026-09-27/h49-horizontal-preview-screen/manifest.json)
contains every triggered snapshot/trial, full responses, analysis, validation,
source audit and source snapshots. Its complete digest verification passes.

## Long evaluation — rejected for accumulated foot drift

The unchanged 2+30 s runs fail foot endpoint drift at every heading:
26.035/29.916/23.336 mm at 0/+π/3/−π/4. Maximum foot excursions are
26.101/29.950/23.354 mm; endpoint measurements do not conceal a later return.
Pelvis drift remains 7.244/7.869/9.484 mm. All speed, support, state and
structural checks pass, with no steps, but this does not override the 10 mm
foot-drift limit. H49 is rejected as a standing repair.

Every first-720-step physical response and native-body digest matches the
corresponding short screen exactly. All 5,760 live steps match their chosen
25-body previews. There are 309,864 copied steps and 5,720 live corrections.
No post-settle speed-bound infeasibility occurs. The complete reports retain
vertical ranges, excursions, residual motion and calculation costs. The
long runs execute concurrently, so their timings are not directly comparable
to the sequential short screens.

The [11,513-artifact long archive](checkpoints/2026-09-27/h49-horizontal-preview-long/manifest.json)
contains all three runs, snapshots/trials, full responses, aggregate analysis
and matching source. Its complete digest verification passes. To make room,
the short-screen raw files were relocated without content changes; all
[4,312 raw digests verify](checkpoints/2026-09-27/h49-local-relocation-verification.json).
The [relocation ledger](checkpoints/2026-09-27/h49-local-relocations.json)
also identifies an interrupted, incomplete archive that is not result
evidence. Both completed archives above remain in the repository.
