# H9 — feasible projection of prior patch references

All hypotheses below are rejected; no production repair is accepted. The
[verified archive](checkpoints/2026-09-26/h9-h11-evaluation/manifest.json) preserves
22 files: full reports, static/local evaluations, source snapshots, and trace
windows around the selected event and each run's linear/angular peaks. Its
extraction entries record the full original trace digests; those large full
traces remain local-only under the named ignored `evidence/` directories.
Verify the archive with `node scripts/archive-standing-evidence.mjs verify
docs/checkpoints/2026-09-26/h9-h11-evaluation/manifest.json`. The staged evaluator
accepts `--hypothesis=h9` or `--hypothesis=h11`; default behavior remains H8.

H8 is rejected. Its exact patch-share constraints become infeasible 13–62 times
per startup run even while production total pressure remains feasible. Returning
immediately to the production hull allocation discards continuity at those events.
H9 tests this specific hard-constraint failure without changing production,
measurements, gains, caps, timing, or acceptance thresholds.

Use the previous commanded patch resultant as an objective, projected onto each
patch's current measured points. Then find the nearest nonnegative point weights
subject to the current total force and horizontal pressure moment. Patch shares
may change when needed for feasibility; no old contact points or measured load
filter enters the plan. Use equal patch weighting divided among current points.
Report every initialization, support-change, rank, or iteration fallback.

Before any runtime test, the regenerated +π/3 175–176 pair must satisfy H8's
≤20 N maximum patch jump, normalized force/moment residual ≤1e-8, and
nonnegative weights. Next require ≥20% forefoot-response reduction with identical
history through tick 175 and no delayed spike in the next five ticks. Only then
run 2+10 s at all three headings, alone and with the independently identified
upper-limb damping correction. Require all stationary/structural limits before
any production adoption or official 2+30 s run. A failure rejects this hypothesis;
it does not authorize a gain or threshold sweep.

H9 result: rejected. The selected static jump is 3.17426 N and local forefoot
response improves 95.94%, with the next five ticks ≤0.16759 rad/s. All six
startup screens fail. With arm damping, linear peaks are 0.07574/0.04420/0.12938
m/s and angular peaks 0.86576/0.56854/2.46316 rad/s. At −π/4 tick 415, the large
forefoot spike occurs with a successful projection and only small load changes:
left forefoot 94.61→92.35 N. Therefore the remaining excitation cannot be
attributed solely to an allocator fallback. Production remains unchanged.

## H10: distinguish reference creep from the remaining excitation

The controller rebases planted foot posture and neutral support references to
each measured slip. H4 tested holding those references without the corrected
load continuity. H10 compares the unchanged H9-plus-damping startup control with
that same intervention plus the already defined idle-reference hold. This is a
factorial test of three identified mechanisms, not a gain sweep. Contact points,
eligibility, and readiness remain measured. Preserve the original 2+10 s,
three-heading stationary and structural bounds; reject the combined hypothesis
if any bound fails. No production hold/release policy follows without a pass.

H10 result: rejected at all headings. Combined angular peaks remain
0.71708/0.63171/1.00769 rad/s and foot endpoint drift
0.01943/0.01248/0.03024 m. No production changes follow.

## H11: test whether commanded forefoot support is necessary

The remaining H9 spike has continuous requested loads and pressure feasibility,
so continuity alone is insufficient. The allocator nevertheless commands about
90 N through a 0.25 kg forefoot when the total pressure can be represented by
the two measured hindfoot patches. That choice requires an active distal torque
despite the forefoot's neutral posture target and intermittent contact geometry.

The next controlled diagnostic preferentially represents the *same* requested
pressure on qualified, current hindfoot points, using the existing preferred-
segment allocator. Use all current qualified patches unchanged when hindfeet
cannot represent that pressure. Retain zero-load entries for eligible forefeet
and report every fallback. This tests unnecessary distal support torque, not
contact removal, new support, a changed force budget, or weaker foot actuation.

Require ≤20 N static jump when the same mapping is applied to both recorded
ticks, exact total force/horizontal moment and nonnegative shares. Then compare
one-tick application at the original tick 176 against the unchanged control;
require ≥20% forefoot-response reduction without postponement over five ticks.
Only then run the existing three-heading 2+10 s startup screens with and without
upper-limb damping. No controller parameter or acceptance threshold changes.

H11 result: rejected at the static stage. The hindfoot-only pressure mapping
still has a 135.26830 N maximum patch-load jump on the recorded pair, above
20 N. Its force/moment constraints pass, but no runtime intervention or startup
screen is authorized by this failed prediction.
