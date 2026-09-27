# H68 — local reuse of the preceding task-response matrix

H67 finds bounded previous-command seeds at three H66 tick-240 states, but all
miss the tracking/pelvis task. Test reusing the last complete fourteen-axis
finite-difference task matrix from tick 239, without taking new finite-difference
probes at tick 240. Preserve the complete original source rows and identify
exactly which prior queries form every column.

Start from H67's clipped previous-torque candidate. Apply the original
native-cap normalization, damped solve, full capped trust rule and six
backtracking fractions to the same task residual. Keep existing arm biases
fixed and vary only the fourteen leg axes. Record the first accepted update
and the result of at most eight iterations using the same cached matrix. A
candidate must remain finite, satisfy all physical speeds and the 0.4 rad/s
foot guard, and reduce the unchanged task objective to replace the current seed.

Replay every original query used to form the cached matrix against its original
tick-239 native state only if that state is available; otherwise explicitly
qualify the cache as derived from the recorded H66 trial log. Always
verify the tick-240 baseline/chosen state and H67 seed exactly. Save all new
candidate body states, commands, objective values, physical failures and source.
Count new query cost separately from the earlier cost of creating the cache.

No contact-transition behavior, cache refresh policy, continuous acceptance or
interactive frame rate is established by this local comparison. The ongoing
H66 histories and all physical acceptance criteria remain unchanged.

## Result — some useful updates, no general search replacement

All three original baselines, chosen states and H67 seeds replay exactly. The
cached matrix produces a guarded improvement from each seed, using 35 new search
queries in total. The final task scores are 0.008125, 0.025642 and 0.021283,
compared with H66's selected 0.023574, 0.001284 and 0.018282. It therefore beats
the original selected correction only at heading 0; no result reaches the full
tracking/pelvis deadband. All retained candidates preserve the speed guards.

The 47 copied queries include twelve baseline/chosen/seed/repeat checks. Cache
construction used 84 already recorded probes from the preceding step; its cost
is excluded from the new-query count. No tick-239 native binary was saved, so
those old probes are not independently replayed here. Complete source rows and
tick-240 native states remain in the [H67 archive](checkpoints/2026-09-27/h67-previous-correction/manifest.json).
These results do not yet justify a continuous cache or performance claim.

The [six-artifact result archive](checkpoints/2026-09-27/h68-cached-task-matrix/manifest.json)
is verified and preserves the cached columns, their source-query names and
input digests, all new trials and invocation source. Targeted lint passes.
