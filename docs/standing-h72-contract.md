# H72 — extend only copied search and retain physical feasibility

H71 confirms an admissible recorded fallback for H66's first long failure and
H69's first −π/4 failure. No such candidate exists among the recorded queries
at H69's first +π/3 failure, tick 621.

Replay the original coupled search at these three saved states with a maximum
of sixteen search iterations instead of eight. These are copied-world search
iterations, not native solver iterations. Require every original coupled query
to match in order, bias, complete body digest and objective before considering
the additional iterations. Preserve original native motor parameters and verify
all baseline/retained body states exactly.

Continue targeting the same 0.08 m/s and 0.4 rad/s reserve. Prefer the original
0.4 rad/s foot guard when available; if it is unavailable, retain the best
actually evaluated candidate satisfying every original 0.1 m/s and 0.5 rad/s
physical bound. If none exists, explicitly report failure. Repeat the selected
candidate and preserve its complete 25-body state and all failed trials.

No physical threshold, native setting, ownership rule or live run changes.
This is a local search-budget/selection comparison, not standing acceptance.

## Result — all three local corrections become admissible

| Saved failure | Search queries | Peak linear (m/s) | Peak angular (rad/s) | Preferred guard | Reserve |
| --- | ---: | ---: | ---: | --- | --- |
| H66 long, 0, tick 978 | 1190 | 0.079861 | 0.399496 | Pass | Pass |
| H69 short, +π/3, tick 621 | 1258 | 0.087485 | 0.326286 | Pass | Miss |
| H69 short, −π/4, tick 497 | 1247 | 0.081777 | 0.400024 | Pass | Miss |

All 1,893 original coupled queries match in order, torque bias, body digest,
objective and guards before any extension. Baselines and retained outputs also
match; each new selected state repeats exactly. The 3,704 total copied queries
include six baseline/retained checks and three chosen repeats. Native inspection
checks 102 motor axes. Targeted lint passes.

In the raw local report, `exactBodyComparisons` counts the restored pre-step
bodies checked for every query (92,600 in total). It does not mean every newly
searched output has an independent recorded reference: 1,893 original trial
outputs plus six baseline/retained outputs do, and three chosen outputs repeat.
None is a new live continuous history.

The [complete archive](checkpoints/2026-09-27/h72-extended-fallback/manifest.json)
preserves the inputs, all trials, exact helper source and the earlier-query
comparisons. The added search iterations change no native solver settings or
physical acceptance limits. Continuous behavior and runtime remain unproven.
