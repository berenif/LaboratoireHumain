# H65 — expand an improving copied-world probe direction

H64's +π/3 search retains eight consecutive −1 Nm changes along the same ankle
axis and still exceeds the angular limit. Test larger steps in that already
measured improving direction, within the same original motor ceiling.

Repeat H64's three saved-state comparisons, preserving its all-body velocity
excess objective, 34 motor axes, eight iterations, ±1 Nm finite differences,
calculated-step trust rule and six backtracking fractions. When a finite probe
improves the objective, evaluate its direction from the current retained bias
at factors 2, 4, 8, 16, 32, 64 and 128, clipping every axis to its original cap.
Skip duplicate clipped biases and retain the lowest finite objective among
these trials, the original probe and the calculated line-search candidate.

All trials stay on copied worlds. Preserve every query, exact baseline,
original-retained and chosen-repeat replays, native configurations and source.
The first 70 queries (baseline, original retained, 68 finite probes) must match
the corresponding H64 queries exactly. Final physical limits and stricter
search guards remain unchanged. A local success would still require a separate
continuous evaluation and would not establish production acceptance.

## Result — all three local reserve targets reached

The final maximum linear/angular speeds are 0.068165 m/s / 0.393011 rad/s,
0.078832 m/s / 0.399320 rad/s and 0.054426 m/s / 0.385074 rad/s, respectively
at 0, +π/3 and −π/4. All three meet both the independent physical limits and
the stricter reserve targets without changing a motor ceiling. All baseline,
original-retained and chosen-repeat body states match exactly; all 210 initial
H64 comparison queries match apart from explicitly recorded signed zeros in
the calculated excess residual. No physical state field is normalized.

The first attempt failed because JSON serialization changes residual `-0` to
`0`. The original source, failed-attempt receipts and reproduced comparator
errors are retained. The corrected comparator permits only that exact zero-sign
difference in residual entries, records every occurrence and strictly compares
all other values and body states.

The [verified 29-artifact archive](checkpoints/2026-09-27/h65-expanded-probe-authority/manifest.json)
contains all 947 queries from the completed comparisons and the earlier failed
attempts. These local results support testing a coupled fallback continuously;
they do not repair or supersede the failed H63 histories.
