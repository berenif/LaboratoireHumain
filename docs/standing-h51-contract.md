# H51 — swing ankle target range within unchanged joint limits

The remaining anatomy assertion measures the full forward vector, including
pitch. Its raised swing foot requests 27.80 degrees of ankle compensation;
the compositor uses only 90% of the unchanged 20-degree anatomical limit,
leaving the foot pitched enough to fail the original 0.99 dot-product bound.
An isolated rotation calculation using the full permitted 20 degrees raises
the dot product from 0.985416 to 0.990754; full recomposition is still required.

Use the existing lift envelope to interpolate target utilization from 90%
when planted to 100% at maximum swing lift, returning to 90% at touchdown.
Do not change actual joint ranges, torque ceilings, clearance, endpoints,
attachment construction, solver settings or any fixture/assertion. This
releases a motor-target reserve only during the raised part of a committed
step; it does not expand the physical anatomical range.

First evaluate an isolated temporary source copy against the unchanged anatomy
assertions while the H49 runs use the unchanged checkout. Preserve both source
versions and the original test, changing only temporary import paths. A
passing candidate must still receive production integration, focused physical
regressions, typecheck/lint and idle-trajectory equivalence before retention.
No source used by the ongoing H49 runs may change.

## Result — retained geometric correction

All four unchanged anatomy tests pass on the isolated candidate. Across
48 sampled swing poses (both sides, three headings and eight phases), joint
anchors remain connected within 1.12e-16 m and anatomical limit errors stay
below 2.34e-16 rad. All 21 idle/start/end comparisons exactly match the
original compositor. The [5-artifact candidate archive](checkpoints/2026-09-27/h51-swing-range-candidate/manifest.json)
retains the original and candidate sources, unchanged assertions, temporary
import mapping, commands and results.

After all H49 runs and archival completed, the tested change was integrated
in `pose.ts`. The same twelve-file selection now passes **69/69 tests**;
all twelve test files are byte-identical to H44. Typecheck and lint of the
modified file pass with no output. The earlier unused `RIGHT` warning in
unchanged `EmbodiedCharacter.ts` is outside this lint selection.

All three fresh 2+30 s plain standing trajectories, initial hashes and every
recorded response exactly match H44. Their standing failures remain open.
The source audit verifies the declared swing-utilization change plus its
comments; the other 66 production/package files are byte-identical to H44.
No solver, collision, anatomical limit, actuator ceiling or acceptance
threshold changes. This repair does not establish balance, transfer,
stepping, recovery or release acceptance.

The [6-artifact production archive](checkpoints/2026-09-27/h51-swing-range-production/manifest.json)
contains the focused tests, static checks, all plain captures, exact replay
analysis, source audit and current source snapshots. Both archives verify.
