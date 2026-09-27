# H48 — online one-step distal preview

H47 establishes local six-axis authority at all three retained peaks, with
exact 25-body predictions and five subsequent bounded steps. A single pulse
does not address earlier or later peaks. Test a state-dependent preview on
every idle physics step, including settling, without consulting heading-specific
peak ticks or future trajectories.

Keep the H22 held references, quiet arms, pressure contour and world damping.
Snapshot the already configured world; predict one step with the same hooks.
When any hindfoot or forefoot is predicted above 0.4 rad/s, use the H47 six
existing distal motors and nonlinear line search to seek all four below
0.3 rad/s. These are controller trigger/reserve values; the independent
acceptance limit remains 0.5 rad/s. Preserve H47's 16-iteration limit, 1 Nm
central probes/update radius, regularization, native motor ceilings and the
0.1 m/s / 0.5 rad/s other-body guards. Start the additive bias from zero each
tick. Retain an improving feasible result if the reserve is unreachable and
report that event explicitly. Do not transfer copied physical state.

Require all 25 live post-step body fields to exactly equal the chosen preview
on every eligible step. Save every intervention's source snapshot, axes, trial
biases, metrics and chosen/live bodies. Report per-step cost and infeasibility.
No preview overrides during a grab, step, after a completed step, or outside
upright/reacting state. Published base motor telemetry must identify the
additional native override separately.

First screen 2+10 s at all three headings with unchanged standing/structural
bounds. Advance to 2+30 s only if that screen passes. Source fingerprints must
remain fixed throughout each run. This diagnostic is not production adoption,
the P1 half-second predictor, or evidence of feasible interactive runtime.

## Result — speed bounds pass, drift rejects the screen

All 2,160 live steps exactly equal the chosen 25-body predictions. After
settling, every speed bound passes at all headings, with angular maxima
0.396672/0.398197/0.392531 rad/s. Foot drift is 9.041/11.208/8.063 mm at
0/+π/3/−π/4, so +π/3 rejects the declared screen. No 2+30 s run follows.
All runs retain double support, upright state, zero steps and structural bounds.
The 13/10/11 infeasible samples occur during settling, none after it. Two
reserve misses are separately retained and do not alter acceptance thresholds.

The 29/15/12 corrections show that repeated existing actuation can suppress
the sampled speed spikes. This does not stop slow positional drift. Preview
cost averages 22.95/17.49/13.81 ms per step and peaks at 585.77/514.57/418.99 ms
on this host, excluding the rest of the app: interactive runtime is unproven.
The [121-artifact archive](checkpoints/2026-09-27/h48-distal-preview/manifest.json)
retains all intervention snapshots/trials, full responses and exact-match
receipts, source, analysis and the explicitly limited tick-1 general traces.
