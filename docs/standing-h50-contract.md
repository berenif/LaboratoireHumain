# H50 — isolated convergence check for horizontal preview

H49's short screen passes, but it needs roughly 53 copied steps per live step.
Its tick-350 retained Jacobian at heading zero proposes 10–16 Nm adjustments,
which the 1 Nm numerical update radius truncates. These are additive motor
biases within unchanged native ceilings, not a proposal to increase capacity.

While the unchanged H49 long runs finish, use only their already completed
short-screen snapshots at tick 350 for each of the three headings. Reproduce
the unperturbed and retained chosen 25-body outputs exactly before any new
query. From H49's chosen bias, make at most four horizontal Gauss–Newton
iterations, with the same ±1 Nm derivative probes and 1e-8 regularization.
Try the full calculated step clamped to original motor ceilings, then
1/2, 1/4, 1/8, 1/16 and 1/32 fractions. Accept only actual nonlinear motion
improvement with all original other-body speed guards and foot angular speed
at most 0.4 rad/s. Preserve all probes and rejected candidates.

The separate invocation is saved as evidence data and executed from standard
input, without changing any source used by the active long runs. Retain its
exact code, source fingerprint, snapshot provenance and replay checks. This
single-state diagnostic can establish faster local convergence only. It
cannot change H49's ongoing result or establish online standing/runtime.

For a fair iteration-count comparison, also run four iterations from the
same retained bias with the original 1 Nm update radius. Keep every other
probe, guard, regularization and line-search value identical. Compare the
actual residuals and query counts, without interpreting concurrent wall time
as interactive performance.

## Result — fewer iterations reach a lower local residual

Both methods reproduce the unperturbed and retained 25-body outputs exactly
at every heading. Starting from the same retained bias, one full step has a
lower actual squared horizontal-motion residual than four 1 Nm-limited steps
at all three headings, while passing all stated speed guards. Both complete
four-iteration comparisons use 54 copied queries, including the two replay
checks; every full step is accepted without backtracking.

After four iterations, maximum horizontal rates are
1.104/1.898/0.854 mm/s with full steps versus
3.037/2.213/1.681 mm/s with limited steps at 0/+π/3/−π/4. Neither method
reaches the 0.2 mm/s deadband. This supports testing a more efficient online
search, but proves neither accumulated drift nor interactive runtime.

The [13-artifact archive](checkpoints/2026-09-27/h50-horizontal-convergence/manifest.json)
retains both exact executable invocations, all 324 copied queries, matched
comparison, source fingerprints and source snapshots. Its digests verify.
No source used by the concurrent H49 long runs was changed.
