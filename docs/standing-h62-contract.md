# H62 — position feedback for accumulated foot drift

H61 bounds all observed speeds but its +π/3 foot drift is 10.095 mm against the
unchanged 10 mm limit. An instantaneous zero-displacement-rate objective can
accumulate residual error. Test adding a return term toward each foot segment's
native horizontal position at initialization, rather than changing any drift
measurement or choosing a new reference at the observation boundary.

At each H61 tick-720 snapshot, compare the original rate task with a position
task from the identical retained H61 bias. For each of the four foot centres,
the new residual is predicted horizontal displacement/dt plus current horizontal
position error divided by a predeclared 1 s return time. The initialization
reference comes from that run's first native pre-step state. Preserve the
bounded pelvis residual, physical normalization, fourteen leg axes, existing
arm biases, native caps, ±1 Nm probes, eight iterations, full guarded updates,
six backtracking fractions and feasible-probe retention.

Keep the 0.2 mm/s search deadband on the tracking residual. Report actual pose
rates and position error separately; the physical drift and speed bounds remain
unchanged. Every baseline and retained 25-body result must replay exactly, and
every candidate must preserve the original actual speed guards and foot angular
≤0.4 rad/s. Save all trials, original binaries, initial reference states and
invocation source. One copied step cannot establish stable return dynamics or
standing acceptance; any continuous test requires its own declared evaluation.

## Result — return direction verified; exact task convergence remains limited

Both baseline and retained predictions replay exactly in every comparison.
The position task reduces the maximum initialization-reference error at all
three states: 6.604→6.518, 11.486→11.294 and 10.048→9.944 mm in one physical
step. All final candidates satisfy the original actual speed guards. Only
+π/3 reaches the complete tracking/pelvis deadband in this eight-iteration
search; the other two retain tracking residuals of 1.871/8.208 mm/s and small
pelvis-target excesses. These search misses remain recorded.

The matching rate-only continuation reaches its task deadband at 0 and +π/3,
but has no return term for accumulated position error. The [fifteen-artifact
archive](checkpoints/2026-09-27/h62-foot-position-authority/manifest.json) retains
all 1,202 queries, references, original native states and invocation source.
Targeted lint passes. A continuous trial can now test the feedback dynamics
from initialization, without assuming exact one-step task convergence.
