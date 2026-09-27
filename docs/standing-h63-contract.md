# H63 — continuous foot-position feedback

H61 passes all speed bounds but narrowly fails foot drift. H62 verifies that
the original leg motors can move the feet toward their initialization positions
under actual speed guards. Add its 1 s position-return term to H61's horizontal
task from initialization. Freeze each of the four foot segments' native starting
positions when the preview is installed; verify equality with the first pre-step
state. Do not choose a reference at the two-second measurement boundary.

For each horizontal component, use predicted displacement/dt plus current
position error divided by 1 s. Keep the 0.2 mm/s deadband on this tracking
residual and report actual pose rates separately. Preserve the bounded pelvis
residual, angular and arm stages, all search iteration/probe/trust rules,
native actuation limits, body/foot speed guards, solver, timestep, collisions,
anatomical ranges and continuous ownership. No state transfer or new actuator.

Every chosen 25-body prediction must match the live step. Preserve complete
trial metrics and hashes, native checkpoint/failure snapshots, support, state,
steps, maximum excursions, vertical ranges and runtime. A task-target miss
remains a miss; it never relaxes the independent drift or speed criterion.

Require all three original headings to pass the unchanged 2+10 s screen before
any 2+30 s evaluation. The local H62 return-direction result does not establish
closed-loop stability, complete convergence or production readiness.

## Launch checkpoint

All three 720-step histories ran under the source freeze recorded in
the [launch checkpoint](checkpoints/2026-09-27/h63-launch-checkpoint.json).
A separate thirty-step H61-mode replay reproduces its native snapshot hashes,
chosen motor biases, copied-query counts, 25-body output hashes and full physical
responses exactly. All 67 production/package files still match H51. The five
diagnostic mathematical tests are reused with their helper/test hashes verified
unchanged. These checks do not establish an H63 standing result.

## Result — rejected at all headings

All three short screens fail speeds. Heading 0 has five failing observation
samples, with peaks of 0.142754 m/s and 1.450523 rad/s. At +π/3 the first
failure is tick 269; double support is lost at 465 and upright state at 466,
followed by a fall. At −π/4 five samples fail, with peaks of 0.126118 m/s and
1.204794 rad/s. No long evaluation follows.

Endpoint foot/pelvis drift is 6.176/8.056 mm at heading 0, 124.511/810.193 mm
at +π/3 and 4.484/10.470 mm at −π/4. The two upright endings do not cancel
their earlier speed failures. The preview stops when its original state guard
disallows correction: +π/3 has 466 predicted steps and 254 skipped steps.
Across all headings, all 1,906 chosen predictions match their live 25-body
states exactly, with 417,945 copied queries. Native checks cover 612 motor-axis
configurations, with three explicit JSON signed-zero differences.

The streaming verifier independently recomputes velocities for every retained
25-body row and all response maxima. It also reproduces endpoint drift and
vertical ranges for the two complete preview histories. The fallen history's
post-preview positions are not in the case log, so its final drift remains
the original harness measurement, explicitly outside that independent check.
Average preview cost is 1.52–1.72 seconds per corrected/previewed physical step;
this remains diagnostic, not an interactive controller.

The [verified complete archive](checkpoints/2026-09-27/h63-foot-position-preview/manifest.json)
contains 243 parts reconstructing 155 original files, including every failed
trajectory, full trial logs and the standalone verifier source.
