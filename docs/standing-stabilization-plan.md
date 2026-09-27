# Coordinated standing stabilization plan

Proposed 2026-09-27 after review of the implementation and recorded experiments.
This is a work plan, not a controller implementation or a new physics result.
Quiet standing and all subsequent acceptance gates remain open. Use the
[implementation prompt](standing-stabilization-prompt.md) to start the work.

The proposed direction is one controller responsible for the complete standing
objective, built around a physically reachable balanced reference, followed by
one reproducible acceptance pipeline. Constrained predictive control is a
candidate to evaluate after local feasibility is established. The deliverable
is a documented operating envelope, bounded computation and repeatable evidence;
finite tests cannot establish permanent stability under arbitrary conditions.

## Evidence and its limits

| Observation | Supported conclusion | Limit |
| --- | --- | --- |
| [H71](standing-h71-contract.md) replays discarded commands within the original speed bounds in two saved states. | Preferred quietness margins must not exclude an otherwise acceptable correction in favor of a violating command. | The bounds checked there concern speed; local success does not establish the complete standing contract. |
| [H72](standing-h72-contract.md) finds acceptable local speed corrections at three saved failures. | More search and corrected selection can repair those isolated queries. | Continuous standing and runtime remain unproven. |
| The diagnostic [distal preview](../scripts/standing-distal-preview.mjs) corrects feet, arms and position before the final [speed objective](../scripts/standing-coupled-speed.mjs). Every copied prediction advances one physics step. | The final search can lose earlier position improvements, and next-step speed bounds do not establish future recoverability. | This is an architectural concern, not a demonstrated cause of every recorded failure. These helpers are diagnostic, not the production standing controller. |
| [H66](standing-h66-contract.md) records average preview time of 1.72–1.88 seconds per physical step under concurrent diagnostic runs. | The existing evidence does not demonstrate interactive execution at the 1/60 s timestep. | This is not an isolated production-controller benchmark. |
| [H42](standing-h42-contract.md) finds rigid impulse assemblies topple while fixed multibody assemblies remain nearly stationary in the recorded window. | Constraint representation remains a material diagnostic concern after inertia correction. | The rigid fixture does not prove the articulated controller has the same failure mechanism. |
| [H73](standing-h73-contract.md) passes local replay and disabled compatibility, but its concurrent enabled attempts exhaust memory. | H73 is a pending comparison candidate; preserve partial outputs and use its serial restart procedure. | No completed serial screen is established. A Node old-space limit does not bound total process or WASM memory or diagnose the failure. |

The diagnostic preview already captures a fixed initialization foot reference.
Retaining fixed stance references is a design requirement, not evidence that
moving references caused the current failures.

## Fixed constraints

Preserve the [physical acceptance contract](physics-acceptance.md) and the
[frozen standing measurements](physics-standing-contract-2026-09-26.md#frozen-measurement-contracts).
Keep timestep, solver settings, finite actuator ceilings, anatomical degrees of
freedom and limits, collision rules, fall guards and continuous Rapier ownership.
The same 25 dynamic bodies must reach the reference from original startup by
physical actuation. Do not teleport, clear velocities, reconstruct bodies,
substitute a saved balanced state for startup, or extend the official settling
window to obtain a pass. Offline copied-state probes remain available.

Physical invariants apply throughout startup and observation. Official quiet
standing measurements retain their declared observation window. Separate these
requirements from optional internal reserve margins; an internal preference
cannot override a mandatory constraint or silently change acceptance.

## Freeze a measurable experiment contract before tuning

Create a versioned run manifest before calibrating or tuning the candidate.
Record the source, dependency and runtime fingerprints, commands, original
startup inputs, evidence locations and exact candidate configuration. Fill in
every quantitative field below with numbers, units and a rationale. Choosing
these experimental values is part of the implementation task; the existing
records do not justify inventing a validated envelope in this document.

| Contract item | Required declaration |
| --- | --- |
| Feasibility investigation | Maximum trials, simulated steps and wall time; permitted reference variables; operating-point residual tolerances and required motor headroom. |
| Local feedback and prediction | State/control variables, contact validity conditions, prediction horizon in steps and seconds, numeric model-error limits, neighborhood bounds and return/hold deadlines. |
| Sustained standing | Observation duration longer than 30 s, repetitions and headings; retain original drift/speed/support/state limits over the declared full interval, and report excursions and vertical ranges without resetting references. |
| Held-out evaluation | Separate calibration and evaluation sets; exact headings, seeds, physical disturbance locations/directions/magnitudes/durations/times, repetitions, admissible recovery envelope and deadlines. Disturbance cases cannot be counted as passes of the undisturbed standing gate. |
| Runtime | Named CPU, OS, Node/browser/backend versions and instrumentation mode; controller deadline, deterministic solve-work limit, full physics-step budget and allowed deadline-miss count; report p50, p95, p99 and maximum costs. The full update must fit the 1/60 s cadence before claiming interactive 60 Hz physics. |
| Memory | Maximum process memory, measurement method, soak duration, sampling interval and allowed growth after warm-up; report JS heap, external allocations and WASM memory separately without double-counting them in a total. |
| Failure policy | Exact command-selection/transition rules for timeout, infeasibility, invalid model/contact state and states outside the validated backup region. |

Do not leave these fields as “small,” “long enough,” or “fast enough.” A missing
field prevents starting the corresponding evaluation. A changed bound or a
held-out case used for tuning creates a new experiment version; retain the old
manifest and failures, and declare a new held-out set before further tuning.
Supplemental gates do not relax the existing official contracts.

## Decision gates for the controller

1. **Establish a balanced reference and local feedback behavior in Rapier.**
   Use the current motors, limits, contacts and solver to find a maintainable
   reference with declared actuator headroom. Demonstrate return from the
   declared small deviations and physical entry from original startup within
   the unchanged settling window. Report contact and solver behavior as well
   as pose and speed. If the bounded investigation fails, preserve the evidence
   and investigate the implicated mechanism, including H42's representation
   concern. This rejects the investigated candidate; it does not prove standing
   impossible or authorize replacing the native constraint system without its
   own preserved physical gates.
2. **Coordinate the complete objective.**
   Optimize foot position, pelvis posture, whole-body motion and motor effort
   together under the mandatory constraints. Keep stance references fixed
   during quiet standing and serialize their state. Later stepping may change
   them only through an explicit authorized stance transition. Independent
   implementation modules are compatible with one coordinated objective.
3. **Decide whether prediction is needed.**
   Evaluate constrained multi-step prediction against the validated local
   feedback baseline. For predictive control, define a terminal region where
   the backup controller can keep satisfying the constraints, and validate
   model errors against Rapier across the declared state, input and contact
   envelope. Recursive feasibility means continued feasibility of the
   constrained problem; it does not by itself establish convergence. A
   mathematical stability claim needs its additional conditions and evidence.
   Otherwise label the result as an empirically tested operating envelope.
4. **Move discovery offline and bound runtime work.**
   Use exact native snapshots for response identification and validation.
   Runtime uses the compact validated model and a limited solve, including a
   practical deadline mechanism. Check model validity each update and when
   contacts change. Revalidate reused commands and warm starts against the
   current state. Evaluate the effective native motor command interface;
   requested effort is not measured delivered torque.

The [MIT MPC discussion](https://underactuated.mit.edu/trajopt.html#section5)
explains recursive feasibility and the separate stability argument.
[Herzog et al.](https://arxiv.org/abs/1410.7284) supports investigating coordinated
momentum control with hierarchical inverse dynamics and LQR; it does not
establish that MPC is necessary or will solve this simulation.

## Behavior when no acceptable solve is available

Distinguish solver infeasibility, deadline expiry, invalid model/contact state,
and preferred-margin misses. A preferred-margin miss alone must not discard a
command that satisfies the mandatory constraints.

On a genuine solve failure, use a backup only when the current state lies in
its validated region and its command remains admissible under current contacts
and actuator limits. Reusing the previous command requires the same check.
The backup and validation must themselves fit the declared runtime budget.
Logging “no admissible command found” is not a command-selection policy.

Outside the validated region, record failure and use an explicit bounded
transition compatible with the existing fall/recovery behavior and Rapier
ownership. Do not manufacture support or a standing pass. The acceptance runner
must record the failing tick and available evidence before stopping dependent
stages; an aborted run is failed or incomplete, never a completed pass. Exercise
timeout, infeasibility, contact invalidation and backup-region exit deliberately.

## One reproducible acceptance operation

Provide one documented command backed by an orchestrator and an explicit shared
controller module used by the application and harness. Make candidate selection
and configuration observable. Verify the application and harness use the same
update order, inputs and module; a diagnostic loader substitution alone does
not establish application integration. Production adoption requires the
applicable gates, not merely the existence of the shared module.

After the feasibility gate and frozen manifest, execute:

1. **Saved failures and failure-policy regressions.** Reproduce original inputs
   and prehistories on their matching source, then evaluate the new candidate
   separately. Include H66 heading 0 tick 978, H69 +π/3 tick 621 and H69 −π/4
   tick 497. Preserve the original outputs; a different controller need not
   reproduce the old chosen commands. Include the deliberately triggered
   solver and backup failures.
2. **Three serial short screens.** Run 2 s settling + 10 s observation from
   original startup at 0, +π/3 and −π/4. All must pass before longer evaluation.
3. **Unchanged official standing.** Run the same candidate for 2+30 s at all
   three headings: pelvis endpoint drift ≤0.03 m, each hindfoot/forefoot
   endpoint drift ≤0.01 m, all-segment peak speed ≤0.1 m/s and angular speed
   ≤0.5 rad/s, zero additional steps, both feet planted, and the accepted
   upright/reacting state. Report maximum excursions and vertical ranges.
4. **Structural and integration regressions.** Complete applicable coherence,
   joint-limit, collision, bounded-actuation, ownership and application-path
   checks. Run cheap relevant checks earlier as needed; physical invariant
   monitoring applies to every preceding simulation, not only this stage.
5. **Declared sustained, held-out and runtime gates.** Execute the frozen longer
   standing and disturbance cases, latency/deadline measurements and memory
   soak. Measure the application path on the declared target before making an
   interactive-performance claim; diagnostic and application costs stay labeled.

Keep physical simulations serial. H73 remains a separate pending comparison
candidate following its own unchanged restart/preflight contract and documented
384 MiB Node old-space setting. That setting is an operational control, not a
total-memory guarantee. H73's success is not a prerequisite for investigating
the coordinated controller, and either candidate's failure must be preserved.

Freeze the candidate across acceptance stages. A relevant implementation or
configuration change invalidates dependent results. Emit machine-readable
pass/fail/incomplete stage results and a non-success exit when any required
stage fails or remains incomplete. Missing artifacts or interrupted runs cannot
be counted as successes.

Preserve the first failure automatically with its complete replay inputs:
native pre-step snapshot and capture phase, controller/reference/model/cache
state or reconstructing history, motor configuration, contacts/hooks and
external inputs, scenario/seed/tick, source/runtime/configuration fingerprints,
commands, outputs and logs. Stream bounded diagnostics to avoid unbounded
retention in memory. Verify archive integrity and replay completeness; a Rapier
snapshot alone does not capture every controller input.

## Completion record

Deliver the shared candidate, the single acceptance command, frozen manifests,
replayable evidence, measured operating envelope, runtime/memory results and an
updated status/TODO ledger. State which gates passed, failed or remain
unexecuted. No standing checkbox closes before its complete acceptance evidence.
Weight transfer, stepping, recovery and full application/release verification
retain their subsequent gates.
