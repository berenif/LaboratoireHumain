# Implementation prompt: coordinated standing stabilization

Use the text below as the implementation request. The accompanying
[plan](standing-stabilization-plan.md) is a proposal; no implementation or new
physics result is claimed by saving this prompt.

---

Implement the next standing-stabilization investigation in this repository.
Aim for a physically reachable balanced stance, one controller responsible for
the complete standing objective, bounded runtime, and one reproducible
acceptance operation. Do the implementation and required verification; do not
stop at another recommendation or describe finite evidence as permanent
stability. Work with one agent and preserve the selected model and reasoning
effort. Respect existing local changes and retain failed evidence.

Read `docs/status.md`, `TODO.md`, `docs/standing-stabilization-plan.md`,
`docs/physics-acceptance.md`, the frozen measurement section of
`docs/physics-standing-contract-2026-09-26.md`, and the H42/H66/H71/H72/H73
contracts before selecting the experiment. Inspect the production path through
`src/character/EmbodiedCharacter.ts`, `src/character/BalanceController.ts` and
native motors, plus the diagnostic helpers
`scripts/standing-distal-preview.mjs` and `scripts/standing-coupled-speed.mjs`.
Reuse verified evidence only while its source and assumptions still match.

Keep the diagnosis qualified. H71 confirms a selection defect in saved states;
H72 repairs three isolated searches. The diagnostic final speed search can
discard earlier position improvements, but this does not prove the cause of
every failure. H66's 1.72–1.88 s is average preview time under concurrent
diagnostic runs. H73 is a pending comparison candidate with memory failures,
not accepted standing or an established serial result.

Preserve the 1/60 s timestep, solver settings, actuator ceilings, anatomical
degrees of freedom and limits, collision rules, fall guards, original startup,
and continuous ownership of the same 25 dynamic Rapier bodies. Reach the stance
through physical actuation. Do not teleport, clear velocities, reset a history,
replace startup with a balanced snapshot, lengthen official settling, or weaken
acceptance thresholds. Keep later transfer, stepping and recovery gates open.

Before tuning, write a versioned experiment manifest with the numeric values
and rationales required by the plan: finite investigation budgets, reference
and motor-headroom tolerances, local neighborhood/model-error bounds, return
deadlines, prediction horizon if used, sustained duration and repetitions,
held-out headings and physical disturbance inputs, target hardware/runtime,
controller deadline and solve-work limit, total update budget and allowed
deadline misses, memory ceiling and allowed growth. Choose ordinary experiment
details from the available evidence and record them. Missing values block the
corresponding evaluation; do not silently tune thresholds after seeing results.

First conduct the bounded feasibility investigation in actual Rapier. Find a
maintainable reference with actuator headroom, demonstrate local feedback
return from the declared deviations, and physical entry from original startup
within the unchanged settling window. If it fails, preserve the first failure
and investigate the implicated mechanism, including H42's constraint
representation concern. Do not proceed into an unbounded search or assume MPC
will repair an unestablished operating point. Exhausting the declared budget
requires an explicit failed/incomplete result and a concrete next diagnostic.

Coordinate foot position, pelvis posture, whole-body motion and motor effort
under one complete objective. Keep stance references fixed during quiet
standing. Mandatory physical constraints and official acceptance are separate
from preferred quietness reserves. Never reject a command satisfying all
mandatory constraints solely because it misses an optional reserve, then
return a violating command.

Evaluate a compact constrained predictive controller only after establishing
the reference and local feedback baseline. If used, predict multiple steps,
define a terminal region where the backup feedback can continue satisfying
constraints, and validate the model and its error bounds against Rapier in the
declared contact/state/input envelope. Recursive feasibility is not itself a
stability proof. Use explicit theoretical conditions if claiming a guarantee;
otherwise report an empirically validated envelope. Move expensive response
identification and snapshot exploration offline; bound runtime solve work and
check model validity each update, including contact changes.

Implement explicit behavior for timeout, infeasibility and model invalidation.
Use a backup only within its validated region and with current-state command
validation that fits the deadline. Recheck any reused command. Outside that
region, record failure and follow a bounded transition compatible with existing
fall/recovery behavior and physical ownership. Logging failure alone is not a
fallback policy. Deliberately exercise each failure path.

Put the candidate in an explicit shared module used by the harness and
application, with observable configuration and verified update-order parity.
Provide one documented acceptance command that runs physical simulations
serially and executes these dependent stages:

1. Saved-failure regressions and injected solver/backup failure cases, retaining
   original H66/H69 inputs and outputs on their matching source.
2. Three 2+10 s screens from original startup at 0, +π/3 and −π/4.
3. Only after all screens pass, unchanged 2+30 s tests at those headings: pelvis
   drift ≤0.03 m; every hindfoot/forefoot drift ≤0.01 m; all-segment peak speeds
   ≤0.1 m/s and ≤0.5 rad/s; no added steps; both feet planted; accepted
   upright/reacting state. Report maximum excursions and vertical ranges.
4. Applicable structural, collision, actuation, ownership and integration
   regressions, with invariant monitoring also active in earlier stages.
5. The predeclared sustained-standing, held-out disturbance, latency/deadline
   and memory gates, including the application path before claiming interactive
   performance. Disturbance tests remain distinct from undisturbed acceptance.

Preserve H73 as a separate pending comparison candidate. If evaluating it,
retain its original sources and partial outputs, recheck disabled compatibility
and use its serial restart procedure with the documented 384 MiB Node
old-space setting. Do not treat that limit as total process/WASM memory, claim
its cause is diagnosed, or make H73 success a prerequisite for this candidate.

Automatically save the first failure and everything needed to replay it:
native pre-step snapshot/capture phase, controller/reference/model/cache state
or reconstructing history, motor configuration, contacts/hooks and external
inputs, scenario/seed/tick, source/runtime/configuration fingerprints, commands,
outputs and logs. Use bounded in-memory diagnostics and verify saved artifacts.
Report pass/fail/incomplete per stage and return non-success if required work
fails or remains incomplete. A relevant candidate change invalidates dependent
results; preserve earlier attempts instead of overwriting them.

Deliver the shared implementation, one acceptance command, the frozen
manifests, replayable evidence, a documented operating envelope and measured
runtime/memory limits. Update `docs/status.md`, `TODO.md` and relevant guides
with the exact results and remaining gates. Report meaningful verification and
limitations concisely. Close no acceptance item without its required evidence.
