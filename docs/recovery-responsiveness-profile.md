# Strike/recovery performance investigation

2026-09-27. **Partial improvement: repeated rolling foot searches removed; get-up remains unsuccessful.** No recovery gain, transition duration, solver budget, contact rule or completion threshold change is retained.

## Measurement and retained changes

The existing one-strike probe starts the actual room, requests one physical strike (impact at 0.3 s), and integrates 600 steps at 60 Hz. It now records wall time, fixed-update time, p95 step time, phase costs, nested planning costs and Rapier integration cost. Nested buckets overlap and must not be added together. Startup is excluded; rendering and browser scheduling are not measured. Headless speed is not an interactive frame-rate claim.

The first 10-second replay took 146.16 wall seconds. A separate 6-second breakdown spent 27.28 seconds in `transfer` and 14.51 seconds in `world.step`. The expensive transfer path repeatedly searched hundreds of floor placements during rolling, although no foot swing had been requested.

`DynamicRecovery.transfer` now retains the captured foot plan during the `roll` transfer stage when that foot has no movement in progress. Arm preparation, bracing and later phases retain their existing revalidation/search path. A regression checks both retaining the rolling plan and resuming the search during bracing.

The first optimized 10-second run took 20.07 seconds; all 20 recorded samples were identical to the initial baseline. The final instrumented run took 26.85 seconds, including hashing every integrated pose and checking geometry every tick. Transfer planning accounted for 25.78 **milliseconds** across 498 calls; Rapier integration still took 20.88 seconds. These individual timings vary with host load/JIT state and are not a statistical benchmark. Real-time operation remains unmet.

`FixedStepLoop` now measures active wall seconds, simulated seconds and their ratio, and includes time discarded by the 250 ms frame clamp in dropped time. Pauses are excluded and Reset clears the counters. Browser tooling can read a detached stats snapshot with:

```js
window.__EMBODIED_DEMO__.timing()
```

A ratio of 1 means one simulated second per active wall second. Frame remainder and discarded catch-up time are retained in the accounting; the fixed timestep and yielding policy are unchanged.

## Rejected experiments

| Trial, 10 simulated seconds | Wall seconds | Recorded result |
| --- | ---: | --- |
| Existing solver, repeated rolling search | 146.16 | No recovery |
| Skip repeated rolling search | 20.07 | All 20 original report samples match; no recovery |
| 20 solver iterations after tick 56 | 11.57 | 14.83 mm sampled self-penetration and 0.497 rad sampled joint-limit error; rejected |
| Rolling stiffness ×0.08, damping ×0.3 | 17.01 | No brace or recovery; not retained |
| Limit requested rolling spine turn to 0.65 rad | 17.24 | No brace or recovery; reverted |

The softer motor and incremental-turn trials did not resolve the controller blockage. Shortening the 1.2-second transitions would not address this replay, which never reaches those transitions. The baseline and retained controller remain stuck in rolling with commanded floor/body clearance blockers. A successful support-producing roll and arm placement still need a controller change and physical validation.

The final every-tick scan also detects a transient 0.5894 rad joint-limit error that the old coarse reports missed (their maximum was 0.02648 rad). An original-controller rerun confirms that this violation already existed. The original and retained controllers have the **same full 600-tick segment trajectory SHA-256**, `7ee2c440cb144a606baefef64bc0ab52fc6c49e0ba162bdf9b49d0b1d68242d5`. That original rerun took 63.47 seconds versus 26.85 seconds for the retained controller, illustrating host timing variability while independently confirming the physical trajectory is unchanged on this replay. Passing the broad no-launch regression does not establish the stricter joint-limit gate or successful recovery.

## Validation and evidence

Sixty targeted tests pass: loop timing, runtime lifecycle, recovery support/controller behavior and native recovery actuation. Typecheck and targeted lint pass. The real-strike regression retains all 25 dynamic bodies, applies active constrained motors, observes no direct pre-solver velocity injection and does not launch the body. It completes zero recoveries. Full physical acceptance and browser visual acceptance have not passed in this work.

Evidence is local and gitignored under `evidence/strike-responsiveness-*.jsonl` and `evidence/strike-responsiveness-tests-final.tap`. Reports include source hashes and source-change detection. `DynamicRecovery-before-responsiveness.ts` preserves the original working-tree controller, including pre-existing uncommitted work; `DynamicRecovery-incremental-roll-trial.ts` preserves the rejected incremental-turn variant.

Reproduce the production probe:

```powershell
$env:PROTOCOL_PROBE_SECONDS='10'
node --max-old-space-size=384 scripts/probe-protocol-one-strike.mjs
node --test --test-concurrency=1 tests/fixed-step-budget.test.mjs tests/demo-runtime.test.mjs tests/recovery-controller-support.test.mjs tests/recovery-native-actuation.test.mjs
npm run typecheck
```

Optional diagnostic-only probe environment variables are `PROTOCOL_RECOVERY_SOLVER_ITERATIONS=20` (after tick 56) and `PROTOCOL_SOFT_ROLL=1`. Leave both unset for production evidence. Neither trial is enabled in the application.
