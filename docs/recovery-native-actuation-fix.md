# Recovery floor-launch correction

2026-09-27. Scope: the unwanted launch after the protocol strike has knocked the subject onto the floor. **This fixes the reproduced launch, not successful get-up or quiet standing.**

## Reproduction and mechanism

The old runtime applied recovery torque impulses to individual bodies before Rapier integrated their joints and contacts. Its coupled motor model limited torque, but a frame-sized impulse could still give a light segment thousands of radians per second before those constraints were solved.

The real protocol strike occurs at 0.3 s. At tick 182 the subject is lying on the floor and begins a second recovery attempt. Three runs share the exact 181-tick body-state prefix (`6c98e14380ef7ca666b096d79be6cb70c58791eec21097bfefcd1f149004b584`). Interventions begin at tick 182; all runs end at tick 300. The fourth comparison uses the same prefix.

| Motor application after tick 181 | Peak pelvis height | Peak segment linear speed | Peak pre-integration angular speed |
| --- | ---: | ---: | ---: |
| Original impulses | 4.0023 m | 114.69 m/s | 4,291.76 rad/s |
| Drive disabled, diagnostic only | 0.1081 m | 1.25 m/s | 16.43 rad/s |
| Same torques distributed over substeps | 2.4022 m | 20.61 m/s | 98.90 rad/s |
| Native constrained motors, active recovery | 0.1301 m | 7.48 m/s | 34.73 rad/s |

Distributing explicit torques alone was rejected: it still launches the body. The native comparison configures feedback inside the solver so damping responds during integration instead of receiving an already-applied velocity kick. It retains active recovery rather than disabling the behavior.

Evidence is retained in `evidence/recovery-force-integration-20260927/` and `evidence/recovery-native-integration-20260927/`. Each contract records source fingerprints; the JSON files contain per-tick measurements. These experiments precede the runtime patch and must not be interpreted as unchanged results when rerunning the probe on later source.

## Runtime change

- `EmbodiedCharacter` injects the native backend at construction and reset.
- `DynamicRecovery` sends its existing active commands through that backend. Settling sends passive-only commands for every joint, replacing the previous active commands.
- `NativeJointMotors` combines active feedback with the shared passive damping/soft-limit law under the existing axis torque ceilings. Its default standing path leaves the passive option off.
- All 25 bodies, contacts, joint limits, solver settings and state-transition guards remain in place. No transform or velocity reset, root lift, frozen body, or reduced strike is used.

Standalone motor-model probes retain the explicit impulse implementation. Production recovery uses the injected native backend, including during protection and settling.

## Validation and limits

The fresh runtime replay starts from the original room startup, applies one real strike, and runs for 10 simulated seconds. The regression checks every tick, verifies no immediate change in position or velocity during recovery command generation, retains all dynamic body objects, and requires active recovery for more than 60 ticks. It reaches floor support at tick 54, executes 500 active recovery ticks, and has maximum post-landing pelvis height **0.122161 m** and maximum segment linear speed **7.972948 m/s**. It reports **zero completed recoveries**: roll-to-brace planning still stalls on clearance/support conditions. Its deliberately broad launch guards are additional regressions, not replacements for the stricter physical acceptance criteria.

Commands and retained results:

```powershell
$env:PROTOCOL_PROBE_SECONDS='10'
node --max-old-space-size=384 scripts/probe-protocol-one-strike.mjs
node --test --test-concurrency=1 tests/recovery-native-actuation.test.mjs tests/protocol-striker.test.mjs tests/recovery-controller-support.test.mjs tests/recovery-motors.test.mjs tests/recovery-merge-regressions.test.mjs tests/joint-motors.test.mjs
npm run typecheck
```

The launch regression, protocol striker, controller support, coupled motor and joint-motor checks pass. The first run also caught a signed-zero assertion in the new passive test; accepting either mathematical zero fixes that assertion, and the targeted passive/lifecycle run passes all six checks. The existing near-extended arm endpoint regression in `recovery-merge-regressions.test.mjs` fails its reach-error assertion at line 28. That test and its geometry implementation are unchanged by this patch. The complete selected suite is therefore **not green**. Typecheck and targeted lint pass, with the existing unused `RIGHT` warning in `EmbodiedCharacter.ts`.

Logs: `evidence/strike-floor-baseline-20260927.jsonl`, `evidence/strike-floor-native-20260927.jsonl`, `evidence/recovery-native-tests-20260927.tap`, and `evidence/recovery-native-targeted-20260927.tap`.

The preliminary standing solver-count trials also failed; no solver-count adjustment was retained. Quiet standing, a reliable roll-to-brace transition, complete get-up, longer repeated recovery and browser visual acceptance remain unverified or unfinished.
