# b0d1e83 recovery correction: partial, not a release

Date: 2026-09-19. Repository: berenif/LaboratoireHumain.
Base: b0d1e83f27c0fa6a70b26aa8c1ae7fb112d03185.
Source correction: 7a6ab46fc9caf60fca375e960729cd6f680b90da.
Branch: fix/b0d1e83-physical-recovery. Main is not changed by this work.

## Corrections retained

1. **Runtime recovery braces:** observed and planned hand supports now share a conservative lever limit. Wrist orientation and floor height converge together with a bounded 128-iteration maximum and an early convergence stop. This rejects an almost straight, overextended arm instead of treating any reachable hand contact as a useful brace.
2. **Runtime fall classification:** an active step no longer unconditionally suppresses already-measured unrecoverability. The existing landing-footprint conditions still protect a viable step. Two new tests cover both cases without synthetic grab forces or counting an attempted step as completed.
3. **Recovery fixtures and independent QA:** crouch and half-kneel seeds now use the merged anatomical frames. Crouch soles are level, both mirrored half-kneels have real leading-foot/trailing-shin floor support, and out-of-profile fixture requests throw instead of silently clipping. The independent mass oracle reads Rapier body mass, world COM and velocity, not mesh origins. Its two new regressions include deliberately corrupt snapshot metadata.

Fixture repairs correct invalid test inputs; they are not evidence that runtime standing balance or get-up is solved. No failed balance-controller or motor-retuning experiments were retained. No historical fixed stream was regenerated. Existing acceptance thresholds, force/power bounds, self-collision policy and physical step counting remain in force. No new root teleport or nonphysical support was introduced.

## Local verification

Environment: Linux, Node 22.16.0, npm 10.9.2, locked dependencies installed from the repository's npm cache. GitHub CI independently uses Node 22.13.0.

| Command / scope | Result |
| --- | --- |
| npm test (includes application build) | 167/172 pass; 5 fail |
| npm run test:recovery | 79/80 pass; 1 fail |
| npm run typecheck | Pass |
| npm run lint | Pass, two pre-existing unused-import warnings in tests/leg-target-frame.test.mjs |
| npm run test:pages | Pass: static export and 12 local asset references verified |
| Selected physics contracts below | 6/6 pass |
| timeout 180s npm run test:physics | Exit 124 after four initial scenario passes; full harness NOT verified |
| Local browser review | NOT verified: page navigation returned net::ERR_BLOCKED_BY_ADMINISTRATOR |

The merged checkpoint recorded 156/168 unit passes and 72/80 recovery passes. Seven existing unit failures are removed; four additional regression tests pass. A full green release is not claimed. Browser policies were not bypassed. No independent motion review or deployment is claimed.

Selected physics command (a subset, not the full acceptance suite):

```sh
PHYSICS_SCENARIO_PATTERN='^(expanded-anatomy-shared-geometry-and-ownership|solver-structural-joint-limits|frozen-recovery-threshold-contract|finite-floor-penetration-semantics|seven-region-picking|floorless-center-of-mass-free-fall)$' npm run test:physics
```

The floorless case uses an independent reduction of actual rigid-body COMs. This fixes the previous oracle's error when segment origins and mass centers differ; it does not modify the production forces or relax the free-fall bounds.

## Remaining release blockers

| Test | Observed failure |
| --- | --- |
| slow pulls stay connected and stepping remains available after release and reversal | Unexpected fall; physical support transfer is still incomplete |
| a planted reversal preserves ownership without a transient unsupported fall | Unexpected fall during reversal |
| fall and recovery motion reject body input and require a fresh press | Strong pull does not reach the required physical fall/lockout in the test window |
| a five-tick strong hand pull still produces a connected Rapier-owned physical fall | Strong-pull physical-fall requirement still fails |
| native fixed streams start from the current 25-segment continuous dynamic assembly | Historical initial leftShoulderGirdle rotation mismatch; capture left unchanged |

The first two failures share a stalled physical load transfer: step intent can remain active while the intended moving foot remains loaded and no real step completes. The next correction must couple the retained sole, reachable landing and measured mass transfer, then pass the unchanged 510-tick pull/reversal scenarios. Do not substitute a timeout-completed step or fake unload/contact events.

The one-time branch-only publication workflow was removed after verifying all eight source/test blob hashes. The remaining verification workflow has read-only repository permissions and records each failing or timed-out suite independently. This branch stays a draft until the remaining gates pass.
