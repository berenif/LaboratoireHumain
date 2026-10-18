# Source-engine phasewise forward step

The opt-in original MuJoCo 2.1.3 diagnostic now measures a forward step and the
final 4,000 consecutive quiet samples. This is one deterministic source-engine
stance, not application integration, impact qualification, or a trained policy.
The original application, source XML, and frozen actors remain unchanged.

## Measured result

The selected unchanged-friction run advances **61.657 mm**, with **39.490 mm**
minimum whole-foot capsule clearance at its peak. It acquires landing load and
holds the unchanged final quiet predicates for **5.000 s**. Cumulative loaded
material-point slip is **2.845 / 3.558 mm**. All 9,605 actual 800 Hz samples have
foot support, zero nonfoot load, no engine warnings, and zero self penetration.
Actual torques remain within 95% of the original gear caps. The final 4,000
samples stay inside joint limits. The inherited source entry has a 9.13 mrad
soft elbow-limit excursion; this recovers and is not concealed or credited as
planned actuation. The replay receipt preserves this caveat.

The controller uses joint torques only: original mass/bias/Jacobians, constrained
inverse dynamics, and measured phase transitions. It transfers weight, lifts,
advances, lands, shares load, then stands. Physical state is never overwritten
after initialization. Penalizing opposing tangential contact forces removed
the observed soft-friction creep; merely hardening stance tracking did not.

Every hinge anchor plus the root has a downward ground ray. Worldbody ground
queries exclude all mannequin geometry. Joint rays feed adjacent-collider
clearance/approach barriers in the actual QP; foot rays determine capsule
clearance and landing height. Ground distance at an anchor is explicitly not
collider clearance. Missing ground hits block commands. Nine zero-integration
tests include changed joint ray → changed motor command, unchanged commands for
far ground, a +170 mm floor, self exclusion, and cross-model cache rejection.

A requested friction comparison rebuilds a separate model at effective μ=1.2
versus source μ=1.0. Both pass the same ≥60 mm / 5 s gates from identical cold
q/qvel entry states. Slip changes to 2.950 / 3.503 mm, without a consistent
improvement, so the original model remains selected. No foreign full cache is
transplanted into the adapted model.

Compact receipts are in `../evidence/getup-source-step-2026-10-18/`. Full local
journals/checkpoints retain all nine branches, including failures: 54,411 actual
integrations / 68.01375 aggregate simulated seconds. Probe07 is preserved but its
59.657 mm final placement is below the strict 60 mm endpoint; the selected
62 mm-reference run closes that precision gap.

## Local commands

Retained original assets are required; they are not redistributed here.

```sh
scripts/getup-step/build_source_step_probe.sh "$SOURCE_SESSION" "$HELPER_SO"
OPENBLAS_NUM_THREADS=1 .venv/mvp/bin/python scripts/getup-step/source_step_probe.py \
  --base "$SOURCE_SESSION" --helper "$HELPER_SO" --output "$NEW_OUTPUT" \
  --seconds 16 --execute
OPENBLAS_NUM_THREADS=1 .venv/mvp/bin/python scripts/getup-step/audit_source_step.py "$NEW_OUTPUT"
```

No pose-suite robustness, learned GetUp adaptation, browser controller port,
release qualification, or default behavior change is implied.
