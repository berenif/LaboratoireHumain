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
soft ankle-limit excursion (the elbows also start outside their limits); this recovers and is not concealed or credited as
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

## Alternating unequal steps

The same opt-in source controller now alternates measured support feet and
requests 30 / 60 / 90 / 45 mm. A two-step bounded qualification completes
31.870 / 61.730 mm, then 4,000 quiet samples, in 15,411 uninterrupted native
integrations. Global loaded material-point slip is 9.478 / 5.099 mm; no flight,
nonfoot support, self penetration or warnings occur. Peak effort is 95%.
The final quiet joint reserve is at least 8.566 mrad. This is **not strict
all-sample joint-limit qualification**: second-lift left hip-z briefly exceeds
its soft limit by 9.907 mrad. Earlier four-step extensions stopped at the unchanged
10 mm global slip guard. Their second-lift stance slip contributed 3.896 mm,
the largest loss. The saved-data audit records each phase's global-slip increment.

Each new step takes its references from the actually landed, loaded, stable
state; there is no inter-step state installation. Per-joint ground barriers and
foot-ray clearance/landing inputs remain active in every phase. Native
self-collider separation adds a distinct, tested arm/thigh avoidance barrier.
Source-engine multi-step validation remains separate from the single-step
loopback UI and the default Rust/WASM application.

## Four unequal steps and quiet hold

The motor-reserved measured-state reference and coherent 2× phase durations
complete **31.906 / 62.058 / 91.844 / 46.803 mm** against requests
30 / 60 / 90 / 45 mm, then **5.000 s quiet**. There are 49,857 uninterrupted
800 Hz integrations over 62.32125 s. Global loaded slip is 8.611 / 5.965 mm;
there is no flight, nonfoot support, self penetration or warning. All recorded
COM/capture states remain inside measured support (minimum 2.542 mm).

All walking joint margins are positive (minimum 5.331 mrad); the original
startup ankle excursion ends by 0.065 s. This inherited entry still prevents a
strict all-startup-sample limit claim. Final quiet reserve is at least
9.910 mrad and actual effort never exceeds 95% of the original caps.

The left/right order, requested distances and quintic path shape are programmed.
Before each step, actual stance geometry, COM and joint state produce a new
contact-qualified static motor-reserve reference. Live COM/velocity, orientation,
joint limits and contact feedback determine motor commands and phase admission.
Foot rays set floor/clearance targets; all 22 joint rays remain QP inputs. Their
avoidance barriers stay dormant on this safe path (nearest nonfoot collider
65.989 mm), as do the self-avoidance barriers. Counterfactual ground, COM,
velocity, load and reserve tests separately prove their command/admission paths.
No physical root assistance or inter-step state installation is used.

Reproduce with the retained pinned source assets and rebuilt helper:
`source_multistep_probe.py --base SOURCE_SESSION --helper HELPER --output NEW_RUN
--planned-support --time-scale 2 --distances .03 .06 .09 .045`. Audit saved data
with `audit_source_multistep.py NEW_RUN/generation-001`. Default single-step
preview and the Rust/WASM app remain unchanged.
