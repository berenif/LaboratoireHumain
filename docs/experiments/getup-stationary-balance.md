# GetUp-style stationary balance experiment

Status: executable native experiment and validated actor initialization. A [bounded SAC pilot](../evidence/getup-stationary-balance-pilot-2026-10-17/README.md) trained an experimental candidate, which was rejected on all four held-out comparisons. **No trained balance controller is enabled in the application.** Public/browser behavior is unchanged; the experiment requires the `stationary-balance-experiment` Cargo feature. This is a training/evaluation entry point, not evidence of natural movement.

## What is reused

The pinned [GetUp source](https://github.com/tianxintao/get_up_control/tree/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e) uses SAC, a strong-to-weak curriculum, then a slower reference-following student. Its SLOW terminal reward tracks uprightness, horizontal velocity, CoM height and joint-pose proximity; its 150-tick terminal window is 3.75 seconds. It does not reward returning to the initial horizontal location or measure loaded-foot slip. See upstream [env.py](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/env.py) and [main.py](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/main.py).

The experiment retains the 16-body, 21-hinge model, free root, Coulomb floor contact, original joint limits and motor gears, native 800 Hz integration and source SLOW PD interface. An action is 21 finite **qpos-order residuals in [-1, 1]**, held for one 40 Hz decision; PD is recomputed every native substep with source holding damping 0.2. There are no added root actuators, foot constraints, friction increases or pose/velocity writes. The normal integration, contacts and integrity checks run unchanged. The old recovery controller neither commands nor forecasts during the experiment.

Observation 120 preserves the original student layout and power scaling:

- 0..66: original physical observation
- 67..91 and 92..116: source standing target now/ahead (constant)
- 117, 118: CoM displacement from the reset location, source horizontal order Z, X, metres
- 119: power, fixed to 1, still the final input

The original student's first-layer power column moves from 117 to 119; new anchor columns are zero. Every other tensor is retained. An inference test compares both raw outputs and actions on every archived student parity fixture, including nonzero anchor inputs. This validates an **initialization only**: until fine-tuned, those zero-weight coordinates cannot affect the actor. `initializedAction` never claims to be learned stationary balance.

## Run

Build with the repository's pinned toolchain, after the normal Rust setup:

```sh
cargo +1.89.0 build --manifest-path rust/Cargo.toml --offline --locked --release \
  -p lh-sim --features stationary-balance-experiment --example getup_stationary_balance
cargo +1.89.0 test --manifest-path rust/Cargo.toml --offline --locked --release \
  -p lh-sim --features stationary-balance-experiment getup::balance::tests::
```

Launch `rust/target/release/examples/getup_stationary_balance GETUP_CHECKPOINT PHYSICS_WALL_BUDGET_S`. The input must be an already-integrated, native-standing GetUp checkpoint. Reset captures one fixed horizontal CoM anchor, without settling or replay. A JSONL request yields one JSONL response:

- `{"op":"reset"}` or `{"op":"observe"}`
- `{"op":"initializedAction"}` returns the expanded pretrained initialization's residual
- `{"op":"step","action":[...21 residuals...],"torso_force_n":[0,0]}` advances at most 20 native substeps
- `{"op":"snapshot"}` returns poses for rendering, without integration
- `{"op":"checkpoint","path":"..."}` and `{"op":"load","path":"..."}` preserve physical state and experiment memory

`torso_force_n` is an optional horizontal [world X, Z] force at torso CoM for that decision, bounded to a norm of 80 N. It is an actual force, not a velocity kick. A small pulse can be repeated by a future curriculum. Invalid actions/forces are rejected before integration. Reset/load do not reset the process's wall-time or actual native-integration counters. The initial smoke used a 46.914406751-second remaining allowance. The separately authorized pilot raised the executable maximum to a 360-second per-process limit. Neither number grants a future run permission. This is not a cross-process aggregate limiter: the caller must subtract every previous physical run and pass the actual current remainder. Reset/load preserve one process budget, but restarting the executable cannot enforce an aggregate ledger. No training allocation is implied. The loop reserves 0.25 s before a decision and checks the limit at every observed native substep with 0.10 s reserve. This is cooperative wall-time admission, not a hard real-time guarantee; all actual time is charged.

A reproducible protocol/small-physics smoke is provided:

```sh
python3 scripts/smoke-getup-stationary-balance.py \
  --binary rust/target/release/examples/getup_stationary_balance \
  --checkpoint outputs/getup-standing/baseline-final-5s/checkpoint-final.bin \
  --output outputs/getup-stationary-balance/smoke --steps 40 --physics-budget 5
```

The seed is the existing deterministic five-second standing continuation, not an arbitrary upright reset. `--steps 0 --physics-budget 0` checks the protocol without physics. Each output directory has a one-shot marker to avoid overwriting evidence. The 40-step smoke includes one 10 N, 25 ms horizontal force pulse. It checks plumbing only, not disturbance robustness or naturalness.

## Objective and evaluation

The candidate reward combines uprightness, nominal CoM height, fixed-anchor proximity and low horizontal velocity. Small bounded costs cover whole-body motion, effort, 40 Hz control changes, native solved-contact slip, unsupported flight, low-clearance scuffing and prolonged unloaded feet. Reward components sum over actual observed native samples and divide by the fixed 20-substep decision duration, including interrupted ticks. These are initial experimental weights, not calibrated proof of naturalness.

There is **no clearance or step bonus**. A completed unloaded reposition counts in evaluation only if it moves horizontally at least 2 cm, reaches 1.5 cm whole-foot clearance, lasts 25–600 ms, and the other foot remains loaded throughout. This is a kinematic/contact event, not proof the step was necessary or successful. Evaluate return to quiet stance after each disturbance separately. Flight, hovering or repeated stepping do not earn a positive bonus.

Outputs retain fixed-anchor drift, continuous native-qualified quiet duration, loaded impulse-weighted contact-slip distance, foot loads/clearance, completed reposition counts and consecutive **800 Hz commanded-torque increment RMS**. Foot-origin motion is not equated with slipping: solved contact-point velocity includes rotation/rolling. Quiet duration additionally requires the unchanged standing predicates, horizontal CoM speed at most 0.05 m/s, CoM within 0.10 m of the original anchor, and less than 1 cm foot drift within that quiet interval. Fall/non-foot support or 0.6 m anchor escape terminates the episode; 20 seconds truncates it. Current naturalness acceptance still requires held-out perturbation directions/magnitudes, recovery success and compact video review. A reward score alone cannot establish it.

## Training feasibility and next boundary

A subsequent [zero-integration recovery-step preflight](../evidence/getup-recovery-step-preflight-2026-10-17/README.md)
tests whether meaningful lift/landing references are reachable in the original
body before further learning. Its bounded contact-gated hybrid probe is not a
learned skill, is not enabled by default, and has not yet executed a physical
rollout. The optional `--upright-entry` admits the recorded pre-quiet 10 s state
without changing any standing, integrity or termination predicate.

`referenceAction` accepts explicit 25-value now/ahead source targets and returns
frozen SLOW residuals, effective targets, clipping and represented/custom-reference
motor-control comparison. `diagnostic` reuses the unchanged standing/contact
report. Both integrate nothing. Transition responses additionally retain every
actual native foot sample for support/clearance evidence.

At the initial smoke stage there was no PyTorch, MuJoCo or Gymnasium installed here and no GPU exposed. The approved pilot subsequently installed official CPU PyTorch 2.9.1+cpu locally; MuJoCo/Gymnasium were unnecessary because the native JSONL process supplies physics. Upstream defaults to ten million transitions, batch size 1024 and 1024×1024 actor/critic networks. The retained native baseline measured 13.085593249 wall seconds for 200 decisions: a **serial physics-only extrapolation** is 18.17 hours per million transitions, before SAC updates, resets and evaluation. It is not a training-duration or success prediction; the new environment's observed smoke throughput is reported separately. Parallel native workers and GPU SAC updates would need separate measurement and authorization. Rapier physics does not become GPU-accelerated by installing PyTorch.

The pilot adapter is `scripts/train-getup-stationary-balance.py`, with exact pinned upstream SAC source under `scripts/getup-training/`. It distinguishes finite-horizon bootstrap from physical termination, excludes zero-integration pseudo-transitions, caps exploration, warms critics, checks finite gradients and retains full learner/replay/RNG state. The native observation’s `trained_stationary_policy` flag describes its bundled controller, which remains absent; the separately trained external actor is identified in the pilot summary. The rejected candidate is kept only as experimental evidence. Future training/resumption needs separate authorization and must carry forward the final ledger; no automatic resume was tested. No default policy replacement or deployment occurred.
