# Python GetUp adaptation: lift, advance, place

Status: **five reviews complete. Minimal Python math/source-bridge implementation is cleared; physical-reference/parity gates and the separate learning-config gate remain mandatory**.
Plan revision: R1 final, 2026-10-17 UTC. All five reviewers verified their corrections. Owner: the sole writer of `agent/contact-landing-mvp`.

## 1. Deliverable and current evidence

The deliverable is one complete physical forward step, then five seconds of balanced standing, using a Python adaptation of the actual GetUp learning/reference pipeline. The target is 25 mm foot lift and 60 mm forward advance. Acceptance uses measured whole-foot clearance of at least 15 mm and signed forward placement of at least 50 mm. A video must show the complete sequence and actual ending.

No complete step has passed yet. Existing model-based experiments produced useful balance, unload and lift evidence, but failed placement/landing/aftercare. Their QP controller is paused. Its offline attainable whole-body forward reference may be used as a **candidate reference**; it is not a successful teacher rollout or a trained skill. The original GetUp body, weights, limits, motor gears and default application path remain the baseline.

Two milestones are separate:
1. Mechanical native proof from a declared actual checkpoint, with all gates below.
2. A runnable opt-in GetUp→step feature with shared native/WASM inference, held-out cases, original GetUp nonregression and browser demonstration. A successful checkpoint-only experiment does not complete milestone 2.

## 2. Pinned source and what is actually reproduced

Upstream: [`tianxintao/get_up_control`, commit `5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e`](https://github.com/tianxintao/get_up_control/tree/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e).
Primary code: [environment](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/env.py), [reference interpolation](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/utils.py), [SAC](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/SAC.py), [training/curriculum](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/main.py).

The source method is: strong direct-torque SAC teacher → power-reduced WEAK teacher → recorded rollout, slowed/resampled → SLOW reference-conditioned residual SAC student. It is not a Cartesian PD controller or a QP. We will reuse its actor/critic/SAC implementation and reference-conditioned student interface; task-specific changes are enumerated below.

### 2.1 Observation, inference and units

Native coordinates are world X,Y,Z with Y up. Source vector order is `(native Z, native X, native Y)`. Joint coordinates are radians; joint speeds rad/s; positions metres; velocities m/s. No observation normalization is added silently.

Physical observation 67:
- 0:21: 21 hinge positions in qpos order.
- 21: head world height, metres.
- 22:34: torso-local positions of left hand, left foot, right hand, right foot, each in source vector order.
- 34:37: world-up expressed in normalized torso-local frame, in source order.
- 37:40: true whole-body COM world velocity, in source order.
- 40:43: free-root **origin** world linear velocity, in source order.
- 43:46: free-root angular velocity expressed in torso-local frame, in source order.
- 46:67: hinge speeds in qpos order.

Teacher source input is physical67+power=68. Student input is physical67+target25(now)+target25(ahead)+power=118. Each target25 is hinge q21 + true COM height + torso-local vertical3. The lookahead is **five 40 Hz policy decisions = 125 ms**, not five native substeps. Current/ahead references clamp to the exact placed-foot terminal target and are padded for the entire hold.

Actor: input→1024 ReLU→1024 ReLU→42, split mean21/log-standard-deviation21. Deterministic action is `tanh(mean)*power`. Stochastic logstd is tanh-mapped to [-5,2], then reparameterized Gaussian and tanh squashing, exactly as pinned SAC.py. Power scales action once; it does not scale motor gear, mass or friction. Source SLOW always uses power=1.0; teacher WEAK power is a separate quantity. Physical proportional gains are gear Nm/rad and damping gains are kd×gear Nm·s/rad.

### 2.2 Two different physical action interfaces

Teacher: 21 normalized motor torques in **actuator order**; native torque is gear times clipped normalized control. Power is already represented by actor output/random-action bounds.

Student: 21 residual joint angles in **qpos order**. At each native substep:

`u = clip(q_ref + residual - q - kd*dq, -1, 1)`, then reorder to actuator order and apply `tau=gear*u`.

`kd=.1` while moving, `.2` during final hold; kp=1. The reference q_ref[k] and residual[k] are held for all twenty native substeps of decision k, while actual q/dq are refreshed each substep. There is no target-velocity or inverse-dynamics torque feedforward in this source-faithful student mode. Qpos→actuator mapping is `[1,0,2,3,4,5,6,8,7,9,10,11,12,14,13,15,16,17,18,19,20]`.

Original MuJoCo teacher uses 200 Hz physics, five substeps per policy action. This adaptation retains the application's Rapier 800 Hz physics and uses twenty substeps per 40 Hz decision. Python owns policy/reference/reward/learning; Rust exposes the existing real engine. This is a documented engine/rate adaptation, not exact MuJoCo trajectory reproduction.

Existing native `slow_controls`, `physical_observation`, `slow_observation` and frozen actor already implement the central formulas. Existing `balance.referenceAction` is a fixed-standing-nominal conversion with holding damping .2 and a combined residual clip; it is **not** the moving-source bridge. Existing MotionProfile is also a different interface and is not used as the student controller.

### 2.3 Source training and declared changes

Source SAC losses:
- `y=r+not_done*gamma*(min(Q1target,Q2target)-alpha*logpi)`.
- Twin critic squared errors to y.
- Actor mean `alpha.detach()*logpi-min(Q1,Q2)`.
- Temperature loss `alpha*(-logpi-target_entropy).detach()`, target entropy -21.
- Polyak tau=.005 after each actor update, policy frequency1. The source's `critic_target_update_freq` argument is unused; do not invent an every-two-updates implementation.

Source preset is immutable and separate from the adapted pilot preset below. Source starting settings: Adam actor1e-5, critic/temperature1e-4, betas(.9,.999), initial temperature .1, batch1024, replay1e6, random warmup10000, maximum10M transitions. Teacher gamma .97; README suggests SLOW .95 while saved student metadata uses .97. New experiment configuration must explicitly select and record the preset; no claim that these disagreeing presets are identical. The first bounded learning pilot uses the README teacher .97/student .95 choice, with asset metadata preserved.

Source WEAK curriculum samples clipped Normal(base power,.04), reduces base by .95 after minimum of ten test returns exceeds60. That return threshold is specific to the original task and cannot be copied into stepping. The imported teacher's final power is about .663420431, not .4.

Source SLOW moving reward tracks two hip speeds, COM height and torso vertical; it contains no joint-pose imitation, foot placement or touchdown reward. Source hold reward tracks uprightness, low horizontal speed, COM height .80326 and a fixed standing pose. Source terminal observation separately uses a 1.4 m COM target; parity tests preserve this source quirk, while the new step target uses its actual planned terminal COM height. Source cubic interpolation may produce non-unit target vertical vectors; source parity does not normalize them. Physical observed vertical uses a normalized body frame. New task targets use a declared FK-derived unit vertical and actual height; they are adapted inputs, not exact original-terminal inference. The step hold uses the newly placed stance, never the old home pose. Source 150 hold decisions (3.75 s) become at least200 complete decisions (5 s) after actual supported landing and upright settling.

Source teacher reset is random fall plus80 zero-action decisions; SLOW training starts at a random reference frame80% of the time. The step task instead begins from a physically obtained standing/GetUp endpoint. Initial pilot resets/random starts may load only complete actually recorded native checkpoints, retaining caches; no offline pose injection. Broader training-reset changes need separate review and cannot count as physical success. No pose/velocity assignment is allowed during a rollout. The optional upstream supervised `distill/` folder is incomplete (missing environment/dependency) and is not the reusable SLOW SAC route.

Upstream/model license gate: the pinned recursive tree has no LICENSE/COPYING/NOTICE and GitHub reports license:null. Redistribution permission is unverified; this is not a legal conclusion about local use. Continue authorized local research and independently written adapter code. Do not newly redistribute upstream source/assets or derived weights without verifying a grant; retain notices/provenance and ask the parent if a rights decision is needed. Existing branch publication authority does not establish upstream rights.

## 3. Minimal implementation and reuse

One Python package under `scripts/getup-step/`, no second simulation engine:
- `source_math.py`: typed observation/target assembly, action mapping, tolerances/rewards and source SAC preset metadata. Pure fixtures/tests.
- `native_env.py`: JSONL adapter for source teacher normalized torque and source moving/holding PD; real substep measurements, checkpoint/reset semantics and aggregate ledger.
- `reference.py`: validated forward whole-body reference import, time parameterization, now/+5 sampling, terminal padding and event-aware phase clock.
- `run.py`: parity-only, `generate-entry` through unchanged default GetUp, and frozen-student bounded evaluation; deterministic seeds/configuration/one-shot outputs.
- `train.py`: source SAC task adapter, explicit teacher/WEAK/student stages, bounded transitions/updates/validation reserve and safe model export.
- `test_source_math.py`, `test_reference.py`, `test_native_env.py`: source fixtures, edge cases, protocol and serialization.

Reuse unchanged `scripts/getup-training/SAC.py` (SHA256 `64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420`), actor binary importer/export/parity code and native JSONL process/accounting patterns. Do not reuse the stationary trainer's120-input expansion, reward, hardcoded old deadline,3000-transition schedule or expired360 s allowance. Torch2.9.1 CPU is already available; no new service/install is needed.

Small experimental Rust changes, only after reviews:
- Add explicit teacher-control and source moving-reference commands in `rust/crates/sim/examples/getup_stationary_balance.rs` and the feature-gated LAB adapter.
- Reuse `getup.rs::slow_controls`; pass actual reference and holding flag, rather than converting through standing nominal. Clear conflicting profile/legacy commands on mode switch.
- Reject nonfinite/wrong-length commands before integration; no external torso force in successful teacher collection or step evaluations. Default GetUp remains untouched.
- Serialize reference/holding/phase/virtual clocks/rate/target, unload/landing/readiness counters, event history and fixed analysis/slip anchors with experiment checkpoints, plus original physical state/contact caches and cumulative budgets. Test next-command equality AND subsequent physical evolution after restore at each phase, not only snapshot roundtrip. Native/WASM product serialization is a later separate integration gate.

Preserve paused QP work in a small local checkpoint labeled offline/reference research. Its scripts and raw archives remain available, without enabling that controller or confusing its evidence with SAC.

## 4. Numerical/source parity gates before movement

P0, zero physics:
1. Check source hashes and exact actor dimensions; compare Python/Rust raw42 outputs (tolerance≤2e-4) and deterministic21 actions (≤5e-5) on all archived parity fixtures; record measured maxima separately.
2. Compare every67/118 observation element and frame transform on known native states; current generalized velocities must be used, not stale rigid-body caches. Check COM versus root-origin velocity separately.
3. Asymmetric q/actuator fixtures exercise swapped0/1,7/8,13/14 indices. Compare source PD with moving .1 and hold .2, clipping/no clipping, negative torques and all21 gear magnitudes. Teacher power is applied once.
4. Check source reference endpoint/off-by-one, +5 clamp, terminal padding, constant target and mean-versus-logstd split. Compare original cubic slowing including separately splined qvel×speed; it is generally different from differentiating interpolated qpos. Keep an exact-source fixture path and a distinctly tagged derivative-consistent adapted path. Document original componentwise quaternion interpolation rather than silently claiming SLERP equivalence. Never normalize source target vertical silently.
5. Check native bridge t0/per-substep actual torque against the Python equation, mode clearing, no command side effects on observe, invalid-input zero integrations and checkpoint continuation equality.
6. Add exact source reward/reset/termination fixtures. Teacher reward is upright×head-standing×low horizontal speed×close feet: upright=1 for source COMz<.5, else linear tolerance torso-up≥.9 with margin1.9/value0; head≥1.55 m uses long-tail margin.3875/value.2; vx/vy bounds±.3 m/s use Gaussian margin1.2; horizontal foot separation≤.9 m uses quadratic margin.38/value0. `_small_control` is defined but unused. Moving SLOW uses source qvel[11,17] (hinges5,11): mean Gaussian speed-error tolerance bounds±.5/margin1.3, transformed to(2+r)/3; height error tolerance bounds0/margin.5; product of vertical-component tolerances bounds±.02/margin.6/value.3. Hold multiplies upright, mean Gaussian horizontal-speed tolerance margin1.2, exp[-10(h-.80326)^2] and exp[-sum(q-standing_q)^2/4]. At last moving action k=N−1 use kd=.1/ref[k], but after integration/increment newk=N the source reward already uses hold. Source motion failure is |h-h_ref[k−1]|>.5, hold failure h<.5; N+150 is truncation. Test random-reset/frame-start behavior separately without enabling offline physical resets.
7. Check SAC target/terminal mask and one update against pinned code with fixed replay/RNG; finite horizon truncation bootstraps, physical terminal does not. Preserve the pinned scaled-action entropy-Jacobian quirk for power<1; do not describe it as a corrected probability density.

P1, small native positive control: run a known original GetUp reference with the frozen student through the new bridge; compare commands/observations and unchanged GetUp behavior. Do not demand bit-identical Rapier versus MuJoCo motion. Stop on any mapping, damping, units or cache mismatch before task learning.

## 5. Forward reference and physical phase contract

Candidate input: local `wholebody-forward-swing-landing-upright-path.json`, generated by joint/root/COM/foot and static-force optimization. It fixes the actual entry right-foot transform, uses three actually measured stance contacts/native force points, credits no planned stop reaction, and keeps future non-elbow joints≥20 mrad inside limits. The reference uses original COM height safety envelope [.780625,.829976] m; do not widen it from new path extrema. Static motor fractions are ~.426 prepare, .402 lift, .394/.387 mid/full placement, .411 landing and .147 final stance. These are direct-torque static existence witnesses, not source-PD/frozen-actor authority, a teacher trajectory or a dynamic certificate. SLOW power=1; evaluate its actual residual/PD torque at entry and along the proposed reference, including damping/clips.

Forward is normalized actual entry torso local+Z projected onto the ground, frozen at step entry. Current witness heading is `[.97177241,0,.23592030]`; its60 mm displacement is `[.05830634,0,.01415522]` m. A fresh GetUp endpoint must rebase world targets to its actual stance and requalify the joint/velocity bridge/current support. Never copy planned root/joint poses into the engine.

Reference phases, initial nominal durations:
1. **Prepare,3 s:** retain both measured foot poses while moving the whole body toward a support-feasible posture. The existing actual checkpoint has only1.31 mm COM margin, so do not pretend5 mm margin exists at entry. No deliberate lift until actual unload/support qualification.
2. **Lift,2 s:** coordinated whole-body motion toward25 mm whole-capsule clearance. Actual left normal load<3 N continuously≥100 ms; actual right load≥80% body weight, COM/capture in actual right support and adequate balance must qualify before deliberate lift.
3. **Advance,2 s:**60 mm forward with at least15 mm **every native translation sample**, including all four capsule endpoints/radii and actual orientation. A meaningful current stopping-clearance check is an admission diagnostic; it is not a native guarantee.
4. **Lower,2 s:** hold forward placement and lower to actual contact. No no-contact/15 mm guard during intended lowering. Reference time alone cannot declare touchdown. Confirm actual left load≥3 N for100 ms, right≥50% body weight, total feet≥80% body weight, with no non-foot support.
5. **Load acquisition,1 s:** after that actual touchdown, follow a new motor-only measured-contact two-foot load-shift reference, checking only currently observed contacts, foot motion and actual combined-support COM/capture hull. Do not wait for10% at the unchanged landing pose: its static maximum left share is only4.73%, so R0 would deadlock. Prospective left contact points remain planning geometry until observed. The load-acquisition path must be generated/checked before execution; an infeasible transition stops.
6. **Placed stance/upright,3 s:** progress after actual left load≥10% body weight and both feet persist≥100 ms. Use the new placed two-foot stance and restore torso≤5° measured tilt; the endpoint's static witness needs25.73% left load. Keep contact/load measurement throughout, never credit planned support before contact.
7. **Hold,≥5 real s:** final reference repeats with .2 damping. Count only4000 consecutive native standing samples after actual landing/settling, using fresh fixed analysis anchors and original physical slip anchors.

Event-clock definition: after the initial bridge, r_cmd=1 only while that phase's measured support/clearance admission remains valid, else0; invalid physical state stops rather than pauses. With held r_cmd, dr/dt=(r_cmd−r)/.1; integrate exactly from actual native-counter dt: r_end=r_cmd+(r_start−r_cmd)exp(−dt/.1), Δτ=r_cmd·dt+(r_start−r_cmd)·.1·(1−exp(−dt/.1)). Thus0≤r≤1 and |r_dot|≤10/s. Each phase endpoint has zero underlying reference velocity and matching next-phase pose; validate derivatives there. Clamp τ at the current phase endpoint while an event gate is pending. Ahead is curve(min(τ+.125·r,current phase end)), a declared held-rate projection rather than prediction of future gate success; pad waiting endpoints. Use q_dot=q_τ r and q_ddot=q_ττ r²+q_τ r_dot; same chain rule for Cartesian quantities. Source commands still hold each sampled q_ref/residual20substeps.

Initial bridge occupies the first .5 s of the3 s preparation phase, with real-time r=1. A quintic Hermite curve matches actual generalized pose/twist and zero initial reference acceleration to the validated reference's .5 s pose/twist/acceleration. Translation and hinges use scalar/vector quintics. For orientation use world-left chart R=Exp(phi)R_entry, omega_world=J_left(phi)phi_dot and alpha_world=J_left phi_ddot+Jdot_left phi_dot; use inverse J to derive endpoint chart derivatives. Recompute FK/COM/clearance/collision/limits on this exact bridge; actual initial f32-neighbor measurements are retained, not projected/reset, and no enlarged physical joint limit is installed. Reject a violating bridge. Do not assume the initial measured foot twist is zero or impose a physical fixed-foot constraint.

Explicit per-phase timeouts fail the episode; never skip ahead: prepare6 s, lift4 s, advance4 s, lower4 s, load acquisition3 s, upright5 s, all subject to the40 s total episode and per-case compute cap. Before each trial admit enough budget for the full final5 s tail. Evaluate final runtime interpolation itself, including C1/PCHIP one-sided acceleration knots, FK, COM, capsule separation and limits; coarse static knots do not cover it.

The old checkpoint entry9.25 s plus18 s nominal suffix needs≥27.25 s episode horizon plus bounded gate delays. Use a declared≤40 s diagnostic limit; final five seconds are never shortened to fit. A reused exact checkpoint is milestone1 scope only. It must be followed by full GetUp→readiness→step without state resets for milestone2.

## 6. Evaluation and learning decision tree

A. After P0/P1, evaluate the **frozen pretrained student** on the validated new reference, first a short preparation/hold then one bounded complete attempt if safe. This is an out-of-distribution inference diagnostic. A failed attempt is useful data, not proof that source GetUp is unsuitable. A successful attempt still needs the full acceptance matrix.

B. If frozen inference fails, first verify observation/action parity and elementary geometric/actuator feasibility. Feasibility-learning for a new teacher does not require a pre-existing successful dynamic teacher; it is allowed to discover one. A qualified actual native rollout is nevertheless mandatory before it becomes SLOW reference data. The offline pose path alone is insufficient. Use task-specific teacher training rather than repeatedly fine-tuning the entire GetUp student on the stationary reward.

C. Proposed teacher task adaptation, subject to R1: condition direct-torque teacher on the same now/ahead reference, giving118 inputs, to expose the requested target motion. This remains an explicitly hybrid/partially observed118-input policy: phase/event counters, contact history, anchor drift/slip are external environment state, not fully encoded. Do not claim the references make the process fully Markov; save complete hybrid state and evaluate ambiguity before any observation expansion. Name it `conditioned_step_teacher`. Copy old physical columns0:67 unchanged, move old power column67→117, set new reference columns67:117 exactlyzero, and use fresh critics. Prove all raw42 outputs AND deterministic21 actions match teacher68 for arbitrary references/powers at initialization. Preserve the original68 teacher asset. This is an explicitly adapted reference-conditioned teacher, not exact original68 behavior after learning. At zero new-column weights, full raw/logstd and deterministic initialization must match teacher68. Student remains118. Do not feed residual angles to a torque-trained actor.

D. Task reward configuration is frozen before collection. Retain/source-test applicable upright/velocity/height terms, replace fixed home targets by the phase target, add smooth foot-position/orientation/clearance tracking and signed forward-placement terms, and add one-time event bonuses only for measured unload→clearance→placement→supported touchdown→quiet completion. No repeating clearance/progress bonus, stationary-anchor penalty or generic unloaded-foot penalty that contradicts the task. Fall/flight/non-foot/slip failures terminate; no reward can override acceptance. Exact weights, tolerances, termination masks and phase observability receive tests and a logged configuration before any optimizer step; no hidden tuning within an evaluation.

E. Stage order is explicit: strong conditioned direct-torque teacher at power1 → WEAK teacher curriculum → qualified actual WEAK complete rollouts → retiming → SLOW reference-conditioned residual RL. WEAK power is reduced only after preregistered physical completion tests; do not copy return60. Record uniform40Hz arrays qpos(N,28),qvel(N,27),com(N,3),com_ori(N,3), plus phase/contact labels. Define P=[[0,0,1],[1,0,0],[0,1,0]], mapping native to source. Then qpos=[P·root_position, quaternion_wxyz(P·R_native·P^T),hinge_q21]; qvel=[P·v_root_origin_world,P·R_native^T·omega_root_world,hinge_dq21], so native physical[40:67] already supplies source-compatible qvel. COM=P·COM_native and vertical=P·R_native^T·worldY. Snapshot xyzw requires basis conjugation then wxyz conversion; never store world angular velocity as source free-root qvel or confuse it with quaternion derivative. Preserve source POST-action recording alignment in source fixtures; new task format includes explicit initial state at t=0, equal-length arrays and valid normalized root quaternions/fully padded velocities. Source's appended zero-quaternion/no-qvel target sentinel is never a physical reset state. Sparse7planned knots first need an explicit physical timeline and uniform samples; they are not teacher data. Keep exact source resampling and derivative-consistent adapted resampling separately tagged. Train SLOW with source residual interface and118-state conditioning, starting from frozen student only with saved baseline/paired evaluation. Random reference starts use actual recorded checkpoints only in the first pilot; all success tests start from the real full sequence. If no valid teacher is found within the pilot, report the exact failure/resource bottleneck before extending.

The previous stationary SAC pilot lost all four passing held-out cases after3000 transitions/1000 updates; it is rejected and will not be repeated unchanged. Source10M-transition training is not a promise of a one-hour CPU result. Do not call random exploration or a hand-generated reference a trained teacher.

## 7. First post-review pilot budget and stop/reassessment rules

One local CPU pilot:≤1 hour total process wall and≤300 s **new aggregate physics**, all processes/engines/retries included. Historical research charged593.400428476 s is separate and cannot be reused as allowance. Account optimizer, setup, rendering and evaluation wall time too; keep native substep counts and uncertain interrupted requests conservatively charged.

Prospective allocation ceilings:10 s source bridge positive control;50 s frozen/reference probes;120 s task data/learning;100 s held-out/nonregression validation reserved;20 s emergency/accounting headroom. Spending unused stages is not automatic permission to consume validation reserve. Per physical trial≤60–90 s; stop after first causal failure and diagnose before a different candidate. Initial learning-viability override: first256 actual new transitions are warmup from the frozen initializer plus Gaussian std .02 in its action units, clipped to the current power/action bounds. Then collection uses the current source stochastic actor (logstd[-5,2]); TD/actor sampling retains the same pinned stochastic SAC math. Log means/stds, realized clipping and behavior-versus-deterministic drift. After warmup, call unchanged SAC.train(batch128) once per4 actual new transitions, at counts260,264,..., at most200 updates;≤2000 transitions is an outer ceiling, so the200-update limit normally binds at1056. Fresh critics/targets and all overrides are hashed separately from immutable source presets. First strong-teacher stage uses power1; do not weaken before completion. Freeze exact dimensioned task-reward weights/tolerances/events/terminal masks in a tested/hash-locked config and obtain reviewer clearance BEFORE collection/optimization. Until then only parity/frozen probes are cleared.

Preserve the frozen actor and fixed zero-physics probe bank. Every25 updates compare deterministic actions; RMS drift>.05 (rad for student residual, normalized control for teacher) is an engineering reassessment trigger, not a convergence metric. Stop/rollback the candidate on known quiet/GetUp regression or failed paired safety checks before consuming the full pilot. This is not a certified trust region.256 samples/200 updates demonstrate protocol/learning viability, not SAC convergence. Measured throughput determines achievable counts. No paid remote/GPU service, full blind finetune or unbounded training. At archived native speed,18 s suffix costs roughly46 s physics and27.25 s full chain roughly70 s, before gate delays; measure anew before admission.10 s P1 is a short parity slice,50 s frozen stage roughly one suffix,100 s validation cannot cover the full endpoint/perturbation matrix. The first pilot is viability-only; the full acceptance matrix needs a later bounded ledger/report, not a false one-hour completion promise.

Stop a trial on nonfinite data, bridge/parity mismatch, integrity/contact/slip failure, actual unsafe state, invalid phase domain, timeout or budget exhaustion; save full state/commands before any recovery. Checkpoint actor/critics/optimizers/replay/RNG/native state/reference clock and ledger together. Before extending the pilot, send measured throughput, total ledger, training/frozen comparison and the next causal hypothesis to the parent. A checkpoint is not a claim of task completion.

## 8. Prospective acceptance, regression and integration

Mechanical pass requires ALL of:
- Full chronological unload→lift→forward→lower→actual landing, with original body/caps and only physical motor commands.
- Whole-capsule clearance≥15 mm at every native advance sample, actual signed forward displacement≥50 mm, absolute lateral displacement≤20 mm relative to the fixed entry heading/origin, and nonintersecting actual capsules. Freeze heading/origin before deliberate unload; yaw cannot redefine forward afterward.
- Measured touchdown/support≥100 ms, actual≥10%BW left-load acquisition, then the FINAL4000 consecutive800 Hz integration samples after upright settling through the declared terminal native sample, each meeting original native standing AND measured torso tilt≤5°. Reader rejects missing/duplicate/nonconsecutive native IDs. Original native standing means head>1.4 m, torso-up≥.97, both feet≥3 N with support persistence≥40 samples, total foot load≥.8Mg, foot-only support, COM/capture≥3 mm inside solved support, every body linear speed≤.22 m/s/angular≤.65 rad/s, and fixed postlanding anchor drift≤10 mm. Store actual predicate results/version; do not score the longest quiet window anywhere in the trial.
- Flight0, non-foot support0, cumulative loaded slip≤10 mm per foot from original anchors. Brief contact or a20 mm generic reposition counter does not meet this definition.
- Complete real-time-labelled video and native measurements agree; no repeated tail, fabricated merged time or offline planned pose used as outcome.

Preregister before collection: seed20261017 actual checkpoint/frozen probe; mechanical replay seeds20261017 and20261018 from the same declared actual entry; integration endpoints supine, prone and side-supported generated by unchanged GetUp with seeds20261017/18/19; held-out cardinal±worldX/±worldZ pulses10 N for.2 s after readiness, seed20261020; repeated invocation two successive requested left steps from actual placed states, with fresh target rebasing. These are prospective cases, not existing passes. After one mechanical proof execute feasible cases in a separately admitted budget and report all outcomes. The repository's full24-case recovery suite remains unqualified by this plan/pilot; do not imply it passes. Preserve frozen baseline GetUp paired tests; any source-getup regression blocks promotion.

Product integration, only after mechanical evidence: export the selected frozen or trained actor in the existing safe binary format; verify Python→Rust→WASM raw/action parity, shared observation/action calculations, checkpoint/pause/resume/reset/repeated invocation and browser runtime. Reuse UI action transport but explicitly extend shared Action contract/schema with an opt-in step request and serialized mode. Add stale/generation-mismatch/pause/reset/repeated-click tests; there is no existing complete step command to assume. Profile inference+800 Hz actual physics against40 Hz decision deadline before claiming interactive performance. Keep default GetUp source behavior and assets; publication remains authorized branch commits/pushes only; no main merge/deploy is included.

Evidence per batch: immutable source/model/reference/actor/binary hashes, command journal, native samples/impulses/torques, all failures, wall/physics budget, compact metrics, reproducible tests and complete video. Raw research archives may stay local; commit compact fixtures/manifests/source/tests. A local-only inventory hash is not a retrieval route. For the new result, provide either verified recipient-accessible uploaded checkpoint/reference/report/video receipts or a clean-checkout deterministic `generate-entry` + replay recipe with all seed/config/source/binary/asset hashes and charged compute. Verify the retrieval or reproduction path. Publish the final real video/report through authorized user attachments; weight redistribution still follows the license gate.

## 9. Five-review launch gate

All reviewers inspected R0 SHA256 `fb4607657558d080fa0c4317abc9d1ee2b20ebc7dcac609b72ff8e835d245126`. R1 incorporates their source order/fixtures, exact timing/bridge, load-acquisition deadlock, bounded-POMDP/pilot and acceptance/retrieval corrections. R1–R4 cleared corrected content SHA256 `b3070dda516e9e37b3fa5e96c1c27c7388a6a34963cb319f28a454643b70bd21`; R5 cleared the final two acceptance wording fixes at SHA256 `5319ed4e197066c17ee9abf33106a5a744f52d46fab9e0be20db5c1f8b69b152`. This final stamp changes review-status metadata only. Parent verifies all five receipts before implementation/launch.

| Review | Owner | Required focus | Status |
|---|---|---|---|
| R1 source fidelity | adapt_getup_step_method | strong/weak/SLOW semantics, action interfaces, SAC, explicit task adaptations | CLEAR after R1 correction verification |
| R2 mathematics/frames | review_unilateral_contact_mvp |118-state math, frames/units/order, damping, interpolation/parity tests | CLEAR after R1 correction verification |
| R3 physical trajectory | audit_stance_model_mismatch | feasible reference versus teacher, real phase/contact/clearance/landing gates | CLEAR after R1 correction verification |
| R4 training/resource | review_getup_training_budget | decision tree, observed pilot risk, bounded compute/validation reserve | CLEAR after R1 correction verification |
| R5 acceptance/integration | review_getup_mvp_acceptance |complete proof, nonregression, native/WASM/browser delivery | CLEAR after R1 correction verification |

Launch order after clearance: preserve research checkpoint → implement/test minimal Python math + native source bridge → known-GetUp parity → bounded frozen reference probe → evidence-led learning branch if needed. The deliverable remains the foot lifting, advancing and landing, followed by five seconds of balance.

## 10. Source-engine learning addendum, 17 October23:54 UTC

This is an explicit post-review adaptation, informed by the separate original
MuJoCo positive-control result. The five reviews above remain the record of the
original plan; this addendum receives focused source/resource review before
its new collection or optimizer scope can be admitted.

### Evidence and engine boundary

The imported WEAK actor completed one original200 Hz MuJoCo2.1.3 get-up from the
exact dm_control1.0.1 reset algorithm: teacher return184.4147, head above1.4 m
from teacher time0.965 s through the final5.29 s. The source diagnostic reports
actual flight/nonfoot/slip, final torso tilt12.96 degrees and no five-degree
quiet-step qualification. Its final state is **not** silently used as a step
start. The compact report is
`docs/experiments/getup-source-engine-comparison.md`.

Learning will therefore first use the verified original MuJoCo source engine
inside Python: teacher stages use the pinned static200 Hz model, and SLOW uses
the pinned high-frequency800 Hz model. Native Rapier remains the application
engine and a separate later transfer/product acceptance gate. No default app
engine, mass, inertia, motor gear, friction, joint limit or installed actor is
changed by this decision. A MuJoCo step would be a source mechanical milestone,
not the final native/browser product.

The attempted transferred-native quiet comparison retains its failed0.2 mm
geometry gate. Native root quaternion norm squared1.00059463 explains a0.737 mm
body-position difference after source unit-quaternion projection. This is not
an exact cross-engine state match, nor proof that every failure is caused by
physics rather than initial-state distribution. No task optimizer was run.

### Minimal source task environment and interface

1. Preserve independently written source math, safe actor loader and source SAC
   equations. A new `source_task_env.py` will wrap the counted official MuJoCo
   adapter, observe67 source physical coordinates, sample a40 Hz policy, and
   hold actuator-order teacher controls for5 source integrations. Source SLOW
   holds qpos-order reference/residual for20 source integrations, recomputing
   measured q/dq PD each800 Hz substep with kd .1 motion/.2 holding. Power and
   original gear are each applied once.
2. Add per-source-integration solved foot/body loads, transported foot-side
   force-bearing material points, actual body/foot poses and velocities,
   whole-capsule clearance, normalized applied motor controls and warnings.
   Keep solved-force versus refreshed-geometry stages explicit. Only actual
   positive force-bearing points enter the declared support/LIP proxy; no
   planned contact or old hull is substituted. These are source predicates,
   not a claim that source soft contacts reproduce native impulse predicates.
3. Reuse the reviewed pure geometry/reward/event calculations through an
   explicit source-to-canonical frame adapter. Use a **separate** hashed
   `task-config-mujoco-v1.json`:200 Hz teacher means5 samples/25 ms transition,
   20 samples/100 ms persistence and1000 consecutive samples/5 s final hold.
   Source SLOW800 Hz and final Rapier acceptance retain20/80/4000 respectively.
   Durations, reward integral fractions and physical-terminal masks must remain
   identical in seconds. No blind reuse of800 Hz counters on the200 Hz world.
4. Freeze forward at real step entry: source torso local +X projected onto the
   ground (equivalent to model/native local +Z under P). Rebase the planned
   +60 mm/25 mm whole-body guide to the **actual** source stance/root/q/dq and
   actual contact geometry. The old B27-native witness is a potential guide,
   not a teacher or an already matched source reset. Regenerate and check the
   bridge, actual installed joint bounds, root rotations and both-foot closure.
5. Freeze pre-action reference/phase for each executed interval reward; update
   measured event timers afterward. Source-native100 ms unload, clearance,
   forward placement, actual3 N touchdown, actual10% BW load acquisition and
   final upright hold gates retain their physical thresholds. Whole-capsule
   advance clearance15 mm, forward≥50 mm/lateral≤20 mm, zero step-window
   nonfoot/flight and per-foot cumulative slip≤10 mm remain the mechanical
   requirements. Initial get-up may have flight/nonfoot before the separately
   qualified and declared step-entry window; it cannot be relabeled as a step.
6. Qualify a source quiet entry first, using source SLOW terminal holding or
   another measured source condition. Require actual five-degree upright,
   continuous five-second quiet/source support proof. The original actor's
   terminal observation sentinel1.4 is preserved in source-parity fixtures;
   any task reference actual-height inputs are explicit adaptations. A final
   WEAK head-height event alone does not admit the stepping task.
7. Checkpoint complete source physical state, warm starts, controls and stage
   caches together with hybrid phase/reference clock/events/anchors, replay,
   all RNGs and budget. The present saved final q/qvel/control/contact packet
   is measured state only, not a complete solver-cache checkpoint. Do not
   advertise exact resume until next-command and continued physical-evolution
   equality pass. A deterministic one-prefix replay to capture discarded
   source caches, if needed, is a separately counted diagnostic.

### Safe initialization and full-checkpoint availability

The pinned source tree does ship actor/critic/critic-target/actor-optimizer/
critic-optimizer/log-alpha/alpha-optimizer files for both WEAK and SLOW. It does
not ship a replay buffer, RNG/environment state or a separate STRONG checkpoint.
Only the safe actor binaries are currently imported locally. Availability of
old critics/optimizers is not evidence they fit a new task.

Preserve the original WEAK actor and prove the audited68→118 zero-reference-
column expansion again. Old physical columns0:67 stay unchanged, old power67
moves to117, reference columns67:117 start exactly zero. For the changed step
reward/action context use fresh twin task critics and target copies, fresh
actor/critic/temperature optimizers and declared alpha. An old teacher critic
has89 inputs; the new conditioned teacher critic has139. This is task
adaptation rather than an exact training resume. No `.pt` is inspected with
unrestricted pickle; any future inspection requires pinned hashes and CPU
`weights_only=True`, without additional safe globals.

Before exploration, test the initialized teacher's actual deterministic controls
on the qualified source quiet entry. Explicitly distinguish the imported WEAK
power0.663420431 from the proposed strong-stage task power1; neither inherits
a stable-stance guarantee. Retain the reviewed deterministic-drift, rollback
and paired frozen-baseline gates with fresh critics. A successful get-up
asset is not automatically a quiet-step teacher. Preserve frozen source GetUp
paired baselines and stop if the initializer cannot supply stable support.
Do not pretend that zero-weight reference inputs condition behavior before
learning. If that baseline fails, report the exact obstruction and review a
separate initialization/curriculum change; do not spend a blind SAC pilot.
Strong→WEAK completion tests and qualified actual WEAK trajectories still precede
SLOW task training. No source optimizer or task collection is currently enabled.

### Throughput and prospective admission

The measured original-source episode included full inference, dual actor parity,
per-integration durable budget writes and complete journals:250 teacher
transitions plus80 reset decisions took3.0843 process seconds, about81.1 teacher
transitions/s conservatively. Actual integration alone cost0.0916405 s for1650
steps. Those are different quantities. A prior synthetic CPU128-batch benchmark
of100 critic updates took3.659 s; it is not a newly measured source-task optimizer
throughput or a convergence estimate. The next fresh budget proposal must include an explicitly bounded end-to-end
task/update benchmark substage and protected validation reserve. Training is
released only after that authorized substage measures throughput and verifies
the remaining reserve; the benchmark is not a circular precondition to proposing
its own budget.

The first Python pilot is closed and its unused300 s allowance is not reused.
The current original-engine comparison phase has its own10 s physics cap and
00:34:04 UTC deadline; it authorizes diagnosis/design, **not task learning**.
Any new collection/optimizer phase requires a tested/hash-locked source config,
qualified entry/reference, measured event/restore wiring, preserved frozen
baseline, focused R1/R4 clearance and a fresh bounded resource proposal. Never
extrapolate the old Rapier178-hour estimate as the source-engine ETA, or imply
that a small viability pilot will train the complete movement.

Addendum review receipts: R1 source fidelity cleared §10 at content SHA256
`63dcd00ce7c89580d14b98ce8c5d44ded6cdd0c7ab282cbeb689cf0a1f073116`.
R4 cleared the final two power/budget-admission clarifications at
`00c0a174fefc8d2b4e355cf01bb15b4fd690f43977c71754923c8a26c8e74747`.
These are design clearances, not collection/optimizer or physical qualification.

### Stabilization admission clarification after source SLOW evidence

The complete source SLOW follow-through reaches actual upright low-velocity
standing but fails the fixed-anchor quiet requirement through measured stance
sliding. Preserve that failure. Requiring an already five-second-quiet policy
before learning quietness would be circular. The exact actual source endpoint
may therefore be admitted as a **training-only initialization** for an explicit
stabilization→forward-step curriculum, after safety/interface/data/budget gates.
It is not an evaluation pass, successful step teacher, or permission to weaken
any final quiet/slip requirement. Curriculum stabilization has its own locked
objective/events/config; full step phases remain unavailable until actual
stabilization and whole-body reference admission. Source collection/optimization
still require the next bounded proposal and remain disabled in this comparison
phase.

## 11. Proposed next phase: source stabilization before stepping

Status: proposal only. The comparison phase ends at 2026-10-18 00:34:04 UTC.
Its remaining allowance is not a task-learning allowance. No task data or
optimizer is enabled by this section or by the new disabled source preset.

### Delivered environment boundary

`scripts/getup-step/source_task.py` decodes actual source q/qvel, true FK/COM,
original actuator torques, solved positive-force support anchors transported to
the current pose, every-body speeds, foot loads and slip. Its stabilization
reference is the actual measured entry pose; target velocity is zero. This is a
new fixed-reference task input, not the original source terminal-height sentinel.
The only phase is `stabilize`. Actual 200 Hz integration IDs drive the reference
clock, with five native samples per 40 Hz transition. Quietness need not hold at
reset. Entry anchors are fixed prospectively, and every sample contributes to
persistence, actual cumulative loaded slip and the final five-second counter.
No right-only polygon is inferred from the bridge's combined support points.
The forward-step curriculum remains disabled until per-foot support telemetry
and a source-entry-specific trajectory are admitted.

`scripts/getup-step/source_task_env.py` connects this core to the original
200 Hz static model, direct actuator-order torque held for five integrations,
actual per-frame failure checks, and the source full-cache/Python sidecar.
The integration callback must own the separately admitted write-ahead resource
ledger. Current config refuses execution before any control write. Checkpoints
bind model/library/code/config/native data, actual packet identity, task clocks,
anchors, all counters/event history, previous controls and complete RNG state.
Restoring an environment never restores/refunds the global budget ledger.
Pure/fake continuation parity is distinct from unmeasured paired engine-dynamics
parity, which remains an explicit admission gate.

Only verified finite physical-terminal partials are valid shorter transitions.
A physical/task failure and absorbing successful five-second goal have mask0;
an extrinsic completed-interval cutoff has mask1 and the actual final observation.
Infrastructure, corrupt observation or uncertain/unfinished requests abort replay.
A recorded SLOW residual rollout is measurement evidence, not a held direct-torque
teacher replay transition. A future teacher interval must verify all five applied
normalized torques against its held actuator action.

### Initialization choices and curriculum

The actual SLOW endpoint is an allowed training-only initialization: torso is
upright and slow, but feet still creep. Its strict quiet proof failed. The WEAK
actor at its trained power0.663420431 failed this source stance after30 ms under
the disclosed cold-cache switch. Power1 is not a separately trained STRONG actor;
the optional diagnostic was correctly skipped after that failure.

Preferred next experiment is an explicitly adapted **SLOW-derived torque
imitation warm start**. Labels must be actual normalized actuator controls from
measured source SLOW packets, not residual angles. A new 118-input direct-torque
actor is fitted to these labels with source action/order/power semantics, using
first3s for fitting and final2s for contiguous held-out validation of the measured
SLOW final5s. Use only each original40Hz decision FIRST800Hz substep, without averaging:
labels at source IDs2401+20n pair with PRE-action packets2400+20n. The first120
pairs (3s) fit and last80 pairs (2s) validate. The future200Hz teacher holds this
approximate torque25ms; fitting cannot establish dynamic equivalence. Fixed entry target25 now/ahead and power1 form its118 inputs. The fit is an approximation;
copying the SLOW residual head is forbidden. At power1, target mean is
atanh(clamp(u, -1+epsilon, 1-epsilon)); action-space error must also be reported: held-out RMS≤0.02 and maximum absolute
error≤0.05 are required before a physics benchmark. Initialize constant source
logstd=ln(0.02), using the inverse source[-5,2] map, and freeze those output rows
during supervised fitting. This std is latent before tanh; it is not constant
normalized-control noise. Epsilon1e-6 is fixed; any saturated label
|u|≥1−epsilon aborts the fit, with counts retained. Failing these fixed thresholds stops this candidate.
Fresh source-task twin critics/targets, alpha and all optimizers are used.
The original actors remain immutable. Any supervised updates require the fresh
phase authorization below, even though no physics is involved.

Alternative is the audited WEAK68-to118 zero-reference-column expansion with
fresh task critics. It is a known failed stance initializer, not a preserved
quiet baseline. It can supply failed stabilization training episodes only under
an explicitly selected/hash-locked protocol. It is not automatically tried if
the preferred fit fails. Choosing another initializer is a new causal decision.

Stabilization uses dense pose/height/velocity/foot/clearance terms and measured
slip/control costs, plus one once-only quiet-completion bonus. Failures remain in
replay with their correct terminal masks; phase deadlines are intrinsic failures.
No unload/advance reward or commanded foot movement is enabled in stage0.
After stabilization evidence, propose preparation/unload, lift, forward60 mm,
touchdown/load-acquisition, upright settling and final5s as later explicit stages.
The original full-step acceptance gates are unchanged. No SLOW task distillation
or WEAK power reduction occurs without qualified actual complete task rollouts.

### Fresh bounded proposal and stop rules

Proposed next phase: at most30 minutes process wall and10 seconds aggregate
physics across all engines, starting only after parent admission. These are new
ceilings, not reused pilot reserves. Maximum actual task transitions1500, including benchmark paired branches,
failed episodes, warmup, training and frozen/candidate validation;
maximum8 seconds per episode, with every failed/reset/probe integration charged.
Nonvalidation physics ceilings are benchmark1s plus training/collection6s,
with3s reserved for validation. All fit/setup/benchmark/training must stop with
at least300 wall seconds remaining. No paid/remote training. Release gates are sequential:

1. Pure admission and initializer fit: lock exact config/code/model/actor/data
   hashes, failure taxonomy, endpoint reward reduction and full RNG/checkpoint
   payload. At most100 supervised actor updates, batch128, one fixed seed;
   preserve fit/validation action error and source baseline actors. No SAC yet.
2. Counted throughput/continuation benchmark: at most64 actual policy intervals,
   1.6 simulated seconds,1 physics second and120 seconds process wall, including
   a bounded same-cache next-command paired continuation. At most16 throwaway
   critic-only updates on valid actual transitions. Discard all benchmark network,
   target, optimizer and RNG state; retain every physical charge. Candidate
   initialization returns to the immutable fitted actor and declared RNG. Measure full decode/event/
   journal/replay/update throughput; pure forward estimates do not release SAC.
   Require the fitted actor to complete the actual held40Hz/200Hz stance benchmark
   without parity or immediate physical failure before SAC actor release.
   Low supervised error alone is insufficient. Admit the remaining episode count
   and a full5s terminal tail from these rates.
3. Stabilization viability pilot: at most1500 actual transitions total,
   256-transition replay warmup using the frozen fitted actor plus0.02 normalized
   Gaussian control noise (one clipping to power1),100 critic-only updates with actor frozen, then at most25 actor
   updates and25 associated critic/alpha updates, at most one per four NEW actual
   collection transitions after warmup; batch128, source learning
   rates/discount/tau. Critic-only warmup manually refreshes target critics after EVERY update with
   source Polyak tau0.005 while actor/alpha parameters and optimizers stay frozen.
   Setting policy_freq huge is insufficient because upstream ties target refresh
   to actor updates. Exact target/untouched-actor tests gate this adaptation.
   The16 benchmark updates are throwaway, separate from100 candidate updates.
   Ordinary source policy_freq1 resumes afterward. Actor updates are separately released after critic warmup.
   Log fresh-Q behavior and deterministic actor drift on fixed real probes.
   After warmup the unchanged source stochastic actor sampling is used.
   Exploration/control bounds and the chosen exact warm-start config must be
   frozen before collection. This is a learning-viability probe, not a plausible
   SAC convergence or full-step promise.
4. Protected validation: retain at least3 physics seconds and5 minutes wall for
   paired frozen-initializer/candidate actual episodes and the full final5s tail.
   Protect640 actual policy intervals for the worst-case pair2×8s×40Hz.
   Thus benchmark≤64 plus training/warmup/failed-episode intervals≤796 leaves
   the shared1500 interval ceiling intact. Check wall, physics AND transition
   reserves before every training admission. Full episode admission is computed from measured throughput before launch;
   unspent stage ceilings do not transfer automatically. The full varied-start/
   repeated-step/product regression matrix cannot be certified by this pilot.

Stop/rollback on parity/nonfinite/infrastructure uncertainty, budget exhaustion,
material safety regression relative to the same failed/stable initializer,
deterministic action RMS drift>0.05 without reassessment, or a failed paired
probe. No hidden reward/cap/standing/slip changes and no second blind candidate.
Failure of this tiny pilot is evidence about this initializer/protocol only.
Exact task config and new resource ledger must receive focused source/math and
budget review before collection or any supervised/SAC optimizer is released.
