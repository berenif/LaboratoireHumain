# Python GetUp step adaptation

Status: source math and the experimental native bridge have passed initial
parity/positive-control tests. **No complete step or trained step policy yet.**
The reviewed implementation plan is
[`docs/experiments/getup-python-step-plan.md`](../../docs/experiments/getup-python-step-plan.md).

The Python code assembles the original 67/118 observations, source rewards,
reference timing and actor inference. The native engine alone integrates the
original body. Teacher actions are normalized actuator-order motor controls;
SLOW actions are qpos-order residual angles against an explicit moving target.
Moving damping is .1, holding damping .2, refreshed every 1/800 s. There is no
QP torque feedforward, root assistance or physical pose installation in this
source bridge. Source SLOW power stays 1.0.

## Pure checks

```sh
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 .venv/mvp/bin/python \
  -m unittest discover -s scripts/getup-step -p 'test_*.py'
```

`actors.py` reads the existing safe 68/118-input binaries without pickle.
Conditioned-teacher118 initialization inserts exactly zero reference columns
and retains all 42 original outputs. It is an initialization, not a learned step.
`reference.py` keeps exact-source componentwise cubic slowing distinct from
planned/derivative-consistent motion. Planned waypoints are not teacher data.
The optional incomplete upstream supervised distillation folder is not used.

## Native build and checks

```sh
cargo +1.89.0 test --manifest-path rust/Cargo.toml --offline --locked --release \
  -p lh-sim --lib --features stationary-balance-experiment getup::balance::tests::
cargo +1.89.0 test --manifest-path rust/Cargo.toml --offline --locked --release \
  -p lh-sim --lib --features stationary-balance-experiment getup::tests::
cargo +1.89.0 build --manifest-path rust/Cargo.toml --offline --locked --release \
  -p lh-sim --features stationary-balance-experiment --example getup_stationary_balance
```

New JSONL operations:
- `sourceState`: now25, ahead25, teacher_power (default1), holding (defaultfalse).
  Returns exact source inputs/frozen predictions and current candidate controls;
  no integrations or command installation.
- `stepSourceStudent`: reference21, residual21, holding, optional audit.
- `stepSourceTeacher`: controls21 already in actuator order, optional audit.

The ordinary legacy/profile operations clear source commands. Source operations
clear profile/torso-force commands. Schema3 LAB checkpoints preserve source mode,
reference and damping; schema1/2 remain readable. Default application behavior is
unchanged. Python controller state is a separate checked sidecar; a full task
resume must restore and validate that state as well as native state.

`run.py parity` uses zero physics. `run.py positive-control` has a fixed3 s
physics cap,280 actual integrations across explicitly independent parity forks.
Both require `--binary`, `--checkpoint`, `--ledger`, `--output`. The ledger is
prepared for a separately admitted pilot; the script does not create a fresh
allowance on restart. Never pass a LAB checkpoint to the raw runtime constructor.
The archived initial checkpoint remains local; the current compact evidence is
not a remote checkpoint retrieval service.

## Remaining launch gates

Generate/rebase the coherent forward reference and initial pose/twist bridge;
check actual support/unload, every capsule endpoint during translation, measured
landing/load acquisition and the final4,000 native standing samples. Run a
bounded frozen-student probe before deciding if teacher/WEAK/SLOW learning is
needed. Task collection/optimization remains blocked until exact reward,
termination and power configuration is tested, hashed and reviewed. No training
entry point is enabled here. Upstream source/weight redistribution rights are
unverified; no new upstream assets or derived weight binaries are included.

## Adapted task scaffolding and offline reference

`task-config-v1.json` is the exact reviewed adapted reward/mask proposal, with
collection and learning still disabled. `task.py` validates units/shapes, ordered
once-only bonuses, finite physical-terminal partial transitions and masks. Goal
completion and intrinsic phase deadlines are absorbing task terminals; external
compute cutoffs bootstrap. This explicitly differs from the source fixed-horizon
success convention. `events.py` requires consecutive measured native IDs and
actual support margins; missing margins are rejected, never replaced by an old
hull or potential contacts. Its tests are synthetic qualification tests, not
observed completion. The native journal now exports actual torso-up and COM
height at800Hz so the stricter final5° condition can be checked.

`body_reference.py`/`build_reference.py` use the existing pure FK, without a QP
controller or a physical state write. Own compact static witness inputs are in
`docs/fixtures/getup-step/`. They generate uniform40Hz source-format arrays with
an explicit timeline, manifold-corrected feet, derivative-consistent root/joint
motion, actual source frames and valid terminal quaternion/zero-twist padding.
They remain planned/unqualified. Example:

```sh
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 .venv/mvp/bin/python \
 scripts/getup-step/build_reference.py \
 --source docs/fixtures/getup-step/forward-static-witness.json \
 --acquisition docs/fixtures/getup-step/load-acquisition-static-witness.json \
 --output /tmp/planned-forward-reference.json
```

The first.5s is an actual-twist quintic/SO3 bridge. A single recorded elbow20 is
one f32 neighbor below its installed limit and closing; only its TARGET is set
to the installed bound with zero reference twist. Actual q/dq and native PD
measured damping are untouched. Other20/root twists match the recorded state.
This is explicitly not full21-joint entry continuity. The free bridge briefly
moves right-foot targets by~141µm/0.0233°, not a physical foot pin.

`hybrid_state.py` hashes native bytes, all adapter/FK/model/actor inputs and
reference/config, checks phase/clocks/counters/events/anchors, and rejects a
changed ledger or implementation. It verifies exact observed state/snapshot on
load and never rewinds process/global accounting. Initial-entry roundtrip is
verified; continued physical equality at every later phase remains a gate.

`probe_reference.py` is a separately capped.25s/200-frame zero-residual source
PD diagnostic, not a trained GetUp result or teacher collection. It will stop on
its preregistered physical/COM/slip conditions. Frozen actual-height admission
already stopped at.25s for21.48mm COM drift, without flight/nonfoot support/fall.
Changing the actual-height target back to source terminal sentinel1.4 would
change the tested task and is not used to conceal that result.

Optional synthetic SAC equation parity uses existing Torch2.9.1CPU; it creates
only throwaway random networks and compares one source update with explicit
critic/actor/temperature/Polyak equations. Run `test_sac_math.py` in an environment
with Torch; the lightweight math environment explicitly skips that optional test.
No task actor is optimized or saved by tests, and no training runner is enabled.

`audit_metrics.py` decodes measured native audit frames into true COM and task
errors, current positive-impulse foot-side solver-anchor hulls, and source-frame
velocity. Its capture point is an instantaneous variable-height LIP proxy, not
an exact native wrench/viability certificate. Point/segment/empty support keeps
an explicitly undefined polygon margin and cannot qualify a 3 mm inset. Actual
raw capsule clearance is compared to the same target geometry without clamping.
The decoder has been replayed over all 220 P4/P5 frames without new integration;
maximum native COM-height reconstruction error is about 0.303 micrometres.
It is not yet a qualified end-to-end task collection/reward runner.
