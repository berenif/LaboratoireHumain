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
