# Minimal physical demo (experimental)

The body starts visibly prone at construction, settles physically for 400 native
samples, and gets up autonomously. After recovery and a three-second wait, use
`P`, the button, or touch to apply a real strike. The default approach is from
behind the current torso; a blocked path is rejected, not replaced with another
angle. The ordinary room-centre strike remains explicitly experimental.

One worker/world continues: no mid-run pose reset, teleport or external support.
The explicit `getup-repeat-balance-acquire-experiment` recruits the existing joint
balancer after 80 consecutive supported, upright, low-motion, torque-admissible
samples. Control admission is **not success**: the original COM/capture, full-body
motion, contact, 800-sample stability and 25-second acceptance remain unchanged.
The handoff uses the existing 20-native-sample bumpless torque interpolation.

## Run

`node scripts/serve-minimal-demo.mjs <verified-worker-assets> 5186`

The assets directory contains generated `lh_worker.js`, `lh_worker_bg.wasm` and
`getup-800hz-3plus1-repeat-cycle-v1.json`; the server checks exact hashes. Runtime
binaries are not committed. This diagnostic view does not replace the richer app.

Rebuild with Rust 1.89.0, the locked offline Cargo dependencies and wasm-bindgen
0.2.108, using a single shared Cargo target:

```
cargo build --manifest-path rust/Cargo.toml --offline --locked --no-default-features --release --target wasm32-unknown-unknown -p lh-worker --features lh-worker/root-translation-carry-experiment,lh-sim/getup-terminal-balance-experiment,lh-sim/standing-dynamics-experiment,lh-sim/unit-root-rotation-experiment,lh-sim/getup-repeat-cycle-experiment,lh-worker/getup-terminal-tail-observer-experiment,lh-worker/getup-policy-adaptation-experiment,lh-sim/getup-repeat-balance-acquire-experiment
wasm-bindgen rust/target/wasm32-unknown-unknown/release/lh_worker.wasm --target web --out-dir <assets> --out-name lh_worker
```

Copy the unchanged profile from `docs/evidence/profiles/getup-800hz-3plus1-repeat-cycle-v1.json`.
The exact ten-feature compiled closure is `capabilities.json`. There is no
wasm-opt transformation. The existing pretrained policies are unchanged.

## Evidence and limits

`node scripts/check-minimal-physics.mjs <assets> <result.json> 0 belly behind`

The bounded actual Node/WASM test passed initial get-up and **two real
strike/fall/get-up cycles in the same world**: 2 impacts, 3 falls, 3 recoveries,
30,960 real + 1,640 forecast integrations, 27.02 wall seconds. The final recovery
earned 800 stable samples within 22.42 seconds of impact. One constructor, no
reset, no direct pelvis force/torque. Result SHA-256:
`734bf46615cdd891caf3d79d3dac53a23b4730f990c1e2209ab6d46717dce68e`.
Worker SHA-256:
`f9b06192c1d9b265dd179e80d8aa6c4ce72ad5cc0fa2dd2bddb9ee8fdb87fa3a`.

This is one constrained trajectory, **not arbitrary impacts or unlimited repeat**.
Earlier neutral starts, ordinary second strikes and the pre-fix rear strike
failed; they were not reset or relabeled as successes. The browser preview is
not yet browser-validated because its owner-login gate prevented automated
access. The test permits at most 120 physical seconds / five wall minutes and
two strikes; it stops honestly on failure. No optimization or training occurs.
