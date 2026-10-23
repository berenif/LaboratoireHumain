# Local GetUp SLOW holding adapter — not distributed

Pinned GetUp source: `5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e`.
The binary format and arithmetic follow `getup/actor.rs` and `actors.py`:
f64 accumulation, f32 layer boundaries, ReLU, deterministic tanh(mean).
No existing standalone actor WASM artifact was found. The worker source exports
`getup_actor`, but rebuilding the whole legacy worker would add unnecessary
build/toolchain work. The small JS reader uses the existing binaries directly;
no Torch, SDK, model conversion, weight copy or installation is needed.

`slow.js` is deliberately excluded from the normal MVP build. The local QA server
loads existing assets from the repository over loopback only. No UI mode or
public package contains these weights. Upstream redistribution rights remain
unverified. The paused foot cycle is never started by this adapter or its tests.

Contracts reused: physical67, student118 = 67+25+25+power1, 21 qpos-order angle
residuals, actuator permutation, gear multiplication after normalized clipping.
40 Hz inference / 20 substeps at 800 Hz; actual q/dq refreshed for PD each substep.
Moving kd=.1 and holding kd=.2 both have fixture coverage. This comparison uses
only holding. The constant current/ahead target is the existing source standing
pose + terminal height sentinel1.4 + vertical[0,0,1]. A 125 ms lookahead of this
constant target is identical. The .80326 reward height is not substituted.
No source spline or missing NPZ is fabricated. Moving reference playback is
outside this implementation. Root velocities stay in MuJoCo's source frames:
linear world, angular body-local. No Rapier ZXY conversion or qpos normalization.

## Recorded validation (not automatic permission to repeat physics)

- `node scripts/verify-getup-slow-local.mjs`: zero integration, 32 network and
  32 PD fixtures, 67 measured source-state observations and cadence/mapping.
- `node scripts/verify-getup-slow-browser.mjs`: Chromium zero-integration parity
  and one bounded holding command on a static fixture clone.
- `node scripts/compare-getup-slow-local.mjs`: single-use physical comparison;
  refuses an existing result. Entry obtained by 8,000 genuine balance ticks;
  two explicit full `mjSTATE_INTEGRATION` forks include warmstart and share the
  same verified starting state. No qpos/dq assignment during either rollout.
  3.75-second horizon = original150 holding decisions. No controller sum,
  root actuation, external assistance, new gain or fallback controller.

Measured network error: Node raw2.670288e-5/action3.293157e-6;
Chromium raw6.198884e-6/action2.145768e-6 (limits2e-4/5e-5).
PD fixture error0; physical67 error4.440892e-16. Chromium inference mean2.356ms
per decision (25ms budget); this is cloud hardware, not a phone benchmark.

Baseline265a638: 3,000 ticks, 3.75s calm, maximum tilt7.949deg,
maximum foot-origin displacement0.0365mm, maximum motor fraction.5063.
SLOW: 480ticks / .600s, stopped at foot-origin displacement30.636mm (>30mm),
maximum tilt8.737deg, final tilt.664deg, both feet still loaded229/168N.
Maximum motor fraction1, longest all-joint saturation interval27.5ms.
No fall was observed before this stopping criterion. SLOW changed COM from
approximately[.052,0,.845] to[.104,.021,.811]m. A more upright torso alone did
not qualify fixed-support calm. No retuning or second comparison followed.

`maxSlipM` in the compact runner result denotes horizontal foot-ORIGIN drift,
not an integrated tangential velocity of loaded contact points. This limitation
is explicit; it cannot be interpreted as an exact frictional slip measurement.
The common start is a balance-controller stance, not a source get-up endpoint;
it may be outside the frozen student's trained distribution. The failure is
limited to this holding comparison, not a universal inability of GetUp.

New integrations: 8,000 entry + 3,000 baseline + 480 SLOW = 11,480.
Confirmed cumulative MuJoCo159,194 PLUS the preserved earlier unknown browser
run. Rapier45,735 unchanged. Durable result and entry state hash/data:
`web/proofs/slow-holding-comparison.json`; pending count0 on completed exit.
Other evidence: `slow-local-parity.json`, `slow-local-browser.json`.
The normal UI/default controller and deployed site are unchanged.
