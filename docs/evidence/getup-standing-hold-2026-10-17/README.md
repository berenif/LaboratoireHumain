# Five-second native standing continuation

Publication availability: this branch keeps text results, configurations, ledgers and checksums. Full compressed traces, checkpoints and video/image payloads are retained in the original local investigation history, rather than published here. See [publication record](../publication-2026-10-17.md). Historical archive paths below describe those retained local files.

The unchanged GetUp controller resumed the grounded lateral trial at 13.025 s and advanced another 5 s, without replaying preparation or the teacher. All 4,000 native substeps passed the original standing predicates, including a 1 cm foot-origin drift bound from the resume state. The existing continuous-standing counter reached 6.02375 s. No terminal-controller replacement was adopted.

Both feet stayed loaded. Maximum foot-origin displacement was 8.21 mm left / 3.24 mm right; maximum body speed 0.069 m/s and angular speed 0.141 rad/s. Solved foot-contact tangential motion was measured using current generalized velocities, including angular motion at each contact point. Its impulse-weighted speed integral was 4.63 mm left / 4.61 mm right. This is a contact-motion diagnostic, not a claim that foot-origin motion is pure sliding.

Physics computation consumed 13.085593249 s of the new 60 s allowance; 46.914406751 s remains. No thresholds, masses, geometry, weights, friction or controls changed. This single continuation does not qualify the recovery suite.

See `summary.json` for exact metrics, checkpoint identities and zero-integration round-trip verification. The original checkpoint is archived in the grounded lateral evidence. Reproduce from that uncompressed checkpoint with:

```sh
cargo run --offline --locked --release --manifest-path rust/Cargo.toml -p lh-sim --example getup_standing_hold -- CHECKPOINT OUTPUT 5 PHYSICS_COMPUTE_SECONDS
```

Use an explicitly authorized physics allowance. The harness writes native diagnostics, sampled physical snapshots, verified input/final checkpoints and metrics. Rendering with `scripts/getup-diagnostic/render_standing_hold.py` combines the original recorded recovery and this actual continuation, adding no repeated final-frame tail and no physics.

Pass `0 0` for the duration and budget to inspect and round-trip a checkpoint with zero integrations.

Read-only probes at 8 s and 10 s found both CoM and capture point outside the solved foot-support hull. An early residual-freeze experiment was skipped because it could suppress necessary foot repositioning. At 12 s the instantaneous standing predicates already pass. This does not establish that all earlier shuffling is necessary or that it cannot be improved.
