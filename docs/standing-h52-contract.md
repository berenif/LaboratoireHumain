# H52 — knee contribution to the remaining horizontal foot motion

H49 fails prolonged drift despite bounded speeds. H50's full distal updates
reduce one-step horizontal motion, but retain 0.854–1.898 mm/s in the three
sampled states, above the declared 0.2 mm/s controller deadband. Test whether
the two existing knee hinges provide missing local actuation authority.

Use the same retained H49 tick-350 snapshots. A read-only native inspector
extracts the original configured motor targets, gains, force ceilings and
models from the snapshot. Check the six known distal axes against their
saved configurations after f32 conversion. For the added left/right shin X
motors, use their exact native parameters; alter only velocity bias, under
their unchanged native force ceilings. All eight controls are single-axis
hinges. No joint, collision, timestep or solver changes are permitted.

Compare six distal controls with those six plus the two knees, both starting
from H50's best six-control bias. Require exact 25-body replay of the original
baseline and H50 result before any new query. Optimize actual predicted
horizontal foot-center motion, using at most eight full Gauss–Newton
iterations, ±1 Nm central probes, 1e-8 regularization and backtracking
fractions 1, 1/2, 1/4, 1/8, 1/16, 1/32. Retain the original 0.1 m/s linear,
0.5 rad/s other-body and 0.4 rad/s foot-angular guards. Stop at the 0.2 mm/s
deadband, retaining infeasibility and every rejected query explicitly.

Record exact source/executable/snapshot provenance and all trial results.
This copied-world comparison is not sustained standing, production adoption
or interactive-runtime acceptance. Production source stays at H51.

## Native inspection qualification

The initial whole-snapshot byte-roundtrip guard stopped the experiment before
any copied queries. The [5-artifact preflight archive](checkpoints/2026-09-27/h52-native-inspector-preflight/manifest.json)
preserves that failure. Section-level inspection isolates all 25 differing
bytes to ordering within the serialized `to_wake_up` HashSet. All 25 members
are identical, as are island-join memberships, the 10,853-byte preceding
joint payload (including all motor fields), and every other world section.
This holds for all three headings. No reserialized native world is used.

The corrected reader requires exact preceding payload and other-section
bytes plus exact unordered wake/join membership. Known motor configurations
must also match independently saved JS values after f32 conversion. These
checks replace an inappropriate whole-file canonical-order assumption;
every physical, copied-state and speed guard remains unchanged.

## Result — some knee authority, residual motion remains

All baseline and H50 states replay exactly for 25 bodies, and all 597 copied
queries preserve their initial states. The eight-control search yields
maximum horizontal rates of 0.794/1.878/0.512 mm/s at 0/+π/3/−π/4;
continued six-control optimization yields 1.035/1.911/0.854 mm/s.
Both methods retain admissible speeds, but neither reaches 0.2 mm/s.

The retained final Jacobians show that the same absolute 1e-8 regularization
used for angular residuals strongly attenuates updates in the much smaller
horizontal-rate units. A dimensionless linear proposal is recorded only as
an estimate; it has not been applied. Some proposals exceed native ceilings,
so an actual capped, nonlinear evaluation is required before any conclusion.

The [16-artifact archive](checkpoints/2026-09-27/h52-knee-authority/manifest.json)
retains native receipts, all trials, exact-state checks, conditioning analysis,
executable digests and source. No production behavior changes.
