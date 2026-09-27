# H61 — arm-speed correction within continuous native actuation

H60 verifies local authority at two different hand-speed failures using the
twenty original arm motor axes. Add its arm-velocity task to H59's diagnostic
preview, after the six-distal-axis angular stage and before the fourteen-leg-axis
foot/pelvis task. Preserve the existing bias from earlier stages; each stage
only adds bias to its declared axes. Begin each tick at zero additive bias.

Construct the added axes from the actual current standing commands, including
the existing world-frame damping contribution, using the unchanged native motor
adapter's targets/gains/feedforward conversion and original caps. The arm stage
uses sixty measured velocity components from ten arm segments, normalized by
0.1 m/s and 0.5 rad/s. Trigger above 0.08 m/s or 0.4 rad/s; use H60's ±1 Nm
probes, cap-normalized full updates, trust scaling, six backtracking fractions,
improving feasible-probe retention and eight-iteration limit. Require original
all-body speed guards and foot angular ≤0.4 rad/s for an accepted arm candidate.

The horizontal task continues to alter only the fourteen leg axes. It retains
all-body guards, so it cannot introduce an accepted arm-speed failure. Record
both intermediate arm corrections and final reserve misses; the final independent
physical speed limits remain 0.1 m/s and 0.5 rad/s. A baseline with simultaneous
uncorrected leg/arm failures may still be infeasible; report it without relaxing
any live acceptance bound.

Start at initialization under unchanged H59 references and allocation. Preserve
the solver, timestep, collisions, anatomical limits, native caps and continuous
Rapier ownership. Match every chosen 25-body prediction exactly to the live step;
never copy a simulated state into the live world. Retain all trial metrics,
motor configurations, bias vectors, chosen/live fields and native checkpoint/
failure snapshots. Keep base motor telemetry distinct from the additive bias.

Require all three original headings to pass the unchanged 2+10 s short screen
before 2+30 s evaluation. Report excursions, vertical ranges, structural/contact
measures, failures, reserve misses and runtime. Local arm authority does not
establish long-run stability, performance, transfer, recovery or production
acceptance.

## Launch and initial verification

The three 720-step runs are in progress with source frozen. Targeted diagnostic
lint passes. The [initial native-verification archive](checkpoints/2026-09-27/h61-initial-native-verification/manifest.json)
contains thirteen verified artifacts: all three original tick-one snapshots,
full first-step records, native motor receipts, verifier source and controller
source. All 102 motor-axis configurations match native f32 values and original
profile caps. Three JSON signed-zero differences are recorded explicitly.
All seventy-five first-step body predictions match their live counterparts.

The previous H59 mode also reproduces its thirty-step heading-zero native
snapshots, physical responses and chosen predictions exactly. This verifies
compatibility; it is not a completed H61 standing result. Current processes,
source and validation scope are in the [launch checkpoint](checkpoints/2026-09-27/h61-launch-checkpoint.json).

## Completed result — all speeds pass; +π/3 drift rejects the screen

All three runs finish upright with two planted feet, zero steps and no observed
support loss. Every post-settle speed stays within the unchanged bounds. Peak
linear speeds are 0.099952/0.099899/0.099880 m/s; angular speeds are
0.497639/0.457446/0.499632 rad/s. Endpoint foot drift is
6.664/10.095/9.319 mm and pelvis drift 2.898/10.680/14.726 mm. The +π/3 foot
drift exceeds 10 mm, so the complete short gate fails and no long run follows.

Every one of 2,160 live steps matches its chosen 25-body prediction. All 415,092
copied cases preserve the native caps; inspection verifies 714 motor-axis
configurations across all checkpoints. The [complete archive](checkpoints/2026-09-27/h61-arm-preview/manifest.json)
contains 150 verified gzip parts reconstructing seventy complete originals,
including every trial, full live results, native states/receipts and source.
The verifier reads logs progressively and checks both each part and the full
reconstructed original's SHA-256. No observations are discarded.

The [archive-tool validation](checkpoints/2026-09-27/preview-chunk-archive-validation/manifest.json)
checks exact byte reconstruction, a UTF-8 character crossing a part boundary,
empty files, legacy per-part verification and rejection of both corrupted parts
and altered whole-file hashes. These six checks concern evidence integrity,
not physics acceptance. Production remains identical to H51.
