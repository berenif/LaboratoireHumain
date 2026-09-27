# H66 — continuous guarded coupled-speed fallback

H63 fails all three standing screens. H65 corrects each first failed state with
the same 34 capped motors by expanding an improving copied-world probe direction.
Extract that search into a shared diagnostic helper and verify its complete
query sequence against all three retained H65 local comparisons before launch.

Preserve H63's initialization foot references, position-return task, pelvis task,
distal angular stage, arm stage, guards and all physical settings. Add the coupled
search only when the final H63 command still exceeds an independent physical
speed bound (0.1 m/s or 0.5 rad/s). Its intermediate trials stay on copies.
Apply a new command only if it meets all original physical bounds and the
existing 0.4 rad/s foot guard. Prefer the search's retained best when guarded;
otherwise use its lowest-objective guarded trial. If none exists, retain the
original command and explicitly record the miss. No predicted state transfers
into the live world.

The fallback keeps H65's eight iterations, 34 axes, normalized velocity-excess
objective, ±1 Nm probes, native-cap trust scaling, six backtracking fractions,
seven probe-expansion factors and duplicate clipping checks. Record triggers,
queries, guarded corrections and reserve misses separately. Retain all native
snapshots, trial metrics and digests, chosen/live 25-body comparisons, support,
state, steps, drift, excursions, vertical ranges and runtime.

Require exact H65 local replay and an unchanged 30-step H63-mode compatibility
replay. Then evaluate the original three headings for 2+10 seconds from native
initialization. No 2+30-second evaluation is justified unless every short screen
passes the unchanged criteria. Production remains unchanged until its separate
integration and complete acceptance gates pass.

## Preflight and launch

The [six-artifact helper replay](checkpoints/2026-09-27/h66-coupled-helper-replay/manifest.json)
is verified. All 947 H65 queries match in order, motor bias, objective, guard
and all 25 physical body states. Only JSON zero signs in calculated residuals
are qualified and recorded; physical fields remain exact. Targeted lint passes.
The thirty-step H63-mode compatibility replay preserves complete physical
responses, native snapshots, selected biases, body hashes and query counts.

The [launch checkpoint](checkpoints/2026-09-27/h66-launch-checkpoint.json) records
three running 720-step screens, their handles and the source freeze. All 67
production/package files match H51 and the five unchanged diagnostic math tests
are reused after verifying their source hashes. At +π/3 the first original
speed violation is during initialization at tick 1, so the fallback is eligible
immediately; it is not delayed to the two-second observation boundary.

The [verified eleven-artifact preflight archive](checkpoints/2026-09-27/h66-preflight/manifest.json)
retains compatibility data and all three first native steps. The +π/3 startup
fallback reaches 0.068970 m/s and 0.398226 rad/s and matches the live world for
all 25 bodies. The other two first steps require no coupled correction. This
initial success does not establish the outcome of the still-running screens.

## Completed short screens — all three pass

All three 2+10-second screens pass the unchanged drift, speed, support, state
and step criteria. Endpoint foot drift is 7.455 / 8.076 / 5.717 mm; pelvis drift
is 10.366 / 6.255 / 4.835 mm at 0, +π/3 and −π/4 respectively. All remain
upright with two planted feet and zero steps. No observed speed exceeds 0.1 m/s
or 0.5 rad/s. All 2,160 chosen 25-body predictions match their live steps.

The verifier independently recomputes every velocity, endpoint drift, maximum
excursion and vertical range. It also checks native parameters, H63's unchanged
prefixes and the first integrated H65 corrections at 0 and −π/4. Complete trial
data are retained. This permits a longer diagnostic evaluation; it does not
establish production standing or interactive runtime.

The ten coupled fallbacks all reach their reserve targets; none misses its
guard. Native inspection verifies 714 motor-axis configurations with three
recorded JSON signed-zero differences. Maximum joint separation is below
0.049 mm, non-excluded self-penetration is zero and floor penetration is below
4.014 mm. Average preview time is 1.72–1.88 seconds per physical step under the
concurrent diagnostic runs, so an interactive implementation remains unresolved.

The [complete short-screen archive](checkpoints/2026-09-27/h66-coupled-speed-preview/manifest.json)
verifies 174 compressed parts reconstructing 59 originals. All 489,616 copied
queries, complete histories, native inspections and verifier source are retained.

## Declared long evaluation

Run the same mode, source, headings, initialization, physical settings and
control parameters for 1,920 native steps: two seconds settling followed by
thirty seconds observation. Extend the diagnostic pulse through step 1,920.
Require the first 720 native snapshots, chosen body hashes, motor biases,
query counts and complete physical responses to reproduce each short screen.
Retain every trial and failure. Apply the original limits independently over
the entire observation interval; do not extrapolate from the short success.

The [long-evaluation launch checkpoint](checkpoints/2026-09-27/h66-long-launch-checkpoint.json)
records the three active handles and their unchanged source. The full verifier
also checks the complete 720-step short prefix before accepting a long result.

The still-running heading-0 history records a first speed failure at tick 978.
Its complete result remains pending; this observed failure already prevents a
long-screen pass. [H71](standing-h71-contract.md) preserves that completed row,
its native state and the hashed input prefix. Local replay confirms 58 recorded
candidates within the physical limits, which the current helper rejects for
missing its stricter 0.4 rad/s foot margin. The unchanged original histories
continue to completion so later behavior is retained.
