# Hip-ROM-only paired source-engine diagnostic

One deterministic opt-in experiment, using the retained original MuJoCo 2.1.3
assets. The source XML, application/default model, motor authority, friction,
controller and four-step program are unchanged. The experimental XML is a
separate local copy. This is not an anatomical-realism or robustness claim.

## Adaptation

Both hips have exactly these range changes, in source degrees:

| Axis | Original | Experimental |
| --- | --- | --- |
| x | −25 / +5 | −35 / +15 |
| z | −30 / +25 | −40 / +35 |
| y | −110 / +20 | −120 / +30 |

Source right axes are +x/+z/+y; left x and z axes are reversed, with the same
numeric ranges. Only the six range attributes change. Gears, torque caps,
stiffness, damping, geometry, mass, contact settings and all other joint limits
are preserved. The +10° per-bound expansion is a controlled engineering probe,
not a proposed human anatomical limit.

## Paired measured results

Each arm starts from the exact same recorded qpos/qvel. Contacts and native
solver state are rebuilt independently for its own model. The unchanged planner
uses each model's compiled limits, including in its scratch worlds. Distances
30 / 60 / 90 / 45 mm, timing scale 2, support-admission predicates and final
4,000-sample quiet gate are identical. No root assistance or inter-step state
installation is used.

| Metric | Original | Wider hips |
| --- | --- | --- |
| Achieved steps, mm | 31.906 / 62.058 / 91.844 / 46.803 | 31.844 / 61.885 / 91.829 / 46.914 |
| Final quiet hold | 5.000 s | 5.000 s |
| Actual native integrations | 49,857 | 49,846 |
| Simulated duration | 62.32125 s | 62.30750 s |
| Global loaded material-point slip, left/right | 8.611 / 5.965 mm | 7.511 / 5.748 mm |
| Minimum measured COM/capture support margin | 2.542 mm | 9.720 mm |
| Peak actual normalized motor effort | 95.000% | 93.615% |
| Minimum walking joint reserve | 5.331 mrad | 7.688 mrad |
| Minimum final quiet joint reserve | 9.910 mrad | 9.780 mrad |
| Absolute joint mechanical work proxy | 235.384 J | 205.724 J |

The work proxy is the integral of sum(abs(actual motor torque × actual joint
velocity)); it is not electrical consumption or metabolic effort. Some individual
hip RMS efforts rise while others fall. Left/right loaded slip falls 12.8% / 3.6%
and the work proxy falls 12.6% in this single paired test.

Additional range is genuinely used: right hip-x reaches +10.257°, right hip-z
+29.972°, and both hip-y joints reach about +29.56°. At least one hip is outside
its *old* limits on 48,398 recorded samples, while the wider model's walking
limits remain respected. The minimum hip reserve to the wider bounds is 0.440°.

Both runs have zero flight samples, nonfoot load, self penetration, engine
warnings or missing/self-eligible joint ground rays. The existing startup caveat
remains: the inherited ankle entry is about 0.52° outside its soft limit, with
negative recorded margins through 0.06375 s. Neither run passes a strict
all-startup-sample joint-limit gate. Walking and final quiet gates pass.

The new original-model arm reproduces the historical four-step final state and
all four step records exactly. Total new physics is 99,703 integrations /
124.62875 aggregate simulated seconds. Preparation, tests, saved-data audits and
rendering add zero integrations. The experimental model is not promoted.

## Checks and reproduction

Five pure recipe tests pass. Native zero-integration preflight verifies identical
q/qvel, geometry, mass, gears and friction; exactly six changed compiled limits;
unchanged other joint limits; cross-model full-cache rejection; and byte-exact
native-state preservation during planning/geometry queries. Final-code preflight
rebuilds byte-identical models and cold caches to those physically executed.
Two existing planner purity/timing tests also pass.

Using the retained local source assets and the same built helper:

```sh
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 .venv/mvp/bin/python \
 scripts/getup-step/source_hip_mobility.py \
 --base "$SOURCE_SESSION" --helper "$HELPER_SO" \
 --reference-folder "$MEASURED_FOUR_STEP_GENERATION" --output "$NEW_PAIR" --execute
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 .venv/mvp/bin/python \
 scripts/getup-step/audit_source_hip_mobility.py "$NEW_PAIR" \
 --historical-reference "$MEASURED_FOUR_STEP_GENERATION"
```

Omitting `--execute` runs preparation and zero-integration preflight only.
The original assets, XML copies, full journals, weights and native caches remain
local and are not redistributed. Compact evidence accompanies this report in
`../evidence/getup-source-hip-mobility-2026-10-18/`. The local measured-pose video
uses pure FK, 624 distinct recorded poses at approximately 10 fps and real-time
62.3075 s duration, with no planned pose, interpolation, physics or terminal tail.
