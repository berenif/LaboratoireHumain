# Five-reviewed gait correction implementation

Status: stopped on bounded numerical-solver failure. Final acceptance is unmet. No stable preset,
original-contact qualification, main merge or deployment is changed. Detailed
hashes, numeric checkpoints and failed attempts remain in the existing
`docs/evidence/source-reviewed-walk-stage1-checkpoint.json`.

## Five actual reviews incorporated

1. Calibrate forward-positive thorax/pelvis axes; separate visual preferences
   from anatomical limits. Measure knee motion and respect rigid two-capsule
   feet. Preserve arm opposition in the instantaneous thorax frame.
2. Prepare one immutable measured-arm/reference command for actual and
   prospective-unload QPs. Admit changed tasks using current support and native
   effort/ankle reserve, rather than naive angle clamps.
3. Audit every native sample, global slip and same-world continuity. Distinguish
   own-foot advances from COM/body travel; compare matching loaded-landings and
   actual-speed front/profile pixels.
4. Implement shared-reference parity, continuous COM/load progression, coupled
   posture, coordinated knee/foot motion, then cadence. Isolate causal factors;
   do not conceal a partial six-step result by stacking unqualified changes.
5. Preserve measured midpoint+35 mm loaded-settle, 80-sample readiness, distinct
   unload/airborne gates, idempotent history and a new changed-reference profile.

## Unchanged physical and arm contracts

- Explicit impratio=8, engine friction=1, original geometry/strength; 800 Hz
  physics, 200 Hz commands, motor bounds ±0.95, loaded slip ≤10 mm per foot.
- Original joints, collisions, ground rays, contact/support/capture constraints
  and phase deadlines. No actual state installation after initialization, root
  assistance, slip reset, hidden world reset or playback timewarp.
- Unload: swing <0.5 N, stance ≥0.8 BW for 80 native samples. Advance: whole-foot
  clearance ≥25 mm, swing <3 N, stance ≥0.8 BW for 80 samples.
- Robust support: ≥100 ms/21 causal commands, pressure/clearance agreement ≥0.8,
  three unique actual survivors and 3 mm reserved margin. Correction remains
  ≤12 mm, ≤0.025 m/s, ≤0.15 m/s² and ≥1 s. Changed tasks reacquire admission.
- Loaded settle: actual received-feet midpoint+35 mm heading, original height,
  1.52 s planned phase and 80-sample measured readiness; never silently switch
  handoff protocols. Final quiet requires 4,000 consecutive native samples.
- Delayed measured ipsilateral thigh drives opposed near-body arms at gain −1.
  Keep origins/history, ≤0.5° max error, excursion ratio ±5%, correlation ≤−0.98,
  exact target/FK identity, no clipping and original reserved map checks.
- Total COM caps: 0.19260552331163003 m/s and 0.3901818436690813 m/s².
- The inherited startup exception ends at native34 for the negative ankle and
  native51 for all joints; it is not a new gait relaxation.

Forward bust 3–5°, pelvis ≤15°, roll ≤5°, early knee gain ≥10° and median interval
≤5 s are soft preferences, not anatomical limits. The reference video contains
four ≥180 mm own-foot advances, about 6.66 s between loaded-landings, under its
older explicit contact model. Ten foot advances do not imply 1.8 m body travel.

## Measured history and approved architecture

Shared immutable contexts now preserve subclass tasks in both QP consumers and
prepare measured arms/contact seek once per command. Continuous COM and earlier
state-valid selection reduce alternating intervals to 6.870 s, still slower
than the video. A seven-step attempt stops in the seventh landing after six
completed advances at global slip 10.00760/8.61442 mm. Stage-one drift tuning is
closed; rejected tangential damping/native-acceleration fixes are not repeated.

The approved ordering revision permits isolated visual pilots before a full
seven-/ten-step pass. The historical v3 posture pilot passes one 181.484 mm
loaded advance and 4,000 quiet samples, with slip 1.418/1.180 mm. Mid-advance
pelvis pitch improves −30.53°→−17.37°, but torso tilt stays about 9.3°, landing is
1.85 s slower and quiet pelvis remains −18.24°. Its return explicitly requests
an incompatible combination of root, lumbar and knee references. It is a
physical-only historical checkpoint, not final visual/cadence success.

The resulting revision gives ONE stance-realized whole-body path ownership of
lower joints, feet, COM, thorax and pelvis jets. Nine lumbar/stance-leg coordinates
per phase derive the root and exact rigid swing-foot continuation; the root is
never applied to the actual world. Original clocks are 0.8/1.2/1.0 s, advance
command 182 mm and lift 40 mm. Declared C2 compatibility slack preserves the
actual prepared command at activation. Predicted arm geometry is separate from
the exact already-prepared measured arm jets used by every actual command.

After actual landing, a separate causal same-foot endpoint retains both received
feet and the original measured COM target. Its geometry, remaining original
settle deadline and incoming C2 jets remain bound; endpoint feasibility never
substitutes for return-path admission. Counts two/ten stay blocked until C2
cross-step ownership, persistent arm history and alternating native pilots are
qualified. Prepared cross-step utilities alone do not remove that guard.

## Recovery and current native-feasibility stage

The shared artifact directory disappeared around 03:09 UTC on 2026-10-19.
Historical raw journals/certificates and iteration-33 coordinates were lost;
compact reports and source/native assets survive. No missing proof is recreated
or reused to qualify a new run. Contact XML rebuild bytes match; the new inspector
is explicitly regenerated. One unchanged prefix reproduces context, feet,
support and arm-map hashes at native 3,200, but full native payload bytes differ.
Only the accepted 27-coordinate vector 0959186d… is recovered exactly.

One explicitly fresh current-input problem uses that anchor, original limits
and all nine actual support faces over all phases: 1,102 geometric rows. It
converges in four iterations (216.08 s) and passes 123 strict samples, 603 actual
200 Hz grid samples and 235 numerical-extrema samples. Planned knee correction
19.23° and COM progress 37.93 mm remain mathematical references only.

For exact activation binding, one cold-prefix world is held at native 3,200
while offline planning/certification runs. Native bytes, time and counter do not
advance; only that same world may resume after current-input preflight and a
fresh 21-command window. Record compute wall time separately; no real-time
computation claim. Failure stops before applying a changed reference.

That fresh candidate is rejected by current native static admission at lift25%:
right_hip_x requires 0.933357 versus the strict 0.925 reserve. A targeted negative
proof confirms zero changed whole-body commands and unchanged actual bytes at
3,200 samples. Eleven of thirteen diagnostic poses exceed the original 0.95
static motor bound. Exact native M*qdd+b shows inertia helps some moving samples
and hurts others; stationary endpoints need 1.12389/1.19972/1.30857. Lift25% lacks
a same-three-contact moving wrench even with diagnostic unbounded motors. These
are local path failures, not a global physical-impossibility proof.

The approved native stage therefore adds effort/contact-wrench constraints to
the same whole-pose path, with soft body/knee preferences yielding to feasibility.
Original moving cones use μ=0.6 and Fn≥0 with NO upper bound; the earlier moving
body-weight cap is corrected and all thirteen results are unchanged. At truly
static poses Fn≤BW is redundant under equilibrium and nonnegative normals.

First restore stationary endpoints, then a bounded complete moving-path solve.
The original strict motor gate remains <0.925; a fixed 1e−7 tighter numerical
planning interior is separately declared. Experimental lumbar ±5° is not a
physical limit, but no observed active bound yet justifies widening it.

The first 54-variable restoration stops after three major iterations/9.63 s on
a Newton-invalid trial. Its exact retained 6bb755… pose passes original endpoint
geometry and lowers native requirements to 0.92499896/0.92498865/0.92500085. The
last still fails reserve. An exact rigid-foot wrench-cone elimination removes
force auxiliaries: all actual 3D rays/facets, root-zero/rank conditions and native
point-Jacobian factorization are verified. Independent nine-force/full27-equation
LP checks remain. Reduced native motors agree with original static LP to 1.45e−15.
Its first local-box SLSQP trial is nevertheless Newton-invalid (4.69 s).

Genuine-domain SQP then rejects undefined requested trials and backtracks without
projection or fabricated constraints/derivatives. Its anchor-directed mode reaches
180 s after 72 accepted points / 73 local QPs, reducing violation only to 7.90e−7;
final checks expire, so admission is false. The next causal substep is explicitly
minimum-step feasibility restoration from that exact retained 545543… point,
using the ORIGINAL fixed 6bb-centered local bounds. It does not optimize or claim
KKT/minimum-anchor optimality; the 095918 anchor remains the later preference.
There is no radius, seed, model or weight scan. Every local optimizer call is
reported separately from the outer trajectory solver.

Minimum-step restoration passes all three original endpoint geometry, map, gap,
independent native-primal and original static-LP gates in 1.47 s / two local QPs.
The largest coordinate change is 3.24e−7 rad; strict motor requirements are
0.92499932 / 0.92499345 / 0.92499990. Candidate 2bbdb51a… is endpoint-feasible
only. A zero-optimizer full moving profile evaluates all 2,025 constraints and
the complete 27-coordinate Jacobian in 107.54 s. Moving COM acceleration fails
by 0.04754 m/s², motor reserve by 0.03428, and capture support by 0.106 mm.
All 218 scalar searches remain present. Full native restoration and actual
physical validation remain pending; endpoint proof never substitutes for them.
The first combined native attempt rejects its initial local QP after 44 local
iterations, before any trajectory step. Exact held world counter/time/full
payload remain unchanged and all 800 commands are original prefix commands.
A read-only linear-feasibility diagnosis distinguishes numerical failure from
experimental design-bound restrictions before another trajectory attempt.

The diagnosis proves the original local linear problem infeasible. Removing
only the experimental lumbar ±5° shape window makes its otherwise-identical
linear problem solver-feasible; all three lift-knot lumbar upper bounds carry
constraint pressure. The tiny raw LP residual is retained, so this is no native
or nonlinear proof. A single conditioned minimum-step QP then solves in two
iterations / 1.24 s with 2.4e−14 unscaled residual and no bound violation.

Explicit physical-lumbar v2 removes that design window in planning, runtime
validation and preflight. Original physical joint limits plus .01-rad interior,
all other guards and every strict/current native gate remain. The old profile
cannot qualify it. One positively row-normalized linear-feasibility initializer
and convex QP still supply only a direction: every actual requested trajectory
trial must pass genuine-domain backtracking and full final admission. Failed
numerical restorations now skip the full final certificate and retain a clear
unattempted result; passing candidates still require all 123 + 603 strict samples.

The complete analytic planner sampler matches every strict field, constraint
and validator bitwise at all thirteen actual nodes and measures 3.41× faster.
Exact per-body positional memoization additionally preserves every strict
field, constraint and check bitwise at all thirteen actual nodes; measured
strict-to-optimized aggregate speedup is 3.97×. Original Newton, scalar searches
and strict final/runtime sampling remain unchanged. A separate
finite planned-jet adapter checks two fresh scratches, native M*qdd+b/full27
witnesses through moving samples, additional original static reserve at entry
and stationary endpoints, and unchanged actual-world bytes. The feature profile
`reviewed-native-planned-jets-wholebody-one-step-v1` remains unqualified. A full
moving producer, exact current-input integration, passing native pilot, new-profile
independent audit and actual-speed visual review are still required.

## Terminal result and stop

Physical-lumbar attempt 003 reaches the 300.04 s compute cap after three genuine
accepted path updates, four LPs and four local QPs. Remaining maximum planner
inequality violation is 0.0129714. All eight rejected full/half trials fail the
fixed Newton branch at the land endpoint, so global steps accept only α=.25.
The closest 27-coordinate candidate and exact hashes are preserved in the
existing evidence JSON. This is numerical nonconvergence, not a proof that no
physical trajectory exists.

The same actual world remains at native 3,200 / 4 s with identical full payload,
800 original-prefix commands, zero changed whole-body commands and no reset or
pose write. Targeted safe-rejection proof passes; no full native qualification
was run. There is no new changed-motion video. Further optimizer/native variants
are stopped. Endpoint-freeze/block-backtracking is an untested hypothesis only.

Historical reproduction uses source 92742f0c…, the retained 003 input/table,
source-freeze manifest, original ABI and two ignored staging wrappers listed
with hashes/arguments in the evidence JSON. The retained attempt is exclusive;
raw witnesses/binaries are not committed. Regeneration would require explicitly
new provenance, not a claim to recover missing original bytes.

## Final acceptance

Require ten alternating own-foot advances ≥180 mm in one uninterrupted world,
then 4,000 quiet native samples, with every unchanged physical, arm, history and
no-assistance gate independently passing. Also require faster loaded-landings
and improved complete actual-speed front/profile pixels: less waiting/lateral
sway, earlier knee flexion with controlled extension, coordinated pelvis and
slight forward bust, and visible near-body opposed arms. Report both sides,
phase intervals, foot separation, COM/body travel and missed soft preferences.
A slower, physical-only or visually partial result is not final success.
