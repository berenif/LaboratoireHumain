# Local arm-pull first-step experiment (24 October 2026)

## Coordinated stance geometry001 — pure milestone (25 October)

This is one geometric candidate from exact001/8195 at10.24375s, not a
continuation or a physical step. The native233 state, qvel and existing
controller are unchanged. No dynamic forward, inverse dynamics, integration or
forecast was called. Existing native-contact001 remains closed with its verdict.

`pull-stance-geometry.mjs` reuses the geometric construction of
`getup-step/plan_source_stance.py` and the current `RelativeFootIK`/`jointCurve`.
It does not import the old World, Clarabel, MuJoCo2.1.3, the force LP or the
source torque-reserve objective. Before solving, the protocol declares one
candidate: COM35mm forward of the measured right-foot origin,15mm below the
measured COM; left-foot XY unchanged, horizontal heading and whole-capsule
clearance0.1mm. The retained frame is the measured frame. Unlike the source
planner, no1.15mm foot compression or stance flattening is introduced. These
are reference construction choices, not controller gains or physical writes.

Thirteen configurations share one Hermite curve and a1.8s quintic progress
clock. All21 articulation targets and the COM/torso/feet tasks come from its
finite FK. The conditional base pose is exported separately: it is the rigid
alignment needed to place the predicted retained foot on the measured frame,
never a root actuator or an acquired physical motion. Recomputing FK of each
conditional generalized pose reproduces all exported world tasks within
2.89e-15. The initial measured qpos remains the first geometric knot.

Results:

- Articulation-induced COM displacement relative to the retained foot:
  −18.470/−89.848/−12.761mm. Conditional world COM displacement:
  −18.704/−89.835/−12.510mm. Target horizontal error is below22µm per axis;
  final vertical error is+2.490mm. Obtained sole-hull inset is9.932mm.
- A continuous chord bound uses conservative serial-chain second derivatives
  on552 intervals, beyond the13 IK knots: maximum position residual bound
  0.4252mm, secondary height bound7.5542mm, orientation bound0.003074rad.
  It certifies this geometric curve, not future contacts or dynamic viability.
- Both inherited knee excesses decrease monotonically; exact cubic extrema
  admit no new ROM excess. After the first knot, minimum margin is0: the
  right hip-x and both knees reach a limit. Source20mrad joint reserve is
  therefore NOT established. Raw12x15 task Jacobians have rank12 at all13
  knots, minimum singular value0.003098 in their declared mixed task units;
  this is not a uniform condition-number or dynamic feasibility certificate.
- Joint speed/acceleration bounds are1.4544rad/s and35.101rad/s². Actual
  reference maxima sampled are0.9494rad/s and27.950rad/s². They are reference
  derivatives, not new physical caps. The curve is C1; finite acceleration
  jumps remain at Hermite knots. Spatial derivatives of this same curve were
  checked with0.2/0.1ms reference-time differences away from knots: maximum
  changes9.61e-8 in velocity and4.70e-7 in acceleration units. No simulation
  clock advances during these checks.

Important limitations before any new dynamic work:

- With the physical root held at its measured pose, the same joint shape moves
  COM25.323mm LEFT and the right foot115.232mm LEFT. Only the relative COM
  shift is independent of common root motion. The separate conditional
  reference requires up to117.802mm of rigid alignment translation; zero of
  that is credited as measured or commanded root motion.
- The reference starts with zero desired velocity while measured joint dq
  reaches0.07623rad/s. Physical dq is preserved. A command/velocity handoff is
  not certified, and this curve cannot simply replace the motor input.
- The measured geometry already compresses the ground: right/left minima
  −0.7251/−0.7478mm. Final left clearance is+0.0800mm. Continuous conservative
  lower bounds are−0.9373/−0.9226mm; no zero-penetration or loaded-contact claim
  follows from this reference. Sole hull is geometric, not a measured COP.
- No torque, reaction, friction, unloading, foot lift, real-time control or
  walking success is qualified. The missing source torque-reserve term is a
  substantive next dynamic question, especially at the active ROM limits.

Local evidence: `web/proofs/pull-stance-geometry-001/` contains the declared
protocol, targets, finite reference, result, independent curve audit, spatial
jets and explicitly conditional generalized reference. `reference.json`
SHA256 is8cbd27890be66a1d025626abf791ec849ed538b69c1b17587ae54093a154ad44.
Eight rejection/negative counterexamples plus a positive common-rigid-transform
invariance check pass (the original audit field groups these as nine checks;
the jets receipt clarifies the label). Finite-difference derivative checks and
full native233 immutability pass. Total1235 kinematics,1235 COM-position and128
Jacobian calls; ZERO dynamic forwards/ticks/forecasts. MuJoCo remains229033
confirmed PLUS unknown history; Rapier45735, main and private site9 unchanged.

The new scripts use only the installed WASM and NumPy. No additional SDK,
weights, engine, download or physics replay. Run in a NEW evidence directory:
`run-pull-stance-geometry.mjs`, then `audit-pull-stance-geometry.mjs`,
`verify-pull-stance-jets.mjs` and `export-pull-stance-reference.mjs` with Node,
each given that directory. They preserve existing files and refuse overwrites.

## Native-contact continuation001 — closed after16 ticks (25 October)

Source9133949 plus the file hashes in started.json. This iteration repairs the
QR dual blocker by retaining the minimum-ratio INDEX before interpolation,
removing that exact face even if interpolation leaves a positive rounding
residue. Exact ties use original row order; zero-length pivots remove a face.
Repeated numerical states and the64-change limit stop the solve. Failure
outputs now reconstruct x from current z and retain the full last dual segment.
No multiplier tolerance, weight, gain, physical cap or model option changed.

32 existing direct-QR cases,6 blocker/output cases and32 native-law algebra
cases pass, without engine calls. Exactly ONE generic saved solve each of003's
entry and terminal QPs passes:8/22 pivots, costs.05457566716637501/10.05732267709922,
backward errors3.4234e-17/1.8736e-16. Both original-H certificates pass. The
terminal stagnation is repaired without fixing its active set in advance.

The new module keeps21 motor decisions. It uses present native pyramid edges
AND joint-limit rows, remapped from island states. At fixed q,v and grab,
force=max(0,-D*(J*a-aref)); on the current branch K=M+J_A'*D_A*J_A.
All27 accelerations, including the free base, follow this law and motor torques.
Existing task, force-preference and regularization costs are substituted through
that affine map; no free auxiliary reactions remain. Only controls and the
explicit user grab are applied. No optimized reaction is injected. Topology and
branch changes are not treated as physical faults. The error estimate uses
strong monotonicity in the mass metric: ||a-a*||_M <= sqrt(F(a)'M^-1F(a)),
propagated to tasks, force bounds and damped Euler velocity. It permits zero
edges. This is an analytical numerical estimate with floating-point checks,
not an interval-arithmetic proof or guarantee of future contact persistence.

Prospective bound:4 pure forwards, then at most2432 actual800Hz ticks,1828 NEW
explicit forwards total,200Hz commands,64 QR pivots per solve,0forecasts,2MB.
Each command permits centre plus at most TWO native candidates. A first local
model inconsistency allows exactly one rebuild from its candidate's native
branch; the second inconsistency stops this approach. No new blend, phase reset,
horizon compression or ref capture is introduced. Native objective cost must
improve over the same-state/same-grab centre beyond its estimated uncertainty.
The original reference start10.2625, unloading deadline12.54375 and run end
13.5625 are preserved. The right stance/left swing mapping remains bodies7/10.

The ONE invocation restored003's exact233 native values at8418/t10.5225 and
complete grab/bridge/material-point memory. The pending grab matched the saved
audit exactly. Reproduction of the old unblended8210 response has qacc error0;
independent RNEA/mass error5.6843e-14. Pure forwards are exactly4: reproduction,
centre8418 and two candidates. The second candidate qualifies; native objective
cost6003.97063 ->20.7763491. Its torso-world-Y acceleration is predicted
-.6722739632324 and native-.6722739632319rad/s², removing the previous opposite
sign for this command. This cost comparison is instantaneous, not stability.

Exactly16 new ticks20ms were integrated,4 command blocks, then the fifth
command point8434/t10.5425 stopped after both local models remained inconsistent.
All10 QR solves pass (4..7 pivots); no physical guard fires. The terminal second
candidate's assumed active left-foot pyramid row16 predicts edge force
-.647147577N while the full native law gives0. This is substantially above
roundoff. The actual native torso-Y acceleration remains close to its prediction
(-.891723954 vs-.890750675rad/s²), and its cost30.9211 is below centre309.6343,
but those facts do NOT certify the affine contact-law model. The mass-based
model-error allowance is too conservative for qualification across this branch.
Both terminal candidates are retained and rejected; neither was applied. No
third candidate, new threshold, tuning, retry or extra integration follows.
This refusal is NOT a physical impossibility or a measured support loss.

Measured initial -> final:
- COM displacement(-.40646,-.52271,-2.27779)mm; lateral movement is toward right.
- COM velocity(-18.58,-12.92,-105.99) ->(-17.44,-43.01,-112.25)mm/s.
- Tilt8.84297 ->8.94759deg; local torso/base rateY.093906 ->.077250rad/s,
  while X rate becomes more negative. No global stability claim.
- Resolved right/left loads154.23/168.65 ->101.27/304.45N; minimum left147.98N.
- Whole-left-sole maximum-.54884mm; no unloading,8mm clearance or lift.
- Retained-body drift5.058mm from original entry; maximum applied command jump
 29.236Nm. All applied controls <= their original caps; root actuator force0.

Native forces refer to the last pre-integration resolution, while geometry is
post-integration (1.25ms apart). The two actual saved geometry frames were
inspected: full mannequin/ground and both feet down. The replay explicitly says
16ticks/20ms and numerical stop. No video, replay simulation or media transfer.

Counts:16 NEW explicit forwards (4 pure +4*3 subsequent),16 actual ticks,
10 QR solves,0forecasts, no pending call. MuJoCo229033 confirmed PLUS preserved
unknown history; Rapier45735 frozen. Explicit native time7.043ms, QR367.820ms,
full runner1539.052ms: offline only, not real-time200Hz. The existing main/site
and previous evidence/verdicts stay unchanged.

Proofs: web/proofs/pull-step-native-contact-001/. Saved-data audit adds no engine
calls. Full terminal QP, solve, both terminal comparisons, native response,
progressive commands/measures, geometry frames and exact final checkpoint are
retained. The journal preserves all applied command blocks; speculative earlier
candidates have compact response records, while full candidate vectors are
retained for the terminal pair. Final native233 restoration was verified.
Checkpoint SHA25685fbb64e6fbf27a1b843a8bdf749f0a0de7f38eda38ac27dd236fafcee0dc25a.


## Direct-QR whole-body003 protocol (25 October)

The local numerical port preserves exactly the existing135 direct cost rows
(66 tasks,45 diagonal regularizers,24 force preferences) and124 entry inequalities.
`pull-direct-qr.mjs` uses column-pivoted Householder QR, a particular active-face
solution and an orthonormal nullspace. It solves CZ directly and reconstructs
multipliers, reusing the existing add/drop/dual-segment active-set policy with
64 changes maximum. C'C and AH^-1A' are never used to solve; H remains a saved
historical diagnostic. QR rank uses dimension/machine-precision scaling and
stops on ambiguous dependency. Original residual checks are preserved. Bounded
triangular QR-factor refinement and active-equality reconstruction correct
floating-point arithmetic only, before certification; no clipping of motors
or physical-state projection occurs.

32 deterministic algebra cases pass: QR reconstruction/orthogonality, known
solutions, independent edge/vertex enumeration, units and cost-scale changes,
45-variable cases, dependency, nonfinite and cap rejection. A synthetic extreme
cost amplification remains conservatively rejected by the unchanged absolute
complementarity criterion; no tolerance is raised to make it pass. Standard
Householder v[0]=1/tau form avoids spurious dense leakage in exact coordinate
permutations. The separate COM/cone/measured-phase tests also pass.

ONE generic solve of the immutable whole-body002 saved QP (no active set supplied)
passes in8 changes. LS cost0.05457566716637501 vs independent0.054575667166374846;
max motor difference3.3711e-9Nm, backward error3.4234e-17, primal1.2519e-18,
complementarity3.0186e-18. Original-H KKT is separately checked. No native calls
or ticks occurred during this qualification. The JS solve took182.70ms, so200Hz
real-time use is NOT qualified. No gains/weights/model/caps/contact law changed.

Next authorized invocation is ONE offline observation from the same8210/native233
checkpoint with complete grab memory. Raw QP and actual blend are evaluated
separately, full C,d/constraints are saved before guards, independent RNEA and
physical checks precede integration. <=4 NEW pure forwards, <=1324 NEW forwards
TOTAL (1326 whole-body cumulative), <=2640 ticks,0 forecasts,2MB new proof.
All previous deadlines/lift/physical stops remain. No retry after result.

### Whole-body003 result: native preflight passed,208 ticks, numerical stop

The single invocation qualified raw-QP AND actual-blend commands separately at
entry, preserved the exact233 native values before integration, zero initial
command jump and actual nonzero velocities, original caps/mapping/root isolation,
posture ROM and terminal002 physical rejection. Independent RNEA/mass error was
5.3291e-15. Three pure forwards were used. No optimized contact force was applied.
The source stayed unchanged during this invocation; there was no retry.

208 actual800Hz ticks (0.260s) ran, with52 accepted command solves. At8418/
t10.52250000000077 the53rd solve stopped after13 add/drop changes with
`active-set numerical stagnation`. The failure branch found no multiplier to
drop after its dual segment; contact-3-cone--1-1 retained1.0880e-18. This points
to a finite-arithmetic blocking-face issue, NOT demonstrated physical infeasibility.
The trial multipliers/fraction were not retained, and the returned failure x is
from the preceding outer iteration; it must not be treated as an accepted new
command or a complete intermediate KKT witness. Full last C,d/constraints and
physical state survive for review. No solver or threshold was adjusted afterward.
Native contact count varied8,7,6,5,4,3 without a fixed-topology veto; final QP had
33 variables/four contacts. The last applied command and complete grab/bridge/
previous material-point memory plus pending interaction are in final.json.gz.

Measured COM displacement was(+4.921,-1.232,-4.449)mm. Final COM velocity was
(-18.580,-12.916,-105.993)mm/s versus(+25.297,-3.412,-2.002)mm/s at entry.
Tilt grew8.1172->8.8430deg; angular rateY grew0.02131->0.09391rad/s.
Final right/left loads154.23/168.65N (initial137.92/262.45N), minimum left load
132.58N, maximum whole-left-foot clearance-0.612mm. No unload<15N, no8mm
separation or lift occurred. Retained-body horizontal drift peaked4.848mm.
No physical guard triggered. The1.8s reference endpoint was NOT reached, so this
short window does not establish whether the complete transfer would work.
All actually transmitted motor fractions were<=1.0. The largest observed command
change was55.149Nm on left_hip_y at10.4675s, from+40.186 to-14.963Nm under its
120Nm cap. Initial continuity does not imply later torque continuity.

The first unblended same-command native check exposes the auxiliary-force model
limit. Predicted right/left reactions232.895/167.837N versus native216.406/190.798N;
COM X acceleration predicted-0.16816 versus native+0.09071m/s²; torso Y predicted
-0.19351 versus native+0.38694rad/s². These are distinct from desired tasks
(-0.32886m/s² COM X and-0.25572rad/s² torso Y). Exact algebraic elimination and
QR optimality therefore did NOT ensure the native task response, despite finite
bounded motors. The frozen limit reaction and free auxiliary contact-force
allocation remain approximations, not constitutive contact predictions.

Result: numerical port QUALIFIED on entry, measured first transfer response,
whole movement INCOMPLETE due numerical stop. No step, successful unload or lift
is claimed. Solving the QR problem does not qualify a usable walking controller.
Counts:208 new ticks,106 new explicit forwards (108 whole-body cumulative),
0 forecasts, no pending call. MuJoCo229017 confirmed PLUS historical unknown;
Rapier45735 frozen. Native time22.063ms, solves7594.896ms, process9079.164ms:
offline only, not real-time200Hz. Stable main/site are unchanged.

Proofs `web/proofs/pull-step-whole-body-003/`: algebraic qualification, complete
initial/latest QP, progressive journal/40Hz geometry trace, raw entry/native
responses, audit and final exact checkpoint. Corrected contact sheet inspected
across beginning/middle/end: full mannequin and ground, both feet down, small
body motion. The first render's inherited wrong20deg caption is preserved as
`initial-render-label-error.png`; `observed-contact-sheet.png` correctly labels
the numerical stop. Rendering adds no physics. No video or media upload.
Final checkpoint SHA256
`9992a1343dfbd8badaaf3861c85fe740a98e6b4ab81ed6cced408c770c3c6c1e`.

## Bounded numerical correction of whole-body001 (25 October)

This prospective continuation preserves whole-body001's zero-tick refusal.
The same QP is transformed by x=Dz, Dii=1/sqrt(Hii), including H,b and every
constraint; its Hessian factor is shared by all right-hand sides. At most three
KKT refinement corrections reuse the final active-face factor, within64 pivots.
Original-coordinate compensated residuals are retained. Stationarity is now
certified by componentwise backward error <=1e-10, with explicit zero-denominator
handling; primal/dual tolerances1e-10 and complementarity1e-8 are unchanged.
This is numerical backward accuracy, not a forward-error bound or native support
certificate. No objective weight, gain, regularizer, cap, model or friction changes.
Algebra tests use a known45-variable optimum, changes of units and objective
scale, face enumeration, rank/cap/nonfinite counterexamples and injected linear
solve errors to verify refinement and no-improvement rejection.

Before solver admission the complete initial and latest QP are stored. The raw
QP vector is evaluated natively on a scratch copy separately from the actually
applied25ms blend. Prediction/native differences are compared only for identical
commands; during blending, the actual response is compared with desired tasks
and explicitly not mislabeled as prediction error. Both native responses and
independent RNEA are required by the entry preflight. Early unload cannot skip
the complete1.8s COM reference acceleration/braking; the measured75ms load dwell
is still necessary, and time alone is never physical braking qualification.

ONE invocation from unchanged checkpoint8210/t10.2625 with233 exact native values
and complete grab memory: <=5 NEW pure forwards (six including whole-body001),
<=1325 NEW forwards total (1326 cumulative), <=2640 ticks/3.3s, no forecast/retry,
2MB new evidence. All prior measured physical/lift stops and transfer deadline
remain. This is offline at simulated200Hz command/800Hz physics, not qualified
real-time control. Stable main/site and all historical FAILs are preserved.

### Whole-body002 result: equilibrated factorization refusal, zero ticks

The single authorized invocation reconstructed the exact8210/t10.2625 native
state and complete grab memory. It consumed ONE new centre forward, no candidate
forward, no tick or forecast. The bounded runner stopped with zero pivots before
completing the Hessian factor: existing MuJoCo Cholesky minDiag1e-12 rejected the
equilibrated matrix. No retry, cutoff adjustment or physical command followed.
Raw-QP and blended-command native checks, RNEA and dynamic lift remain NOT REACHED.
The32 numerical algebra tests and separate task/cone/phase checks pass, including
manufactured45-variable KKT, units/cost scaling, injected refinement success,
no-improvement rejection, invalid ranks and cap counterexamples. An exact COM
transition test initially distinguished +0 and -0; it was corrected to mathematical
equality before the single native invocation, with no physical parameter change.

Unlike001,002 preserves H,b,124 constraint rows, elimination matrices, tasks,
external input and limit reactions before solver admission. Read-only NumPy
inspection of these saved matrices (no optimized candidate or native call) finds:
original Hessian diagonal1327.11..56816270.45; original eigenvalue estimate
9.9486e-7..1.2913e8 (condition~1.298e14); equilibrated estimate9.7148e-14..10.2384
(condition~1.054e14). Independent Cholesky succeeds, minimum squared pivot
6.8877e-13 AFTER equilibration versus1.9928e-5 before. Its relative factorization
residual is1.49e-16. The retained absolute1e-12 factor pivot floor therefore changes
the numerical rank decision under an otherwise exact congruence. These estimates
indicate serious conditioning; they do not prove accurate commands, impossible
motion or exact mathematical rank. This is a numerical-policy blocker requiring
review, not grounds to relax caps or declare a physical impossibility.

Complete proof: `web/proofs/pull-step-whole-body-002/`, with initial/latest full QP,
progressive ledger/journal, matrix diagnosis, algebra receipt, inspected identical
beginning/end mannequin images and exact native final checkpoint. Native233 and
complete grab both match input; no pending call/integration. Final checkpoint SHA256
`f8e782d2c1c61225e480a4a6c6558e447375791b954b4b49a3aa47b74cab18aa`.
QP SHA256`18456fd8d6c5428d03213787e6f0ab4d0608a87231a3226a10d3af595d43fb39`.
New count0ticks/1forward, whole-body cumulative2forwards. MuJoCo remains228809
confirmed PLUS preserved unknown history; Rapier45735 frozen. Last resolved loads
remain137.92N right/262.45N left, minimum left clearance-0.855mm, tilt8.117deg.
Native cost2.686ms, refused solve2.872ms; no real-time qualification follows.
Main/site and all historical FAILs are preserved. No new step/lift is demonstrated.

## Prospective whole-body source-task increment (25 October)

The fixed-regime finite-difference diagnostic is closed at42cf275: eight native
evaluations, zero new ticks. A cone edge changed state while its contact retained
8.88N. This is not a lost ground contact or a cap violation.

`scripts/pull-whole-body.mjs` ports the COM/torso/foot/posture tasks of
`scripts/getup-step/source_step_probe.py`, retaining source acceleration feedback
gains: COM45/13, torso40/12, feet100/20 and posture20/8. Cost weights12/4/25
(stance),20(swing)/0.2 remain declared engineering priorities. Residual units
are normalized by g for translation, g/entryCOMheight for rotation/joints,
body weight for contact forces, and gear for motor controls. Source diagonal
regularizers1e-3 motors,1e-5 accelerations,1e-8 forces and shear0.1/normal0.001
preferences remain declared, not a parameter sweep. No actuator authority,
model, friction, hip range or stable application mode changes.

All27 dynamics equations eliminate acceleration; decision variables are21
controls plus3 contact-frame force/BW coordinates per PRESENT native foot
constraint. The actual anisotropic pyramidal cone replaces the old source box.
No contact is added/removed by phase, no force is transmitted except joint
actuation and the recorded user grab. Current native joint-limit reactions are
explicit frozen inputs for one command, then recomputed; soft-contact force
realization and any change of limit reaction remain approximations, measured by
the native candidate response. Optimized forces are never injected. Native
branch changes, changing contact counts, nonmonotone angular components and
the initial100R/299L allocation are not universal safety/admission conditions.

The current support-point centroid is a convex combination inside the observed
right-foot hull (also meaningful for segment/point degeneracy). A1.8s quintic
COM task provides acceleration then braking toward that point. Current COM
height and torso/feet orientation are soft measured-state references, not
virtual root motion or rigid stance constraints. All desired posture angles
are within ROM; actual soft-limit excursions/velocities are untouched. A25ms
convex transition of total controls from the recorded command occurs BEFORE
native evaluation, never after admission. One motor owner replaces Balance;
its torques are not added to Balance PD.

One opt-in command200Hz/physics800Hz trial starts from exact233-value8210
checkpoint at10.2625s, with complete grab/bridge memory, not a new preparation.
The original transfer deadline10.24375+2.3s is retained. Left<15N/right>100N
for75ms permits a20mm whole-sole lift task. Measured sole>=8mm and left<1N
for100ms qualifies ONLY this first lift; it does not qualify a step or landing.
Stop at qualification, deadline, physical guard or2640 new ticks/3.3s.
Limits:6 pure forwards,1326 total explicit forwards,64 active-set pivots per
solve,0 forecasts,2MB proofs. There is no automatic retry or gain adjustment.
The commanded body-point grab, clock, caps,20degree tilt,25mm retained drift,
retained load<1N for25ms, fall/nonplantar and finite-state checks are preserved.

`verify-pull-whole-body-pure.mjs` checks signs, quintic jets, anisotropic cones,
absent contacts, cap/nonfinite counterexamples and measured phase gates with
zero native dynamics calls. The single-run harness additionally verifies exact
state/command entry, native actuator mapping, independent full RNEA and actual
terminal002 geometry. Complete last prediction/response and checkpoint survive
any refusal, alongside progressive compact commands, native responses and40Hz
body geometries. These changes are experimental local files, not deployed UI.

### Whole-body001 result: stopped in pure solve, zero ticks

The single invocation used ONE native forward at exact8210/t10.2625 with the
saved grab input. The45-variable problem used8 contacts and8 active-set pivots.
The existing solver returned `solved:false`: normalized primal violation
9.647838e-13, dual violation0 and complementarity9.013367e-13 pass its checks,
but absolute stationarity9.732289e-8 exceeds its unchanged1e-8 condition.
Its failure message lacks a reason for this terminal KKT check (`undefined`);
the complete retained diagnostics identify the stationarity gate precisely.

No candidate native forward, RNEA candidate check, physical tick, lift or unload
was performed. Initial algebraic sign/cone/cap/phase counterexamples passed;
complete pure/native qualification remains INCOMPLETE/FAIL. The vector's tiny
inequality residual is not native support feasibility, and the optimality
failure does not prove mechanical impossibility. Both ankle-y entries of the
unapplied vector are at the physical20Nm cap within numerical error. No
tolerance, gain, cap, model or solver-policy adjustment/retry followed.

Measured state and grab memory are exactly unchanged: COM
[.053643713,.000039258,.845361060]m, velocity
[.025296924,-.003411506,-.002002370]m/s, tilt8.117161degrees and last resolved
right/left137.9217/262.4470N. Native time remains10.2625s. The fixed historical
verdicts are unchanged. New count:0 ticks/1 explicit forward/0 forecasts;
MuJoCo228809 confirmed PLUS preserved unknown history, Rapier45735 frozen.

Cost: native3.069ms, active-set solve27.879ms, process286.487ms. This single
solve does not establish real-time200Hz operation. No walking claim follows.
Proofs are `web/proofs/pull-step-whole-body-001/`: progressive journal, raw last
native centre, complete last optimization result, ledger, audit, exact final
checkpoint and inspected two-frame replay. Both frames show the same actual
pose and full mannequin/floor, correctly labelled zero ticks. Checkpoint SHA256
`216bf912ab5a3fe9b6f44ee4b76ee8ba17b16e27f79fcb0f20024723d823321c`.

Opt-in only: `?pull-step=local` on loopback, then select Balance. Main/site and
ordinary Balance, passive and GetUp remain separate. This is not continuous
walking. The preserved old foot-cycle is not invoked.

Causal hypothesis: the former timer/angle cycle had no horizontal foot destination
and did not coordinate the joint shape with the measured support-relative COM.
A native-Jacobian reference for both tasks may transfer load and permit a first
step toward a real arm pull. Failure to unload, track the sole clearance or hold
the retained foot would refute this candidate; pure FK is not its validation.

Implementation uses existing native `mj_jacBody`, `mj_jacSubtreeCom` and Cholesky
routines, on separate MjData. The relative Jacobian removes common rigid root
motion, including support-frame rotation. No physical pose, velocity, contact,
model, motor limit or friction is changed. Fifteen existing trunk/leg hinges
coordinate COM XY, swing-foot position/orientation and torso orientation.
COM height is a secondary posture objective within30mm; it is not an obligatory
constant-height constraint. Iterative FK, active-ROM columns and monotone Hermite
interpolation verify a finite path. Quintic phase progress starts/ends smoothly;
intermediate FK nodes are not stops. Reference speed is measured, not claimed
as physical actuator authority.

The measured-shape trajectory qgeom is distinct from compliant servo reference
`qservo = nominalBalance + qgeom - qentry`. This retains Balance's existing PD
preload instead of cancelling it by capturing qactual. Servo ROM projection and
its residual are exposed. The COM feedback retains the initial equilibrium
offset and follows the same relative displacement in the measured stance frame.
The existing gains/caps and single torque owner remain. Total requests transition
from current nominal Balance over25ms, before final saturation. Ankle feedback
share follows measured contact load with50ms filtering, not a 1/1→2/0 phase jump.
This approximate preload does NOT establish that qgeom will be realized.

## Frozen single-run protocol

- One reset, Balance10s (8000 ticks); no other physical preparation.
- Existing `beginGrab/moveGrab/release` on left forearm body15 at local
  `[.09,-.09,.09]`; spring120N/m, damper12Ns/m, original180N force cap.
- Move the target WORLD+X by250mm using a smooth ramp over0.5s; keep its initial
  Y/Z. Hold that same world target, release3.8s after grab. No other force/push.
  This is a reproducible pointer-target profile through the actual grab force
  path, not an independently prescribed force or a mouse-input/mobile QA test.
- Forward traction>8N, dominant over lateral, AND capture proxy advances8mm
  from quiet COM or actual vx>0.02m/s triggers the plan. The capture proxy is
  indicative, not a viability proof. Right support/left swing for this first
  forward-only case; no alternating-step claim.
- Destination advance is clamped40–90mm from measured COM/vx/traction at trigger.
- Transfer nominal1.8s, unload≤2.3s. Left load<15N and right>100N continuously75ms
  before lift. Actual native contacts are never removed.
- Lift aims20mm whole-sole clearance; require actual minimum≥8mm and left load<1N
  for50ms. Lift deadline0.7s; transport0.6s; lower0.5s/reception deadline0.8s.
- Loaded reception>30N for50ms, return0.6s. Release before lift cancels and returns;
  in-flight release finishes landing. No new step on release.
- Success: observed unload, entire-sole clearance, ≥30mm advance DURING flight,
  loaded reception, both feet>30N, tilt<0.2rad and COM speed<0.03m/s continuously
  for the final1s after release. Sliding cannot substitute for flight advance.
- Retained foot drift>25mm, retained load<1N for25ms, nonfoot support,
  tilt>20°, existing measured fall, nonfinite/cap anomaly or phase deadline stop.
  Whole run≤16s/12800 actual ticks, forecasts0; interaction/calming≤6s.
  One attempt; no post-result tuning or physical retry.

Loads are from the last resolved substep; geometry is refreshed separately for
measurement and labelled. Joint torque actually transmitted is checked against
gear×ctrl and physical caps. Root generalized actuation and applied generalized
forces must remain zero. The only external body force is the disclosed grab.

## Validation scope

`node scripts/verify-mujoco-pull-step-pure.mjs` performs zero integrations,
including baseline command equality, relative-Jacobian finite differences,
finite path/interpolation/ROM, entry continuity, release and pre-integration
failure. It does not establish single-support or contact feasibility.

`node scripts/run-mujoco-pull-step.mjs <new-proof-directory>` is the exclusive
single-run runner; never invoke it without a separately authorized runtime.
It saves counters/poses progressively and a complete final native state. Proof
budget2MB; no media transfer or publication. Historical FAIL results remain.

## Observed single-run result — FAIL before transfer

Source40d62457dcaf22f96b3e9908f6ee7b978f944282. The only physical run stopped
at10.24375s /8195 ticks (8000 standing +195 with grab). The trigger did react to
real forward traction12.5995N and COM vx0.02057m/s, but the finite-path ROM check
rejected the measured initial shape before a transfer command was applied.

The native soft joint limits had allowed right/left knees to2.05042° /2.05698°
against the nominal2° bound. The input pose at path parameter0 is required to be
exactly the measured pose; the strict nominal-ROM assertion rejects that input.
This is a demonstrable initialization/validation mismatch, not evidence that
single support is physically impossible. The pure tests used a nominal-ROM seed
and missed this observed entry. No guard, gain, controller or cap was changed
following the result, and no retry was made.

At stop: COM+1.218mm forward since grab, torso tilt8.081°, right/left loads
195.840/203.680N; minimum whole left sole−0.748mm (native contact penetration).
No unload, lift, flight, placement or reception occurred; retained-foot drift
was0.038mm. Requested/applied motors stayed within the original caps; initial
reset landing had reached caps, while the largest at stop was60.42% of its cap.
No nonfoot contact or physical fall. The arm force remained an explicit user
grab, with no root/generalized assistance. The FK trajectory remains unvalidated.

Local proof directory `web/proofs/pull-step-001/`: progressive compressed trace,
ledger, exact native final checkpoint (pure state restore verified), diagnostic
and one inspected contact sheet. The sheet shows the full mannequin and floor,
with actual positions at0,10,10.05,10.125,10.2,10.24375s; it is labelled replay,
not a new simulation. No video/export/publication was produced.

MuJoCo confirmed total228054 PLUS the preserved prior unknown run; Rapier45735
unchanged. Main, remote branches and the private deployed site are unchanged.
Only this local experimental branch contains the candidate and its failure.

## Prospective correction and restored continuation 002

Only the entry validator changes. A measured geometric anchor may contain the
exact nominal-ROM excess already observed; the first Hermite segment must reduce
that excess monotonically, reach strict ROM at its first solved node, and never
introduce an excess elsewhere. The full cubic extrema and derivative extrema
are checked analytically on every segment, including between sampled FK checks.
The explicit anchor must equal the first configuration. All subsequent paths
remain strict. This exception does not apply to motor references, which remain
ROM-admissible from entry. Physical q/dq and native limits are never projected.
The desired shape starts with zero time derivative via the existing quintic;
this does not erase the outward physical knee velocities used by PD damping.

Exact001 checkpoint SHA256
`bdc3c9bd7167a8bc23ea583b55628d0c8d5ee878dd7a6fe3336a03bac41c743c`:
233 native integration values restored and compared exactly. The entry path
residual is0.327260mm, global speed bound1.365378rad/s (dense observed1.214929),
piecewise acceleration bound57.726326rad/s². These finite reference bounds are
not physical actuator authority or a claim of exact following. Intermediate
points remain C1 continuous; no physical velocity cap or gain is increased.
The complete entry command equals current Balance at the same restored state;
its largest difference from the preceding physically applied command is0.062205Nm
(the state has advanced since that preceding command). Physical knee damping
contributions remain−0.656052/−0.679578Nm. Six adverse ROM/nonfinite fixtures
are rejected. Future-reference command snapshots on the unchanged observed
state are finite and capped; their largest request69.770Nm is not a prediction
of future physical torque demand.

The old native snapshot does NOT include grab.previous. Continuation002 explicitly
starts a new grab point history on the same body15/local[.09,−.09,.09]. During
20 ticks (25ms), its external arm force is the convex quintic blend of the last
recorded force[12.599545569839282,0.69783056692452,0.12339019719056488]N and the
unchanged current grab law. The first newly computed point velocity uses native
Jacobian×physical qvel; later values use successive real point samples as before.
At tick0 the transmitted force is exactly the recorded force, at tick20 it is
entirely current grab. Both endpoints and the convex blend obey180N. No force is
applied to the root or another body. This is an authorized different interaction
entry, NOT a reconstruction of missing history or bitwise historical continuation.
Target timing, point, gains and caps remain unchanged; release at absolute13.8s.

`verify-pull-entry-pure.mjs` performs no integration. After it and the prior pure
checks pass, `continue-mujoco-pull-step.mjs web/proofs/pull-step-002` is authorized
once: at most4605 new native ticks, zero forecasts, absolute end16s. No new10s
preparation. Transfer entry/deadline remain10.24375/12.54375s; all physical stops
and observable step criteria above remain unchanged. The failed001 evidence is
immutable. The new checkpoint also retains full grab/bridge metadata. Proofs
are progressive and limited to2MB, including a geometry replay contact sheet.

## Observed continuation002 — FAIL, tilt stop before unloading

Tested source `5ad172d`. Exactly740 additional native ticks, zero forecasts:
10.24375→11.16875s (0.925s). Scenario counter8935; confirmed MuJoCo total228794
plus the preserved unknown historical run. Rapier45735 remains frozen. No retry.
The233-value native restoration passed; the declared25ms arm-force entry was
used. The original001 checkpoint and FAIL remain unchanged.

The corrected geometric reference was admitted and transfer executed. The
measured torso tilt grew8.081→20.009°, triggering the unchanged20° stop before
release or unloading deadline. The body's ground-fall flag remained false:
this is loss of acceptable balance, not an observed impact on the ground.
COM moved+162.225mm forward,34.861mm toward the retained right side, and29.658mm
down; final forward velocity0.387154m/s. The left load first increased to a
sampled226.739N and never reached the<15N unload criterion (minimum187.162N).
Final right/left loads179.129/187.162N. The entire left sole never cleared the
floor (best−0.748mm, final−0.986mm); retained-foot drift≤2.132mm. No flight,
placement, loaded reception, release or final calm were reached.

Both sagittal ankles saturated under the unchanged20Nm caps. Final total
requests146.020/154.493Nm were applied as20/20Nm. Saturation is visible by the
10.4s trace sample; its exact first tick/duration was not recorded. Sampled grab
force maximum23.959N, only on the forearm; no generalized/root support force.
All21 actual torques were checked every executed tick for finitude, cap and
gear×ctrl equality. This is an applied-cap PASS, not exact following of requests.

Observed cause of termination: forward lean/speed grew with feet planted while
the sagittal motor requests exceeded capacity. Hypothesis, not established
uniquely: this transfer does not unload soon enough for the imposed pull before
fixed-foot balance authority is exhausted. Neither a global inability to step
nor a gain adjustment is justified by this one result. The validator correction
is verified; mechanical step success is still absent.

`web/proofs/pull-step-002/` contains the progressive ledger and40 trace rows,
compact review, exact final native checkpoint and one inspected six-frame replay
sheet. Full body/ground remain visible from entry through increasing forward
lean; both feet remain planted. No additional simulation, video, media export,
push, deployment or controller adjustment after the result.
Final gzip SHA256:
`e00b3428d6ec07351f1f869641ff7990bf11a109086578ba9b73c04a9d30c9fd`.

## Load-feedback candidate — rejected by the pure dynamic gate, no trial003

This implementation remains opt-in (`loadFeedback:false` by default), local and
unpublished. Stable Balance and SLOW commands are unchanged. It records solved
contact positions, normals and full forces, and proposes a feedback correction
through the existing PD reference, not an external force. For ground reaction
changes δf, δτ=−Jcontactᵀδf and δq=Kp⁻¹δτ on the21 actuated axes. Root components
are reported and never applied. Existing ROM and final motor caps remain active;
the reference offset is filtered on the already declared25ms entry timescale.
There is no integral accumulator. Existing PD gains, friction and motor limits
are unchanged; this candidate is explicitly a new load-feedback law.

The approximate lateral goal uses the current COM height's natural frequency
ω=√(g/h), acceleration ω²(yGoal−y)−2ω·vy, and the measured arm force/moment.
A zero roll angular-momentum-rate target gives a desired CoP using the ground
lateral force required by that acceleration. Normal loads are projected to the
present two-foot resultant segment while reserving100N on the retained foot.
The implied acceleration is recalculated after projection and reported apart
from the unrestricted goal. This is an approximation, not a physical guarantee.

**The native pure check rejects this normal-only command formulation.** On the
exact001 q/dq at10.24375s, using recorded COM velocity and frozen recorded grab
force in both compared commands, the projected goal is ay=−0.529679m/s² (right).
The desired right/left loads are100/299.498N; full correction predicts native
94.667/304.810N, so it DOES affect load distribution with the intended sign.
But predicted COM ay is+0.099248m/s² (left), versus+0.014576 without correction,
and roll angular-momentum rate grows0.363299→22.741934Nm rather than approaching
its requested zero. The first filtered update also shifts the wrong way:
ay+0.018705m/s² and roll moment1.454718Nm. These are fixed-state evaluations,
not a forecast or an observation of a future fall/25ms transient.

The target's required tangential reaction is not commanded by the normal-only
correction. Opposite vertical-force changes sum to zero but create18.437Nm of
roll moment. Dropping the six unactuated components does not make that moment
vanish. Native MuJoCo resolves a physically consistent response that differs
from the requested task. The six independently recomputed base-equation
residuals are below8.2e−13; no generalized/root force was applied. Virtual-work
sign/unit error is below1.4e−17. All21 native evaluated motor efforts remain finite/capped;
contact friction ratio≤0.117 and servo ROM checks pass. This therefore is a
command/task failure, not solver nonconvergence or proof that walking is impossible.

002 is independently rejected by the existing20° physical stop. A preview beyond
that guard would expose nominal Balance commands which the runtime would NOT
integrate; these are not reused as a valid continuation. No trial is launched
from either checkpoint. The placement-retarget implementation is left pending
because its required support-exchange primitive failed this gate; updating a
foot destination cannot repair this demonstrated roll/force mismatch.

`node scripts/verify-pull-load-pure.mjs` tests the two saved states, normal-only
virtual work, projected moment/force demand, complete native response, friction,
ROM/caps, absence of a load command on missing contact, and physical-state
immutability. `web/proofs/pull-step-load-pure/result.json` is the compact result.
The historical relative-FK/entry/Balance/policy/package pure checks still pass.
No integration or forecast was performed; MuJoCo stays228794 confirmed plus the
preserved unknown historical run, Rapier45735. No gain sweep, tangential solver,
new physical trial, media export, push or deployment follows this rejected gate.

## Coupled native-response witness — 44 evaluations, zero integration

`scripts/verify-pull-coupled-pure.mjs` is a standalone diagnostic, **not wired
into the controller**. It evaluates one baseline, 42 central motor perturbations
and one candidate on independent copies of the exact001 native233 state.
Each copy includes the same warmstart; the current recorded arm force is applied
at body15's same material point. No root/generalized force is added. `mj_forward`
computes the native normal/tangential reactions and all27 accelerations; no
`mj_step`, transition finite differences, forecast or physical integration occurs.
The native saved ctrl is a later unintegrated proposal, not the last applied
command. After exact restoration, the diagnostic explicitly installs the last
applied command from the ledger; the maximum difference is0.032659Nm.

The objective was fixed before evaluation. Outputs are COM acceleration xyz,
centroidal angular-momentum rate xyz and right/left normal loads. Scales are
g, mgh and mg (h above the current contact plane); task weights are1 for the
six centroidal components and0.25 for the two loads. Lateral acceleration and
load goals retain the previous declared demand, roll moment targets zero, and
the other centroidal outputs retain their baseline values. Motor variables are
gear-normalized controls. Regularization is1e-4 times squared control change;
central perturbations are0.001. One Cholesky solve supplies one direction,
scaled analytically by0.473909 to respect a fixed0.1 control trust radius and
the unchanged caps. This is a local limited least-squares direction, not a claim
to solve a general bounded optimization or a new feedback controller.

The sole candidate passes all declared local gates. Native COM ay changes
+0.014596→−0.242923m/s² toward the right support, against a−0.529679 goal
(47.314% of the requested change). Centroidal roll-moment rate decreases
0.350796→0.184064Nm. Ground lateral force changes−0.101661→−10.619803N;
right/left normal loads change195.817/203.668→150.437/249.048N. Thus this is
an initial propulsive redistribution, **not left-foot unloading already achieved**.
The largest changes are left/right hip-x−4.000/+3.998Nm and abdomen-x−1.744Nm;
sagittal ankle changes are only+0.00539/−0.00318Nm. The full21 commands and
actual native efforts are saved in the result, with maximum effort60.405% of cap.

The independently assembled27 dynamic residuals are≤1.231e-13, including the
six free-base equations. Friction utilization is≤0.19914. All eight contacts
remain active, with the same native constraint regime for all44 evaluations.
Normalized nonlinear prediction error is6.122e-15, below the predeclared0.009577
limit. Task cost decreases0.0102285→0.00283592; regularization cost is2.230e-6.
Terminal002 is still rejected at20.009° by the unchanged controller guard,
using kinematics only and zero direct dynamics evaluations.

**This PASS is only a fixed-state command witness.** The native right-contact
normal accelerations are+0.175…+0.601m/s² and left-contact values
−0.592…−0.180m/s². These compliant-contact accelerations are not a prediction
that support persists. Root local angular acceleration x increases
0.0802→1.2763rad/s² despite the smaller whole-body roll-moment rate; centroidal
moment control does not by itself establish upright torso control. Neither a
future motor transition, unloading, flight, placement nor reception was tested.
The candidate must not be described as a completed support-transfer controller.

Evidence is `web/proofs/pull-step-coupled-pure/{protocol.json,evaluations.ndjson,
result.json}` (49,535bytes). All44 begin/end records are durable. Native solve
time totals11.146ms (maximum3.268ms), whole diagnostic386.937ms on this cloud;
this is not a browser/phone control-rate benchmark. Existing evidence refuses
a duplicate execution. MuJoCo remains228794 confirmed plus preserved unknown
history, Rapier45735. No runtime controller, main, site or dependency changed.

## Constrained witness — algebra feasible, native regime guard rejects

The next local diagnostic (`verify-pull-constrained-native-pure.mjs`) reuses
the complete saved baseline, eight-output Jacobian, scales, weights, lambda1e-4
and0.1 control trust radius. Exactly21 one-sided perturbations recover the
missing torso/contact sensitivities; every common output matches its former
sample exactly. There is no new baseline forward solve. The only additional
candidate brings the total to22 completed native forward evaluations, zero
physical ticks and zero forecasts.

The small active-set Cholesky adapter passes19 algebra tests (including a
contradictory system and an independent2D active-face enumeration). It finds a
solution in23 of64 permitted pivots. All92 linear inequalities pass individually:
two torso constraints, eight nonnegative normals,32 native pyramidal-friction
facets, retained load>=100N, four endpoint-gap guards,42 motor/trust bounds,
lateral progress and two global-roll bounds. Caps and trust are in the solve;
there is no final command clipping or scaling. The torso conditions are
thetaDD<=0 for the currently growing tilt and upDot·upDD<=0. The latter bounds
squared orientation speed, not physical kinetic energy. These are constraints,
not higher objective weights.

Gap jets use the actual capsule endpoint centres minus radius, including their
Jdot*qvel, rather than a permanently fixed contact material point. Their
1.25ms Taylor margins are a conservative local diagnostic only. Positive normal
acceleration is not rejected in isolation and no physical contact is welded.

**The candidate is NOT natively admitted.** The22nd forward calculation finishes,
then the full `efc_state` comparison fails, before candidate telemetry is saved.
There are22 durable begin records and21 end records; the catch result independently
records22 completed evaluations. This is a known completed call, not an uncertain
or retryable runtime. The exception text says `Native contact regime changed`,
but that array contains joint-limit rows as well as contacts.

A position-stage-only metadata read (no acceleration/force solve, qacc and all233
integration values unchanged) identifies row0 as the **right-knee soft limit**,
joint7. It changes QUADRATIC(1)→SATISFIED(0). The34 rows comprise two knee limits
and32 contact-pyramid rows. This invalidates the assumed constant constraint
regime; it does not establish foot support loss. The geometry signature comparison
had passed. The full final constraint-state array and force/acceleration telemetry
were not preserved, so no claim is made about all remaining native constraints.

Only the linear prediction is available: ay−0.286565m/s², tilt acceleration
−0.339359rad/s², right load135.895N and endpoint gaps−0.427…−0.725mm. These are
NOT measured native candidate results. The constrained algebra is feasible;
physical feasibility after the regime change remains undetermined. No other
candidate, parameter adjustment, forward retry or integration follows this stop.

`web/proofs/pull-step-constrained-pure/` preserves protocol, evaluation journal,
result and a saved-data audit, including the missing-telemetry limitation.
Native time totals7.393ms, complete diagnostic382.740ms on this cloud. MuJoCo
stays228794 confirmed plus unknown history; Rapier45735. Active controllers,
stable main/site, all previous verdicts and dependencies remain unchanged.

## Exact candidate telemetry repair — one native recheck, still rejected

The previous Node process had exited; its missing response was not recoverable
in memory. The saved baseline controls and 21 IEEE754 increments do preserve
the exact candidate: the same `u0[i] + saved.solve.x[i]` expression is evaluated,
without optimization, clipping or new perturbations. The input checkpoint,
model, prior result and sensitivity journal are hash-checked. All 233 native
integration values are checked immediately after restoration, before installing
the same command and recorded arm force at the same material point.

`verify-pull-constrained-native-pure.mjs --recheck-saved-candidate` performs exactly
one `mj_forward`, with a separate exclusive proof directory. It writes all native
forces, accelerations, actuator efforts, contacts, constraint types/IDs/states
and constraint residual inputs **before** the first comparison. No old proof is
overwritten. The old nonqualified result is preserved; this is an explicitly
authorized additional diagnostic, not a retroactive PASS or a simulation retry.

The eight contacts and all 32 ground-contact constraint states are unchanged.
Only the right-knee joint-limit row changes QUADRATIC → SATISFIED; its reaction
is zero. The left-knee limit remains active. This is not a foot losing contact.
The following are now native results, not predictions:

| Existing condition | Native result | Outcome |
|---|---:|---|
| Growing tilt: thetaDD <= 0 | -0.333019963 rad/s² | Pass |
| `upDot·upDD <= 0` | +0.000285776081 s⁻³ | **Fail**, tolerance 1e-10 |
| Eight normal forces >= 0 | minimum 26.335201 N | 8/8 pass |
| 32 pyramidal friction facets | worst residual -17.734491 N | 32/32 pass |
| Retained right load >= 100 N | 135.924908 N | Pass |
| Four current right endpoint gap jets | -0.724521 to -0.427486 mm | 4/4 pass |
| 42 motor/trust inequalities | largest violation 9.47e-15 control units | 42/42 pass |
| Lateral progress | COM ay -0.286637694 m/s² | Pass |
| Two non-worsening global-roll bounds | HdotX +0.345334055 Nm, baseline +0.350796094 | 2/2 pass |

Thus **91/92 inequalities pass; the candidate remains rejected**. The failing
quantity is half the derivative of squared upright-vector speed, not rotational
kinetic energy or full angular speed including yaw. It falls from the baseline
0.027054679 to 0.000285776 s⁻³, but stays positive. Its three dot-product
contributions are -0.014300040, +0.014863300 and -0.000277483 s⁻³. Decreasing the
scalar tilt acceleration therefore does not satisfy the separate orientation
speed condition. No tolerance is widened to accept this result.

The linear model put this condition on its boundary (2.64e-15 s⁻³). The native
error 0.000285776 is within the previously declared prediction-error allowance
0.005410936, while the native inequality itself fails. These are different tests.
All other feature prediction checks pass; normalized common-output error is
0.000481402 versus its 0.012236428 bound. The one observed joint-limit regime
change invalidates exact affine equivalence. Lower task cost (0.010228501 →
0.001624616) likewise does not qualify the command.

COM acceleration xyz is [0.248276773, -0.286637694, -0.025535670] m/s²;
right/left loads are 135.924908/263.588570 N. Largest motor utilization is
70.4211% (left ankle-y, 14.084219/20 Nm); no root actuator effort is present.
All 27 independently assembled dynamics residuals, including six free-base
equations, are <= 3.91e-14. Current right material-contact normal accelerations
are +0.2245 to +1.0600 m/s² with penetration and small signed normal velocities.
This is compatible with the soft-contact response at this instant; the local
gap jets do not establish persistent support, unloading or a step. Root local
angular accelerations are [5.444564, -0.283786, -26.123648] rad/s², also retained
explicitly rather than conflated with centroidal moment or upright-vector speed.

Evidence: `web/proofs/pull-step-constrained-recheck/`, 61,612 bytes. Native raw
SHA256 `1c537202749fce9115edd93620a078b1052e253eaf67eb7cb66a94fd753a4d3e`;
result SHA256 `e3d7b13b84cc503d2cedb7ebbe28ab3efdb8915b8b7c70981a1ae6837ae3d02a`.
`node scripts/verify-pull-recheck-recording.mjs` audits the saved response,
reconstructs the contact/cap/gap conditions and verifies that rejection did not
discard telemetry. It imports no engine and performs no native evaluation.

Cost: one completed forward call, 1.973 ms native / 214.707 ms whole diagnostic
on this cloud. Zero optimizer calls, perturbations, integration ticks or forecasts.
MuJoCo remains 228794 confirmed plus unknown history; Rapier remains 45735.
There is no activation in the controller, dynamic-readiness claim, push or site
change. This result neither proves physical impossibility nor supplies a safe
feedback controller; it identifies the remaining failed local condition.

## Recentered axis guards — fixed-state PASS, no physical step

The next authorized diagnostic is `scripts/verify-pull-axis-native-pure.mjs`.
It reuses the complete native response above as its centre, at the same exact
233-value physical input and the same recorded arm force. There is no new
baseline evaluation. The 21 finite differences are 0.001 control units, directed
inward whenever a positive perturbation would leave the ORIGINAL ±0.1 envelope.
Their directions are recorded before the first call; no failed sample is replaced.

All original targets, scales, weights, caps, friction and 92 conditions remain.
With `d = u-uc` and `offset = uc-u0`, the objective regularizes
`lambda*||offset+d||²`; its normal-equation RHS includes `-lambda*offset`.
Motor/trust rows bound the total `u-u0`, not an additional ±0.1 around `uc`.
There is no command clipping after admission.

Six additional linear guards bound each body-local angular velocity component:
`|omega_i + h*alpha_i| <= |omega_i|`, with the actual `h=0.00125s`.
These prohibit compensation between axes and cover axial rotation too. They are
a conservative, first-order braking guard for this particular entry, not a
universal balance law or proof that an intended lateral lean is impossible.

The old scalar condition remains. Its predicted limit reserves the already
declared error allowance: `g_pred + max(1e-10, .2*|g_pred-g_center|) <= 0`.
Since `g_center=+0.0002857760814277`, the interior limit is
`-0.00007144402035693 s^-3`. The allowance is still tested against the native
response; it is not assumed to bound every future state or active-set change.

The 12 new algebra checks cover frames, centripetal acceleration, finite
differences of rotation, signed/zero angular velocities, inter-axis cancellation,
the error reserve and the non-cumulative control envelope. The 19 existing
Cholesky/active-set checks also pass. They instantiate no model and add no native
dynamics evaluations. A streaming-format test additionally checks that flushed
gzip records can be decoded before end-of-stream, entirely in memory.

The single authorized calculation completes **21 sensitivities + one candidate**.
Every sensitivity and the candidate retain the saved constraint regime; the
right-knee limit stays satisfied and the left one stays active. All raw native
outputs are compressed and flushed before guards. The journal has 22 begin,
22 native-response and 22 end records. The solve uses 25/64 pivots, with
normalized primal violation 1.32e-14, stationarity 1.56e-15 and complementarity
1.13e-16. Every one of the 98 native conditions passes separately:

| Condition group | Count | Native value / worst residual |
|---|---:|---|
| Tilt acceleration and upright-vector speed derivative | 2 | -0.001901072738 rad/s²; -0.00007144402014 s⁻³ |
| Nonnegative normals | 8 | minimum 25.212511 N |
| Pyramidal friction facets | 32 | worst residual -18.317063 N |
| Retained right load >=100 N | 1 | 136.643656 N |
| Current right capsule endpoint gap jets | 4 | -0.724586 to -0.427440 mm |
| Per-axis angular-velocity guards | 6 | worst residual -7.55e-15 rad/s |
| Original motor and trust envelope | 42 | worst residual +1.32e-14 control units, within unchanged 1e-10 tolerance |
| Lateral progress | 1 | COM ay -0.291901267 m/s² |
| Global-roll moment bounds | 2 | HdotX +0.334472135 Nm, below original +0.350796094 Nm |

The tilt/scalar predictions are -0.001901072744 rad/s² and
-0.00007144402039186 s⁻³. Scalar error is 2.51e-13 versus the reserved
7.1444e-5 allowance. All 53 feature prediction checks pass; common normalized
output error is 6.51e-14 versus 0.000903789. Maximum 27-equation dynamics residual
is 1.48e-14, including the unactuated base. Maximum motor utilization remains
70.4211% (left ankle-y 14.084219/20 Nm). No root effort is applied.

Body-local angular accelerations are approximately [0, -0.001624442, +20.320248]
rad/s², giving the *first-order proxy* velocities
[0.002344882, 0.043978620, -0.094870253] rad/s from
[0.002344882, 0.043980650, -0.120270563]. Thus axial rotation is braked;
it is not a new axial acceleration in the direction of the current spin.
COM acceleration is [0.244811952, -0.291901267, -0.026339877] m/s².
The lateral change supplies 56.31% of the original requested correction.
Loads are 136.643656/262.836976 N (right/left), still bilateral. Task cost is
0.001678960 versus 0.001624616 at the prior centre and 0.010228501 at the original
baseline; the tighter constraints slightly worsen the objective, as expected.

**This is only a fixed-state command PASS.** Tilt speed remains positive at
0.043957781 rad/s; its braking is weak. Neither support persistence, left-foot
unloading, clearance, placement nor reception has been observed. The instantaneous
limits do not certify enough braking before the existing 20-degree stop, and this
diagnostic is not installed in the controller. A separate reviewed dynamic
observation would be required; no such permission is inferred from this PASS.

Evidence: `web/proofs/pull-step-axis-pure/`, **61,617 bytes**, including all
predicted/native condition values. Result gzip SHA256
`9b54af040aaefa9155513549f2b0b9ec3e2fa3e2139d41baed6ca52d14e000d3`;
journal gzip SHA256
`d505e2dfe772d1d8e83c1ecf6ca086de2e1f05c553052f78f149b7b8ed188802`.
The saved-data-only audit `verify-pull-axis-recording.mjs` independently
reconstructs all 98 conditions and checks exact inputs, efforts, original bounds,
record ordering, source hashes and the scalar reserve. It adds no evaluations.
Native calls total 6.140 ms; algebra 14.996 ms; whole diagnostic 347.077 ms on
this cloud. This is not a mobile or online-control performance measurement.
MuJoCo stays 228794 confirmed plus unknown history, Rapier 45735; zero ticks and
forecasts, no retry, active-controller modification, push, deployment or install.

## Short dynamic axis window — 13 ticks, candidate 14 rejected

The next separately authorized experiment is `scripts/run-pull-axis-window.mjs`.
It is a local runner, never imported by the app. Its prospective maximum is160
actual ticks (200ms),3680 explicit `mj_forward` calls and64 pivots per solve,
with zero forecasts. No second attempt or automatic extension is allowed. The
original001233-value checkpoint is restored exactly at10.24375s; the prior
20tick declared grab-force entry is reused, with point history now retained.

Every live tick calculates the arm force ONCE, freezes it in all copied states,
evaluates one centre,21 inward sensitivities and one sole native candidate, then
applies at most one `mj_step`. The admitted total controls have a single owner;
`Physics.control` is never called and no blending/clipping is added afterward.
All geometry, contact frames, Jacobians and velocity biases are rebuilt at the
current state. Sensitivities/candidate are compared with their current centre's
regime, not the historical entry's regime. The original fixed `u0±0.1` envelope,
objectives, scales, regularization and caps remain. An inter-tick regime change
is reported; it is not alone interpreted as physical contact loss.

The scalar reserve now handles any sign of the centre, using three equivalent
linear inequalities: `gp<=-tol`, `(1+r)gp-r*gc<=0`, `(1-r)gp+r*gc<=0`, with the
unchangedr=.2. Fifteen guard algebra tests (including three signs and rejected
counterexamples) and19 active-set tests pass. The old saved98-condition audit
also still passes; no historical result is rewritten. Raw native accelerations,
contact forces and constraint states are gzip-flushed before guards. A full last
native response and the exact live checkpoint are retained on stop. An uncertain
integration reservation is never cleared merely by entering a cleanup block.

**Observed result: window INCOMPLETE/rejected, no physical retry.** Exactly13
steps were integrated,8195→8208 and10.24375→10.26000s (16.25ms). All13 applied
commands passed98 native conditions and the prediction checks. The14th command
was calculated but NOT applied. The322 explicit forward calls all completed
(14×23); no pending integration/forward or count uncertainty remains.

| Measured quantity | Entry | Final live state |
|---|---:|---:|
| COM lateral velocity (right is negativeY) | +1.189mm/s | -2.941mm/s |
| COM forward velocity | 20.569mm/s | 24.641mm/s |
| Body-local angular velocityY | .043981rad/s | .021687rad/s |
| Body-local angular velocityZ | -.120271rad/s | -.047244rad/s |
| Torso tilt | 8.08145deg | 8.11410deg |
| Last resolved right/left loads | 195.840/203.680N | 137.749/262.300N |
| Whole left-sole minimum | -.7478mm | -.8375mm |

COM displacement is+.3695mm forward,-.0187mm toward the right support and
-.0282mm vertically. Maximum retained-foot horizontal drift is.0378mm. No
non-foot support, loss of retained support or20degree stop occurs. Maximum
actual motor utilization is70.4211%. The largest command jump is12.0000Nm at
activation, within the unchanged120Nm hip caps. This is explicitly a bounded
activation, not continuous torque or a reproduced historical grab gesture.
The tiny lateral displacement and increased left load establish no unloading,
clearance, flight, placement or landing.

At the rejected14th command,96/98 native inequalities pass. Two declared
LOCAL DIAGNOSTIC guards fail:

- Negative axial-velocity guard: predicted residual approximately-1.27e-16rad/s,
  native+1.5302724e-6rad/s, versus unchanged1e-10 tolerance.
- Global roll-moment rate: predicted+.350796094Nm at the allowed boundary,
  native+.365499871Nm, exceeding the declared reference by.014703778Nm.

Five per-feature prediction-error checks also fail (two contact0 rows, one
contact3 row, right load and one gap jet). These are prediction failures, not
negative actual normal loads or actual cone/cap violations. The combined
normalized output error.000473521 remains below its.004701763 allowance; that
aggregate check does not override individual failures. All candidate friction,
normal-force and physical motor caps pass. Independent27-equation residuals
across evaluations are<=6.26e-13. All14 algebra solves succeed (maximum42/64
pivots). The current centre regime is unchanged within every set of23 calls;
a real regime change between live ticks is retained in the records. Thus a
constant final regime did NOT suffice to establish exact affine prediction.
This does not certify infeasibility or establish the source of prediction error.

A separate limitation is already visible in the applied transitions. At tick0,
root angular `qacc` is approximately[0,-.000270740,20.320292]rad/s², whereas
`(qvel_after-qvel_before)/dt` is[.004177446,.018113760,19.471470]rad/s². The
first-order angular guards therefore did not bound the actual integrated
components: X/Y briefly grow despite the predicted guards. The native step
and direct forward are distinct calculations. The implementation records both
`nativeStepQacc` and integrated acceleration; implicit damping/integration is a
hypothesis to inspect in code/documentation, not a demonstrated cause here.
This issue must be distinguished from candidate14's linearization error before
proposing another dynamic test. No tolerance/gain/model change follows the stop.

`verify-pull-axis-window-recording.mjs` imports no engine and adds zero calls or
steps. It verifies322 begin/native/end records,13 application events, reconstructs
all98 conditions at rejection, and confirms that the live checkpoint contains
command13 (not the rejected command14). All233 values roundtrip exactly in the
runner. Grab.previous, target, local point, bridge and pending frozen input are
saved. Source/model/input hashes and old evidence are preserved. The compact
start/end geometry replay was visually inspected: full body/ground visible,
labelled13ticks, no suggestion that a step occurred.

Proofs: `web/proofs/pull-step-axis-window/`. Final checkpoint gzip SHA256
`808e25db5e819b0bb228c619f7c09a987dd451ee272f3210b99e4495b428f430`;
forward journal SHA256
`42fb63cfcabcfb3a95d62c81b8c7da6bbe9fe870190ab594b84c3316f2ed0380`.
Native calls total36.848ms, active-set solves161.089ms, whole runner1423.349ms on
this cloud; not a real-time800Hz/browser/mobile performance qualification.
MuJoCo is228807 confirmed PLUS historical unknown; Rapier45735 unchanged.
No default controller, main/site, dependency, Library transfer or publication
changes. The declared160tick window was not completed and no pas is claimed.

## Euler/island diagnostic correction — qualified pure candidate, stopped after two ticks

The 91c3d60 result above remains INCOMPLETE. Its identical global `efc_state`
arrays did not establish an unchanged resolved regime: in MuJoCo3.17 Newton's
island path, resolved states live in `iefc_state`. Forces are gathered back;
states are not. The new adapter uses `map_efc2iefc`, verifies the inverse mapping,
row type/id and island/global force agreement, and independently checks the
native pyramidal/limit force law against `J*qacc-aref`. No engine flag changes.

The Euler adapter applies `(M+h*B)^-1*M*qacc` using the existing Cholesky,
current mass and declared linear joint damping. It explicitly rejects unsupported
polynomial/servo damping, integrators or non-default flags. Rebuilding mass at
the saved001 state (kinematics/comPos/makeM, no forward) reproduces all27
accelerations actually integrated in the old first tick to1.63536e-13, hence
velocity error2.04420e-16. Native233 values stay exact. Old first-tick X/Y
angular acceleration is now correctly rejected by the discrete velocity guard.
The original instantaneous torso guards remain. Gap jets use the semi-implicit
Euler coefficient; these jets are predictions, not measured future contacts.

Current resolved-branch inequalities are added to the SAME Cholesky active-set
search. The existing20% reserve also protects the diagnostic torso, gap, axial,
global-roll and branch boundaries. Normals, friction, caps, objective eight
outputs/scales/weights, lambda1e-4, perturbation.001 and TOTAL originalu0±.1
remain unchanged. Axis non-growth is still a conservative local diagnostic,
not a necessary condition for walking. Branch guards constrain this local
linear model, not physical contacts. Raw native outputs precede guards.

One exclusive runner was launched: `run-pull-euler-window.mjs`, source91c3d60
plus source hashes in started.json. It restores the exact8208/t10.260 state and
COMPLETE grab.previous/target/local/20tick bridge/pending input. The first
frozen input exactly matches the saved pending input. No13tick replay or new
force bridge. Maximum24 pure forwards, then128 physical ticks,2968 explicit
forwards total,64 pivots/solve,0forecasts,2MB proofs; no retry.

**Pure PASS, dynamic window INCOMPLETE/stopped.** Reproduction of the old
rejected14th response is exact (qacc max error0), and confirms its stale global
states versus resolved island states. The first qualified candidate passes all
131 native inequalities (98 previous conditions, corrected discrete predictors,
plus33 branch-domain conditions), all per-feature prediction checks, and all
linear reserve rows in38pivots. Pure calls are exactly24. The next admitted
candidate also passes131 conditions in37pivots. Native normalized prediction
errors are1.403e-14 and1.958e-14. The two actual Euler velocity errors are
6.9294e-15 and2.2204e-16, below the fixed1e-12 check.

Exactly TWO physical ticks were applied:8208→8210,10.260→10.2625s. The next
CENTRE forward (before sensitivities, solve or proposal) fails the unchanged
independent27-equation residual guard1e-7. Saved-only audit reconstructs its
maximum root residual2.332787e-3Nm on root rotationX, maximum joint residual
1.698278e-3Nm. The native Newton solve reported ONE iteration, not exhaustion
of its100-iteration model budget. Resolved branch/force-law checks still pass.
This response does not satisfy our declared residual certificate; it does NOT
prove physical infeasibility, a cap violation or a measured support loss.
The recorded data do not identify which native stopping criterion fired;
solver cost/decrement statistics were not saved. No tolerance, iteration limit,
controller gain or command was changed after this stop.

| Measured quantity | Restored entry | After two ticks |
|---|---:|---:|
| COM forward velocity |24.6411mm/s|25.2969mm/s|
| COM lateral velocity |−2.9413mm/s|−3.4115mm/s|
| Torso local angular velocityY |.0216871rad/s|.0213057rad/s|
| Torso tilt |8.114106deg|8.117159deg|
| Last resolved right/left loads |137.749/262.300N|137.922/262.447N|
| Whole left-sole minimum |−.8375mm|−.8546mm|

COM movement is+.06283mm forward,−.00825mm rightward,−.00498mm vertically.
This is no useful transfer/unloading or step. Maximum retained-foot drift from
original001 is.04706mm, command jump from last APPLIED command6.04914Nm,
motor utilization70.4211%. No fall/non-foot support/retained-support-loss guard
fires. Reactions are now labelled at the PRE-integration resolution time;
geometry is updated at the POST-integration time (1.25ms apart).

Counts:48 COMPLETED explicit forwards =1 reproduction +2×23 qualified batches
+1 rejected centre. Each of the2 physical mj_step calls also invokes its native
internal forward; no extra explicit calls or forecasts were made. MuJoCo228809
confirmed PLUS unchanged historical unknown, Rapier45735. The last applied
command, full233 native state and complete grab/pending input are saved.
`verify-pull-euler-window-recording.mjs` reconstructs the residual via only
kinematics/mass/Jacobians and verifies all counts/conditions/restoration with
ZERO additional forwards/steps. Separate tests pass15 torso/reserve cases,
19 existing Cholesky cases, island permutation/force/type counterexamples and
the recorded Euler transition.

Evidence: `web/proofs/pull-step-euler-window/`. Final checkpoint gzip SHA256
`b0b188d3e5e80a980ea81108e316e8af27ca93c52f4973069edaa4411b1201bd`;
journal SHA256
`2c2908660086d47dbe788bba8df1fef037d14a333b7859f25f9bb9d0dbe66f69`.
The beginning/end collision-geometry replay was inspected (full mannequin and
ground, explicitly two ticks/no step). Native explicit calls11.338ms,
Cholesky82.207ms, full runner919.814ms on this cloud; no real-time/browser
performance qualification. No active app controller/main/site/push/Library
change, no dependency, no reset or deleted pre-existing proof. Further physics
requires review of the native residual certificate AND transfer task; this
experiment is closed without retry.

Primary implementation references:
- https://raw.githubusercontent.com/google-deepmind/mujoco/3.17.0/src/engine/engine_forward.c
- https://raw.githubusercontent.com/google-deepmind/mujoco/3.17.0/src/engine/engine_solver.c
- https://raw.githubusercontent.com/google-deepmind/mujoco/3.17.0/src/engine/engine_core_smooth.c

## Stationarity certificate — final diagnostic attempt closed before integration

The c9fb8aa window and its two ticks remain INCOMPLETE. This separate attempt
implements only the reviewed numerical certificate; it does not alter the web
controller, model, native solver tolerance, gains, caps, objectives, regularizer,
original u0±.1 envelope or any of the131 admission conditions. Historical
runners/evidence are preserved. The eight-contact topology and fixed100/299.498N
right/left targets remain restricted to this initial propulsion observation.

`pull-numeric-certificate.mjs` independently reconstructs M from qpos on a
scratch MjData (kinematics/comPos/makeM only) and J from contact frames, point
Jacobians, pyramid edges and joint-limit signs. It checks these against native
M/J and J'force. With r=M*a-smooth-J'force and the resolved current branch,
K=M+J_A'D_AJ_A, it solves K*deltaA=-r with the existing Cholesky. Backward error,
finite values, positive definiteness, unchanged branch with floating-point sign
margin, and normalized Newton decrement <= UNCHANGED native1e-8 are required.
Constraint generalized forces have their native units: contact N, hinge-limit
Nm. The normalized decrement follows the native coordinate metric, not a new
uniform physical force/torque threshold. The module never writes controls,
native qacc, qpos, qvel, forces, flags or time.

Correction effects are mapped through (M+hB)^-1*M into Euler acceleration and
velocity, and through -D*J into contact forces. Shadow feature evaluations at
the SAME q/qdot quantify their effect on torso, gap, axis, branch, friction,
loads and centroidal outputs. They are numerical-error estimates within the
verified piecewise-quadratic region, not physical forecasts or interval-arithmetic
proofs. Their absolute effect plus a128*machine-epsilon arithmetic allowance
is added to native guard margins. Centre uncertainty tightens search rows;
candidate/centre/sensitivity uncertainty is propagated through finite differences
and added to prediction errors. The existing20% linearization error budget stays
separate and unchanged. An inconclusive branch/certificate stops the attempt;
no branch search, epsilon tuning or native solver refinement is attempted.

The zero-forward test checks bad mapping, large residual, nonfinite input,
wrong solve sign, branch crossing and insufficient margins. On the saved8210
state it reconstructs exactly the recorded branch Jacobians and generalized
contact force; all233 state values remain exact. The normalized decrement is
5.676850123495e-10, corrected minimum edge .001284046061N, maximum Euler velocity
correction4.254368842e-7rad/s and backward residual3.04e-18. This passes the new
certificate without claiming the old1e-7 guard passed.

One exclusive `run-pull-certified-window.mjs` launch was made from native8210,
t10.262500000000637, gzip SHA256
b0b188d3e5e80a980ea81108e316e8af27ca93c52f4973069edaa4411b1201bd.
It restores all233 native values and complete grab/bridge/pending input. No
13-tick or2-tick replay. The frozen interaction matches saved pending input
exactly: force [15.2097812,1.1061050,-.8716042]N at the same material point.
The retained reaction/force metrics in the initial trace refer to the LAST
APPLIED tick, one substep earlier, not this unevaluated future input.
Declared bounds:1 reproduction +126*23 explicit forwards=2899,126 steps,
64 pivots/solve,0forecasts,2MB evidence, original end10.4200. Source c9fb8aa
plus tested file hashes are recorded in started.json.

**STOP during pure sensitivities: ZERO new ticks, no candidate or solve.**
Exactly8 explicit forwards completed: reproduction, centre and sensitivities0..5.
All8 numerical certificates pass. Reproduction qacc error is0. The +.001
normalized right_hip_y perturbation (+.12Nm on an independent copy only) changes
resolved pyramid row12/contact2/right foot from QUADRATIC1 to SATISFIED0.
Its unconstrained edge lambda changes+.001960296783N to-.006659797797N;
the native edge force therefore becomes0. The contact itself remains loaded:
perturbed contact normal8.87774877N, all-contact minimum friction facet slack
3.86962607N, right/left totals137.233854/263.580617N. Thus the fixed-branch
linear sensitivity domain failed; this is not a measured lost foot or physical
infeasibility. No modified perturbation, candidate, active-set solve, command
application, retry or post-result tuning follows this stop.

Final native233 values, time,8210 steps, last applied controls and complete
grab memory remain EXACTLY the input values. No integration or forward is
pending/uncertain. All raw outputs precede guards; full failed native response
and checkpoint are saved. `verify-pull-certified-recording.mjs` audits counts,
state/command ownership and branch transition from saved files only, without
importing MuJoCo. New ticks0, forecasts0; MuJoCo228809 confirmed PLUS unchanged
historical unknown, Rapier45735. No physical progress, unloading, lift or step
was newly observed. The two identical geometry-replay panels were visually
inspected; their explicit0-tick label and ground/full-body view are retained.

Proofs: web/proofs/pull-step-certified-window/. Checkpoint gzip SHA256
`a8732adaf30a4a706ded2da7227cb41b040acfd491be12067318257a85a4aa49`;
journal SHA256
`45a080e92cb5dbd36297299312fc6ac477c5914586d51e23007b714aaa3312a1`.
Native calls4.407ms, certificate work26.693ms, no active-set solve, whole runner
625.333ms; no real-time/browser qualification. This diagnostic phase is CLOSED.
The structural next issue is a transfer task that transitions propulsion to
braking/unloading using actual contacts: a forever-fixed8-contact regime and
100D/299.498G load target cannot constitute that transition. This result does
not authorize another numerical micro-window. Main/site, Library, dependencies,
previous evidence and historical verdicts remain unchanged.
