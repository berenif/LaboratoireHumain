# Recovery: mechanical feasibility before controller changes

Local investigation from `c597959f46f37715035349c8cd0a699b6822111a`, on
`investigate/physical-support`. No push, merge, deployment, anatomy change,
qualification change or production controller change. Automatic recovery
remains disabled by default and unqualified.

## Decision and scope

Do not replace the mannequin on the evidence available. Its current mass,
joint ranges and torque limits admit a useful family of gravity-balanced
flat-sole poses. This is evidence against a blanket strength or morphology
obstruction, not a proof that the complete initial-to-standing motion exists.
The important unresolved problem is contact acquisition and dynamic transfer
into that family, followed by tracking and ordinary-controller handoff.

Stop modifying phase constants independently. Build one complete, constrained
trajectory, verify its mechanical residuals, then execute it with the same
contacts, limits and effort budget. The work below supplies a reproducible
baseline and a first static inverse-dynamics discriminator.

## Reproduced baseline

The pristine native acceptance executable reproduces `landed-crouch-left`:
no recovery, final `Brace`, six retries, no integrity failure, preserved dynamic
ownership. The pinned `recovery-profile-v24.json` is unchanged. Two independent
read-only 2-second traces at both 0 and 60 degrees are byte-identical. All eight
quarter-second segment snapshots in the zero-heading trace match the pristine
acceptance run exactly. Tick-level contact aggregates and stamps are not
compared against a substep observer as if their observation boundaries matched.

At 0.050 s, the current balance function calls the capture point interior,
although solved loaded points span only about one micrometre in the forward
axis (Z approximately 0.051891 m). COM Z is 0.036471 m and capture Z 0.030998 m.
The controller's polygon comes from low geometric vertices of loaded segments;
it is not the convex hull of loaded solver points. This is a concrete support
representation mismatch. It is not yet a demonstrated unique cause of failure.

At 0.083333 s the COM velocity is +0.051101 m/s and acceleration +1.607941 m/s².
At 0.25 s velocity is +0.151545 m/s; at 0.5 s it is +0.313372 m/s. The ankle
reaches 20 degrees while the later capture point escapes forward. Contacts on
hindfeet do appear: their existence alone does not establish a controlled
transfer or stable capture.

Native `motor.impulse` is the last solver value, not the summed delivery over
all internal passes. The final trace separately records `delivered_impulse/dt`,
`delivered_abs_impulse/dt`, and the installed cap. Do not infer delivered effort
from the legacy `lastSolverImpulseNms` field alone.

## What the model does and does not explain

| Aspect | Finding | Consequence |
| --- | --- | --- |
| Morphology | 1.84 m, 72.2 kg; 0.42 m thigh and 0.40 m shin; 25 bodies | No dimensional contradiction establishes impossibility. The selected “crouch” fixture is a shallow, elevated pose, not a floor-to-standing demonstration. |
| Joint freedom | Hip −20/+110°, knee 0/140°, ankle −45/+20°, forefoot −20/+45° | The ankle bound actively constrains this strategy. Do not change it merely because tracking reaches it. Other routes may use pelvis translation, trunk flexion and intermediate support. |
| Mass/inertia | Explicit geometry-derived mass centres and positive stored principal inertias | Static equilibrium uses these mass centres. Positive inertias alone do not validate dynamic conditioning; the 0.2 kg ankle and separate foot bodies deserve solver checks if a mechanically feasible hold fails. |
| Actuation | Sagittal nominal caps: hip 150, knee 165, ankle 110, forefoot 35 Nm; shared 760 Nm | Nominal strength is different from the current allocator's installed caps and actual delivery. The model is not a physiological muscle/power model. |
| Contact | Separate rigid hindfoot and forefoot; small heel extent; initially toe-dominated support | Contact mode is a planning constraint. A full geometric sole cannot count as acquired support. Static vertical reactions do not certify dynamic friction response. |
| Control | Local joint tracking, root orientation effort and inferred support coexist | A failure here cannot identify anatomy as its cause. Validate inverse-dynamics effort and loaded contact acquisition before another feedback redesign. |

## Static inverse-dynamics discriminator

`plan_static.py` uses canonical joint frames and the same non-Euler anatomical
coordinate map as native FK. It optimizes 41 symmetric flat-sole poses from
about 0.90 to 0.978 m pelvis height. Actual foot and forefoot bottom vertices
supply potential support locations. For each pose a linear program solves:

- Total weight, roll moment and pitch moment equilibrium with nonnegative
  vertical ground forces; horizontal force is zero, so friction demand is zero.
- Each subtree's gravity/contact moment about its anatomical joint, converted
  through the coordinate Jacobian to generalized joint torque.
- Every permitted-axis torque ceiling and the absolute summed 760 Nm budget.

All 41 sampled poses also pass native convex geometry inspection: zero measured
floor/self penetration and zero anatomical limit excess, with zero integrated
steps and unchanged body snapshots. This does not certify interpolation.
An independent force/moment sum gives maximum vertical residual 2.28e-13 N
and pitch/roll residual below 4.27e-14 Nm.

The 41 LPs are feasible. Maximum summed required torque is 334.169 Nm; maximum
individual utilization is 0.554725. This is a sampled, static branch. It does
not include the initial toe pivot, prove interpolation collision freedom,
include acceleration/Coriolis terms, or qualify an actual recovery.

The isolated native hold diagnostic seeds selected poses **before integration**,
installs those coordinate targets and gravity feedforward in bounded native
motors, and lets all 25 dynamic bodies evolve. No body is repositioned during
integration and no pelvis force or torque is installed. Gains retain the pinned
profile's ordinary native scale; caps share exactly the original 760 Nm budget.
This is a discriminator for tracking/contact consistency, not a live-recovery
controller and not an accepted replacement for existing qualification tests.

### Actual isolated native outcomes

| Planned pose | Native outcome at 7 s | Pelvis displacement | Maximum segment linear speed in final second |
| --- | --- | --- | --- |
| 1/41, low | Remains upright with drift; no admission claimed | 0.01607 m | 0.04088 m/s |
| 21/41, middle | Falls forward | 1.50525 m | 0.02367 m/s |
| 41/41, high | Falls forward | 1.47300 m | 0.00495 m/s |

All three retain integrity. Low final speed in the last two cases means the body
has settled on the floor, **not** that standing passed. The test executable's
`ok` means the diagnostic completed and wrote its reports. It is not a physical
pass assertion. The 10 mm hold-drift screening objective was not achieved.
Installed cap sums differ from 760 Nm by at most 0.000122 Nm from f32 addition;
no nominal budget or anatomical ceiling was raised.

This static feedforward plus independent joint PD experiment is rejected as a
controller. Its source and executable are retained only in ignored evidence
(`static-hold-probe.rs`, `static-hold-test-binary`). It does not justify stronger
actuators: the middle/high poses initially remain close to their planned states,
then slowly drift and tip. A static force distribution is not necessarily the
one generated by a compliant closed-loop articulated contact system. Next,
solve and control contact wrench/COM dynamics, with an independent audit of
feedforward sign, wrench residual and delivered effort before live recovery.

![Pristine physical baseline](../evidence/physical-support-2026-10-10/baseline-frames.png)

![Native isolated holds: two falls](../evidence/physical-support-2026-10-10/static-native-outcomes.png)

## Method and predeclared decisions

1. **Geometry and static feasibility.** Validate canonical FK, all joint ranges,
   ground clearance and nonexcluded convex-pair collisions at dense samples.
   Solve compressive-contact equilibrium and per-axis/aggregate efforts. Accept
   only poses satisfying unchanged tolerances. Reject a quasi-static route if
   no solution exists; this does not reject all dynamic routes or the mannequin.
2. **Native held equilibria.** At early, middle and final poses, compare planned
   gravity torques, installed caps, actual delivery and solved contact wrench.
   Record integrity, COM/foot drift, speeds and loads. A 7-second isolated hold
   is only a screening test; final admission needs the existing standing gate.
   If a pose fails despite adequate static margin, isolate allocation/feedback
   versus contact/inertia implementation before increasing anatomical strength.
3. **Complete contact-sequence trajectory.** Use reduced sagittal direct
   collocation with full canonical masses/COM/inertias: exact initial pose and
   velocity; toe pivot; acquired heel support; COM recentering; extension; stable
   terminal pose with zero velocity and acceleration. Enforce Newton–Euler
   residuals, joint bounds, no penetration, contact complementarity, friction,
   torque limits and aggregate budget. Include velocity-continuous transitions.
   Refine the time grid and re-evaluate residuals independently. A kinematic
   corridor or point-capture criterion alone is insufficient.
4. **Intermediate support decision.** If the feet-only trajectory has no
   independently feasible solution within declared bounds after mesh refinement,
   solve a specified hand/foot or knee/foot contact sequence under the same model.
   Choose intermediate support if it yields a feasible complete route while the
   feet-only route does not. Optimizer failure alone is not an impossibility proof.
5. **Execution.** Track the verified full-body trajectory using constrained
   inverse dynamics/native joint efforts, with measured contact and momentum
   feedback. Use the real state to authorize contact transitions, not planned
   percentage completion. Any feedforward and feedback share existing caps.
   Compare against the frozen baseline at 0 and 60 degrees, with no pose writes,
   reset, hidden pelvis assistance or changes to qualification.
6. **Admission and robustness.** Require the existing complete recovery and
   standing-observation contract, ordinary-control handoff, deterministic repeat,
   then the remaining fixtures and disturbance/heading matrix. Unit tests,
   geometry, static LP feasibility and isolated holds are separate evidence.

The initial toe-only pose is not statically balanced: with zero horizontal
reaction and an effectively line-like support, its COM is behind the contacts.
A dynamic initial pivot or another support is therefore required. This is a
statement about the selected contact mode, not a proof that hands/knees are
necessary for all possible motion.

## Evidence and reproduction

Local evidence root: `evidence/physical-support-2026-10-10/`.

- `baseline-acceptance`, `baseline/recovery.json`: pristine executable and full
  unchanged fixture outcome.
- `trace-a/`, `trace-b/`: identical initial native traces at two headings.
- `trace-final-a/`, `trace-final-b/`: traces with summed native motor delivery added.
- `static-geometry.json`, `static-residual-audit.json`: native sampled geometry
  and independent static force/moment residuals.
- `plan_static.py`, `static-plan.json`: constrained static planner and 41 results.
- `static-native.json`: isolated native holds, when present; inspect actual
  outcomes rather than interpreting test-process success as physics success.
- `baseline-frames.png`, `baseline-support.png`, `static-poses.png`: rendered
  native snapshots and explicitly labelled static planning images.
- Installation/build/test logs retain failures and retries.

Run the ignored read-only capture explicitly with `LH_SUPPORT_OUT` pointing to
an absent absolute directory, using the pinned Rust 1.89.0 release test binary:
`recovery::support_probe::capture_support_trace --ignored --exact`.
The archived static discriminator takes `LH_STATIC_PLAN` and a fresh
`LH_STATIC_OUT`; run its saved binary with
`recovery::support_probe::hold_static_plan --ignored --exact`. Its experimental
controller is excluded from the final source. The retained
`inspect_static_geometry` diagnostic uses `LH_STATIC_PLAN` and `LH_GEOMETRY_OUT`
and integrates zero physical steps. Neither is part of release qualification.

The requested Library package identity is
`libfile_3e62df16c0e88191bbc8d274391f5f03` (`lh-astra-handoff.zip`). Preparation
succeeded but the prescribed download helper failed twice. A HEAD diagnostic
identified `Tunnel connection failed: 403 Forbidden`; no bytes were received.
No parent-environment patch or rejected corridor was applied. Results sent in
messages are attributed to that other investigation, not reproduced here.

The 23 focused recovery unit tests pass. Read-only diagnostic completion and
41 native geometric inspections are recorded separately from failed native hold
screening and the failed complete baseline recovery.

No complete native workspace, TypeScript, WASM/browser or release suite is
claimed passing by this investigation. The reported pre-existing hybrid joint
and four TypeScript failures remain relevant until reproduced and resolved.

## Whole-body continuation: support wrench, not only posture

Evidence is in `evidence/whole-body-recovery-2026-10-10/`. These are isolated
native experiments, initially seeded at the previously inspected static poses;
none executes or qualifies the complete fixture-to-ordinary-stance recovery.
The complete fixture baseline still has zero recoveries.

Giving the forefoot its full nominal 35 Nm allocation from the unchanged
760 Nm aggregate budget improves the middle static hold (35.5 mm root drift at
7 s), but the low pose drifts 145.6 mm and the high pose falls. The allocation
issue is causal but is not a sufficient controller repair. This variant is
archived as `contact-reserve-probe.rs`, outside the retained source.

A first inverse-dynamics discriminator uses body mass, inertia, gravity and
native anatomical motor efforts, with no body forces or pose updates. The high
pose remains upright for 7 s, with 4.64 mm root displacement. Its last second
has maximum speeds over all 25 segments of 0.01531 m/s and 0.34923 rad/s.
However, this prototype omits one global moment equation and drifts about
−0.50 rad in pelvis yaw. It is rejected; the low pose is infeasible immediately,
and the middle pose is not angularly stable. `inverse-hold.json` contains all
25 initial poses and native snapshots at 60 Hz; the full canonical coordinates
are in `static-plan.json`. `inverse-centroidal-5equations.rs` preserves the
rejected controller. `physical-progress-unqualified.png` shows this distinction.
It is individually saved in Library as
`libfile_c0ef4fa02b8c8191a0e53d82a7699c27`.

Enforcing all six force/moment equations rejects the available native contact
wrenches after 8.3–12.5 ms. This is independently confirmed with SciPy/HiGHS:
all 15 recorded requests are infeasible. It is also confirmed by a necessary
bound for the original **circular** Coulomb cone, not only the conservative
friction diamond used in the optimizer. For example, at 8.3 ms the required
pitch moment is 70.694 Nm, while the loaded/candidate solver contacts can
provide at most 39.173 Nm. The required sagittal centre of pressure is −99.91 mm;
the rear solver point is only −55.06 mm. Existing zero-load solver rows are
allowed to acquire compression in this prediction but never count as measured
support evidence. This establishes why flat geometric soles alone cannot
validate a posture servo during the actual transient. It does not establish
that feet-only recovery is impossible.

The independent audit is `check_wrench.py` and
`independent-wrench-feasibility.json`. The analytic bound follows from
`Mx = y_ref Fz - sum(z_i Fn_i) + sum((y_i-y_ref) Ft_z_i)`,
`Fn_i >= 0`, and `abs(Ft_z_i) <= mu Fn_i`; it includes the small differences
in contact heights. No contact force or friction threshold is increased.

Allowing the foot pitch to pivot while choosing the contact centroid as the
pressure target survives longer, but rotates backwards and loses feasibility
at 0.65–0.88 s (330–407 mm pelvis displacement). It is also rejected and
archived as `free-pivot-probe.rs` with `free-pivot-hold.json`. A prescribed
quintic joint trajectory with a dynamically free toe pivot is being screened
separately (`toe_dynamics.py`, `toe-rollouts.json`). A failed prescribed family
cannot be interpreted as a proof that a hand or knee is necessary.

The coupled intervention documented below solves task accelerations and compressive
contact forces together, with all six global wrench equations, native per-axis
caps and an L1 effort bound inside the unchanged collective budget. A fixed
pressure target cannot replace this coupling. Any resulting controller remains
an experimental diagnostic until the actual initial pose, anatomical contacts,
standing admission, ordinary handoff and full stability window are executed.

The retained support correction obtains the balance polygon from persistent,
currently positive native solver impulses through `planted_contacts`, rather
than from nearby mesh vertices. Degenerate point/line contacts still define a
bounded braking orientation but cannot pass `inside_support` with its unchanged
3 mm inset. Existing geometry, anatomical motors, contact persistence and all
qualification thresholds remain unchanged. A fixture regression exercises the
native toe-only contact at both headings without writing a body state.

## Requested checkpoint: 2026-10-11

The coupled QP test is now complete. It chooses generalized accelerations and
contact forces together, enforces all six global equations, per-axis nominal
caps and the existing aggregate budget, and installs only anatomical motors.
Its primal/dual solver passes an independent analytic bound/equality example.
With the reduced six-coordinate model, all three prepared native holds reach
7 seconds without an integrity failure or infeasible request:

| Prepared pose | Pelvis displacement at 7 s | Maximum linear speed in last second | Maximum angular speed in last second |
|---|---:|---:|---:|
| Low | 15.28 mm | 0.05673 m/s | 0.30674 rad/s |
| Middle | 8.91 mm | 0.01716 m/s | 0.24070 rad/s |
| High | 3.18 mm | 0.01387 m/s | 0.35029 rad/s |

The high pose's yaw is approximately −0.004 rad, versus approximately −0.50 rad
in the rejected incomplete-wrench controller. This is a meaningful mechanical
improvement, but a prepared native hold is not a recovery. The actual crouch
attempt toward the high pose falls and loses feasibility after 1.60 seconds;
no standing admission or original handoff occurs. Native snapshots, exact
canonical coordinates and delivered motor impulses are retained. The separate
Library image is `libfile_aacd9e2fcb548191b292a6a11ff34327`
(`coupled-control-unqualified.png`).

A preparation-first attempt uses the low pose, and only advances to the high
pose after persistent loaded hindfeet, upright feet, measured COM/capture
containment and the original all-body stability bounds persist for the original
0.55 s. This attempt never earns that transition: it fails at 1.57 s. It slows
the COM to +0.01234 m/s at 83 ms, versus +0.05110 m/s in the pristine witness,
but loses hindfoot loading by 250 ms and falls. Therefore this route is rejected,
not promoted on the strength of its initial improvement.

The native force-based motors solve damping at the end of each small solver
step. Their requested and delivered torques differ materially during motion.
The conversion test accounts for the average sample time `(N+1)/(2N)*dt`,
where the pinned native island uses N=20+5=25 solver substeps. It starts with a
backward COM velocity of −0.00305 m/s at 83 ms but still falls, and the delivered
joint torques do not match the reduced model sufficiently. It is rejected.
Changing gain values or the qualification thresholds is not the remedy.

The final discriminant replaces the reduced posture model with six free-base
accelerations plus all **46** canonical actuated coordinate accelerations.
Body Jacobians follow the actual canonical frames; force/moment requirements
include mass, inertia, gravity and velocity bias. The current native material
contact points also impose velocity-level sticking consistency. Joint limits,
compressive frictional contact and the native motor envelopes are constrained;
the cap installation explicitly clamps every axis to its original nominal
ceiling. This makes the kinematic assumption explicit rather than inventing
independent accelerations for bodies connected by joints.

This stricter model executes only 37.5 ms, then refuses a request. The native
COM velocity at the last recorded 33.3 ms frame is −0.02195 m/s forward-axis
(backward); pelvis displacement at refusal is 3.51 mm. There is no integrity
failure, no stable preparation, no rise, no original handoff and no recovery.
The QP/mode refusal is not a proof that the actual feet-only mechanism is
impossible. The exact refusing constraint class has not yet been isolated;
`full-body-qp.json`, `full-body-qp-probe.rs` and its saved binary preserve the
result. No further controller variant was opened after this requested
checkpoint.

### Decision before further implementation

A validated full controller is not ready for ordinary integration. First audit
the rejected full-body constraint system independently: distinguish numerical
solver failure, inconsistent currently-sticking contact rows, native joint-limit
transients and torque-envelope infeasibility. If the assumed sticking set is
incompatible, revise the contact-mode decision (pivot, release, heel acquisition)
and solve a complete dynamically feasible route before another native trial.
Prepared static success does not license a posture-only route. An intermediate
hand/knee remains a candidate, not a proven necessity. Do not continue a blind
sweep of poses, phase timings or gains.

The only retained production change is the measured-contact balance correction.
It removes the witness's false `inside` conclusion at 50 ms, while retaining
braking toward a point/line and the unchanged inset qualification predicate.
Its full 25 s crouch traces at both headings still finish in Brace with six
retries and zero recoveries. Initial two-second repeats are exactly identical.
This correction addresses truthful support/admission; it does not repair the
recovery dynamics. The earlier forward acceleration can be worse, and is
reported in `support-patch-audit.json`, not hidden.

All controller experiments, including incomplete, free-pivot, reduced coupled
and full-body variants, are archived outside the final Rust source. The retained
ignored probes are read-only trace capture and zero-integration geometry
inspection. No tests/limits are weakened. The final 24 focused recovery tests
pass, as do Clippy with `-D warnings`, formatting after trailing-whitespace
cleanup and `git diff --check`. Their receipts are in `final-*.log`. No complete workspace, hybrid-articulation, TypeScript, WASM or
browser suite is claimed. The parent's reported pre-existing hybrid test and
four TypeScript failures remain unresolved here.

Replay exports `*-replay.json` include all 25 initial/final poses, native pose and
velocity frames, all canonical non-Euler coordinates, resolved loaded contacts,
requested torques and native delivered impulses. `native-effort-audit.json`
counts only the 46 configured anatomical axes; inactive native motor slots have
sentinel ceilings and must not enter the aggregate audit. Old unqualified
variants exhibit float-rounding ceiling sums of about 760.00008 Nm; these are
reported literally, not used to relax the 760 Nm bound. The final full-body
variant reserves a conservative QP margin and clamps nominal axis ceilings, but
its native cap sum still rounds to 760.000152 Nm. No variant is qualified on
this audit; a future installation must retain the original aggregate bound
without rounding it upward.

The branch remains local and uncommitted on the exact requested reference commit;
no push, merge or publication occurred. Incoming parent corrections were never
materialized and are not claimed as present.

Isolated controller probes call the native world directly; their constructor's
nominal PGS setting is 20, while the original advance pipeline may select 32
while falling. They do not execute ordinary motion/controller observation
unless an explicitly recorded original handoff is reached (none is reached
here). Consequently their positive prepared holds are native physical screening,
not frozen fixture acceptance. The retained production correction's 25 s
traces do execute the original complete advance pipeline. Input profile values,
anatomy, gravity and joint/contact definitions are unchanged.
