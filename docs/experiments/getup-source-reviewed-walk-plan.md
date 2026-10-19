# Five-reviewed gait correction implementation

Status: experimental, opt-in, unqualified. Stable preset is unchanged.

The delivered 1× video demonstrates four own-foot advances ≥180 mm, not ten.
The later explicit contact impratio=8 witness completes six advances and stops
at the seventh transfer, 49.4625 s, on the unchanged per-foot 10 mm loaded-slip
limit. Its 7.63375 s median loaded-landing interval is slower than the delivered
video's approximately 6.66 s. Neither establishes the requested final result.

## Five actual reviews incorporated

1. Calibrate thorax/pelvis axes and distinguish visual hypotheses from anatomy;
   measure the mobile knee, respect the rigid-foot geometry, and preserve 1:1
   opposition in the instantaneous thorax frame.
2. Preserve existing strict arm gates; use the same prepared references in real
   and prospective-unload QPs; admit posture dynamically with motor/ankle reserve.
3. Audit every native sample and global slip; separate own-foot advances from
   body travel; compare like loaded-landing events at physical speed.
4. Implement shared-reference parity first, then continuous COM/load progression,
   then posture, coordinated knee/foot references, and cadence, one factor at a
   time. Do not hide the six-step failure by stacking unisolated changes.
5. Preserve loaded-settle midpoint+35 mm and measured 80-sample readiness,
   distinct unload <0.5 N and airborne <3 N gates, idempotent preparation, and a
   separately versioned audit for diagnostics and the changed gait.

## Ordered implementation and gates

- Immutable context: prepare delayed measured-thigh arms and contact-seek once
  per native command; actual and hypothetical single QP assembly consume it.
  No class cast dropping subclass tasks, recursive observer, or forecast rollout.
  Native/phase/reference identities, causal derivative anchors, sources and
  controls are bound into receipts; legacy no-context behavior remains exact.
- Continuous COM: preserve times, posture and loaded-settle readiness. Plan C2
  references inside each applicable actual loaded-support/selected triangle
  inset, including reference capture. Carry full jets across phase and step
  handoffs. No instantaneous per-command projection; refuse infeasible paths.
  Freeze witness total speed/acceleration envelope before overlap. Add a soft
  load preference or earlier robust selection only as separately audited factors.
- Posture: calibrate forward-positive, heading-removed thorax-world,
  pelvis-world and thorax-relative-pelvis; search jointly reserved references
  before soft separate body tasks. +3–5° thorax and ≤15° pelvis are hypotheses.
- Knee/foot: early mobile-knee flexion then controlled extension through feasible
  joint/foot jets; keep ≥25 mm whole-foot clearance for 80 samples before advance.
- Cadence: only after preceding dynamic gates, shorten one phase parameter at
  a time without changing measured admission windows, reserves or deadlines.

Each isolated factor requires real one-step+5s and two-alternating-step+5s
pilots with executed independent audits. Two/seven-step runs are diagnostics,
never final qualification. Before posture, exceed the former failure horizon
and admit at least step seven without a guard violation.

## Final acceptance

Ten alternating own-foot advances ≥180 mm in one uninterrupted world, then
4,000 consecutive quiet samples at 800 Hz. Retain original motor ±0.95,
friction/geometry, all joint/collision/ray/support/capture gates, global loaded
slip ≤10 mm per foot, delayed arm maximum error ≤0.5°, excursion ±5%, correlation
≤−0.98, exact target/FK checks and persistent origins. No state installs after
initialization, root assistance, slip reset, playback timewarp or preset promotion.

Also require physically faster loaded-landing intervals and improved actual-speed
front/profile pixels: less waiting/lateral sway, earlier mobile-knee flexion and
coordinated pelvis/slight forward bust. Publish all intervals and phase metrics,
common prefix and whole sequence, both sides, and missed tuning hypotheses.
Report foot separation, COM progression and body travel; ten own-foot advances
do not establish 1.8 m body travel. A physical-only, slower or visually unchanged
result is partial. Explicit impratio=8 never qualifies the original contact model.

## Experimental implementation checkpoint

Shared immutable references and the first continuous-COM factor pass independent
one-step+5s and two-step+5s diagnostics. Same-contact cadence is unchanged. The
seven-step continuation stops on the original global-slip guard during seventh
landing at53.78375s: six loaded advances, left/right slip10.00653/8.51826mm,
zero final quiet. Its targeted failure replay is consistent; no full passing
qualification is claimed. This exceeds the former failure horizon without
completing the required seven-step gate, so posture remains unstacked.

The next opt-in factor projects one planned COM endpoint toward the planned
landing/stance midpoint using only the current actual support. It preserves
per-face origin margins up to4mm and retains every-command3mm validation, C2
jets, the frozen total envelope and the original measured settle target. The
legacy10mm mode stays unchanged. Forty pure projector tests pass; physical
pilots and independent new-mode replay remain required. The compact evidence
receipt records exact hashes, partial flags and known limitations.

The projected-endpoint one-step trial finishes its5s quiet but increases both
feet's cumulative slip (1.611/1.641mm versus1.289/1.497mm). It is rejected,
not independently physically qualified or promoted; no two-step expansion is
justified. The next finite trial keeps the validated10mm COM policy and adds
only a weak acquired-load-retention squared hinge. A freely absorbing slack,
fixed0.0005N^-1 weight, original corner/shear costs and all admissions remain.
If its matched effect is negligible, do not spend long continuation runs on it;
prioritize earlier state-valid causal selection without shortening history or
measured admission gates.

The fixed soft-load one-step trial also finishes five quiet seconds, but left/
right slip increases by0.026%/0.010%; it is rejected without a two-step expansion.
The next isolated mode, `continuous-com-causal`, retains the validated10mm COM
path and starts risk observation/selection from actual loaded-state eligibility
rather than nominal transfer timers. It still requires the complete100ms/21
command history, risk agreement, actual three-point survivors, original bounded
correction, minimum transfer and all measured lift gates. An exact-rational
piecewise-polynomial certificate must admit the summed nominal/corrective COM
speed and acceleration within the unchanged frozen envelope before selection.
After selection, every transfer command also validates its complete reference
against the current actual support hull. The inherited preselection startup is
unchanged. Seven retained nominal-transfer prescreens and151 focused tests pass.
The one-/two-step pilots finish their five quiet seconds, with the first landing
0.9975s earlier and the alternating interval6.870s versus7.63375s. This remains
slower than the delivered video's approximately6.66s. Two-step cumulative slip
is2.98037/2.43396mm. Independent new-mode qualification remains pending; one
seven-step gate continuation stops during seventh landing at48.215s, with six
loaded advances and left/right slip10.00760/8.61442mm. The one-/two-step reviewed
audits pass, including21/42 exact native witnesses; no ten-step/final pass exists.

## Observed-limit sequence revision

Stage-one COM/load/selection tuning is closed. Its repeatable cumulative-slip
limit does not justify indefinitely blocking the requested visual correction.
Permit isolated posture and coordinated knee/foot one-/two-step pilots before
the seven-step ordering gate, with independent audits and every actual physical
and arm gate unchanged. Do not stack an unqualified factor or promote a preset.
The combined final acceptance remains ten alternating advances plus4,000 quiet
native samples, faster cadence and improved full actual-speed front/profile
pixels. This is a sequence revision, not a relaxed final test.

The coupled static seeds remain marginal and require fresh current-support LP,
geometry and full-jet checks. Their initial reference inherits a0.020rad ankle
reserve; the improved endpoint requires0.060rad. A C2 transition must report that
distinction rather than invent a reset or assert endpoint reserves throughout.
The fixed-root knee family is rejected; moving pelvis/root transport must be
included in a realizable hip/knee/ankle-foot pair.

The shared controller now has a pure final-context visual hook before both QP
consumers, with measured-arm identity and original capture/unload ownership
enforced. Separate C2 pelvis/thorax and moving-frame coordinated leg/foot modules
are implemented;66 focused integration tests pass, including exact disabled QP
parity, FK/IK jets, contact-seek and waits. The leg math fixtures are not admitted
source gaits. The first actual-state posture retarget has a reserved static
endpoint and a3mm-support-bounded COM handoff; native posture pilots remain
pending their explicit current-version causal admission.

The isolated native posture entry point now freezes one current-foot-matched
+1° thorax candidate after the nominal transfer reference finishes, using one
bounded static family and an independent counter-zero recheck. Its provisional
triangle grants no admission. A new complete100ms risk window must confirm that
same triangle under the changed references; the frozen candidate is never
retargeted inside the window. The total COM overlap is certified before selection,
with current measured support checked every command. The initial ankle-reference
floor is explicitly inherited, while the endpoint requires0.060rad. Separate soft
thorax/pelvis tasks activate continuously, delayed arm targets remain identical,
and C2 lower-joint/SO(3) references return during the original measured settle.
This is a new unqualified profile, requiring its own native pilot and replay;
static tests and the old profile cannot greenlight it.

The first native posture attempt stops safely at4.000s before applying a changed
posture: its reserved endpoint passes every static check, but the old nominal
COM is2.073mm outside the proposed triangle at the handoff's start. The complete
3mm path gate refuses it. The next isolated profile therefore uses the already
qualified early-selection baseline, whose correction is complete before4s.
That completed COM offset and safety geometry are retained provisionally; the
posture path embeds the offset exactly once. The earlier window cannot admit
posture lift: a separate complete100ms window under the new tasks must confirm
the same triangle before the unchanged lift gates. No retarget, new correction,
physical deadline change or weakened support gate is introduced.

The causal-baseline posture trial acquires its own21-command history at4.1s,
passes the original80-sample unload/clearance/landing gates, and records a real
181.484mm loaded advance at8.98s. It stops at9.175s in settle, with0s quiet and
0.659/0.711mm global slip, when the complete reference acceleration exceeds the
frozen cap. The targeted replay checks all7,340 native rows,22 source bindings,
and exact reuse of the qualified baseline prefix through3,200. No passing audit
is claimed. Actual-speed matched pixels show pelvis pitch improving from−30.53°
to−17.37° in mid-advance; torso total tilt stays~9.3°, and landing is1.85s later.

Its unchanged measured-target return is181.927mm long. The old1.52s quintic peaks
at0.224416m/s and0.454620m/s², exceeding both frozen caps. A single analytical
seven-cubic C2 acceleration/coast family fits the SAME target,1.52s duration and
caps, with25.271mm conservative current-hull/capture margin. It preserves the
legacy path object when that path already fits, rejects genuine nonzero incoming
jets, and retains every-command current-support checks. This is a new opt-in
profile and must complete a fresh native pilot; no reserve or deadline is widened.

The bounded-settle posture pilot physically completes one 181.484 mm loaded
advance at 8.98 s and 4,000 quiet samples, with 1.418/1.180 mm global loaded slip.
Its separately versioned independent audit passes all 12,402 native rows,
3,101 commands and 42 exact native witnesses. The original clock-expectation
failure is retained; an independent all-command proof verifies the new complete
settle owner embeds the completed correction once, rather than adding a second
faded offset. The loaded prefix is exactly unchanged from the preceding trial.
Actual-speed pixels show the mid-advance pelvis improvement, but final pelvis
pitch is −18.24° versus −2.64° baseline, and the first landing is 1.85 s slower.
This is a physical checkpoint,
not completion of the requested visual, cadence or ten-step result. The pelvis
task remains active at weight 4 through quiet; inspect its coupled return target
and achieved tracking before choosing a correction.

Two finite moving-root/both-leg endpoint attempts also fail: the hard-thorax
family misses the swing foot by 29.432 mm; the original-soft-thorax family misses
it by 16.440 mm and produces only 6.885° of the requested 10° knee gain, with 23.750 mm
planned clearance. Neither is admitted. Moreover, the inherited weak-joint
reference is off the simultaneous task-FK manifold by 3.469 mm COM and 2.646 mm foot
position. A pure declared C2 compatibility-slack bridge now preserves the real
prepared command at activation; no pose or velocity projection is permitted.
Do not repeat goal/weight scans. The approved substantive planning revision now
uses one stance-realized whole-body trajectory, allows its COM to move inside
the unchanged actual-support/capture and total-derivative bounds, and derives the
coordinated COM/body/knee/foot jets from that trajectory. This revision needs
explicit reference ownership, current-state endpoint/path admission and native
one-/two-step evidence before combining it with ten-step acceptance. One causal
same-foot quiet endpoint matches feet/COM/heading to 7.95e−14, but its first C2
return from the old posture landing exceeds the unchanged acceleration cap at
0.2375 s (0.43010 m/s² versus 0.39018). Its endpoint and return are distinct results;
no native use follows from geometric feasibility. One finite free-COM full-flight
horizon is now being solved, with exact Cartesian foot commands and analytical
FK jets; no weight/goal grid, root installation or duplicate invocation is used.
