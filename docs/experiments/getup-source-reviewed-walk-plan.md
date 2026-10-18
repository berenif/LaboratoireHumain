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
