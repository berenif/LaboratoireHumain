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
