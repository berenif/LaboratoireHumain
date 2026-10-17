# First Python source-adaptation pilot

The five-reviewed plan is implemented through source-interface parity, short
native diagnostics and disabled task scaffolding. **No complete step or trained
step actor is qualified.** The pilot began on 2026-10-17 at 22:34:04 UTC. Its
one-hour / 300 s new-physics bounds and protected validation reserve still apply.

## Measured outcomes

- P0: Python/native observations, full 42 actor outputs, 21 actions and both PD
  damping branches match. The rebuilt telemetry-only binary repeats these
  zero-integration checks.
- P1: 280 real native integrations, 0.903372796 s physics. Legacy hold and explicit
  source hold evolve identically. Moving and teacher-interface saved
  continuations match. Maximum actual torque equation error is 3.37e-6 Nm. The
  torque branch calibrates an already-supported command, not a learned teacher.
- P2: frozen SLOW with actual-pose / actual-height / actual-vertical targets
  stopped after 200 native frames (0.25 s) for 21.48 mm horizontal COM drift.
  Flight and non-foot support stayed zero; loaded slip was about 2.39 mm per foot.
  No fall was observed or claimed. A single exact recorded-command replay
  recovered a missing failed-state checkpoint and matched every sample. It is
  charged. Substituting the old terminal input's 1.4 m sentinel would change the
  tested task and is not used to hide this OOD result.
- P4: zero-residual moving-source-PD preparation stopped after 160 native frames
  (0.20 s) for 24.23 mm COM drift. Flight and non-foot support stayed zero;
  cumulative slip remained below 10 mm. Actual torque matched the source
  equation within 2.28e-6 Nm. Entry pose/twist matched except for the declared
  elbow target correction. This tests uncompensated PD response, not reference
  infeasibility or a learned teacher.
- P5: imported final WEAK teacher68 at its saved power 0.663420431 stopped after
  60 native frames (0.075 s) for right loaded slip of 11.88 mm. Flight and
  non-foot support stayed zero. Raw/action parity and torque parity passed
  (maximum torque error 1.91e-6 Nm). This is a quiet SLOW/native initial-state
  distribution probe; no separate strong-stage asset is available or tested.
- Total new charged physics: **2.930621222 s, 900 integrations**. Historical
  research charged 593.400428476 s separately and is not reusable allowance.

## Implementation checks

All 62 Python tests pass with the existing SciPy and Torch CPU installations.
Nine Rust source/bridge tests pass; the original four observer tests passed in
P1. Source math preserves the 67/118 frames, actuator-order torque versus
qpos-order residuals, moving damping 0.1 / holding 0.2, reward/end boundary quirks
and original caps. A synthetic test matches one unchanged SAC update against
explicit target, critic, actor, temperature and Polyak equations. Throwaway
random-network tests are not task training.

R1/R2/R4 reviewed the dimensioned task reward/config scaffolding. It validates
error widths, frames, event order, finite partial physical terminals and masks.
Goal success and intrinsic phase deadlines are absorbing; external collection
cutoffs bootstrap. This is an explicit adaptation from the source fixed horizon.
**Collection and optimization remain disabled.**

The forward reference uses own static whole-body witnesses and a separate
prospective load-acquisition path. Pure FK/dense IK generates uniform 40 Hz
arrays, source-frame root twist, true COM/vertical, valid terminal quaternions
and five seconds of zero-twist padding. Advance clearance is about 25 mm, but
this is planned geometry, not a qualified native rollout or teacher dataset.

The first 0.5 s is an actual-twist quintic/SO3 bridge. Recorded elbow20 starts one
f32 neighbor below its installed lower limit while closing. Only its target is
held at that installed bound with zero reference twist; measured state and PD
damping stay real. Other 20 joints/root match actual initial twist. Temporary
right-foot target drift is about 141 µm / 0.0233 degrees. No physical foot pin,
enlarged limit or planned stop-force support is added. Initial ankle7/hip11
planning-margin exceptions are recorded separately.

Checked sidecars hash native bytes, adapter/FK/model/actor inputs and reference
configuration, validate phase/clocks/counters/events/anchors, verify exact
observed state/snapshot and never rewind the ledger. Entry-only deterministic
roundtrip passes. Full stochastic RNG/replay/optimizer restoration and each
later phase's continued physical-evolution parity remain open.

## What blocks task collection

1. The actual per-native support-geometry/error reader must feed the measured
   event/reward code. Missing margins are rejected, not filled with an old hull
   or prospective contacts. Synthetic event tests do not qualify real support.
2. The event clock, reference, reward and source command runner need end-to-end
   wiring and checkpoint-continuation tests, including full stochastic state.
3. A native-qualified teacher/control initialization is still needed. Frozen
   SLOW, zero residual and imported WEAK stance probes cannot be promoted.
4. Successful actual WEAK rollouts must precede SLOW task data/training. There
   is no optimizer or collection admission, no full-step proof and no browser
   product claim from this pilot.

Compact reports: `docs/evidence/getup-python-step-2026-10-17/pilot/`.
Static inputs: `docs/fixtures/getup-step/`, regenerated with
`scripts/getup-step/build_reference.py`. Raw native checkpoints/journals remain
local; their hashes are not remote retrieval links. Final actual-motion delivery
still needs the reviewed accessible artifact or deterministic reproduction route.
