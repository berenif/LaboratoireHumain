# Bounded recovery-step session, 2026-10-17

Publication availability: this branch keeps text results, configurations, ledgers and checksums. Full compressed traces, checkpoints and video/image payloads are retained in the original local investigation history, rather than published here. See [publication record](../publication-2026-10-17.md). Historical archive paths below describe those retained local files.

Session authorized for 12:11:31–13:11:31 UTC; physical work stops at 13:06.
All tested interventions are experimental, untrained hybrids around frozen GetUp.
The original body, motor limits, friction, policy weights and default runtime are
unchanged. No root force, body pin, pose/velocity write or contact assistance was
used. Static contact-force/limit-reaction calculations are planning only.

## Final result

No qualifying step has been achieved. The preregistered target remains a 6 cm
placement with actual >=5 cm unload–land displacement, >=1.5 cm whole-capsule
clearance, safe support, improved capture/support, and >=5 s returned standing.

1. A natural-unload gated IK override recontacted before lifting and worsened
   standing/slip. The matched frozen baseline stood for 5.999 s.
2. Rising/persistent unload plus a sagittal lift reached 4.93 mm clearance,
   then recontacted and escaped its anchor. Legal absolute targets suppressed
   learned hip thrust and the existing PD damping opposed knee flexion.
3. Adding the IK displacement to the learned action preserved that bias, but
   lifted only 2.42 mm and produced two native flight samples. It later stood
   for 5.056 s. The short transient-unload strategy was rejected.
4. Deliberate quiet-state joint-bias transfer approached the support target by
   2.85 cm, with right load share at most 66.8%; no flight or fall.
5. Original SLOW now/ahead reference conditioning improved approach to 5.79 cm,
   but right load reached only 70.3%; it regained 6.504 s anchored standing.
6. Extending the identical hold disproved a simple lag explanation: right
   load plateaued below 72.2%, never sustaining the preregistered 85% share.
7. A new inverse-statics + recentered frozen-SLOW feedback controller passed
   its **zero-shift prerequisite**: the full-model 1–6 s hold maintained exactly
   5 s native anchored standing, no flight/non-foot support, 0.733/0.728 mm
   loaded slip and 9.998 mm maximum horizontal COM drift. This is not a step.
8. Full dynamic transfer then failed at 4.034 s, despite static feasibility:
   the actual COM/load did not follow the planned lateral shift. There were 27
   native flight samples; no source-return or qualified step was reached.
   Clipping appeared only after instability, not at its onset.
9. The final same-path/controller guarded-retreat ablation paused progression
   when measured COM/capture/load lagged. It returned from only 13.79% of the
   intended path. There was no flight, non-foot support or action clipping;
   exact original GetUp commands resumed at 6.525 s. The source-only 10–16 s
   window held 6.000 s anchored standing with 1.36/1.37 mm loaded slip. This
   validates safe retreat in this case, not a completed transfer or step.

Physical experiments stopped at 13:02:14 UTC. The final aggregate ledger is
373.019794112 physical wall seconds, 115,991 native substeps, with no uncertain outstanding physical requests. Old
unused allowances were not carried into this session.

## Model-based positive control

Static planning uses true primitive mass COMs, floating-root force/moment
balance, nonnegative vertical contact forces, motor and represented-action
caps, passive springs, and correctly signed unilateral lower-limit reactions
only at active bounds. Both loaded ankles initially use real joint-stop
support. Releasing a bound prematurely is not a valid inverse-static model.

The controller is source(actual) + blend * (qref - qnom + motorFF/gear -
source(zero-velocity-FKref)), with all ordinary action clipping retained. The
synthetic physical first37 fields agree with native observation within 7.93e-7;
NumPy/native actor parity is 4.67e-7. Full-model zero-shift hold was tested before
any planned transfer. Reference path progress and model handoff are separate.

For the planned transfer, 41 optimized knots and the actual 40 Hz schedule are
validated offline. Joint-arc timing bounds reference speed to 0.219 rad/s;
interpolation foot-origin error is 0.735 mm, motor-equilibrium error 0.0337 Nm,
and root-moment residual 0.0171 Nm. These are static planning checks, not proof
of dynamic stability or natural movement.

## Reproduction and retained evidence

Each batch config contains exact checkpoint/binary/script hashes and its cap.
The single-process budget ledgers and session aggregate retain consumed time.
Compressed JSONL protocols retain every command/action and actual 800 Hz foot
sample; compressed case JSON retains frames, control notes and support/capture
history. Checkpoints remain in the recorded workspace locations with hashes in
archives.json. Failed candidates were not substituted or deleted.

The pure recovery-step suite passes 15 tests and the pure guarded-model suite
passes 5 tests (20 total, zero physical integration). The unchanged native
build and original npm/Rust preflight regressions are recorded separately in
../getup-recovery-step-preflight-2026-10-17/. Rendering uses recorded poses only;
no interpolated or repeated final poses may imply unmeasured stability.

## Disposition and reproduction

The requested improved natural step was **not achieved**. Only zero-shift model
holding and one guarded retreat passed their limited prerequisites. Dynamic
support transfer remains unresolved, so these hybrids are not enabled in the
application or presented as learned stepping. The known-good frozen GetUp
controller remains the reference. No full RL fine-tune was run in this hour.

The final failed-transfer comparison is a recorded 15 s same-checkpoint pair:
`outputs/getup-recovery-step/session-20261017T121131Z/render/batch08-equilibrium-transfer/support-transfer-comparison.mp4`.
The baseline is continuous actual recording; the candidate is blank after its
4.034 s physical stop rather than holding a terminal pose. Its Library upload
is `libfile_e1109551b6208191a413634459b68589`.

Pure checks, from the repository root:

```
python3 scripts/probe-getup-recovery-step-test.py
python3 scripts/probe-getup-equilibrium-test.py
OPENBLAS_NUM_THREADS=1 python3 docs/evidence/getup-recovery-session-2026-10-17/quiet-transfer-offline/validate-equilibrium-schedule.py
```

The archived math helpers resolve the repository and their own evidence
folder portably; this is a path-only adaptation of the retained session helpers.
Their JSON inputs preserve the initial observation, root, contacts and model
planning constraints. Static optimization/validation integrates no dynamics.
Physical reruns require a separately approved fresh allowance and deadline;
expired configs are evidence, not future execution authorization. Complete
JSONL action traces permit deterministic ordinary-motor replay without relying
on the current exploratory controller's later edits.
