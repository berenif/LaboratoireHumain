# Recovery-step preflight: no physical rollout yet

Publication availability: this branch keeps text results, configurations, ledgers and checksums. Full compressed traces, checkpoints and video/image payloads are retained in the original local investigation history, rather than published here. See [publication record](../publication-2026-10-17.md). Historical archive paths below describe those retained local files.

The exact retained 10 s lateral checkpoint is the representative shuffling
interval, before CoM/capture enter solved support. Its unchanged native standing
predicates currently fail. An explicit `--upright-entry` permits a diagnostic
experiment from that state; it does not relax success, integrity or termination
criteria. The normal standing-only experiment entry remains the default.

## Measured without integrating

- Pure forward kinematics reconstruct all recorded body transforms within
  6.73e-7 of the snapshot, using the original model and current joint angles.
- A requested 2.5 cm foot-origin lift yields only 6.0 mm right / 7.5 mm left
  whole-capsule clearance in the limit-respecting mathematical leg probe.
- A 6 cm origin lift yields 2.67 cm right / 2.90 cm left clearance, with
  approximately 2.4 cm position error because the original ankle limit prevents
  preserving the requested foot orientation. This is reachability evidence,
  not an executed physical step.
- Conditioning frozen SLOW on that right-leg lift changes unrelated outputs by
  up to 0.569 rad. The desired hip-reference change is -0.222 rad, while the net
  effective hip-target change is +0.126 rad. The effective swing ankle target
  remains below its limit. The represented residual does not overflow and its
  motor controls match the custom-reference PD controls at this state.

Consequently the first proposed physical mechanism test is explicitly **hybrid**:
keep the learned frozen SLOW reactive on the stance leg and upper body, and
blend only a naturally unloaded swing leg toward a legal IK target. This is not
a learned stepping skill or a default controller replacement.

## Bounded executable probe

`scripts/probe-getup-recovery-step.py` admits one capture-directed step (at most 6 cm) only
after a foot is naturally unloaded below 3 N and the other supports at least
70% bodyweight. Horizontal progression waits for measured 1.5 cm whole-foot
clearance. Early native recontact or lost stance support triggers a smooth
50 ms return to source control at the next 40 Hz decision; this is not an
800 Hz within-tick emergency stop. There is no root actuation, foot constraint,
pose/velocity write, new friction, joint-limit change, teacher forecast or
changed network weight.

The experimental native response retains the actual twenty 800 Hz foot samples
per decision, rather than inferring support continuity from visual frames. Its
clearance uses actual capsule endpoints and radii, replacing the compound-AABB
approximation only in experimental measurement.

The default paired proposal is eight simulated seconds per controller from the
same 10 s checkpoint. Acceptance requires a true cleared unload–land step of at least 5 cm,
maintained stance contact, improved support/capture, no flight/non-foot/integrity
failure, and five continuous seconds satisfying unchanged standing and 1 cm
foot-anchor criteria. Compare loaded contact slip and parasitic repositions with
the paired baseline. Capture/support histories and the common post-intervention
window (starting 0.5 s after source handoff) keep that comparison time-matched. Any later robustness/learning claim requires further
held-out cases and video review.

`--execute` is required to integrate. Without it, the recorded preflight consumed
**zero physical wall seconds and zero native integrations**. Its zero allowance
and deadline authorize no later run. A physical execution requires its own
explicit allowance and future deadline, carried through the single-process
aggregate ledger. No rollout, training, default replacement or deployment has
occurred at this stage.

Twelve pure tests cover admission, untouched non-swing actions, clearance gating,
native recontact, smooth release, uninterrupted phase blending, native stance-loss/phase-reset guards and canonical
kinematics. Per-joint requested/represented targets expose residual clipping; the
exact first zero-override source-handoff tick is retained. Interrupted execution
is recorded as incomplete with its error and conservative ledger, never as a
zero-physics preflight. Synthetic interruption and clipping regressions test
those reporting paths without a physical integration. The 61 existing npm
tests also pass. `reference-response.json` retains every response, including the
unfavorable reference sensitivities.
