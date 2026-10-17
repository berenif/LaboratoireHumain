# Actual-contact balance and partial stepping, 2026-10-17

Publication availability: this branch keeps text results, configurations, ledgers and checksums. Full compressed traces, checkpoints and video/image payloads are retained in the original local investigation history, rather than published here. See [publication record](../publication-2026-10-17.md). Historical archive paths below describe those retained local files.

Authorized session: 13:30:42–14:30:42 UTC, with all physics stopped by14:25.
Actual final integration ended14:18:52 UTC. No further physical run was made.

## Result

A measured-state whole-body controller achieved the missing weight transfer.
A complete, qualifying natural step **was not achieved**.

- The corrected zero-shift controller held12 s native anchored standing.
- The matched5 mm sign test moved actual COM4.796 mm and capture5.147 mm
  beyond the zero-shift control. Its timed return to original SLOW failed the
  five-second anchored-standing gate, including a separately retained exact
  six-second continuation. This old handoff gate remains failed.
- Under the separately approved GetUp→persistent-balancer architecture, a3 cm
  shift achieved29.877 mm actual COM displacement,30.728 mm capture movement,
  and59.17% right load. All12 s were anchored standing; final five-second
  return held, with1.59/3.43 mm loaded slip and no flight/non-foot support.
- A larger measured transfer achieved100% right support and1.805 s actual
  left unloading. The old timed COM return then destabilized the free leg.
- A contact-gated swing subsequently produced actual whole-foot lift and a
  six-centimetre placement. It did not complete landing and returned standing.
  Batch08 was cleanly stopped at9.675 s. The20 Hz recorded poses show26.4 mm
  maximum whole-capsule gap and66.0 mm foot-origin movement from pre-lift;
  these are **not** a completed unload–land event.
- An isolated COM-task priority ablation,10→100, reduced capture escape during
  placement, but batch09 stopped on solver termination at9.625 s before
  landing. Right loaded slip18.27 mm already exceeded the unchanged1 cm gate.
  No flight/non-foot support was integrated in either controlled-step trial.

The unchanged acceptance remains actual>=5 cm unload–land displacement,
>=1.5 cm minimum whole-capsule clearance during horizontal placement, safe
measured landing, no prohibited support/flight, loaded slip<=1 cm, and>=5 s
post-step native anchored standing. No threshold was relaxed after a result.
The representative original10 s shuffling checkpoint was not repaired or
retested by this quiet-state mechanism proof. No learned new skill is claimed.

## Controller and causal findings

The experimental controller uses actual generalized velocities, mass/COM
Jacobians, golden COM inertia, native armature/passive springs/damping, actual
contact material points, signed near-bound stop reactions, and hard floating
force/moment dynamics. Nonnegative contact normals and a conservative friction
pyramid, joint-acceleration guards, original motor limits, and represented
residual limits are retained. These contact/stop variables exist only in the
optimizer. The engine receives ordinary21 motor residuals at40 Hz. It retains
800 Hz inner PD, real contact solving and all original joint limits. No root
force, body pin, root/velocity/pose write, or simulated contact assistance is
used. Default runtime and frozen policy weights are unchanged.

The continuous dynamics model is an approximation to native implicit passive
integration. Held residuals represent solved torque exactly at decision time,
not at every subsequent native substep; actual800 Hz torques are retained.
Near-stop tolerances are predictive, rather than strict instantaneous
complementarity. Normal acceleration is hard and tangential stabilization soft.

Independent exact-state replay at batch08 t9.125 reproduces the old outward
COM acceleration+0.086711 m/s² despite desired braking−0.300. Priority100 gives
−0.286674, within all hard constraints and41.319% maximum motor fraction.
At t8.725 it similarly corrects the opposite-sign acceleration without hitting
motor limits. This justified the isolated ablation; it did not qualify a step.

Batch09's final SLSQP linesearch status is **not proof of infeasibility**:
its reported equality error1.79e−7 and friction error9.04e−10 are small.
Independent reconstruction also confirms acceleration, motor/residual, contact
force and stop-reaction bounds within stated numerical tolerances. The iterate
is hard-feasible but not proven optimal, so this is a solver-status rejection.
The actual right foot is rolling rapidly, while the all-normal-equality model
allocates substantial force to an opening point. Opening speed alone does not
prove the point must release: it is still approximately0.199 mm below the
floor, and native penetration repair may require impulse. Exact paired gap
and native contact bias are not exported. The next causal investigation is
those quantities and explicit contact active/release complementarity, checked
against actual native impulses. Sole flattening is a separate untested idea;
its optional code was removed from the final implementation, and no flat-sole
physical experiment ran.

## Evidence, budget and disposition

`session-ledger.json` is authoritative:259.739657291 physical wall seconds,
78,979 native substeps, zero uncertain outstanding physical requests.
Sum of probe process wall time335.101065571 s includes physical work and
other probe overhead; solver27.493924983 s and state reconstruction19.900875704 s
are also reported separately. The one-hour session additionally includes
build, implementation, review, rendering and archiving. No old unused allowance
was carried forward, and no training/paid remote service ran.

Each batch retains exact config/hashes, budget, summary, executed controller
copies, full compressed JSONL commands/native samples, frames/control history,
and final checkpoint. `archives.json` hashes original and compressed bytes.
The original quiet seed is included. Failed traces were not overwritten.
Current scripts contain final diagnostic-only bound reporting; each batch's
executed copies/hashes preserve its precise earlier version. These copies are
evidence, not fresh authorization or automatically runnable from this folder.

Pure checks, zero integration (33 tests, all passed):

```
OPENBLAS_NUM_THREADS=1 python3 scripts/getup-contact-dynamics-test.py
OPENBLAS_NUM_THREADS=1 python3 scripts/getup-contact-step-test.py
python3 scripts/probe-getup-recovery-step-test.py
python3 scripts/probe-getup-equilibrium-test.py
```

The native release build passed. Its only Rust change adds actual actuator
motor torque telemetry to the experimental800 Hz sample. No new physical
source-getup regression was claimed; existing source evidence is preserved.

Videos use actual recorded poses only, with no interpolation or repeated tail.
The matched zero-shift baseline is the measured balancer followed by original
SLOW after its scheduled release, as labeled in the video.

- Stable3 cm comparison: Library `libfile_bc0200720ac48191ac1e5391e052630d`.
- Honest partial-step comparison: Library `libfile_e990249acaa08191a297fc0141e6f77d`.
  The candidate becomes blank after its physical stop; final balance is not
  implied. Render reports and decoded-pixel previews are retained.

These experimental probes are not enabled in the application. The original
functional frozen GetUp remains the default and reference. No merge/deploy or
remote push was performed in this session.
