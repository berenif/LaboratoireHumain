# H77-v1 — implicit native posture feedback

H77 is a separately versioned, opt-in successor to H74. H74/H75 remain
reproducible failed experiments and their manifests, source, and evidence were
not overwritten. H77 was **not promoted**: its frozen feasibility stage fails,
so the normal room and playground startup paths remain on the legacy controller.

## Implemented controller identity

For every anatomical motor axis, H77 retains H74's bounded request

```text
u = clamp((kp * error - kd * rate + taskTorque) / (1 + regularization), +/-cap)
```

but configures Rapier with the real reference, nonzero `kp`, and the existing
`kd`. The native feedforward term is only
`u - kp * configuredError + kd * rate`. This reproduces the current-state H74
request while allowing native coordinate/rate feedback inside each solver
substep. It does not change body count, degrees of freedom, limits, motor caps,
collision ownership, solver settings, fall guards, or the zero direct-pelvis
force/torque rule.

The application and harness selector supports explicit `legacy`, `h74-v1`, and
`h77-v1` identities. H77 reports authority (`standing`, `suspended`,
`reacquiring`, or `transition`), stance revision, step trigger reason, native-PD
request, residual feedforward, readback equivalence, and headroom. It yields to
the existing bounded controller during grabs and steps. A stepped stance is
replaced, never mutated, only after a qualified landing/capture and at least
0.10 s of bilateral load-bearing contact.

## H76 evidence boundary

[H76](standing-h76-contract.md) is frozen as the six-snapshot, zero-physical-step
build/read diagnostic. Its scalar motor, limit, lock, Jacobian, effective-mass,
RHS/CFM, and stored-impulse rows are coherent. That result is static and does
not explain within-step forefoot motion.

## H74/H77 comparison

The three preserved H75 pre-step states contain 46 motor axes each. H74 and H77
requests differ by at most `2.4868995751603507e-14 Nm`, below the retained H75
f32 tolerance `1.90735e-6 Nm`; all ceilings are unchanged and every H77 native
axis has nonzero `kp`. Live raw native readback has maximum current-state request
error `7.020261989509891e-7 Nm`, also within tolerance.

The frozen 360-tick reference comparison is negative:

| Reference | H74 → H77 pelvis error (m) | H74 → H77 foot error (m) | H74 → H77 linear (m/s) | H74 → H77 angular (rad/s) | H77 improves every metric |
| --- | ---: | ---: | ---: | ---: | --- |
| 0 | 0.0327242 → 0.0334898 | 0.00983493 → 0.0106719 | 0.0605509 → 0.0648878 | 0.596351 → 0.806190 | No |
| −0.005 m | 0.0269707 → 0.0329325 | 0.0101649 → 0.0106575 | 0.0715417 → 0.0582046 | 0.974726 → 0.763567 | No |
| +0.005 m | 0.0276273 → 0.0216111 | 0.0110200 → 0.00618359 | 0.0645294 → 0.0406230 | 0.695524 → 0.514140 | Yes |

H77 therefore fails the required “improve every reference” gate. Contact-share,
side-floor, airborne-descent, pelvis-reserve, and landing-headroom variants were
run as separately retained diagnostics and rejected; none replaces H77-v1.

## Actual native substep branch

Because the comparison failed, the diagnostic instruments the actual Rapier
substeps without changing their output. The saved-state trace is byte-identical
to its control native step for all three H75 states. The live H77 peak trace is
also output-neutral and records 25 substeps at approximately 0.000666667 s each,
with 46 motor, 46 limit, and 98 lock rows per substep, warm-start and solved
impulses, sole normal/friction/twist impulses, and pre/post body velocities.

All 46 H77 motor RHS values respond across the live substeps, no motor row is at
its bound, and maximum bound utilization is 0.235934. Native-to-WASM results are
close but not bit-identical, so the evidence is directional rather than a
replacement for standing acceptance. The required branch is
`correct-joint-rows/contact-wrench`: requested joint impulses are finite,
responsive, and uncapped, while the destabilizing response is associated with
the sole/contact solve. No locked-row rank/sign or native representation repair
is justified, and H27, H28, and H43 were not reused.

The retained slow-pull/reversal traces then isolate a later transfer failure:
after a qualified touchdown, an opposite-side transfer can be admitted before
that intended retained sole carries useful load; it loses support, the transfer
is cancelled, and the subsequent committed step misses its landing. Extending
measured-share retention, hard-projecting pressure to the retained side, and
tracking the rate-limited support target all fail earlier and were reverted.
The exact trace hashes and negative outcomes are archived in
`evidence/standing-h77-v1/balance-contact-branch-01/report.json`. A safe repair
therefore needs a separately versioned, time-consistent pressure/landing model;
no speculative allocator variant is retained in H77.

## Formal acceptance result

Run:

```text
node scripts/standing-acceptance.mjs --controller=h77-v1 evidence/standing-h77-v1/acceptance-03
```

Result: **fail**, with unchanged source fingerprints. Eight policy/controller
tests and all six injected failure transitions pass. All three reference trials
fail before local return is authorized:

| Reference | First failure | Exact reason | Measured limiting value |
| --- | ---: | --- | --- |
| 0 | tick 132 | reference entry/hold and official speed | 0.661885 rad/s angular speed |
| −0.005 m | tick 120 | reference entry/hold | 0.00545593 m foot error |
| +0.005 m | tick 120 | reference entry/hold | 0.0216047 m pelvis error |

The saved-failure/rescue, 2+10 s screens, 2+30 s official standing, structural,
sustained, held-out, runtime, memory, and normal-browser-path stages are
**incomplete**, not passes. Promotion is prohibited.

The final serial focused balance/controller selection reports 25/28 pass,
2 fail, and 1 historical-source-dependent skip when run without the archive
environment. Both active-step commitment assertions now pass without weakening
their checks. The separate slow-pull/reversal case still fails at tick 304 and
the planted-reversal case at tick 228, so required return-to-upright behavior is
still unproved.

Final repository checks on this source are exact:

- TypeScript: pass.
- ESLint: exit 0 with 6 pre-existing unused-symbol warnings and no errors.
- Production build: pass, with the existing large-chunk warning.
- Complete unit suite: 325/339 pass, 13 fail, 1 skip. This includes the two
  pull/reversal failures (ticks 304/228), existing recovery/geometry failures,
  one Vite SSR transport timeout, and an H77 no-step selector check that reached
  `transition` under concurrent load; the same H77 policy selection passes 8/8
  serially in the acceptance preflight.
- `standing-acceptance.mjs --controller=h77-v1`: fail at reference feasibility
  as recorded above.
- Full physics harness: 6/63 scenarios pass and 57 fail. The structural
  ownership, joint-limit, fall-threshold, finite-floor, and floorless free-fall
  checks include passes, while all official idle headings and the rescue/
  recovery scenarios remain red.

No downstream check can override the failed feasibility prerequisite. The
normal-browser-path gate was not reached and remains incomplete.

## Rollout state

- `h77-v1`: retained as an explicit failed experiment and diagnostic candidate.
- `h74-v1`: retained explicitly for exact replay; not retuned or promoted.
- `legacy`: remains the normal room and playground startup and rollback path.
- A future contact-wrench/load-distribution repair must receive a new version
  and rerun the full gate from original startup. Thresholds and guards remain
  unchanged.
