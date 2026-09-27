# H32 — isolate CCD in the rigid constraint calibration

The fixed impulse-joint assembly topples while equivalent compound and fixed
multibody fixtures remain stable. This does not prove the articulated standing
controller has the same failure. Previous isolated precision, convergence,
contact-softness, friction-pass and warm-start comparisons did not repair it.

Test whether continuous collision detection contributes to the rigid fixture's
instability. Use the exact heading-zero tick-one snapshot from the verified
native f32 comparison. Replay its normal parameters and one intervention:
`max_ccd_substeps = 0`. Keep every other initial field and solver parameter
identical. Require the normal run to reproduce every saved body-state sample
from the earlier native f32 control exactly. Record the first CCD-active body
tick and total CCD-active body-step count; these flags indicate eligibility,
not proof that a sweep changed a position.

Compare complete retained body-state samples and separate any pre-collapse
effect from post-impact differences. An identical trajectory rules out this
switch as an explanation for that fixture. A difference only after collapse
cannot explain its onset. Any improvement would require deeper source-backed
diagnosis, not adoption of disabled CCD: the application's collision rules and
frozen settings remain unchanged. No standing acceptance is claimed.

CCD is not a repair for this fixture. All 23 retained native control body-state
samples replay exactly. The CCD-enabled and disabled runs have identical
retained samples through native tick 120, when the pelvis is already at
0.781448 m height and −0.632239 m forward position. CCD first becomes active at
tick 99, but the saved tick-120 body state remains identical; later impact
motion differs. The sparse sample schedule does not establish the exact first
divergent tick. Only `max_ccd_substeps` differs between the parameter records.

Evidence: [verified H32 archive](checkpoints/2026-09-27/h32-ccd-calibration/manifest.json).
