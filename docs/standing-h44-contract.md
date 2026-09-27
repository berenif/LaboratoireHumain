# H44 — finite-state mass diagnostics

H43 produces finite published positions, rotations and zero velocities while
16 bodies' centers of mass and inverse inertias are NaN. The existing finite
flag ignores the published `centerOfMass` and `massKg` fields, so it remains
true at tick 133. A later inertia inversion raises an error at tick 134.

Include every provided mass and center-of-mass value in the published-pose
finiteness check. Preserve the existing error code and all physical behavior.
This strengthens an acceptance guard; it does not repair the rejected joint
experiment. Keep that experiment stopped at its first failed guard.

Require red/green regressions for an invalid native center of mass with finite
geometry, and invalid published masses independently of geometry. Recheck the
focused production selection and exact plain standing histories. Replaying H43
must reject the same physical trajectory at its first non-finite mass state.

## Result

Both regressions fail under the original guard and pass unchanged after the
two additional finite checks. The native fixture supplies an invalid local
mass center without moving or stepping its body, proving that finite geometry
can coexist with an invalid physical center of mass. The second fixture checks
NaN and both signed infinities in mass independently of finite geometry/COM.

The twelve-file selection passes **68/69 tests**. The unchanged swing-foot
assertion remains the only failure. Typecheck passes; targeted lint reports
zero errors and the pre-existing unused `RIGHT` warning in `EmbodiedCharacter.ts`.

H43 now stops at tick 133 with `finite=false` and `NONFINITE_CHARACTER_STATE`.
Every physical history hash through that tick equals the earlier failed run.
All three fresh 2+30 s plain-controller histories and every recorded physical
response field exactly equal H41. The guard correction changes no forces,
poses, joint settings, collisions or acceptance thresholds and closes no
standing, transfer, recovery or release gate.

The source audit verifies that the only production change since H41 is these
two checks; the other 66 production/package files are identical. Evidence:
[18-artifact archive](checkpoints/2026-09-27/h44-finite-mass/manifest.json),
including red/green test sources, validation, plain captures and earlier
rejection of the same invalid physical trajectory.
