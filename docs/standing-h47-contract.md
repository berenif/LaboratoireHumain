# H47 — coupled distal actuation authority

H46's two forefoot motors reduce the sampled peaks but leave +π/3 above the
angular-speed bound. The retained post-step states include substantial motion
outside forefoot flexion, including hindfoot roll. Test the existing six distal
actuators together: bilateral ankle X, hindfoot Z and forefoot X.

Use the same H22 states, exact snapshot/live fidelity checks, ±1 Nm central
probes, 1 Nm maximum update, line search, 1e-8 regularization and original motor
ceilings. Minimize angular-speed squares of all four hindfoot/forefoot segments;
retain the 0.1 m/s linear and 0.5 rad/s other-segment angular guards. Allow up to
16 numerical iterations for six controls, and verify the normal-equation solve
with a known coupled system and redundant actuator columns.

Preserve the original two-axis arithmetic and H46 evidence. If a useful distal
bias exists, replay it for one live tick and check the next five ticks with
the unchanged physical bounds. All 25 predicted/live outputs must match exactly.
Local success can motivate an online controller experiment; it is not a
standing, transfer, prediction or release acceptance result.

## Result — local authority verified

Each heading evaluates 209 copied cases. All unperturbed histories exactly
reproduce H46/H22, and all 25 live pulse outputs equal the selected predictions.
The pulse-plus-five-step angular maxima are 0.205641, 0.173001 and 0.136675
rad/s at 0/+π/3/−π/4. Linear maxima are 0.020899, 0.019379 and 0.016451 m/s;
all three windows retain double support, zero steps and upright/reacting state.

The complete +π/3 history still has the earlier tick-276 peak of 0.619441
rad/s. These isolated corrections therefore establish local actuation
authority, not sustained standing. The two numerical solver tests and targeted
lint pass. No production or physical setting changes follow.

The [21-artifact archive](checkpoints/2026-09-27/h47-distal-authority/manifest.json)
contains all cases, peak snapshots, pulse measurements, numerical verification
and source snapshots. Reports retain full response histories; trace extracts
are bounded as declared in the manifest.
