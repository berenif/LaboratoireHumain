# H53 — dimensionless numerical search for horizontal authority

H52's continued horizontal search still moves knee biases roughly 1 Nm per
iteration because the inherited absolute regularization competes with the
small translational Jacobian. Correct the numerical units before concluding
that actuator authority is absent.

Repeat the same six-versus-eight-control, tick-350 comparisons from the same
H50 states. Express each residual in units of the unchanged 0.1 m/s linear
bound, and each control increment as a fraction of its unchanged native
torque ceiling. Solve the same 1e-8-regularized normal equation in these
dimensionless units, then convert back to Nm. This preserves the unregularized
motion objective; it changes numerical conditioning and the regularization's
preference among actuator increments, not native motor gains or capacity.

Keep H52's eight-iteration budget, ±1 Nm physical probes, full-step/backtracking
fractions, native caps, initial/retained exact replay checks, and every actual
nonlinear speed guard. Report saturation and failed line search through the
retained trials. A large linear proposal is not authority evidence until its
capped nonlinear result passes. Preserve the original dimensional method for
replay. No controller or standing adoption follows this local comparison.

## Result — one deadband success, two unresolved states

All exact-state and native-parameter checks pass. With eight controls,
heading zero reaches 0.0128 mm/s within the unchanged speed guards. The
other selected results retain 1.903 and 0.399 mm/s at +π/3 and −π/4;
the six-control comparisons also miss the deadband. The
[15-artifact archive](checkpoints/2026-09-27/h53-dimensionless-authority/manifest.json)
preserves all 343 queries and source. No online controller is evaluated.

At +π/3, the last Newton direction produces rejected or non-improving line
candidates, while an already computed, feasible negative knee probe has a
lower actual objective than the retained iterate (9.226e-6 versus 9.635e-6).
The optimizer discarded that available improvement. Its stopping result
therefore cannot establish absence of actuator authority. The next comparison
must retain feasible improvements from its probes and control large proposed
increments before interpreting the result physically.
