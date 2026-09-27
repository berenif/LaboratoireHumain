# H35 — align stance damping feedback with measured angular displacement

The retained H34/H22 traces provide consecutive pre-command body rotations
and published angular velocities. The earlier rigid calibration already shows
that integrating end-of-step velocity does not reproduce measured displacement.
The next read-only calculation must quantify that distinction for the loaded
leg joints before interpreting an intervention.

Keep H22 and change one stance feedback sample. For each leg body, derive the
shortest world angular increment between consecutive measured rotations over
the unchanged timestep. Subtract the parent's corresponding increment to
estimate relative angular motion over that interval. Add
`damping * (published relative angular velocity - interval relative angular velocity)`
to the existing native motor's feedforward. This biases the sampled damping
intent toward the interval measurement while retaining the native implicit
damping term during the solve. It is not an exact replacement of the solver's
within-step velocity or proof that its published velocity is invalid.

Apply the correction only during the declared idle intervention, on the
existing thigh, shin, ankle, hindfoot and forefoot axes. Keep arm/world damping,
balance feedback, local targets, gains and torque ceilings unchanged. Record
the maximum projected correction and both angular-motion measurements. Reset
the observation history on non-standing controller phases. Use no integral,
filter, gain sweep, body force, pose write, velocity write or solver change.

Compare to exactly replayed H22 controls at all three headings over the usual
two-plus-ten-second screen. Reject on any unchanged standing or structural
failure. A passing candidate must continue to the full official
two-plus-thirty-second gate before production consideration.

H35 is rejected. All three headings remain upright with double support, but
foot drift is 0.01265/0.01245/0.02364 m. The −π/4 angular peak is 0.61454 rad/s.
The correction is active for all 720 ticks and its largest projected axis
terms are 7.17146/7.80820/7.84306 Nm before the unchanged final motor ceiling.
All three H22 control histories replay exactly. The read-only input analysis
quantifies the measurement distinction on 28 leg-axis intervals per heading;
it does not establish that the published velocity is an invalid measurement.
No production adoption follows.

Evidence: [verified H35 archive](checkpoints/2026-09-27/h35-interval-evaluation/manifest.json).
