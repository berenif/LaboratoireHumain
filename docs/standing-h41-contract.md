# H41 — relaxed elbow clearance

The unchanged anatomy test exposes an inward relaxed right elbow during lean.
The arm counterbalance shifts its wrist inward; a fixed elbow pole does not
preserve lateral clearance. Correct the geometric bend on the two-bone elbow
circle, retaining the wrist endpoint and bone lengths. Apply the existing
25 mm relaxed lateral offset to the elbow only when that hand is not grabbed.
Keep the closest admissible bend; when geometrically infeasible, retain the
largest possible clearance without stretching. Existing joint clipping remains.

Require the original anatomy assertions, bounded reach, anchor, collision and
physical coherence tests. Check analytic endpoint/length preservation, inactive
constraint identity, infeasible limits and world-frame covariance. Fresh
standing evidence is required because relaxed arm targets can change.

The separate swing-foot failure remains open. Its ankle target reaches the
existing 90% dorsiflexion limit (18 degrees), leaving about 9.8 degrees of foot
pitch. Neither the fixture, assertion nor ankle range is changed here.

No standing, transfer, recovery or release acceptance is inferred from this
kinematic correction. All physical settings and acceptance thresholds remain.

## Result

The original elbow test now passes for all four headings, both lean inputs,
three torso twists and both sides. Minimum measured lateral elbow clearance
is 0.025 m. Four additional analytic tests cover the closest feasible elbow,
unchanged wrist and lengths, inactive/continuous activation, infeasible
clearance and rigid-world covariance. The geometric constraint precedes the
existing joint clipping; it is not a promise of clearance for arbitrary poses.

The eleven-file selection passes **66/67 tests**, including both physical
chest-drag tests. The only failure remains the original swing-foot assertion;
none of its inputs, thresholds or assertions changed. Typecheck passes. An
unused import in the new test produced one initial lint warning; removing it
leaves its four tests and targeted lint passing. Production and all test
assertions are identical between these validation captures.

All three fresh 2+30 s plain-controller captures exactly match H40, including
initial/trajectory hashes and every recorded physical response field. The
elbow constraint is inactive along these idle histories. Foot drift remains
0.05895/0.04451/0.05023 m at 0/+π/3/−π/4, and all headings still fail the
linear and angular speed limits. No acceptance checkbox is closed.

The retained foot diagnostic measures 0.01093 rad horizontal yaw error but
0.17065 rad pitch during swing; the full-forward-vector agreement is 0.98542,
below the unchanged 0.99 assertion. Its ankle coordinate is exactly the
existing 0.31416 rad target limit. This identifies clipping in this solution,
not a proof that every possible whole-leg solution is infeasible.

Evidence: [12-artifact archive](checkpoints/2026-09-27/h41-elbow-clearance/manifest.json)
with validation, test sources, kinematics, complete standing reports and
production/diagnostic source snapshots.
