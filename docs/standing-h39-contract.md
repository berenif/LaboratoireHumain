# H39 — interval damping with geometric inertias

H38 confirms a physical mass-property error and corrects it in the current
factory. That changes the plant on which H35's damping comparison was rejected.
Re-evaluate that one declared feedback change on the corrected plant: paired
H22 and interval-relative-angular-rate damping, unchanged gains and motor caps,
at all three headings for 2 s settling plus 10 s observation.

The interval mode adds the previously specified damping feedforward difference;
it does not replace the native within-step damping solver. No gain search,
integral term, pose latch, changed collision rule or solver setting is included.
Keep all H38 mass-property corrections and existing diagnostic identity/finite
guards. Require every original standing bound before advancing to 2+30 s.

This remains a diagnostic controller comparison. The production mass-property
candidate has two independently confirmed focused regressions: the left-hand
chest-drag fixture no longer reaches contact, and the tiny ankle-impulse response
exceeds its unchanged prediction tolerance. Both original tests pass with the
archived pre-correction factory. Neither is waived by any standing result.

H39 is rejected. Candidate angular peaks are 0.51437, 0.50900 and 0.63706 rad/s
at 0/+π/3/−π/4, so all three exceed the unchanged 0.5 limit. All remain upright
with double support and zero steps. No controller change is adopted.

Evidence: [verified H39 archive](checkpoints/2026-09-27/h39-interval-evaluation/manifest.json).
The subsequent [candidate report](inertia-correction-2026-09-27.md) records the
controlled correction to the impulse-response measurement separately.
