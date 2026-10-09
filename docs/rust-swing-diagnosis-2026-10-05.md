# Swing reach and delayed lift diagnosis — 2026-10-05

The second `slow-right-foot-heading` swing starts from an outdated foot anchor
and requests a landing outside the current two-link IK reach. Its moving foot
remains loaded throughout the nominal trajectory while hip roll exhausts its
allocated motor cap. Strict stepping remains **1/9**, with **4/9** physically
upright fixtures. This checkpoint adds measurements and preserves the existing
controller behavior.

The dedicated acceptance executable produced two byte-identical full reports.
After removing only the new `swingTrace` field, every report equals the retained
later-transfer baseline: commands, targets, contacts, delivered motors, phase
events, failures and final observations. All **86 workspace tests**, formatting
and native/WASM Clippy pass on unchanged Rust sources.

The receipt, source snapshots, executable, comparisons and reports are under
`evidence/migration-swing-diagnosis-2026-10-05/final/`. The initial disk-full
linking failure remains under `verified/`. Generated native compiler cache was
moved to the task's temporary build directory; its location is recorded in the
final receipt. Source and retained physics evidence were preserved.

## Measurements

`StepController::diagnose_swing` runs only in the diagnostic scenario's trace
callback, after motor installation and before physics integration. It reports
the requested foot center, bounded IK endpoint, endpoint from installed joint
targets, terminal IK endpoint, joint limits, motor targets and measured pose.
Every FK sample uses the same measured pre-integration pelvis. The observer adds
the resulting pose, canonical sole loads and per-substep delivered motor impulses.
The nine strict fixtures contain **1,473** live-swing substep records.

Reach excess measures the distance beyond the controller's two-link reach model
with its heading-aligned foot offset. It describes that IK construction, rather
than an exhaustive search over every possible anatomical foot orientation.
The regression covers reachable and unreachable goals at two headings,
separates installed corrections from IK intent, rejects non-swing samples and
checks unchanged full body observations.

| Measurement | First right-foot swing | Second right-foot swing |
| --- | ---: | ---: |
| Admission | tick 79, substep 3 | tick 180, substep 0 |
| Foot distance from saved takeoff anchor at admission | 1.87 mm | 132.56 mm |
| Planned horizontal travel from saved anchor | 103.93 mm | 169.41 mm |
| Measured horizontal target gap immediately after admission | 103.25 mm | 301.48 mm |
| Terminal reach excess at admission | 0.53 mm | 76.06 mm |
| Terminal bounded-IK residual at admission | 0.24 mm | 75.89 mm |
| Requested lift at 0.225 s | 40.00 mm | 40.00 mm |
| Installed endpoint lift at 0.225 s | 40.00 mm | 59.06 mm |
| Measured lift after that substep | 35.82 mm | 9.55 mm |
| Moving-foot load before that substep | 0 N | 144.90 N |
| First measured moving-foot load below 3 N | 0.008 s | 0.721 s |
| Verified liftoff | 0.129 s | 0.829 s |
| Horizontal target gap at nominal completion's first fully clamped sample | 7.00 mm | 290.83 mm |
| Bounded-IK residual at that sample | <0.01 mm | 105.07 mm |

The first fully clamped endpoint sample is at 0.454 s because the accumulated
float clock remains just below 0.45 s on the previous sample. All controller
timing and physical criteria are unchanged. Lift distances use the saved foot
center anchor; actual liftoff additionally requires sole clearance above 10 mm
for this profile, load below 3 N and the original 0.05-second proof.

![Requested, bounded, installed and measured paths for both right-foot swings](../evidence/migration-swing-diagnosis-2026-10-05/final/right-foot-swings.png)

## Diagnosis and next repair

Landing at tick 114 records both measured feet as planted anchors. The next
request starts at tick 127 and retains that old moving-foot position until
admission at tick 180. The measured foot has since moved 133 mm, leaving an
immediate 129 mm gap to the installed endpoint. Its requested landing already
misses the bounded IK endpoint by 76 mm at admission. All 323 observed substeps
of this second swing have positive reach excess.

During all **109 nominal-trajectory substeps**, right hip roll delivers at least
99% of its allocated impulse cap, averaging 99.85%. The axis's installed cap is
40.23 N m. The maximum observed position tracking error over the second swing is
0.301 rad. This documents a capacity limit under the current geometry and load;
it does not establish that another budget distribution would repair the step.

Installed and intended FK endpoints agree within 0.00011 mm throughout both
right-foot swings. Balance position corrections therefore do not explain their
large tracking gap. At nominal completion, the second foot still carries
161.71 N. Its later liftoff accompanies an installed endpoint 147.55 mm above the
saved anchor and a pelvis already descending to 0.928 m. The foot remains
274.16 mm from the landing target; the fixture falls at tick 260.

The next candidate should plan from the measured takeoff pose at admission and
qualify a bounded landing path together with the retained stance. Recheck
unloading and post-landing dynamics under the same readiness, lift, touchdown,
slip and fall requirements. These measurements prioritize takeoff continuity
and path feasibility before another lift-clock or motor-reserve trial.
Production stepping/recovery and release admission remain unqualified.
