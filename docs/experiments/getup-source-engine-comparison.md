# Source-engine diagnostic, 2026-10-17

Status: bounded original-source positive control completed on one seed. No step, learned task teacher or five-second final stance has
been demonstrated. Native Rapier remains the application engine.

## Question and boundary

The imported WEAK actor failed the short native quiet-stance diagnostic despite
observation/action/torque parity. Before changing training, check the frozen
asset in its original200 Hz MuJoCo environment. This is a new phase,23:34:04 UTC
17 October to00:34:04 UTC18 October, limited to10 new aggregate physics-wall
seconds. The earlier one-hour Python pilot is closed at2.930621222 physics
seconds/900 native substeps; its unused allowance does not transfer.

Official MuJoCo2.1.3 was retrieved from PyPI into an isolated local directory.
The pinned GetUp `data/humanoid_static.xml` and its three visual includes were
retrieved by verified Git blob hashes. Upstream files, wheel, shared library and
weights are not added to this Git checkpoint. Source/license uncertainty remains
as documented in the reviewed Python plan.

## Transferred native quiet configuration: gate rejected

The zero-step parameter check agrees on mass (1.8e-15 kg), inertia
(5.6e-17 kg m²), gears,21 joint coordinates and27 generalized velocities.
Same-observation actor error is1.43e-6 raw/6.85e-7 action.

The geometry check fails the original0.2 mm bound:

- Native torso quaternion squared norm:1.000594629534111
- Pure raw-root FK agrees with actual native body positions within0.817 µm
- Pure normalized-root FK agrees with MuJoCo within6.67e-16 m
- Maximum source/native body-position difference:0.736706 mm at the right foot
- Geometry-observation error:0.732865 mm
- Frozen action change from the source FK observation:0.002054

Normalizing this root changes the contact depths. The reconstructed whole-foot
bottoms are native L−1.889750/R−1.018141 mm, source
L−1.153646/R−0.282629 mm. Source solver caches cannot be transferred either.
The diagnostic source world alone receives q/qvel; native state is untouched.
No time integration or physical settling was performed for these checks, and
the failed gate is preserved rather than relaxed.

## Original-source positive-control protocol

One declared fresh static-model task RandomState seed10, not a claim to match
main.py's constructor-reset RNG position. The exact dm_control1.0.1 algorithm:
root XYZ remains qpos0; normalize four independent positive-uniform quaternion
components; sample every limited hinge in compiled model order/ranges; reject
any configuration with contacts using static forward calls. Initial qvel is0.
All attempted poses and final RNG state are saved.

Then80 zero-action40 Hz decisions (400 source steps/2 s), followed by at most250
frozen-WEAK decisions (1250 steps/6.25 s). Each decision holds direct actuator
controls for5×0.005 s Euler integrations. Power0.663420431 is applied once.
Source stepping is mj_step2→mj_step1; retained force/contact-anchor stages are
explicitly separate from refreshed post-step geometry and velocity. No PD,
reference, optimizer or assistance force enters this positive control.

The count ceiling is1650 integrations/8.25 simulated seconds. The10-second
physics budget includes all settling and failures. Each synchronous C call has
a durable pending marker, a lifetime counter, measured elapsed charging and a
cooperative alarm bounded by both remaining physics and phase deadline. A
one-shot marker and exclusive ledger prevent duplicate trials. Warnings,
nonfinite state, torque/inference parity errors or budget exhaustion abort.
Flight/nonfoot during a get-up are measured rather than misclassified as a
quiet-stance failure. Only completed teacher decisions enter the original
multiplicative teacher return; no small-control factor is added.

The source preflight has0 integrations, one accepted reset pose, raw/action
parity1.907e-6/1.043e-6, and verifies zero-budget step rejection without mutation.
All17 transitive runtime input hashes are pinned.73 Python tests pass, including
five added randomizer/RNG/count/list-packet fixtures. Narrow source, mechanics and accounting
reviews precede the physical call.

## Reproduction

Own code: `scripts/getup-step/source_engine.py`, `source_bridge.c`,
`diagnose_source_geometry.py`, `source_positive_control.py` and four tests.
The bridge includes the existing independently written MuJoCo diagnostic ABI.
Build against the official2.1.3 headless library and headers. The local receipt
records the exact download URLs, SHA256, source Git blobs and compiler command;
compact receipts/results belong under `docs/evidence/getup-source-engine-2026-10-17/`.
Read `prepare` before `run`; the runner verifies its complete preflight hash set
and never refunds its ledger on reset. Raw journals remain local.

A successful source episode would establish this asset works in one source
reset distribution. It would not isolate initial-state OOD from every native
engine difference, certify a transferred full engine state, or supply step
teacher data.

## Observed source result

The first call stopped after one0.005 s zero-control integration because the
Python velocity validator applied abs to a JSON list. It supplied no teacher
evidence. That attempt, its pending markers and0.000065178 physics seconds are
preserved. A focused list/array/NaN regression passed, and the explicitly
authorized corrected restart retained that charge.

The corrected source episode completed400 settling and1250 teacher integrations
(8.25 simulated seconds),250 completed controlled decisions:

- Original teacher return:184.414734
- First head above1.4 m:0.965 s after teacher control begins
- Maximum head height:1.645525 m
- Final head height1.467499 m; torso-up0.974521
- Raw/action implementation errors≤1.049e-5/3.576e-6
- Every applied actuator torque agrees with gear×control exactly
- Corrected physics cost0.091640497 s; total phase0.091705675 s/1651 steps

Ground/nonfoot/flight and substantial foot sliding occurred during the get-up,
as expected for this task's reset and recovery, and are retained in the report.
The report separately lists119 fully aerial samples and216 both-feet-unloaded
samples; the latter matches the native task's flight definition. The native
quiet-stance gate is not applied to get-up motion.

This is useful evidence that the imported WEAK asset works in this one
source-engine reset distribution. It is not a five-second native stance or
forward-step proof, nor a complete causal separation of engine differences
from initial-state distribution. The original transferred-quiet geometry gate
still fails. No task collection, curriculum advancement or optimizer was run.

## Original-source SLOW follow-through

The same measured WEAK run supplies the original source reference collector:
first post-policy head crossing at0.975 s, then exactly one extra decision gives
40 post-action frames. The exact source cubic at speed0.5 yields80 moving
reference frames. The original high-frequency XML is verified at0.00125 s;
SLOW policy is40 Hz, residual/reference held for20 integrations, actual q/dq
refreshed every substep, moving kd0.1/holding0.2. Source terminal observation
height1.4 and standing pose are preserved. The teacher's static-model solver
cache is never transferred to the different model/timestep.

The bounded observation window was80 moving plus240 hold decisions:40 hold
settling decisions, then the FINAL200 decisions/4000 integrations/5 seconds.
Only that final window could freeze an analysis anchor or accrue quiet samples.
All quiet conditions include actual≤5-degree torso tilt, source foot loads/body
velocities,3 mm COM/capture support inset, height/self-contact limits and fixed
foot-origin drift≤10 mm. These are versioned source predicates, not a claim of
native Rapier equivalence.

A serializer rejected numpy.bool after the first source integration. That one
integration and its physical cost were retained. The actual complete source
cache was saved; after a full-packet finite-JSON regression, the exact same
first command resumed its remaining19 substeps with its original residual and
reference. The first sample was recovered from that exact cache. The final
journal has unique consecutive source IDs0..6400, without a repeated integration.

The whole6400-frame/8-second source episode completed. Head finished1.452587 m,
torso tilt1.73885 degrees. Source observation/PD/direct motor-torque equations
were independently verified, warnings0. All4000 final samples pass every quiet
condition **except fixed foot-anchor drift**. Drift first exceeds10 mm at4.480 s
and ends20.5898 mm; maximum consecutive valid run is1183 samples/1.47875 s.
Actual final-five-second loaded sliding is L20.13/R20.68 mm, so this is genuine
sliding rather than merely foot-origin rotation. The strict final-five-second
quiet-entry proof fails and is not relabeled.

Pure FK half-second windows show ongoing foot-origin transport: final L3.02 and
R3.30 mm/s, with foot separation increasing from0.201 to0.232 m. No additional
five-second extension was launched under a 'currently settled' assumption.
The source upright endpoint remains a real, low-velocity **training-only**
initialization candidate; learning quietness cannot require an already quiet
five-second policy as a circular prerequisite. Evaluation success still retains
all physical gates.

The source cache now preserves mjData's pointer-free header, fixed-layout main
buffer, stack and own diagnostic state. Fresh allocated pointers are retained;
XML/own bridge/official library/payload hashes bind the checkpoint. Zero-step
restore is byte-exact and corrupt checkpoints reject. Next-command/continued
physical-evolution parity has not yet been independently paired, so exact
resume admission for a task learner remains open.

## Frozen torque initializer on the actual SLOW endpoint

A new200 Hz static-source world received the actual800 Hz SLOW endpoint q/qvel.
Only timestep differs between the verified model XMLs; zero-step body positions,
rotations and physical67 matched exactly. Solver/contact caches were deliberately
not transferred. This is a training-initializer calibration, not a quiet entry.

The fixed WEAK power0.663420431 direct-torque actor failed the predeclared
10 mm per-foot loaded-slip condition after6 integrations/30 ms. Right slip was
7.936 mm at frame5 and10.711 mm at frame6; the next integration was not executed.
Left slip was1.656 mm, COM drift1.125 mm, warnings/flight/nonfoot0. Actor action
error≤8.12e-7 and the direct torque equation error was0. Optional power1 was
skipped after the failure. It would not have been a separately trained STRONG
checkpoint in any case.

Total comparison phase:8057 source integrations, comprising1657 at200 Hz and
6400 at800 Hz, aggregate16.285 simulated seconds and0.447573970 physics seconds.
The original serialization-abort integration remains included. No Rapier
integration, task collection or optimizer occurred in this source phase.
The prospective10-second allowance was a ceiling, not a target to spend.

The new stabilization-only source environment is disabled scaffolding. It uses
actual combined measured support, fixed entry anchors, source-clock counters,
explicit replay-terminal masks and native/controller/RNG checkpoints. Its pure
fixtures are not a dynamically demonstrated teacher or a successful step. The
next learning proposal is separately recorded in plan section11.
