# GetUp adapter and preserved local experiment record

Current packaging (24 October 2026): the known-entry scenario and live SLOW
continuation are included in the static build, using the explicitly authorized
tracked runtime inputs. The controller and all historical verdicts below remain
unchanged. Earlier descriptions of local-only packaging describe those experiments
at the time, not the current build. GetUp rights remain unverified; this note is
not a statement that absence of a located license proves a legal prohibition.

## Original local holding adapter

Pinned GetUp source: `5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e`.
The binary format and arithmetic follow `getup/actor.rs` and `actors.py`:
f64 accumulation, f32 layer boundaries, ReLU, deterministic tanh(mean).
No existing standalone actor WASM artifact was found. The worker source exports
`getup_actor`, but rebuilding the whole legacy worker would add unnecessary
build/toolchain work. The small JS reader uses the existing binaries directly;
no Torch, SDK, model conversion, weight copy or installation is needed.

`slow.js` is deliberately excluded from the normal MVP build. The local QA server
loads existing assets from the repository over loopback only. No UI mode or
public package contains these weights. Upstream redistribution rights remain
unverified. The paused foot cycle is never started by this adapter or its tests.

Contracts reused: physical67, student118 = 67+25+25+power1, 21 qpos-order angle
residuals, actuator permutation, gear multiplication after normalized clipping.
40 Hz inference / 20 substeps at 800 Hz; actual q/dq refreshed for PD each substep.
Moving kd=.1 and holding kd=.2 both have fixture coverage. This comparison uses
only holding. The constant current/ahead target is the existing source standing
pose + terminal height sentinel1.4 + vertical[0,0,1]. A 125 ms lookahead of this
constant target is identical. The .80326 reward height is not substituted.
No source spline or missing NPZ is fabricated. Moving reference playback is
outside this implementation. Root velocities stay in MuJoCo's source frames:
linear world, angular body-local. No Rapier ZXY conversion or qpos normalization.

## Recorded validation (not automatic permission to repeat physics)

- `node scripts/verify-getup-slow-local.mjs`: zero integration, 32 network and
  32 PD fixtures, 67 measured source-state observations and cadence/mapping.
- `node scripts/verify-getup-slow-browser.mjs`: Chromium zero-integration parity
  and one bounded holding command on a static fixture clone.
- `node scripts/compare-getup-slow-local.mjs`: single-use physical comparison;
  refuses an existing result. Entry obtained by 8,000 genuine balance ticks;
  two explicit full `mjSTATE_INTEGRATION` forks include warmstart and share the
  same verified starting state. No qpos/dq assignment during either rollout.
  3.75-second horizon = original150 holding decisions. No controller sum,
  root actuation, external assistance, new gain or fallback controller.

Measured network error: Node raw2.670288e-5/action3.293157e-6;
Chromium raw6.198884e-6/action2.145768e-6 (limits2e-4/5e-5).
PD fixture error0; physical67 error4.440892e-16. Chromium inference mean2.356ms
per decision (25ms budget); this is cloud hardware, not a phone benchmark.

Baseline265a638: 3,000 ticks, 3.75s calm, maximum tilt7.949deg,
maximum foot-origin displacement0.0365mm, maximum motor fraction.5063.
SLOW: 480ticks / .600s, stopped at foot-origin displacement30.636mm (>30mm),
maximum tilt8.737deg, final tilt.664deg, both feet still loaded229/168N.
Maximum motor fraction1, longest all-joint saturation interval27.5ms.
No fall was observed before this stopping criterion. SLOW changed COM from
approximately[.052,0,.845] to[.104,.021,.811]m. A more upright torso alone did
not qualify fixed-support calm. No retuning or second comparison followed.

`maxSlipM` in the compact runner result denotes horizontal foot-ORIGIN drift,
not an integrated tangential velocity of loaded contact points. This limitation
is explicit; it cannot be interpreted as an exact frictional slip measurement.
The common start is a balance-controller stance, not a source get-up endpoint;
it may be outside the frozen student's trained distribution. The failure is
limited to this holding comparison, not a universal inability of GetUp.

New integrations: 8,000 entry + 3,000 baseline + 480 SLOW = 11,480.
Confirmed cumulative MuJoCo159,194 PLUS the preserved earlier unknown browser
run. Rapier45,735 unchanged. Durable result and entry state hash/data:
`web/proofs/slow-holding-comparison.json`; pending count0 on completed exit.
Other evidence: `slow-local-parity.json`, `slow-local-browser.json`.
The normal UI/default controller and deployed site are unchanged.

## Seed10 WEAK → measured-reference SLOW (local, 23 October 2026)

Approved single WEAK episode and single SLOW reprise, no tuning. Scripts:
`run-getup-weak-local.mjs`, `verify-getup-weak-browser.mjs`,
`getup-step/local_seed_reference.py`, `render-getup-local.mjs`.
The normal UI, build, active balance controller and deployed site are unchanged.

Preparation reuses the actual `source_positive_control.sample_pose` AST with
NumPy RandomState(10), compiled model joint order/ranges and contact rejection.
The first candidate is accepted. The full source high-frequency XML is retained;
its presentation includes/materials are removed exactly as in the MVP build.
For WEAK only, timestep becomes .005 and Euler is explicit. Static historical
XML and old initial state remain unavailable; this is MuJoCo 3.17.0, not a claim
of identical 2.1.3 dynamics. Source `mj_step2 -> mj_step1` stepping is used.
WEAK physical67+power feeds teacher68, 40Hz decisions, five 200Hz substeps,
power .663420431 once, direct actuator controls without PD or root assistance.
Chromium 16 WEAK fixtures: raw2.670288e-5/action3.293157e-6, zero ticks.

WEAK: 400 passive +1250 active ticks =8.25s. At the active entry head=.20153m,
torso-up=.02057 (lying, not advertised as a prescribed prone pose). First head
above1.4m after .975s; 3.825s continuous upright bilateral foot support observed.
Final head1.47744m, tilt3.258deg; right foot alone loaded446.67N at the final
sample while the other readjusts. An initial software gate requiring bilateral
load at this exact final instant marked false; the original ledger is preserved
in `weak-completed-original-ledger.json`. This gate contradicted the approved
permission for foot readjustments. Qualification was corrected using the actual
standing interval and final upright foot-only support, with no WEAK rerun,
gain or seed change. This does not qualify WEAK as a quiet fixed-foot controller.

SLOW: measured post-decision teacher frames through head>1.4m plus one extra
frame. The common `reference.measured_teacher_frames` collector no longer assumes
40/80 frames; this actual trial happens to give40 measured ->80 cubic frames
at speed .5, through the existing source_retime. Pure recomputation is identical
for all recorded reference arrays; a6-frame case and missing-extra-frame rejection
also pass. A distinct SLOW episode initializes from the first measured posture
and source-retimed velocity (qvel ×.5); no solver cache is copied across rates.
No state is rewritten during either episode. This is a reprise, not a hidden
continuation from the final WEAK standing state.

SLOW student118 uses current/+5 targets, terminal sentinel1.4,40Hz inference,
800Hz PD kd.1 moving/.2 holding,21 bounded motor controls.80 moving decisions
+150 holding decisions =4600 ticks /5.75s. All150 holding samples are upright
on both feet, no nonfoot support, no fall, no balance-controller fallback.
Final head1.44924m, tilt2.243deg, COM speed.001456m/s, foot loads204.41/196.28N.
During the3.75s hold: maximum tilt17.034deg, COM speed.17675m/s, foot-origin drift
54.73/32.93mm (readjustments allowed; not exact contact slip). Final1s: max COM
speed.004060m/s, tilt2.236–2.267deg, foot-origin drift1.341/.991mm. Thus the
observed motion settles; it does not demonstrate arbitrary-pose get-up or
long-term balance. No 30mm drift veto was used.

Every integrated substep checked actuator gear×control torque equality (max
error0), finite state and unactuated six root coordinates, with all applied
external generalized/body forces zero. Max cap fractionWEAK=.66342044;
SLOW reaches1 during the motion. At40Hz holding samples none is saturated;
this does not claim absence of saturation between sampled holding commands.

Durable local evidence: `web/proofs/getup-weak-seed10/` preflight, ledger,
initial/final mjSTATE_INTEGRATION snapshots, teacher/reference arrays and
progressively flushed poses/contacts at40Hz. New6250 ticks; confirmed MuJoCo
165444 PLUS unchanged unknown historical run. Rapier45735 unchanged.

Video: `LaboratoireHumain-GetUp-relevage.mp4`,331946bytes,5.775s,1280×720/40fps.
It shows only the continuous SLOW episode, replaying recorded real geometries
in the browser renderer; no physics replay or pose interpolation. Beginning,
middle and end visually checked. Library `libfile_71395547e5508191b644368d8e38b20b`.
Weights/sources are not embedded in the video or published; rights remain
unresolved. No new dependencies, public push or deployment.

## One continuous passive-fall → WEAK cycle (23 October 2026)

Plan reviewed/approved before implementation. `run-getup-continuous-weak.mjs`
adds only sequencing and recording; it imports the existing DenseActor,
physicalObservation and Physics. Same model hash as previous WEAK, Euler200Hz,
policy40Hz, power.663420431, direct geared controls, no PD or SLOW. No deployed
site changes. Entry is the actual prior WEAK final state at t8.25s (not SLOW):
`getup-weak-seed10/weak-final.json`, SHA256
`8b9ef9f76bd26ec5047c5b1d13ecbf4715820ef3c9a8324568dd7cdbc9473281`.
All233 mjSTATE_INTEGRATION values, qpos and qvel were restored and read back
exactly before the cycle. Entry head1.47744m/tilt3.258deg, COM speed.15044m/s;
it is upright but is not claimed to be a stationary equilibrium. Preparation
cost0 ticks. State/time installation occurs only once before this cycle.

Predeclared in `getup-continuous-weak/preflight.json`: exactly400 passive ticks;
fall=head<.9m and (torso vertical<.5 or nonfoot load>=3N), four consecutive40Hz
samples, also present at the end of passive fall. Return standing=head>1.4m,
tilt<30deg, feet carry >=half body weight, no nonfoot load>=3N, ten consecutive
40Hz samples. Either foot may carry load; no foot-drift veto. Once standing,
at most750 WEAK holding ticks; persistent measured fall stops immediately.
Maximum1250 recovery ticks, total maximum2400. No gains, caps or friction changed.

Single result: FAIL of the complete cycle. Passive fall observed at t8.95s,
then WEAK reactivated at t10.25s with the actual fallen q/dq, no state reset.
At passive end head.08972m, torso tilt92.438deg. Standing gate reached after
980 recovery ticks /4.9s at t15.15s: head1.47413m, tilt15.750deg, COM speed
.36211m/s, right foot336.38N. This is a mobile recovery, not calm standing.
After235 holding ticks /1.175s, persistent fall at t16.325s stopped the trial:
head.12716m, tilt76.415deg, nonfoot impact. No retry, alternate posture or gain
adjustment. Holding3.75s was NOT achieved.

All1615 integrated ticks checked motor caps and gear×control equality (maxerror0),
zero root actuator forces, zero applied external body/generalized forces,
finite state and continuous time. Maxmotorfraction=.66342044.400+980+235=1615
new ticks; preparation0. Confirmed MuJoCo167059 PLUS prior unknown run preserved;
Rapier45735 unchanged. Poses at40Hz contain324 frames, actual simulation8.075s;
video8.1s includes the final displayed frame. The whole cycle is shown without
phase cuts, speed changes or interpolation. Initial/fallen/recovery-entry/
holding-entry/final integration states and progressive ledger remain local.

Video `LaboratoireHumain-WEAK-cycle-continu.mp4`,655396bytes,1280×720/40fps,
Library `libfile_142bcb53156081919b67d724ac5c17af`. Beginning, fallen state, command
resumption, returned-standing state and final fall checked visually. Caption:
replay of recorded poses, real speed; no new simulation. No weights in video,
no package/push/deployment, GetUp rights still unresolved.

## Continuous passive fall → existing SLOW (single approved iteration)

`run-getup-continuous-slow.mjs` reuses the same entry, model, actor reader,
observations and source PD. No new teacher, gains, model or controller. The
candidate comes from the FAILED preceding WEAK cycle. Existing
measured_teacher_frames/source_retime selects170 post-decision samples from
10.275s through the first head>1.4m at14.475s plus one at14.500s. Speed.5 gives
340 targets /8.5s. Its last measured state is AIRBORNE: right foot minimum8.03mm,
left58.51mm, no solved loads. It is not a stable teacher or validated landing
trajectory. The terminal target remains existing standingPose/sentinel1.4/001.

Preflight checks the timestep setter leaves all233 integration-state values
unchanged. Actual trial restores only the same standing entry before the cycle.
After400 passive ticks, qpos/qvel/time match the previous trial exactly. In this
same world the timestep changes200→800Hz: full integration state preserved,
no qpos/qvel installation, no scaled physical velocity or warmstart reset.
The retimed qvel exists only in the reference file. SLOW40Hz/PD800Hz, kd.1/.2,
21 motors and source caps unchanged; applied root/generalized/body assistance0.

Observation: full340 moving +150 holding decisions completed. No fall during
the3.75s observation. At reference end t18.75s a flight already started at18.70s
continues: minima R15.4/L40.1mm. First loaded right contact18.825s (1181N),
bilateral18.85s (R808/L141N); both feet stay loaded through22.50s, with no new
flight or nonfoot support after landing. Reactions refer to the last solved
substep; geometry is refreshed after it (1.25ms stage difference in SLOW).

The predeclared AUTOMATIC verdict remains false: it rejected ANY airborne
sample after the reference, including two samples from this pre-existing
flight, rather than distinguishing a new flight after reception. We do not
rewrite this verdict or tune/retry. Measured reception is nevertheless retained:
3.65s bilateral support and final85 consecutive40Hz calm samples (2.125s).
Calm means bilateral>=3N each, summed feet>=.8bodyweight, no nonfoot>=3N,
head>1.4m, torso tilt<20deg, COM speed<.05m/s, root angular speed<.5rad/s.
No displacement veto. Final head1.452m, tilt1.789deg, COM speed.01321m/s,
loads R211.62/L189.04N. Last1s maxCOMspeed.02368m/s; tilt.975–1.789deg.
Foot-origin drift since bilateral reception R37.88/L35.61mm (not exact contact
slip). This is not proof of fixed-foot equilibrium or arbitrary-fall recovery.

Per-tick torque/gear equality and21 caps passed (maxfraction1); rootactuation
and external forces remained zero, finite state/time continuity passed.
New400 passive+9800 SLOW=10200 ticks, preparation0; confirmed177259 PLUS old
unknown run retained. Rapier45735 unchanged. No second attempt.

Evidence under `web/proofs/getup-continuous-slow/`: reference+audit, preflight,
progressive ledger/571 poses at40Hz, fallen/holding/final full states. Video
`LaboratoireHumain-SLOW-cycle-observe.mp4` shows all14.25s physical cycle at real
speed (file14.275s includes last frame), no interpolation or physics replay.
The original rendering's automatic-failure caption is preserved in
`LaboratoireHumain-SLOW-cycle-continu.mp4`; the delivery rendering changes only
that final caption to observed reception/calm duration, not the recorded motion
or ledger verdict. Six-frame board and final PNG extracted from delivery MP4
and visually inspected. No omitted phase. Côté mapping R=body7/L=body10.

Official Library batch helper failed before upload confirmation:
`hosted apps tools/list request failed: network`. No confirmed Library IDs and
no switch to another transfer route. Local deliverables remain available.
No weights packaged, deployment, public push, installation or site change.

## Local browser scenario (approved bounded integration)

Open `/web/dist/index.html?getup=local` through the existing repository-root
loopback preview server. The normal deployed MVP does not expose this button;
`local-getup/`, checkpoints, reference and weights remain excluded from its build.
The entry, model, reference and actor are checked by SHA256 before replacement
of the current world. GetUp is explicitly a KNOWN-ENTRY experiment, not a policy
that can be activated from any MVP posture. The sole initial restoration includes
positions, velocities, time and all233 mjSTATE_INTEGRATION values.

`cycle.js` extracts the sequencing of `run-getup-continuous-slow.mjs`: 400 passive
steps at200Hz,340 SLOW decisions then150 holding decisions,20 steps each at800Hz.
Same network/observation/PD functions, source gears and gains; no new reference.
No `Physics.step()`/balance call is made while GetUp owns commands. The camera
tracks the measured COM horizontally only for framing; it never writes state.
Grab/push/mode are temporarily unavailable, camera/pause remain available.
Reset explicitly quits the scenario; finishing pauses integrations and retains
its visible report. No silent handoff to the balance controller.

The prospective observer logs the whole movement but evaluates terminal reception
within the final observation window. Bilateral loads during the get-up propulsion
(e.g. t18.350, head.757m) are not mislabeled as completed terminal reception.
A flight already underway at the end of the reference can land during this
window; a new flight AFTER its loaded reception is reported separately. Same
3N/foot,.8weight,head1.4m,tilt20deg,COMspeed.05m/s,angularspeed.5rad/s thresholds.
Contact observations remain40Hz with last-substep reaction/refreshed-geometry
stage distinction. This does not edit the af04325 FAIL verdict or qualify a
reference generated from the failed WEAK rollout as a stable teacher.

`node scripts/verify-getup-cycle-ui.mjs` performs only replay/pure/UI checks.
`--run` is a single-use browser acceptance, with pending reserve before each
5/20-step decision, durable compressed40Hz samples, final state and10200 cap.
Existing ledger/poses prohibit a repeated run. No physical validation is implied
by the pure check. No Library retry, public package or deployment is part of this
local experiment; GetUp rights and the official media-transfer403 remain blockers.

### Browser acceptance result (one execution, no retry)

Runtime commit a3a9a68, Chromium1440x1000, official WASM3.17.0: 400 passive +
9800 SLOW =10200 actual steps. No extra preparation or forecasts. Confirmed
MuJoCo187459 PLUS the prior unknown run; Rapier45735 unchanged. The subsequent
UI reset/pure checks integrated zero steps. `getup-browser-cycle/ledger.json`
has completed=true,pendingMaxTicks=0, and a final233-value integration state.

All571 sampled qpos, qvel and ctrl vectors are EXACTLY equal to af04325, as is
the complete final integration state. Thus the UI extraction did not change the
recorded trajectory. Sole initial restoration, continuous time and full state
preservation at200→800Hz passed. All21 applied efforts respected gears/caps;
maxratio1, gear error0, root actuation/external applied forces0 per tick.

Terminal reception at18.850s; no new flight after that reception observed at40Hz.
150 final observation samples,85 consecutive calm samples at the end (2.125s
sample-count convention;2.100s between first/last sample). Final head1.45168m,
tilt1.789deg,COMspeed.01321m/s,loads R211.62/L189.04N. This observed reception
and calm is distinct from the unchanged historical automatic FAIL. Neither a
reference from a stable teacher nor arbitrary-fall robustness is established.

UI evidence: local-only opt-in, default path no weights fetched, paused
mode/reset/camera/grab/release checks, physical cycle through the button, command
selection/push locked during the cycle, persistent terminal report, no ticks
while paused after completion, explicit reset returns to MVP. A final pure fix
also preserves the previously selected passive/balance mode across that reset.
No phone test or deployment occurred.

Video LaboratoireHumain-GetUp-navigateur.mp4:915710bytes,1280x720,40fps,16.275s.
It replays only measured poses:14.25s physical movement plus final frame/2s
explicit EN PAUSE display. Beginning/fall/moving/reception/final/paused frames
were decoded from the MP4 and visually inspected; body and ground remain visible.
The UI screenshot shows the persistent report. No extra physical replay, weights
inside media, upload attempt or new Library ID. Official helper403 remains.

## Explicit SLOW continuation (one approved bounded observation)

The historical cycle remains unchanged. Its completed local UI now offers a
separate continuation, attaching to the living final world without any state
setter. For the authorized comparison only, the local URL
`?getup=local&continue=checkpoint` exposes the explicit button “Restaurer la fin
enregistrée puis continuer SLOW”. This restores once all233 integration values
from `getup-browser-cycle/final-state.json`, SHA256
`a8f29036d655785d8450ebe657b1b613dc929f5178ef0cb3a7a457fb43b95049`.
No fall/get-up is replayed; start time remains22.499999999997232s.

SLOW retains the same terminal target, stateless actor,40Hz decisions and800Hz
per-substep PD(.2 holding damping), source gears and limits. No Balance or
`Physics.step()` call occurs while it owns commands. The existing user-force
writer is factored as `applyPush()` without changing ordinary MVP behavior.
After800 calm native steps, exactly120 steps apply15N world+X at the torso COM;
no external moment or other body force. The maximum is4000 new steps (5s),
zero forecasts. Camera/pause remain available; other interactions stay locked
for this measurement. It finishes explicitly paused, preserving both reports.

Before runtime the criteria are fixed: existing calm = each foot>=3N, combined
foot load>=80%weight, no other body>=3N, head>1.4m,torso<20deg,COMspeed<.05m/s,
root angular speed<.5rad/s. Calm is sampled at40Hz. Loss before the first second
stops without pushing. At every native substep, stop on either foot<3N,
nonfoot>=3N,new flight (total load<.1N and both gaps>1mm), measured fall, invalid
state or actuator error. Normal completion additionally requires the final
800 steps covered by consecutive calm samples. These finite-case criteria do
not establish arbitrary-disturbance stability. Reactions remain last-substep
values; refreshed geometry is1.25ms later. Historical af04325 FAIL is unchanged.

`node scripts/verify-getup-continuation.mjs` is pure/browser wiring only.
`--run` executes the single authorized observation and refuses an existing
ledger/pose file. Reserve20 before each decision, then save actual ticks/poses;
terminal full state and report are durable. `render-getup-local.mjs
--slow-continuation` replays only those recorded poses, never integrates.
The preview uses the already-running repository server on127.0.0.1:5199.
No Library/network retry, deployment or weights distribution is authorized.

### Continuation result — one physical run

Runtime60405ded377828e3c23ae8e4a06edb9ef37668a6 completed4000 native steps,
zero forecasts/replay/preparation. t22.500→27.500s, MuJoCo191459 confirmed
PLUS the prior unknown run; Rapier45735 unchanged. Ledger completed=true,
pendingMaxTicks=0. The exact233-value final state round-trips through mj_set/get
with zero integrations (SHA2567d954fc41d529504cfd565b65558712d7cde098d71edd1dcdf271dcc2a91e383).

All first-second samples satisfy calm. The single torso+X pulse ran from23.500
to23.650s, exactly120 steps. No loss of bilateral loaded support, new flight,
nonfoot support or fall triggered the native-step guard. Final132 consecutive
calm40Hz samples =3.300s by sample-count convention (3.275s between endpoints),
including all40 samples of the last second. Peak COMspeed.06803m/s falls to
.001099m/s. Peak torso tilt2.973deg, final2.118deg. Final loadsR206.73/L193.96N;
minimum sampled loadsR157.64/L136.71N. Maximum actual torque ratio.484079,
gear error0; no saturation or unrequested external force. Root actuator torque0.
Maximum horizontal COM excursion22.78mm; foot-origin excursionsR11.35/L10.16mm
include any rolling/rotation and are not a measurement of pure contact slip.
This establishes this one bounded response, not general standing robustness.

The physical run finished and wrote final state before the browser QA exited1:
its attempted Pause click timed out because refresh ran while getupBusy was
true, leaving the button disabled between expensive frames. No physical error
or ambiguous tick remains. The UI-only correction refreshes after clearing busy.
A no-integration advance stub then verifies Pause/resume/camera; actual physical
pause/resume has NOT been retested. No second physical run was made.

Local media: `getup-slow-continuation/LaboratoireHumain-SLOW-maintien.mp4`,
257599B,1280×720,40fps,7.025s =5s physical recording plus endpoint frame and2s
explicitly paused display. Recorded201 poses are replayed without interpolation
or engine integration. Decoded beginning/pulse/final images were inspected:
whole body/ground visible, small bounded sway and loaded feet, final caption
in header so it does not obscure the head. The earlier render with an overlapping
caption remains preserved; the `-maintien` video is the delivery candidate.
No Library attempt/ID (known403 restriction), site change or weights package.

## Live cycle → continuation validation — incomplete, no retry

The single authorized browser execution used harness899229c and the unchanged
872d878 application/controllers. It completed10200 real steps, t8.25→22.50s:
400 passive plus9800 SLOW, zero forecasts. All571 recorded cycle geometries,
qpos, qvel and commands match the preceding browser-cycle recording. Terminal
reception18.850s and final85 calm samples (2.125s sample-count convention) were
observed again. The cycle's final pause froze the entire233-value integration
state and counters over the measured wall-time pause.

The actual “Continuer SLOW” click attached a new continuation to the SAME living
Physics object, with exactly the same233-value integration state, no checkpoint
restore and zero intermediate mj_setState calls. However, the harness interpreted
the previous cycle's done=true during the asynchronous module import as an end
of the new continuation. Its assertion then stopped the attempt using the
existing blur/pause handler. The saved state proves continuation0, pulse0,
worldsteps10200, pending0, paused=true; it equals the cycle final state exactly.
There is no ambiguous ongoing execution. This is a validation-script failure,
not an observed physical failure of the continuation, and not a complete PASS.

Only the harness wait was corrected to require continuation===true before
accepting its pause/terminal event, while retaining an independent app-failure
exit. `node scripts/verify-getup-live-journey.mjs --check-waits` checks seven
states, including the stale completed cycle, with ZERO integrations. No second
physical run was performed. Pause/camera/resume DURING the scheduled pulse and
the complete live continuation remain unvalidated by this attempt. The earlier
4000-step checkpoint continuation result and historical af04325 FAIL stay intact.

Evidence: `web/proofs/getup-live-journey/ledger.json`, `cycle-final.json`,
`stopped-state.json`, progressive compressed poses, and compact audit. A partial
video/capture replays ONLY these actual geometries at40Hz, with no physics,
interpolation, omitted physical phase or invented continuation. The duplicate
zero-duration handoff sample is omitted only from rendering. The final caption
explicitly states interruption before continuation/push; it is not a screenshot
of the original UI, whose browser closed after the harness failure.

New10200 steps; confirmed MuJoCo201659 PLUS the unchanged old unknown run.
Rapier45735 remains frozen. Application, controllers, reference, weights,
deployed private site and all historical results remain unchanged. No transfer
was attempted under the known403 restriction; no new Library ID exists.

## Restored Pause/camera/resume acceptance (24 October 2026)

Approved single test, runtime8a114f6, unchanged application/controller872d878.
`verify-getup-live-journey.mjs --restored-pause` selects the EXISTING explicit
checkpoint-continuation UI and performs a zero-integration restoration preflight;
adding `--run` executes once into its own `getup-restored-pause/` evidence folder.
The 233 integration values match both the saved live-journey stop and the existing
checkpoint exactly, before any integration. This is an explicitly RESTORED
observation, not a continuation of the closed browser world or a replay of the
fall/get-up. Source/model/actor/PD/gains/caps are unchanged; SLOW remains sole owner.

Result: limited PASS. 4000 native800Hz steps, t22.500→27.500s, zero forecasts.
After800 calm steps, torso+X force15N starts23.500s. At continuation tick840
(40 force ticks, t23.550s), the real Pause handler is activated. Camera zoom/reset
works during a347.6ms wall-time pause. All233 state values, simulated time,
integration/pulse counters and remaining pulse (.100s) are exactly unchanged.
Resume executes the remaining80 force ticks; total120 ticks/.150s, ending23.650s.
No intermediate state setter, world replacement, root actuation or extra force.

All40 first-second and40 final-second samples satisfy the existing calm criteria.
Final132 consecutive calm40Hz samples=3.300s (3.275s between sampled endpoints).
No native-step support loss, nonfoot support, new flight or fall guard fired.
Final COMspeed.001099m/s, torso tilt2.118deg, loadsR206.73/L193.96N.
Maximum COMspeed.06803m/s, tilt2.973deg, actual torque fraction.4840783;
gear error0, no saturation. Reactions remain last solved substep, geometry
refreshed (1.25ms stage distinction). Final pause and frozen counters verified.

All201 recorded qpos/qvel/ctrl/geometry samples and the full final integration
state equal the earlier unpaused checkpoint continuation. Thus the actual UI
pause did not change the simulated pulse duration or this measured trajectory.
This finite restored test does NOT turn the incomplete live journey into PASS,
does not change historical af04325 FAIL, and does not establish general recovery.

Durable ledger/pause/full final state/compact audit and original final browser
screenshot are saved in `web/proofs/getup-restored-pause/`. Screenshot inspected:
whole body and floor visible, upright; the textual terminal report is separately
preserved in ui.json. No redundant video, new Library attempt, deployment or
weights package. New4000 steps, confirmed MuJoCo205659 PLUS the old unknown run;
Rapier45735 frozen. No retry or second physical execution.

## Complete live browser journey (24 October 2026)

Single approved execution, runtime0ba3df0, application/controllers unchanged.
`verify-getup-live-journey.mjs --full-journey-final` is zero-integration preflight;
`--run` uses a new guarded evidence folder, current cumulative counter and the
existing UI/phase-wait/pause checks. It never selects the checkpoint-continuation
URL. The final report is brought into view using the EXISTING sidebar scroll.
Only the verification script changed, not the renderer, model, policy or UI.

Result: PASS LIMITED TO THIS KNOWN-ENTRY JOURNEY. 10200 cycle steps plus4000
continuation steps =14200 actual integrations, zero forecasts, t8.25→27.50s.
One initial known-entry restoration precedes passive fall; then the SAME world
runs passive200Hz→SLOW800Hz, cycle-final pause, actual Continuer SLOW click,
continuation and final pause. The233 values at the live handoff match exactly;
no intermediate state setter/world replacement/checkpoint or Balance handoff.
The initial cycle has observed terminal reception and2.125s sampled final calm.

After800 calm continuation steps, the existing torso15N+X pulse runs. Actual
Pause at global tick11040 (continuation840, force40, t23.550s) plus camera
zoom/reset freezes all233 integration values, time, counters and.100s remaining
pulse for368.8ms wall time. Resume applies exactly80 more force ticks. Total
force120 native steps/.150s, t23.500→23.650s; no duration extension from UI pause.
Final132 consecutive calm40Hz samples=3.300s, including all40 final-second
samples; no continuation support-loss/nonfoot/flight/fall guard fires. Final
pause and frozen counters verified again, also after scrolling the report.

Final COMspeed.001099m/s, tilt2.118deg, foot loadsR206.73/L193.96N. Continuation
peak COMspeed.06803m/s, tilt2.973deg, applied torque fraction.4840783, gear error0.
Cycle max torque fraction1; all21 caps remain enforced. Reactions retain the
last-substep/refreshed-geometry distinction. All571 cycle samples and all201
continuation samples match their earlier physical recordings in qpos/qvel/ctrl/
geometries; final233 values match the prior restored continuation. Raw772 rows
retain the duplicated zero-duration live handoff sample explicitly.

Evidence `web/proofs/getup-full-journey-final/`: progressive compressed poses,
cycle/final full states, pause comparison, ledger, compact audit and original
browser screenshot. The screenshot was inspected: mannequin/ground and BOTH
persisted reports are visible, with no redundant video or replay. The combined
PASS is written only after this evidence is saved. Browser then closes cleanly.

This closes the complete local known-entry journey gap; it does not establish
arbitrary-fall recovery, unrestricted post-get-up interaction or phone delivery.
Historical af04325 FAIL and the earlier INCOMPLETE live attempt remain intact.
Confirmed MuJoCo219859 PLUS the unchanged unknown old run; Rapier45735 frozen.
No retry, transfer under403, weights distribution, public push or deployment.

## Reconstructing the local scenario from the Git backup

The two runtime inputs are now tracked at their original paths, byte-for-byte:
`web/proofs/getup-weak-seed10/weak-final.json` (3,507 bytes, the complete known
entry state) and `web/proofs/getup-continuous-slow/reference.json` (421,156 bytes,
the measured/retimed candidate). `cycle.js` verifies their SHA256 identities.
The reference still comes from the failed WEAK cycle; its provenance and the
historical verdict are unchanged. These files are inputs, not a video replay.

From the repository root, with Node >=22.13 and a browser available:

```sh
npm ci
npm run build
node scripts/verify-mujoco-package.mjs # no physics
npm start
```

Open `http://127.0.0.1:5185/`. The build copies the existing modules and links
immutable runtime inputs into `web/dist/getup-assets`, with relative URLs and
identity checks. Serving only dist now supports the complete known-entry flow.
The page starts paused; clicking Essai GetUp runs the known-entry scenario.
Continuer SLOW uses its live final world; it needs no saved final checkpoint.

`package-lock.json` pins the official Apache-2.0 MuJoCo package to 3.17.0 with
its integrity hash. A fresh install needs the package registry or a matching
cache; vendor WASM, node_modules and build output are not in this backup.
The optional `continue=checkpoint` path and historical audit harness still
require the repository-root loopback server, excluded local evidence and, for
the harness, environment-specific
browser tooling. Media and other checkpoints remain excluded. This backup
does not deploy the site or establish arbitrary-pose or phone support.
Only data/schema/hash and Git checks accompanied this addition: no clean
install or new physical execution was performed.
