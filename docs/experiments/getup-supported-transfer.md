# GetUp weight transfer, without a step

This is a default-off, gravity-only **control experiment**, not implemented
walking or a new standing acceptance rule. The published default and successful
V3 Recovery binaries remain unchanged. The bounded pilot is now closed without
a successful transfer; the conditional diagnosis is recorded below.

## Small first milestone

The explicit `gravity-only-supported-transfer-v1` profile requires its compiled
feature and repeated GetUp recovery. A request can take motor ownership only
after the existing 800-sample recovery pass, current 800-sample stability and the
completed 20-sample V3 blend. It never takes authority at the earlier 80-sample
control-admission boundary.

One bounded linear intention uses the actual GetUp source-hinge Jacobians to
shift COM toward the nearest point in the retained foot's actual 3 mm-inset contact hull, while requesting
fixed positions/orientations for both feet and fixed torso orientation. Virtual
root coordinates participate only in this algebraic intention: only the 21
joint targets reach `apply_controls`, as equal/opposite joint torques. No pose,
velocity, world, root force, model, motor limit or contact parameter is changed.
The solve rejects excessive linear residuals, targets outside original joint
limits and changes over 0.35 rad; it is not a nonlinear reachability proof.

The COM intention is limited to 120 mm and 50 mm/s, with 25 ms start/stop ramps.
Transfer qualification must occur within 2.5 physical seconds. There is no lift
or swing trajectory. A separate gait support-intention branch permits single
support; the ordinary bilateral standing controller is unchanged. Its desired
reactions are inverse-statics inputs, never claims of measured support.

Qualification requires 20 consecutive actual samples with retained-floor load
at least 80% body weight, other-foot load below 3 N, no nonfoot support, and COM
and capture point inside the nondegenerate actual retained contact hull with the
existing 3 mm margin. Every requested command must be finite and unclipped, with
5% normalized torque reserve. Existing 10 mm foot-origin and 20 mm anchored
material-point excursion limits remain; new solved contact points are compared
against the original foot pose too. Every native sample updates compact maxima,
load/contact gates and continuity counters. No successful standing predicate is
weakened to credit intentional unloading.

After qualification, the controller returns along its attained reference (not
an assumed completed ramp), then requires ordinary supported standing with 800
stable samples within a separate four-second return bound. Its first command
must match the previous joint-balance command. Failure latches and never silently
hands motors to another owner. An actual fall cancels the transfer before the
next motor command and retains its diagnostic result while the existing V3
same-world recovery lifecycle takes over. No native/impact clock is reset.

This first experiment explicitly refuses active grabs or striker motion. It
does **not** yet solve external-wrench feasibility, reactive stepping under a
user pull, alternating walking, or terrain support.

## Checkpoints and bounded verification

Transfer state is serialized and validated. **Active-transfer restoration is
explicitly rejected** in this first milestone; an active checkpoint is exact
local diagnostic evidence, not an admitted resume path. Terminal state may be
restored with the existing checks. Unsupported profile/build/state combinations
fail closed, rather than dropping an unknown active controller.

Focused tests exercise source hinge/Jacobian mapping, intended root-wrench
closure, bounded ramps, exclusive ownership, fall cancellation and rejection of
active restoration. They use no physical integration and are not effectiveness
proof. The subsequent explicit pilot is capped at 37,000 total real + forecast
integrations and ten minutes for both transfers and returns, with no automatic
retry. It starts only after source review/publication. Either side failing stops
physical experiments before any swing work.

One shared compiler target is reused, with a separately measured baseline and
384 MiB additional build/temp allowance plus 16 MiB compact evidence. No training,
new dependency, repeated recovery regression or archive framework is introduced.

The explicit pilot entrypoint is `scripts/check-getup-transfer.mjs --execute`
with a separately bound worker build receipt and a new output directory. Run it
under an outer hard `timeout --signal=TERM --kill-after=5s 600s` launcher, not just
the cooperative JavaScript timer. Before each `advance(1)`, its fsynced ledger
reserves 5,020 integrations. Successful readback reconciles real plus local
forecast deltas across controller cycles; uncertain calls retain that reservation.
A hard timeout does not authorize a replacement process. Both transfer requests
share the same constructor, world and ledger; initial prone settling is included.

### First local integration stop and conserved correction

The first local pilot stopped before its first forecast, after 400 real settling
integrations. The controller-free teacher fork had retained the new real-owner
profile selector while disabling its required controller. The correction clears
only that selector in the shadow; the real profile and all numerical policies
stay unchanged. No transfer effectiveness was tested. Its original files remain
unchanged. The corrected pilot requires that closed ledger as a hash-bound input,
charges its 400 permanently against the same 37,000 total, and has at most 36,600
new integrations / 599 seconds. Use the outer timeout of 599 seconds for this
corrected attempt. No old runtime, cursor or physical state is imported.

The second local pilot earned initial standing but its harness requested transfer
before the terminal balance controller had bound. Rust correctly rejected the
request; no transfer command ran. The harness now also waits for the existing
post-blend `experimental-joint-balance-holding` status; Rust still independently
checks all entry conditions and the finite/unclipped preview. The two closed
ledgers total 5,480 integrations (400 + 4,520 real + 560 forecast), leaving at most
31,520 new integrations and 594 seconds under the same original allocation.
Both ledger hashes are required by the corrected entrypoint. This is a JS-only
correction using the same verified worker, not a changed physical controller.

### Geometry diagnosis and the right-only bounded proposal

The correctly timed third request was refused before any transfer motor command.
A separate unchanged-worker replay stopped at the same earned standing entry and
read actual contacts: the left centroid was 156.004 mm from the bound COM
reference, and the right 146.413 mm. This was the over-distance branch, not the
previously conflated near-zero case. The closest 3 mm-inset points require
134.283 mm on the left and 118.489 mm on the right. **Left is infeasible under
this command bound at this stance.** No physical transfer has yet passed.

Selection now projects the bound reference onto the convex hull inset instead
of targeting its centroid. It uses double precision for clipping/projection,
10 micrometres of additional inward numerical clearance, then checks the
rounded target with the existing 3 mm predicate and unchanged 120 mm limit.
Near-zero and over-distance refusals are separate and include actual distance.
All posture, torque, speed, time and measured-support gates remain unchanged.
This does not manufacture support or prove dynamic reachability.

The final proposed pilot requested **only right transfer and return**, not both
sides or a step. The three closed pilots plus geometry replay charge 16,200
integrations once against the original 37,000, leaving at most 20,800 new.
Its last two ledger inputs are the geometry ledger and its hash-bound prior
pilot ledger. Full 5,020 dispatch reservations remain, with an outer 584-second
timeout and a fresh output directory. It stops at the first failure; no bound
increase, swing, stance widening, reset or automatic retry is authorized.


## Closed pilot: fixed-foot static construction is not feasible

The last right-side request passed target geometry but was refused by the
unchanged joint-posture guard before transfer ownership. Across the four pilots
and one diagnostic replay, **21,560 real + forecast integrations** were charged
once. No transfer, unloading, return, step or walking success is claimed. All
closed evidence remains intact; no further physics is authorized by this budget.

The diagnosed production source is public commit
`9a49c1bc2578ad809136033ec369aa457e4ac39a`, tree
`c7597a69c4790f880bebfdcf9cd319233dbb7578`. Offline reconstruction from retained
transforms verified the source-hinge mapping. A bounded nonlinear kinematic path
could keep both feet fixed, but **kinematic closure is not torque feasibility**.
It was not promoted into the controller.

The current reaction allocator restricts each COP to a sole centerline and uses
bilateral load sharing for lateral balance. Its residual 32.22 N swing-load
intention is therefore not evidence of physical impossibility. Correct static
single support instead requires total force equal to body weight and resultant
COP at the COM projection. With up to 3 N swing load, weighted COPs must still
produce that same resultant; all whole-body force and moment equations apply.

For the proposed endpoint, that corrected contact wrench requires
**45.83–46.46 Nm** at the right ankle-y motor, versus **19 Nm** allowed with the
unchanged 5% reserve. A conservative relaxation over the original 120 mm COM
domain, actual 3 mm-inset contact hull, allowed ankle angles and Coulomb friction
still requires **at least 33.55 Nm**. The lateral displacement consumes nearly
the whole command radius, leaving the feasible COM forward of the ankle and a
large real support lever arm. Changing the allocator alone does not fix this.

This is a conditional NO-GO for the **fixed-foot, interior-joint, quasi-static
construction**, not a proof that walking is impossible. Changed earned stance,
foot placement/orientation, momentum, other external support and hard joint-stop
reactions are outside the calculation. No motor/ROM/contact/stability limit was
raised. Actual generalized velocity and the distinct old motor-target transition
also remain unverified for the proposed path. Timing is tight: 118.5 mm at
50 mm/s already takes about 2.37 s before all admission/qualification overhead.

The next causal design question is physically preparing a feasible bilateral
stance/contact geometry **before** attempting unloading, not increasing force,
teleporting a body or silently resetting the displacement reference. It requires
a separate bounded design. The deployed default and successful V3 Recovery
variants and the full interface are unchanged.

Compact offline diagnosis receipt SHA-256:
`eb19e7385bd63b58332bf4e654adebeed40496a6a0cdcf465b1047810fb48bdb`.

### One explicit offline stance-preparation candidate

A separately authorized **offline-only** candidate then tested 60 mm rearward
bilateral COM preparation followed by right COM `(x, z) = (-0.051, -0.570)` m.
Its new, explicit envelope was 150 mm maximum excursion from the original
physical COM and 180 mm outward path—not the old 120 mm region and not a hidden
reference reset. Actual algebraic excursion was 142.051 mm (142.372 mm from the
stored reference); the two-leg path length was 178.629 mm. The preceding
33.55 Nm lower bound applies only to the old region, so it does not exclude this
separate construction.

Endpoint-first, fixed-work source-hinge IK retained original foot/torso anchors,
original-entry joint changes at most 0.35 rad and the original ROM interior. The
prepared bilateral endpoint passed ideal stationary inverse statics: all 21
normalized motor demands were at most 0.6953. At the final right-only endpoint,
COM was 3.994 mm inside the actual contact hull, intended normal loads were
400.680 N right / zero left, and force/moment residuals were negligible. The
ankle demand fell to 16.21 Nm, but **right hip-x required -38.13275 Nm, or
-0.953319 normalized: it fails the unchanged 0.95 reserve**.

The candidate stopped there. No tolerance was changed, no endpoint search or
path extension followed, and no physical integration/build/production edit was
performed. This is one failed posture solution, not proof that the body cannot
step. The next causal question is torque-aware posture selection under the same
joint/contact limits—not arbitrary gains or higher actuation. Current generalized
velocities, takeover continuity, dynamic path and return remain unverified.

Exact candidate-result SHA-256:
`23c8dd51011a7f05affce49546a386cd55fe1c2bd181445e404855c5bc4d5f27`.
