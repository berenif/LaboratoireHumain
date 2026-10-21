# GetUp weight transfer, without a step

This is a default-off, gravity-only **control experiment**, not implemented
walking or a new standing acceptance rule. The published default and successful
V3 Recovery binaries remain unchanged. No physical result is claimed here.

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

The next proposed pilot requests **only right transfer and return**, not both
sides or a step. The three closed pilots plus geometry replay charge 16,200
integrations once against the original 37,000, leaving at most 20,800 new.
Its last two ledger inputs are the geometry ledger and its hash-bound prior
pilot ledger. Full 5,020 dispatch reservations remain, with an outer 584-second
timeout and a fresh output directory. It stops at the first failure; no bound
increase, swing, stance widening, reset or automatic retry is authorized.
