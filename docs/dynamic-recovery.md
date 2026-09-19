# Dynamic falling and automatic recovery

The character estimates balance while standing, attempts corrective steps, and enters protective falling when measured support and momentum exceed its remaining recovery capacity. The same 25 dynamic Rapier bodies remain authoritative through every phase. Body dragging is locked during `falling`, `fallen`, and `recovering`, and requires a fresh press after standing returns.

This document describes the implementation rules. [Balance controller](balance-controller.md) covers standing and stepping. [Physics acceptance](physics-acceptance.md) defines the independent fixtures, limits, and generated evidence.

## Ownership and state

`EmbodiedCharacter` owns the Rapier world, bodies, joints, simulation clock, state changes, and snapshots. `DynamicRecovery` reads integrated body poses and contact evidence and returns bounded joint-motor intent. `pose.ts`, `recovery-joints.ts`, and the inverse-kinematics helpers generate targets; none writes runtime transforms.

| Motion state | Recovery phase | Physical behavior |
| --- | --- | --- |
| `upright`, `reacting`, `stepping` | `none` | Continuous dynamic bodies with standing joint motors; dragging available |
| `falling` | `protect` | Protective joint targets; dragging locked |
| `fallen` | `settle` | Passive resistance without protective posture drive; dragging locked |
| `recovering` | `roll`, `brace`, `kneel`, `stand` | Contact-gated recovery motors; dragging locked |

State changes update targets and motor strength only. They do not rebuild bodies or joints and do not call translation, rotation, velocity, or body-type setters. Recovery completion captures the measured joint coordinates and blends standing targets and strength on the existing bodies, preserving position and momentum.

The outer timestep is 1/60 second. Rapier uses 20 solver iterations, four internal PGS iterations, and up to four CCD substeps per update. Dynamic bodies may sleep at rest. The contact observer retains real solver patches at sleeping equilibrium and reports raw measured impulse separately from equilibrium support load; a missing manifold removes support.

## Contact evidence

After each integration, recovery queries actual contact manifolds between enabled support surfaces and the canonical convex segment colliders. Support surfaces include the finite floor and playground geometry. A qualifying contact has solver distance at most 0.012 m and an upward normal Y component of at least 0.65. Solved normal impulse divided by the timestep gives load. A segment becomes load-bearing after at least 3 N persists for 0.05 consecutive seconds; losing load resets persistence.

Eligibility follows anatomical role and phase. Head and neck are excluded from the support sets used for recovery progression, although their physical landing contact can be observed. Rolling can use loaded trunk and limb surfaces; bracing uses distal arms and lower legs; kneeling uses hands and lower-leg/foot support; standing requires loaded hindfoot or forefoot support on both sides. Ankles count as foot segments for landing classification and may provide lower-leg support in rolling, bracing, or kneeling. Diagnostics retain the exact contacted segment.

Landing requires either qualifying non-foot contact or two loaded foot sides in an already supported crouch for 0.10 consecutive seconds. This is exactly six qualifying integrated observations at 60 Hz, without an additional timestep; a nonqualifying observation resets persistence. Settling requires loaded contact plus mass-weighted RMS linear speed at most 0.65 m/s and angular speed at most 1.8 rad/s for 0.30 consecutive seconds. All 25 segment masses participate.

## Protective motion and recovery progression

Fall direction is expressed relative to the measured heading. Forward protection brings bent arms toward a brace, lateral protection moves the near shoulder-girdle/arm chain, and backward protection distributes flexion across lumbar, ribcage, and head/neck joints. All targets use the same frame-based joint coordinates and asymmetric limits as standing and Rapier's constraints.

Route selection uses established support and physical orientation:

- balanced bilateral foot support selects a crouch rise;
- one planted foot with opposite lower-leg support selects half-kneeling;
- a prone body with reachable arm support prepares a brace; and
- other side or supine poses roll toward usable arm support.

The route, leading side, and roll side remain deterministic until a retry. Foot and hand targets are checked for reach, joint limits, collider clearance, and support margin. Movement does not advance merely because a timer elapsed.

| Transition | Required evidence |
| --- | --- |
| `protect` → `settle` | Qualifying loaded landing contact or supported crouch for 0.10 consecutive seconds |
| `settle` → route | Persistent loaded contact and settling-speed bounds |
| `roll` → `brace` | Established reachable arm/lower-limb support and pose progress |
| `brace` → `kneel` | At least 0.20 s plus qualifying lower support, torso rise, and pelvis rise |
| `kneel` → `stand` | At least 0.20 s plus bilateral loaded feet and positive support margin |
| `stand` → `upright` | Stable foot support, posture, and motion for 0.55 consecutive seconds |

Stable completion requires both foot sides, nonnegative measured support margin, upright hindfeet, pelvis and ribcage up-vector Y at least 0.97, pelvis height above 0.93 m, RMS linear speed at most 0.22 m/s, and RMS angular speed at most 0.65 rad/s.

## Bounded muscle-like motors

Each joint profile defines its parent and child frames, permitted coordinates, asymmetric hard limits, passive resistance, damping, and maximum torque. Shared coordinate functions generate targets, measure actual pose, compute motor error, and report limit error. The Rapier 0.20.0 raw multi-axis limit calls are isolated behind the tested joint adapter.

The coupled implicit motor solver works from one measured state and propagates saturation between bodies. Active posture feedback, passive damping, and near-limit resistance share the existing joint-profile coordinate torque budgets in one solve. Torque is projected only onto permitted world axes and applied equally and oppositely to child and parent; a separate passive impulse cannot add torque above the actuator ceiling. Passive resistance remains available when posture strength is zero. Settling uses passive-only mechanics so continuing protective drive cannot contaminate its low-motion observation.

Contact-aware inverse statics distributes descendant weight and horizontal correction through loaded support chains. These are still internal joint couples: only Rapier's external contact reactions can support or raise the center of mass. `assistanceForce` and `assistanceTorque` remain zero; no pelvis force, unpaired root torque, or hidden lift is available. With the floor removed, the mass-weighted center of mass continues free fall.

Each allocated contact force acts at the centroid of its measured solver patch. The planner adjusts nonnegative shares across those centroids toward the requested pressure point, preserves the measured support split when patches can supply the same moment, and remains bounded by friction, joint budgets, and the 1.35-body-weight vertical limit. Planned pressure or load cannot establish a contact or authorize transfer before measured support is available.

Planted supports capture their measured poses and material contact patches. Drift is the greatest horizontal displacement of any captured patch point under the current body transform, rather than body-center displacement or patch-centroid displacement. This allows pivoting about a fixed contact while detecting slip at an edge of a rotating patch. Drift above 0.05 m invalidates the anchor; contact absence has the existing 0.10 s grace. A slipped anchor cannot recapture the same continuously loaded contact: it must first unload and establish fresh persistent load. Diagnostics report this material-patch drift.

Before deliberately releasing support, the center of mass projected 0.15 s forward must fit within the convex hull of the remaining measured contact patches. Planned, unloaded, and deliberately released contacts cannot enlarge that hull.

The half-kneel acceptance fixture reconstructs connected anatomical chains and seats the leading articulated sole at the opposite shin's actual convex-surface floor height using legal knee, ankle, and hindfoot coordinates. Mirrored and yawed fixtures retain zero initial momentum. This setup correction preserves intended support geometry; runtime bodies remain solver-owned and the recovery thresholds are unchanged.

## Retry, interaction, and lifecycle

A recovering phase returns to settling after more than 0.20 seconds without eligible support or more than 3.0 seconds without measurable progress. Retry increments diagnostics and clears the relevant persistence. There is no timeout that teleports or forces the character upright; a missing floor or obstruction leaves the same assembly dynamic and able to retry.

`bodyInputAvailable` is the shared gate for picking, pointer capture, UI status, and replay. A fall clears the active grab and stored user force/torque immediately. Commands submitted during lockout are discarded, and a pointer held across recovery does not resume automatically. Camera gestures, pause/resume, renderer switching, and Reset remain available. Reset is the one lifecycle operation allowed to replace the assembly and restore its initial state.

Diagnostics report `physicsOwnership: "rapier-dynamic"`, exact selected segment and region, measured joint coordinates and targets, joint-limit error, motor torque and saturation, contact count/load/supporting segments, center of mass and support margin, phase progress, retries, and grab-controller loads. Canvas2D and WebGL consume the same physical snapshot and canonical geometry.
