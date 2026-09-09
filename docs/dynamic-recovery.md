# Dynamic falling and automatic recovery

The character estimates balance while upright, attempts corrective steps, and yields to dynamic physics when support and momentum exceed its remaining recovery capacity. Rapier retains ownership throughout the fall, landing, and get-up. Body dragging becomes available again only after persistent, supported standing.

This document describes the implementation and its fixed numerical rules. [Balance controller](balance-controller.md) explains the upright estimator and limb solver. [Physics acceptance](physics-acceptance.md) defines independent assertions, fixtures, and evidence; it is the place to check validation outcomes.

## Ownership and state

`EmbodiedCharacter` owns the Rapier world, body creation, handoffs, and snapshots. `BalanceController` owns procedural balance and step intent. `pose.ts` solves the connected upright skeleton. `DynamicRecovery` reads contact evidence and supplies bounded muscle and pelvis impulses without owning the simulation clock. Core contracts remain in `src/core/types.ts`; pointer interaction and rendering remain separate modules.

| Motion state | Recovery phase | Pose authority and body input |
| --- | --- | --- |
| `upright`, `reacting`, `stepping` | `none` | Root character motor and procedural pose; dragging allowed when unpaused and no recovery seed is active |
| `upright` return transition | `none` | Root character motor and 0.75-second recovered-pose blend; dragging locked |
| `falling` | `protect` | Dynamic Rapier segments; dragging locked |
| `fallen` | `settle` | Dynamic Rapier segments; dragging locked |
| `recovering` | `roll`, `brace`, `kneel`, `stand` | Dynamic Rapier segments; dragging locked |

The fixed timestep remains 1/60 second, with **16 solver iterations** and up to **4 CCD substeps** within an integration. The CCD budget resolves fast limb/floor impacts without allowing collision stopping to stretch the linked joint anchors; it does not change the outer 60 Hz simulation clock. Recovery changes motion through force and torque impulses. It does not execute poses with body translation, rotation, velocity, or body-type setters. Initial bounded velocity assignment at a handoff and an explicit Reset are separate operations. Active ragdoll bodies cannot sleep, so supporting solver loads remain observable.

## Contact evidence

After each integration, `observe` queries actual contact pairs between the existing floor collider and every character collider. A geometric floor contact requires solver distance at most **0.012 m** and an upward world normal whose Y component is at least **0.65**. The code derives load from the solved normal impulse divided by the timestep. A segment becomes load-bearing after load of at least **3 N** persists for **0.05 consecutive seconds**. Losing that load resets its persistence.

Touching alone does not authorize assistance. A load-bearing segment must also belong to the current phase's eligible set. Head and neck contacts never authorize pelvis assistance.

| Phase | Eligible load-bearing segments |
| --- | --- |
| `protect`, `settle` | None; protective joint muscles remain active |
| `roll` | Torso, pelvis, upper arms, forearms, hands, thighs, shins, feet |
| `brace` | Hands, forearms, shins, feet |
| `kneel` | Hands, shins, feet |
| `stand` | Feet |

Landing requires loaded **non-foot** floor contact for **0.10 consecutive seconds**, or two loaded feet in an already supported low crouch (torso-up Y > 0.65 and pelvis Y < 0.85 m). Settling requires a loaded floor contact together with mass-weighted RMS linear speed at most **0.65 m/s** and RMS angular speed at most **1.8 rad/s**, continuously for **0.30 seconds**. Each RMS includes all sixteen segments, weighted by the declared segment masses.

## Protective motion and phase progression

At fall entry, the initial protective direction is expressed relative to the character heading. Forward protection brings bent arms forward; lateral protection abducts the near arm; backward protection bends the torso and tucks the head. The arm, elbow, torso, head, hip, knee, and ankle targets are relative joint angles constrained by the anatomical target limits in `src/core/humanoid.ts`.

After the existing settling interval, route selection prefers established support: balanced planted soles select a crouch rise; a planted foot with the opposite shin supporting selects half-kneeling; a prone body with arm support selects arm-assisted preparation; remaining poses roll toward arm support. The chosen route, leading side, and roll side remain fixed until a retry. Ties use deterministic side order. Reachable foot placements are calculated with the shared two-bone solver and anatomical joint limits before comparing required travel.

Measured physical conditions may skip preparation. A balanced crouch can enter `stand` directly only with both persistent loaded soles, foot-up Y > 0.85, torso-up Y > 0.88, pelvis Y > 0.55 m, and projected mass inside the actual sole support hull. A supported half-kneel can start in `kneel`; that phase includes the initial rise and trailing-foot placement. A prone body first repositions unusable arms while its existing body and limb contacts remain grounded.

| Transition | Required contact and pose evidence |
| --- | --- |
| `protect` → `settle` | Loaded non-foot floor contact for 0.10 s, or an already supported low crouch |
| `settle` → selected preparation | Loaded floor contact and the settling motion bounds for 0.30 s |
| `roll` → `brace` | Established bracing support; ordinary route requires at least 0.20 s, torso-up Y > 0.25 and pelvis Y > 0.28 m; a prone preparation may instead establish measured usable arm support over grounded body support |
| `brace` → `kneel` | At least 0.20 s in phase; a load-bearing shin or foot; torso-up Y > 0.65; pelvis Y > 0.38 m |
| `kneel` → `stand` | At least 0.20 s in phase; both feet load-bearing and sole-up Y > 0.85; torso-up Y > 0.88; pelvis Y > 0.55 m |
| `stand` → procedural `upright` | The stable standing conditions below persist for 0.55 s |

Stable standing requires **both load-bearing feet**, each foot-up Y **> 0.97**, torso and pelvis up-vector Y **≥ 0.97**, pelvis Y **> 0.93 m**, mass-weighted RMS linear speed **≤ 0.22 m/s**, and RMS angular speed **≤ 0.65 rad/s**. Failure of any condition resets the stable timer. Elapsed phase time alone cannot complete a transition.

## Bounded muscles and assistance

Joint muscles use relative rotation targets from the same reachable two-bone geometry as the upright solver. Targets blend from the measured entry rotations and retain the anatomical knee and elbow bend directions. All muscle impulses are computed from one measured state, using copied world inverse-inertia tensors. A coupled implicit proportional/derivative solve accounts for shared bodies; bounded block solves propagate each saturated torque into neighboring motors. Every joint impulse has an equal and opposite parent impulse. No target is written into a dynamic body's pose.

| Joint group | Maximum torque magnitude |
| --- | --- |
| Torso, thighs, shins | **110 Nm** per joint |
| Upper arms | **30 Nm** |
| Forearms | **20 Nm** |
| Feet | **65 Nm** |
| Neck and head | **22 Nm** |
| Hands | **9 Nm** |

Residual pelvis assistance retains the 950 N total force ceiling and is capped at **60 Nm** of torque. Its upward component is at most **20% of body weight**, and is zero throughout rolling or without adequate loaded lower support and projected balance. There is no upward rolling target and no body-weight compensation term. The pelvis orientation target joins the same coupled torque solve and blends from the measured entry rotation.

Joint load compensation uses the measured contact distribution and descendant masses. Those torques act internally, with equal and opposite impulses; only Rapier's floor reactions and the separately capped residual assistance can raise the character.

A plant captures its world target when contact has persisted for 0.05 s. Sliding cannot move that target. The controller retains it until a deliberate release or sustained loss of support. A deliberately released support cannot count again until a measured unload followed by three freshly loaded frames. Releasing an adjacent segment does not erase that unload history.

Weight transfer uses the full mass-weighted center of mass and velocity. Before deliberately releasing loaded support, its position projected **0.15 seconds** forward must lie inside the convex hull of the remaining loaded solver contact points. Released, unloaded, and merely planned contacts cannot enlarge that hull. A point or line provides no support area. The release margin and released segments are recorded for independent verification.

Disabling the floor invalidates its contact evidence before the next assistance decision. Unsupported recovery remains dynamic and can retry.

## Retry instead of forced completion

A recovering phase returns to `settle` after more than **0.20 seconds** without eligible supporting contact, or more than **3.0 seconds** without sufficient progress. Progress is measured from physical height, torso orientation, and active placement error. Arm preparation measures hand travel toward captured targets; a successful placement need not raise the pelvis. Improvement exceeding 0.015 resets the stall timer. Route and transfer-stage changes capture new measured entry targets.

During `stand`, a **new best consecutive stable interval** also resets the stall timer. The `bestStableTime` field retains the longest interval achieved in the current phase; only an interval longer than that record, with a **1e-9 s** comparison tolerance, counts as new progress. Repeated short intervals cannot indefinitely reset a stall. The completion condition still requires **0.55 uninterrupted seconds**, and the stall threshold remains **3.0 seconds**. Phase entry resets the pose-progress record, best stable interval, and support-loss timer.

A retry increments the diagnostic counter and clears settling persistence. Protective muscles continue while the system waits for loaded contact and low motion again. There is no timeout that teleports the character upright. The independent **25 simulated second** recovery limit is an acceptance requirement for representative falls on the existing unobstructed floor; missing support or an obstruction must keep the body dynamic and able to retry.

## Transfers and input isolation

Fall entry captures every current world segment position and rotation and carries its linear and angular velocity into Rapier, bounded to **3 m/s** and **6 rad/s**. It commits the dynamic state and immediately clears the active grab and external grab contribution. Spherical joints use the matching anatomical anchors.

Recovery completion is evaluated from the latest integrated contact, pose, and velocity evidence before applying another motor or assistance impulse, so the copied velocities are the actual last-integrated Rapier velocities. After stable recovery, the upright motor starts at the recovered pelvis world position and heading, with the actual recovered feet and every segment pose as its seed. The first procedural snapshot preserves the recovered poses at the same simulation instant. During the **0.75 s** transition, normal balance stepping and fall decisions are suspended while rotations blend toward the ordinary upright solver and child positions are reconstructed through matching joint anchors. At completion, procedural velocities are zeroed and the balance estimator is recalibrated before normal control resumes. The acceptance contract independently limits every segment's immediate handoff discontinuity to **2.5 cm** and **3°**, using the shortest relative rotation, with no intervening integration.

`bodyInputAvailable` is the shared diagnostic gate for picking, interaction, and status text. `EmbodiedCharacter` rejects body begin/move commands in `falling`, `fallen`, and `recovering`, and throughout the 0.75-second procedural settling transition after authority returns to the motor. `DemoRuntime` synchronizes that availability before and after every fixed substep. On lockout, `PointerInteraction` discards queued begin/move/terminal commands and releases capture in that same runtime update, before another catch-up substep can integrate.

Cleanup calls `clearBodyInput` directly and does not advance a physics step. A pointer held across a fall, or pressed during lockout, has no retained grab to resume; body control needs a fresh press after standing. Camera gestures, pause/resume, renderer switching, and Reset remain available. Reset restores the original position and heading, and the runtime preserves whether the simulation was paused.

Diagnostics expose COM and capture-point balance, stance/support intent, instability persistence, actual dynamic contact loads and ages, recovery phase and timers, retry count, bounded assistance, maximum joint motor torque, handoff measurements, and body-input availability. External grab diagnostics remain separate from motor and recovery assistance diagnostics. Procedural `balance` diagnostics are **null during dynamic states and throughout the 0.75-second return transition**, then populate after the recalibrated controller resumes; this prevents stale support or force data from a previous authority. The balance step count starts a new cycle after recovery.

Floor penetration is measured against the **finite, enabled floor collider** with Rapier `contactShape` and each segment's actual oriented shape. Only negative contact distance contributes positive penetration depth. Clearance, an absent contact beyond the floor, or a disabled floor contributes zero; an infinite plane approximation is not used.

## Current implementation verification

The new crouch path has passed all four mirrored and rotated crouch fixtures, with 2.02–2.05-second recovery and 2.2–3.4 cm maximum measured planted drift in `evidence/crouch-tuning-production.json`. These are intermediate results, not acceptance of all routes. Arm-assisted, rolling, and half-kneeling recoveries are still being tuned. The full physics harness and final visual acceptance must pass before this iteration is complete.