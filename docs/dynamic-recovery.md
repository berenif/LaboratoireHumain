# Dynamic falling and automatic recovery

The character estimates balance while upright, attempts corrective steps, and yields to dynamic physics when support and momentum exceed its remaining recovery capacity. Rapier retains ownership throughout the fall, landing, and get-up. Body dragging becomes available again only after persistent, supported standing.

This document describes the implementation and its fixed numerical rules. [Balance controller](balance-controller.md) explains the upright estimator and limb solver. [Physics acceptance](physics-acceptance.md) defines independent assertions, fixtures, and evidence; it is the place to check validation outcomes.

## Ownership and state

`EmbodiedCharacter` owns the Rapier world, body creation, handoffs, and snapshots. `BalanceController` owns procedural balance and step intent. `pose.ts` solves the connected upright skeleton. `DynamicRecovery` reads contact evidence and supplies bounded muscle and pelvis impulses without owning the simulation clock. Core contracts remain in `src/core/types.ts`; pointer interaction and rendering remain separate modules.

| Motion state | Recovery phase | Pose authority and body input |
| --- | --- | --- |
| `upright`, `reacting`, `stepping` | `none` | Root character motor and procedural pose; dragging allowed when unpaused |
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

Landing requires loaded **non-foot** floor contact for **0.10 consecutive seconds**; already planted feet cannot establish landing. Settling requires a loaded floor contact together with mass-weighted RMS linear speed at most **0.65 m/s** and RMS angular speed at most **1.8 rad/s**, continuously for **0.30 seconds**. Each RMS includes all sixteen segments, weighted by the declared segment masses.

## Protective motion and phase progression

At fall entry, the initial protective direction is expressed relative to the character heading. Forward protection brings bent arms forward; lateral protection abducts the near arm; backward protection bends the torso and tucks the head. The arm, elbow, torso, head, hip, knee, and ankle targets are relative joint angles constrained by the anatomical target limits in `src/core/humanoid.ts`.

Once settled, the actual landed torso orientation selects forward, backward, left, or right recovery. Its world forward/right axes determine which side faces the floor. Rolling uses a corresponding pelvis pitch target: **-0.30 rad forward**, **-0.50 rad backward**, or **-0.45 rad lateral**. A backward roll keeps the torso flexed at **+0.40 rad** relative to the pelvis and targets upper-arm pitch **+0.40 rad**. Other roll orientations target torso pitch **+0.15 rad** and upper-arm pitch **-1.45 rad**. These targets retain trunk flexion while the body establishes bracing contact; the following phases progressively bring the pelvis over lower supporting segments and extend the legs.

| Transition | Required contact and pose evidence |
| --- | --- |
| `protect` → `settle` | Loaded non-foot floor contact for 0.10 s |
| `settle` → `roll` | Loaded floor contact and the settling motion bounds for 0.30 s |
| `roll` → `brace` | At least 0.20 s in phase; a load-bearing hand/forearm, shin, or foot; torso-up Y > 0.25; pelvis Y > 0.28 m |
| `brace` → `kneel` | At least 0.20 s in phase; a load-bearing shin or foot; torso-up Y > 0.65; pelvis Y > 0.38 m |
| `kneel` → `stand` | At least 0.20 s in phase; both feet load-bearing; torso-up Y > 0.88; pelvis Y > 0.55 m |
| `stand` → procedural `upright` | The stable standing conditions below persist for 0.55 s |

Stable standing requires **both load-bearing feet**, each foot-up Y **> 0.97**, torso and pelvis up-vector Y **≥ 0.97**, pelvis Y **> 0.93 m**, mass-weighted RMS linear speed **≤ 0.22 m/s**, and RMS angular speed **≤ 0.65 rad/s**. Failure of any condition resets the stable timer. Elapsed phase time alone cannot complete a transition.

## Bounded muscles and assistance

Joint muscles use an implicit proportional/derivative response based on world inverse inertia. The implementation copies Rapier's temporary inertia result before another query can reuse its backing buffer. Each target remains anatomically bounded, and the resulting torque vector is independently capped before applying equal and opposite parent/child torque impulses.

| Joint group | Maximum torque magnitude |
| --- | --- |
| Torso, thighs, shins | **110 Nm** per joint |
| Upper arms | **30 Nm** |
| Forearms | **20 Nm** |
| Feet | **65 Nm** |
| Neck and head | **22 Nm** |
| Hands | **9 Nm** |

Pelvis assistance has separate vector caps of **950 N** and **300 Nm**. It can act only in `roll`, `brace`, `kneel`, or `stand` while at least one eligible, persistent load-bearing contact exists. Impulses equal the bounded force or torque multiplied by the fixed timestep. Joint muscles and pelvis assistance are separate from external grab forces.

The `rollHeight` target starts at the actual pelvis Y position when `roll` begins. With qualifying support, it can advance when torso-up Y is **> 0.25**, or while a prone body has both a persistent load-bearing **hand or forearm** and an eligible load-bearing **thigh, shin, or foot**. The latter path lets the braced trunk rise before it points upward; it does not bypass the contact and pose requirements for entering `brace`.

Each qualifying tick sets `rollHeight = min(0.43, max(previousRollHeight, actualPelvisY) + dt * 0.22)`: the target follows the higher of its previous value and the current pelvis height, adds a **0.22 m/s** rise, and caps the result at **0.43 m**. While rolling with torso-up Y **< 0.25**, the upward assistance component remains limited to **55% of total body weight**; the overall **950 N** force and **300 Nm** torque caps still apply. This starts from the landed height and limits lift while the trunk is inverted, while allowing established arm-and-leg support to make progress.

Pelvis height targets are **0.57 m** in `brace` and **0.72 m** in `kneel`. In supported `stand`, a bounded progress variable advances at **0.6/s**, raising the target from **0.72 to 1.012 m** and extending the hip/knee targets. This variable shapes the motor target; it cannot declare standing. Horizontal assistance targets the landed pelvis location in early phases and the current supported foot center, corrected for the ankle offset and heading, in kneeling/standing.

Assistance authorization stops when qualifying support disappears; the next integration receives no pelvis assistance without suitable support. Disabling the floor between updates invalidates its contact evidence before applying assistance. Unsupported recovery remains dynamic.

## Retry instead of forced completion

A recovering phase returns to `settle` after more than **0.20 seconds** without eligible supporting contact, or more than **3.0 seconds** without sufficient progress. Pose progress is the reduction in absolute pelvis-height error plus **0.4 × max(0, 1 - torso-up Y)**; an improvement over the best error by more than **0.015** resets the stall timer. The progress metric uses the final phase height goal, including **0.43 m** for rolling, separately from the gradual motor height target.

During `stand`, a **new best consecutive stable interval** also resets the stall timer. The `bestStableTime` field retains the longest interval achieved in the current phase; only an interval longer than that record, with a **1e-9 s** comparison tolerance, counts as new progress. Repeated short intervals cannot indefinitely reset a stall. The completion condition still requires **0.55 uninterrupted seconds**, and the stall threshold remains **3.0 seconds**. Phase entry resets the pose-progress record, best stable interval, and support-loss timer.

A retry increments the diagnostic counter and clears settling persistence. Protective muscles continue while the system waits for loaded contact and low motion again. There is no timeout that teleports the character upright. The independent **25 simulated second** recovery limit is an acceptance requirement for representative falls on the existing unobstructed floor; missing support or an obstruction must keep the body dynamic and able to retry.

## Transfers and input isolation

Fall entry captures every current world segment position and rotation and carries its linear and angular velocity into Rapier, bounded to **3 m/s** and **6 rad/s**. It commits the dynamic state and immediately clears the active grab and external grab contribution. Spherical joints use the matching anatomical anchors.

Recovery completion is evaluated from the latest integrated contact, pose, and velocity evidence before applying another motor or assistance impulse, so the copied velocities are the actual last-integrated Rapier velocities. After stable recovery, the upright motor starts at the recovered pelvis world position and heading, with the actual recovered feet and every segment pose as its seed. The first procedural snapshot preserves the recovered poses at the same simulation instant. A **0.75 s** transition blends rotations toward the ordinary upright solver and reconstructs child positions through matching joint anchors. The acceptance contract independently limits every segment's immediate handoff discontinuity to **2.5 cm** and **3°**, using the shortest relative rotation, with no intervening integration.

`bodyInputAvailable` is the shared diagnostic gate for picking, interaction, and status text. `EmbodiedCharacter` rejects body begin/move commands in `falling`, `fallen`, and `recovering`. `DemoRuntime` synchronizes that availability before and after every fixed substep. On lockout, `PointerInteraction` discards queued begin/move/terminal commands and releases capture in that same runtime update, before another catch-up substep can integrate.

Cleanup calls `clearBodyInput` directly and does not advance a physics step. A pointer held across a fall, or pressed during lockout, has no retained grab to resume; body control needs a fresh press after standing. Camera gestures, pause/resume, renderer switching, and Reset remain available. Reset restores the original position and heading, and the runtime preserves whether the simulation was paused.

Diagnostics expose COM and capture-point balance, stance/support intent, instability persistence, actual dynamic contact loads and ages, recovery phase and timers, retry count, bounded assistance, maximum joint motor torque, handoff measurements, and body-input availability. External grab diagnostics remain separate from motor and recovery assistance diagnostics. Procedural `balance` diagnostics are **null during dynamic states and on the immediate return handoff**, then populated by the next procedural update; this prevents stale support or force data from a previous authority. The balance step count starts a new cycle after recovery.

Floor penetration is measured against the **finite, enabled floor collider** with Rapier `contactShape` and each segment's actual oriented shape. Only negative contact distance contributes positive penetration depth. Clearance, an absent contact beyond the floor, or a disabled floor contributes zero; an infinite plane approximation is not used.
