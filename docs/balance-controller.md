# Physical standing and balance controller

`BalanceController.update` runs at 60 Hz and reads the current Rapier segment poses, mass-weighted momentum, exact grab anchor, and measured loaded foot contacts. It produces stance, step, trunk/arm reaction, and fall intent. Those outputs become bounded joint-motor targets; the controller never owns a body transform and never applies a root force.

## Measured state and support

The center of mass and velocity include all 25 declared segment masses. Horizontal velocity is filtered with a 14/s response, and the capture point is the horizontal center of mass plus velocity divided by `sqrt(9.81 / COM height)`.

Support is built from loaded Rapier contacts on each side's ankle, hindfoot, and forefoot. Exact solver contact points form the support hull. During initial contact loading only, the canonical hindfoot and forefoot sole patches provide a pose-derived fallback. A manipulated or swinging foot is excluded. Eligibility persists for 0.05 seconds before it becomes stance support.

The controller stores a neutral center-of-mass offset relative to the ankles at reset. Its `rootTarget` is a motor/IK goal registered to measured support, not a kinematic translation. Actual pelvis motion always comes from Rapier integration and ground reaction through the support chain.

## Fixed balance limits

`BALANCE_LIMITS` in `src/character/BalanceController.ts` is the executable source.

| Quantity | Value / rule |
| --- | --- |
| Pose fallback sole clearance | -0.025 to +0.035 m |
| Planted-target horizontal tolerance | 0.045 m |
| Manipulated-foot exclusion | Grab displacement over 0.05 m |
| Virtual pull estimate | 620 N/m plus 12 Ns/m, capped at 620 N |
| Balance acceleration intent | Capped at 3.6 m/s² |
| Internal target-speed state | Capped at 2.5 m/s |
| Step trigger margin | Half the hindfoot half-width, capped at 0.03 m |
| Step reach / travel | At most 0.36 m from pelvis reference / 0.43 m from start |
| Step duration | Adaptive 0.18–0.34 s |
| Double-support cooldown | 0.18 s |
| Marginal instability persistence | 0.12 s |
| Unrecoverable support margin | -0.43 m with no viable landing footprint |
| Unrecoverable speed | Over 1.85 m/s while margin is below -0.12 m |

The grab estimate here is used for balance prediction only. `GrabAnchorController` separately applies the force-, torque-, and power-limited physical command at the exact picked segment-local surface point.

## Postural response and stepping

Small capture-point errors are distributed into ankle, hip, lumbar, ribcage, and arm targets. Knee flexion increases with reaction magnitude and during a swing. `composeUprightPose` and the inverse-kinematics solver keep all expanded chains connected and clamp targets through the same asymmetric joint profiles used by Rapier and diagnostics.

Corrective steps begin only after startup settling or a measured disturbance. The anticipated capture point includes external-force acceleration. The first lateral step widens toward the disturbance; later steps alternate. A released manipulated foot receives its own landing step. Reach and travel are bounded before inverse kinematics, so an unreachable drag cannot lengthen a limb.

During a swing, fall decisions include the reachable landing footprint and predicted touchdown momentum. A narrow instantaneous single-foot hull therefore does not by itself force a fall. Conversely, excessive speed, lost support, or a capture point beyond both current and reachable support can overwhelm the finite actuators and commit a physical fall.

## Physical actuation

`EmbodiedCharacter` converts the target pose into profile coordinates and invokes the generalized coupled motor solver. Each command:

- uses parent/child reference frames and only the profile's permitted axes;
- respects per-axis motor strength and reports saturation;
- includes independent passive damping and progressively increasing near-limit resistance; and
- applies equal-and-opposite torque impulses to the connected bodies.

Gravity and balance compensation are expressed through contact-loaded ankle, hip, and trunk chains. They create no net internal force or unpaired torque. With no floor, the center of mass remains in free fall. With support, Rapier friction and normal impulses provide the external reaction.

## Diagnostics and deterministic checks

Balance diagnostics report center of mass, velocity, capture point, support center and margin, supporting feet, recovery capacity, external-force estimate, balance acceleration, instability persistence, and step target. Top-level diagnostics additionally report joint coordinates and targets, structural-limit error, actual motor torque and saturation, and measured contact counts and normal load.

Focused tests cover mass weighting, connected unreachable targets, heading equivariance, measured-foot exclusion, slow pulls, direction reversal, release during swing, stronger corrective stepping, and overpowering falls. The full harness adds 30-second standing at three headings, structural limits with posture motors absent, continuous body ownership, floorless free fall, recovery, input lockout, and lifecycle checks.
