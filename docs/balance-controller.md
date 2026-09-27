# Physical standing and balance controller

This guide describes the current implementation. Its controller parameters are separate from the independent [acceptance contract](physics-acceptance.md). See [current status](status.md) for recorded failures and verification scope.

The [coordinated standing stabilization plan](standing-stabilization-plan.md) now has an opt-in [H74-v1 implementation](standing-h74-contract.md), separate from the default controller described here. `CoordinatedStandingController` fixes its reference at startup, combines posture/foot/motion/effort feedback, and supplies the existing capped native motors. Select it with `?standingCandidate=h74-v1` on the flat-floor application path; configuration and transition reason appear in `diagnostics.coordinatedStanding`. Both the application and serial harness use `characterFrame` and the same candidate module. All three H74 reference trials fail; its latched transition to existing balance/recovery is not a validated standing backup. The diagnostic preview helpers remain separate and H73 remains pending.

`BalanceController.update` runs at 60 Hz and reads the current Rapier segment poses, mass-weighted momentum, exact grab anchor, and measured loaded foot contacts. It produces stance, step, trunk/arm reaction, and fall intent. Those outputs become bounded joint-motor targets; the controller never owns a body transform and never applies a root force.

## Measured state and support

The center of mass and velocity include all 25 declared segment masses. Horizontal velocity is filtered with a 14/s response, and the capture point is the horizontal center of mass plus velocity divided by `sqrt(9.81 / COM height)`.

Support is built from loaded Rapier contacts on each side's ankle, hindfoot, and forefoot. Exact solver contact points form the support hull; a near-floor sole pose or planned endpoint does not establish support. A manipulated or actively swinging foot is excluded. Eligibility persists for 0.05 seconds before it becomes stance support.

The controller stores a neutral center-of-mass offset relative to the ankles at reset. Its `rootTarget` is a motor/IK goal registered to measured support, not a kinematic translation. Actual pelvis motion always comes from Rapier integration and ground reaction through the support chain.

## Controller parameters

`BALANCE_LIMITS` in `src/character/BalanceController.ts` is the executable source.

| Quantity | Value / rule |
| --- | --- |
| Clearance used to identify an airborne foot | More than 0.035 m above its nominal sole height |
| Planted-target horizontal tolerance | 0.045 m |
| Manipulated-foot exclusion | Grab displacement over 0.05 m |
| Virtual pull estimate | 620 N/m plus 12 Ns/m, capped at 620 N |
| Balance acceleration intent | Capped at 3.6 m/s² |
| Internal target-speed state | Capped at 2.5 m/s |
| Step trigger margin | 0.14 m, applied to the anticipated capture-point support margin |
| Step reach / travel | At most 0.36 m from pelvis reference / 0.43 m from start |
| Step duration | Adaptive 0.48–0.68 s |
| Double-support cooldown | 0 s after qualified measured touchdown; the next lift still requires measured transfer readiness |
| Marginal instability persistence | 0.12 s |
| Unrecoverable support margin | -0.43 m with no viable landing footprint |
| Unrecoverable speed | Over 1.85 m/s while margin is below -0.12 m |

Production supplies `appliedGrabForce`, the bounded physical grab impulse divided by the timestep, including explicit zero after release. The virtual spring values above are a fallback for planning fixtures that omit both `appliedGrabForce` and its compatibility alias `externalForce`. `GrabAnchorController` applies the physical command at the exact picked segment-local surface point.

## Postural response and stepping

Small capture-point errors are distributed into ankle, hip, lumbar, ribcage, and arm targets. Knee flexion increases with reaction magnitude and during a swing. `composeUprightPose` and the inverse-kinematics solver keep all expanded chains connected and clamp targets through the same asymmetric joint profiles used by Rapier and diagnostics.

Corrective steps begin only after startup settling or a measured disturbance. The anticipated capture point includes external-force acceleration. The first lateral step widens toward the disturbance; later steps alternate. A released manipulated foot receives its own landing step. Reach and travel are bounded before inverse kinematics, so an unreachable drag cannot lengthen a limb.

Selecting an intentional step starts a weight-transfer interval with both feet still commanded to the floor. The retained side must carry at least 52% of whole-body weight in qualified measured contacts, while the moving side carries no more than 25%, for 0.10 consecutive seconds before swing begins. During the swing the moving side must actually unload below 3 N for 0.05 consecutive seconds. Elapsed swing duration completes target interpolation, not touchdown: the landing side must then establish measured hindfoot or forefoot load within 0.09 m horizontally of its target for 0.10 consecutive seconds before the step completes. The newly measured support then starts the double-support cooldown.

During a swing, fall decisions include the reachable landing footprint and predicted touchdown momentum. A narrow instantaneous single-foot hull therefore does not by itself force a fall. Conversely, excessive speed, lost support, or a capture point beyond both current and reachable support can overwhelm the finite actuators and commit a physical fall.

## Physical actuation

`EmbodiedCharacter` converts the target pose into profile coordinates and invokes `NativeJointMotors.apply` before integration. Each standing command:

- uses parent/child reference frames and only the profile's permitted axes;
- configures Rapier force-based position/velocity motors with per-axis effort ceilings;
- encodes gravity/load feedforward as a velocity bias within the same ceiling; and
- participates in Rapier's contact and structural-joint solve. Standing does not apply the recovery solver's separate JS torque impulses or near-limit passive-resistance calculation.

`motorTorqueSource: "native-request"` identifies the bounded requested wrench, not a measured delivered impulse. `motorSaturationRatio` compares the uncapped request with its cap and may exceed one even though the configured effort is bounded. The pinned Rapier API does not expose delivered native motor impulse readback. Recovery has a distinct [actuation path](dynamic-recovery.md#bounded-muscle-like-motors).

Gravity and balance compensation are expressed through contact-loaded ankle, hip, and trunk chains. They create no net internal force or unpaired torque. With no floor, the center of mass remains in free fall. With support, Rapier friction and normal impulses provide the external reaction.

## Measured contacts and commanded load allocation

`planContactLoads` first selects measured, loaded, upward-facing contacts, excluding released or otherwise unavailable supports. It expands each patch into its actual manifold points; the reported patch point is a fallback when no point list is available. Measured load determines eligibility and provides a reference share where that branch uses one. It does not itself prescribe the commanded force split.

Standing uses the projected-pressure branch. `distributeSupportLoad` constructs the measured support hull and uses mean-value coordinates for the final distribution, with limiting vertex and edge cases. A preferred retained-side distribution is used only when it can represent the requested pressure. A configured minimum measured-share fraction can retain part of the measured split; the projected branch does not generally preserve it. The nonprojected branch instead iterates toward pressure while regularizing toward measured shares.

Per-point allocations are combined into one equivalent resultant location per segment. That location can differ from the patch centroid. These planned forces generate joint compensation; they are not external forces injected at the floor. Friction, support-chain torque, and the 1.35-body-weight vertical cap bound the plan. `pressureFeasible` and `pressureForceResidualNm` expose incompatibility; inspect requested and allocated force separately. Planned pressure never substitutes for measured weight transfer.

Mean-value coordinates are continuous within a fixed nondegenerate hull. Changing which measured points own hull vertices can redistribute load abruptly. The [standing investigation](physics-standing-contract-2026-09-26.md) records this limitation and rejected experiments; no general continuity repair is accepted.

## Diagnostics and deterministic checks

Balance diagnostics report center of mass, velocity, capture point, support center and margin, available and measured supporting feet, recovery capacity, applied external force, balance acceleration, instability persistence, and step target. Top-level diagnostics additionally report joint coordinates and targets, structural-limit error, torque with its `motorTorqueSource`, request saturation, and measured contact counts and normal load. A bounded native request is not proof of delivered torque or successful balance.

Focused tests cover mass weighting, connected unreachable targets, heading equivariance, measured-foot exclusion, slow pulls, direction reversal, release during swing, stronger corrective stepping, and overpowering falls. The full harness adds 30-second standing at three headings, structural limits with posture motors absent, continuous body ownership, floorless free fall, recovery, input lockout, and lifecycle checks.
