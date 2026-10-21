# Native GetUp locomotion: first ownership increment

This opt-in work uses the current 21 scalar GetUp hinges. It does not enable
the incompatible legacy StepController, promote the closed static-transfer
experiment, or change the deployed default / V3 Recovery binaries.

`getup-native-locomotion-experiment` and profile selector
`getup_native_locomotion: "native-joint-owner-v1"` are both required.
The profile retains the existing repeated V3 recovery controller. Static
transfer and external stationary-controller overrides cannot coexist.

## State and motor authority

`getup_native_observation()` reads current generalized q/dq, body twists,
root and COM, resolved floor-foot contact points/impulses, and all-body
support loads without stepping. World Y is up; q/dq are in source hinge
order and normalized motor commands are in actuator order. Contact evidence
is from the completed native interval, not inferred from desired sole geometry.
The report explicitly limits detailed contacts to the flat floor.

The actual bounded `Grab::apply` impulse, point and body-COM torque are stamped
with their command-input native interval. Pointer displacement is not passed
off as delivered force. This ephemeral receipt is not restored as an input.

An explicit passive mode supplies zero active motor controls while preserving
gravity, contact, joint limits, passive stiffness and damping. Active re-entry
ramps the live joint-controller command from zero over 20 native samples.
It cannot interrupt fixture preparation or an incomplete recovery. Existing
fixture preparation, source recovery and terminal balance are separately
identified. Source holding and the terminal blend remain recovery-owned even
if the recovery counter has already incremented.

All controls still pass through the existing single `getup::apply_controls`:
equal/opposite hinge couples, no extra root force or pose/velocity writes.
The teacher fork clears this real-world owner selector. Serialized ownership
is validated against its profile/build/generation; it is not silently dropped.

## What this increment does not claim

`Step` is a reserved owner and is refused until a native dynamic command path
is implemented and checked. No step or walking success is claimed. Synthetic
earned-state tests exercise the real completed-recovery/current-800/blend-20
readiness predicate without replaying a physical get-up. They are not physical
acceptance. Actual grab-to-step, contact acquisition, fall handback and UI
exposure are the subsequent vertical slice; no new physical run is admitted
by this source checkpoint.

The full interface and its explicit trial reset/mode changes remain intact.
The earlier 21,560 integrated samples remain closed evidence, not a reusable
or reset budget for this new controller.
