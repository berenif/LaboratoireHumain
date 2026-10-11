//! Bounded Rapier predictions from an already prepared physical substep.
//!
//! Capture only after all forces, motors, environment updates and PGS settings
//! for the substep have been installed. Predictions own their copied worlds;
//! neither capture nor evaluation changes the real simulation.

use crate::Simulation;
use rapier3d::prelude::*;

/// A change to one anatomical motor velocity target. The existing force cap,
/// stiffness, damping, and target position are retained byte for byte.
/// A ForceBased velocity target drives an implicit, coupled constraint: it does
/// not promise an arbitrary mean torque. Plan from the complete native response;
/// a zero delta retains the command instead of re-inverting delivered impulses.
#[derive(Clone, Copy, Debug)]
pub(crate) struct MotorTargetDelta {
    pub joint: ImpulseJointHandle,
    pub axis: JointAxis,
    pub delta_target_velocity_rad_s: f32,
}

pub(crate) const DEFAULT_EVALUATION_BUDGET: usize = 32;
pub(crate) const MAX_HORIZON_STEPS: usize = 4;

pub(crate) struct PreparedNativeStep<'a> {
    baseline: PhysicsWorld,
    filter: &'a crate::Filter,
    body_handles: Vec<RigidBodyHandle>,
    allowed_motors: Vec<(ImpulseJointHandle, usize)>,
    remaining_evaluations: usize,
    remaining_integrated_steps: usize,
}

pub(crate) struct NativePrediction {
    world: PhysicsWorld,
    body_handles: Vec<RigidBodyHandle>,
}

pub(crate) struct NativeHorizonPrediction {
    predictions: Vec<NativePrediction>,
}

impl NativeHorizonPrediction {
    pub(crate) fn first(&self) -> &NativePrediction {
        &self.predictions[0]
    }

    pub(crate) fn terminal(&self) -> &NativePrediction {
        self.predictions.last().unwrap()
    }

    pub(crate) fn steps(&self) -> usize {
        self.predictions.len()
    }

    /// Every predicted physical state in chronological order, including the
    /// first state used to verify the next real integration.
    pub(crate) fn predictions(&self) -> &[NativePrediction] {
        &self.predictions
    }
}

impl NativePrediction {
    /// Read-only access to the simulated world, including resolved contacts.
    pub(crate) fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    pub(crate) fn body(&self, index: usize) -> Option<&RigidBody> {
        self.body_handles
            .get(index)
            .and_then(|handle| self.world.bodies.get(*handle))
    }

    pub(crate) fn body_count(&self) -> usize {
        self.body_handles.len()
    }
}

impl<'a> PreparedNativeStep<'a> {
    pub(crate) fn capture(sim: &'a Simulation) -> Self {
        Self::with_budget(sim, DEFAULT_EVALUATION_BUDGET)
    }

    pub(crate) fn with_budget(sim: &'a Simulation, max_evaluations: usize) -> Self {
        let allowed_motors = sim
            .model
            .segments
            .iter()
            .zip(&sim.joints)
            .filter_map(|(segment, handle)| Some((segment.joint_profile.as_ref()?, (*handle)?)))
            .flat_map(|(profile, handle)| {
                profile
                    .axes
                    .iter()
                    .map(move |axis| (handle, crate::axis(axis.coordinate).unwrap() as usize))
            })
            .collect();
        Self {
            baseline: copy_world(&sim.world),
            filter: &sim.filter,
            body_handles: sim.bodies.clone(),
            allowed_motors,
            remaining_evaluations: max_evaluations,
            remaining_integrated_steps: max_evaluations.saturating_mul(MAX_HORIZON_STEPS),
        }
    }

    pub(crate) fn remaining_evaluations(&self) -> usize {
        self.remaining_evaluations
    }

    pub(crate) fn remaining_integrated_steps(&self) -> usize {
        self.remaining_integrated_steps
    }

    pub(crate) fn predict(
        &mut self,
        changes: &[MotorTargetDelta],
    ) -> Result<NativePrediction, String> {
        let mut world = self.prepare(changes, 1)?;
        world.step_with_events(self.filter, &());
        Ok(NativePrediction {
            world,
            body_handles: self.body_handles.clone(),
        })
    }

    /// Hold the installed motor commands constant for up to four native
    /// integrations. The first result is compared with the next real step;
    /// the terminal result is only a conditional lookahead, not a state to
    /// install or an assertion about future controller choices.
    pub(crate) fn predict_horizon(
        &mut self,
        changes: &[MotorTargetDelta],
        steps: usize,
    ) -> Result<NativeHorizonPrediction, String> {
        let mut world = self.prepare(changes, steps)?;
        let mut predictions = Vec::with_capacity(steps);
        for _ in 0..steps {
            world.step_with_events(self.filter, &());
            predictions.push(NativePrediction {
                world: copy_world(&world),
                body_handles: self.body_handles.clone(),
            });
        }
        Ok(NativeHorizonPrediction { predictions })
    }

    /// Predict the same bounded velocity changes as `predict_horizon`, then
    /// re-emit the explicitly supplied motor commands before every later step.
    /// This models a controller which rebuilds absolute velocities between
    /// integrations. Positions, gains, caps, models and motor masks must remain
    /// identical to the prepared baseline. Live solver impulses are preserved.
    pub(crate) fn predict_reprepared_horizon(
        &mut self,
        changes: &[MotorTargetDelta],
        later_commands: &[(ImpulseJointHandle, GenericJoint)],
        steps: usize,
    ) -> Result<NativeHorizonPrediction, String> {
        let mut seen = Vec::new();
        for (handle, command) in later_commands {
            if seen.contains(handle) {
                return Err("Duplicate later native motor command".into());
            }
            seen.push(*handle);
            let original = &self
                .baseline
                .impulse_joints
                .get(*handle)
                .ok_or("Unknown later native motor joint")?
                .data;
            if command.motor_axes != original.motor_axes
                || command.enforce_motor_vector_limit != original.enforce_motor_vector_limit
            {
                return Err("Later command changes native motor masks or vector cap".into());
            }
            for (axis, (a, b)) in command.motors.iter().zip(original.motors).enumerate() {
                if a.target_pos.to_bits() != b.target_pos.to_bits()
                    || a.stiffness.to_bits() != b.stiffness.to_bits()
                    || a.damping.to_bits() != b.damping.to_bits()
                    || a.max_force.to_bits() != b.max_force.to_bits()
                    || a.model != b.model
                {
                    return Err("Later command changes native position, gain, cap or model".into());
                }
                if !a.target_vel.is_finite()
                    || (a.target_vel.to_bits() != b.target_vel.to_bits()
                        && !self.allowed_motors.contains(&(*handle, axis)))
                {
                    return Err("Invalid later anatomical motor velocity".into());
                }
            }
        }
        let mut world = self.prepare(changes, steps)?;
        let mut predictions = Vec::with_capacity(steps);
        for step in 0..steps {
            if step > 0 {
                for (handle, command) in later_commands {
                    restore_motor_command(
                        &mut world
                            .impulse_joints
                            .get_mut(*handle, true)
                            .ok_or("Later native joint disappeared during copy")?
                            .data,
                        command,
                    );
                }
            }
            world.step_with_events(self.filter, &());
            predictions.push(NativePrediction {
                world: copy_world(&world),
                body_handles: self.body_handles.clone(),
            });
        }
        Ok(NativeHorizonPrediction { predictions })
    }

    fn prepare(
        &mut self,
        changes: &[MotorTargetDelta],
        steps: usize,
    ) -> Result<PhysicsWorld, String> {
        if !(1..=MAX_HORIZON_STEPS).contains(&steps) {
            return Err("Native prediction horizon must contain one to four steps".into());
        }
        if self.remaining_evaluations == 0 || self.remaining_integrated_steps < steps {
            return Err("Native prediction evaluation or integration budget exhausted".into());
        }
        // Validate against the untouched baseline before cloning or spending a
        // physics step. Applying the same axis twice would hide a caller error.
        let mut seen = Vec::with_capacity(changes.len());
        for change in changes {
            if !matches!(
                change.axis,
                JointAxis::AngX | JointAxis::AngY | JointAxis::AngZ
            ) {
                return Err("Native prediction accepts anatomical angular motors only".into());
            }
            if !change.delta_target_velocity_rad_s.is_finite() {
                return Err("Non-finite native motor target delta".into());
            }
            let key = (change.joint.0.into_raw_parts(), change.axis as usize);
            if seen.contains(&key) {
                return Err("Duplicate native motor target delta".into());
            }
            if !self
                .allowed_motors
                .contains(&(change.joint, change.axis as usize))
            {
                return Err("Motor is not a modeled anatomical joint axis".into());
            }
            seen.push(key);
            let joint = self
                .baseline
                .impulse_joints
                .get(change.joint)
                .ok_or("Unknown native motor joint")?;
            let motor = &joint.data.motors[change.axis as usize];
            if !motor.max_force.is_finite() || motor.max_force < 0.0 {
                return Err("Invalid native motor force cap".into());
            }
            if !(motor.target_vel + change.delta_target_velocity_rad_s).is_finite() {
                return Err("Non-finite resulting native motor target".into());
            }
        }

        let mut world = copy_world(&self.baseline);
        for change in changes {
            // Do not call get_mut for a zero delta: get_mut marks the joint as
            // edited, which would needlessly perturb a baseline prediction.
            if change.delta_target_velocity_rad_s != 0.0 {
                let joint = world
                    .impulse_joints
                    .get_mut(change.joint, true)
                    .ok_or("Native motor joint disappeared during copy")?;
                joint.data.motors[change.axis as usize].target_vel +=
                    change.delta_target_velocity_rad_s;
            }
        }
        self.remaining_evaluations -= 1;
        self.remaining_integrated_steps -= steps;
        Ok(world)
    }
}

/// Reinstall command parameters without replacing warm-start or delivery data.
fn restore_motor_command(data: &mut GenericJoint, command: &GenericJoint) {
    data.motor_axes = command.motor_axes;
    data.enforce_motor_vector_limit = command.enforce_motor_vector_limit;
    for (motor, prior) in data.motors.iter_mut().zip(command.motors) {
        motor.target_pos = prior.target_pos;
        motor.target_vel = prior.target_vel;
        motor.stiffness = prior.stiffness;
        motor.damping = prior.damping;
        motor.max_force = prior.max_force;
        motor.model = prior.model;
    }
}

// Rapier's PhysicsWorld does not implement Clone. Every persistent set is
// cloneable; the pipeline itself is workspace data and is omitted by Rapier's
// serde snapshots. Construct a fresh pipeline just as deserialization does,
// while preserving cached contacts, islands, joints and CCD state directly.
fn copy_world(source: &PhysicsWorld) -> PhysicsWorld {
    PhysicsWorld {
        gravity: source.gravity,
        integration_parameters: source.integration_parameters,
        physics_pipeline: PhysicsPipeline::new(),
        islands: source.islands.clone(),
        broad_phase: source.broad_phase.clone(),
        narrow_phase: source.narrow_phase.clone(),
        bodies: source.bodies.clone(),
        colliders: source.colliders.clone(),
        impulse_joints: source.impulse_joints.clone(),
        multibody_joints: source.multibody_joints.clone(),
        ccd_solver: source.ccd_solver.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Profile;
    use std::time::Instant;

    fn assert_bodies_bitwise_equal(a: &Simulation, b: &NativePrediction) {
        assert_eq!(b.body_count(), 25);
        for (i, handle) in a.bodies.iter().enumerate() {
            let real = &a.world.bodies[*handle];
            let predicted = b.body(i).unwrap();
            assert_eq!(
                real.translation().to_array().map(f32::to_bits),
                predicted.translation().to_array().map(f32::to_bits)
            );
            assert_eq!(
                real.rotation().to_array().map(f32::to_bits),
                predicted.rotation().to_array().map(f32::to_bits)
            );
            assert_eq!(
                real.linvel().to_array().map(f32::to_bits),
                predicted.linvel().to_array().map(f32::to_bits)
            );
            assert_eq!(
                real.angvel().to_array().map(f32::to_bits),
                predicted.angvel().to_array().map(f32::to_bits)
            );
        }
    }

    fn assert_motor_impulses_bitwise_equal(a: &Simulation, b: &NativePrediction) {
        for handle in a.joints.iter().flatten() {
            let actual = &a.world.impulse_joints.get(*handle).unwrap().data;
            let predicted = &b.world().impulse_joints.get(*handle).unwrap().data;
            for (x, y) in actual.motors.iter().zip(predicted.motors) {
                assert_eq!(x.impulse.to_bits(), y.impulse.to_bits());
                assert_eq!(x.delivered_impulse.to_bits(), y.delivered_impulse.to_bits());
                assert_eq!(
                    x.delivered_abs_impulse.to_bits(),
                    y.delivered_abs_impulse.to_bits()
                );
                assert_eq!(
                    x.delivered_angular_impulse_1.to_array().map(f32::to_bits),
                    y.delivered_angular_impulse_1.to_array().map(f32::to_bits)
                );
                assert_eq!(
                    x.delivered_angular_impulse_2.to_array().map(f32::to_bits),
                    y.delivered_angular_impulse_2.to_array().map(f32::to_bits)
                );
            }
        }
    }

    #[test]
    fn reconstructed_four_step_horizon_matches_loaded_contact_execution() {
        // Actual prepared witness at 10.016667 s, before the first divergent
        // block. Captured with the original command policy and physical caps.
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("native_horizon_regression_commands.json")).unwrap();
        let mut sim =
            Simulation::new(0., crate::recovery_contract::pinned_profile(), true).unwrap();
        assert_eq!(
            sim.bodies,
            serde_json::from_value::<Vec<RigidBodyHandle>>(fixture["bodyHandles"].clone()).unwrap()
        );
        assert_eq!(
            sim.colliders,
            serde_json::from_value::<Vec<ColliderHandle>>(fixture["colliderHandles"].clone())
                .unwrap()
        );
        assert_eq!(
            sim.floor,
            serde_json::from_value::<ColliderHandle>(fixture["floor"].clone()).unwrap()
        );
        sim.world =
            serde_json::from_str(include_str!("native_horizon_regression_world.json")).unwrap();
        let mut commands: Vec<(ImpulseJointHandle, GenericJoint)> =
            serde_json::from_value(fixture["retainedCommands"].clone()).unwrap();
        for delta in fixture["cumulativeDeltas"].as_array().unwrap() {
            let handle: ImpulseJointHandle =
                serde_json::from_value(delta["joint"].clone()).unwrap();
            let (_, command) = commands.iter_mut().find(|(h, _)| *h == handle).unwrap();
            command.motors[delta["axis"].as_u64().unwrap() as usize].target_vel +=
                delta["delta"].as_f64().unwrap() as f32;
        }
        let mut model = PreparedNativeStep::with_budget(&sim, 3);
        let frozen = model.predict_horizon(&[], 4).unwrap();
        let exact = model.predict_reprepared_horizon(&[], &commands, 4).unwrap();
        let mut bad = commands.clone();
        bad[0].1.motors[3].max_force += 1.;
        assert!(model.predict_reprepared_horizon(&[], &bad, 4).is_err());
        assert_eq!(model.remaining_evaluations(), 1);
        drop(model);
        let load: f32 = sim
            .colliders
            .iter()
            .map(|h| crate::measurement::floor_normal_impulse(&sim.world, sim.floor, *h).unwrap())
            .sum();
        assert!(
            load > 0.,
            "Requires loaded native contacts, not just geometry"
        );
        let mut frozen_diverged = false;
        for step in 0..4 {
            if step > 0 {
                for (handle, command) in &commands {
                    restore_motor_command(
                        &mut sim
                            .world
                            .impulse_joints
                            .get_mut(*handle, true)
                            .unwrap()
                            .data,
                        command,
                    );
                }
            }
            sim.world.step_with_events(&sim.filter, &());
            let prediction = &exact.predictions()[step];
            let mut actual_hash = 0xcbf29ce484222325_u64;
            for handle in &sim.bodies {
                let body = &sim.world.bodies[*handle];
                for value in body
                    .translation()
                    .to_array()
                    .into_iter()
                    .chain(body.rotation().to_array())
                    .chain(body.linvel().to_array())
                    .chain(body.angvel().to_array())
                {
                    for byte in value.to_bits().to_le_bytes() {
                        actual_hash = (actual_hash ^ byte as u64).wrapping_mul(0x100000001b3);
                    }
                }
            }
            assert_eq!(
                format!("{actual_hash:016x}"),
                fixture["expectedActualSteps"][step]["bodyHashF32Fnv1a"]
                    .as_str()
                    .unwrap(),
                "Serialized witness must reproduce independently recorded real execution"
            );
            for (index, collider) in sim.colliders.iter().enumerate() {
                let actual_load =
                    crate::measurement::floor_normal_impulse(&sim.world, sim.floor, *collider)
                        .unwrap()
                        / sim.profile.dt_s;
                let recorded_load = fixture["expectedActualSteps"][step]["normalLoadsN"][index]
                    .as_f64()
                    .unwrap() as f32;
                assert_eq!(actual_load.to_bits(), recorded_load.to_bits());
            }
            assert_bodies_bitwise_equal(&sim, prediction);
            assert_motor_impulses_bitwise_equal(&sim, prediction);
            for collider in &sim.colliders {
                let a = crate::measurement::floor_normal_impulse(&sim.world, sim.floor, *collider)
                    .unwrap();
                let b = crate::measurement::floor_normal_impulse(
                    prediction.world(),
                    sim.floor,
                    *collider,
                )
                .unwrap();
                assert_eq!(
                    a.to_bits(),
                    b.to_bits(),
                    "step {} collider {:?}",
                    step + 1,
                    collider
                );
            }
            assert_eq!(
                serde_json::to_vec(&sim.world.narrow_phase.contact_pairs().collect::<Vec<_>>())
                    .unwrap(),
                serde_json::to_vec(
                    &prediction
                        .world()
                        .narrow_phase
                        .contact_pairs()
                        .collect::<Vec<_>>()
                )
                .unwrap()
            );
            let stale = frozen.predictions()[step].world();
            let different = sim.colliders.iter().any(|h| {
                crate::measurement::floor_normal_impulse(&sim.world, sim.floor, *h)
                    .unwrap()
                    .to_bits()
                    != crate::measurement::floor_normal_impulse(stale, sim.floor, *h)
                        .unwrap()
                        .to_bits()
            });
            if step == 0 {
                assert!(!different);
            } else {
                frozen_diverged |= different;
            }
        }
        assert!(
            frozen_diverged,
            "Regression must distinguish the original frozen-command horizon"
        );
    }

    #[test]
    fn zero_velocity_delta_preserves_warm_contacts_and_motor_delivery() {
        let fixtures: serde_json::Value = serde_json::from_str(super::super::FIXTURES).unwrap();
        let seed: Vec<lh_model::Pose> = serde_json::from_value(
            fixtures["cases"]
                .as_array()
                .unwrap()
                .iter()
                .find(|c| c["id"] == "landed-crouch-left")
                .unwrap()["poses"]
                .clone(),
        )
        .unwrap();
        let mut sim = Simulation::build(
            0.,
            crate::recovery_contract::pinned_profile(),
            true,
            None,
            Some(&seed),
        )
        .unwrap();
        sim.world.integration_parameters.num_internal_pgs_iterations = sim
            .profile
            .fall_internal_pgs_iterations
            .max(sim.profile.internal_pgs_iterations);
        for _ in 0..8 {
            sim.world.step_with_events(&sim.filter, &());
        }
        let actual_load: f32 = sim
            .colliders
            .iter()
            .map(|h| crate::measurement::floor_normal_impulse(&sim.world, sim.floor, *h).unwrap())
            .sum();
        assert!(
            actual_load > 0.,
            "Regression requires native resolved floor contact"
        );
        let zero: Vec<_> = sim
            .model
            .segments
            .iter()
            .zip(&sim.joints)
            .filter_map(|(segment, h)| {
                Some(MotorTargetDelta {
                    joint: (*h)?,
                    axis: crate::axis(segment.joint_profile.as_ref()?.axes[0].coordinate).unwrap(),
                    delta_target_velocity_rad_s: 0.,
                })
            })
            .collect();
        let mut prepared = PreparedNativeStep::with_budget(&sim, 2);
        let original = prepared.predict(&[]).unwrap();
        let explicit_zero = prepared.predict(&zero).unwrap();
        for handle in sim.joints.iter().flatten() {
            let before = &sim.world.impulse_joints.get(*handle).unwrap().data;
            let after = &explicit_zero
                .world()
                .impulse_joints
                .get(*handle)
                .unwrap()
                .data;
            assert_eq!(before.motor_axes, after.motor_axes);
            assert_eq!(
                before.enforce_motor_vector_limit,
                after.enforce_motor_vector_limit
            );
            for (x, y) in before.motors.iter().zip(after.motors) {
                assert_eq!(x.target_pos.to_bits(), y.target_pos.to_bits());
                assert_eq!(x.target_vel.to_bits(), y.target_vel.to_bits());
                assert_eq!(x.stiffness.to_bits(), y.stiffness.to_bits());
                assert_eq!(x.damping.to_bits(), y.damping.to_bits());
                assert_eq!(x.max_force.to_bits(), y.max_force.to_bits());
                assert_eq!(x.model, y.model);
            }
        }
        drop(prepared);
        sim.world.step_with_events(&sim.filter, &());
        for prediction in [&original, &explicit_zero] {
            assert_bodies_bitwise_equal(&sim, prediction);
            assert_motor_impulses_bitwise_equal(&sim, prediction);
        }
    }

    #[test]
    fn unperturbed_prediction_matches_all_physical_bodies() {
        let mut sim = Simulation::new(0.0, Profile::default(), true).unwrap();
        let mut predictor = PreparedNativeStep::capture(&sim);
        let predicted = predictor.predict(&[]).unwrap();
        drop(predictor);
        sim.world.step_with_events(&sim.filter, &());
        assert_bodies_bitwise_equal(&sim, &predicted);
        assert_motor_impulses_bitwise_equal(&sim, &predicted);
    }

    #[test]
    fn invalid_changes_and_budget_leave_world_untouched() {
        let sim = Simulation::new(0.0, Profile::default(), true).unwrap();
        let handle = sim.joints.iter().flatten().copied().next().unwrap();
        let mut predictor = PreparedNativeStep::with_budget(&sim, 1);
        let bad = MotorTargetDelta {
            joint: handle,
            axis: JointAxis::AngX,
            delta_target_velocity_rad_s: f32::NAN,
        };
        assert!(predictor.predict(&[bad]).is_err());
        assert_eq!(predictor.remaining_evaluations(), 1);
        assert_eq!(predictor.remaining_integrated_steps(), 4);
        let zero = MotorTargetDelta {
            delta_target_velocity_rad_s: 0.0,
            ..bad
        };
        assert!(predictor.predict(&[zero, zero]).is_err());
        assert_eq!(predictor.remaining_evaluations(), 1);
        assert!(predictor.predict_horizon(&[], 0).is_err());
        assert!(predictor.predict_horizon(&[], 5).is_err());
        assert_eq!(predictor.remaining_integrated_steps(), 4);
        let _ = predictor.predict(&[]).unwrap();
        assert_eq!(predictor.remaining_integrated_steps(), 3);
        assert!(predictor.predict(&[]).is_err());
    }

    #[test]
    fn perturbation_isolated_from_baseline_and_real_world() {
        let sim = Simulation::new(0.0, Profile::default(), true).unwrap();
        let handle = sim.joints.iter().flatten().copied().next().unwrap();
        let original = sim.world.impulse_joints.get(handle).unwrap().data.motors[3].target_vel;
        let mut predictor = PreparedNativeStep::with_budget(&sim, 3);
        let before = predictor.predict(&[]).unwrap();
        let altered = predictor
            .predict(&[MotorTargetDelta {
                joint: handle,
                axis: JointAxis::AngX,
                delta_target_velocity_rad_s: 0.2,
            }])
            .unwrap();
        let after = predictor.predict(&[]).unwrap();
        assert_eq!(
            sim.world.impulse_joints.get(handle).unwrap().data.motors[3]
                .target_vel
                .to_bits(),
            original.to_bits()
        );
        assert_eq!(
            before
                .world()
                .impulse_joints
                .get(handle)
                .unwrap()
                .data
                .motors[3]
                .target_vel
                .to_bits(),
            original.to_bits()
        );
        assert_eq!(
            after
                .world()
                .impulse_joints
                .get(handle)
                .unwrap()
                .data
                .motors[3]
                .target_vel
                .to_bits(),
            original.to_bits()
        );
        assert_eq!(
            altered
                .world()
                .impulse_joints
                .get(handle)
                .unwrap()
                .data
                .motors[3]
                .target_vel
                .to_bits(),
            (original + 0.2).to_bits()
        );
        for index in 0..25 {
            let a = before.body(index).unwrap();
            let b = after.body(index).unwrap();
            assert_eq!(
                a.translation().to_array().map(f32::to_bits),
                b.translation().to_array().map(f32::to_bits)
            );
            assert_eq!(
                a.rotation().to_array().map(f32::to_bits),
                b.rotation().to_array().map(f32::to_bits)
            );
            assert_eq!(
                a.linvel().to_array().map(f32::to_bits),
                b.linvel().to_array().map(f32::to_bits)
            );
            assert_eq!(
                a.angvel().to_array().map(f32::to_bits),
                b.angvel().to_array().map(f32::to_bits)
            );
        }
    }

    /// Replays the archived motor requests into the actual native world, then
    /// compares a fresh clone at the loaded, near-limit state. The report path
    /// is explicit so this focused regression does not depend on evidence
    /// files being part of a published crate.
    #[test]
    #[ignore = "set LH_NATIVE_PREFIX_REPORT to matched-a/baseline.json"]
    fn archived_loaded_prefix_matches_25_bodies_and_times_copy() {
        let report_path = std::env::var("LH_NATIVE_PREFIX_REPORT").unwrap();
        let report: serde_json::Value =
            serde_json::from_slice(&std::fs::read(report_path).unwrap()).unwrap();
        let fixtures: serde_json::Value = serde_json::from_str(super::super::FIXTURES).unwrap();
        let seed: Vec<lh_model::Pose> = serde_json::from_value(
            fixtures["cases"]
                .as_array()
                .unwrap()
                .iter()
                .find(|case| case["id"] == "landed-crouch-left")
                .unwrap()["poses"]
                .clone(),
        )
        .unwrap();
        let mut sim = Simulation::build(
            0.0,
            crate::recovery_contract::pinned_profile(),
            true,
            None,
            Some(&seed),
        )
        .unwrap();
        let frames = report["trace"].as_array().unwrap();
        let critical = frames.len() - 1;
        for (step, frame) in frames.iter().enumerate() {
            for actuation in frame["actuation"].as_array().unwrap() {
                let id = actuation["id"].as_str().unwrap();
                let index = sim.model.segments.iter().position(|s| s.id == id).unwrap();
                let joint = sim
                    .world
                    .impulse_joints
                    .get_mut(sim.joints[index].unwrap(), true)
                    .unwrap();
                for motor in actuation["motors"].as_array().unwrap() {
                    let coordinate = motor["coordinate"]
                        .as_str()
                        .unwrap()
                        .chars()
                        .next()
                        .unwrap();
                    let axis = crate::axis(coordinate).unwrap();
                    joint
                        .data
                        .set_motor_model(axis, MotorModel::ForceBased)
                        .set_motor(
                            axis,
                            motor["target"].as_f64().unwrap() as f32,
                            motor["targetVelocityRadS"].as_f64().unwrap() as f32,
                            motor["stiffnessNmPerRad"].as_f64().unwrap() as f32,
                            motor["dampingNmsPerRad"].as_f64().unwrap() as f32,
                        )
                        .set_motor_max_force(axis, motor["ceilingNm"].as_f64().unwrap() as f32);
                }
            }
            sim.world.integration_parameters.num_internal_pgs_iterations = sim
                .profile
                .fall_internal_pgs_iterations
                .max(sim.profile.internal_pgs_iterations);
            let mut horizon = None;
            if step == critical {
                let copy_start = Instant::now();
                let mut predictor = PreparedNativeStep::with_budget(&sim, 9);
                let copy_time = copy_start.elapsed();
                let mut durations = Vec::new();
                let mut predicted = None;
                for _ in 0..8 {
                    let start = Instant::now();
                    predicted = Some(predictor.predict(&[]).unwrap());
                    durations.push(start.elapsed());
                }
                let horizon_start = Instant::now();
                horizon = Some(predictor.predict_horizon(&[], 4).unwrap());
                let horizon_time = horizon_start.elapsed();
                assert_eq!(horizon.as_ref().unwrap().steps(), 4);
                assert_eq!(predictor.remaining_evaluations(), 0);
                assert_eq!(predictor.remaining_integrated_steps(), 24);
                drop(predictor);
                sim.world.step_with_events(&sim.filter, &());
                assert_bodies_bitwise_equal(&sim, predicted.as_ref().unwrap());
                assert_motor_impulses_bitwise_equal(&sim, predicted.as_ref().unwrap());
                assert_bodies_bitwise_equal(&sim, horizon.as_ref().unwrap().first());
                assert_motor_impulses_bitwise_equal(&sim, horizon.as_ref().unwrap().first());
                eprintln!(
                    "native prepared copy {:?}, clone-plus-step {:?}, clone-plus-four-steps {:?}",
                    copy_time, durations, horizon_time
                );
            } else {
                sim.world.step_with_events(&sim.filter, &());
            }
            assert_eq!(
                serde_json::to_value(&sim.snapshot().segments).unwrap(),
                frame["snapshot"]["segments"],
                "archived replay diverged at physical step {step}"
            );
            if step == critical {
                for predicted in horizon.as_ref().unwrap().predictions().iter().skip(1) {
                    sim.world.step_with_events(&sim.filter, &());
                    assert_bodies_bitwise_equal(&sim, predicted);
                    assert_motor_impulses_bitwise_equal(&sim, predicted);
                }
                assert_bodies_bitwise_equal(&sim, horizon.as_ref().unwrap().terminal());
            }
        }
    }
}
