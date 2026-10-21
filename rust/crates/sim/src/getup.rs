//! Pinned GetUp body, actuator contract and opt-in pretrained controller.
//! Serial reduced-coordinate hinges retain oblique axes and offset ankle pivots.
use crate::*;
pub mod actor;
pub mod adaptation;
pub mod controller;
pub mod fixtures;
pub mod physical_roll;
pub mod build_identity;
#[cfg(feature = "getup-terminal-tail-observer-experiment")]
pub mod roll_inspection;
pub mod reference;
pub mod standing;
pub mod transfer;
pub mod locomotion;
mod supported_standing;
#[cfg(feature = "standing-dynamics-experiment")]
pub use supported_standing::{dynamic_balance_diagnostic, dynamic_balance_fixture_invariants};
#[cfg(feature = "stationary-balance-experiment")]
pub mod balance;

pub fn source_vector(v: Vector) -> [f32; 3] {
    [v.z, v.x, v.y]
}

pub(crate) fn collider(part: &lh_model::Segment, index: usize) -> Result<ColliderBuilder, String> {
    let reference = part.reference.as_ref().ok_or("Missing GetUp link")?;
    if reference.source_body.is_none() {
        // An inert, disabled handle keeps presentation/contact indexing stable.
        // This coordinate frame has exactly zero mass, inertia and collision volume.
        return Ok(ColliderBuilder::ball(0.0).mass(0.0).enabled(false));
    }
    let shapes = reference
        .primitives
        .iter()
        .map(|p| {
            let a = vector(p.a);
            let b = vector(p.b);
            if p.kind == "sphere" {
                (Pose::from_translation(a), SharedShape::ball(p.radius))
            } else {
                (Pose::IDENTITY, SharedShape::capsule(a, b, p.radius))
            }
        })
        .collect::<Vec<_>>();
    let m = lh_model::reference_mass(part)?;
    Ok(ColliderBuilder::compound(shapes)
        .mass_properties(MassProperties::with_principal_inertia_frame(
            vector(m.center),
            part.mass_kg,
            vector(m.principal),
            rotation(m.frame),
        ))
        .friction(1.0)
        .friction_combine_rule(CoefficientCombineRule::Max)
        .restitution(0.0)
        .contact_skin(0.0)
        .user_data(index as u128 + 1)
        .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS))
}

pub(crate) fn assemble(s: &mut Simulation, support_height: f32) -> Result<(), String> {
    for (i, part) in s.model.segments.iter().enumerate() {
        if let (Some(parent), Some(p)) = (&part.parent, &part.joint_profile) {
            let parent = s
                .model
                .segments
                .iter()
                .position(|n| &n.id == parent)
                .ok_or("Missing source parent")?;
            s.world
                .insert_multibody_joint(s.bodies[parent], s.bodies[i], anatomical_joint(p)?)
                .ok_or("Invalid GetUp articulation")?;
        }
    }
    let id = *s
        .world
        .multibody_joints
        .rigid_body_link(s.bodies[0])
        .ok_or("Missing free root")?;
    let mb = s
        .world
        .multibody_joints
        .get_multibody_mut(id.multibody)
        .ok_or("Missing articulation")?;
    mb.forward_kinematics(&s.world.bodies, true);
    mb.set_self_contacts_enabled(true);
    if mb.ndofs() != 27 {
        return Err("GetUp requires free root plus 21 hinges".into());
    }
    // The pinned MJCF uses <freejoint>, whose source semantics explicitly
    // exclude joint-default damping and armature. Rapier otherwise gives its
    // free angular DOFs a 0.1 damping default. Keep all six root DOFs passive.
    mb.damping_mut().rows_mut(0, 6).fill(0.0);
    mb.armature_mut().rows_mut(0, 6).fill(0.0);
    let mut displacement = vec![0.0; mb.ndofs()];
    for (i, part) in s.model.segments.iter().enumerate() {
        if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let link = mb
                .links()
                .find(|l| l.rigid_body_handle() == s.bodies[i])
                .ok_or("Missing hinge link")?;
            let dof = link.assembly_id();
            let link_id = link.link_id();
            let axis = &part.joint_profile.as_ref().unwrap().axes[0];
            mb.armature_mut()[dof] = h.armature;
            mb.damping_mut()[dof] = axis.damping_nms_per_rad;
            mb.link_mut(link_id).unwrap().joint.set_spring(
                3,
                axis.passive_stiffness_nm_per_rad,
                0.0,
            );
            displacement[dof] = (s.model.reference_metadata["standingPose"][h.qpos_index]
                .as_f64()
                .ok_or("Missing source pose")? as f32)
                .clamp(axis.min_radians, axis.max_radians);
        }
    }
    // Initial construction only. No pose/velocity writes after tick zero.
    mb.apply_displacements(&displacement);
    mb.forward_kinematics(&s.world.bodies, false);
    mb.update_rigid_bodies(&mut s.world.bodies, true);
    s.world
        .bodies
        .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
    let bottom = s
        .colliders
        .iter()
        .filter(|h| s.world.colliders[**h].is_enabled())
        .map(|h| s.world.colliders[*h].compute_aabb().mins.y)
        .fold(f32::INFINITY, f32::min);
    displacement.fill(0.0);
    // Preserve the selected terrain elevation while normalizing source posture.
    displacement[1] = support_height + 0.002 - bottom;
    mb.apply_displacements(&displacement);
    mb.forward_kinematics(&s.world.bodies, false);
    mb.update_rigid_bodies(&mut s.world.bodies, true);
    s.world
        .bodies
        .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
    Ok(())
}

pub fn joint_state(s: &Simulation) -> Result<([f32; 21], [f32; 21]), String> {
    let mut q = [0.0; 21];
    let mut dq = [0.0; 21];
    for (i, p) in s.model.segments.iter().enumerate() {
        if let Some(h) = p.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let id = s
                .world
                .multibody_joints
                .rigid_body_link(s.bodies[i])
                .ok_or("Missing hinge")?;
            let mb = s
                .world
                .multibody_joints
                .get_multibody(id.multibody)
                .ok_or("Missing multibody")?;
            let link = mb.link(id.id).ok_or("Missing link")?;
            q[h.qpos_index] = link.joint.coords()[3];
            dq[h.qpos_index] = mb.joint_velocity(link)[0];
        }
    }
    Ok((q, dq))
}

/// Source SLOW: residuals and reference in qpos order; output in actuator order.
/// ctrl clipping precedes multiplication by gear. No target-velocity feedforward.
pub fn slow_controls(
    model: &Model,
    reference: &[f32; 21],
    residual: &[f32; 21],
    q: &[f32; 21],
    dq: &[f32; 21],
    holding: bool,
) -> Result<[f32; 21], String> {
    if reference
        .iter()
        .chain(residual)
        .chain(q)
        .chain(dq)
        .any(|x| !x.is_finite())
    {
        return Err("Non-finite GetUp command".into());
    }
    let kd = if holding { 0.2 } else { 0.1 };
    let mut controls = [0.0; 21];
    for p in &model.segments {
        if let Some(h) = p.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let i = h.qpos_index;
            controls[h.actuator_index] =
                (reference[i] + residual[i].clamp(-1.0, 1.0) - q[i] - kd * dq[i]).clamp(-1.0, 1.0);
        }
    }
    Ok(controls)
}

pub(crate) fn configure(s: &mut Simulation) -> Result<(), String> {
    locomotion::validate(s)?;
    let physical = physical_observation(s)?;
    let q: &[f32; 21] = physical[..21].try_into().unwrap();
    let dq: &[f32; 21] = physical[46..67].try_into().unwrap();
    #[cfg(feature = "stationary-balance-experiment")]
    if let Some(residual) = s.balance_residual {
        let reference = controller::standing_target(&s.model)[..21].try_into().unwrap();
        let controls = if let Some(source) = &s.balance_source_command {
            source.controls(&s.model, &residual, q, dq)?
        } else if let Some(profile) = &s.balance_motion_profile {
            let t = (s.substeps_completed - profile.start_substep) as f32 * s.profile.dt_s;
            if t > 0.025 { return Err("Expired experimental motion profile".into()); }
            let mut controls = [0.0; 21];
            for p in &s.model.segments {
                if let Some(h) = p.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
                    let i = h.qpos_index;
                    controls[h.actuator_index] = profile.control(i, h.gear, q[i], dq[i], t);
                }
            }
            controls
        } else { slow_controls(&s.model, &reference, &residual, q, dq, true)? };
        s.diagnostics.getup = Some(lh_contracts::GetUpObservation {
            controller: if let Some(source) = &s.balance_source_command { source.label() } else if s.balance_motion_profile.is_some() { "stationary-balance-motion-profile-untrained" } else { "stationary-balance-external-residual-untrained" }.into(),
            learned_policy_loaded: false,
            source_bodies: 16, coordinate_frames: 9, active_hinges: 21,
            joint_angles_rad: q.to_vec(), joint_velocities_radps: dq.to_vec(),
            motor_controls: controls.to_vec(), physics_step_s: s.profile.dt_s,
            policy_step_s: 0.025, observation: physical.to_vec(), policy: None,
        });
        apply_controls(s, &controls)?;
        let torso = s.model.segments.iter().position(|p| p.id == "torso").unwrap();
        s.world.bodies[s.bodies[torso]].add_force(
            Vector::new(s.balance_torso_force_n[0], 0.0, s.balance_torso_force_n[1]), true);
        return Ok(());
    }
    let controls = if s.getup_fixture.as_ref().is_some_and(|f|f.preparing) {
        Ok([0.0;21])
    } else if locomotion::passive(s) {
        Ok([0.0;21])
    } else if let Some(mut controller) = s.getup_controller.take() {
        let result = controller.command(s, &physical);
        s.getup_controller = Some(controller);
        result
    } else {
        let reference: [f32; 21] = controller::standing_target(&s.model)[..21]
            .try_into()
            .unwrap();
        supported_standing::controls(s, &reference, q, dq)
    };
    let owner = if s.getup_fixture.as_ref().is_some_and(|f|f.preparing) {
        locomotion::Owner::FixturePreparation
    } else if locomotion::passive(s) { locomotion::Owner::Passive
    } else if s.getup_controller.as_ref().is_some_and(|c|c.native_recovery_owner(s.motion,s.substeps_completed)) {
        locomotion::Owner::Recovery
    } else { locomotion::Owner::Balance };
    let controls = controls.and_then(|c|locomotion::finish_command(s,owner,c));
    s.diagnostics.getup = Some(lh_contracts::GetUpObservation {
        controller: if s.getup_fixture.as_ref().is_some_and(|f|f.preparing) {
            "fixture-zero-control-preparation"
        } else if locomotion::passive(s) {
            "native-passive-zero-motors"
        } else if let Some(controller) = &s.getup_controller {
            controller.controller_label()
        } else if cfg!(feature = "standing-dynamics-experiment") {
            "experimental-cop-complete-centroidal-standing-balance"
        } else {
            "measured-contact-standing-balance"
        }
        .into(),
        learned_policy_loaded: s.getup_controller.is_some(),
        source_bodies: 16,
        coordinate_frames: 9,
        active_hinges: 21,
        joint_angles_rad: q.to_vec(),
        joint_velocities_radps: dq.to_vec(),
        motor_controls: controls.as_ref().map(|c| c.to_vec()).unwrap_or_default(),
        physics_step_s: s.profile.dt_s,
        policy_step_s: 0.025,
        observation: physical.to_vec(),
        policy: s.getup_controller.as_ref().map(|c| c.status.clone()),
    });
    let controls = controls?;
    apply_controls(s, &controls)?;
    locomotion::record_application(s,owner,controls);
    Ok(())
}

#[cfg(feature = "getup-terminal-tail-observer-experiment")]
#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct MotorApplication {
    command_input_native_substep: u64,
    intended_controls: [f32; 21],
    applied_controls: [f32; 21],
    intended_torque_nm: [f32; 21],
    applied_torque_nm: [f32; 21],
    override_applied: bool,
    overridden_actuator_indices: Vec<usize>,
}

pub(crate) fn apply_controls(s: &mut Simulation, controls: &[f32; 21]) -> Result<(), String> {
    if controls.iter().any(|x| !x.is_finite() || x.abs() > 1.0) {
        return Err("Invalid normalized motor control".into());
    }
    let mut effort = 0.0;
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    let mut receipt = MotorApplication {
        command_input_native_substep: s.substeps_completed,
        intended_controls: *controls, applied_controls: *controls,
        intended_torque_nm: [0.0; 21], applied_torque_nm: [0.0; 21],
        override_applied: false, overridden_actuator_indices: Vec::new(),
    };
    for (i, p) in s.model.segments.iter().enumerate() {
        if let Some(h) = p.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let parent = s
                .model
                .segments
                .iter()
                .position(|n| Some(&n.id) == p.parent.as_ref())
                .ok_or("Missing actuator parent")?;
            let jp = p.joint_profile.as_ref().unwrap();
            let axis = (*s.world.bodies[s.bodies[parent]].rotation()
                * rotation(jp.parent_frame.rotation)
                * Vector::X)
                .normalize();
            let tau = h.gear * controls[h.actuator_index];
            s.world.bodies[s.bodies[i]].add_torque(axis * tau, true);
            s.world.bodies[s.bodies[parent]].add_torque(-axis * tau, true);
            effort += tau.abs();
            #[cfg(feature = "getup-terminal-tail-observer-experiment")]
            {
                receipt.intended_torque_nm[h.actuator_index] = tau;
                receipt.applied_torque_nm[h.actuator_index] = tau;
            }
        }
    }
    s.diagnostics.requested_motor_effort_nm = effort;
    #[cfg(feature = "getup-terminal-tail-observer-experiment")]
    { s.last_motor_application = Some(receipt); }
    Ok(())
}

#[cfg(test)]
#[path = "getup/motor_application_tests.rs"]
mod motor_application_tests;

/// Current twists from the articulation's generalized velocities, never stale
/// RigidBody velocity caches. Returned linear velocities are at each body's CoM.
/// Root generalized translation is the derivative of its free-joint origin.
pub(crate) fn current_twists(s: &Simulation) -> Result<Vec<(Vector, Vector)>, String> {
    let (_, dq) = joint_state(s)?;
    let id = s
        .world
        .multibody_joints
        .rigid_body_link(s.bodies[0])
        .ok_or("Missing root")?;
    let mb = s
        .world
        .multibody_joints
        .get_multibody(id.multibody)
        .ok_or("Missing articulation")?;
    let root = mb.joint_velocity(mb.link(id.id).ok_or("Missing root link")?);
    if root.len() != 6 {
        return Err("GetUp root must be free".into());
    }
    let mut origin = vec![(Vector::ZERO, Vector::ZERO); s.bodies.len()];
    origin[0] = (
        Vector::new(root[0], root[1], root[2]),
        Vector::new(root[3], root[4], root[5]),
    );
    for (i, p) in s.model.segments.iter().enumerate().skip(1) {
        let parent = s
            .model
            .segments
            .iter()
            .position(|n| Some(&n.id) == p.parent.as_ref())
            .ok_or("Missing parent")?;
        if parent >= i {
            return Err("GetUp tree is not topological".into());
        }
        let jp = p.joint_profile.as_ref().ok_or("Missing joint frame")?;
        let rb = &s.world.bodies[s.bodies[i]];
        let prb = &s.world.bodies[s.bodies[parent]];
        let rotation = prb.rotation().normalize();
        let anchor = prb.translation() + rotation * vector(jp.parent_frame.anchor);
        let axis = (rotation * crate::rotation(jp.parent_frame.rotation) * Vector::X).normalize();
        let rate = p
            .reference
            .as_ref()
            .and_then(|r| r.hinge.as_ref())
            .map_or(0.0, |h| dq[h.qpos_index]);
        let (v, w) = origin[parent];
        let angular = w + axis * rate;
        origin[i] = (
            v + w.cross(anchor - prb.translation()) + angular.cross(rb.translation() - anchor),
            angular,
        );
    }
    Ok(origin
        .into_iter()
        .enumerate()
        .map(|(i, (v, w))| {
            let body = &s.world.bodies[s.bodies[i]];
            (
                v + w.cross(body.rotation().normalize() * body.local_center_of_mass()),
                w,
            )
        })
        .collect())
}

pub fn physical_observation(s: &Simulation) -> Result<[f32; 67], String> {
    let (q, dq) = joint_state(s)?;
    let mut out = [0.0; 67];
    out[..21].copy_from_slice(&q);
    let body = |name: &str| -> Result<&RigidBody, String> {
        let i = s
            .model
            .segments
            .iter()
            .position(|p| p.id == name)
            .ok_or_else(|| format!("Missing {name}"))?;
        Ok(&s.world.bodies[s.bodies[i]])
    };
    let torso = body("torso")?;
    let frame = torso.rotation().normalize();
    out[21] = body("head")?.translation().y;
    for (k, name) in ["left_hand", "left_foot", "right_hand", "right_foot"]
        .into_iter()
        .enumerate()
    {
        out[22 + k * 3..25 + k * 3].copy_from_slice(&source_vector(
            frame.inverse() * (body(name)?.translation() - torso.translation()),
        ));
    }
    out[34..37].copy_from_slice(&source_vector(frame.inverse() * Vector::Y));
    let twists = current_twists(s)?;
    let com_velocity = s
        .bodies
        .iter()
        .zip(&twists)
        .map(|(h, (v, _))| *v * s.world.bodies[*h].mass())
        .sum::<Vector>()
        / s.total_mass();
    out[37..40].copy_from_slice(&source_vector(com_velocity));
    let id = s
        .world
        .multibody_joints
        .rigid_body_link(s.bodies[0])
        .ok_or("Missing root")?;
    let mb = s
        .world
        .multibody_joints
        .get_multibody(id.multibody)
        .ok_or("Missing articulation")?;
    let root = mb.joint_velocity(mb.link(id.id).ok_or("Missing root link")?);
    out[40..43].copy_from_slice(&source_vector(Vector::new(root[0], root[1], root[2])));
    out[43..46].copy_from_slice(&source_vector(
        frame.inverse() * Vector::new(root[3], root[4], root[5]),
    ));
    out[46..67].copy_from_slice(&dq);
    if out.iter().any(|x| !x.is_finite()) {
        return Err("Non-finite GetUp observation".into());
    }
    Ok(out)
}

pub fn slow_observation(physical: &[f32; 67], now: &[f32; 25], ahead: &[f32; 25]) -> [f32; 118] {
    let mut out = [0.0; 118];
    out[..67].copy_from_slice(physical);
    out[67..92].copy_from_slice(now);
    out[92..117].copy_from_slice(ahead);
    out[117] = 1.0;
    out
}

/// Signed primitive-to-floor vertical clearance, independent of whether a
/// unilateral contact currently carries impulse. This is observation only.
pub(crate) fn foot_floor_clearance(s: &Simulation) -> Result<[f32; 2], String> {
    let floor_top = s.world.colliders[s.floor].compute_aabb().maxs.y;
    let mut clearance = [0.0; 2];
    for (side, name) in ["left_foot", "right_foot"].into_iter().enumerate() {
        let i = s.model.segments.iter().position(|part| part.id == name)
            .ok_or("Missing observed foot")?;
        let body = &s.world.bodies[s.bodies[i]];
        let reference = s.model.segments[i].reference.as_ref().ok_or("Missing observed foot primitives")?;
        let bottom = reference.primitives.iter().flat_map(|primitive| {
            let endpoints = if primitive.kind == "sphere" { [primitive.a, primitive.a] }
                else { [primitive.a, primitive.b] };
            endpoints.map(|endpoint|
                (body.rotation() * vector(endpoint) + body.translation()).y - primitive.radius)
        }).fold(f32::INFINITY, f32::min);
        clearance[side] = bottom - floor_top;
    }
    if clearance.iter().any(|v| !v.is_finite()) { return Err("Invalid observed foot clearance".into()); }
    Ok(clearance)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn source_freejoint_has_zero_root_damping_and_armature_without_changing_hinges() {
        let s = Simulation::new(0.0, Profile::default(), false).unwrap();
        let id = s.world.multibody_joints.rigid_body_link(s.bodies[0]).unwrap();
        let mb = s.world.multibody_joints.get_multibody(id.multibody).unwrap();
        assert!(mb.damping().rows(0,6).iter().all(|x|*x==0.0));
        assert!(mb.armature().rows(0,6).iter().all(|x|*x==0.0));
        for (i,part) in s.model.segments.iter().enumerate() {
            assert_eq!(s.world.bodies[s.bodies[i]].angular_damping(),0.0);
            if let Some(hinge) = part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
                let link = mb.links().find(|l|l.rigid_body_handle()==s.bodies[i]).unwrap();
                assert_eq!(mb.damping()[link.assembly_id()],part.joint_profile.as_ref().unwrap().axes[0].damping_nms_per_rad);
                assert_eq!(mb.armature()[link.assembly_id()],hinge.armature);
            }
        }
    }
    #[test]
    fn source_assembly_retains_selected_terrain_elevation_without_integration() {
        let baseline = Simulation::new(0.0, Profile::default(), true).unwrap();
        for difficulty in crate::environment::DIFFICULTIES {
            for station in crate::environment::STATIONS {
                let settings = lh_contracts::PlaygroundSettings { station, difficulty };
                let s = Simulation::new_playground(0.0, Profile::default(), true, settings).unwrap();
                let offset = s.environment.as_ref().unwrap().spawn_offset(&s.world, &s.model).unwrap();
                for (body, original) in s.bodies.iter().zip(&baseline.bodies) {
                    let delta = s.world.bodies[*body].translation()
                        - baseline.world.bodies[*original].translation();
                    assert!(delta.distance(offset) < 0.00002, "{settings:?}: {delta:?} vs {offset:?}");
                }
                let bottom = s.colliders.iter().filter(|h| s.world.colliders[**h].is_enabled())
                    .map(|h| s.world.colliders[*h].compute_aabb().mins.y)
                    .fold(f32::INFINITY, f32::min);
                assert!((bottom - offset.y - 0.002).abs() < 0.00002, "{settings:?}");
                assert_eq!(s.substeps_completed, 0);
                assert_eq!(s.tick, 0);
            }
        }
    }
    #[test]
    fn observation_uses_current_generalized_motion_and_unit_frames_without_integration() {
        let mut s = Simulation::new(0.3, Profile::default(), true).unwrap();
        let id = *s
            .world
            .multibody_joints
            .rigid_body_link(s.bodies[0])
            .unwrap();
        let rates: Vec<f32> = (0..27).map(|i| (i as f32 * 0.37).sin() * 0.3).collect();
        s.world
            .multibody_joints
            .get_multibody_mut(id.multibody)
            .unwrap()
            .generalized_velocity_mut()
            .copy_from_slice(&rates);
        assert!(
            s.bodies
                .iter()
                .all(|h| s.world.bodies[*h].linvel() == Vector::ZERO)
        );
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let twists = current_twists(&s).unwrap();
        let observation = physical_observation(&s).unwrap();
        assert_eq!(&observation[40..43], &[rates[2], rates[0], rates[1]]);
        let mb = s
            .world
            .multibody_joints
            .get_multibody(id.multibody)
            .unwrap();
        let epsilon = 0.001;
        let forward: Vec<_> = rates.iter().map(|v| v * epsilon).collect();
        let backward: Vec<_> = rates.iter().map(|v| -v * epsilon).collect();
        let mut maximum_fd_error = 0.0_f32;
        for (i, h) in s.bodies.iter().enumerate() {
            let link = s.world.multibody_joints.rigid_body_link(*h).unwrap().id;
            let plus =
                mb.forward_kinematics_single_link(&s.world.bodies, link, Some(&forward), None);
            let minus =
                mb.forward_kinematics_single_link(&s.world.bodies, link, Some(&backward), None);
            let center = s.world.bodies[*h].local_center_of_mass();
            let derivative = (plus * center - minus * center) / (2.0 * epsilon);
            maximum_fd_error = maximum_fd_error.max(derivative.distance(twists[i].0));
            assert!(
                derivative.distance(twists[i].0) < 0.0003,
                "{} {:?} {:?}",
                s.model.segments[i].id,
                derivative,
                twists[i].0
            );
        }
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
        assert_eq!(s.substeps_completed, 0);
        println!(
            "Current-velocity observer: max CoM velocity error against geometric finite differences {maximum_fd_error} m/s; zero physics"
        );
        let root = s.bodies[0];
        let rotation = *s.world.bodies[root].rotation();
        s.world.bodies[root].set_rotation(
            Rotation::from_xyzw(
                rotation.x * 0.9999,
                rotation.y * 0.9999,
                rotation.z * 0.9999,
                rotation.w * 0.9999,
            ),
            false,
        );
        let observed = physical_observation(&s).unwrap();
        let norm = observed[34..37].iter().map(|x| x * x).sum::<f32>();
        assert!((norm - 1.0).abs() < 1e-6);
    }
    #[test]
    fn source_assembly_preserves_mass_hinges_and_valid_initial_pose_without_integration() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        assert_eq!(s.substeps_completed, 0);
        assert!((s.total_mass() - 40.84402122).abs() < 0.0001);
        let (q, dq) = joint_state(&s).unwrap();
        assert!(dq.iter().all(|x| *x == 0.0));
        let mut axes = 0;
        let mut bodies = 0;
        let mut geoms = 0;
        let mut virtuals = 0;
        for (i, p) in s.model.segments.iter().enumerate() {
            let r = p.reference.as_ref().unwrap();
            geoms += r.primitives.len();
            if r.source_body.is_some() {
                bodies += 1;
                assert!((s.world.bodies[s.bodies[i]].mass() - p.mass_kg).abs() < 1e-5);
            } else {
                virtuals += 1;
                assert_eq!(s.world.bodies[s.bodies[i]].mass(), 0.0);
                assert!(!s.world.colliders[s.colliders[i]].is_enabled());
            }
            if let Some(h) = &r.hinge {
                axes += 1;
                let a = &p.joint_profile.as_ref().unwrap().axes[0];
                assert!(
                    q[h.qpos_index] >= a.min_radians - 1e-6
                        && q[h.qpos_index] <= a.max_radians + 1e-6
                );
                let id = s
                    .world
                    .multibody_joints
                    .rigid_body_link(s.bodies[i])
                    .unwrap();
                let mb = s
                    .world
                    .multibody_joints
                    .get_multibody(id.multibody)
                    .unwrap();
                let link = mb.link(id.id).unwrap();
                let limits = &link.joint.data.limits[3];
                assert_eq!([limits.min, limits.max], [a.min_radians, a.max_radians]);
                let serialized = serde_json::to_string(&[limits.min, limits.max]).unwrap();
                let restored: [f32; 2] = serde_json::from_str(&serialized).unwrap();
                assert_eq!(restored, [limits.min, limits.max]);
                assert!(!link.joint.kinematic);
                assert_eq!(mb.armature()[link.assembly_id()], 0.01);
            }
        }
        assert_eq!((bodies, geoms, axes, virtuals), (16, 19, 21, 9));
        let expected = &s.model.reference_metadata["standingBodyPoses"];
        let expected_root: Vec3 =
            serde_json::from_value(expected["torso"]["position"].clone()).unwrap();
        let root = s.world.bodies[s.bodies[0]].translation();
        for (i, p) in s
            .model
            .segments
            .iter()
            .enumerate()
            .filter(|(_, p)| p.reference.as_ref().unwrap().source_body.is_some())
        {
            let position: Vec3 =
                serde_json::from_value(expected[&p.id]["position"].clone()).unwrap();
            let orientation: Quat =
                serde_json::from_value(expected[&p.id]["rotation"].clone()).unwrap();
            let body = &s.world.bodies[s.bodies[i]];
            assert!(
                (body.translation() - root - vector(position) + vector(expected_root)).length()
                    < 1e-5,
                "{} source FK position",
                p.id
            );
            assert!(
                (rotation(orientation).inverse() * *body.rotation())
                    .to_scaled_axis()
                    .length()
                    < 1e-5,
                "{} source FK orientation",
                p.id
            );
        }

        measurement::inspect(&mut s, 0).unwrap();
        assert!(s.diagnostics.max_self_penetration_m < 1e-5);
        assert!(s.diagnostics.max_floor_penetration_m < 1e-5);
        assert!(s.diagnostics.max_anchor_separation_m < 1e-5);
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let obs = physical_observation(&s).unwrap();
        assert_eq!(&obs[..21], &q);
        assert_eq!(&obs[46..], &dq);
        assert!(obs[21] > 1.4 && obs[21] < 1.8);
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }
    #[test]
    fn source_motor_permutation_clipping_and_slow_observation_contract() {
        let model = Model::canonical().unwrap();
        assert_eq!(
            model.reference_metadata["actuatorToQpos"],
            serde_json::json!([
                1, 0, 2, 3, 4, 5, 6, 8, 7, 9, 10, 11, 12, 14, 13, 15, 16, 17, 18, 19, 20
            ])
        );
        let mut q = [0.0; 21];
        q[0] = 0.2;
        q[1] = -0.3;
        q[7] = 2.0;
        q[8] = -2.0;
        let mut dq = [0.0; 21];
        dq[2] = 1.0;
        let u = slow_controls(&model, &[0.0; 21], &[0.0; 21], &q, &dq, false).unwrap();
        assert_eq!([u[0], u[1], u[2], u[7], u[8]], [0.3, -0.2, -0.1, 1.0, -1.0]);
        assert_eq!(
            slow_controls(&model, &[0.0; 21], &[0.0; 21], &q, &dq, true).unwrap()[2],
            -0.2
        );
        let obs = slow_observation(&[2.0; 67], &[3.0; 25], &[4.0; 25]);
        assert_eq!(&obs[..67], &[2.0; 67]);
        assert_eq!(&obs[67..92], &[3.0; 25]);
        assert_eq!(&obs[92..117], &[4.0; 25]);
        assert_eq!(obs[117], 1.0);
    }
}
