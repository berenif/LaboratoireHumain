//! Focused tests for the explicitly activated, root-only arithmetic experiment.

use super::multibody_joint::RootTranslationIntegratorState;
use crate::alloc_prelude::vec;
use crate::prelude::*;

fn free_root(position: Vector) -> (Multibody, RigidBodySet, RigidBodyHandle) {
    let mut bodies = RigidBodySet::new();
    let body = bodies.insert(RigidBodyBuilder::dynamic().translation(position));
    let mut mb = Multibody::with_root(body, false);
    mb.forward_kinematics(&bodies, true);
    bodies.index_mut_internal(body).changes = RigidBodyChanges::empty();
    (mb, bodies, body)
}

fn active_root(position: Vector) -> (Multibody, RigidBodySet, RigidBodyHandle) {
    let (mut mb, bodies, body) = free_root(position);
    mb.set_root_translation_compensation_enabled(true).unwrap();
    (mb, bodies, body)
}

fn remainder(mb: &Multibody) -> [f64; 3] {
    mb.root_translation_integrator_state()
        .unwrap()
        .remainderbits
        .map(f64::from_bits)
}

fn seed_remainder(mb: &mut Multibody) {
    let mut displacement = vec![0.0; mb.ndofs()];
    displacement[0] = 2.0_f32.powi(-26);
    mb.apply_displacements(&displacement);
    assert_ne!(remainder(mb)[0], 0.0);
}

#[test]
fn feature_starts_inactive_and_disabled_root_keeps_legacy_arithmetic() {
    let (mut mb, _, _) = free_root(Vector::ONE);
    assert_eq!(mb.root_translation_integrator_state(), None);
    let mut legacy = mb.root().joint;
    let displacement = [1.0e-9, -1.0e-9, 5.0e-8, 0.01, -0.02, 0.03];
    for _ in 0..500 {
        mb.apply_displacements(&displacement);
        legacy.apply_displacement(&displacement);
    }
    assert_eq!(mb.root().joint.coords(), legacy.coords());
    assert_eq!(mb.root().joint.joint_rot(), legacy.joint_rot());
    mb.set_root_translation_compensation_enabled(true).unwrap();
    seed_remainder(&mut mb);
    let before = mb.root_translation_integrator_state();
    mb.set_root_translation_compensation_enabled(true).unwrap();
    assert_eq!(before, mb.root_translation_integrator_state());
    mb.set_root_translation_compensation_enabled(false).unwrap();
    assert_eq!(mb.root_translation_integrator_state(), None);
}

#[test]
fn five_hundred_tiny_rows_conserve_requested_f32_displacements() {
    // Coordinates and the 500-row scale mirror the diagnostic, not its dynamics.
    let initial = Vector::new(0.15154053, 1.2816663, -0.58667237);
    let (mut mb, _, _) = active_root(initial);
    let mut legacy = mb.clone();
    legacy
        .set_root_translation_compensation_enabled(false)
        .unwrap();
    let velocities = [2.0e-5, 1.0e-3, -6.0e-4, 0.0, 0.0, 0.0];
    let dt = 0.00005;
    mb.generalized_velocity_mut().copy_from_slice(&velocities);
    legacy
        .generalized_velocity_mut()
        .copy_from_slice(&velocities);
    let mut requested = [0.0_f64; 3];
    for _ in 0..500 {
        for i in 0..3 {
            requested[i] += f64::from(velocities[i] * dt);
        }
        mb.integrate(dt);
        legacy.integrate(dt);
    }
    for i in 0..3 {
        let delivered = f64::from(mb.root().joint.coords()[i]) - f64::from(initial[i]);
        assert!((delivered + remainder(&mb)[i] - requested[i]).abs() < 1.0e-15);
    }
    assert_eq!(legacy.root().joint.coords()[0], initial.x);
    assert_ne!(mb.root().joint.coords()[0], initial.x);
}

#[test]
fn product_stays_f32_and_normal_integration_matches_displacement_path() {
    let (mut normal, _, _) = active_root(Vector::ONE);
    let mut displacement = normal.clone();
    let dt = 0.1_f32;
    let velocities = [0.1_f32, -0.3, 0.7, 0.02, -0.03, 0.04];
    normal
        .generalized_velocity_mut()
        .copy_from_slice(&velocities);
    normal.integrate(dt);
    displacement.apply_displacements(&velocities.map(|v| v * dt));
    assert_eq!(
        normal.root().joint.coords(),
        displacement.root().joint.coords()
    );
    assert_eq!(
        normal.root_translation_integrator_state(),
        displacement.root_translation_integrator_state()
    );
    let dq = velocities[0] * dt;
    assert_ne!(f64::from(dq), f64::from(velocities[0]) * f64::from(dt));
    assert_eq!(
        f64::from(normal.root().joint.coords()[0]) - 1.0 + remainder(&normal)[0],
        f64::from(dq)
    );
    // Angular products/quaternion operations still follow the literal legacy path.
    let mut legacy = MultibodyJoint::free(Pose::from_translation(Vector::ONE));
    legacy.integrate(dt, &velocities);
    assert_eq!(normal.root().joint.joint_rot(), legacy.joint_rot());
    for i in 3..6 {
        assert_eq!(normal.root().joint.coords()[i], legacy.coords()[i]);
    }
}

#[test]
fn negative_zero_cancellation_and_large_coordinates() {
    let (mut mb, _, _) = active_root(Vector::new(-1.0, 1.0, -0.0));
    let initial = mb.root().joint.coords();
    let mut requested = [0.0_f64; 3];
    let increments = [
        [2.0_f32.powi(-25), -2.0_f32.powi(-25), f32::from_bits(1)],
        [-2.0_f32.powi(-25), 2.0_f32.powi(-25), -f32::from_bits(1)],
        [0.0, -0.0, 0.0],
    ];
    for displacement in increments.into_iter().cycle().take(501) {
        let mut full = [0.0; 6];
        full[..3].copy_from_slice(&displacement);
        mb.apply_displacements(&full);
        for i in 0..3 {
            requested[i] += f64::from(displacement[i]);
        }
    }
    for i in 0..3 {
        assert_eq!(
            f64::from(mb.root().joint.coords()[i]) - f64::from(initial[i]) + remainder(&mb)[i],
            requested[i]
        );
    }
    let (mut large, _, _) = active_root(Vector::splat(16_777_216.0));
    for _ in 0..500 {
        large.apply_displacements(&[0.125, -0.125, 0.0, 0.0, 0.0, 0.0]);
    }
    assert_eq!(
        f64::from(large.root().joint.coords()[0]) - 16_777_216.0 + remainder(&large)[0],
        62.5
    );
    assert_eq!(
        f64::from(large.root().joint.coords()[1]) - 16_777_216.0 + remainder(&large)[1],
        -62.5
    );
}

#[test]
fn routine_fk_and_torque_preserve_residue_but_teleport_resets_it() {
    let (mut mb, mut bodies, body) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    mb.forward_kinematics(&bodies, false);
    mb.update_rigid_bodies(&mut bodies, true);
    bodies.index_mut_internal(body).changes = RigidBodyChanges::empty();
    let before = mb.root_translation_integrator_state();
    for _ in 0..10 {
        bodies[body].add_torque(Vector::Y, true);
        assert!(!bodies[body].changes.contains(RigidBodyChanges::POSITION));
        mb.forward_kinematics(&bodies, true);
        assert_eq!(before, mb.root_translation_integrator_state());
    }
    bodies[body].set_position(Pose::from_translation(Vector::splat(2.0)), true);
    mb.forward_kinematics(&bodies, true);
    assert_eq!(remainder(&mb), [0.0; 3]);
    assert_eq!(
        mb.root_translation_integrator_state().unwrap().qbits,
        [2.0_f32.to_bits(); 3]
    );
}

#[test]
fn routine_fk_zero_sign_canonicalization_preserves_other_axis_residue() {
    let (mut mb, mut bodies, body) = active_root(Vector::new(1.0, 1.0, -0.0));
    seed_remainder(&mut mb);
    let before = remainder(&mb);
    mb.forward_kinematics(&bodies, false);
    mb.update_rigid_bodies(&mut bodies, true);
    // Model the zero-sign canonicalization of otherwise identical f32 poses.
    bodies.index_mut_internal(body).pos.position.translation.z = 0.0;
    bodies
        .index_mut_internal(body)
        .pos
        .next_position
        .translation
        .z = 0.0;
    bodies.index_mut_internal(body).changes = RigidBodyChanges::empty();
    mb.forward_kinematics(&bodies, true);
    assert_eq!(before, remainder(&mb));
    assert_eq!(
        mb.root_translation_integrator_state().unwrap().qbits[2],
        0.0_f32.to_bits()
    );
}

#[test]
fn same_pose_set_position_is_noop_and_explicit_reset_is_available() {
    let (mut mb, mut bodies, body) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    let before = mb.root_translation_integrator_state();
    let pose = *bodies[body].position();
    bodies[body].set_position(pose, false);
    assert!(!bodies[body].changes.contains(RigidBodyChanges::POSITION));
    mb.forward_kinematics(&bodies, true);
    assert_eq!(before, mb.root_translation_integrator_state());
    mb.reset_root_translation_compensation();
    assert_eq!(remainder(&mb), [0.0; 3]);
    assert_eq!(*bodies[body].position(), pose);
}

#[test]
fn mutable_root_access_and_untracked_coordinate_edit_rebase_history() {
    let (mut mb, _, _) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    let _ = mb.root_mut();
    assert_eq!(remainder(&mb), [0.0; 3]);
    seed_remainder(&mut mb);
    let _ = mb.link_mut(0);
    assert_eq!(remainder(&mb), [0.0; 3]);
    seed_remainder(&mut mb);
    let _ = mb.links_mut();
    assert_eq!(remainder(&mb), [0.0; 3]);
    seed_remainder(&mut mb);
    mb.links[0].joint.coords[0] = 2.0;
    assert!(
        mb.clone()
            .restore_root_translation_integrator_state(
                mb.root_translation_integrator_state().unwrap()
            )
            .is_err()
    );
    mb.apply_displacements(&[0.0; 6]);
    assert_eq!(remainder(&mb), [0.0; 3]);
    assert_eq!(
        mb.root_translation_integrator_state().unwrap().qbits[0],
        2.0_f32.to_bits()
    );
}

#[test]
fn root_type_and_invalid_frames_disable_without_changing_fallback_arithmetic() {
    let (mut mb, mut bodies, body) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    bodies[body].set_body_type(RigidBodyType::Fixed, false);
    mb.forward_kinematics(&bodies, true);
    assert_eq!(mb.root_translation_integrator_state(), None);
    assert!(mb.set_root_translation_compensation_enabled(true).is_err());
    bodies[body].set_body_type(RigidBodyType::Dynamic, false);
    mb.forward_kinematics(&bodies, true);
    assert_eq!(mb.root_translation_integrator_state(), None);
    mb.set_root_translation_compensation_enabled(true).unwrap();
    mb.links[0].joint.data.local_frame2.translation.x = 0.1;
    let mut legacy = mb.root().joint;
    let disp = [1.0e-9; 6];
    legacy.apply_displacement(&disp);
    mb.apply_displacements(&disp);
    assert_eq!(mb.root_translation_integrator_state(), None);
    assert_eq!(mb.root().joint.coords(), legacy.coords());
    assert!(mb.set_root_translation_compensation_enabled(true).is_err());
}

#[test]
fn locked_or_kinematic_root_cannot_activate_and_uses_legacy_path() {
    for kinematic in [false, true] {
        let (mut mb, _, _) = active_root(Vector::ONE);
        seed_remainder(&mut mb);
        if kinematic {
            mb.links[0].joint.kinematic = true;
        } else {
            mb.links[0].joint.data.locked_axes = JointAxesMask::LIN_X;
        }
        let mut legacy = mb.root().joint;
        let disp = [0.001; 6];
        legacy.apply_displacement(&disp);
        mb.apply_displacements(&disp);
        assert_eq!(mb.root_translation_integrator_state(), None);
        assert_eq!(mb.root().joint.coords(), legacy.coords());
        assert_eq!(mb.root().joint.joint_rot(), legacy.joint_rot());
        assert!(mb.set_root_translation_compensation_enabled(true).is_err());
    }
}

#[test]
fn replacing_root_body_disables_history_even_when_coordinate_bits_match() {
    let (mut mb, mut bodies, _) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    let replacement = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::ONE));
    mb.root_mut().rigid_body = replacement;
    let stale = mb.root_translation_integrator_state().unwrap();
    assert!(
        mb.clone()
            .restore_root_translation_integrator_state(stale)
            .is_err()
    );
    mb.apply_displacements(&[0.0; 6]);
    assert_eq!(mb.root_translation_integrator_state(), None);
}

#[test]
fn child_links_keep_legacy_integration_and_mutation_preserves_root_residue() {
    let (mut mb, mut bodies, _) = active_root(Vector::ONE);
    let child = bodies.insert(RigidBodyBuilder::dynamic());
    mb.add_link(
        Some(0),
        MultibodyJoint::free(Pose::from_translation(Vector::ONE)),
        child,
    );
    seed_remainder(&mut mb);
    let before = mb.root_translation_integrator_state();
    let _ = mb.link_mut(1);
    assert_eq!(before, mb.root_translation_integrator_state());
    let mut legacy = mb.links[1].joint;
    let disp = [1.0e-9; 12];
    for _ in 0..500 {
        mb.apply_displacements(&disp);
        legacy.apply_displacement(&disp[6..]);
    }
    assert_eq!(mb.links[1].joint.coords(), legacy.coords());
    assert_eq!(mb.links[1].joint.joint_rot(), legacy.joint_rot());
}

#[test]
fn revolute_child_uses_literal_legacy_angular_updates() {
    let (mut mb, mut bodies, _) = active_root(Vector::ONE);
    let child = bodies.insert(RigidBodyBuilder::dynamic());
    let hinge = RevoluteJointBuilder::new(Vector::Z).build();
    mb.add_link(Some(0), MultibodyJoint::new(hinge.into(), false), child);
    let mut legacy = mb.links[1].joint;
    let disp = [1.0e-9, -2.0e-9, 3.0e-9, 0.0, 0.0, 0.0, 0.001];
    for _ in 0..500 {
        mb.apply_displacements(&disp);
        legacy.apply_displacement(&disp[6..]);
    }
    assert_eq!(mb.links[1].joint.coords(), legacy.coords());
    assert_eq!(mb.links[1].joint.joint_rot(), legacy.joint_rot());
}

#[test]
fn clone_and_branch_fk_copy_accumulator_without_mutating_live_state() {
    let (mut mb, bodies, _) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    let before = mb.root_translation_integrator_state();
    let disp = [3.0 * 2.0_f32.powi(-26), 0.0, 0.0, 0.01, 0.02, 0.03];
    let branch_pose = mb.forward_kinematics_single_link(&bodies, 0, Some(&disp), None);
    assert_eq!(before, mb.root_translation_integrator_state());
    let mut copied = mb.clone();
    assert_eq!(before, copied.root_translation_integrator_state());
    copied.apply_displacements(&disp);
    copied.forward_kinematics(&bodies, false);
    assert_eq!(branch_pose, *copied.root().local_to_world());
    assert_eq!(before, mb.root_translation_integrator_state());
    assert_ne!(before, copied.root_translation_integrator_state());
}

#[test]
fn append_rebases_retained_root_and_removal_starts_new_roots_inactive() {
    let (mut mb, mut bodies, _) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    let child = bodies.insert(RigidBodyBuilder::dynamic());
    let rhs = Multibody::with_root(child, false);
    mb.append(rhs, 0, MultibodyJoint::fixed(Pose::IDENTITY));
    assert_eq!(remainder(&mb), [0.0; 3]);
    seed_remainder(&mut mb);
    for split in mb.remove_link(1, true) {
        assert_eq!(split.root_translation_integrator_state(), None);
    }
}

#[test]
fn restore_validates_exact_bits_finite_values_and_one_ulp_bound() {
    let (mut mb, _, _) = active_root(Vector::ONE);
    let original = mb.root_translation_integrator_state().unwrap();
    for invalid_remainder in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        2.0_f64.powi(-22),
    ] {
        let mut invalid = original;
        invalid.remainderbits[0] = invalid_remainder.to_bits();
        assert!(
            mb.restore_root_translation_integrator_state(invalid)
                .is_err()
        );
        assert_eq!(mb.root_translation_integrator_state(), Some(original));
    }
    let mut mismatched = original;
    mismatched.qbits[0] += 1;
    assert!(
        mb.restore_root_translation_integrator_state(mismatched)
            .is_err()
    );
    let mut valid = original;
    valid.remainderbits[0] = 2.0_f64.powi(-23).to_bits();
    valid.remainderbits[1] = (-2.0_f64.powi(-23)).to_bits();
    mb.restore_root_translation_integrator_state(valid).unwrap();
    assert_eq!(mb.root_translation_integrator_state(), Some(valid));
}

#[test]
fn half_ulp_double_rounding_history_and_extreme_boundaries_roundtrip() {
    let (mut mb, _, _) = active_root(Vector::ONE);
    mb.apply_displacements(&[2.0_f32.powi(-24), 0.0, 0.0, 0.0, 0.0, 0.0]);
    mb.apply_displacements(&[2.0_f32.powi(-60), 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!(remainder(&mb)[0] > 2.0_f64.powi(-24));
    let state = mb.root_translation_integrator_state().unwrap();
    mb.clone()
        .restore_root_translation_integrator_state(state)
        .unwrap();
    for q in [
        0.0,
        -0.0,
        f32::from_bits(1),
        -f32::from_bits(1),
        f32::MAX,
        -f32::MAX,
    ] {
        let (mut boundary, _, _) = active_root(Vector::splat(q));
        let mut state = boundary.root_translation_integrator_state().unwrap();
        let ulp = if q.abs() == f32::MAX {
            2.0_f64.powi(104)
        } else {
            f64::from(f32::from_bits(1))
        };
        state.remainderbits = [ulp.to_bits(), (-ulp).to_bits(), (-0.0_f64).to_bits()];
        boundary
            .restore_root_translation_integrator_state(state)
            .unwrap();
        assert_eq!(boundary.root_translation_integrator_state(), Some(state));
        state.remainderbits[0] = (ulp * 2.0).to_bits();
        assert!(
            boundary
                .restore_root_translation_integrator_state(state)
                .is_err()
        );
    }
}

#[test]
fn nonfinite_integration_remains_nonfinite_and_cannot_be_restored() {
    let (mut mb, _, _) = active_root(Vector::splat(f32::MAX));
    mb.apply_displacements(&[f32::MAX, f32::NAN, f32::INFINITY, 0.0, 0.0, 0.0]);
    for i in 0..3 {
        assert!(!mb.root().joint.coords()[i].is_finite());
    }
    let state = mb.root_translation_integrator_state().unwrap();
    assert!(mb.restore_root_translation_integrator_state(state).is_err());
    assert!(!mb.root().joint.coords()[0].is_finite());
}

#[test]
#[cfg(feature = "serde-serialize")]
fn raw_world_serialization_omits_active_residue_and_sidecar_keeps_exact_bits() {
    let (mut mb, _, _) = active_root(Vector::ONE);
    seed_remainder(&mut mb);
    let mut inactive = mb.clone();
    inactive
        .set_root_translation_compensation_enabled(false)
        .unwrap();
    let bytes = bincode::serialize(&mb).unwrap();
    assert_eq!(bytes, bincode::serialize(&inactive).unwrap());
    let mut restored: Multibody = bincode::deserialize(&bytes).unwrap();
    assert_eq!(restored.root_translation_integrator_state(), None);
    let state = mb.root_translation_integrator_state().unwrap();
    let json = serde_json::to_string(&state).unwrap();
    let exact: RootTranslationIntegratorState = serde_json::from_str(&json).unwrap();
    assert_eq!(state, exact);
    restored
        .restore_root_translation_integrator_state(exact)
        .unwrap();
    assert_eq!(restored.root_translation_integrator_state(), Some(state));
    let unknown = json.replace('}', ",\"unknown\":1}");
    assert!(serde_json::from_str::<RootTranslationIntegratorState>(&unknown).is_err());
}
