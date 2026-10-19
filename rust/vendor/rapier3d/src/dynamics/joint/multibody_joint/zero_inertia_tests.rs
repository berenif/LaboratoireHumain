//! Matrix-only arithmetic fixtures, never integrated physical trajectories.
use super::Multibody;
use crate::alloc_prelude::*;
use crate::prelude::*;
use crate::utils::AngularInertiaOps;
use alloc::format;

fn fixture(heading: Real, phase: Real, mass_mode: usize) -> (Multibody, RigidBodySet) {
    fixture_with_root(heading, phase, mass_mode, false)
}

fn fixture_with_root(
    heading: Real,
    phase: Real,
    mass_mode: usize,
    fixed_root: bool,
) -> (Multibody, RigidBodySet) {
    // Same body/DoF cardinality and branching pattern as the GetUp assembly;
    // independent synthetic masses/geometry, including massive descendants of
    // exactly massless coordinate frames and three fixed attachments.
    let parents = [
        0, 0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 4, 11, 12, 13, 14, 15, 0, 17, 18, 19, 0, 21, 22, 23,
    ];
    let frames = [2, 5, 6, 9, 11, 12, 15, 17, 21];
    let fixed = [1, 20, 24];
    let mut bodies = RigidBodySet::new();
    let mut joints = MultibodyJointSet::new();
    let handles: Vec<_> = (0..25)
        .map(|i| {
            let mut mass = if frames.contains(&i) {
                0.0
            } else {
                0.5 + i as Real * 0.03
            };
            let mut inertia = if frames.contains(&i) {
                Vector::ZERO
            } else {
                Vector::new(0.02, 0.03, 0.04) * (1.0 + i as Real * 0.02)
            };
            if i == 2 && mass_mode == 1 {
                inertia = Vector::new(0.01, 0.02, 0.03); // zero mass is not enough
            }
            if i == 2 && mass_mode == 2 {
                mass = 0.2; // zero angular inertia is not enough
            }
            let builder = if fixed_root && i == 0 {
                RigidBodyBuilder::fixed()
            } else {
                RigidBodyBuilder::dynamic()
            };
            bodies.insert(
                builder
                    .translation(Vector::new(i as Real * 0.01, 1.0, -0.02))
                    .rotation(Vector::Y * heading)
                    .additional_mass_properties(MassProperties::new(Vector::ZERO, mass, inertia)),
            )
        })
        .collect();
    let colliders = ColliderSet::new();
    for handle in &handles {
        bodies[*handle].recompute_mass_properties_from_colliders(&colliders);
    }
    for i in 1..25 {
        let anchor1 = Vector::new(0.02, -0.07, 0.03);
        let anchor2 = Vector::new(-0.01, 0.025, -0.02);
        let joint: GenericJoint = if fixed.contains(&i) {
            FixedJointBuilder::new()
                .local_anchor1(anchor1)
                .local_anchor2(anchor2)
                .build()
                .into()
        } else {
            RevoluteJointBuilder::new(Vector::new(1.0, 0.3, -0.2).normalize())
                .local_anchor1(anchor1)
                .local_anchor2(anchor2)
                .build()
                .into()
        };
        joints
            .insert(handles[parents[i]], handles[i], joint, true)
            .unwrap();
    }
    let id = *joints.rigid_body_link(handles[0]).unwrap();
    let mb = joints.get_multibody_mut(id.multibody).unwrap();
    mb.forward_kinematics(&bodies, true);
    assert_eq!(mb.ndofs(), if fixed_root { 21 } else { 27 });
    let mut displacement = vec![0.0; mb.ndofs()];
    for (i, value) in displacement
        .iter_mut()
        .enumerate()
        .skip(if fixed_root { 0 } else { 6 })
    {
        *value = phase * ((i * 7 % 11) as Real - 5.0) * 0.03;
    }
    mb.apply_displacements(&displacement);
    mb.forward_kinematics(&bodies, false);
    mb.update_rigid_bodies(&mut bodies, true);
    for i in 0..mb.ndofs() {
        mb.velocities[i] = phase * ((i * 3 % 13) as Real - 6.0) * 0.04;
        mb.armature[i] = 0.01 + i as Real * 0.001;
        mb.damping[i] = 0.02 + i as Real * 0.002;
    }
    for link in &mut mb.links.0 {
        link.joint.set_spring(3, 0.1, 0.0);
    }
    mb.update_velocities(&mut bodies);
    (mb.clone(), bodies)
}

fn assert_bits(label: &str, actual: &[Real], reference: &[Real]) {
    assert_eq!(actual.len(), reference.len(), "{label} length");
    for (i, (a, b)) in actual.iter().zip(reference).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "{label}[{i}]: {a:?} != {b:?}");
    }
}

fn assert_matrix_and_factor_parity(mb: &Multibody, bodies: &RigidBodySet, dt: Real) {
    assert_matrix_and_factor_parity_against::<false>(mb, bodies, dt)
}

fn assert_matrix_and_factor_parity_against<const REFERENCE_ZERO_SKIP: bool>(
    mb: &Multibody,
    bodies: &RigidBodySet,
    dt: Real,
) {
    let mut actual = mb.clone();
    let mut reference = mb.clone();
    actual.update_mass_matrix_impl::<true, true>(dt, bodies);
    reference.update_mass_matrix_impl::<REFERENCE_ZERO_SKIP, false>(dt, bodies);
    assert_bits(
        "M",
        actual.augmented_mass.as_slice(),
        reference.augmented_mass.as_slice(),
    );
    assert_bits(
        "M_acc",
        actual.acc_augmented_mass.as_slice(),
        reference.acc_augmented_mass.as_slice(),
    );
    assert_bits(
        "LU",
        actual.inv_augmented_mass.lu_internal().as_slice(),
        reference.inv_augmented_mass.lu_internal().as_slice(),
    );
    assert_bits(
        "LU_acc",
        actual.acc_inv_augmented_mass.lu_internal().as_slice(),
        reference.acc_inv_augmented_mass.lu_internal().as_slice(),
    );
    assert_eq!(
        format!("{:?}", actual.inv_augmented_mass.p()),
        format!("{:?}", reference.inv_augmented_mass.p())
    );
    assert_eq!(
        format!("{:?}", actual.acc_inv_augmented_mass.p()),
        format!("{:?}", reference.acc_inv_augmented_mass.p())
    );
    assert_bits(
        "C scratch",
        actual.i_coriolis_dt.as_slice(),
        reference.i_coriolis_dt.as_slice(),
    );
    for i in 0..actual.links.len() {
        assert_bits(
            "C_v",
            actual.coriolis_v[i].as_slice(),
            reference.coriolis_v[i].as_slice(),
        );
        assert_bits(
            "C_w",
            actual.coriolis_w[i].as_slice(),
            reference.coriolis_w[i].as_slice(),
        );
    }
}

#[test]
fn exact_zero_inertia_elision_preserves_mass_matrices_lu_and_coriolis_bits() {
    for heading in [
        0.0,
        core::f32::consts::FRAC_PI_3 as Real,
        -core::f32::consts::FRAC_PI_4 as Real,
    ] {
        for phase in [0.0, 0.7, -1.3] {
            for mass_mode in 0..3 {
                let (mb, bodies) = fixture(heading, phase, mass_mode);
                let zero_count = mb
                    .links
                    .iter()
                    .filter(|link| {
                        let rb = &bodies[link.rigid_body];
                        rb.mprops.effective_mass() == Vector::ZERO
                            && rb.mprops.effective_angular_inertia().into_matrix()
                                == crate::math::Matrix::ZERO
                    })
                    .count();
                assert_eq!(zero_count, if mass_mode == 0 { 9 } else { 8 });
                assert_matrix_and_factor_parity(&mb, &bodies, 0.00125 / 25.0);
            }
        }
    }
}

#[test]
fn exact_zero_inertia_nonfinite_operands_retain_dense_propagation_bits() {
    for value in [Real::NAN, Real::INFINITY] {
        let (mb, bodies) = fixture(0.4, 0.7, 0);
        assert_matrix_and_factor_parity(&mb, &bodies, value);
        let mut bad_jacobian = mb.clone();
        bad_jacobian.body_jacobians[2][(0, 0)] = value;
        assert_matrix_and_factor_parity(&bad_jacobian, &bodies, 0.00125 / 25.0);
        let mut bad_early_massive = mb.clone();
        bad_early_massive.body_jacobians[0][(0, 0)] = value;
        assert_matrix_and_factor_parity(&bad_early_massive, &bodies, 0.00125 / 25.0);
        let mut bad_velocity = bodies.clone();
        bad_velocity[mb.links[2].rigid_body].vels.angvel.x = value;
        assert_matrix_and_factor_parity(&mb, &bad_velocity, 0.00125 / 25.0);
    }
    let (mut mb, mut bodies) = fixture(-0.4, -0.8, 0);
    mb.body_jacobians[2][(0, 0)] = -0.0;
    assert_matrix_and_factor_parity(&mb, &bodies, -0.0);
    for link in &mb.links.0 {
        bodies[link.rigid_body].vels.angvel = Vector::splat(Real::MAX * 0.75);
    }
    assert_matrix_and_factor_parity(&mb, &bodies, 0.00125 / 25.0);
}

#[test]
fn ancestor_prefix_elision_matches_zero_only_and_dense_reference_bits() {
    for fixed_root in [false, true] {
        for kinematic_link in [false, true] {
            for phase in [0.0, 0.7, -1.3] {
                let (mut mb, bodies) = fixture_with_root(0.6, phase, 0, fixed_root);
                if kinematic_link {
                    mb.links[3].joint.kinematic = true;
                }
                for dt in [0.00125 / 25.0, 0.00125 / 10.0, -0.0] {
                    assert_matrix_and_factor_parity_against::<true>(&mb, &bodies, dt);
                    assert_matrix_and_factor_parity(&mb, &bodies, dt);
                }
                // Repeated matrix updates retain all scratch/reordering state.
                mb.update_mass_matrix_impl::<true, true>(0.00125 / 25.0, &bodies);
                assert_matrix_and_factor_parity_against::<true>(&mb, &bodies, 0.00125 / 25.0);
            }
        }
    }
    let (mb, bodies) = fixture(0.4, 0.7, 0);
    for value in [-0.0, Real::NAN, Real::INFINITY] {
        let mut suffix = mb.clone();
        let end = suffix.links[3].assembly_id + suffix.links[3].joint.ndofs();
        suffix.body_jacobians[3][(0, end)] = value;
        assert_matrix_and_factor_parity_against::<true>(&suffix, &bodies, 0.00125 / 25.0);
    }
    let mut huge = mb.clone();
    let end = huge.links[3].assembly_id + huge.links[3].joint.ndofs();
    huge.body_jacobians[3].fill(0.0);
    huge.body_jacobians[3]
        .columns_mut(0, end)
        .fill(Real::MAX / 100.0);
    let mut heavy = bodies.clone();
    heavy[huge.links[3].rigid_body].mprops.effective_inv_mass = Vector::splat(1.0e-19);
    assert!(
        (heavy[huge.links[3].rigid_body].mprops.effective_mass().x
            * huge.body_jacobians[3][(0, 0)])
            .is_infinite()
    );
    let mut overflow_reference = huge.clone();
    overflow_reference.update_mass_matrix_impl::<true, false>(0.00125 / 25.0, &heavy);
    assert!(overflow_reference.augmented_mass[(end, 0)].is_nan());
    assert_matrix_and_factor_parity_against::<true>(&huge, &heavy, 0.00125 / 25.0);
}

#[test]
fn bounded_prefix_operands_match_dense_at_and_above_eligibility_boundary() {
    for magnitude in [65536.0, 65536.0 * (1.0 + Real::EPSILON)] {
        let (mut mb, mut bodies) = fixture(0.4, 0.7, 0);
        let link = &mb.links[3];
        let end = link.assembly_id + link.joint.ndofs();
        bodies[link.rigid_body].mprops.effective_inv_mass = Vector::splat(1.0 / magnitude);
        mb.body_jacobians[3].fill(0.0);
        mb.body_jacobians[3].columns_mut(0, end).fill(magnitude);
        assert_matrix_and_factor_parity_against::<true>(&mb, &bodies, 0.00125 / 25.0);
        assert_matrix_and_factor_parity(&mb, &bodies, 0.00125 / 25.0);
    }
}

#[test]
fn bounded_prefix_invalidates_on_massless_coriolis_before_later_links() {
    let (mut mb, bodies) = fixture(0.4, 0.7, 0);
    // Keep body velocity/J finite so the local zero-inertia skip still applies.
    // Only the massless frame's joint velocity poisons Coriolis propagation.
    let rb = &bodies[mb.links[2].rigid_body];
    assert_eq!(rb.mprops.effective_mass(), Vector::ZERO);
    assert_eq!(
        rb.mprops.effective_angular_inertia().into_matrix(),
        crate::math::Matrix::ZERO
    );
    assert!(rb.vels.angvel.is_finite());
    assert!(mb.body_jacobians[2].iter().all(|x| x.is_finite()));
    mb.links[2].joint_velocity.linvel.x = Real::INFINITY;
    let mut reference = mb.clone();
    reference.update_mass_matrix_impl::<true, false>(0.00125 / 25.0, &bodies);
    assert!(reference.coriolis_v[2].iter().any(|x| x.is_nan()));
    assert!(reference.acc_augmented_mass.iter().any(|x| x.is_nan()));
    assert_matrix_and_factor_parity_against::<true>(&mb, &bodies, 0.00125 / 25.0);
    assert_matrix_and_factor_parity(&mb, &bodies, 0.00125 / 25.0);
}
