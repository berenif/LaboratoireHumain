//! Read-only journal at the CCD solve boundary. This is a planner prerequisite,
//! not a support certificate: angular delivery and a roundoff bound remain open.
use crate::measurement;
use rapier3d::prelude::*;
use rapier3d::utils::OrthonormalBasis;
use serde::Serialize;
use std::cell::RefCell;

#[derive(Clone, Debug, Serialize)]
pub(crate) struct FloorImpulse {
    pub collider: ColliderHandle,
    pub normal_ns: [f64; 3],
    pub tangent_ns: [f64; 3],
    pub points: Vec<FloorPoint>,
}
#[derive(Clone, Debug, Serialize)]
pub(crate) struct FloorPoint {
    pub manifold_index: usize,
    pub selected_contact_index: u32,
    pub friction: f32,
    pub normal_impulse_ns: f32,
    pub tangent_impulse_world_ns: [f32; 3],
    pub frozen_world_point: [f32; 3],
    pub body_local_point: [f32; 3],
    pub point_velocity_before_solve_mps: [f32; 3],
    pub point_velocity_after_solve_mps: [f32; 3],
    pub tangent_speed_before_solve_mps: f32,
    pub tangent_speed_after_solve_mps: f32,
}
#[derive(Clone, Debug, Serialize)]
pub(crate) struct SolveRecord {
    pub native_substep: u64,
    pub ccd_interval: usize,
    pub dt_s: f64,
    pub momentum_before_ns: [f64; 3],
    pub momentum_after_ns: [f64; 3],
    pub gravity_and_user_impulse_ns: [f64; 3],
    pub floor_impulse_ns: [f64; 3],
    pub linear_residual_ns: [f64; 3],
    pub floors: Vec<FloorImpulse>,
    pub observation_error: Option<String>,
    pub angular_balance_verified: bool,
    pub angular_before_fixed_ns: [f64; 3],
    pub angular_after_fixed_ns: [f64; 3],
    pub external_angular_ns: [f64; 3],
    pub floor_angular_ns: [f64; 3],
    pub inverse_damping_estimate_ns: [f64; 3],
    pub unresolved_angular_ns: [f64; 3],
    pub angular_after_positions_ns: Option<[f64; 3]>,
    pub geometry_transport_ns: Option<[f64; 3]>,
}
struct Journal {
    bodies: Vec<RigidBodyHandle>,
    colliders: Vec<ColliderHandle>,
    floor: ColliderHandle,
    gravity: Vector,
    substep: u64,
    interval: usize,
    before: [f64; 3],
    external: [f64; 3],
    records: Vec<SolveRecord>,
    angular_before: [f64; 3],
    external_angular: [f64; 3],
    before_velocities: Vec<(Vector, Vector)>,
}
thread_local! {static ACTIVE: RefCell<Option<Journal>> = const {RefCell::new(None)};}
pub(crate) fn activate(s: &crate::Simulation) {
    assert!(
        s.grab.is_none() && s.striker.is_none() && s.environment.is_none(),
        "This diagnostic excludes interaction and moving environment forces"
    );
    assert_eq!(s.bodies.len(), 25);
    assert!(
        matches!(
            s.world.integration_parameters.friction_model,
            FrictionModel::Coulomb
        ),
        "Only Coulomb angular reporting is admitted"
    );
    assert_eq!(
        s.world.colliders.len(),
        26,
        "Only the native floor is admitted"
    );
    assert!(
        s.world.multibody_joints.iter().next().is_none(),
        "Multibody observation is not admitted"
    );
    assert!(
        s.world
            .bodies
            .iter()
            .all(|(handle, b)| s.bodies.contains(&handle) || b.is_fixed()),
        "Unmeasured dynamic/kinematic body"
    );
    for handle in &s.bodies {
        let b = &s.world.bodies[*handle];
        assert!(
            b.is_dynamic() && !b.is_sleeping() && b.activation().normalized_linear_threshold < 0.0
        );
        assert_eq!(
            b.linear_damping(),
            0.0,
            "Linear damping is not accounted for"
        );
        assert!(
            b.mass_properties().flags.is_empty(),
            "Locked axes are not admitted"
        );
    }
    ACTIVE.with(|slot| {
        assert!(slot.borrow().is_none(), "Already observing a history");
        *slot.borrow_mut() = Some(Journal {
            bodies: s.bodies.clone(),
            colliders: s.colliders.clone(),
            floor: s.floor,
            gravity: s.world.gravity,
            substep: 0,
            interval: 0,
            before: [0.0; 3],
            external: [0.0; 3],
            records: Vec::new(),
            angular_before: [0.0; 3],
            external_angular: [0.0; 3],
            before_velocities: vec![(Vector::ZERO, Vector::ZERO); 25],
        });
    });
}
pub(crate) fn begin_step(substep: u64) {
    ACTIVE.with(|slot| {
        if let Some(j) = &mut *slot.borrow_mut() {
            j.substep = substep;
            j.interval = 0;
        }
    });
}
fn add_scaled(out: &mut [f64; 3], vector: Vector, factor: f64) {
    for (a, b) in out.iter_mut().zip(vector.to_array()) {
        *a += f64::from(b) * factor;
    }
}
fn momentum(handles: &[RigidBodyHandle], bodies: &RigidBodySet) -> [f64; 3] {
    let mut result = [0.0; 3];
    for h in handles {
        let b = &bodies[*h];
        add_scaled(&mut result, b.linvel(), f64::from(b.mass()));
    }
    result
}
pub(crate) fn before_solve(dt: Real, bodies: &RigidBodySet) {
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(j) = slot.as_mut() else { return };
        j.before = momentum(&j.bodies, bodies);
        j.angular_before = angular(&j.bodies, bodies);
        j.external_angular = [0.0; 3];
        j.external = [0.0; 3];
        for (i, h) in j.bodies.iter().enumerate() {
            let b = &bodies[*h];
            j.before_velocities[i] = (b.linvel(), b.angvel());
            // This first worker has no moving environment, grab or striker.
            // Sleep/damping/locked translation require separate admission.
            if !b.is_sleeping() {
                add_scaled(
                    &mut j.external,
                    j.gravity,
                    f64::from(dt) * f64::from(b.mass()) * f64::from(b.gravity_scale()),
                );
                add_scaled(&mut j.external, b.user_force(), f64::from(dt));
                let mut impulse = [0.0; 3];
                add_scaled(
                    &mut impulse,
                    j.gravity,
                    f64::from(dt) * f64::from(b.mass()) * f64::from(b.gravity_scale()),
                );
                add_scaled(&mut impulse, b.user_force(), f64::from(dt));
                add(
                    &mut j.external_angular,
                    cross(b.center_of_mass().to_array().map(f64::from), impulse),
                );
                add_scaled(&mut j.external_angular, b.user_torque(), f64::from(dt));
            }
        }
    });
}
pub(crate) fn after_solve(dt: Real, bodies: &RigidBodySet, phase: &NarrowPhase) {
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(j) = slot.as_mut() else { return };
        let mut floors = Vec::new();
        let mut total = [0.0; 3];
        let mut floor_angular = [0.0; 3];
        let mut error = None;
        for collider in &j.colliders {
            let Some(pair) = phase.contact_pair(j.floor, *collider) else {
                continue;
            };
            let result = (|| -> Result<FloorImpulse, String> {
                let normal = measurement::solver_normal_impulse(pair)?;
                let first = pair.collider1 == *collider;
                let body_index = j.colliders.iter().position(|c| c == collider).unwrap();
                let body = &bodies[j.bodies[body_index]];
                let mut point_records = Vec::new();
                for (manifold_index, m) in pair.solver_manifolds().iter().enumerate() {
                    if !m.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES) {
                        continue;
                    }
                    let dir = -m.data.normal;
                    let t1 = dir.orthonormal_vector();
                    let t2 = dir.cross(t1);
                    for contact in &m.data.solver_contacts {
                        let point = &m.points[contact.contact_indices()[0] as usize];
                        let data = &point.data;
                        let impulse = dir * data.impulse
                            + t1 * data.tangent_impulse.x
                            + t2 * data.tangent_impulse.y;
                        let arm = if first {
                            data.solver_dp1
                        } else {
                            data.solver_dp2
                        };
                        let position = std::array::from_fn(|k| {
                            f64::from(body.center_of_mass().to_array()[k])
                                + f64::from(arm.to_array()[k])
                        });
                        let world_point = body.center_of_mass() + arm;
                        let local_point = body.position().inverse_transform_point(world_point);
                        let (v, w) = j.before_velocities[body_index];
                        let before_velocity = v + w.cross(arm);
                        let after_velocity = body.linvel() + body.angvel().cross(arm);
                        let normal_on_body = dir * if first { 1.0 } else { -1.0 };
                        let tangent_on_body = (t1 * data.tangent_impulse.x
                            + t2 * data.tangent_impulse.y)
                            * if first { 1.0 } else { -1.0 };
                        let speed = |velocity: Vector| {
                            (velocity - normal_on_body * velocity.dot(normal_on_body)).length()
                        };
                        point_records.push(FloorPoint {
                            manifold_index,
                            selected_contact_index: contact.contact_indices()[0],
                            friction: m.data.friction,
                            normal_impulse_ns: data.impulse,
                            tangent_impulse_world_ns: tangent_on_body.to_array(),
                            frozen_world_point: world_point.to_array(),
                            body_local_point: local_point.to_array(),
                            point_velocity_before_solve_mps: before_velocity.to_array(),
                            point_velocity_after_solve_mps: after_velocity.to_array(),
                            tangent_speed_before_solve_mps: speed(before_velocity),
                            tangent_speed_after_solve_mps: speed(after_velocity),
                        });
                        add(
                            &mut floor_angular,
                            cross(
                                position,
                                impulse
                                    .to_array()
                                    .map(|v| f64::from(v) * if first { 1.0 } else { -1.0 }),
                            ),
                        );
                    }
                }
                let normal_sign = if pair.collider2 == *collider {
                    1.0
                } else {
                    -1.0
                };
                let tangent_sign = -normal_sign;
                let mut tangent = [0.0; 3];
                for m in pair.solver_manifolds() {
                    if m.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES)
                        && !m.data.solver_contacts.is_empty()
                    {
                        if !m.data.reported_tangent_impulse.is_finite() {
                            return Err("Nonfinite tangent report".into());
                        }
                        add_scaled(&mut tangent, m.data.reported_tangent_impulse, tangent_sign);
                    }
                }
                Ok(FloorImpulse {
                    collider: *collider,
                    normal_ns: normal.to_array().map(|v| f64::from(v) * normal_sign),
                    tangent_ns: tangent,
                    points: point_records,
                })
            })();
            match result {
                Ok(floor) => {
                    for (k, value) in total.iter_mut().enumerate() {
                        *value += floor.normal_ns[k] + floor.tangent_ns[k];
                    }
                    floors.push(floor);
                }
                Err(e) => error = Some(e),
            }
        }
        let after = momentum(&j.bodies, bodies);
        let residual = std::array::from_fn(|k| after[k] - j.before[k] - j.external[k] - total[k]);
        // At this boundary the pose/inertia cache still belongs to the start
        // of the interval. Internal joint rows and numerical operations are
        // intentionally not imputed from this momentum difference.
        let angular_after = angular(&j.bodies, bodies);
        // Algebraic inverse-damping estimate only: native pre/post-damping
        // operands must be journaled before this can enter a certificate.
        let mut inverse_damping = [0.0; 3];
        for handle in &j.bodies {
            let b = &bodies[*handle];
            add(
                &mut inverse_damping,
                spin(b).map(|v| v * f64::from(dt) * f64::from(b.angular_damping())),
            );
        }
        let angular_residual = std::array::from_fn(|k| {
            angular_after[k] - j.angular_before[k] + inverse_damping[k]
                - j.external_angular[k]
                - floor_angular[k]
        });
        j.records.push(SolveRecord {
            native_substep: j.substep,
            ccd_interval: j.interval,
            dt_s: f64::from(dt),
            momentum_before_ns: j.before,
            momentum_after_ns: after,
            gravity_and_user_impulse_ns: j.external,
            floor_impulse_ns: total,
            linear_residual_ns: residual,
            floors,
            observation_error: error,
            angular_balance_verified: false,
            angular_before_fixed_ns: j.angular_before,
            angular_after_fixed_ns: angular_after,
            external_angular_ns: j.external_angular,
            floor_angular_ns: floor_angular,
            inverse_damping_estimate_ns: inverse_damping,
            unresolved_angular_ns: angular_residual,
            angular_after_positions_ns: None,
            geometry_transport_ns: None,
        });
        j.interval += 1;
    });
}
pub(crate) fn take_records() -> Vec<SolveRecord> {
    ACTIVE.with(|slot| {
        slot.borrow_mut()
            .as_mut()
            .map_or_else(Vec::new, |j| std::mem::take(&mut j.records))
    })
}
pub(crate) fn finish() {
    ACTIVE.with(|slot| {
        *slot.borrow_mut() = None;
    });
}

fn add(out: &mut [f64; 3], value: [f64; 3]) {
    for k in 0..3 {
        out[k] += value[k];
    }
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn spin(b: &RigidBody) -> [f64; 3] {
    let i = b
        .mass_properties()
        .effective_world_inv_inertia
        .inverse_unchecked();
    let w = b.angvel().to_array().map(f64::from);
    [
        f64::from(i.m11) * w[0] + f64::from(i.m12) * w[1] + f64::from(i.m13) * w[2],
        f64::from(i.m12) * w[0] + f64::from(i.m22) * w[1] + f64::from(i.m23) * w[2],
        f64::from(i.m13) * w[0] + f64::from(i.m23) * w[1] + f64::from(i.m33) * w[2],
    ]
}
fn angular(handles: &[RigidBodyHandle], bodies: &RigidBodySet) -> [f64; 3] {
    let mut result = [0.0; 3];
    for handle in handles {
        let b = &bodies[*handle];
        add(&mut result, spin(b));
        add(
            &mut result,
            cross(
                b.center_of_mass().to_array().map(f64::from),
                b.linvel()
                    .to_array()
                    .map(|v| f64::from(v) * f64::from(b.mass())),
            ),
        );
    }
    result
}
pub(crate) fn after_positions(bodies: &RigidBodySet) {
    ACTIVE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(j) = slot.as_mut() else { return };
        let after = angular(&j.bodies, bodies);
        let record = j.records.last_mut().unwrap();
        record.angular_after_positions_ns = Some(after);
        record.geometry_transport_ns = Some(std::array::from_fn(|k| {
            after[k] - record.angular_after_fixed_ns[k]
        }));
    });
}
