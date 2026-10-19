//! Diagnostic only. The hooks read solver reporting before CCD contact refresh.
//! No solver parameters, bodies, contacts, or impulses are modified.
use crate::Profile;
use rapier3d::prelude::*;
use serde::Serialize;
use std::sync::Mutex;

#[derive(Clone, Serialize)]
struct Interval {
    dt_s: f32,
    before_velocity: [f32; 3],
    after_velocity: [f32; 3],
    normal_impulse_on_body_ns: [f32; 3],
    tangent_impulse_on_body_ns: [f32; 3],
    manifold_normals: Vec<[f32; 3]>,
    selected_contacts: usize,
}
struct Trace {
    body: RigidBodyHandle,
    collider: ColliderHandle,
    floor: ColliderHandle,
    before: Mutex<Vector>,
    intervals: Mutex<Vec<Interval>>,
}
fn v(x: Vector) -> [f32; 3] {
    [x.x, x.y, x.z]
}
fn impulse(
    narrow: &NarrowPhase,
    floor: ColliderHandle,
    collider: ColliderHandle,
) -> (Vector, Vector, Vec<[f32; 3]>, usize) {
    let Some(pair) = narrow.contact_pair(floor, collider) else {
        return (Vector::ZERO, Vector::ZERO, vec![], 0);
    };
    let normal_sign = if pair.collider2 == collider {
        1.0
    } else {
        -1.0
    };
    let tangent_sign = -normal_sign;
    let normal =
        crate::measurement::solver_normal_impulse(pair).expect("valid solver normal") * normal_sign;
    let mut tangent = Vector::ZERO;
    let mut normals = vec![];
    let mut selected = 0;
    for m in pair.solver_manifolds() {
        if m.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES)
            && !m.data.solver_contacts.is_empty()
        {
            tangent += tangent_sign * m.data.reported_tangent_impulse;
            normals.push(v(m.data.normal));
            selected += m.data.solver_contacts.len();
        }
    }
    (normal, tangent, normals, selected)
}
impl PhysicsHooks for Trace {
    fn before_solver_step(&self, _dt: Real, bodies: &RigidBodySet) {
        *self.before.lock().unwrap() = bodies[self.body].linvel();
    }
    fn after_solver_step(&self, dt: Real, bodies: &RigidBodySet, narrow: &NarrowPhase) {
        let (normal, tangent, normals, selected) = impulse(narrow, self.floor, self.collider);
        self.intervals.lock().unwrap().push(Interval {
            dt_s: dt,
            before_velocity: v(*self.before.lock().unwrap()),
            after_velocity: v(bodies[self.body].linvel()),
            normal_impulse_on_body_ns: v(normal),
            tangent_impulse_on_body_ns: v(tangent),
            manifold_normals: normals,
            selected_contacts: selected,
        });
    }
}

/// Replays the canonical fixture unchanged, including the complete 960-step history.
/// A second uninstrumented replay checks exact body-state parity after every step.
pub fn replay(
    profile: &Profile,
    foot: &str,
    warmstart: f32,
    scenario: &str,
) -> Result<serde_json::Value, String> {
    if !matches!(scenario, "rest" | "changing" | "unloading" | "contact-loss") {
        return Err("Unknown load fixture".into());
    }
    let model = lh_model::Model::canonical()?;
    let s = model
        .segments
        .iter()
        .find(|s| s.id == foot)
        .ok_or("Unknown foot")?;
    let heading = -std::f32::consts::PI / 4.0;
    let (mut world, body, floor, collider) = super::foot_fixture(s, heading, true, profile)?;
    let (mut control, cb, cf, _) = super::foot_fixture(s, heading, true, profile)?;
    world.integration_parameters.warmstart_coefficient = warmstart;
    control.integration_parameters.warmstart_coefficient = warmstart;
    let trace = Trace {
        body,
        collider,
        floor,
        before: Mutex::new(Vector::ZERO),
        intervals: Mutex::new(vec![]),
    };
    let mass = world.bodies[body].mass();
    let ballast = (72.2 / 2.0 - s.mass_kg) * 9.81;
    let mut rows = vec![];
    let mut max_final_normal_error = 0.0_f32;
    let mut max_interval_vector_error = 0.0_f32;
    let mut max_summed_vector_error = 0.0_f32;
    let mut exact_state_parity = true;
    let mut ccd_intervals = 0;
    for step in 0..960 {
        let force = if step < 480 {
            -ballast
        } else {
            match scenario {
                "changing" => {
                    if step < 720 {
                        -ballast - 30.0
                    } else {
                        -ballast + 20.0
                    }
                }
                "unloading" => s.mass_kg * 9.81 + 20.0,
                _ => -ballast,
            }
        };
        if scenario == "contact-loss" && step == 480 {
            world.colliders[floor].set_enabled(false);
            control.colliders[cf].set_enabled(false);
        }
        let before = world.bodies[body].linvel();
        for (w, b) in [(&mut world, body), (&mut control, cb)] {
            w.bodies[b].reset_forces(false);
            w.bodies[b].add_force(Vector::Y * force, true);
        }
        trace.intervals.lock().unwrap().clear();
        world.step_with_events(&trace, &());
        control.step();
        let a = &world.bodies[body];
        let b = &control.bodies[cb];
        let state_a = [
            a.translation().to_array().to_vec(),
            a.rotation().to_array().to_vec(),
            a.linvel().to_array().to_vec(),
            a.angvel().to_array().to_vec(),
        ]
        .concat();
        let state_b = [
            b.translation().to_array().to_vec(),
            b.rotation().to_array().to_vec(),
            b.linvel().to_array().to_vec(),
            b.angvel().to_array().to_vec(),
        ]
        .concat();
        exact_state_parity &= state_a
            .iter()
            .zip(&state_b)
            .all(|(a, b)| a.to_bits() == b.to_bits());
        let intervals = trace.intervals.lock().unwrap().clone();
        ccd_intervals += intervals.len();
        let expected =
            mass * (a.linvel() - before) - Vector::Y * (-mass * 9.81 + force) * profile.dt_s;
        let mut sum = Vector::ZERO;
        let mut interval_errors = vec![];
        for i in &intervals {
            let measured = Vector::from_array(i.normal_impulse_on_body_ns)
                + Vector::from_array(i.tangent_impulse_on_body_ns);
            let predicted = mass
                * (Vector::from_array(i.after_velocity) - Vector::from_array(i.before_velocity))
                - Vector::Y * (-mass * 9.81 + force) * i.dt_s;
            let residual = measured - predicted;
            max_interval_vector_error = max_interval_vector_error.max(residual.length());
            interval_errors.push(v(residual));
            sum += measured;
        }
        let final_normal = crate::measurement::floor_normal_impulse(&world, floor, collider)?;
        let final_normal_error = (final_normal - expected.y).abs();
        let summed_error = (sum - expected).length();
        max_final_normal_error = max_final_normal_error.max(final_normal_error);
        max_summed_vector_error = max_summed_vector_error.max(summed_error);
        if final_normal_error > 0.002 || summed_error > 0.002 || step == 398 || intervals.len() > 1
        {
            let (refresh_normal, refresh_tangent, normals, selected) =
                impulse(&world.narrow_phase, floor, collider);
            rows.push(serde_json::json!({"outerStep":step,"outerDtS":profile.dt_s,"expectedContactImpulseNs":v(expected),"latestRefreshedNormalImpulseOnBodyNs":v(refresh_normal),"latestRefreshedTangentImpulseOnBodyNs":v(refresh_tangent),"latestRefreshedManifoldNormals":normals,"latestRefreshedSelectedContacts":selected,"legacyNormalOnlyErrorNs":final_normal_error,"summedIntervalImpulseNs":v(sum),"summedVectorErrorNs":summed_error,"intervalResidualsNs":interval_errors,"intervals":intervals}));
        }
    }
    Ok(
        serde_json::json!({"diagnosticOnly":true,"releaseAccepted":false,"profile":profile,"segment":foot,"heading":heading,"warmstart":warmstart,"scenario":scenario,"outerSteps":960,"observedSolverIntervals":ccd_intervals,"instrumentationExactBodyStateParityEveryStep":exact_state_parity,"thresholdNs":0.002,"maxLegacyNormalOnlyErrorNs":max_final_normal_error,"maxIntervalVectorErrorNs":max_interval_vector_error,"maxSummedIntervalVectorErrorNs":max_summed_vector_error,"discriminatingRows":rows}),
    )
}
