//! Read-only journal at the CCD solve boundary. This is a planner prerequisite,
//! not a support certificate: angular delivery and a roundoff bound remain open.
use crate::measurement;
use rapier3d::prelude::*;
use serde::Serialize;
use std::cell::RefCell;

#[derive(Clone, Debug, Serialize)]
pub(crate) struct FloorImpulse {
    pub collider: ColliderHandle,
    pub normal_ns: [f64; 3],
    pub tangent_ns: [f64; 3],
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
}
thread_local! {static ACTIVE: RefCell<Option<Journal>> = const {RefCell::new(None)};}
pub(crate) fn activate(s: &crate::Simulation) {
    assert!(
        s.grab.is_none() && s.striker.is_none() && s.environment.is_none(),
        "This diagnostic excludes interaction and moving environment forces"
    );
    assert_eq!(s.bodies.len(), 25);
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
        j.external = [0.0; 3];
        for h in &j.bodies {
            let b = &bodies[*h];
            // This first worker has no moving environment, grab or striker.
            // Sleep/damping/locked translation require separate admission.
            if !b.is_sleeping() {
                add_scaled(
                    &mut j.external,
                    j.gravity,
                    f64::from(dt) * f64::from(b.mass()) * f64::from(b.gravity_scale()),
                );
                add_scaled(&mut j.external, b.user_force(), f64::from(dt));
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
        let mut error = None;
        for collider in &j.colliders {
            let Some(pair) = phase.contact_pair(j.floor, *collider) else {
                continue;
            };
            let result = (|| -> Result<FloorImpulse, String> {
                let normal = measurement::solver_normal_impulse(pair)?;
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
