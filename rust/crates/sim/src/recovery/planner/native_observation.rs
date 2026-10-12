//! Read-only adaptation of the completed native substep into support evidence.
//! No integration, FK sweep, controller mutation, or force installation occurs.
use super::contact_observation::SolveRecord;
use super::support_state::{
    BODY_COUNT, BodyMotion, CcdImpulse, CurrentContact, MIN_NORMAL_Y, NATIVE_HZ,
    SegmentContactState, SegmentRole, SupportObservation,
};
use crate::{Simulation, measurement, recovery_contract};
use rapier3d::prelude::*;

/// The prototype deliberately admits the native static floor only. Its CCD
/// journal currently observes that floor, so accepting other support surfaces
/// here would create different boundaries for the two kinds of evidence.
pub(crate) fn observe(
    s: &Simulation,
    history_start_substep: u64,
    ccd_records: &[SolveRecord],
) -> Result<SupportObservation, String> {
    if s.model.segments.len() != BODY_COUNT
        || s.bodies.len() != BODY_COUNT
        || s.colliders.len() != BODY_COUNT
        || s.profile.dt_s != lh_contracts::SUBSTEP_S
        || s.profile.tick_substeps != 4
        || s.environment.is_some()
        || s.striker.is_some()
        || s.grab.is_some()
        || s.world.multibody_joints.iter().next().is_some()
    {
        return Err(
            "Native support adapter requires 25 bodies, 240 Hz, and the static floor scene".into(),
        );
    }
    let elapsed = s
        .substeps_completed
        .checked_sub(history_start_substep)
        .ok_or("Support observation history starts in the future")?;
    let floor = s
        .world
        .colliders
        .get(s.floor)
        .ok_or("Missing observed floor")?;
    if floor.parent().is_some() || !floor.is_enabled() {
        return Err("Observed support floor must be enabled and world attached".into());
    }
    let mass = s.total_mass();
    if !mass.is_finite() || mass <= 0.0 {
        return Err("Invalid measured total mass".into());
    }
    let mut bodies = Vec::with_capacity(BODY_COUNT);
    let mut segments = Vec::with_capacity(BODY_COUNT);
    let mut com_velocity = Vector::ZERO;
    for (i, part) in s.model.segments.iter().enumerate() {
        let body = s
            .world
            .bodies
            .get(s.bodies[i])
            .ok_or("Missing observed body")?;
        let collider = s
            .world
            .colliders
            .get(s.colliders[i])
            .ok_or("Missing observed collider")?;
        if !body.is_dynamic() || collider.parent() != Some(s.bodies[i]) {
            return Err("Observed anatomical collider/body ownership mismatch".into());
        }
        com_velocity += body.linvel() * body.mass();
        bodies.push(BodyMotion {
            segment: i,
            position_m: body.translation().to_array().map(f64::from),
            rotation_xyzw: body.rotation().to_array().map(f64::from),
            linear_velocity_mps: body.linvel().to_array().map(f64::from),
            angular_velocity_radps: body.angvel().to_array().map(f64::from),
        });
        let mut contacts = Vec::new();
        if let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) {
            // Validate the selected-point indexing and impulse data by the same
            // path as the original observer; never ContactPair::total_impulse.
            measurement::solver_normal_impulse(pair)?;
            let first = pair.collider1 == s.colliders[i];
            for manifold in pair.solver_manifolds() {
                if !manifold
                    .data
                    .solver_flags
                    .contains(SolverFlags::COMPUTE_IMPULSES)
                {
                    continue;
                }
                let normal = manifold.data.normal;
                let upward = f64::from(normal.y) * if first { -1.0 } else { 1.0 };
                for selected in manifold.data.solver_contacts.iter() {
                    let point = manifold
                        .points
                        .get(selected.contact_indices()[0] as usize)
                        .ok_or("Missing selected native contact")?;
                    let (a, b) = manifold
                        .data
                        .solver_contact_world_points(selected, &s.world.bodies);
                    let world_point = if first { a } else { b };
                    let local_point = body.position().inverse_transform_point(world_point);
                    // Resolve anchors through CURRENT poses. A selected point
                    // that has already separated cannot inherit its CCD impulse
                    // as evidence of current support. Use the original 12 mm
                    // contact distance, without changing the original observer.
                    let live_separation = f64::from((b - a).dot(normal));
                    if let Some(contact) = selected_contact(
                        local_point.to_array().map(f64::from),
                        world_point.to_array().map(f64::from),
                        upward,
                        f64::from(point.data.impulse),
                        live_separation,
                        f64::from(s.profile.dt_s),
                    )? {
                        contacts.push(contact);
                    }
                }
            }
        }
        segments.push(SegmentContactState {
            segment: i,
            id: part.id.clone(),
            role: role(&part.role),
            persistent_load_n: f64::from(s.integrated_support_loads_n[i]),
            loaded_substeps: s.integrated_support_substeps[i],
            contacts,
        });
    }
    let ccd_impulses = ccd_impulses(s.substeps_completed, &s.colliders, ccd_records)?;
    let observation = SupportObservation {
        native_substep: s.substeps_completed,
        time_s: elapsed as f64 / NATIVE_HZ as f64,
        weight_n: f64::from(mass * 9.81),
        center_of_mass_m: s.center_of_mass().to_array().map(f64::from),
        center_of_mass_velocity_mps: (com_velocity / mass).to_array().map(f64::from),
        support_height_m: f64::from(s.support_height()),
        bodies,
        segments,
        ccd_impulses,
    };
    observation.validate()?;
    Ok(observation)
}

fn role(role: &str) -> SegmentRole {
    match role {
        "hindfoot" => SegmentRole::Hindfoot,
        "forefoot" => SegmentRole::Forefoot,
        "hand" => SegmentRole::Hand,
        "shin" => SegmentRole::Shin,
        _ => SegmentRole::Other,
    }
}

fn selected_contact(
    local_point_m: [f64; 3],
    world_point_m: [f64; 3],
    upward_normal: f64,
    impulse_ns: f64,
    live_separation_m: f64,
    enclosing_native_dt_s: f64,
) -> Result<Option<CurrentContact>, String> {
    if local_point_m
        .iter()
        .chain(&world_point_m)
        .any(|v| !v.is_finite())
        || !upward_normal.is_finite()
        || !(-1.0..=1.0).contains(&upward_normal)
        || !impulse_ns.is_finite()
        || impulse_ns < 0.0
        || !live_separation_m.is_finite()
        || enclosing_native_dt_s != f64::from(lh_contracts::SUBSTEP_S)
    {
        return Err("Invalid selected native contact measurement".into());
    }
    if upward_normal < MIN_NORMAL_Y
        || impulse_ns == 0.0
        || live_separation_m
            > f64::from(
                recovery_contract::contract()
                    .recovery_acceptance
                    .contact_distance_m,
            )
    {
        return Ok(None);
    }
    let current_upward_load_n = upward_normal * impulse_ns / enclosing_native_dt_s;
    if !current_upward_load_n.is_finite() {
        return Err("Nonfinite native selected contact load".into());
    }
    Ok(Some(CurrentContact {
        local_point_m,
        world_point_m,
        upward_normal,
        current_upward_load_n,
    }))
}

fn ccd_impulses(
    substep: u64,
    colliders: &[ColliderHandle],
    records: &[SolveRecord],
) -> Result<Vec<CcdImpulse>, String> {
    let mut result = Vec::new();
    for (interval, record) in records.iter().enumerate() {
        if record.native_substep != substep
            || record.ccd_interval != interval
            || !record.dt_s.is_finite()
            || record.dt_s <= 0.0
            || record.observation_error.is_some()
        {
            return Err("Incomplete, stale or invalid CCD record boundary".into());
        }
        for (index, floor) in record.floors.iter().enumerate() {
            if record.floors[..index]
                .iter()
                .any(|previous| previous.collider == floor.collider)
            {
                return Err("Duplicate collider in CCD interval".into());
            }
            let segment = colliders
                .iter()
                .position(|c| *c == floor.collider)
                .ok_or("CCD impulse belongs to an unobserved collider")?;
            result.push(CcdImpulse {
                native_substep: substep,
                ccd_interval: interval,
                segment,
                interval_s: record.dt_s,
                normal_ns: floor.normal_ns,
                tangent_ns: floor.tangent_ns,
            });
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selected_contact_needs_current_geometry_and_upward_load_without_integration() {
        let sample = |normal, impulse, separation| {
            selected_contact(
                [0.0, 0.0, -0.03],
                [0.0; 3],
                normal,
                impulse,
                separation,
                f64::from(lh_contracts::SUBSTEP_S),
            )
        };
        assert!(sample(1.0, 1.0, 0.02).unwrap().is_none());
        assert!(sample(0.64, 1.0, 0.0).unwrap().is_none());
        assert!(sample(1.0, 0.0, 0.0).unwrap().is_none());
        assert!(sample(1.0, f64::NAN, 0.0).is_err());
        let contact = sample(1.0, 1.0, 0.0).unwrap().unwrap();
        assert_eq!(contact.local_point_m[2], -0.03);
        assert_eq!(
            contact.current_upward_load_n,
            1.0 / f64::from(lh_contracts::SUBSTEP_S)
        );
    }

    #[test]
    fn selected_heel_does_not_wait_for_segment_persistence_without_integration() {
        // There is deliberately no segment persistence argument. The heel's
        // twelve-substep clock begins at the first observed posterior contact.
        assert!(
            selected_contact(
                [0.0, 0.0, -0.03],
                [0.0; 3],
                1.0,
                0.02,
                0.0,
                f64::from(lh_contracts::SUBSTEP_S)
            )
            .unwrap()
            .is_some()
        );
        assert!(ccd_impulses(0, &[], &[]).unwrap().is_empty());
    }
}
