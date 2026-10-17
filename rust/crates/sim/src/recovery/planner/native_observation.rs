//! Read-only adaptation of the completed native substep into support evidence.
//! No integration, FK sweep, controller mutation, or force installation occurs.
use super::contact_observation::SolveRecord;
use super::native_contact_diagnostics::{
    ContactAdmissionPolicy, ContactBoundary, ContactExclusion, ManifoldBoundary,
    ManifoldPointBoundary, PairBoundary, PoseEvidence, SelectedContactBoundary,
    SelectedContactSample, classify_selected_contact,
};
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
    let mut contact_pairs = Vec::with_capacity(BODY_COUNT);
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
        // Diagnostic extraction only: it neither changes the selected-contact
        // path above nor treats cached/unselected impulses as current support.
        contact_pairs.push(contact_pair_boundary(s, i, body));
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
        native_contact_boundary: Some(ContactBoundary {
            schema_version: 1,
            native_substep: s.substeps_completed,
            enclosing_native_dt_s: f64::from(s.profile.dt_s),
            floor_collider: s.floor.into_raw_parts().into(),
            pairs: contact_pairs,
        }),
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
    let classification = classify_selected_contact(
        SelectedContactSample {
            local_point_m,
            world_point_m,
            upward_normal,
            impulse_ns,
            live_separation_m,
            enclosing_native_dt_s,
        },
        contact_admission_policy(),
    )?;
    Ok(classification
        .current_upward_load_n
        .map(|current_upward_load_n| CurrentContact {
            local_point_m,
            world_point_m,
            upward_normal,
            current_upward_load_n,
        }))
}

fn contact_admission_policy() -> ContactAdmissionPolicy {
    ContactAdmissionPolicy {
        minimum_upward_normal: MIN_NORMAL_Y,
        maximum_live_separation_m: f64::from(
            recovery_contract::contract()
                .recovery_acceptance
                .contact_distance_m,
        ),
        required_native_dt_s: f64::from(lh_contracts::SUBSTEP_S),
    }
}

fn pose_evidence(pose: &rapier3d::math::Pose) -> PoseEvidence {
    PoseEvidence {
        translation_m: pose.translation.to_array().map(f64::from),
        rotation_xyzw: pose.rotation.to_array().map(f64::from),
    }
}

/// All reads use the existing world at its completed native substep. No narrow
/// phase update, collision query, integration, FK, or solver mutation is invoked.
fn contact_pair_boundary(s: &Simulation, segment: usize, body: &RigidBody) -> PairBoundary {
    let pair = s
        .world
        .narrow_phase
        .contact_pair(s.floor, s.colliders[segment]);
    let mut result = PairBoundary {
        segment,
        segment_collider: s.colliders[segment].into_raw_parts().into(),
        pair_present: pair.is_some(),
        pair_colliders: None,
        collider_poses: [None, None],
        segment_is_collider1: None,
        solver_uses_clusters: false,
        raw_manifolds: Vec::new(),
        solver_clusters: Vec::new(),
    };
    if let Some(pair) = pair {
        let first = pair.collider1 == s.colliders[segment];
        let clustered = !pair.solver_clusters.is_empty();
        result.pair_colliders = Some([
            pair.collider1.into_raw_parts().into(),
            pair.collider2.into_raw_parts().into(),
        ]);
        result.segment_is_collider1 = Some(first);
        result.solver_uses_clusters = clustered;
        let collider_poses = [pair.collider1, pair.collider2].map(|handle| {
            s.world
                .colliders
                .get(handle)
                .map(|collider| collider.position())
        });
        result.collider_poses = collider_poses.map(|pose| pose.map(pose_evidence));
        result.raw_manifolds = pair
            .manifolds
            .iter()
            .enumerate()
            .map(|(index, manifold)| {
                manifold_boundary(s, body, first, manifold, index, !clustered, collider_poses)
            })
            .collect();
        result.solver_clusters = pair
            .solver_clusters
            .iter()
            .enumerate()
            .map(|(index, manifold)| {
                manifold_boundary(s, body, first, manifold, index, true, collider_poses)
            })
            .collect();
    }
    result
}

fn manifold_boundary(
    s: &Simulation,
    body: &RigidBody,
    segment_first: bool,
    manifold: &rapier3d::geometry::ContactManifold,
    manifold_index: usize,
    is_current_solver_collection: bool,
    collider_poses: [Option<&rapier3d::math::Pose>; 2],
) -> ManifoldBoundary {
    let computes_impulses = manifold
        .data
        .solver_flags
        .contains(SolverFlags::COMPUTE_IMPULSES);
    let normal = manifold.data.normal;
    let upward = f64::from(normal.y) * if segment_first { -1.0 } else { 1.0 };
    let collection_reasons = || {
        let mut reasons = Vec::new();
        if !is_current_solver_collection {
            reasons.push(ContactExclusion::InactiveSolverCollection);
        }
        if !computes_impulses {
            reasons.push(ContactExclusion::SolverImpulsesDisabled);
        }
        reasons
    };
    let points = manifold
        .points
        .iter()
        .enumerate()
        .map(|(point_index, point)| {
            let current_world_point =
                |side: usize, local_point: Vector, subshape_pose: Option<&rapier3d::math::Pose>| {
                    collider_poses[side].map(|pose| {
                        pose.transform_point(subshape_pose.map_or(local_point, |subshape| {
                            subshape.transform_point(local_point)
                        }))
                    })
                };
            let current_p1 = current_world_point(0, point.local_p1, manifold.subshape_pos1());
            let current_p2 = current_world_point(1, point.local_p2, manifold.subshape_pos2());
            let selected_contact_indices: Vec<_> = manifold
                .data
                .solver_contacts
                .iter()
                .enumerate()
                .filter_map(|(index, selected)| {
                    (selected.contact_indices()[0] as usize == point_index).then_some(index)
                })
                .collect();
            let mut filtered_reasons = collection_reasons();
            if selected_contact_indices.is_empty() {
                filtered_reasons.push(ContactExclusion::UnselectedManifoldPoint);
            }
            ManifoldPointBoundary {
                point_index,
                local_p1_m: point.local_p1.to_array().map(f64::from),
                local_p2_m: point.local_p2.to_array().map(f64::from),
                current_world_p1_m: current_p1.map(|point| point.to_array().map(f64::from)),
                current_world_p2_m: current_p2.map(|point| point.to_array().map(f64::from)),
                live_geometric_separation_m: current_p1
                    .zip(current_p2)
                    .map(|(a, b)| f64::from((b - a).dot(normal))),
                stored_distance_m: f64::from(point.dist),
                feature_id1_raw: point.fid1.0,
                feature_id2_raw: point.fid2.0,
                normal_impulse_ns: f64::from(point.data.impulse),
                tangent_impulse_ns: [
                    f64::from(point.data.tangent_impulse[0]),
                    f64::from(point.data.tangent_impulse[1]),
                ],
                warmstart_normal_impulse_ns: f64::from(point.data.warmstart_impulse),
                warmstart_tangent_impulse_ns: [
                    f64::from(point.data.warmstart_tangent_impulse[0]),
                    f64::from(point.data.warmstart_tangent_impulse[1]),
                ],
                warmstart_twist_impulse_ns: f64::from(point.data.warmstart_twist_impulse),
                warmstart_tangent_world_ns: point
                    .data
                    .warmstart_tangent_world
                    .to_array()
                    .map(f64::from),
                solver_dp1_m: point.data.solver_dp1.to_array().map(f64::from),
                solver_dp2_m: point.data.solver_dp2.to_array().map(f64::from),
                selected_contact_indices,
                filtered_reasons,
            }
        })
        .collect();
    let selected_contacts = manifold
        .data
        .solver_contacts
        .iter()
        .enumerate()
        .map(|(selected_contact_index, selected)| {
            let manifold_point_index = selected.contact_indices()[0];
            let point = manifold.points.get(manifold_point_index as usize);
            let (a, b) = manifold
                .data
                .solver_contact_world_points(selected, &s.world.bodies);
            let world_point = if segment_first { a } else { b };
            let local_point = body.position().inverse_transform_point(world_point);
            let live_separation_m = f64::from((b - a).dot(normal));
            let classification = point
                .ok_or_else(|| "Missing selected native contact".to_string())
                .and_then(|point| {
                    classify_selected_contact(
                        SelectedContactSample {
                            local_point_m: local_point.to_array().map(f64::from),
                            world_point_m: world_point.to_array().map(f64::from),
                            upward_normal: upward,
                            impulse_ns: f64::from(point.data.impulse),
                            live_separation_m,
                            enclosing_native_dt_s: f64::from(s.profile.dt_s),
                        },
                        contact_admission_policy(),
                    )
                });
            let mut filtered_reasons = collection_reasons();
            let (current_upward_load_n, measurement_error) = match classification {
                Ok(classification) => {
                    filtered_reasons.extend(classification.filtered_reasons);
                    (
                        classification
                            .current_upward_load_n
                            .filter(|_| filtered_reasons.is_empty()),
                        None,
                    )
                }
                Err(error) => (None, Some(error)),
            };
            SelectedContactBoundary {
                selected_contact_index,
                manifold_point_index,
                contact_id_raw: selected.contact_id[0],
                anchor1_raw: selected.anchor1.to_array().map(f64::from),
                anchor2_raw: selected.anchor2.to_array().map(f64::from),
                stored_distance_m: f64::from(selected.dist),
                tangent_velocity_mps: selected.tangent_velocity.to_array().map(f64::from),
                world_anchor1_m: a.to_array().map(f64::from),
                world_anchor2_m: b.to_array().map(f64::from),
                segment_local_point_m: local_point.to_array().map(f64::from),
                segment_world_point_m: world_point.to_array().map(f64::from),
                live_separation_m,
                normal_impulse_ns: point.map(|p| f64::from(p.data.impulse)),
                upward_normal_on_segment: upward,
                admitted_as_current_contact: current_upward_load_n.is_some(),
                current_upward_load_n,
                filtered_reasons,
                measurement_error,
            }
        })
        .collect();
    ManifoldBoundary {
        manifold_index,
        is_current_solver_collection,
        subshape1: manifold.subshape1,
        subshape2: manifold.subshape2,
        subshape_pose1: manifold.subshape_pos1().map(pose_evidence),
        subshape_pose2: manifold.subshape_pos2().map(pose_evidence),
        local_normal1: manifold.local_n1.to_array().map(f64::from),
        local_normal2: manifold.local_n2.to_array().map(f64::from),
        world_normal: normal.to_array().map(f64::from),
        upward_normal_on_segment: upward,
        solver_flags_bits: manifold.data.solver_flags.bits(),
        computes_impulses,
        rigid_body1: manifold
            .data
            .rigid_body1
            .map(|handle| handle.into_raw_parts().into()),
        rigid_body2: manifold
            .data
            .rigid_body2
            .map(|handle| handle.into_raw_parts().into()),
        relative_dominance: manifold.data.relative_dominance,
        friction: f64::from(manifold.data.friction),
        restitution: f64::from(manifold.data.restitution),
        reported_tangent_impulse_ns: manifold
            .data
            .reported_tangent_impulse
            .to_array()
            .map(f64::from),
        points,
        selected_contacts,
    }
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
