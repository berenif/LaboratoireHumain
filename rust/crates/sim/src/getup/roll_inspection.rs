//! Immutable all-body contact geometry and detached current matrix inspection.
//! No contact is synthesized, no force/pose is applied, and no world is stepped.
use crate::*;
use rapier3d::utils::OrthonormalBasis;

pub fn diagnostic(s: &Simulation) -> Result<serde_json::Value, String> {
    let before = bincode::serialize(&s.world).map_err(|e| e.to_string())?;
    let physical = super::physical_observation(s)?;
    let twists = super::current_twists(s)?;
    let mut floors = Vec::new();
    for (i, part) in s.model.segments.iter().enumerate() {
        let mut contacts = Vec::new();
        if let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) {
            for (manifold_index, manifold) in pair.solver_manifolds().iter().enumerate() {
                let body_is_second = pair.collider2 == s.colliders[i];
                let normal = manifold.data.normal * if body_is_second { 1.0 } else { -1.0 };
                for (solver_index, contact) in manifold.data.solver_contacts.iter().enumerate() {
                    let index = contact.contact_indices()[0] as usize;
                    let data = &manifold.points.get(index).ok_or("Missing native solver point")?.data;
                    let (a, b) = manifold.data.solver_contact_world_points(contact, &s.world.bodies);
                    let (point, floor_point) = if body_is_second { (b, a) } else { (a, b) };
                    let rb = &s.world.bodies[s.bodies[i]];
                    let com = rb.translation() + rb.rotation().normalize() * rb.local_center_of_mass();
                    let lever = if body_is_second { data.solver_dp2 } else { data.solver_dp1 };
                    let velocity = twists[i].0 + twists[i].1.cross(point - com);
                    let solver_velocity = twists[i].0 + twists[i].1.cross(lever);
                    let force_dir = -manifold.data.normal;
                    let tangent1 = force_dir.orthonormal_vector();
                    let tangent2 = force_dir.cross(tangent1);
                    let tangent_sign = if pair.collider1 == s.colliders[i] { 1.0 } else { -1.0 };
                    let tangent_impulse = tangent_sign * (tangent1 * data.tangent_impulse.x + tangent2 * data.tangent_impulse.y);
                    contacts.push(serde_json::json!({"manifoldIndex":manifold_index,"solverIndex":solver_index,
                        "sourcePointIndex":index,"point":vec3(point),"floorPoint":vec3(floor_point),
                        "normal":vec3(normal),"signedGapM":(point-floor_point).dot(normal),
                        "cachedSolverDistanceM":contact.dist,"normalImpulseNs":data.impulse,
                        "tangentImpulseWorldNs":vec3(tangent_impulse),"currentPointVelocityMps":vec3(velocity),
                        "solverLeverFromCom":vec3(lever),"currentSolverPointVelocityMps":vec3(solver_velocity),
                        "solverComputesImpulses":manifold.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES)}));
                }
            }
        }
        floors.push(serde_json::json!({"bodyIndex":i,"body":part.id,"massKg":s.world.bodies[s.bodies[i]].mass(),
            "enabledCollider":s.world.colliders[s.colliders[i]].is_enabled(),
            "integratedLoadN":s.integrated_support_loads_n[i],"supportNativeSamples":s.integrated_support_substeps[i],
            "floorFriction":s.world.colliders[s.colliders[i]].friction(),"contacts":contacts}));
    }
    let id = *s.world.multibody_joints.rigid_body_link(s.bodies[0]).ok_or("Missing root")?;
    let actual = s.world.multibody_joints.get_multibody(id.multibody).ok_or("Missing articulation")?;
    let mut detached = actual.clone();
    // This refresh is intentionally applied ONLY to a detached clone. At dt0
    // Rapier explicitly excludes dt*damping and dt²*spring from this matrix.
    let inverse_diagonal = detached.dof_inverse_inertia(&s.world.bodies);
    let inverse = detached.inv_augmented_mass().try_inverse().ok_or("Singular detached mass matrix")?;
    if detached.ndofs() != 27 || inverse.nrows() != 27 || inverse.ncols() != 27 {
        return Err("Inspection requires all unchanged27 generalized coordinates with no removed DOFs".into());
    }
    let inverse_rows: Vec<Vec<f32>> = (0..27).map(|r| (0..27).map(|c| inverse[(r,c)]).collect()).collect();
    let mut bodies = Vec::new();
    for (i, part) in s.model.segments.iter().enumerate() {
        let link = s.world.multibody_joints.rigid_body_link(s.bodies[i]).ok_or("Missing body link")?;
        let j = detached.body_jacobian(link.id);
        let rb = &s.world.bodies[s.bodies[i]];
        let j_rows: Vec<Vec<f32>> = (0..6).map(|r| (0..27).map(|c| j[(r,c)]).collect()).collect();
        bodies.push(serde_json::json!({"bodyIndex":i,"body":part.id,"massKg":rb.mass(),
            "origin":vec3(rb.translation()),"rotation":{ "x":rb.rotation().x,"y":rb.rotation().y,"z":rb.rotation().z,"w":rb.rotation().w },
            "localCom":vec3(rb.local_center_of_mass()),"currentComVelocity":vec3(twists[i].0),
            "currentAngularVelocity":vec3(twists[i].1),"detachedCurrentComJacobian":j_rows,
            "sourcePrimitiveCount":part.reference.as_ref().map(|r|r.primitives.len()),"collisionExclusions":part.collision_exclusions}));
    }
    let mut collision_queries = Vec::new();
    for i in 0..s.colliders.len() {
        if !s.world.colliders[s.colliders[i]].is_enabled() { continue; }
        for j in i+1..s.colliders.len() {
            if !s.world.colliders[s.colliders[j]].is_enabled() { continue; }
            let a = handle_key(s.colliders[i]); let b = handle_key(s.colliders[j]);
            if s.filter.exclusions.contains(&(a.min(b),a.max(b))) { continue; }
            let ca = &s.world.colliders[s.colliders[i]]; let cb = &s.world.colliders[s.colliders[j]];
            let mut selected_rows = Vec::new();
            if let Some(pair) = s.world.narrow_phase.contact_pair(s.colliders[i], s.colliders[j]) {
                for manifold in pair.solver_manifolds() {
                    let forward = pair.collider1 == s.colliders[i];
                    let normal = manifold.data.normal * if forward { 1.0 } else { -1.0 };
                    for contact in &manifold.data.solver_contacts {
                        let data = &manifold.points[contact.contact_indices()[0] as usize].data;
                        let (a,b) = manifold.data.solver_contact_world_points(contact, &s.world.bodies);
                        let (a,b) = if forward { (a,b) } else { (b,a) };
                        selected_rows.push(serde_json::json!({"pointA":vec3(a),"pointB":vec3(b),
                            "normalAtoB":vec3(normal),"signedGapM":(b-a).dot(normal),
                            "normalImpulseNs":data.impulse,"cachedSolverDistanceM":contact.dist,
                            "solverComputesImpulses":manifold.data.solver_flags.contains(SolverFlags::COMPUTE_IMPULSES)}));
                    }
                }
            }
            let query = rapier3d::parry::query::contact(ca.position(), ca.shape(), cb.position(), cb.shape(), 3.0)
                .map_err(|e| format!("Collision query: {e:?}"))?;
            collision_queries.push(serde_json::json!({"a":s.model.segments[i].id,"b":s.model.segments[j].id,
                "friction":0.0,"geometricQueryNotSolverContact":true,
                "actualSelectedSolverContacts":selected_rows,
                "query":query.map(|c|serde_json::json!({"pointA":vec3(c.point1),"pointB":vec3(c.point2),
                    "normalAtoB":vec3(c.normal1),"signedGapM":c.dist}))}));
        }
    }
    let after = bincode::serialize(&s.world).map_err(|e|e.to_string())?;
    if before != after { return Err("Immutable contact inspection changed the real physical world".into()); }
    Ok(serde_json::json!({"schema":"getup-roll-native-checkpoint-inspection-v1","physicsIntegrations":0,
        "nativeSubsteps":s.substeps_completed,"physicalSeconds":s.substeps_completed as f64/800.0,
        "realWorldBytesUnchanged":true,"modelMassKg":s.total_mass(),"physicalObservationWords":physical.map(f32::to_bits).to_vec(),
        "profile":s.profile,"floorContacts":floors,"bodyData":bodies,"eligibleSelfCollisionGeometry":collision_queries,
        "matrix":{"method":"detached clone dof_inverse_inertia refresh atdt0; no real cache/state write",
            "includesHingeArmature":true,"includesDtDampingOrSpring":false,"ndofs":detached.ndofs(),
            "inverseMassRows":inverse_rows,"inverseMassDiagonal":inverse_diagonal.as_slice(),
            "damping":actual.damping().as_slice(),"armature":actual.armature().as_slice(),
            "generalizedVelocity":actual.generalized_velocity().as_slice()},
        "semantics":{"floorGeometry":"actual selected solver anchors, current body-local anchor reconstruction, current generalized point motion; zero impulse rows retained",
            "floorLoad":"native integrated support observation; point impulses cover the latest selected solver interval",
            "rolling":"anchor locations are observations at this instant, not pinned future material points",
            "selfCollisionQueries":"exact current shape query geometry for eligible pairs; not selected solver contacts",
            "limitations":"No isolated native bias export or future contact sequence/dynamic feasibility is claimed"}}))
}
