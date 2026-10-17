//! Read-only solved-foot contact motion. Foot origins alone cannot distinguish
//! loaded sliding, rolling about a contact, and an unloaded repositioning.
use crate::*;

pub(crate) struct FootContact {
    pub body: usize,
    pub point: Vector,
    pub floor_point: Vector,
    pub signed_gap_m: f32,
    pub cached_distance_m: f32,
    pub solver_lever_from_com: Vector,
    pub solver_point_velocity: Vector,
    pub normal: Vector,
    pub impulse: f32,
    pub velocity: Vector,
}

fn point_velocity(linear: Vector, angular: Vector, offset_from_com: Vector) -> Vector {
    linear + angular.cross(offset_from_com)
}

pub(crate) fn contacts(s: &Simulation) -> Result<Vec<FootContact>, String> {
    let twists = super::current_twists(s)?;
    let mut out = Vec::new();
    for name in ["left_foot", "right_foot"] {
        let i = s
            .model
            .segments
            .iter()
            .position(|p| p.id == name)
            .ok_or("Missing foot")?;
        let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) else {
            continue;
        };
        for manifold in pair.solver_manifolds() {
            let normal = manifold.data.normal
                * if pair.collider2 == s.colliders[i] {
                    1.0
                } else {
                    -1.0
                };
            if normal.y < 0.65 {
                continue;
            }
            for contact in &manifold.data.solver_contacts {
                let point_data = &manifold.points[contact.contact_indices()[0] as usize].data;
                let impulse = point_data.impulse;
                if impulse <= 0.0 {
                    continue;
                }
                let (a, b) = manifold
                    .data
                    .solver_contact_world_points(contact, &s.world.bodies);
                let (point, floor_point) = if pair.collider2 == s.colliders[i] {
                    (b, a)
                } else {
                    (a, b)
                };
                let body = &s.world.bodies[s.bodies[i]];
                let com =
                    body.translation() + body.rotation().normalize() * body.local_center_of_mass();
                let solver_lever = if pair.collider2 == s.colliders[i] {
                    point_data.solver_dp2
                } else {
                    point_data.solver_dp1
                };
                out.push(FootContact {
                    body: i,
                    point,
                    floor_point,
                    signed_gap_m: (point - floor_point).dot(normal),
                    cached_distance_m: contact.dist,
                    solver_lever_from_com: solver_lever,
                    solver_point_velocity: point_velocity(twists[i].0, twists[i].1, solver_lever),
                    normal,
                    impulse,
                    velocity: point_velocity(twists[i].0, twists[i].1, point - com),
                });
            }
        }
    }
    Ok(out)
}

pub fn contact_diagnostic(s: &Simulation) -> Result<serde_json::Value, String> {
    let contacts = contacts(s)?;
    let twists = super::current_twists(s)?;
    let minimum = recovery_contract::contract()
        .recovery_acceptance
        .minimum_load_n;
    let mut feet = Vec::new();
    // Multibody scenes use one solver group at the island's maximum extra
    // iteration count. This reports the human/floor group's expected h,
    // separately from the unchanged 800 Hz simulation frame.
    let extra = s
        .bodies
        .iter()
        .map(|h| s.world.bodies[*h].additional_solver_iterations())
        .max()
        .unwrap_or(0);
    let solver_h = s.world.integration_parameters.dt
        / (s.world.integration_parameters.num_solver_iterations + extra) as f32;
    let softness = &s.world.integration_parameters.static_contact_softness;
    for name in ["left_foot", "right_foot"] {
        let i = s
            .model
            .segments
            .iter()
            .position(|p| p.id == name)
            .ok_or("Missing foot")?;
        let rows: Vec<_> = contacts
            .iter()
            .filter(|c| c.body == i)
            .map(|c| {
                let tangent = c.velocity - c.normal * c.velocity.dot(c.normal);
                serde_json::json!({"point":vec3(c.point), "normal":vec3(c.normal),
                "floorPoint":vec3(c.floor_point),"signedGapM":c.signed_gap_m,
                "cachedSolverDistanceM":c.cached_distance_m,"normalVelocityMps":c.velocity.dot(c.normal),
                "solverLeverFromCom":vec3(c.solver_lever_from_com),"solverPointVelocityMps":vec3(c.solver_point_velocity),
                "normalImpulseNs":c.impulse,"velocityMps":vec3(c.velocity),
                "tangentVelocityMps":vec3(tangent),"tangentSpeedMps":tangent.length()})
            })
            .collect();
        let impulse: f32 = contacts
            .iter()
            .filter(|c| c.body == i)
            .map(|c| c.impulse)
            .sum();
        let weighted_speed: f32 = contacts
            .iter()
            .filter(|c| c.body == i)
            .map(|c| (c.velocity - c.normal * c.velocity.dot(c.normal)).length() * c.impulse)
            .sum();
        feet.push(serde_json::json!({"body":name,"loadN":s.integrated_support_loads_n[i],
            "loaded":s.integrated_support_loads_n[i]>=minimum,"supportSubsteps":s.integrated_support_substeps[i],
            "origin":vec3(s.world.bodies[s.bodies[i]].translation()),
            "comVelocityMps":vec3(twists[i].0),"angularVelocityRadps":vec3(twists[i].1),
            "impulseWeightedTangentSpeedMps":if impulse>0.0 {Some(weighted_speed/impulse)} else {None},
            "contacts":rows}));
    }
    Ok(
        serde_json::json!({"physicsIntegrations":0,"velocitySource":"current-generalized-motion",
        "gapSource":"paired body-local solver anchors resolved at current post-solve poses; cached distance is last full narrow-phase update",
        "nativeFrameDtS":s.world.integration_parameters.dt,
        "nativeBaseSolverIterations":s.world.integration_parameters.num_solver_iterations,
        "humanMaximumAdditionalSolverIterations":extra,
        "expectedHumanFloorSolverSubstepS":solver_h,
        "staticContactErpInvS":softness.erp_inv_dt(solver_h),
        "staticContactCfmFactor":softness.cfm_factor(solver_h),
        "staticContactNaturalFrequencyHz":s.world.integration_parameters.static_contact_softness.natural_frequency,
        "staticContactDampingRatio":s.world.integration_parameters.static_contact_softness.damping_ratio,
        "maximumCorrectiveVelocityMps":s.world.integration_parameters.max_corrective_velocity(),
        "interpretation":"Post-solve velocity at solved foot-side contact points; foot-origin displacement also includes rolling and unloaded repositioning.","feet":feet}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contact_point_motion_includes_angular_velocity_without_integration() {
        assert_eq!(
            point_velocity(Vector::X, Vector::Y * 2.0, Vector::Z),
            Vector::X * 3.0
        );
        assert_eq!(
            point_velocity(Vector::X, Vector::Y * 2.0, Vector::ZERO),
            Vector::X
        );
        let s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let report = contact_diagnostic(&s).unwrap();
        assert_eq!(report["physicsIntegrations"], 0);
        assert!(
            report["feet"]
                .as_array()
                .unwrap()
                .iter()
                .all(|f| f["loaded"] == false && f["contacts"].as_array().unwrap().is_empty())
        );
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
        assert_eq!(s.substeps_completed, 0);
    }
}
