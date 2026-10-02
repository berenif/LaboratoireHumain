//! Acceptance uses actual body geometry and state, independent of controller decisions.
use crate::Simulation;
use lh_model::Joint;
use rapier3d::parry::bounding_volume::{Aabb, BoundingVolume};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

pub const MAX_ANCHOR_M: f32 = 0.08;
pub const MAX_FLOOR_M: f32 = 0.08;
pub const MAX_SELF_M: f32 = 0.005;
pub const MAX_LIMIT_RAD: f32 = 0.06;

/// Current solver contacts, excluding cached points not solved this step.
/// Rapier retains reporting impulses on those unselected manifold points;
/// `ContactPair::total_impulse` sums them as well. This path does not rescale
/// impulses or change warm-start state. Momentum fixtures validate it separately.
pub fn solver_normal_impulse(pair: &ContactPair) -> Result<Vector, String> {
    filtered_normal_impulse(pair, |_| true)
}
fn filtered_normal_impulse(
    pair: &ContactPair,
    eligible: impl Fn(Vector) -> bool,
) -> Result<Vector, String> {
    let mut impulse = Vector::ZERO;
    for manifold in pair.solver_manifolds() {
        if !manifold
            .data
            .solver_flags
            .contains(SolverFlags::COMPUTE_IMPULSES)
        {
            continue;
        }
        if !manifold.data.normal.is_finite() {
            return Err("Non-finite contact normal".into());
        }
        let mut selected = [u32::MAX; 4];
        for (count, contact) in manifold.data.solver_contacts.iter().enumerate() {
            let index = contact.contact_indices()[0];
            if count == selected.len() || selected[..count].contains(&index) {
                return Err("Invalid or duplicate solver contact index".into());
            }
            selected[count] = index;
            let point = manifold
                .points
                .get(index as usize)
                .ok_or("Missing solver contact point")?;
            if !point.data.impulse.is_finite() || point.data.impulse < 0.0 {
                return Err("Invalid normal impulse".into());
            }
            if eligible(manifold.data.normal) {
                impulse += manifold.data.normal * point.data.impulse;
            }
        }
    }
    if !impulse.is_finite() {
        return Err("Non-finite summed contact impulse".into());
    }
    Ok(impulse)
}

/// Only upward load on the anatomical collider can support it. Side walls and
/// a downward ceiling contact cannot be mistaken for a planted foot.
pub fn support_normal_impulse(pair: &ContactPair, collider: ColliderHandle) -> Result<f32, String> {
    let sign = if pair.collider2 == collider {
        1.0
    } else {
        -1.0
    };
    let impulse = filtered_normal_impulse(pair, |normal| normal.y * sign >= 0.65)?;
    Ok((impulse.y * sign).max(0.0))
}

pub fn floor_normal_impulse(
    world: &PhysicsWorld,
    floor: ColliderHandle,
    collider: ColliderHandle,
) -> Result<f32, String> {
    world
        .narrow_phase
        .contact_pair(floor, collider)
        .map(solver_normal_impulse)
        .transpose()
        .map(|impulse| impulse.map_or(0.0, |i| i.y.abs()))
}
/// Actual world-space tangential impulse on the measured collider. Reporting
/// only: the pair ordering selects the sign, without projecting or rescaling.
/// The current solver manifolds, not cached unselected points, own the total.
pub fn floor_tangent_impulse(
    world: &PhysicsWorld,
    floor: ColliderHandle,
    collider: ColliderHandle,
) -> Result<Vector, String> {
    let Some(pair) = world.narrow_phase.contact_pair(floor, collider) else {
        return Ok(Vector::ZERO);
    };
    solver_normal_impulse(pair)?;
    let sign = if pair.collider1 == collider {
        1.0
    } else {
        -1.0
    };
    let mut result = Vector::ZERO;
    for manifold in pair.solver_manifolds() {
        if manifold
            .data
            .solver_flags
            .contains(SolverFlags::COMPUTE_IMPULSES)
            && !manifold.data.solver_contacts.is_empty()
        {
            let impulse = manifold.data.reported_tangent_impulse;
            if !impulse.is_finite() {
                return Err("Non-finite tangent impulse".into());
            }
            result += sign * impulse;
        }
    }
    if !result.is_finite() {
        return Err("Non-finite summed tangent impulse".into());
    }
    Ok(result)
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Failure {
    pub tick: u64,
    pub substep: u32,
    pub check: String,
    pub offending: String,
    pub measured: f32,
    pub threshold: f32,
}
impl Failure {
    pub fn new(
        tick: u64,
        substep: u32,
        check: &str,
        offending: String,
        measured: f32,
        threshold: f32,
    ) -> Self {
        Self {
            tick,
            substep,
            check: check.into(),
            offending,
            measured,
            threshold,
        }
    }
}
pub fn coordinates(parent: Rotation, child: Rotation, p: &Joint) -> [f32; 3] {
    let mut relative = (parent * crate::rotation(p.parent_frame.rotation)).inverse()
        * (child * crate::rotation(p.child_frame.rotation));
    if relative.w < 0.0 {
        relative = -relative;
    }
    [relative.x, relative.y, relative.z].map(|v| {
        let a = 2.0 * v.atan2(relative.w);
        (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
    })
}
fn depth(a: &Collider, b: &Collider, aabb_a: &Aabb, aabb_b: &Aabb) -> Result<f32, String> {
    if !aabb_a.intersects(aabb_b) {
        return Ok(0.0);
    }
    rapier3d::parry::query::contact(a.position(), a.shape(), b.position(), b.shape(), 0.0)
        .map(|c| c.map_or(0.0, |c| (-c.dist).max(0.0)))
        .map_err(|e| format!("Unsupported shape measurement: {e:?}"))
}
pub fn inspect(s: &mut Simulation, substep: u32) -> Result<(), Failure> {
    let fail = |check: &str, offending: String, measured: f32, threshold: f32| {
        Failure::new(s.tick, substep, check, offending, measured, threshold)
    };
    let environment = s
        .environment_colliders()
        .map(|handle| {
            let collider = s
                .world
                .colliders
                .get(handle)
                .ok_or_else(|| fail("ownership", "environment".into(), 0.0, 0.0))?;
            Ok((collider, collider.compute_aabb()))
        })
        .collect::<Result<Vec<_>, Failure>>()?;
    // Collider poses cannot change during this read-only inspection. Cache
    // their exact transformed bounds for this call only; every pair still
    // performs the same rejection and the same penetrating-shape query.
    let mut bounds = [None; 25];
    for (i, h) in s.bodies.iter().enumerate() {
        let body = s
            .world
            .bodies
            .get(*h)
            .ok_or_else(|| fail("ownership", s.model.segments[i].id.clone(), 0.0, 0.0))?;
        let c = s
            .world
            .colliders
            .get(s.colliders[i])
            .ok_or_else(|| fail("ownership", s.model.segments[i].id.clone(), 0.0, 0.0))?;
        if !body.is_dynamic() || c.parent() != Some(*h) {
            return Err(fail("ownership", s.model.segments[i].id.clone(), 0.0, 0.0));
        }
        if !body.translation().is_finite()
            || !body.rotation().is_finite()
            || !body.linvel().is_finite()
            || !body.angvel().is_finite()
        {
            return Err(fail("finite", s.model.segments[i].id.clone(), 0.0, 0.0));
        }
        let body_bounds = *bounds[i].get_or_insert_with(|| c.compute_aabb());
        for (surface, surface_bounds) in &environment {
            if !surface.is_enabled() {
                continue;
            }
            let d = depth(surface, c, surface_bounds, &body_bounds)
                .map_err(|e| fail("measurement", e, 0.0, 0.0))?;
            s.diagnostics.max_floor_penetration_m = s.diagnostics.max_floor_penetration_m.max(d);
            if d > MAX_FLOOR_M {
                return Err(fail(
                    "floor",
                    s.model.segments[i].id.clone(),
                    d,
                    MAX_FLOOR_M,
                ));
            }
        }
        for (j, bound_j) in bounds
            .iter_mut()
            .enumerate()
            .take(s.bodies.len())
            .skip(i + 1)
        {
            if s.model.segments[i]
                .collision_exclusions
                .contains(&s.model.segments[j].id)
                || s.model.segments[j]
                    .collision_exclusions
                    .contains(&s.model.segments[i].id)
            {
                continue;
            }
            let other = s
                .world
                .colliders
                .get(s.colliders[j])
                .ok_or_else(|| fail("ownership", s.model.segments[j].id.clone(), 0.0, 0.0))?;
            let other_bounds = *bound_j.get_or_insert_with(|| other.compute_aabb());
            let d = depth(c, other, &body_bounds, &other_bounds)
                .map_err(|e| fail("measurement", e, 0.0, 0.0))?;
            s.diagnostics.max_self_penetration_m = s.diagnostics.max_self_penetration_m.max(d);
            if d > MAX_SELF_M {
                return Err(fail(
                    "self-contact",
                    format!("{}/{}", s.model.segments[i].id, s.model.segments[j].id),
                    d,
                    MAX_SELF_M,
                ));
            }
        }
        if let Some(p) = &s.model.segments[i].joint_profile {
            let h = s.joints[i]
                .ok_or_else(|| fail("ownership", s.model.segments[i].id.clone(), 0.0, 0.0))?;
            let joint = s
                .world
                .impulse_joints
                .get(h)
                .ok_or_else(|| fail("ownership", s.model.segments[i].id.clone(), 0.0, 0.0))?;
            let parent = s
                .world
                .bodies
                .get(joint.body1())
                .ok_or_else(|| fail("ownership", s.model.segments[i].id.clone(), 0.0, 0.0))?;
            let pa = parent
                .position()
                .transform_point(crate::vector(p.parent_frame.anchor));
            let pb = body
                .position()
                .transform_point(crate::vector(p.child_frame.anchor));
            let separation = pa.distance(pb);
            s.diagnostics.max_anchor_separation_m =
                s.diagnostics.max_anchor_separation_m.max(separation);
            if separation > MAX_ANCHOR_M {
                return Err(fail(
                    "joint-anchor",
                    s.model.segments[i].id.clone(),
                    separation,
                    MAX_ANCHOR_M,
                ));
            }
            let q = coordinates(*parent.rotation(), *body.rotation(), p);
            let mut error_sq = 0.0;
            for (k, c) in ['x', 'y', 'z'].iter().enumerate() {
                let e = if let Some(a) = p.axes.iter().find(|a| a.coordinate == *c) {
                    (a.min_radians - q[k]).max(q[k] - a.max_radians).max(0.0)
                } else {
                    q[k].abs()
                };
                error_sq += e * e;
            }
            let error = error_sq.sqrt();
            s.diagnostics.max_limit_error_rad = s.diagnostics.max_limit_error_rad.max(error);
            if error > MAX_LIMIT_RAD {
                return Err(fail(
                    "joint-limit",
                    s.model.segments[i].id.clone(),
                    error,
                    MAX_LIMIT_RAD,
                ));
            }
        }
    }
    Ok(())
}

/// Independent standing evidence from the inherited stable-up/contact contract.
/// A character resting on its trunk cannot pass a quiet standing observation.
pub fn standing_support(s: &Simulation, substep: u32) -> Result<(), Failure> {
    for (i, segment) in s.model.segments.iter().enumerate() {
        if matches!(
            segment.id.as_str(),
            "pelvis" | "torso" | "leftFoot" | "rightFoot"
        ) {
            let up = (*s.world.bodies[s.bodies[i]].rotation() * Vector::Y).y;
            if up < 0.97 {
                return Err(Failure::new(
                    s.tick,
                    substep,
                    "standing-up",
                    segment.id.clone(),
                    up,
                    0.97,
                ));
            }
        }
    }
    for side in ["left", "right"] {
        let mut supported = false;
        for (i, segment) in s.model.segments.iter().enumerate().filter(|(_, s)| {
            s.id.starts_with(side) && matches!(s.role.as_str(), "hindfoot" | "forefoot")
        }) {
            let c = &s.world.colliders[s.colliders[i]];
            let floor = &s.world.colliders[s.floor];
            let distance = rapier3d::parry::query::contact(
                floor.position(),
                floor.shape(),
                c.position(),
                c.shape(),
                0.012,
            )
            .map_err(|e| {
                Failure::new(
                    s.tick,
                    substep,
                    "standing-measurement",
                    format!("{}: {e:?}", segment.id),
                    1.0,
                    0.0,
                )
            })?
            .map_or(f32::MAX, |c| c.dist);
            let load = floor_normal_impulse(&s.world, s.floor, s.colliders[i])
                .map_err(|e| Failure::new(s.tick, substep, "standing-measurement", e, 1.0, 0.0))?
                / s.profile.dt_s;
            supported |= floor.is_enabled() && distance <= 0.012 && load >= 3.0;
        }
        if !supported {
            return Err(Failure::new(
                s.tick,
                substep,
                "standing-foot-support",
                side.into(),
                0.0,
                3.0,
            ));
        }
    }
    Ok(())
}

#[derive(Debug, Default, Serialize)]
pub struct QuietMetrics {
    pub max_linear_mps: f32,
    pub max_angular_radps: f32,
    pub max_pelvis_drift_m: f32,
    pub max_foot_drift_m: f32,
}
impl QuietMetrics {
    /// Called after every 1/240 s integration, never just at snapshot boundaries.
    pub fn observe(
        &mut self,
        s: &Simulation,
        reference: &lh_contracts::Snapshot,
        substep: u32,
    ) -> Result<(), Failure> {
        standing_support(s, substep)?;
        for (i, h) in s.bodies.iter().enumerate() {
            let b = &s.world.bodies[*h];
            let id = &s.model.segments[i].id;
            let linear = b.linvel().length();
            let angular = b.angvel().length();
            self.max_linear_mps = self.max_linear_mps.max(linear);
            self.max_angular_radps = self.max_angular_radps.max(angular);
            let drift = b
                .translation()
                .distance(crate::vector(reference.segments[i].position));
            let check = if linear > 0.1 {
                Some(("quiet-linear-speed", linear, 0.1))
            } else if angular > 0.5 {
                Some(("quiet-angular-speed", angular, 0.5))
            } else {
                None
            };
            if let Some((name, v, t)) = check {
                return Err(Failure::new(s.tick, substep, name, id.clone(), v, t));
            }
            if id == "pelvis" {
                self.max_pelvis_drift_m = self.max_pelvis_drift_m.max(drift);
                if drift > 0.03 {
                    return Err(Failure::new(
                        s.tick,
                        substep,
                        "quiet-pelvis-drift",
                        id.clone(),
                        drift,
                        0.03,
                    ));
                }
            }
            if matches!(s.model.segments[i].role.as_str(), "hindfoot" | "forefoot") {
                self.max_foot_drift_m = self.max_foot_drift_m.max(drift);
                if drift > 0.01 {
                    return Err(Failure::new(
                        s.tick,
                        substep,
                        "quiet-foot-drift",
                        id.clone(),
                        drift,
                        0.01,
                    ));
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Profile;
    #[test]
    fn reused_bounds_match_unfiltered_geometric_penetration_after_pose_changes() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let head = s
            .model
            .segments
            .iter()
            .position(|part| part.id == "head")
            .unwrap();
        for angle in [0.0, 0.4, -0.7] {
            for offset in [-0.15, 0.0, 0.1, 0.5] {
                let center = s.world.bodies[s.bodies[0]].translation();
                s.world.bodies[s.bodies[head]]
                    .set_translation(center + Vector::new(offset, 0.0, 0.0), true);
                s.world.bodies[s.bodies[head]].set_rotation(Rotation::from_rotation_y(angle), true);
                s.world
                    .bodies
                    .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
                let a = &s.world.colliders[s.colliders[0]];
                let b = &s.world.colliders[s.colliders[head]];
                let expected = rapier3d::parry::query::contact(
                    a.position(),
                    a.shape(),
                    b.position(),
                    b.shape(),
                    0.0,
                )
                .unwrap()
                .map_or(0.0, |contact| (-contact.dist).max(0.0));
                assert_eq!(
                    depth(a, b, &a.compute_aabb(), &b.compute_aabb())
                        .unwrap()
                        .to_bits(),
                    expected.to_bits()
                );
            }
        }
    }
    #[test]
    fn resting_on_the_trunk_is_not_quiet_standing() {
        let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
        s.world.bodies[s.bodies[0]]
            .set_rotation(Rotation::from_rotation_x(std::f32::consts::FRAC_PI_2), true);
        let failure = standing_support(&s, 0).expect_err("lying pelvis must fail");
        assert_eq!(failure.check, "standing-up");
        assert!(failure.measured < 0.97);
    }
    #[test]
    fn deliberate_floor_limit_ownership_and_nonfinite_violations_fail() {
        for kind in ["floor", "joint-limit", "ownership", "finite"] {
            let mut s = Simulation::new(0.0, Profile::default(), true).expect("fixture");
            match kind {
                "floor" => {
                    s.world.bodies[s.bodies[0]].set_translation(Vector::new(0.0, -0.2, 0.0), true)
                }
                "joint-limit" => {
                    let i = s
                        .model
                        .segments
                        .iter()
                        .position(|s| s.id == "leftShin")
                        .expect("shin");
                    let j = s
                        .world
                        .impulse_joints
                        .get(s.joints[i].expect("joint"))
                        .expect("joint alive")
                        .body1();
                    let p = s.model.segments[i].joint_profile.as_ref().expect("profile");
                    let anchor = s.world.bodies[j]
                        .position()
                        .transform_point(crate::vector(p.parent_frame.anchor));
                    let r =
                        *s.world.bodies[s.bodies[i]].rotation() * Rotation::from_rotation_x(-0.8);
                    s.world.bodies[s.bodies[i]].set_rotation(r, true);
                    s.world.bodies[s.bodies[i]]
                        .set_translation(anchor - r * crate::vector(p.child_frame.anchor), true);
                }
                "ownership" => {
                    s.world.bodies[s.bodies[0]].set_body_type(RigidBodyType::Fixed, true)
                }
                _ => s.world.bodies[s.bodies[0]].set_linvel(Vector::new(f32::NAN, 0.0, 0.0), true),
            }
            // Update fixture collider transforms without integrating.
            s.world
                .bodies
                .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
            let f = inspect(&mut s, 0).expect_err("known violation must fail");
            assert_eq!(f.check, kind);
        }
    }
    #[test]
    fn deliberate_nonexcluded_overlap_fails() {
        let mut s = Simulation::new(0.0, Profile::default(), false).expect("fixture");
        let head = s
            .model
            .segments
            .iter()
            .position(|s| s.id == "head")
            .expect("head");
        let pelvis = s.world.bodies[s.bodies[0]].translation();
        s.world.bodies[s.bodies[head]].set_translation(pelvis, true);
        s.world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
        assert_eq!(
            inspect(&mut s, 0).expect_err("overlap").check,
            "self-contact"
        );
    }
}
