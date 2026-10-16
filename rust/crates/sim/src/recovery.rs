//! Contact-gated get-up motor intent. Physics bodies are read-only here.
//! Initialization fixtures are constructed before the first integration.
use crate::{Profile, Simulation, measurement, rotation, vector};
use lh_contracts::Motion;
use lh_model::{Model, Pose as BodyPose, Segment};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

#[cfg(test)]
pub(crate) mod native_predictor;
#[cfg(test)]
pub(crate) mod reference_mapping;
#[cfg(test)]
mod trajectory_geometry;
#[cfg(test)]
pub(crate) mod trajectory_program;

#[cfg(test)]
pub(crate) fn offline_trajectory_before_step(
    s: &mut Simulation,
    substep: u32,
) -> Result<(), measurement::Failure> {
    #[cfg(feature = "experimental-solver-step-observation")]
    if s.coordination.is_some() { return Ok(()); }
    if support_probe::getup_ab::active() {
        return Ok(());
    }
    support_probe::full_trajectory::before_step(s, substep)?;
    support_probe::hand_support_experiment::before_step(s, substep)?;
    support_probe::offline_trajectory::before_step(s, substep)
}

#[cfg(test)]
pub(crate) fn getup_ab_configure(s: &mut Simulation) -> Result<bool, String> {
    support_probe::getup_ab::configure(s)
}

#[cfg(all(test, feature = "experimental-solver-step-observation"))]
pub(crate) mod planner;

pub const FIXTURES: &str = include_str!("../data/recovery-v1.json");
const RATE: f32 = 1.0;
const STAND_RATE: f32 = RATE;
const HANDOFF_S: f64 = 0.6;

mod roll_arm;
use roll_arm::{RollArm, RollArmStage};

fn rolling_orientation(torso: Rotation, lead: usize) -> Rotation {
    let forward_y = (torso * Vector::Z).y;
    let lateral_y = (torso * Vector::X).y;
    if forward_y * forward_y + lateral_y * lateral_y < 1e-8 {
        return torso;
    }
    // Roll around the measured head-to-pelvis axis. Slerping to a fixed
    // world-prone pose instead asks a supine body to bend its spine in pitch.
    let mut angle = (-lateral_y).atan2(-forward_y);
    let direction = if lead == 0 { 1.0 } else { -1.0 };
    if forward_y > 0.0 && angle * direction < 0.0 {
        angle += direction * std::f32::consts::TAU;
    }
    torso * Rotation::from_rotation_y(angle * 0.18)
}

fn blend_targets(current: &mut [[f32; 3]], desired: &[[f32; 3]], maximum_delta: f32) {
    let maximum = current
        .iter()
        .zip(desired)
        .flat_map(|(a, b)| a.iter().zip(b).map(|(a, b)| (a - b).abs()))
        .fold(0.0_f32, f32::max);
    let blend = if maximum > maximum_delta {
        maximum_delta / maximum
    } else {
        1.0
    };
    for (a, b) in current.iter_mut().zip(desired) {
        for k in 0..3 {
            a[k] += (b[k] - a[k]) * blend;
        }
    }
}

#[derive(Deserialize)]
struct FixtureSet {
    cases: Vec<Fixture>,
}
#[derive(Deserialize)]
struct Fixture {
    id: String,
    heading: f32,
    poses: Vec<BodyPose>,
}

pub(crate) fn validate_seed(model: &Model, poses: &[BodyPose]) -> Result<(), String> {
    if poses.len() != model.segments.len() {
        return Err("Recovery seed topology".into());
    }
    for (i, (segment, pose)) in model.segments.iter().zip(poses).enumerate() {
        let q = rotation(pose.rotation);
        if pose.id != segment.id
            || !pose.position.finite()
            || !pose.linear_velocity.finite()
            || !pose.angular_velocity.finite()
            || !q.is_finite()
            || (q.length_squared() - 1.0).abs() > 1e-4
            || vector(pose.linear_velocity).length_squared() != 0.0
            || vector(pose.angular_velocity).length_squared() != 0.0
        {
            return Err(format!("Invalid recovery seed {}", segment.id));
        }
        if let Some(joint) = &segment.joint_profile {
            let parent = model
                .segments
                .iter()
                .position(|s| Some(&s.id) == segment.parent.as_ref())
                .ok_or("Missing seed parent")?;
            if parent >= i {
                return Err("Recovery seed order".into());
            }
            let a = vector(poses[parent].position)
                + rotation(poses[parent].rotation) * vector(joint.parent_frame.anchor);
            let b = vector(pose.position) + q * vector(joint.child_frame.anchor);
            if a.distance(b) > 0.0001 {
                return Err(format!("Disconnected recovery seed {}", segment.id));
            }
        }
    }
    Ok(())
}

fn index(s: &Simulation, id: &str) -> usize {
    s.model
        .segments
        .iter()
        .position(|part| part.id == id)
        .expect("canonical recovery segment")
}
fn body<'a>(s: &'a Simulation, id: &str) -> &'a RigidBody {
    &s.world.bodies[s.bodies[index(s, id)]]
}
fn measured_coordinates(s: &Simulation) -> Result<Vec<[f32; 3]>, String> {
    let mut targets = s.rest_targets.clone();
    for (i, part) in s.model.segments.iter().enumerate() {
        if let Some(profile) = &part.joint_profile {
            let joint = s
                .world
                .impulse_joints
                .get(s.joints[i].ok_or("Lost recovery joint")?)
                .ok_or("Lost recovery joint")?;
            targets[i] = measurement::coordinates(
                *s.world.bodies[joint.body1()].rotation(),
                *s.world.bodies[s.bodies[i]].rotation(),
                profile,
            );
        }
    }
    Ok(targets)
}
fn up(s: &Simulation, id: &str) -> f32 {
    (*body(s, id).rotation() * Vector::Y).y
}
fn load(s: &Simulation, id: &str) -> f32 {
    let i = index(s, id);
    if s.integrated_support_substeps[i]
        < crate::recovery_contract::substeps(
            crate::recovery_contract::contract()
                .recovery_acceptance
                .load_persistence_s,
        )
    {
        return 0.0;
    }
    s.integrated_support_loads_n[i]
}
fn foot(s: &Simulation, side: &str) -> bool {
    load(s, &format!("{side}Foot")) + load(s, &format!("{side}Forefoot")) >= 3.0
}

/// A loaded anatomical surface point, retained until supported transfer.
/// The local point belongs to the segment body, not its CoM or collider frame.
#[derive(Debug, Clone, Copy)]
struct PlantedAnchor {
    segment: usize,
    local_point: Vector,
    world_point: Vector,
}

// Only persistent, currently solved upward contacts contribute transfer evidence.
fn planted_contacts(
    s: &Simulation,
    segments: &[usize],
) -> Result<Vec<(PlantedAnchor, f32)>, String> {
    let mut contacts = Vec::new();
    for &segment in segments {
        if load(s, &s.model.segments[segment].id) <= 0.0 {
            continue;
        }
        let collider = s.colliders[segment];
        // This also validates duplicate/cached contact reporting consistently
        // with the physical support measurement.
        s.support_normal_impulse(collider)?;
        for pair in s.world.narrow_phase.contact_pairs_with(collider) {
            let first = pair.collider1 == collider;
            let other = if first {
                pair.collider2
            } else {
                pair.collider1
            };
            // Match Simulation::support_normal_impulse exactly. The striker
            // itself may collide with a limb, but cannot supply planted support.
            if !(other == s.floor
                || s.environment.as_ref().is_some_and(|e| e.contains(other))
                || s.striker
                    .as_ref()
                    .is_some_and(|s| s.room_colliders.contains(&other)))
                || !s.world.colliders[other].is_enabled()
            {
                continue;
            }
            for manifold in pair.solver_manifolds() {
                if !manifold
                    .data
                    .solver_flags
                    .contains(SolverFlags::COMPUTE_IMPULSES)
                {
                    continue;
                }
                let upward = manifold.data.normal.y * if first { -1.0 } else { 1.0 };
                if !upward.is_finite() {
                    return Err("Non-finite planted contact normal".into());
                }
                if upward < 0.65 {
                    continue;
                }
                for contact in manifold.data.solver_contacts.iter() {
                    let point = manifold
                        .points
                        .get(contact.contact_indices()[0] as usize)
                        .ok_or("Missing planted solver contact point")?;
                    if !point.data.impulse.is_finite() || point.data.impulse < 0.0 {
                        return Err("Invalid planted contact impulse".into());
                    }
                    let strength = upward * point.data.impulse;
                    if strength <= 0.0 {
                        continue;
                    }
                    let (a, b) = manifold
                        .data
                        .solver_contact_world_points(contact, &s.world.bodies);
                    let world_point = if first { a } else { b };
                    let local_point = s.world.bodies[s.bodies[segment]]
                        .position()
                        .inverse_transform_point(world_point);
                    if !world_point.is_finite() || !local_point.is_finite() {
                        return Err("Non-finite planted contact point".into());
                    }
                    contacts.push((
                        PlantedAnchor {
                            segment,
                            local_point,
                            world_point,
                        },
                        strength,
                    ));
                }
            }
        }
    }
    Ok(contacts)
}

fn planted_contact(s: &Simulation, segments: &[usize]) -> Result<Option<PlantedAnchor>, String> {
    Ok(planted_contacts(s, segments)?
        .into_iter()
        .reduce(|selected, candidate| {
            if candidate.1 > selected.1 {
                candidate
            } else {
                selected
            }
        })
        .map(|(anchor, _)| anchor))
}

// A release proof must exclude every contact whose constraint will be removed.
// For knee-to-sole transfer only that shin is removed; a swing removes the
// whole leg, so its shin and both sole segments cannot prove its own release.
fn supported_transfer(s: &Simulation, released: &[usize]) -> Result<bool, String> {
    supported_transfer_with(s, released, false)
}

fn supported_transfer_with(
    s: &Simulation,
    released: &[usize],
    rolling: bool,
) -> Result<bool, String> {
    let weight = s.total_mass() * 9.81;
    let released_load: f32 = released
        .iter()
        .map(|&i| load(s, &s.model.segments[i].id))
        .sum();
    if released_load > 0.25 * weight {
        return Ok(false);
    }
    let remaining: Vec<usize> = s
        .model
        .segments
        .iter()
        .enumerate()
        .filter(|(i, part)| {
            !released.contains(i)
                && (matches!(
                    part.role.as_str(),
                    "shin" | "hand" | "hindfoot" | "forefoot"
                ) || (rolling && !matches!(part.role.as_str(), "head" | "neck")))
                && load(s, &part.id) >= 3.0
        })
        .map(|(i, _)| i)
        .collect();
    let contacts = planted_contacts(s, &remaining)?;
    let remaining_load = contacts.iter().map(|(_, impulse)| *impulse).sum::<f32>() / s.profile.dt_s;
    if !remaining_load.is_finite() {
        return Err("Non-finite remaining recovery support".into());
    }
    if remaining_load < 0.8 * weight {
        return Ok(false);
    }
    let points = contacts
        .iter()
        .map(|(anchor, _)| {
            let point = s.heading.inverse() * anchor.world_point;
            [point.x, point.z]
        })
        .collect::<Vec<_>>();
    let com = s.center_of_mass();
    let velocity: Vector = s
        .bodies
        .iter()
        .map(|handle| {
            let body = &s.world.bodies[*handle];
            body.linvel() * body.mass()
        })
        .sum::<Vector>()
        / s.total_mass();
    if !com.is_finite() || !velocity.is_finite() {
        return Err("Non-finite recovery transfer state".into());
    }
    let capture = s.heading.inverse()
        * (com + velocity * ((com.y - s.support_height()).max(0.2) / 9.81).sqrt());
    let local_com = s.heading.inverse() * com;
    Ok(
        crate::stepping::inside_support(points.clone(), [local_com.x, local_com.z], 0.003)
            && crate::stepping::inside_support(points, [capture.x, capture.z], 0.003),
    )
}

fn planted_leg_targets(
    s: &Simulation,
    anchor: PlantedAnchor,
    root: Pose,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    let mut chain = vec![anchor.segment];
    loop {
        let part = &s.model.segments[*chain.last().ok_or("Empty planted leg chain")?];
        let parent = index(s, part.parent.as_ref().ok_or("Missing planted leg parent")?);
        if s.model.segments[parent].id == "pelvis" {
            break;
        }
        if chain.len() >= 5 {
            return Err("Invalid canonical planted leg chain".into());
        }
        chain.push(parent);
    }
    chain.reverse();
    crate::reaching::limb_targets(s, &chain, root, anchor.local_point, anchor.world_point)
}

/// Preserve Stand's coordinated two-link rise. Its center target is derived
/// from the retained contact with the endpoint in canonical upright orientation.
fn upright_foot_center(s: &Simulation, side: &str, anchor: PlantedAnchor) -> Vector {
    let foot = &s.model.segments[index(s, &format!("{side}Foot"))];
    let endpoint = &s.model.segments[anchor.segment];
    let offset = vector(endpoint.initial.position) - vector(foot.initial.position)
        + rotation(endpoint.initial.rotation) * anchor.local_point;
    anchor.world_point - s.heading * offset
}

type LegPlan = Vec<(usize, [f32; 3])>;

/// Project the virtual rise into poses that preserve both planted sole
/// position and orientation within the unchanged native joint bounds.
fn flat_stance_plan(
    s: &Simulation,
    root: Pose,
    goals: [Vector; 2],
    anchors: [PlantedAnchor; 2],
) -> Result<Option<(Pose, [LegPlan; 2])>, String> {
    stance_plan_at_pitch(s, root, goals, anchors, None)
}

fn stance_plan_at_pitch(
    s: &Simulation,
    mut root: Pose,
    goals: [Vector; 2],
    anchors: [PlantedAnchor; 2],
    pitches: Option<[f32; 2]>,
) -> Result<Option<(Pose, [LegPlan; 2])>, String> {
    let feet = [index(s, "leftFoot"), index(s, "rightFoot")];
    let plan = |root: Pose| -> Result<Option<[LegPlan; 2]>, String> {
        let solve = |side: usize| match pitches {
            Some(pitch) => crate::stepping::leg_targets_with_root_and_pitch(
                s,
                feet[side],
                goals[side],
                root,
                pitch[side],
            ),
            None => crate::stepping::leg_targets_with_root(s, feet[side], goals[side], root),
        };
        let legs = [solve(0)?, solve(1)?];
        for (side, ((leg, goal), anchor)) in legs.iter().zip(goals).zip(anchors).enumerate() {
            let sole = pitches.map_or(s.heading, |pitch| {
                s.heading * Rotation::from_rotation_x(pitch[side])
            });
            let chain: Vec<_> = leg.iter().map(|(i, _)| *i).collect();
            let angles: Vec<_> = leg.iter().map(|(_, angle)| *angle).collect();
            let pose = crate::reaching::endpoint_pose(s, &chain, root, &angles)?;
            if pose.translation.distance(goal) > 0.0001
                || (pose.rotation * Vector::Y).dot(sole * Vector::Y) < 0.99995
            {
                return Ok(None);
            }
            let mut support_chain = chain;
            let mut support_angles = angles;
            if anchor.segment != *support_chain.last().ok_or("Empty standing leg")? {
                support_chain.push(anchor.segment);
                support_angles.push(s.rest_targets[anchor.segment]);
            }
            let support_pose =
                crate::reaching::endpoint_pose(s, &support_chain, root, &support_angles)?;
            if support_pose
                .transform_point(anchor.local_point)
                .distance(anchor.world_point)
                > 0.0001
                || (support_pose.rotation * Vector::Y).dot(sole * Vector::Y) < 0.99995
            {
                return Ok(None);
            }
        }
        Ok(Some(legs))
    };
    let mut upper = root;
    upper.translation.y = s.support_height() + 2.0;
    upper = match crate::stepping::reachable_stance_root_at_pitch(s, feet, goals, upper, pitches) {
        Ok(reachable) => reachable,
        Err(error) if error == "Requested stance has no vertical leg reach" => return Ok(None),
        Err(error) => return Err(error),
    };
    root.translation.y = root.translation.y.min(upper.translation.y);
    if let Some(legs) = plan(root)? {
        return Ok(Some((root, legs)));
    }
    let Some(mut legs) = plan(upper)? else {
        return Ok(None);
    };
    // A low virtual pelvis can require dorsiflexion beyond the ankle limit.
    // Raising its IK intent to an admissible pose keeps the exact soles;
    // the coordinated native position/velocity trajectory remains bounded.
    let mut low = root.translation.y;
    for _ in 0..12 {
        let mut candidate = root;
        candidate.translation.y = (low + upper.translation.y) * 0.5;
        if let Some(candidate_legs) = plan(candidate)? {
            upper = candidate;
            legs = candidate_legs;
        } else {
            low = candidate.translation.y;
        }
    }
    Ok(Some((upper, legs)))
}

/// Balance against persistent, currently loaded solver points. Near-floor
/// vertices cannot supply a reaction moment until contact resolves there.
/// A braking orientation is an IK/motor request, never a body pose write.
fn standing_balance(s: &Simulation) -> Result<Option<(Rotation, bool)>, String> {
    let soles = s
        .model
        .segments
        .iter()
        .enumerate()
        .filter(|(i, part)| {
            matches!(part.role.as_str(), "hindfoot" | "forefoot")
                && load(s, &part.id) >= 3.0
                && s.integrated_support_loads_n[*i] >= 3.0
        })
        .map(|(i, _)| i)
        .collect::<Vec<_>>();
    let points = planted_contacts(s, &soles)?
        .into_iter()
        .map(|(anchor, _)| {
            let local = s.heading.inverse() * anchor.world_point;
            [local.x, local.z]
        })
        .collect::<Vec<_>>();
    let com = s.center_of_mass();
    let velocity = s
        .bodies
        .iter()
        .map(|handle| {
            let body = &s.world.bodies[*handle];
            body.linvel() * body.mass()
        })
        .sum::<Vector>()
        / s.total_mass();
    if !com.is_finite() || !velocity.is_finite() {
        return Err("Non-finite recovery balance state".into());
    }
    let height = (com.y - s.support_height()).max(0.2);
    let capture = s.heading.inverse() * (com + velocity * (height / 9.81).sqrt());
    let capture_point = [capture.x, capture.z];
    // A point/line contact cannot certify the inset support polygon. It can
    // still define a bounded braking request toward the measured support.
    let safe = crate::stepping::project_support(&points, capture_point, 0.003).or_else(|| {
        points
            .iter()
            .flat_map(|a| points.iter().map(move |b| (a, b)))
            .map(|(a, b)| {
                let d = [b[0] - a[0], b[1] - a[1]];
                let norm = d[0] * d[0] + d[1] * d[1];
                let t = if norm > 1e-12 {
                    ((capture_point[0] - a[0]) * d[0] + (capture_point[1] - a[1]) * d[1]) / norm
                } else {
                    0.0
                }
                .clamp(0.0, 1.0);
                [a[0] + t * d[0], a[1] + t * d[1]]
            })
            .min_by(|a, b| {
                let distance = |p: &[f32; 2]| {
                    (p[0] - capture_point[0]).powi(2) + (p[1] - capture_point[1]).powi(2)
                };
                distance(a).total_cmp(&distance(b))
            })
    });
    let Some(safe) = safe else {
        return Ok(None);
    };
    let local_com = s.heading.inverse() * com;
    let inside = crate::stepping::inside_support(points.clone(), [local_com.x, local_com.z], 0.003)
        && crate::stepping::inside_support(points, [capture.x, capture.z], 0.003);
    let tilt = Vector::new(
        -(capture.z - safe[1]).atan2(height),
        0.0,
        (capture.x - safe[0]).atan2(height),
    )
    .clamp_length_max(
        crate::recovery_contract::contract()
            .recovery_acceptance
            .stable_up_dot
            .acos(),
    );
    let yaw = (s.heading.inverse() * *body(s, "pelvis").rotation())
        .to_scaled_axis()
        .y;
    Ok(Some((
        s.heading * Rotation::from_scaled_axis(tilt) * Rotation::from_rotation_y(yaw),
        inside,
    )))
}

fn balanced_stance_plan(
    s: &Simulation,
    root: Pose,
    goals: [Vector; 2],
    anchors: [PlantedAnchor; 2],
    orientation: Rotation,
    pitches: Option<[f32; 2]>,
) -> Result<Option<(Pose, [LegPlan; 2])>, String> {
    // Only an orientation accepted by bilateral position/orientation FK and
    // unchanged native bounds may become the pelvis effort's reference.
    for fraction in (0..12).map(|i| 0.5_f32.powi(i)).chain([0.0]) {
        let candidate =
            Pose::from_parts(root.translation, root.rotation.slerp(orientation, fraction));
        if let Some(plan) = stance_plan_at_pitch(s, candidate, goals, anchors, pitches)? {
            return Ok(Some(plan));
        }
    }
    Ok(None)
}

struct MeasuredStance {
    root: Pose,
    legs: [LegPlan; 2],
    goals: [Vector; 2],
    pitches: [f32; 2],
}

fn measured_stance_plan(
    s: &Simulation,
    root: Pose,
    anchors: [PlantedAnchor; 2],
) -> Result<Option<MeasuredStance>, String> {
    let measured = ["leftFoot", "rightFoot"].map(|id| {
        (s.heading.inverse() * *body(s, id).rotation())
            .to_scaled_axis()
            .x
    });
    // Request the smallest reachable toe pivot, with every contact point fixed.
    // The independent standing proof still requires the frozen upright bound.
    for fraction in [0.0, 0.125, 0.25, 0.5, 0.75, 1.0] {
        let pitches = measured.map(|pitch| pitch * fraction);
        let goals = std::array::from_fn(|side| {
            let foot =
                &s.model.segments[index(s, if side == 0 { "leftFoot" } else { "rightFoot" })];
            let endpoint = &s.model.segments[anchors[side].segment];
            let offset = vector(endpoint.initial.position) - vector(foot.initial.position)
                + rotation(endpoint.initial.rotation) * anchors[side].local_point;
            anchors[side].world_point
                - s.heading * Rotation::from_rotation_x(pitches[side]) * offset
        });
        let plan = if fraction == 0.0 {
            flat_stance_plan(s, root, goals, anchors)?
        } else {
            stance_plan_at_pitch(s, root, goals, anchors, Some(pitches))?
        };
        if let Some((root, legs)) = plan {
            return Ok(Some(MeasuredStance {
                root,
                legs,
                goals,
                pitches,
            }));
        }
    }
    Ok(None)
}

pub(crate) fn has_persistent_support(s: &Simulation) -> bool {
    s.model.segments.iter().any(|part| load(s, &part.id) >= 3.0)
}

pub(crate) fn standing_reference(s: &Simulation) -> Vector {
    let feet = ["leftFoot", "rightFoot"].map(|id| body(s, id).translation());
    standing_reference_at_feet(s, feet)
}

fn standing_reference_at_feet(s: &Simulation, feet: [Vector; 2]) -> Vector {
    let initial_feet =
        ["leftFoot", "rightFoot"].map(|id| vector(s.model.segments[index(s, id)].initial.position));
    let initial_com = s.model.segments.iter().fold(Vector::ZERO, |sum, part| {
        sum + (vector(part.initial.position)
            + rotation(part.initial.rotation) * vector(part.golden_mass.center_of_mass))
            * part.mass_kg
    }) / s.total_mass();
    (feet[0] + feet[1]) * 0.5
        + s.heading * (initial_com - (initial_feet[0] + initial_feet[1]) * 0.5)
}

/// A planted joint can track its coordinate while transmitting a large support
/// moment. Reserve that measured load path inside the same native effort budget;
/// coordinate error alone does not describe the solver's holding demand.
pub(crate) fn support_effort_reserves(s: &Simulation) -> [[f32; 3]; 25] {
    let mut reserves = [[0.0; 3]; 25];
    if !s
        .recovery
        .as_ref()
        .is_some_and(|r| r.standing() || r.status.phase == Phase::Handoff)
    {
        return reserves;
    }
    let com = s.center_of_mass();
    let loads = ["left", "right"].map(|side| {
        if foot(s, side) {
            load(s, &format!("{side}Foot")) + load(s, &format!("{side}Forefoot"))
        } else {
            0.0
        }
    });
    for (i, part) in s.model.segments.iter().enumerate() {
        if !matches!(
            part.role.as_str(),
            "thigh" | "shin" | "ankle" | "hindfoot" | "forefoot"
        ) {
            continue;
        }
        let Some(profile) = &part.joint_profile else {
            continue;
        };
        let Some(joint) = s.joints[i].and_then(|handle| s.world.impulse_joints.get(handle)) else {
            continue;
        };
        let parent = &s.world.bodies[joint.body1()];
        let anchor = parent
            .position()
            .transform_point(vector(profile.parent_frame.anchor));
        let moment =
            (com - anchor).cross(-Vector::Y * loads[usize::from(part.id.starts_with("right"))]);
        let frame = *parent.rotation() * rotation(profile.parent_frame.rotation);
        for axis in &profile.axes {
            let (k, basis) = match axis.coordinate {
                'x' => (0, Vector::X),
                'y' => (1, Vector::Y),
                _ => (2, Vector::Z),
            };
            reserves[i][k] = moment
                .dot(frame * basis)
                .abs()
                .min(axis.max_motor_torque_nm);
        }
    }
    if let Some(controller) = s
        .recovery
        .as_ref()
        .filter(|r| r.status.phase == Phase::Handoff)
    {
        let remaining = 1.0 - controller.standing_authority().0;
        for reserve in reserves.iter_mut().flatten() {
            *reserve *= remaining;
        }
    }
    reserves
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum Phase {
    Roll,
    Brace,
    Kneel,
    PlantLead,
    PlantTrailing,
    Stand,
    Handoff,
    Ordinary,
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Status {
    pub phase: Phase,
    pub phase_time_s: f32,
    pub qualified_time_s: f32,
    pub stable_time_s: f32,
    pub loaded_feet: [bool; 2],
    pub elapsed_s: f32,
    pub stalled_s: f32,
    pub unsupported_s: f32,
    pub retries: u32,
    pub last_retry: Option<&'static str>,
    pub progress: f32,
}
#[derive(Clone)]
pub(crate) struct Controller {
    pub status: Status,
    targets: Vec<[f32; 3]>,
    hands: [Vector; 2],
    lead: usize,
    stand_feet: Option<[Vector; 2]>,
    stand_root: Option<Pose>,
    stand_goal: Option<Rotation>,
    best_progress: f32,
    velocities: Vec<[f32; 3]>,
    foot_anchors: [Option<PlantedAnchor>; 2],
    hand_anchors: [Option<PlantedAnchor>; 2],
    knee_anchors: [Option<PlantedAnchor>; 2],
    handoff_from: Option<Vec<[f32; 3]>>,
    stable_substeps: u64,
    handoff_substeps: u64,
    settle_substeps: u64,
    roll_arm: Option<RollArm>,
}
impl Controller {
    pub fn new(s: &Simulation) -> Result<Self, String> {
        if s.model.segments.len() != 25 {
            return Err("Recovery requires the complete anatomy".into());
        }
        let feet = [foot(s, "left"), foot(s, "right")];
        let height = body(s, "pelvis").translation().y - s.support_height();
        let phase = if feet.iter().all(|x| *x) && up(s, "torso") > 0.65 && height > 0.5 {
            Phase::Stand
        } else if up(s, "torso") > 0.65 && height > 0.30 {
            Phase::PlantLead
        } else if (*body(s, "torso").rotation() * Vector::Z).y < -0.45 {
            Phase::Brace
        } else {
            Phase::Roll
        };
        let targets = measured_coordinates(s)?;
        let hands = ["left", "right"].map(|side| {
            let shoulder = body(s, &format!("{side}UpperArm")).translation();
            let sign = if side == "left" { -1.0 } else { 1.0 };
            let p = shoulder + s.heading * Vector::new(sign * 0.12, 0.0, 0.18);
            Vector::new(
                p.x,
                s.surface_height(p.x, p.z, shoulder.y + 0.1) + 0.035,
                p.z,
            )
        });
        Ok(Self {
            status: Status {
                phase,
                phase_time_s: 0.0,
                qualified_time_s: 0.0,
                stable_time_s: 0.0,
                loaded_feet: feet,
                elapsed_s: 0.0,
                stalled_s: 0.0,
                unsupported_s: 0.0,
                retries: 0,
                last_retry: None,
                progress: 0.0,
            },
            targets,
            hands,
            lead: if phase == Phase::Roll {
                usize::from((*body(s, "torso").rotation() * Vector::X).y > 0.0)
            } else {
                usize::from(feet[1] && !feet[0])
            },
            stand_feet: None,
            stand_root: None,
            stand_goal: None,
            best_progress: f32::NEG_INFINITY,
            velocities: vec![[0.0; 3]; s.model.segments.len()],
            foot_anchors: [None; 2],
            hand_anchors: [None; 2],
            knee_anchors: [None; 2],
            handoff_from: None,
            stable_substeps: 0,
            handoff_substeps: 0,
            settle_substeps: 0,
            roll_arm: None,
        })
    }
    pub fn standing(&self) -> bool {
        self.status.phase == Phase::Stand
    }
    pub fn handed_off(&self) -> bool {
        matches!(self.status.phase, Phase::Handoff | Phase::Ordinary)
    }
    pub(crate) fn standing_authority(&self) -> (f32, f32) {
        if !self.handed_off() {
            return (0.0, 0.0);
        }
        let blend = |substeps: u64| {
            let t =
                (substeps as f32 / crate::recovery_contract::substeps(HANDOFF_S) as f32).min(1.0);
            t * t * (3.0 - 2.0 * t)
        };
        (
            blend(self.handoff_substeps),
            blend(self.handoff_substeps.saturating_sub(1)),
        )
    }
    pub fn velocities(&self) -> &[[f32; 3]] {
        &self.velocities
    }
    pub fn arm_transfer_stage(&self) -> Option<String> {
        self.roll_arm.as_ref().map(|arm| {
            format!("{}:{:?}", if arm.side == 0 { "left" } else { "right" }, arm.stage)
        })
    }
    pub fn native_root_authority(&self, s: &Simulation, part: &Segment) -> bool {
        if self.status.phase == Phase::Ordinary || part.role != "thigh" {
            return false;
        }
        if self.status.phase == Phase::Roll {
            return self.roll_root_goal().is_some()
                && self.roll_root_legs()[usize::from(part.id.starts_with("right"))];
        }
        let side = if part.id.starts_with("right") {
            "right"
        } else {
            "left"
        };
        foot(s, side) || load(s, &format!("{side}Shin")) >= 3.0
    }
    pub fn root_torque(&self, s: &Simulation, part: &Segment) -> Vector {
        if !self.native_root_authority(s, part) {
            return Vector::ZERO;
        }
        let side = usize::from(part.id.starts_with("right"));
        let loaded = if self.status.phase == Phase::Roll { self.roll_root_legs() } else {
            ["left", "right"].map(|side| foot(s, side) || load(s, &format!("{side}Shin")) >= 3.0)
        };
        if !loaded[side] {
            return Vector::ZERO;
        }
        let pelvis = body(s, "pelvis");
        let reference = if self.status.phase == Phase::Roll {
            self.roll_root_goal().unwrap_or(*pelvis.rotation())
        } else if self.standing() || self.status.phase == Phase::Handoff {
            self.stand_goal.unwrap_or(s.heading)
        } else {
            s.heading
        };
        let mut orientation = reference.inverse() * *pelvis.rotation();
        if orientation.w < 0.0 {
            orientation = -orientation;
        }
        let error = orientation.to_scaled_axis();
        let omega = reference.inverse() * pelvis.angvel();
        // Equal and opposite hip actuation restores the measured root. Only
        // loaded legs participate; the existing native motors enforce caps.
        let authority = if self.status.phase == Phase::Handoff {
            1.0 - self.standing_authority().0
        } else {
            1.0
        };
        reference * (error * 1000.0 + omega * 140.0) * authority
            / loaded.into_iter().filter(|loaded| *loaded).count() as f32
    }
    pub fn weight(&self, part: &Segment, coordinate: char) -> f32 {
        if self.handed_off() {
            return 1.0;
        }
        if self.standing() {
            return match part.role.as_str() {
                "thigh" => {
                    if coordinate == 'x' {
                        1.0
                    } else {
                        0.22
                    }
                }
                "shin" => 0.60,
                "ankle" | "hindfoot" | "forefoot" => 1.0,
                "lumbar" | "ribcage" => {
                    if coordinate == 'x' {
                        0.30
                    } else {
                        0.12
                    }
                }
                "upper-arm" | "forearm" | "shoulder-girdle" | "hand" | "forearm-twist" => 0.06,
                _ => 0.05,
            };
        }
        match part.role.as_str() {
            "thigh" => {
                if coordinate == 'x' {
                    1.0
                } else {
                    0.18
                }
            }
            "shin" => 1.0,
            "ankle" => 0.65,
            "hindfoot" => 0.35,
            "forefoot" => 0.15,
            "lumbar" | "ribcage" => {
                if coordinate == 'x' {
                    0.7
                } else {
                    0.25
                }
            }
            "upper-arm" | "forearm" | "shoulder-girdle" | "hand" | "forearm-twist" => {
                if self.standing() {
                    0.08
                } else if coordinate == 'x' {
                    0.8
                } else {
                    0.3
                }
            }
            _ => 0.05,
        }
    }
    fn enter(&mut self, phase: Phase) {
        self.status.phase = phase;
        self.status.phase_time_s = 0.0;
        self.status.qualified_time_s = 0.0;
        self.status.stalled_s = 0.0;
        self.status.stable_time_s = 0.0;
        self.best_progress = f32::NEG_INFINITY;
        self.stand_feet = None;
        self.stand_root = None;
        self.stable_substeps = 0;
        self.handoff_substeps = 0;
        self.settle_substeps = 0;
        if phase != Phase::Roll {
            self.roll_arm = None;
        }
    }
    fn retained_leg_anchor(&self, side: usize) -> Option<PlantedAnchor> {
        self.knee_anchors[side].or(self.foot_anchors[side])
    }
    fn release_supported_knees(&mut self, s: &Simulation) -> Result<(), String> {
        for side in 0..2 {
            if let Some(knee) = self.knee_anchors[side]
                && self.foot_anchors[side].is_some()
                && supported_transfer(s, &[knee.segment])?
            {
                self.knee_anchors[side] = None;
            }
        }
        Ok(())
    }
    fn prepare_leg_motion(&mut self, s: &Simulation, side: usize) -> Result<bool, String> {
        let requested = match self.status.phase {
            Phase::PlantLead => side == self.lead && self.foot_anchors[side].is_none(),
            Phase::PlantTrailing => side != self.lead,
            _ => return Ok(self.retained_leg_anchor(side).is_none()),
        };
        if !requested {
            return Ok(false);
        }
        if self.retained_leg_anchor(side).is_none() {
            return Ok(true);
        }
        let name = if side == 0 { "left" } else { "right" };
        let released = [
            index(s, &format!("{name}Shin")),
            index(s, &format!("{name}Foot")),
            index(s, &format!("{name}Forefoot")),
        ];
        if up(s, "torso") <= 0.65 || !supported_transfer(s, &released)? {
            return Ok(false);
        }
        self.knee_anchors[side] = None;
        self.foot_anchors[side] = None;
        Ok(true)
    }
    pub fn update(&mut self, s: &Simulation) -> Result<Vec<[f32; 3]>, String> {
        let dt = s.profile.dt_s;
        self.status.phase_time_s += dt;
        self.status.elapsed_s += dt;
        if self.handed_off() {
            self.handoff_substeps += 1;
            self.status.phase_time_s = self.handoff_substeps as f32 * dt;
            let from = self
                .handoff_from
                .as_ref()
                .ok_or("Missing measured handoff")?;
            let blend = |time: f32| {
                let t = (time / HANDOFF_S as f32).min(1.0);
                t * t * (3.0 - 2.0 * t)
            };
            let now = blend(self.status.phase_time_s);
            let next = blend(self.status.phase_time_s + dt);
            for (i, target) in self.targets.iter_mut().enumerate() {
                for (k, coordinate) in target.iter_mut().enumerate() {
                    let delta = s.rest_targets[i][k] - from[i][k];
                    *coordinate = from[i][k] + delta * now;
                    self.velocities[i][k] = delta * (next - now) / dt;
                }
            }
            if self.status.phase == Phase::Handoff
                && self.handoff_substeps >= crate::recovery_contract::substeps(HANDOFF_S)
            {
                self.status.phase = Phase::Ordinary;
            }
            return Ok(self.targets.clone());
        }
        self.status.unsupported_s = if has_persistent_support(s) {
            0.0
        } else {
            self.status.unsupported_s + dt
        };
        let feet = [foot(s, "left"), foot(s, "right")];
        self.status.loaded_feet = feet;
        let shins = [load(s, "leftShin") >= 3.0, load(s, "rightShin") >= 3.0];
        let height = body(s, "pelvis").translation().y - s.support_height();
        let torso_up = up(s, "torso");
        for (side, name) in ["left", "right"].into_iter().enumerate() {
            if feet[side] && self.foot_anchors[side].is_none() {
                self.foot_anchors[side] = planted_contact(
                    s,
                    &[
                        index(s, &format!("{name}Foot")),
                        index(s, &format!("{name}Forefoot")),
                    ],
                )?;
            }
            if load(s, &format!("{name}Hand")) >= 3.0 && self.hand_anchors[side].is_none() {
                self.hand_anchors[side] = planted_contact(s, &[index(s, &format!("{name}Hand"))])?;
            }
            if shins[side] && self.knee_anchors[side].is_none() {
                self.knee_anchors[side] = planted_contact(s, &[index(s, &format!("{name}Shin"))])?;
            }
        }
        self.release_supported_knees(s)?;
        // Stand has sole IK only. A still-planted knee must not lose its
        // constraint merely because the phase or initialization chose Stand.
        if self.standing() && self.knee_anchors.iter().any(Option::is_some) {
            self.enter(Phase::PlantTrailing);
        }
        let progress = match self.status.phase {
            Phase::Roll => -(*body(s, "torso").rotation() * Vector::Z).y,
            Phase::Brace | Phase::Kneel => height + torso_up,
            Phase::PlantLead => height + u8::from(feet[self.lead]) as f32,
            Phase::PlantTrailing => height + feet.iter().filter(|f| **f).count() as f32,
            Phase::Stand | Phase::Handoff | Phase::Ordinary => height + torso_up,
        };
        self.status.progress = progress;
        if progress > self.best_progress + 0.005 {
            self.best_progress = progress;
            self.status.stalled_s = 0.0;
        } else {
            self.status.stalled_s += dt;
        }
        if f64::from(self.status.stalled_s)
            >= crate::recovery_contract::contract()
                .recovery_acceptance
                .stall_s
            && self.status.unsupported_s < 0.2
        {
            self.status.retries += 1;
            self.status.last_retry = Some("stalled-phase");
            self.lead = 1 - self.lead;
            if self.roll_arm.as_ref().is_some_and(|arm| arm.stage != RollArmStage::Roll) {
                // A failed release proof cannot trigger a new roll while the
                // arm is being unloaded. Keep the same measured maneuver.
                self.lead = 1 - self.lead;
                self.status.last_retry = Some("arm-support-not-ready");
                self.status.stalled_s = 0.0;
            } else {
                let measured = Self::new(s)?;
                self.targets = measured.targets;
                self.hands = measured.hands;
                self.enter(measured.status.phase);
            }
        }
        let qualified = match self.status.phase {
            Phase::Roll => {
                (*body(s, "torso").rotation() * Vector::Z).y < -0.6 && has_persistent_support(s)
            }
            Phase::Brace => shins.iter().all(|x| *x) && height > 0.28 && torso_up > 0.40,
            Phase::Kneel => shins.iter().all(|x| *x) && height > 0.38 && torso_up > 0.75,
            Phase::PlantLead => feet[self.lead] && torso_up > 0.65,
            Phase::PlantTrailing => {
                feet.iter().all(|x| *x)
                    && self.knee_anchors.iter().all(Option::is_none)
                    && torso_up > 0.65
            }
            Phase::Stand | Phase::Handoff | Phase::Ordinary => false,
        };
        self.status.qualified_time_s = if qualified {
            self.status.qualified_time_s + dt
        } else {
            0.0
        };
        if f64::from(self.status.qualified_time_s) + 1e-6
            >= crate::recovery_contract::contract()
                .recovery_acceptance
                .phase_minimum_s
        {
            let next = match self.status.phase {
                Phase::Roll => Phase::Brace,
                Phase::Brace => Phase::Kneel,
                Phase::Kneel => Phase::PlantLead,
                Phase::PlantLead => Phase::PlantTrailing,
                _ => Phase::Stand,
            };
            self.enter(next);
        }
        let mut desired = s.rest_targets.clone();
        if self.standing() {
            let [Some(left), Some(right)] = self.foot_anchors else {
                // Persistent-load reporting alone cannot fabricate a surface
                // anchor. Retain measured motor intent until real contacts exist.
                self.velocities.fill([0.0; 3]);
                return Ok(self.targets.clone());
            };
            let planted = *self.stand_feet.get_or_insert_with(|| {
                [
                    upright_foot_center(s, "left", left),
                    upright_foot_center(s, "right", right),
                ]
            });
            let canonical_midpoint =
                (vector(s.model.segments[index(s, "leftFoot")].initial.position)
                    + vector(s.model.segments[index(s, "rightFoot")].initial.position))
                    * 0.5;
            let mut goal = (planted[0] + planted[1]) * 0.5
                + s.heading * (vector(s.model.segments[0].initial.position) - canonical_midpoint);
            let balance = standing_balance(s)?;
            let measured = body(s, "pelvis").translation();
            let mut center_error = goal - measured;
            center_error.y = 0.0;
            let mut root = self.stand_root.unwrap_or_else(|| {
                Pose::from_parts(
                    Vector::new(
                        measured.x,
                        measured.y.max(s.support_height() + 0.40),
                        measured.z,
                    ),
                    s.heading,
                )
            });
            // Keep knee reserve until measured mass has moved over the newly
            // loaded footprint. The virtual root belongs only to leg IK.
            if center_error.length() > 0.06 || torso_up < 0.80 {
                goal.y = root.translation.y;
            }
            // The planted-leg geometry must use the physical pelvis in the
            // support plane. A lagging horizontal virtual root creates a hip
            // task that opposes the measured balance request.
            root.translation.x = measured.x;
            root.translation.z = measured.z;
            root.translation.y += (goal.y - root.translation.y).clamp(-0.15 * dt, 0.15 * dt);
            let yaw = (s.heading.inverse() * *body(s, "pelvis").rotation())
                .to_scaled_axis()
                .y;
            let orientation =
                balance.map_or(s.heading * Rotation::from_rotation_y(yaw), |(q, _)| q);
            root.rotation = *body(s, "pelvis").rotation();
            if let Some(actual) = measured_stance_plan(s, root, [left, right])? {
                if let Some((reachable, _)) = balanced_stance_plan(
                    s,
                    actual.root,
                    actual.goals,
                    [left, right],
                    orientation,
                    Some(actual.pitches),
                )? {
                    self.stand_root = Some(actual.root);
                    self.stand_goal = Some(reachable.rotation);
                    for leg in actual.legs {
                        for (i, angles) in leg {
                            desired[i] = angles;
                        }
                    }
                } else {
                    desired = self.targets.clone();
                }
            } else {
                // An unreachable request cannot release a retained contact.
                desired = self.targets.clone();
            }
        }
        if !self.standing() {
            let measured = *body(s, "pelvis").position();
            let mut root = measured;
            let mut midpoint = measured.translation;
            if matches!(self.status.phase, Phase::PlantLead | Phase::PlantTrailing) {
                if let Some(lead) = self.foot_anchors[self.lead] {
                    midpoint.x = lead.world_point.x;
                    midpoint.z = lead.world_point.z;
                }
                root.translation += (midpoint - measured.translation).clamp_length_max(0.08);
            }
            // Plan within a bounded neighborhood of the measured pelvis. The
            // virtual pose is used only for reachable limb intent, never written.
            if self.status.phase != Phase::Roll {
                root.translation.y += 0.06 * torso_up.max(0.0);
                root.rotation = measured.rotation.slerp(s.heading, 0.08);
            }
            for (side, name) in ["left", "right"].into_iter().enumerate() {
                let foot_id = format!("{name}Foot");
                let present = body(s, &foot_id).translation();
                let moving = self.prepare_leg_motion(s, side)?;
                let sign = if side == 0 { -1.0 } else { 1.0 };
                let mut goal = self.foot_anchors[side]
                    .map(|a| a.world_point)
                    .unwrap_or_else(|| {
                        let offset = match self.status.phase {
                            Phase::Roll => {
                                measured.rotation * Vector::new(sign * 0.09, -0.42, -0.22)
                            }
                            Phase::Brace | Phase::Kneel => {
                                s.heading * Vector::new(sign * 0.09, 0.0, -0.32)
                            }
                            _ => s.heading * Vector::new(sign * 0.09, 0.0, 0.035),
                        };
                        measured.translation + offset
                    });
                if moving {
                    if self.status.phase == Phase::PlantTrailing {
                        goal.x = midpoint.x + (s.heading * Vector::new(sign * 0.18, 0.0, 0.0)).x;
                        goal.z = midpoint.z + (s.heading * Vector::new(sign * 0.18, 0.0, 0.0)).z;
                    }
                    let floor = s.surface_height(goal.x, goal.z, measured.translation.y + 0.2);
                    goal.y = floor + 0.045;
                    let horizontal =
                        Vector::new(goal.x - present.x, 0.0, goal.z - present.z).length();
                    if self.status.phase == Phase::PlantTrailing && horizontal > 0.08 {
                        goal.y += 0.10;
                    }
                    goal = present + (goal - present).clamp_length_max(0.12);
                }
                if let Some(anchor) = self.retained_leg_anchor(side).filter(|_| !moving) {
                    // A knee remains the active contact constraint until its
                    // own release is earned, including when a sole is detected.
                    for (i, angles) in planted_leg_targets(s, anchor, root)? {
                        desired[i] = angles;
                    }
                } else {
                    for (i, angles) in
                        crate::stepping::leg_targets_with_root(s, index(s, &foot_id), goal, root)?
                    {
                        desired[i] = angles;
                    }
                }
                let hand_id = format!("{name}Hand");
                let shoulder = body(s, &format!("{name}UpperArm")).translation();
                if let Some(anchor) = self.hand_anchors[side] {
                    // Release bracing only after bilateral sole support exists.
                    let sole_load: f32 = ["left", "right"]
                        .into_iter()
                        .map(|side| {
                            load(s, &format!("{side}Foot")) + load(s, &format!("{side}Forefoot"))
                        })
                        .sum();
                    if !feet.iter().all(|f| *f) || sole_load < 0.8 * s.total_mass() * 9.81 {
                        self.hands[side] = anchor.world_point;
                    } else {
                        self.hand_anchors[side] = None;
                        self.hands[side] = body(s, &hand_id).translation();
                    }
                } else {
                    let mut goal = shoulder + s.heading * Vector::new(sign * 0.10, 0.0, 0.20);
                    goal.y = s.surface_height(goal.x, goal.z, shoulder.y + 0.1) + 0.035;
                    // A high shoulder cannot command an unreachable floor brace.
                    let reach = vector(s.model.segments[index(s, &hand_id)].initial.position)
                        .distance(vector(
                            s.model.segments[index(s, &format!("{name}UpperArm"))]
                                .initial
                                .position,
                        ));
                    if goal.distance(shoulder) <= reach + 0.02 {
                        self.hands[side] = goal;
                    } else {
                        self.hands[side] = body(s, &hand_id).translation();
                    }
                }
                for (i, angles) in crate::reaching::arm_targets(
                    s,
                    index(s, &hand_id),
                    self.hand_anchors[side].map_or(Vector::ZERO, |a| a.local_point),
                    self.hands[side],
                )? {
                    desired[i] = angles;
                }
            }
            if self.status.phase == Phase::Roll {
                for (i, part) in s.model.segments.iter().enumerate() {
                    if matches!(part.role.as_str(), "lumbar" | "ribcage") {
                        desired[i] = s.rest_targets[i];
                    }
                }
            }
        }
        if self.status.phase == Phase::Roll {
            self.update_roll_arm(s, &mut desired)?;
        }
        // Entering Stand does not itself transfer a planted hand's load.
        // Keep its actual surface point until the same bilateral-sole gate
        // used during the preceding phases has earned release.
        if self.standing() {
            let sole_load: f32 = ["left", "right"]
                .into_iter()
                .map(|side| load(s, &format!("{side}Foot")) + load(s, &format!("{side}Forefoot")))
                .sum();
            for side in 0..2 {
                if let Some(anchor) = self.hand_anchors[side] {
                    if feet.iter().all(|f| *f) && sole_load >= 0.8 * s.total_mass() * 9.81 {
                        self.hand_anchors[side] = None;
                    } else {
                        for (i, angles) in crate::reaching::arm_targets(
                            s,
                            anchor.segment,
                            anchor.local_point,
                            anchor.world_point,
                        )? {
                            desired[i] = angles;
                        }
                    }
                }
            }
        }
        for (i, part) in s.model.segments.iter().enumerate() {
            if let Some(p) = &part.joint_profile {
                for (k, c) in ['x', 'y', 'z'].into_iter().enumerate() {
                    if let Some(axis) = p.axes.iter().find(|a| a.coordinate == c) {
                        desired[i][k] = desired[i][k].clamp(axis.min_radians, axis.max_radians);
                        self.targets[i][k] =
                            self.targets[i][k].clamp(axis.min_radians, axis.max_radians);
                    } else {
                        desired[i][k] = 0.0;
                        self.targets[i][k] = 0.0;
                    }
                }
            }
        }
        // One common progress parameter preserves the coordinated hip/knee/
        // ankle path. Independent rate clamps pitch the sole during a rise.
        let prior = self.targets.clone();
        if self.standing() {
            // Keep the commanded stance trajectory that started at measured
            // coordinates. Replacing it with actual coordinates every substep
            // erases the native position task whenever loaded limbs lag.
            blend_targets(&mut self.targets, &desired, STAND_RATE * dt);
        } else {
            blend_targets(&mut self.targets, &desired, RATE * dt);
        }
        for (i, (target, prior)) in self.targets.iter().zip(prior).enumerate() {
            self.velocities[i] = std::array::from_fn(|k| (target[k] - prior[k]) / dt);
        }
        Ok(self.targets.clone())
    }
}

impl Simulation {
    pub fn recovery_status(&self) -> Option<&Status> {
        self.recovery.as_ref().map(|r| &r.status)
    }
    pub(crate) fn observe_recovery(&mut self) {
        let contract = crate::recovery_contract::contract();
        let support = has_persistent_support(self);
        let upright_pose = measurement::standing_support(self, 0).is_ok()
            && body(self, "pelvis").translation().y - self.support_height() > 0.93
            && ["left", "right"].iter().all(|side| foot(self, side));
        let stable = upright_pose
            && self.bodies.iter().all(|h| {
                self.world.bodies[*h].linvel().length()
                    <= contract.recovery_acceptance.stable_linear_mps
                    && self.world.bodies[*h].angvel().length()
                        <= contract.recovery_acceptance.stable_angular_radps
            });
        let failed = self.recovery.as_ref().is_some_and(|r| {
            f64::from(r.status.unsupported_s) >= contract.recovery_acceptance.support_loss_s
                || (r.standing() || r.handed_off())
                    && (body(self, "pelvis").translation().y - self.support_height() < 0.56
                        || up(self, "torso") < 1.25_f32.cos())
                || r.handed_off() && !support
        });
        if failed {
            self.motion = Motion::Falling;
            self.recovery = None;
            self.stepping = None;
            self.motion_observation = crate::motion::Observation::default();
            return;
        }
        let balance_ready =
            standing_balance(self).is_ok_and(|balance| balance.is_some_and(|(_, inside)| inside));
        let settled = upright_pose
            && balance_ready
            && self.bodies.iter().all(|h| {
                self.world.bodies[*h].linvel().length()
                    <= contract.recovery_acceptance.settle_linear_mps
                    && self.world.bodies[*h].angvel().length()
                        <= contract.recovery_acceptance.settle_angular_radps
            });
        let handoff_ready =
            if let Some(controller) = self.recovery.as_mut().filter(|r| r.standing()) {
                controller.settle_substeps = if settled {
                    controller.settle_substeps + 1
                } else {
                    0
                };
                controller.status.qualified_time_s =
                    controller.settle_substeps as f32 * self.profile.dt_s;
                controller.settle_substeps
                    >= crate::recovery_contract::substeps(
                        contract.recovery_acceptance.settle_persistence_s,
                    )
            } else {
                false
            };
        if handoff_ready {
            let measured = match Controller::new(self) {
                Ok(c) => c.targets,
                Err(_) => return,
            };
            let Some([Some(left), Some(right)]) = self.recovery.as_ref().map(|r| r.foot_anchors)
            else {
                return;
            };
            let planted = [
                upright_foot_center(self, "left", left),
                upright_foot_center(self, "right", right),
            ];
            let ordinary = match crate::stepping::StepController::recovered_stance(self, planted) {
                Ok(c) => c,
                Err(_) => return,
            };
            self.rest_com = standing_reference_at_feet(self, ordinary.planted_feet());
            self.rest_com_roundoff = Vector::ZERO;
            self.stepping = Some(ordinary);
            let controller = self.recovery.as_mut().expect("active recovery");
            controller.enter(Phase::Handoff);
            controller.handoff_from = Some(measured.clone());
            controller.targets = measured;
        }
        if let Some(controller) = self.recovery.as_mut() {
            // The native stance owns all support actuation during handoff and
            // hold. An integrated interruption restarts the consecutive proof.
            controller.stable_substeps = if controller.status.phase == Phase::Ordinary && stable {
                controller.stable_substeps + 1
            } else {
                0
            };
            controller.status.stable_time_s = controller.stable_substeps as f32 * self.profile.dt_s;
            if controller.stable_substeps
                >= crate::recovery_contract::substeps(contract.acceptance.stable_observation_s)
            {
                self.motion = Motion::Upright;
                self.counters.recoveries += 1;
                self.recovery = None;
                // Keep the independently supplied planted stance reference;
                // completing the hold must not rebase it to the current COM.
                self.motion_observation = crate::motion::Observation::default();
            }
        }
    }
}

pub fn fixture_trials(
    profile: &Profile,
    selected: Option<&str>,
) -> Result<Vec<serde_json::Value>, String> {
    if !profile.recovery_enabled {
        return Err("Recovery diagnostic requires recovery_enabled".into());
    }
    let fixtures: FixtureSet = serde_json::from_str(FIXTURES).map_err(|e| e.to_string())?;
    let mut reports = Vec::new();
    for fixture in fixtures
        .cases
        .into_iter()
        .filter(|f| selected.is_none_or(|id| f.id == id))
    {
        let mut s = Simulation::build(
            fixture.heading,
            profile.clone(),
            true,
            None,
            Some(&fixture.poses),
        )?;
        let initial = s.snapshot();
        let original = s.bodies.clone();
        let original_joints = s.joints.clone();
        let original_colliders = s.colliders.clone();
        let generation = s.generation;
        let mut failure = None;
        let mut trace = Vec::new();
        let mut handoff_substeps = 0_u32;
        let mut stable_return_s = None;
        let contract = crate::recovery_contract::contract();
        let limit = contract.acceptance.recovery_limit_s;
        let hold = contract.acceptance.stable_observation_s;
        let required_substeps = crate::recovery_contract::substeps(hold) as u32;
        for _ in 0..((limit + hold) * f64::from(lh_contracts::TICK_HZ)).ceil() as u32 {
            if let Err(e) = s.advance_tick_observed(|state, substep| {
                if state.bodies != original
                    || state.joints != original_joints
                    || state.colliders != original_colliders
                    || state.generation != generation
                {
                    return Err(measurement::Failure::new(
                        state.tick,
                        substep,
                        "recovery-ownership",
                        "assembly/generation replaced".into(),
                        1.0,
                        0.0,
                    ));
                }
                if state.counters.recoveries > 0 {
                    stable_return_s.get_or_insert(
                        state.substeps_completed as f64
                            / f64::from(lh_contracts::TICK_HZ * lh_contracts::SUBSTEPS),
                    );
                    measurement::standing_support(state, substep)?;
                    let speed = state
                        .bodies
                        .iter()
                        .map(|h| {
                            let b = &state.world.bodies[*h];
                            (b.linvel().length() / contract.recovery_acceptance.stable_linear_mps)
                                .max(
                                    b.angvel().length()
                                        / contract.recovery_acceptance.stable_angular_radps,
                                )
                        })
                        .fold(0.0_f32, f32::max);
                    if state.motion != Motion::Upright || speed > 1.0 {
                        return Err(measurement::Failure::new(
                            state.tick,
                            substep,
                            "recovery-handoff",
                            "standing control lost the recovered pose".into(),
                            speed.max(if state.motion != Motion::Upright {
                                2.0
                            } else {
                                0.0
                            }),
                            1.0,
                        ));
                    }
                    handoff_substeps += 1;
                }
                Ok(())
            }) {
                failure = Some(e);
                break;
            }
            if s.tick % 15 == 0 || handoff_substeps == 1 {
                trace.push(
                    serde_json::json!({"snapshot":s.snapshot(),"recovery":s.recovery_status(),
                        "actuation":diagnostic_actuation(&s), "stanceIntent":s.recovery.as_ref().map(|r|
                            serde_json::json!({"root":r.stand_root.map(|root| crate::vec3(root.translation)),
                                "goalAxisRad":r.stand_goal.map(|goal|
                                    crate::vec3((s.heading.inverse() * goal).to_scaled_axis()))}))}),
                );
            }
            if handoff_substeps >= required_substeps
                || stable_return_s.is_none()
                    && s.substeps_completed >= crate::recovery_contract::substeps(limit)
            {
                break;
            }
        }
        let ownership = s.bodies == original
            && s.joints == original_joints
            && s.colliders == original_colliders
            && s.generation == generation
            && s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic());
        let passed = failure.is_none()
            && ownership
            && s.counters.recoveries == 1
            && s.motion == Motion::Upright
            && handoff_substeps >= required_substeps
            && stable_return_s.is_some_and(|time| time <= limit);
        reports.push(serde_json::json!({"id":fixture.id,"heading":fixture.heading,"passed":passed,
            "initial":initial,"final":s.snapshot(),"status":s.recovery_status(),"failure":failure,"dynamicOwnership":ownership,
            "handoffStandingSubsteps":handoff_substeps,"stableReturnTimeS":stable_return_s,
            "recoveryLimitS":limit,"stableObservationS":hold,"trace":trace}));
    }
    if reports.is_empty() {
        return Err("Unknown recovery fixture".into());
    }
    Ok(reports)
}

fn diagnostic_actuation(s: &Simulation) -> Vec<serde_json::Value> {
    s.model.segments.iter().enumerate().filter_map(|(i, part)| {
        let profile = part.joint_profile.as_ref()?;
        let joint = s.world.impulse_joints.get(s.joints[i]?)?;
        let parent = &s.world.bodies[joint.body1()];
        let child = &s.world.bodies[s.bodies[i]];
        let measured = measurement::coordinates(*parent.rotation(), *child.rotation(), profile);
        let frame = *parent.rotation() * rotation(profile.parent_frame.rotation);
        let root_effort = s.recovery.as_ref().map_or(Vector::ZERO,
            |r| frame.inverse() * r.root_torque(s, part));
        let motors = profile.axes.iter().map(|axis| {
            let k = match axis.coordinate { 'x' => 0, 'y' => 1, _ => 2 };
            let m = &joint.data.motors[k + 3];
                serde_json::json!({"coordinate":axis.coordinate,"measured":measured[k],"target":m.target_pos,
                    "plannedVelocityRadS":s.recovery.as_ref().map_or(0.0, |r| r.velocities()[i][k]),
                    "rootEffortNm":root_effort[k],
                    "targetVelocityRadS":m.target_vel,"stiffnessNmPerRad":m.stiffness,"dampingNmsPerRad":m.damping,
                    "lastSolverImpulseNms":m.impulse,"ceilingNm":m.max_force})
        }).collect::<Vec<_>>();
        Some(serde_json::json!({"id":part.id,"motors":motors}))
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rolling_orientation_preserves_long_axis_on_recorded_browser_states() {
        // Actual torso rotations at recovery entry and at the subsequent stall.
        for torso in [
            Rotation::from_xyzw(-0.68539315, 0.325458, 0.10550473, 0.6427924),
            Rotation::from_xyzw(-0.79252297, 0.3768578, 0.07043452, 0.4742621),
        ] {
            let lead = usize::from((torso * Vector::X).y > 0.0);
            let route = rolling_orientation(torso, lead);
            assert!((route * Vector::Y - torso * Vector::Y).length() < 1e-6);
            assert!((route * Vector::Z).y < (torso * Vector::Z).y - 0.1);
            let yaw = Rotation::from_rotation_y(1.0471976);
            let rotated = rolling_orientation(yaw * torso, lead);
            assert!(rotated.dot(yaw * route).abs() > 1.0 - 1e-6);
        }
    }

    #[test]
    fn rolling_orientation_retry_changes_the_supine_route() {
        let torso = Rotation::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let left = rolling_orientation(torso, 0);
        let right = rolling_orientation(torso, 1);
        assert!((left * Vector::Z).y < 0.9);
        assert!((right * Vector::Z).y < 0.9);
        assert!((left * Vector::Z).x * (right * Vector::Z).x < 0.0);
        assert!((left * Vector::Y - torso * Vector::Y).length() < 1e-6);
        assert!((right * Vector::Y - torso * Vector::Y).length() < 1e-6);
    }

    #[test]
    fn measured_toe_pivot_keeps_both_contact_points_inside_native_bounds() {
        for id in ["landed-crouch-left", "landed-crouch-left-heading"] {
            let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
            let fixture = fixtures.cases.iter().find(|f| f.id == id).unwrap();
            let s = Simulation::build(
                fixture.heading,
                crate::recovery_contract::pinned_profile(),
                true,
                None,
                Some(&fixture.poses),
            )
            .unwrap();
            let anchors = ["leftForefoot", "rightForefoot"].map(|id| {
                let segment = index(&s, id);
                let local_point = Vector::new(0.0, -0.045, 0.06);
                PlantedAnchor {
                    segment,
                    local_point,
                    world_point: body(&s, id).position().transform_point(local_point),
                }
            });
            let before = serde_json::to_value(s.snapshot()).unwrap();
            let plan = measured_stance_plan(&s, *body(&s, "pelvis").position(), anchors)
                .unwrap()
                .expect("reachable planted crouch");
            for (side, leg) in plan.legs.iter().enumerate() {
                let mut chain: Vec<_> = leg.iter().map(|(i, _)| *i).collect();
                let mut angles: Vec<_> = leg.iter().map(|(_, q)| *q).collect();
                chain.push(anchors[side].segment);
                angles.push(s.rest_targets[anchors[side].segment]);
                let pose = crate::reaching::endpoint_pose(&s, &chain, plan.root, &angles).unwrap();
                assert!(
                    pose.transform_point(anchors[side].local_point)
                        .distance(anchors[side].world_point)
                        < 0.0001
                );
                for (i, q) in leg {
                    for axis in &s.model.segments[*i].joint_profile.as_ref().unwrap().axes {
                        let k = ['x', 'y', 'z']
                            .iter()
                            .position(|c| *c == axis.coordinate)
                            .unwrap();
                        assert!((axis.min_radians..=axis.max_radians).contains(&q[k]));
                    }
                }
            }
            assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
        }
    }

    #[test]
    fn unreachable_stance_is_no_plan_without_an_ownership_error() {
        let mut s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
        for _ in 0..36 {
            s.advance_tick().unwrap();
        }
        let anchors = ["left", "right"].map(|side| {
            planted_contact(
                &s,
                &[
                    index(&s, &format!("{side}Foot")),
                    index(&s, &format!("{side}Forefoot")),
                ],
            )
            .unwrap()
            .unwrap()
        });
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let mut root = *body(&s, "pelvis").position();
        root.translation.z += 10.0;
        assert!(measured_stance_plan(&s, root, anchors).unwrap().is_none());
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
    }

    #[test]
    fn standing_handoff_keeps_anchor_derived_soles_instead_of_elevated_centers() {
        let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = fixtures
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let s = Simulation::build(
            fixture.heading,
            crate::recovery_contract::pinned_profile(),
            true,
            None,
            Some(&fixture.poses),
        )
        .unwrap();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let planted = ["left", "right"].map(|side| {
            let id = format!("{side}Forefoot");
            let segment = index(&s, &id);
            let local_point = Vector::new(0.0, -0.045, 0.06);
            upright_foot_center(
                &s,
                side,
                PlantedAnchor {
                    segment,
                    local_point,
                    world_point: body(&s, &id).position().transform_point(local_point),
                },
            )
        });
        for (side, goal) in ["left", "right"].into_iter().zip(planted) {
            assert!(body(&s, &format!("{side}Foot")).translation().y - goal.y > 0.02);
        }
        let stance = crate::stepping::StepController::recovered_stance(&s, planted).unwrap();
        assert_eq!(stance.planted_feet(), planted);
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
    }

    #[test]
    fn toe_only_support_cannot_certify_geometric_sole_balance() {
        let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        for fixture in fixtures
            .cases
            .into_iter()
            .filter(|f| f.id.starts_with("landed-crouch-left"))
        {
            let mut s = Simulation::build(
                fixture.heading,
                crate::recovery_contract::pinned_profile(),
                true,
                None,
                Some(&fixture.poses),
            )
            .unwrap();
            for _ in 0..3 {
                s.advance_tick().unwrap();
            }
            let before = serde_json::to_value(s.snapshot()).unwrap();
            let (_, inside) = standing_balance(&s)
                .unwrap()
                .expect("persistent toe support");
            assert!(
                !inside,
                "toe line cannot supply the rearward balance moment"
            );
            assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
        }
    }

    #[test]
    fn measured_capture_brakes_toward_loaded_support_without_body_writes() {
        for heading in [0.0, 0.6] {
            let mut s =
                Simulation::new(heading, crate::recovery_contract::pinned_profile(), true).unwrap();
            for _ in 0..36 {
                s.advance_tick().unwrap();
            }
            assert!(standing_balance(&s).unwrap().is_some());
            for sign in [-1.0, 1.0] {
                for handle in &s.bodies {
                    s.world.bodies[*handle].set_linvel(s.heading * Vector::Z * sign, true);
                }
                let before = serde_json::to_value(s.snapshot()).unwrap();
                let (goal, inside) = standing_balance(&s).unwrap().unwrap();
                let local = (s.heading.inverse() * goal).to_scaled_axis();
                assert!(!inside);
                assert!(local.x * sign < 0.0);
                assert!(
                    local.length()
                        <= crate::recovery_contract::contract()
                            .recovery_acceptance
                            .stable_up_dot
                            .acos()
                            + 0.001
                );
                assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
            }
        }
    }

    #[test]
    fn entry_settling_cannot_count_as_ordinary_stability() {
        let mut s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
        for _ in 0..36 {
            s.advance_tick().unwrap();
        }
        assert!(measurement::standing_support(&s, 0).is_ok());
        assert!(foot(&s, "left") && foot(&s, "right"));
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Stand;
        // The physical controller captures solved anchors before observation.
        // Preserve that boundary in this isolated settling/hold regression.
        controller.foot_anchors = ["left", "right"].map(|side| {
            planted_contact(
                &s,
                &[
                    index(&s, &format!("{side}Foot")),
                    index(&s, &format!("{side}Forefoot")),
                ],
            )
            .unwrap()
        });
        assert!(controller.foot_anchors.iter().all(Option::is_some));
        s.recovery = Some(controller);
        s.motion = Motion::Recovering;
        s.world.bodies[s.bodies[0]].set_linvel(Vector::Z * 0.3, true);
        let contract = crate::recovery_contract::contract();
        let settle =
            crate::recovery_contract::substeps(contract.recovery_acceptance.settle_persistence_s);
        for _ in 0..settle - 1 {
            s.observe_recovery();
            assert_eq!(s.recovery_status().unwrap().phase, Phase::Stand);
            assert_eq!(s.counters.recoveries, 0);
        }
        s.observe_recovery();
        assert_eq!(s.recovery_status().unwrap().phase, Phase::Handoff);
        s.recovery.as_mut().unwrap().status.phase = Phase::Ordinary;
        let hold = crate::recovery_contract::substeps(contract.acceptance.stable_observation_s);
        for _ in 0..hold {
            s.observe_recovery();
            assert_eq!(s.counters.recoveries, 0);
            assert_eq!(s.recovery.as_ref().unwrap().stable_substeps, 0);
        }
        for handle in &s.bodies {
            s.world.bodies[*handle].set_linvel(Vector::ZERO, true);
            s.world.bodies[*handle].set_angvel(Vector::ZERO, true);
        }
        for _ in 0..hold - 1 {
            s.observe_recovery();
            assert_eq!(s.counters.recoveries, 0);
        }
        s.observe_recovery();
        assert_eq!(s.counters.recoveries, 1);
        assert_eq!(s.motion, Motion::Upright);
    }

    #[test]
    fn loaded_root_effort_survives_relative_motion_inside_native_limits() {
        let mut s = forefoot_only_support(0.0);
        s.world.bodies[s.bodies[0]].set_angvel(Vector::new(0.2, -0.1, 0.15), true);
        for id in ["leftThigh", "rightThigh"] {
            let segment = index(&s, id);
            s.world.bodies[s.bodies[segment]].set_angvel(Vector::new(0.8, -0.35, 0.25), true);
        }
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Stand;
        let thigh = index(&s, "leftThigh");
        let measured_rotation = *s.world.bodies[s.bodies[thigh]].rotation();
        s.world.bodies[s.bodies[thigh]]
            .set_rotation(measured_rotation * Rotation::from_rotation_x(0.1), true);
        s.recovery = Some(controller);
        s.motion = Motion::Recovering;
        let before = serde_json::to_value(s.snapshot().segments).unwrap();
        s.configure_motors().unwrap();
        for (side, loaded) in [("left", true), ("right", false)] {
            let i = index(&s, &format!("{side}Thigh"));
            let joint = s.world.impulse_joints.get(s.joints[i].unwrap()).unwrap();
            let profile = s.model.segments[i].joint_profile.as_ref().unwrap();
            let parent = &s.world.bodies[joint.body1()];
            let child = &s.world.bodies[joint.body2()];
            let frame = *parent.rotation() * rotation(profile.parent_frame.rotation);
            let relative =
                frame.inverse() * (*child.rotation() * rotation(profile.child_frame.rotation));
            let v = relative.xyz();
            let jac =
                (relative.w * relative.w * Vector::X + relative.w * v.cross(Vector::X) + v.x * v)
                    / (relative.w * relative.w + v.x * v.x);
            let coordinate_velocity = jac.dot(frame.inverse() * (child.angvel() - parent.angvel()));
            let measured =
                measurement::coordinates(*parent.rotation(), *child.rotation(), profile)[0];
            let motor = &joint.data.motors[3];
            let recovery = s.recovery.as_ref().unwrap();
            let root_effort = (frame.inverse() * recovery.root_torque(&s, &s.model.segments[i])).x;
            assert_eq!(root_effort.abs() > 0.01, loaded);
            let actual_effort = motor.stiffness * (motor.target_pos - measured)
                + motor.damping * (motor.target_vel - coordinate_velocity);
            let expected = motor.stiffness * (motor.target_pos - measured)
                + motor.damping * recovery.velocities()[i][0]
                + root_effort
                - if loaded {
                    0.0
                } else {
                    motor.damping * coordinate_velocity
                };
            assert!((actual_effort - expected).abs() < 0.0001);
            assert_eq!(
                recovery.native_root_authority(&s, &s.model.segments[i]),
                loaded
            );
        }
        let total: f32 = s
            .model
            .segments
            .iter()
            .enumerate()
            .filter_map(|(i, part)| {
                let profile = part.joint_profile.as_ref()?;
                let joint = s.world.impulse_joints.get(s.joints[i]?)?;
                Some(
                    profile
                        .axes
                        .iter()
                        .map(|axis| {
                            let k = match axis.coordinate {
                                'x' => 0,
                                'y' => 1,
                                _ => 2,
                            };
                            let ceiling = joint.data.motors[k + 3].max_force;
                            assert!(ceiling <= axis.max_motor_torque_nm + 0.001);
                            ceiling
                        })
                        .sum::<f32>(),
                )
            })
            .sum();
        assert!(total <= s.profile.aggregate_motor_cap_nm + 0.001);
        assert_eq!(serde_json::to_value(s.snapshot().segments).unwrap(), before);
    }

    #[test]
    fn supported_rise_projects_infeasible_flat_soles_without_body_writes() {
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        for id in ["landed-crouch-left", "landed-crouch-left-heading"] {
            let fixture = set.cases.iter().find(|f| f.id == id).unwrap();
            let s = Simulation::build(
                fixture.heading,
                crate::recovery_contract::pinned_profile(),
                true,
                None,
                Some(&fixture.poses),
            )
            .unwrap();
            let before = serde_json::to_value(s.snapshot()).unwrap();
            let anchors = ["left", "right"].map(|side| {
                let segment = index(&s, &format!("{side}Forefoot"));
                let local_point = Vector::new(0.0, -0.045, 0.06);
                PlantedAnchor {
                    segment,
                    local_point,
                    world_point: s.world.bodies[s.bodies[segment]]
                        .position()
                        .transform_point(local_point),
                }
            });
            let goals = [
                upright_foot_center(&s, "left", anchors[0]),
                upright_foot_center(&s, "right", anchors[1]),
            ];
            let mut root = *body(&s, "pelvis").position();
            root.translation.y = 0.94;
            let raw =
                crate::stepping::leg_targets_with_root(&s, index(&s, "leftFoot"), goals[0], root)
                    .unwrap();
            let raw_chain: Vec<_> = raw.iter().map(|(i, _)| *i).collect();
            let raw_angles: Vec<_> = raw.iter().map(|(_, angles)| *angles).collect();
            let raw_pose =
                crate::reaching::endpoint_pose(&s, &raw_chain, root, &raw_angles).unwrap();
            // Positional refinement alone admits this tilted sole.
            assert!((raw_pose.rotation * Vector::Y).y < 0.99995);
            let (projected, legs) = flat_stance_plan(&s, root, goals, anchors).unwrap().unwrap();
            assert!(projected.translation.y > root.translation.y);
            assert_eq!(projected.translation.x, root.translation.x);
            assert_eq!(projected.translation.z, root.translation.z);
            for ((leg, goal), anchor) in legs.iter().zip(goals).zip(anchors) {
                let mut chain: Vec<_> = leg.iter().map(|(i, _)| *i).collect();
                let mut angles: Vec<_> = leg.iter().map(|(_, angle)| *angle).collect();
                let pose = crate::reaching::endpoint_pose(&s, &chain, projected, &angles).unwrap();
                assert!(pose.translation.distance(goal) <= 0.0001);
                assert!((pose.rotation * Vector::Y).y >= 0.99995);
                chain.push(anchor.segment);
                angles.push(s.rest_targets[anchor.segment]);
                assert!(
                    crate::reaching::endpoint(&s, &chain, projected, &angles, anchor.local_point)
                        .unwrap()
                        .distance(anchor.world_point)
                        <= 0.0001
                );
                for (i, angles) in leg {
                    for axis in &s.model.segments[*i].joint_profile.as_ref().unwrap().axes {
                        let k = match axis.coordinate {
                            'x' => 0,
                            'y' => 1,
                            _ => 2,
                        };
                        assert!((axis.min_radians..=axis.max_radians).contains(&angles[k]));
                    }
                }
            }
            assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
        }
    }

    #[test]
    fn supported_rise_retains_a_bounded_trajectory_from_measured_entry() {
        let mut s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
        for _ in 0..36 {
            s.advance_tick().unwrap();
        }
        assert!(foot(&s, "left") && foot(&s, "right"));
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Stand;
        let measured = measured_coordinates(&s).unwrap();
        assert_eq!(controller.targets, measured);
        for _ in 0..3 {
            let prior = controller.targets.clone();
            controller.update(&s).unwrap();
            assert!(controller.foot_anchors.iter().all(Option::is_some));
            if let Some(root) = controller.stand_root {
                assert_eq!(root.translation.x, body(&s, "pelvis").translation().x);
                assert_eq!(root.translation.z, body(&s, "pelvis").translation().z);
                assert!(
                    (root.rotation * body(&s, "pelvis").rotation().inverse())
                        .to_scaled_axis()
                        .length()
                        < 1e-5
                );
            }
            for (i, part) in s.model.segments.iter().enumerate() {
                if let Some(profile) = &part.joint_profile {
                    for axis in &profile.axes {
                        let k = match axis.coordinate {
                            'x' => 0,
                            'y' => 1,
                            _ => 2,
                        };
                        let from = prior[i][k].clamp(axis.min_radians, axis.max_radians);
                        let target = controller.targets[i][k];
                        assert!((axis.min_radians..=axis.max_radians).contains(&target));
                        assert!((target - from).abs() <= STAND_RATE * s.profile.dt_s + 1e-6);
                        assert!(controller.velocities[i][k].abs() <= STAND_RATE + 1e-4);
                        assert!(
                            (controller.velocities[i][k] - (target - from) / s.profile.dt_s).abs()
                                < 1e-4
                        );
                    }
                }
            }
        }
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
    }

    #[test]
    fn planted_support_reserves_effort_even_at_measured_joint_targets() {
        let mut s = forefoot_only_support(0.0);
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Stand;
        s.recovery = Some(controller);
        let before = s.snapshot();
        let reserves = support_effort_reserves(&s);
        assert!(reserves[index(&s, "leftAnkle")][0] > 0.0);
        for (i, part) in s.model.segments.iter().enumerate() {
            for (k, reserve) in reserves[i].iter().enumerate() {
                let ceiling = part
                    .joint_profile
                    .as_ref()
                    .and_then(|profile| {
                        profile
                            .axes
                            .iter()
                            .find(|axis| axis.coordinate == ['x', 'y', 'z'][k])
                    })
                    .map_or(0.0, |axis| axis.max_motor_torque_nm);
                assert!(reserve.is_finite() && *reserve >= 0.0 && *reserve <= ceiling);
                if part.id.starts_with("right")
                    || !matches!(
                        part.role.as_str(),
                        "thigh" | "shin" | "ankle" | "hindfoot" | "forefoot"
                    )
                {
                    assert_eq!(*reserve, 0.0);
                }
            }
        }
        assert_eq!(
            serde_json::to_value(s.snapshot()).unwrap(),
            serde_json::to_value(before).unwrap()
        );
        s.recovery.as_mut().unwrap().status.phase = Phase::Handoff;
        assert!(
            support_effort_reserves(&s)
                .iter()
                .flatten()
                .any(|v| *v > 0.0)
        );
        s.recovery.as_mut().unwrap().status.phase = Phase::Ordinary;
        assert_eq!(support_effort_reserves(&s), [[0.0; 3]; 25]);
    }

    // Isolate an actual solved forefoot-floor contact. The other anatomy stays
    // dynamic and jointed, but cannot supply a competing support contact.
    fn segment_only_support(heading: f32, id: &str) -> Simulation {
        let profile = Profile {
            reaching_iterations: 12,
            reaching_damping_m2: 0.0001,
            ..Profile::default()
        };
        let mut s = Simulation::new(heading, profile, true).unwrap();
        let forefoot = index(&s, id);
        for (i, &collider) in s.colliders.iter().enumerate() {
            s.world.colliders[collider].set_enabled(i == forefoot);
        }
        let floor_y = s.world.colliders[s.floor].compute_aabb().maxs.y;
        let sole_y = s.world.colliders[s.colliders[forefoot]]
            .compute_aabb()
            .mins
            .y;
        let shift = Vector::Y * (floor_y - sole_y - 0.0002);
        for &handle in &s.bodies {
            let body = &mut s.world.bodies[handle];
            let mut pose = *body.position();
            pose.translation += shift;
            body.set_position(pose, true);
        }
        s.world.step_with_events(&s.filter, &());
        let force = s.support_normal_impulse(s.colliders[forefoot]).unwrap() / s.profile.dt_s;
        assert!(
            force >= 3.0,
            "fixture must have a positive solved forefoot load"
        );
        // Persistence is supplied explicitly: this focused test exercises
        // contact selection and virtual IK, not the separately tested clock.
        s.integrated_support_loads_n[forefoot] = force;
        s.integrated_support_substeps[forefoot] = crate::recovery_contract::substeps(
            crate::recovery_contract::contract()
                .recovery_acceptance
                .load_persistence_s,
        );
        s
    }

    fn forefoot_only_support(heading: f32) -> Simulation {
        segment_only_support(heading, "leftForefoot")
    }

    #[test]
    fn brace_and_kneel_retain_actual_knee_anchors_without_transfer() {
        let s = segment_only_support(0.0, "leftShin");
        let shin = index(&s, "leftShin");
        let anchor = planted_contact(&s, &[shin]).unwrap().unwrap();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        for phase in [
            Phase::Brace,
            Phase::Kneel,
            Phase::PlantLead,
            Phase::PlantTrailing,
        ] {
            let mut controller = Controller::new(&s).unwrap();
            controller.status.phase = phase;
            controller.lead = usize::from(phase == Phase::PlantTrailing);
            controller.knee_anchors[0] = Some(anchor);
            assert!(!controller.prepare_leg_motion(&s, 0).unwrap());
            let retained = controller.retained_leg_anchor(0).unwrap();
            assert_eq!(retained.segment, shin);
            assert_eq!(retained.local_point, anchor.local_point);
            assert_eq!(retained.world_point, anchor.world_point);
        }
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }

    #[test]
    fn reported_sole_load_cannot_skip_a_still_planted_knee() {
        let mut s = segment_only_support(0.0, "leftShin");
        let shin = index(&s, "leftShin");
        let anchor = planted_contact(&s, &[shin]).unwrap().unwrap();
        let sole = index(&s, "leftForefoot");
        // A persistence/reporting value without a current solved sole point
        // cannot prove transfer or replace the real knee constraint.
        s.integrated_support_loads_n[sole] = s.total_mass() * 9.81;
        s.integrated_support_substeps[sole] = s.integrated_support_substeps[shin];
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Kneel;
        controller.knee_anchors[0] = Some(anchor);
        controller.release_supported_knees(&s).unwrap();
        assert!(foot(&s, "left"));
        assert!(controller.foot_anchors[0].is_none());
        assert!(!controller.prepare_leg_motion(&s, 0).unwrap());
        assert_eq!(
            controller.retained_leg_anchor(0).unwrap().world_point,
            anchor.world_point
        );
        // Even phase initialization and enough elapsed phase time cannot
        // turn stale bilateral sole reports into permission to omit knee IK.
        let other_sole = index(&s, "rightFoot");
        s.integrated_support_loads_n[other_sole] = s.total_mass() * 9.81;
        s.integrated_support_substeps[other_sole] = s.integrated_support_substeps[shin];
        controller.status.phase = Phase::Stand;
        let before = serde_json::to_value(s.snapshot()).unwrap();
        for _ in 0..120 {
            controller.update(&s).unwrap();
            assert_eq!(controller.status.phase, Phase::PlantTrailing);
            assert_eq!(
                controller.retained_leg_anchor(0).unwrap().world_point,
                anchor.world_point
            );
        }
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
    }

    #[test]
    fn remaining_soles_earn_knee_transfer_but_reject_leg_release_and_outward_capture() {
        let mut s = Simulation::new(0.0, crate::recovery_contract::pinned_profile(), true).unwrap();
        for _ in 0..36 {
            s.advance_tick().unwrap();
        }
        let knee = index(&s, "leftShin");
        assert!(foot(&s, "left") && foot(&s, "right"));
        assert!(supported_transfer(&s, &[knee]).unwrap());
        let leg = [knee, index(&s, "leftFoot"), index(&s, "leftForefoot")];
        assert!(
            !supported_transfer(&s, &leg).unwrap(),
            "the moving leg cannot count its own sole support"
        );
        // Keep the exact current solved support; only measured momentum now
        // puts capture beyond that footprint, which must revoke release.
        for handle in &s.bodies {
            s.world.bodies[*handle].set_linvel(Vector::Z * 2.0, true);
        }
        assert!(!supported_transfer(&s, &[knee]).unwrap());
    }

    #[test]
    fn moving_leg_and_reported_other_load_cannot_prove_its_own_release() {
        let mut s = forefoot_only_support(0.0);
        let own = [
            index(&s, "leftShin"),
            index(&s, "leftFoot"),
            index(&s, "leftForefoot"),
        ];
        let other = index(&s, "rightFoot");
        // Do not let the released-load bound short-circuit this regression:
        // the existing own forefoot contact must be excluded geometrically.
        s.integrated_support_loads_n[own[2]] = 3.0;
        s.integrated_support_loads_n[other] = s.total_mass() * 9.81;
        s.integrated_support_substeps[other] = s.integrated_support_substeps[own[2]];
        assert!(!supported_transfer(&s, &own).unwrap());
        s.integrated_support_loads_n[other] = 0.0;
        // A low unloaded knee by itself is not sufficient; neither the body's
        // currently loaded forefoot nor generic support may be counted twice.
        assert!(!supported_transfer(&s, &own).unwrap());
    }

    #[test]
    fn forefoot_only_support_retains_surface_anchor_during_virtual_leg_planning() {
        for heading in [0.0, 0.6] {
            let s = forefoot_only_support(heading);
            let hindfoot = index(&s, "leftFoot");
            let forefoot = index(&s, "leftForefoot");
            assert_eq!(load(&s, "leftFoot"), 0.0);
            assert!(foot(&s, "left"));
            let anchor = planted_contact(&s, &[hindfoot, forefoot]).unwrap().unwrap();
            assert_eq!(anchor.segment, forefoot);
            assert!(
                anchor.local_point.length() > 0.02,
                "surface point cannot be body center"
            );
            assert!(
                s.world.bodies[s.bodies[forefoot]]
                    .position()
                    .transform_point(anchor.local_point)
                    .distance(anchor.world_point)
                    < 1e-6
            );
            let before = serde_json::to_value(s.snapshot()).unwrap();
            let mut controller = Controller::new(&s).unwrap();
            controller.status.phase = Phase::PlantLead;
            controller.lead = 0;
            for _ in 0..3 {
                controller.update(&s).unwrap();
                let retained = controller.foot_anchors[0].unwrap();
                assert_eq!(retained.segment, forefoot);
                assert_eq!(retained.local_point, anchor.local_point);
                assert_eq!(retained.world_point, anchor.world_point);
            }
            let mut root = *body(&s, "pelvis").position();
            root.translation += s.heading * Vector::new(0.003, -0.004, 0.005);
            let targets = planted_leg_targets(&s, anchor, root).unwrap();
            let chain: Vec<_> = targets.iter().map(|&(i, _)| i).collect();
            assert_eq!(
                chain.last(),
                Some(&forefoot),
                "IK must include the toe joint"
            );
            assert_eq!(chain.len(), 5);
            let measured: Vec<_> = chain
                .iter()
                .map(|&i| {
                    let p = s.model.segments[i].joint_profile.as_ref().unwrap();
                    let parent = s
                        .world
                        .impulse_joints
                        .get(s.joints[i].unwrap())
                        .unwrap()
                        .body1();
                    measurement::coordinates(
                        *s.world.bodies[parent].rotation(),
                        *s.world.bodies[s.bodies[i]].rotation(),
                        p,
                    )
                })
                .collect();
            let uncorrected =
                crate::reaching::endpoint(&s, &chain, root, &measured, anchor.local_point).unwrap();
            let angles: Vec<_> = targets.iter().map(|&(_, angles)| angles).collect();
            let planned =
                crate::reaching::endpoint(&s, &chain, root, &angles, anchor.local_point).unwrap();
            assert!(
                planned.distance(anchor.world_point)
                    < uncorrected.distance(anchor.world_point) * 0.5,
                "virtual pelvis motion must restore the retained surface point, not the foot center"
            );
            assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
        }
    }

    #[test]
    fn stand_center_comes_from_forefoot_contact_at_canonical_upright_orientation() {
        let s = forefoot_only_support(0.6);
        let forefoot = index(&s, "leftForefoot");
        let anchor = planted_contact(&s, &[index(&s, "leftFoot"), forefoot])
            .unwrap()
            .unwrap();
        let center = upright_foot_center(&s, "left", anchor);
        let foot = &s.model.segments[index(&s, "leftFoot")];
        let toe = &s.model.segments[forefoot];
        let endpoint = Pose::from_parts(
            center + s.heading * (vector(toe.initial.position) - vector(foot.initial.position)),
            s.heading * rotation(toe.initial.rotation),
        );
        assert!(
            endpoint
                .transform_point(anchor.local_point)
                .distance(anchor.world_point)
                < 1e-6
        );
        // A body-center anchor loses both the sole offset and the forefoot's
        // canonical displacement from the hindfoot used by coordinated rise.
        assert!(center.distance(anchor.world_point) > 0.05);
        assert!(anchor.local_point.length() > 0.02);
    }

    #[test]
    fn standing_authority_and_native_root_effort_transfer_continuously() {
        let mut s = forefoot_only_support(0.0);
        s.world.bodies[s.bodies[0]].set_angvel(Vector::X * 0.2, true);
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Handoff;
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let part = &s.model.segments[index(&s, "leftThigh")];
        let initial = controller.root_torque(&s, part);
        assert!(initial.length() > 1.0);
        let steps = crate::recovery_contract::substeps(HANDOFF_S);
        let mut prior = 0.0;
        for substep in 0..=steps {
            controller.handoff_substeps = substep;
            let (authority, previous) = controller.standing_authority();
            assert!((0.0..=1.0).contains(&authority));
            assert!((previous - prior).abs() < 1e-6);
            assert!(authority >= prior && authority - prior < 0.011);
            assert!(
                (controller.root_torque(&s, part) - initial * (1.0 - authority)).length() < 1e-4
            );
            prior = authority;
        }
        assert_eq!(prior, 1.0);
        controller.status.phase = Phase::Ordinary;
        assert_eq!(controller.root_torque(&s, part), Vector::ZERO);
        assert_eq!(serde_json::to_value(s.snapshot()).unwrap(), before);
    }

    #[test]
    fn measured_handoff_has_matching_native_rates_and_cannot_count_during_blend() {
        let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = fixtures
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let mut s =
            Simulation::build(0.0, Profile::default(), true, None, Some(&fixture.poses)).unwrap();
        let mut controller = Controller::new(&s).unwrap();
        controller.enter(Phase::Handoff);
        controller.handoff_from = Some(controller.targets.clone());
        s.motion = Motion::Recovering;
        s.recovery = Some(controller);
        let before = serde_json::to_value(s.snapshot().segments).unwrap();
        for _ in 0..143 {
            s.configure_motors().unwrap();
            assert_eq!(s.recovery_status().unwrap().phase, Phase::Handoff);
            assert_eq!(s.counters.recoveries, 0);
            let mut caps = 0.0;
            let scale = s
                .profile
                .motor_scale
                .min(s.profile.aggregate_motor_cap_nm / s.motor_ceiling_sum);
            for (i, part) in s.model.segments.iter().enumerate() {
                if let Some(profile) = &part.joint_profile {
                    let joint = s.world.impulse_joints.get(s.joints[i].unwrap()).unwrap();
                    for axis in &profile.axes {
                        let k = ['x', 'y', 'z']
                            .iter()
                            .position(|c| *c == axis.coordinate)
                            .unwrap();
                        let motor = &joint.data.motors[k + 3];
                        assert!(
                            (motor.target_vel - s.recovery.as_ref().unwrap().velocities[i][k])
                                .abs()
                                < 1e-5
                        );
                        assert!(motor.max_force <= axis.max_motor_torque_nm);
                        assert_eq!(
                            motor.stiffness,
                            axis.passive_stiffness_nm_per_rad
                                * s.profile.posture_stiffness_multiplier
                                * scale
                        );
                        assert_eq!(
                            motor.damping,
                            axis.damping_nms_per_rad * s.profile.posture_damping_multiplier * scale
                        );
                        caps += motor.max_force;
                    }
                }
            }
            assert!(caps <= s.profile.aggregate_motor_cap_nm + 1e-3);
        }
        s.configure_motors().unwrap();
        assert_eq!(s.recovery_status().unwrap().phase, Phase::Ordinary);
        assert_eq!(s.counters.recoveries, 0);
        assert_eq!(serde_json::to_value(s.snapshot().segments).unwrap(), before);
    }
    #[test]
    fn recovery_requires_real_support_and_remains_opt_in() {
        assert!(!Profile::default().recovery_enabled);
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = set
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let profile = Profile {
            recovery_enabled: true,
            ..Profile::default()
        };
        let mut s = Simulation::build(0.0, profile, false, None, Some(&fixture.poses)).unwrap();
        let original = s.bodies.clone();
        for _ in 0..15 {
            s.advance_tick().unwrap();
            assert!(s.recovery.is_none());
            assert_eq!(s.counters.recoveries, 0);
            assert_eq!(s.bodies, original);
            assert!(s.bodies.iter().all(|h| s.world.bodies[*h].is_dynamic()));
        }
    }
    #[test]
    fn recovery_caps_and_input_lockout_survive_an_interrupting_impact() {
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = set
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let profile = Profile {
            recovery_enabled: true,
            ..Profile::default()
        };
        let mut s = Simulation::build(0.0, profile, true, None, Some(&fixture.poses)).unwrap();
        for _ in 0..10 {
            s.advance_tick().unwrap();
        }
        assert_eq!(s.motion, Motion::Recovering);
        let mut ceiling_sum = 0.0;
        for (i, handle) in s.joints.iter().enumerate() {
            if let (Some(handle), Some(p)) = (handle, &s.model.segments[i].joint_profile) {
                let joint = s.world.impulse_joints.get(*handle).unwrap();
                for axis in &p.axes {
                    let motor = joint
                        .data
                        .motor(crate::axis(axis.coordinate).unwrap())
                        .unwrap();
                    assert!(motor.max_force > 0.0 && motor.max_force <= axis.max_motor_torque_nm);
                    assert!(motor.target_vel.is_finite());
                    ceiling_sum += motor.max_force;
                }
            }
        }
        assert!(ceiling_sum <= s.profile.aggregate_motor_cap_nm + 1e-3);
        assert!(
            s.begin_grab(0, 1, lh_model::Vec3::default(), lh_model::Vec3::default())
                .is_err()
        );
        let poses = serde_json::to_value(s.snapshot().segments).unwrap();
        let handles = s.bodies.clone();
        s.observe_impact();
        assert_eq!(s.motion, Motion::Falling);
        assert!(s.recovery.is_none());
        assert!(s.grab.is_none());
        assert_eq!(s.counters.recoveries, 0);
        assert_eq!(s.bodies, handles);
        assert_eq!(serde_json::to_value(s.snapshot().segments).unwrap(), poses);
    }
    #[test]
    fn failed_rise_returns_to_passive_falling_without_changing_a_pose() {
        let set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = set
            .cases
            .iter()
            .find(|f| f.id == "landed-prone-left")
            .unwrap();
        let mut s =
            Simulation::build(0.0, Profile::default(), true, None, Some(&fixture.poses)).unwrap();
        let mut controller = Controller::new(&s).unwrap();
        controller.status.phase = Phase::Stand;
        s.recovery = Some(controller);
        s.motion = Motion::Recovering;
        let before = serde_json::to_value(s.snapshot().segments).unwrap();
        s.observe_recovery();
        assert_eq!(s.motion, Motion::Falling);
        assert!(s.recovery.is_none());
        assert_eq!(s.counters.recoveries, 0);
        assert_eq!(serde_json::to_value(s.snapshot().segments).unwrap(), before);
    }
    #[test]
    fn bounded_rise_preserves_the_sagittal_sole_orientation_during_interpolation() {
        let mut q = [[0.34, 0.0, 0.0], [0.66, 0.0, 0.0], [0.32, 0.0, 0.0]];
        let target = [[0.167, 0.0, 0.0], [0.342, 0.0, 0.0], [0.175, 0.0, 0.0]];
        for _ in 0..120 {
            let before = q;
            blend_targets(&mut q, &target, 1.0 / 240.0);
            assert!((q[1][0] - q[0][0] - q[2][0]).abs() < 1e-6);
            for (a, b) in q.iter().zip(before) {
                assert!((a[0] - b[0]).abs() <= 1.0 / 240.0 + 1e-6);
            }
        }
        assert_eq!(q, target);
    }
    #[test]
    fn canonical_seed_validation_rejects_reordered_disconnected_and_moving_bodies() {
        let model = Model::canonical().unwrap();
        let mut set: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        for fixture in &set.cases {
            validate_seed(&model, &fixture.poses).unwrap();
        }
        let poses = &mut set.cases[0].poses;
        poses.swap(0, 1);
        assert!(validate_seed(&model, poses).is_err());
        poses.swap(0, 1);
        poses[0].position.x += 0.1;
        assert!(validate_seed(&model, poses).is_err());
        poses[0].position.x -= 0.1;
        poses[0].linear_velocity.x = 1.0;
        assert!(validate_seed(&model, poses).is_err());
    }
    #[test]
    fn recovery_planning_preserves_every_dynamic_pose_and_caps_coordinates() {
        let fixtures: FixtureSet = serde_json::from_str(FIXTURES).unwrap();
        let fixture = fixtures
            .cases
            .iter()
            .find(|f| f.id == "landed-crouch-left")
            .unwrap();
        let s =
            Simulation::build(0.0, Profile::default(), true, None, Some(&fixture.poses)).unwrap();
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let mut controller = Controller::new(&s).unwrap();
        let targets = controller.update(&s).unwrap();
        assert_eq!(before, serde_json::to_value(s.snapshot()).unwrap());
        for (i, p) in s
            .model
            .segments
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.joint_profile.as_ref().map(|p| (i, p)))
        {
            for axis in &p.axes {
                let k = match axis.coordinate {
                    'x' => 0,
                    'y' => 1,
                    _ => 2,
                };
                assert!((axis.min_radians..=axis.max_radians).contains(&targets[i][k]));
            }
        }
    }
}

#[cfg(test)]
mod support_probe;
