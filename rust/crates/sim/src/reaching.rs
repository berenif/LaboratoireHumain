//! Bounded virtual arm kinematics. This module cannot mutate the physics world.
//! Candidate poses are temporary calculations used only to request motor targets.
use crate::{Simulation, measurement, rotation, vector};
use rapier3d::prelude::*;

fn coordinate_index(coordinate: char) -> Result<usize, String> {
    match coordinate {
        'x' => Ok(0),
        'y' => Ok(1),
        'z' => Ok(2),
        _ => Err("Invalid reaching coordinate".into()),
    }
}

fn endpoint(
    s: &Simulation,
    chain: &[usize],
    root: Pose,
    angles: &[[f32; 3]],
    anchor: Vector,
) -> Result<Vector, String> {
    let mut pose = root;
    for (&index, angle) in chain.iter().zip(angles) {
        let joint = s.model.segments[index]
            .joint_profile
            .as_ref()
            .ok_or("Missing reaching joint")?;
        // Invert the uncoupled coordinates 2 atan2(q_i, q_w), rather than
        // composing Euler rotations that would represent different coordinates.
        let half = Vector::new(angle[0], angle[1], angle[2]) * 0.5;
        let tangent = Vector::new(half.x.tan(), half.y.tan(), half.z.tan());
        let norm = (1.0 + tangent.length_squared()).sqrt();
        if !norm.is_finite() {
            return Err("Non-finite reaching orientation".into());
        }
        let relative = Rotation::from_xyzw(
            tangent.x / norm,
            tangent.y / norm,
            tangent.z / norm,
            1.0 / norm,
        );
        let orientation = pose.rotation
            * rotation(joint.parent_frame.rotation)
            * relative
            * rotation(joint.child_frame.rotation).inverse();
        pose = Pose::from_parts(
            pose.transform_point(vector(joint.parent_frame.anchor))
                - orientation * vector(joint.child_frame.anchor),
            orientation,
        );
    }
    let point = pose.transform_point(anchor);
    if !point.is_finite() {
        return Err("Non-finite reaching endpoint".into());
    }
    Ok(point)
}

pub(crate) fn arm_targets(
    s: &Simulation,
    segment: usize,
    anchor: Vector,
    target: Vector,
) -> Result<Vec<(usize, [f32; 3])>, String> {
    if !anchor.is_finite() || !target.is_finite() {
        return Err("Non-finite reaching intent".into());
    }
    let selected = s
        .model
        .segments
        .get(segment)
        .ok_or("Missing grab segment")?;
    if selected.role != "hand" || s.profile.reaching_iterations == 0 {
        return Ok(Vec::new());
    }
    let mut chain = vec![segment];
    while s.model.segments[*chain.last().ok_or("Empty arm chain")?].role != "shoulder-girdle" {
        let parent = s.model.segments[*chain.last().ok_or("Empty arm chain")?]
            .parent
            .as_ref()
            .ok_or("Missing arm parent")?;
        let index = s
            .model
            .segments
            .iter()
            .position(|p| &p.id == parent)
            .ok_or("Unknown arm parent")?;
        if chain.len() >= 5 {
            return Err("Invalid canonical arm chain".into());
        }
        chain.push(index);
    }
    chain.reverse();
    let root_id = s.model.segments[chain[0]]
        .parent
        .as_ref()
        .ok_or("Missing arm root")?;
    let root_index = s
        .model
        .segments
        .iter()
        .position(|p| &p.id == root_id)
        .ok_or("Unknown arm root")?;
    let root = *s
        .world
        .bodies
        .get(s.bodies[root_index])
        .ok_or("Lost arm root")?
        .position();
    let mut angles = Vec::with_capacity(chain.len());
    for &index in &chain {
        let joint = s.model.segments[index]
            .joint_profile
            .as_ref()
            .ok_or("Missing arm joint")?;
        let handle = s.joints[index].ok_or("Missing arm joint handle")?;
        let parent_handle = s
            .world
            .impulse_joints
            .get(handle)
            .ok_or("Lost arm joint")?
            .body1();
        let parent = s.world.bodies.get(parent_handle).ok_or("Lost arm parent")?;
        let child = s
            .world
            .bodies
            .get(s.bodies[index])
            .ok_or("Lost arm child")?;
        let mut current = measurement::coordinates(*parent.rotation(), *child.rotation(), joint);
        for axis in &joint.axes {
            let c = coordinate_index(axis.coordinate)?;
            current[c] = current[c].clamp(axis.min_radians, axis.max_radians);
        }
        angles.push(current);
    }
    for _ in 0..s.profile.reaching_iterations {
        for k in (0..chain.len()).rev() {
            let joint = s.model.segments[chain[k]]
                .joint_profile
                .as_ref()
                .ok_or("Missing arm joint")?;
            for axis in &joint.axes {
                let c = coordinate_index(axis.coordinate)?;
                let position = endpoint(s, &chain, root, &angles, anchor)?;
                let saved = angles[k][c];
                angles[k][c] += 0.001;
                let derivative = (endpoint(s, &chain, root, &angles, anchor)? - position) / 0.001;
                let delta = (derivative.dot(target - position)
                    / (derivative.length_squared() + s.profile.reaching_damping_m2))
                    .clamp(
                        -s.profile.reaching_coordinate_increment_rad,
                        s.profile.reaching_coordinate_increment_rad,
                    );
                angles[k][c] = (saved + delta).clamp(axis.min_radians, axis.max_radians);
            }
        }
    }
    Ok(chain.into_iter().zip(angles).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Profile;
    #[test]
    fn virtual_reach_improves_both_arms_at_rotated_headings_without_writing_bodies() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let s = Simulation::new(heading, Profile::default(), true).expect("fixture");
            for id in ["leftHand", "rightHand"] {
                let hand = s
                    .model
                    .segments
                    .iter()
                    .position(|p| p.id == id)
                    .expect("hand");
                let anchor = Vector::new(0.01, -0.02, 0.01);
                let start = s.world.bodies[s.bodies[hand]]
                    .position()
                    .transform_point(anchor);
                let target = start + s.heading * Vector::new(0.0, 0.08, 0.1);
                let before = serde_json::to_vec(&s.snapshot()).expect("before");
                let targets = arm_targets(&s, hand, anchor, target).expect("targets");
                let indices: Vec<_> = targets.iter().map(|p| p.0).collect();
                let angles: Vec<_> = targets.iter().map(|p| p.1).collect();
                let root = *s.world.bodies[s.bodies[2]].position();
                let resting: Vec<_> = indices.iter().map(|&i| s.rest_targets[i]).collect();
                let initial_fk =
                    endpoint(&s, &indices, root, &resting, anchor).expect("initial FK");
                assert!(
                    (initial_fk - start).length() < 1e-5,
                    "canonical world transforms independently match virtual chain"
                );
                let reached = endpoint(&s, &indices, root, &angles, anchor).expect("endpoint");
                assert!((reached - target).length() < (start - target).length() * 0.5);
                for &(i, angle) in &targets {
                    for axis in &s.model.segments[i]
                        .joint_profile
                        .as_ref()
                        .expect("joint")
                        .axes
                    {
                        let c = coordinate_index(axis.coordinate).expect("coordinate");
                        assert!((axis.min_radians..=axis.max_radians).contains(&angle[c]));
                    }
                }
                assert_eq!(before, serde_json::to_vec(&s.snapshot()).expect("after"));
            }
        }
    }
}
