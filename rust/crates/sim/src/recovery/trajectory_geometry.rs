//! Native convex-geometry service for coordinated reference adaptation.
//! No integration, forces, pose replay, contact constraint or production hook.
use super::trajectory_program::{Command, Coordinates, validate_command};
use crate::{Profile, Simulation, quat, vector};
use rapier3d::prelude::*;
use serde::Deserialize;
use serde_json::{Value, json};
use std::io::{BufRead, Write};

#[derive(Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct Request {
    coordinates: Option<Coordinates>,
    program_substep: Option<u32>,
    root_position: [f32; 3],
    root_rotation: [f32; 4],
}

fn query(
    s: &mut Simulation,
    request: Request,
    program: Option<&super::trajectory_program::CompiledProgram>,
) -> Result<Value, String> {
    let coordinates = match (request.coordinates, request.program_substep) {
        (Some(q), None) => q,
        (None, Some(step)) => {
            program
                .ok_or("No compiled geometry program")?
                .command(step)
                .positions
        }
        _ => return Err("Choose coordinates or a compiled program clock".into()),
    };
    validate_command(
        &s.model,
        &Command {
            positions: coordinates,
            velocities: [[0.0; 3]; 25],
        },
    )?;
    let root = Rotation::from_array(request.root_rotation);
    if !root.is_finite()
        || (root.length_squared() - 1.0).abs() > 1e-4
        || request.root_position.iter().any(|v| !v.is_finite())
    {
        return Err("Invalid geometric root".into());
    }
    let mut poses = [Pose::IDENTITY; 25];
    poses[0] = Pose::from_parts(Vector::from_array(request.root_position), root);
    for i in 1..25 {
        let part = &s.model.segments[i];
        let p = part.joint_profile.as_ref().unwrap();
        let j = s
            .model
            .segments
            .iter()
            .position(|s| Some(&s.id) == part.parent.as_ref())
            .unwrap();
        let tangent = coordinates[i].map(|v| (v * 0.5).tan());
        let relative = Rotation::from_xyzw(tangent[0], tangent[1], tangent[2], 1.0).normalize();
        let child = poses[j].rotation
            * crate::rotation(p.parent_frame.rotation)
            * relative
            * crate::rotation(p.child_frame.rotation).inverse();
        let anchor = poses[j].translation + poses[j].rotation * vector(p.parent_frame.anchor);
        poses[i] = Pose::from_parts(anchor - child * vector(p.child_frame.anchor), child);
    }
    for (i, pose) in poses.iter().enumerate() {
        s.world.bodies[s.bodies[i]].set_position(*pose, false);
    }
    s.world
        .bodies
        .propagate_modified_body_positions_to_colliders(&mut s.world.colliders);
    let mut constraints = Vec::new();
    let mut pair_bounds = Vec::new();
    let mut floor_bounds = Vec::new();
    let floor = &s.world.colliders[s.floor];
    for i in 0..25 {
        let a = &s.world.colliders[s.colliders[i]];
        let aabb_a = a.compute_aabb();
        floor_bounds.push(aabb_a.mins.y);
        if let Some(c) = rapier3d::parry::query::contact(
            floor.position(),
            floor.shape(),
            a.position(),
            a.shape(),
            0.03,
        )
        .map_err(|e| format!("Floor query: {e:?}"))?
        {
            constraints.push(json!({"a":-1,"b":i,"distance":c.dist,"normal":c.normal1.to_array(),
                "localA":c.point1.to_array(),"localB":poses[i].inverse_transform_point(c.point2).to_array()}));
        }
        for j in i + 1..25 {
            if s.model.segments[i]
                .collision_exclusions
                .contains(&s.model.segments[j].id)
                || s.model.segments[j]
                    .collision_exclusions
                    .contains(&s.model.segments[i].id)
            {
                continue;
            }
            let b = &s.world.colliders[s.colliders[j]];
            if let Some(c) = rapier3d::parry::query::contact(
                a.position(),
                a.shape(),
                b.position(),
                b.shape(),
                0.03,
            )
            .map_err(|e| format!("Self query: {e:?}"))?
            {
                pair_bounds.push(json!({"a":i,"b":j,"distanceLowerBound":c.dist}));
                constraints.push(
                    json!({"a":i,"b":j,"distance":c.dist,"normal":c.normal1.to_array(),
                    "localA":poses[i].inverse_transform_point(c.point1).to_array(),
                    "localB":poses[j].inverse_transform_point(c.point2).to_array()}),
                );
            } else {
                let aabb_b = b.compute_aabb();
                let gap = (aabb_a.mins - aabb_b.maxs)
                    .max(aabb_b.mins - aabb_a.maxs)
                    .max(Vector::ZERO);
                pair_bounds.push(json!({"a":i,"b":j,"distanceLowerBound":gap.length().max(0.03)}));
            }
        }
    }
    let mut diagnostics = lh_contracts::Diagnostics::default();
    let refusal = crate::measurement::inspect_world(s, &s.world, &mut diagnostics, 0).err();
    assert_eq!(s.substeps_completed, 0);
    Ok(
        json!({"poses":poses.iter().enumerate().map(|(i,p)|json!({"id":s.model.segments[i].id,
        "position":crate::vec3(p.translation),"rotation":quat(p.rotation)})).collect::<Vec<_>>(),
        "constraints":constraints,"pairBounds":pair_bounds,"floorBounds":floor_bounds,
        "thresholds":{"floor":null,"self":null},"penetrationPolicy":"diagnostic-only",
        "nativeRefusal":refusal,"physicalIntegrations":0}),
    )
}

#[test]
#[ignore = "explicit native geometry service; never integrates a world"]
fn serve_trajectory_geometry() {
    let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
    let program = std::env::var("LH_GEOMETRY_PROGRAM").ok().map(|path| {
        serde_json::from_str::<super::trajectory_program::Program>(
            &std::fs::read_to_string(path).unwrap(),
        )
        .unwrap()
        .compile(&s.model)
        .expect("Geometry program must pass the native command contract")
    });
    let handles = s.bodies.clone();
    let generation = s.generation;
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut writer = stdout.lock();
    for line in stdin.lock().lines() {
        let result = line
            .map_err(|e| e.to_string())
            .and_then(|line| serde_json::from_str::<Request>(&line).map_err(|e| e.to_string()))
            .and_then(|r| query(&mut s, r, program.as_ref()));
        let value = match result {
            Ok(v) => v,
            Err(e) => json!({"error":e}),
        };
        writeln!(
            writer,
            "GEOMETRY {}",
            serde_json::to_string(&value).unwrap()
        )
        .unwrap();
        writer.flush().unwrap();
        assert_eq!(handles, s.bodies);
        assert_eq!(generation, s.generation);
    }
}
