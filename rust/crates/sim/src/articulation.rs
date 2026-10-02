//! Native, opt-in feasibility evidence for the unchanged canonical articulation.
//!
//! This module is deliberately separate from runtime stepping. It retains the
//! canonical bodies, colliders, frames, limits, and configured motor budgets,
//! then replaces only the joint representation during assembly. A failed native
//! first step is evidence of an unsupported candidate, never release admission.
use crate::{Profile, Simulation, anatomical_joint, measurement, rotation};
use rapier3d::prelude::*;
use serde_json::{Value, json};
use std::panic::{AssertUnwindSafe, catch_unwind};

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "Non-string native panic".into())
}

fn relative_rotation(
    s: &Simulation,
    parent: usize,
    child: usize,
    joint: &GenericJoint,
) -> Rotation {
    (s.world.bodies[s.bodies[parent]].rotation() * joint.local_frame1.rotation).inverse()
        * (s.world.bodies[s.bodies[child]].rotation() * joint.local_frame2.rotation)
}

/// Probe the engine's joint coordinate semantics without moving a rigid body.
/// A closed path in its accumulated coordinates need not close in orientation.
fn spherical_coordinate_probe(model: &lh_model::Model) -> Result<Value, String> {
    let segment = model
        .segments
        .iter()
        .find(|s| s.joint_profile.as_ref().is_some_and(|j| j.axes.len() == 3))
        .ok_or("Missing canonical spherical joint")?;
    let profile = segment.joint_profile.as_ref().ok_or("Missing joint")?;
    let mut joint = MultibodyJoint::new(anatomical_joint(profile)?, false);
    // Stay within this actual joint's positive and negative anatomical ranges.
    let excursion = |axis: char| -> Result<f32, String> {
        let a = profile
            .axes
            .iter()
            .find(|a| a.coordinate == axis)
            .ok_or("Missing axis")?;
        Ok(0.4_f32.min(a.max_radians * 0.5).min(-a.min_radians * 0.5))
    };
    let (x, y) = (excursion('x')?, excursion('y')?);
    for displacement in [[x, 0.0, 0.0], [0.0, y, 0.0], [-x, 0.0, 0.0], [0.0, -y, 0.0]] {
        joint.apply_displacement(&displacement);
    }
    let native = joint.coords();
    let measured = measurement::coordinates(
        Rotation::IDENTITY,
        rotation(profile.parent_frame.rotation)
            * joint.joint_rot()
            * rotation(profile.child_frame.rotation).inverse(),
        profile,
    );
    let max_error = (0..3)
        .map(|i| (native[i + 3] - measured[i]).abs())
        .fold(0.0_f32, f32::max);
    Ok(json!({
        "segment": segment.id,
        "probe": "Closed generalized-coordinate path x,y,-x,-y; no rigid-body writes",
        "excursionRadians": [x, y],
        "nativeCoordinatesRadians": [native[3], native[4], native[5]],
        "anatomicalCoordinatesRadians": measured,
        "maxCoordinateMismatchRadians": max_error,
        "sameCoordinateSemantics": max_error <= 1.0e-5,
        "implication": "Native multi-axis limits and motor targets use accumulated angular velocity, not canonical quaternion-derived anatomical coordinates. Reusing the same numbers does not preserve the anatomical limit/motor contract."
    }))
}

/// Build every canonical segment with the normal constructor; transfer its exact
/// joint data before the first integration. No body pose/velocity setter is used.
fn assemble(profile: &Profile) -> Result<(Simulation, Value), String> {
    assemble_selected(profile, false, true)
}

fn assemble_selected(
    profile: &Profile,
    hinges_only: bool,
    floor_enabled: bool,
) -> Result<(Simulation, Value), String> {
    let mut s = Simulation::new(0.0, profile.clone(), floor_enabled)?;
    let before_bodies = serde_json::to_value(&s.world.bodies).map_err(|e| e.to_string())?;
    let before_colliders = serde_json::to_value(&s.world.colliders).map_err(|e| e.to_string())?;
    let mut joints = Vec::new();
    let mut max_initial_rotation_error = 0.0_f32;
    let mut max_initial_anchor_error = 0.0_f32;
    for child in 0..s.model.segments.len() {
        if hinges_only
            && s.model.segments[child]
                .joint_profile
                .as_ref()
                .is_none_or(|j| j.axes.len() != 1)
        {
            continue;
        }
        let Some(handle) = s.joints[child].take() else {
            continue;
        };
        let parent_id = s.model.segments[child]
            .parent
            .as_ref()
            .ok_or("Missing parent")?;
        let parent = s
            .model
            .segments
            .iter()
            .position(|p| &p.id == parent_id)
            .ok_or("Unknown parent")?;
        let data = s
            .world
            .remove_impulse_joint(handle)
            .ok_or("Missing impulse joint")?;
        let before_joint = serde_json::to_value(data).map_err(|e| e.to_string())?;
        let relative = relative_rotation(&s, parent, child, &data);
        let desired = s.world.bodies[s.bodies[parent]].position().inverse()
            * *s.world.bodies[s.bodies[child]].position();
        let handle = s
            .world
            .insert_multibody_joint(s.bodies[parent], s.bodies[child], data)
            .ok_or("Canonical tree rejected by MultibodyJointSet")?;
        let (multibody, link_index) = s
            .world
            .multibody_joints
            .get_mut(handle)
            .ok_or("Missing multibody")?;
        let joint = &mut multibody.link_mut(link_index).ok_or("Missing link")?.joint;
        // Initial generalized coordinates are part of assembly. We never call
        // forward_kinematics/update_rigid_bodies to overwrite a body pose.
        let angular = relative.to_scaled_axis();
        match joint.ndofs() {
            1 => {
                let axis = (0..3).find(|i| !data.locked_axes.bits() & (1 << (i + 3)) != 0)
                    .ok_or("Missing revolute axis")?;
                joint.apply_displacement(&[angular[axis]]);
            }
            2 if angular.length() <= 1.0e-6 => {}
            2 => return Err("Non-neutral canonical wrist cannot initialize: Rapier two-axis integrate is unimplemented".into()),
            3 => joint.apply_displacement(&angular.to_array()),
            _ => return Err("Unexpected canonical angular DOFs".into()),
        }
        let assembled = joint.body_to_parent();
        // Quaternion sign does not change rotation. Avoid acos roundoff near 1.
        let delta = (assembled.rotation.inverse() * desired.rotation)
            .to_scaled_axis()
            .length();
        max_initial_rotation_error = max_initial_rotation_error.max(delta);
        max_initial_anchor_error =
            max_initial_anchor_error.max((assembled.translation - desired.translation).length());
        let copied = before_joint == serde_json::to_value(joint.data).map_err(|e| e.to_string())?;
        joints.push(
            json!({"segment": s.model.segments[child].id, "angularDofs": joint.ndofs(),
            "exactGenericJointDataPreserved": copied, "kinematic": joint.kinematic}),
        );
    }
    let multibodies: Vec<_> = s.world.multibody_joints.multibodies().collect();
    let bodies_unchanged =
        before_bodies == serde_json::to_value(&s.world.bodies).map_err(|e| e.to_string())?;
    let colliders_unchanged =
        before_colliders == serde_json::to_value(&s.world.colliders).map_err(|e| e.to_string())?;
    let report = json!({
        "bodyCount": s.bodies.len(), "dynamicBodyCount": s.bodies.iter().filter(|h| s.world.bodies[**h].is_dynamic()).count(),
        "anatomicalColliderCount": s.colliders.len(), "massKg": s.total_mass(),
        "impulseJointCount": s.world.impulse_joints.len(), "multibodyJointCount": joints.len(),
        "multibodyCount": multibodies.len(),
        "linkCount": multibodies.iter().map(|m| m.num_links()).sum::<usize>(),
        "generalizedDofsIncludingFreeRoot": multibodies.iter().map(|m| m.ndofs()).sum::<usize>(),
        "freeRoot": s.world.bodies[s.bodies[0]].is_dynamic()
            && multibodies.iter().all(|m| m.root().joint.ndofs() == 6),
        "physicalAngularDofs": s.model.segments.iter().filter_map(|s| s.joint_profile.as_ref()).map(|j| j.axes.len()).sum::<usize>(),
        "removedBilateralLockingRows": if hinges_only { joints.len() * 5 } else { 98 },
        "standaloneBodyCount": s.bodies.iter().filter(|h| s.world.multibody_joints.rigid_body_link(**h).is_none()).count(),
        "bodiesUnchangedDuringAssembly": bodies_unchanged,
        "collidersUnchangedDuringAssembly": colliders_unchanged,
        "maxInitialRelativeRotationErrorRadians": max_initial_rotation_error,
        "maxInitialRelativeTranslationErrorM": max_initial_anchor_error,
        "jointData": joints,
    });
    Ok((s, report))
}

fn damping_report(s: &Simulation) -> Value {
    let multibodies: Vec<_> = s.world.multibody_joints.multibodies().map(|m| {
        json!({"links": m.num_links(), "dofs": m.ndofs(), "generalizedDamping": m.damping().as_slice()})
    }).collect();
    json!({"equivalentToImpulseBaseline": false,
        "configuredBodyAngularDamping": s.profile.angular_damping,
        "multibodies": multibodies,
        "interpretation": "Rapier applies generalized damping (including free-root angular damping), while staged_island_solver/worker.rs skips ordinary rigid-body damping for multibody links. The configured body damping remains stored but is not applied on those links. These equations differ; this diagnostic does not compensate with forces or velocity writes.",
        "sources": ["rust/vendor/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs:273", "rust/vendor/rapier3d/src/dynamics/solver/staged_island_solver/worker.rs:810"]})
}

fn disable_motors(s: &mut Simulation) {
    for (_, joint) in s.world.impulse_joints.iter_mut() {
        joint.data.motor_axes = JointAxesMask::empty();
    }
    let handles: Vec<_> = s
        .world
        .multibody_joints
        .iter()
        .map(|(h, _, _, _)| h)
        .collect();
    for handle in handles {
        let (multibody, link) = s
            .world
            .multibody_joints
            .get_mut(handle)
            .expect("owned joint");
        multibody
            .link_mut(link)
            .expect("owned link")
            .joint
            .data
            .motor_axes = JointAxesMask::empty();
    }
}

fn linear_momentum(s: &Simulation) -> Vector {
    s.bodies
        .iter()
        .map(|h| s.world.bodies[*h].linvel() * s.world.bodies[*h].mass())
        .sum()
}

/// Read the final reduced-coordinate state without repairing or writing a body
/// velocity. This must remain separate from the unchanged body-observation gate.
fn generalized_momentum(s: &Simulation, include_body_details: bool) -> (Vector, f32, Vec<Value>) {
    let mut momentum = Vector::ZERO;
    let mut max_difference = 0.0_f32;
    let mut comparisons = Vec::new();
    for (i, handle) in s.bodies.iter().enumerate() {
        let body = &s.world.bodies[*handle];
        let velocity = if let Some(link) = s.world.multibody_joints.rigid_body_link(*handle) {
            let m = s
                .world
                .multibody_joints
                .get_multibody(link.multibody)
                .expect("owned multibody");
            let velocity = m.body_jacobian(link.id) * m.generalized_velocity();
            Vector::new(velocity[0], velocity[1], velocity[2])
        } else {
            body.linvel()
        };
        let difference = (velocity - body.linvel()).length();
        max_difference = max_difference.max(difference);
        momentum += velocity * body.mass();
        if include_body_details {
            comparisons.push(
                json!({"segment": s.model.segments[i].id, "massKg": body.mass(),
            "storedLinearVelocityMps": body.linvel().to_array(),
            "jacobianTimesGeneralizedVelocityMps": velocity.to_array(),
            "differenceMps": difference}),
            );
        }
    }
    (momentum, max_difference, comparisons)
}

fn velocity_writeback_probe(profile: &Profile, damping_disabled: bool) -> Result<Value, String> {
    let mut control_profile = profile.clone();
    if damping_disabled {
        control_profile.id = format!("{}-causal-zero-damping", profile.id);
        control_profile.angular_damping = 0.0;
    }
    let (mut s, _) = assemble_selected(&control_profile, true, false)?;
    disable_motors(&mut s);
    if damping_disabled {
        let roots: Vec<_> = s
            .world
            .multibody_joints
            .multibodies()
            .map(|m| m.root().rigid_body_handle())
            .collect();
        for root in roots {
            let index = s
                .world
                .multibody_joints
                .rigid_body_link(root)
                .expect("root")
                .multibody;
            s.world
                .multibody_joints
                .get_multibody_mut(index)
                .expect("owned multibody")
                .damping_mut()
                .fill(0.0);
        }
    }
    let root_com_offsets: Vec<_> = s.world.multibody_joints.multibodies().map(|m| {
        let body = &s.world.bodies[m.root().rigid_body_handle()];
        json!({"localCenterOfMassM": body.local_center_of_mass().to_array(), "offsetM": body.local_center_of_mass().length()})
    }).collect();
    let initial_body_speed = s
        .bodies
        .iter()
        .map(|h| {
            s.world.bodies[*h]
                .linvel()
                .length()
                .max(s.world.bodies[*h].angvel().length())
        })
        .fold(0.0_f32, f32::max);
    let initial_generalized_speed = s
        .world
        .multibody_joints
        .multibodies()
        .map(|m| m.generalized_velocity().amax())
        .fold(0.0_f32, f32::max);
    let articulated_mass: f32 = s
        .bodies
        .iter()
        .filter(|h| s.world.multibody_joints.rigid_body_link(**h).is_some())
        .map(|h| s.world.bodies[*h].mass())
        .sum();
    let before = linear_momentum(&s);
    let attempt = catch_unwind(AssertUnwindSafe(|| {
        s.world.step_with_events(&s.filter, &())
    }));
    if let Err(payload) = attempt {
        return Ok(
            json!({"integrationCompleted": false, "panic": panic_message(payload), "dampingDisabled": damping_disabled}),
        );
    }
    let expected = before + s.world.gravity * (s.total_mass() * s.profile.dt_s);
    let stored = linear_momentum(&s);
    let (generalized, max_velocity_difference, comparisons) = generalized_momentum(&s, true);
    let predicted_lag = -s.world.gravity
        * (articulated_mass * s.profile.dt_s
            / (s.profile.solver_iterations + s.profile.additional_body_iterations) as f32);
    Ok(
        json!({"scope": "Read-only first-step velocity writeback attribution; this does not replace the failed body-state gate",
        "integrationCompleted": true, "floorEnabled": false, "motorsEnabled": false,
        "dampingDisabled": damping_disabled, "profile": control_profile,
        "initialBodySpeedMaximum": initial_body_speed, "initialGeneralizedSpeedMaximum": initial_generalized_speed,
        "multibodyMassKg": articulated_mass, "multibodyRootCenterOfMassOffsets": root_com_offsets,
        "expectedMomentumNs": expected.to_array(), "storedBodyMomentumNs": stored.to_array(),
        "generalizedMomentumNs": generalized.to_array(),
        "storedBodyMomentumResidualNs": (stored - expected).length(),
        "generalizedMomentumResidualNs": (generalized - expected).length(),
        "maxStoredVsGeneralizedLinearVelocityMps": max_velocity_difference,
        "predictedOneOuterPassGravityLagNs": predicted_lag.to_array(),
        "bodyVelocityComparisons": comparisons,
        "sourceObservation": "helpers.rs updates rigid-body velocities only when !is_last_substep. worker.rs copies final solver velocities into multibody.velocities at final writeback without updating associated rigid-body velocities. J*qdot is measured separately here, without correcting any body state.",
        "sources": ["rust/vendor/rapier3d/src/dynamics/solver/staged_island_solver/helpers.rs:142", "rust/vendor/rapier3d/src/dynamics/solver/staged_island_solver/worker.rs:884"]}),
    )
}

/// Independent body ownership/anchor/limit observations, using the same hard
/// thresholds and quaternion coordinates as the production inspector.
fn passive_integrity(s: &Simulation, substep: usize) -> Result<(f32, f32), Value> {
    let mut max_anchor = 0.0_f32;
    let mut max_limit = 0.0_f32;
    for (i, segment) in s.model.segments.iter().enumerate() {
        let body = &s.world.bodies[s.bodies[i]];
        if !body.is_dynamic() || s.world.colliders[s.colliders[i]].parent() != Some(s.bodies[i]) {
            return Err(json!({"check": "ownership", "segment": segment.id, "substep": substep}));
        }
        if !body.translation().is_finite()
            || !body.rotation().is_finite()
            || !body.linvel().is_finite()
            || !body.angvel().is_finite()
        {
            return Err(json!({"check": "finite", "segment": segment.id, "substep": substep}));
        }
        if let Some(j) = &segment.joint_profile {
            let parent = s
                .model
                .segments
                .iter()
                .position(|p| Some(&p.id) == segment.parent.as_ref())
                .expect("canonical parent");
            let parent = &s.world.bodies[s.bodies[parent]];
            let separation = parent
                .position()
                .transform_point(crate::vector(j.parent_frame.anchor))
                .distance(
                    body.position()
                        .transform_point(crate::vector(j.child_frame.anchor)),
                );
            let q = measurement::coordinates(*parent.rotation(), *body.rotation(), j);
            let error = ['x', 'y', 'z']
                .iter()
                .enumerate()
                .map(|(k, c)| {
                    let e = j
                        .axes
                        .iter()
                        .find(|a| a.coordinate == *c)
                        .map_or(q[k].abs(), |a| {
                            (a.min_radians - q[k]).max(q[k] - a.max_radians).max(0.0)
                        });
                    e * e
                })
                .sum::<f32>()
                .sqrt();
            max_anchor = max_anchor.max(separation);
            max_limit = max_limit.max(error);
            for (check, measured, threshold) in [
                ("joint-anchor", separation, measurement::MAX_ANCHOR_M),
                ("joint-limit", error, measurement::MAX_LIMIT_RAD),
            ] {
                if measured > threshold {
                    return Err(
                        json!({"check": check, "segment": segment.id, "substep": substep, "measured": measured, "threshold": threshold}),
                    );
                }
            }
        }
    }
    Ok((max_anchor, max_limit))
}

fn passive_trial(
    profile: &Profile,
    hybrid: bool,
    floor_enabled: bool,
    steps: usize,
) -> Result<Value, String> {
    let mut s = if hybrid {
        assemble_selected(profile, true, floor_enabled)?.0
    } else {
        Simulation::new(0.0, profile.clone(), floor_enabled)?
    };
    disable_motors(&mut s);
    let mut completed = 0;
    let mut first_failure = None;
    let mut first_generalized_failure = None;
    let mut first_integrity_failure = None;
    let mut max_momentum_error = 0.0_f32;
    let mut max_generalized_error = 0.0_f32;
    let mut max_velocity_difference = 0.0_f32;
    // All initial velocities are zero. Retain each read-only generalized
    // observation for the next momentum delta without repairing body state.
    let mut previous_generalized = linear_momentum(&s);
    let mut samples = Vec::new();
    let mut max_anchor = 0.0_f32;
    let mut max_limit = 0.0_f32;
    let mut physics_ms = 0.0;
    for step in 0..steps {
        let before = linear_momentum(&s);
        let start = std::time::Instant::now();
        let attempt = catch_unwind(AssertUnwindSafe(|| {
            s.world.step_with_events(&s.filter, &())
        }));
        physics_ms += start.elapsed().as_secs_f64() * 1000.0;
        if let Err(payload) = attempt {
            let failure = json!({"check": "native-integration", "substep": step, "panic": panic_message(payload)});
            first_failure.get_or_insert_with(|| failure.clone());
            first_integrity_failure = Some(failure);
            break;
        }
        completed += 1;
        let mut contact = Vector::ZERO;
        for collider in &s.colliders {
            contact.y += measurement::floor_normal_impulse(&s.world, s.floor, *collider)?;
            contact += measurement::floor_tangent_impulse(&s.world, s.floor, *collider)?;
        }
        let residual = linear_momentum(&s)
            - before
            - s.world.gravity * (s.total_mass() * profile.dt_s)
            - contact;
        let error = residual.length();
        max_momentum_error = max_momentum_error.max(error);
        let (generalized, velocity_difference, _) = generalized_momentum(&s, false);
        let generalized_residual = generalized
            - previous_generalized
            - s.world.gravity * (s.total_mass() * profile.dt_s)
            - contact;
        previous_generalized = generalized;
        let generalized_error = generalized_residual.length();
        max_generalized_error = max_generalized_error.max(generalized_error);
        max_velocity_difference = max_velocity_difference.max(velocity_difference);
        samples.push(json!({"substep": step, "storedMomentumResidualNs": error,
            "generalizedMomentumResidualNs": generalized_error, "velocityDifferenceMps": velocity_difference}));
        if (!generalized_error.is_finite() || generalized_error > 0.002)
            && first_generalized_failure.is_none()
        {
            first_generalized_failure = Some(
                json!({"check": "generalized-linear-momentum", "substep": step,
                "measuredNs": generalized_error, "thresholdNs": 0.002, "residualNs": generalized_residual.to_array()}),
            );
        }
        if error > 0.002 && first_failure.is_none() {
            first_failure = Some(
                json!({"check": "linear-momentum", "substep": step, "measuredNs": error, "thresholdNs": 0.002,
                "residualNs": residual.to_array(), "floorContactImpulseNs": contact.to_array()}),
            );
        }
        match passive_integrity(&s, step) {
            Ok((anchor, limit)) => {
                max_anchor = max_anchor.max(anchor);
                max_limit = max_limit.max(limit);
            }
            Err(failure) => {
                if let Some(value) = failure["measured"].as_f64() {
                    if failure["check"] == "joint-limit" {
                        max_limit = max_limit.max(value as f32);
                    } else if failure["check"] == "joint-anchor" {
                        max_anchor = max_anchor.max(value as f32);
                    }
                }
                first_failure.get_or_insert_with(|| failure.clone());
                first_integrity_failure = Some(failure);
                break;
            }
        }
    }
    Ok(
        json!({"variant": if hybrid { "hybrid-single-axis" } else { "impulse-baseline" },
        "floorEnabled": floor_enabled, "motorsEnabled": false, "requestedSubsteps": steps, "completedSubsteps": completed,
        "passed": completed == steps && first_failure.is_none(), "firstFailure": first_failure,
        "generalizedChecksPassed": completed == steps && first_generalized_failure.is_none() && first_integrity_failure.is_none(),
        "firstGeneralizedFailure": first_generalized_failure, "firstIntegrityFailure": first_integrity_failure,
        "maxLinearMomentumResidualNs": max_momentum_error, "momentumThresholdNs": 0.002,
        "maxGeneralizedMomentumResidualNs": max_generalized_error,
        "maxStoredVsGeneralizedLinearVelocityMps": max_velocity_difference,
        "momentumSamples": samples,
        "maxAnchorSeparationM": max_anchor, "maxLimitErrorRadians": max_limit,
        "physicsTotalMs": physics_ms, "physicsMeanMsPerSubstep": physics_ms / completed.max(1) as f64,
        "scope": "Passive full-anatomy integration, ownership, separate stored-body and read-only J*qdot linear momentum including solved floor impulses, anchors and angular limits. Both momentum streams retain the 0.002 N.s threshold. Continue after a momentum failure only to gather bounded diagnostic evidence; stop for structural failure/panic. No angular-momentum, self-penetration or quiet-standing qualification."}),
    )
}

/// Hinge-only experiment. Never changes runtime selection or release admission.
pub fn diagnose_hybrid(profile: &Profile) -> Result<Value, String> {
    let (s, assembly) = assemble_selected(profile, true, true)?;
    let damping = damping_report(&s);
    let mut passive = Vec::new();
    for floor in [false, true] {
        for hybrid in [false, true] {
            passive.push(passive_trial(profile, hybrid, floor, 240)?);
        }
    }
    let writeback = [
        velocity_writeback_probe(profile, false)?,
        velocity_writeback_probe(profile, true)?,
    ];
    Ok(
        json!({"schema": 1, "scope": "Native hinge-only articulation feasibility; no default/runtime/release admission",
        "passed": false, "releaseAccepted": false, "productionEligible": false,
        "assembly": assembly, "damping": damping, "passiveTrials": passive,
        "velocityWritebackAttribution": writeback,
        "activeQuietStanding": {"status": "notRun", "reason": "The unchanged body-state linear-momentum prerequisite failed; no shared controller helpers were added."},
        "nativeCostComparison": {"status": "notQualified", "reason": "Passive-only timings, including a structural early stop with the floor, do not establish matched active quiet-standing performance."}}),
    )
}

/// Return diagnostic evidence, including an explicitly failed admission result.
/// The native panic hook may print the upstream `todo!()` location; the unwind
/// is caught here so the CLI still saves a complete, fresh report and exits 1.
pub fn diagnose(profile: &Profile) -> Result<Value, String> {
    let (mut s, assembly) = assemble(profile)?;
    let spherical = spherical_coordinate_probe(&s.model)?;
    let mut wrist_probes = Vec::new();
    for segment in &s.model.segments {
        let Some(j) = segment.joint_profile.as_ref().filter(|j| j.axes.len() == 2) else {
            continue;
        };
        let mut native = MultibodyJoint::new(anatomical_joint(j)?, false);
        let probe = catch_unwind(AssertUnwindSafe(|| {
            native.integrate(profile.dt_s, &[0.0, 0.0])
        }));
        wrist_probes.push(json!({"segment": segment.id, "axes": j.axes.iter().map(|a| a.coordinate).collect::<Vec<_>>(),
            "zeroVelocityIntegrationCompleted": probe.is_ok(), "panic": probe.err().map(panic_message)}));
    }
    // Use the canonical collision filter, real floor, and normal physics world.
    // No controller/body-state updates occur between assembly and this step.
    let attempt = catch_unwind(AssertUnwindSafe(|| {
        s.world.step_with_events(&s.filter, &())
    }));
    let complete = attempt.is_ok();
    let skipped = json!({"status": "notQualified", "passed": false,
        "reason": "Full articulation integration and anatomical coordinate semantics are prerequisites; a first-step smoke attempt cannot establish this check."});
    Ok(json!({
        "schema": 1, "scope": "Native reduced-coordinate feasibility only; unchanged canonical anatomy and default profile; no runtime or release admission",
        "passed": false, "releaseAccepted": false, "productionEligible": false,
        "assembly": assembly,
        "twoAxisProbes": wrist_probes,
        "sphericalCoordinateProbe": spherical,
        "nativeFirstStep": {"attemptedSubsteps": 1, "completedSubsteps": usize::from(complete),
            "integrationCompleted": complete, "panic": attempt.err().map(panic_message),
            "upstreamBoundary": "rust/vendor/rapier3d/src/dynamics/joint/multibody_joint/multibody_joint.rs: two-angular-DOF integrate, jacobian, and jacobian_mul_coordinates contain todo!()"},
        "qualification": {"momentum": skipped, "jointLimits": skipped, "quietStanding": skipped,
            "nativeCostComparison": skipped},
        "requiredWork": [
            "Implement and independently validate two-axis reduced-coordinate kinematics and Jacobians without adding a wrist axis or another rigid body.",
            "Implement multi-axis motor and asymmetric-limit constraints in the canonical anatomical coordinates with matching Jacobians.",
            "Then validate whole-body momentum, ownership, limits, quiet standing and equal-duration native/browser cost before any runtime proposal."
        ]
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hybrid_preserves_canonical_anatomy_and_exposes_stale_velocity_writeback() {
        let report = diagnose_hybrid(&Profile::default()).expect("hybrid diagnostic");
        let a = &report["assembly"];
        assert_eq!(a["bodyCount"], 25);
        assert_eq!(a["dynamicBodyCount"], 25);
        assert_eq!(a["anatomicalColliderCount"], 25);
        assert_eq!(a["multibodyCount"], 4);
        assert_eq!(a["linkCount"], 16);
        assert_eq!(a["standaloneBodyCount"], 9);
        assert_eq!(a["impulseJointCount"], 12);
        assert_eq!(a["multibodyJointCount"], 12);
        assert_eq!(a["physicalAngularDofs"], 46);
        assert_eq!(a["removedBilateralLockingRows"], 60);
        for key in [
            "freeRoot",
            "bodiesUnchangedDuringAssembly",
            "collidersUnchangedDuringAssembly",
        ] {
            assert_eq!(a[key], true, "{key}");
        }
        for joint in a["jointData"].as_array().unwrap() {
            assert_eq!(joint["angularDofs"], 1);
            assert_eq!(joint["exactGenericJointDataPreserved"], true);
            assert_eq!(joint["kinematic"], false);
        }
        let passive = report["passiveTrials"].as_array().unwrap();
        assert_eq!(passive[0]["passed"], true);
        assert_eq!(passive[0]["completedSubsteps"], 240);
        assert_eq!(passive[1]["passed"], false);
        assert_eq!(passive[1]["completedSubsteps"], 240);
        assert_eq!(passive[1]["generalizedChecksPassed"], true);
        assert_eq!(passive[1]["firstFailure"]["check"], "linear-momentum");
        assert_eq!(passive[3]["passed"], false);
        assert_eq!(passive[3]["generalizedChecksPassed"], false);
        assert_eq!(passive[3]["firstIntegrityFailure"]["check"], "joint-limit");
        assert_eq!(passive[3]["firstIntegrityFailure"]["segment"], "rightThigh");
        assert!(
            passive[3]["maxLimitErrorRadians"].as_f64().unwrap()
                > f64::from(measurement::MAX_LIMIT_RAD)
        );
        for probe in report["velocityWritebackAttribution"].as_array().unwrap() {
            assert_eq!(probe["integrationCompleted"], true);
            assert_eq!(probe["initialBodySpeedMaximum"], 0.0);
            assert_eq!(probe["initialGeneralizedSpeedMaximum"], 0.0);
            assert!(probe["storedBodyMomentumResidualNs"].as_f64().unwrap() > 0.04);
            assert!(probe["generalizedMomentumResidualNs"].as_f64().unwrap() < 0.002);
            assert!(
                probe["maxStoredVsGeneralizedLinearVelocityMps"]
                    .as_f64()
                    .unwrap()
                    > 0.001
            );
        }
        assert_eq!(report["damping"]["equivalentToImpulseBaseline"], false);
        assert_eq!(report["passed"], false);
        assert_eq!(report["releaseAccepted"], false);
    }

    #[test]
    fn canonical_articulation_preserves_ownership_but_cannot_complete_a_native_step() {
        let report = diagnose(&Profile::default()).expect("diagnostic report");
        let a = &report["assembly"];
        for key in [
            "bodyCount",
            "dynamicBodyCount",
            "anatomicalColliderCount",
            "linkCount",
        ] {
            assert_eq!(a[key], 25, "{key}");
        }
        assert_eq!(a["multibodyCount"], 1);
        assert_eq!(a["impulseJointCount"], 0);
        assert_eq!(a["multibodyJointCount"], 24);
        assert_eq!(a["generalizedDofsIncludingFreeRoot"], 52);
        for key in [
            "freeRoot",
            "bodiesUnchangedDuringAssembly",
            "collidersUnchangedDuringAssembly",
        ] {
            assert_eq!(a[key], true, "{key}");
        }
        assert!(
            a["maxInitialRelativeRotationErrorRadians"]
                .as_f64()
                .unwrap()
                < 1.0e-5
        );
        assert!(a["maxInitialRelativeTranslationErrorM"].as_f64().unwrap() < 1.0e-5);
        for joint in a["jointData"].as_array().unwrap() {
            assert_eq!(joint["exactGenericJointDataPreserved"], true);
            assert_eq!(joint["kinematic"], false);
        }
        let wrists = report["twoAxisProbes"].as_array().unwrap();
        assert_eq!(wrists.len(), 2);
        assert!(
            wrists
                .iter()
                .all(|p| p["zeroVelocityIntegrationCompleted"] == false)
        );
        assert_eq!(report["nativeFirstStep"]["integrationCompleted"], false);
        assert_eq!(report["nativeFirstStep"]["completedSubsteps"], 0);
        assert_eq!(report["passed"], false);
        assert_eq!(report["releaseAccepted"], false);
    }

    #[test]
    fn spherical_native_coordinates_do_not_preserve_anatomical_limit_semantics() {
        let report = spherical_coordinate_probe(&lh_model::Model::canonical().unwrap()).unwrap();
        assert_eq!(report["sameCoordinateSemantics"], false);
        assert!(report["maxCoordinateMismatchRadians"].as_f64().unwrap() > 1.0e-4);
    }
}
