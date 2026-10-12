//! Opt-in command owner for the frozen transition, shared by real and copied worlds.
//! Native prediction selection lives outside this controller, so previews cannot recurse.
use super::{
    coordinated_runtime as runtime, geometric_tasks as tasks, native_observation, support_state,
};
use crate::{Simulation, recovery, recovery::trajectory_program::CompiledProgram};
use rapier3d::prelude::*;
use serde::Deserialize;
use serde_json::{Value, json};

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct ReferenceTasks {
    pub rows: Vec<ReferenceRow>,
    pub material_targets: Vec<MaterialTarget>,
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct MaterialTarget {
    pub segment_index: usize,
    pub local_point_m: [f64; 3],
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct ReferenceRow {
    pub substep: u64,
    pub com_position_m: [f64; 3],
    pub com_velocity_mps: [f64; 3],
    pub feet: Vec<FootTarget>,
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct FootTarget {
    pub side: String,
    pub hindfoot_rotation_xyzw: [f64; 4],
    pub forefoot_rotation_xyzw: [f64; 4],
}

pub(crate) const PARAMETERS: &str = r#"{"version":"coordinated-transition-v1","controlHz":60,"previewHz":10,"previewSteps":480,"candidates":[0.0,0.5,1.0,1.5],"comPositionGainPerS":2.0,"comSpeedLimitMps":0.15,"footRotationGainPerS":2.0,"posturePositionGainPerS":2.0,"postureWeight":0.25,"regularization":0.05,"hardMaterialVelocityToleranceMps":0.005,"materialMatchToleranceM":0.012,"maximumTargetOffsetRad":0.12,"iterations":512,"supportProof":"original thresholds unchanged"}"#;

/// Initial fixture construction only. No settling, integration, or later reset.
pub(crate) fn scene(
    program: recovery::trajectory_program::Program,
    reference: ReferenceTasks,
    enabled: bool,
    heading: f32,
) -> Result<Simulation, String> {
    if ![0., std::f32::consts::PI / 3.].contains(&heading) {
        return Err("Unfrozen fixture heading".into());
    }
    let fixtures: recovery::FixtureSet =
        serde_json::from_str(recovery::FIXTURES).map_err(|e| e.to_string())?;
    let fixture = fixtures
        .cases
        .into_iter()
        .find(|f| f.id == "landed-crouch-left")
        .ok_or("Missing frozen fixture")?;
    let mut poses = fixture.poses;
    let rotate = Rotation::from_rotation_y(heading - fixture.heading);
    for pose in &mut poses {
        pose.position = crate::vec3(rotate * crate::vector(pose.position));
        pose.rotation = crate::quat(rotate * crate::rotation(pose.rotation));
        pose.linear_velocity = crate::vec3(rotate * crate::vector(pose.linear_velocity));
        pose.angular_velocity = crate::vec3(rotate * crate::vector(pose.angular_velocity));
    }
    let mut sim = Simulation::build(
        heading,
        crate::recovery_contract::pinned_profile(),
        true,
        None,
        Some(&poses),
    )?;
    let program = program.compile(&sim.model)?;
    sim.coordination = Some(Controller::new(&sim, program, reference, enabled)?);
    Ok(sim)
}

#[derive(Clone)]
pub(crate) struct Controller {
    program: CompiledProgram,
    reference: ReferenceTasks,
    pub enabled: bool,
    pub feedback_scale: f64,
    pub memory: runtime::Memory,
    pub patches: support_state::MaterialTracker,
    pub start: u64,
    previous_step: Option<u64>,
    best_progress: f32,
    stalled_s: f32,
    unsupported_s: f32,
    maximum_cap: f64,
    last: Value,
}

impl Controller {
    pub(crate) fn new(
        sim: &Simulation,
        program: CompiledProgram,
        reference: ReferenceTasks,
        enabled: bool,
    ) -> Result<Self, String> {
        if reference.rows.len() != program.duration_substeps() as usize / 4 + 1
            || reference
                .rows
                .iter()
                .enumerate()
                .any(|(i, r)| r.substep != i as u64 * 4)
            || reference
                .material_targets
                .iter()
                .any(|t| t.segment_index >= 25 || t.local_point_m.iter().any(|v| !v.is_finite()))
        {
            return Err(
                "Reference tasks must cover the entire compiled program on the 60 Hz grid".into(),
            );
        }
        for row in &reference.rows {
            if row
                .com_position_m
                .iter()
                .chain(row.com_velocity_mps.iter())
                .any(|v| !v.is_finite())
                || row.feet.len() != 2
                || row.feet[0].side != "left"
                || row.feet[1].side != "right"
            {
                return Err("Invalid reference body tasks".into());
            }
            for foot in &row.feet {
                for q in [foot.hindfoot_rotation_xyzw, foot.forefoot_rotation_xyzw] {
                    if q.iter().any(|v| !v.is_finite())
                        || (q.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-5
                    {
                        return Err("Reference foot quaternion must be finite and unit".into());
                    }
                }
            }
        }
        let expected = [18, 19, 23, 24];
        if reference.material_targets.len() != 4
            || expected.iter().any(|i| {
                reference
                    .material_targets
                    .iter()
                    .filter(|t| t.segment_index == *i)
                    .count()
                    != 1
            })
        {
            return Err(
                "Reference must define the four native foot material targets exactly once".into(),
            );
        }
        let q = program.command(0).positions;
        let measured = recovery::support_probe::offline_trajectory::coordinates(sim)?;
        if q.iter()
            .flatten()
            .zip(measured.iter().flatten())
            .any(|(a, b)| (a - b).abs() > 1e-6)
        {
            return Err("Initial command differs from the actual native fixture".into());
        }
        Ok(Self {
            program,
            reference,
            enabled,
            feedback_scale: 1.,
            memory: runtime::initialize(&sim.model, q, sim.substeps_completed, 0)?,
            patches: Default::default(),
            start: sim.substeps_completed,
            previous_step: None,
            best_progress: 0.,
            stalled_s: 0.,
            unsupported_s: 0.,
            maximum_cap: 0.,
            last: Value::Null,
        })
    }
    pub(crate) fn evidence(&self) -> Value {
        json!({"enabled":self.enabled,"feedbackScale":self.feedback_scale,"start":self.start,"previousStep":self.previous_step,
            "q":self.memory.previous,"nextSubstep":self.memory.next_substep,"rates":self.memory.rates.to_vec(),
            "patches":self.patches,"bestProgress":self.best_progress,"stalledS":self.stalled_s,"unsupportedS":self.unsupported_s,"maximumCapNm":self.maximum_cap,"last":self.last})
    }
    pub(crate) fn configure(&mut self, sim: &mut Simulation) -> Result<(), String> {
        if ![0., 0.5, 1., 1.5].contains(&self.feedback_scale) {
            return Err("Unfrozen coordination proposal".into());
        }
        let step = sim
            .substeps_completed
            .checked_sub(self.start)
            .ok_or("Coordination clock rewind")?;
        if self.previous_step.is_some_and(|old| step != old + 1) || step >= 6000 {
            return Err("Coordination clock discontinuity/deadline".into());
        }
        let nominal = self.program.command(step as u32);
        let measured = recovery::support_probe::offline_trajectory::coordinates(sim)?;
        let progress =
            recovery::support_probe::getup_ab::path_progress(sim, &self.program, &measured);
        if progress > self.best_progress + 0.005 {
            self.best_progress = progress;
            self.stalled_s = 0.;
        } else {
            self.stalled_s += sim.profile.dt_s;
        }
        self.unsupported_s = if recovery::has_persistent_support(sim) {
            0.
        } else {
            self.unsupported_s + sim.profile.dt_s
        };
        let acceptance = &crate::recovery_contract::contract().recovery_acceptance;
        if self.unsupported_s as f64 >= acceptance.support_loss_s {
            return Err("Original support-loss duration".into());
        }
        if self.stalled_s as f64 >= acceptance.stall_s {
            return Err("Original measured joint-path stagnation duration".into());
        }
        let observation = native_observation::observe(sim, self.start, &[])?;
        let mut solve_report = Value::Null;
        if self.enabled && step % 4 == 0 {
            // Acquire only native contacts currently observed and loaded. Each
            // retained material point remains the same until an explicit pivot replacement.
            for patch in &mut self.patches.active {
                patch.observe(&observation, 0.012)?;
            }
            for segment in &observation.segments {
                if !matches!(
                    segment.role,
                    support_state::SegmentRole::Hindfoot | support_state::SegmentRole::Forefoot
                ) || segment.loaded_substeps < 12
                {
                    continue;
                }
                if !self
                    .patches
                    .active
                    .iter()
                    .any(|p| p.segment == segment.segment)
                {
                    let planned = self
                        .reference
                        .material_targets
                        .iter()
                        .find(|t| t.segment_index == segment.segment);
                    let selected = segment
                        .contacts
                        .iter()
                        .enumerate()
                        .filter(|(_, c)| c.current_upward_load_n >= 3.)
                        .min_by(|(_, a), (_, b)| {
                            let score = |p: [f64; 3]| {
                                planned.map_or(p[2], |target| {
                                    p.iter()
                                        .zip(target.local_point_m)
                                        .map(|(a, b)| (a - b).powi(2))
                                        .sum()
                                })
                            };
                            score(a.local_point_m).total_cmp(&score(b.local_point_m))
                        });
                    if let Some((contact, _)) = selected {
                        self.patches.acquire(
                            &observation,
                            segment.segment,
                            contact,
                            support_state::ContactMode::Hold,
                        )?;
                    }
                }
            }
            // This frozen family rotates about retained forefoot material
            // points, then adds heels. It does not plan a material replacement.
            if self
                .patches
                .active
                .iter()
                .any(|p| !p.current_contact_present)
            {
                return Err(
                    "Retained material support vanished; this route has no replacement".into(),
                );
            }
            let reference =
                &self.reference.rows[(step / 4).min(self.reference.rows.len() as u64 - 1) as usize];
            let rotated = |a: [f64; 3]| sim.heading * Vector::from_array(a.map(|v| v as f32));
            let com_ref = rotated(reference.com_position_m);
            let v_ref = rotated(reference.com_velocity_mps);
            let error = com_ref - sim.center_of_mass();
            let mut com_goal = v_ref + error * (2. * self.feedback_scale as f32);
            // CoM vertical speed is a task too; this bound is intent, not a relaxed diagnostic.
            com_goal = com_goal.clamp_length_max(0.15);
            let mut posture = nominal.clone();
            for i in 0..25 {
                for k in 0..3 {
                    posture.velocities[i][k] = (nominal.velocities[i][k]
                        + 2. * (nominal.positions[i][k] - self.memory.previous[i][k]))
                        .clamp(-1., 1.);
                }
            }
            let mut goals = tasks::TaskGoals {
                nominal: posture,
                com_velocity_mps: com_goal.to_array().map(f64::from),
                com_weight: self.feedback_scale,
                point_tasks: vec![],
                orientation_tasks: vec![],
                posture_weight: 0.25,
                regularization: 0.05,
                retained_velocity_tolerance_mps: 0.005,
            };
            for foot in &reference.feet {
                for (suffix, target) in [
                    ("Foot", foot.hindfoot_rotation_xyzw),
                    ("Forefoot", foot.forefoot_rotation_xyzw),
                ] {
                    let segment = recovery::index(sim, &format!("{}{suffix}", foot.side));
                    let desired = sim.heading * Rotation::from_array(target.map(|v| v as f32));
                    let actual = *sim.world.bodies[sim.bodies[segment]].rotation();
                    let mut delta = desired * actual.inverse();
                    if delta.w < 0. {
                        delta = -delta;
                    }
                    let velocity = delta.to_scaled_axis() * (2. * self.feedback_scale as f32);
                    goals.orientation_tasks.push(tasks::OrientationGoal {
                        segment,
                        world_angular_velocity_radps: velocity.to_array().map(f64::from),
                        weight: self.feedback_scale,
                    });
                }
            }
            let layout = runtime::axes(&sim.model)?;
            if self.patches.active.is_empty() {
                // The initial fixture is 1 mm above the floor. This acquisition
                // prefix is reference tracking only, never a claimed support.
                // Refuse after the unchanged original unsupported duration.
                if step as f64 * sim.profile.dt_s as f64
                    >= crate::recovery_contract::contract()
                        .recovery_acceptance
                        .support_loss_s
                {
                    return Err(
                        "No native retained support acquired before original support-loss deadline"
                            .into(),
                    );
                }
                let next = self.program.command((step + 4) as u32);
                for (i, a) in layout.iter().enumerate() {
                    self.memory.rates[i] = ((next.positions[a.segment][a.coordinate]
                        - self.memory.previous[a.segment][a.coordinate])
                        as f64
                        / (4. * sim.profile.dt_s as f64))
                        .clamp(-1., 1.);
                }
                solve_report =
                    json!({"phase":"unloaded acquisition prefix","supportCertified":false});
            } else {
                let mut problem = tasks::assemble(sim, &self.patches.active, &goals)?;
                for (i, a) in layout.iter().enumerate() {
                    let previous = self.memory.previous[a.segment][a.coordinate] as f64;
                    let target = nominal.positions[a.segment][a.coordinate] as f64;
                    let dt = 4. * sim.profile.dt_s as f64;
                    problem.lower[i] = problem.lower[i]
                        .max(((target - 0.12).max(a.minimum as f64) - previous) / dt);
                    problem.upper[i] = problem.upper[i]
                        .min(((target + 0.12).min(a.maximum as f64) - previous) / dt);
                }
                let solution = problem.solve()?;
                self.memory.rates = solution.rates;
                let mut check = self.memory.clone();
                let end = sim
                    .substeps_completed
                    .checked_add(4)
                    .ok_or("Owner command clock overflow")?;
                for native in sim.substeps_completed..end {
                    let (emitted, next) = runtime::command(&sim.model, &check, native)?;
                    let mut rates = [0.; 46];
                    for (i, a) in layout.iter().enumerate() {
                        rates[i] = emitted.velocities[a.segment][a.coordinate] as f64;
                    }
                    problem.verify_rates(&rates)?;
                    check = next;
                }
                solve_report = json!({"iterations":solution.iterations,"hardError":solution.maximum_constraint_error,"projectedGradientResidual":solution.projected_gradient_residual});
            }
        }
        let command = if self.enabled && !self.patches.active.is_empty() {
            let (command, next) =
                runtime::command(&sim.model, &self.memory, sim.substeps_completed)?;
            self.memory = next;
            command
        } else {
            // Same reference; coherent finite-difference velocity on the exact
            // command grid, no corrective feedback or different cap allocation.
            let mut command = nominal.clone();
            for i in 0..25 {
                for k in 0..3 {
                    command.velocities[i][k] =
                        (command.positions[i][k] - self.memory.previous[i][k]) / sim.profile.dt_s;
                }
            }
            self.memory.previous = command.positions;
            self.memory.next_substep = sim
                .substeps_completed
                .checked_add(1)
                .ok_or("Owner command clock overflow")?;
            command
        };
        recovery::trajectory_program::validate_command(&sim.model, &command)?;
        if sim.recovery.is_none() {
            if self.previous_step.is_some() {
                return Err("Original recovery observer refused; no restart".into());
            }
            sim.recovery = Some(recovery::Controller::new(sim)?);
            sim.motion = lh_contracts::Motion::Recovering;
            sim.stepping = None;
        }
        let anchors = ["left", "right"].map(|side| {
            recovery::planted_contact(
                sim,
                &[
                    recovery::index(sim, &format!("{side}Foot")),
                    recovery::index(sim, &format!("{side}Forefoot")),
                ],
            )
        });
        let controller = sim.recovery.as_mut().unwrap();
        controller.status.phase = if step >= u64::from(self.program.duration_substeps()) {
            recovery::Phase::Stand
        } else {
            recovery::Phase::Brace
        };
        controller.status.elapsed_s = step as f32 * sim.profile.dt_s;
        controller.status.phase_time_s = controller.status.elapsed_s;
        controller.status.unsupported_s = self.unsupported_s;
        controller.status.stalled_s = self.stalled_s;
        controller.status.progress = progress;
        controller.foot_anchors = [anchors[0].clone()?, anchors[1].clone()?];
        controller.targets = command.positions.to_vec();
        controller.velocities = command.velocities.to_vec();
        sim.targets = command.positions.to_vec();
        let (cap, ratio, requests) = recovery::support_probe::offline_trajectory::install_targets(
            sim,
            &command.positions,
            &command.velocities,
            recovery::support_probe::offline_trajectory::Allocation::OwnedRequests,
        )?;
        self.maximum_cap = self.maximum_cap.max(cap);
        self.previous_step = Some(step);
        self.last = json!({"commandSubstep":step,"positions":command.positions,"velocities":command.velocities,"solver":solve_report,
            "capSumNm":cap,"anatomicalCapRatio":ratio,"requests":requests,"terminalRequested":step>=self.program.duration_substeps() as u64});
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn inputs() -> (recovery::trajectory_program::Program, ReferenceTasks) {
        let dir = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-prototype-2026-10-12/"
        );
        (
            serde_json::from_str(
                &std::fs::read_to_string(format!("{dir}continuation-candidate-program.json"))
                    .unwrap(),
            )
            .unwrap(),
            serde_json::from_str(
                &std::fs::read_to_string(format!("{dir}continuation-tasks.json")).unwrap(),
            )
            .unwrap(),
        )
    }
    #[test]
    fn shared_acquisition_prefix_installs_identical_native_motors_without_integration() {
        let (program, tasks) = inputs();
        let mut a = scene(program.clone(), tasks.clone(), false, 0.).unwrap();
        let mut b = scene(program, tasks, true, 0.).unwrap();
        let bodies = serde_json::to_value(&a.world.bodies).unwrap();
        a.configure_motors().unwrap();
        b.configure_motors().unwrap();
        assert_eq!(
            serde_json::to_value(&a.world.impulse_joints).unwrap(),
            serde_json::to_value(&b.world.impulse_joints).unwrap()
        );
        assert_eq!(serde_json::to_value(&a.world.bodies).unwrap(), bodies);
        assert_eq!(a.substeps_completed, 0);
        assert_eq!(b.substeps_completed, 0);
        assert!(a.coordination.as_ref().unwrap().maximum_cap <= 760.);
        assert!(!crate::Profile::default().recovery_enabled);
    }
    #[test]
    fn malformed_reference_is_refused_before_any_step() {
        let (program, mut tasks) = inputs();
        tasks.rows[3].feet[0].side = "unknown".into();
        assert!(scene(program, tasks, true, 0.).is_err());
    }
    #[test]
    fn handoff_releases_owner_and_preserves_original_measured_gate_without_integration() {
        let (program, tasks) = inputs();
        let mut sim = scene(program, tasks, true, 0.).unwrap();
        sim.configure_motors().unwrap();
        sim.recovery.as_mut().unwrap().status.phase = recovery::Phase::Handoff;
        assert_eq!(
            sim.configure_motors().unwrap_err(),
            "Missing measured handoff"
        );
        assert!(sim.coordination.is_none());
        assert_eq!(sim.substeps_completed, 0);
    }
}
