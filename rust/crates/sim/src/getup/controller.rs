//! Opt-in pretrained recovery. One exact-state teacher branch, then source SLOW.
//! Branch integration is bounded, counted and never copied back to the real body.
use super::{actor, reference};
use crate::*;

pub const TEACHER_POWER: f32 = 0.663420431;
pub const REFERENCE_SPEED: f64 = 0.5;
pub const MAX_TEACHER_TICKS: usize = 250;
pub const TEACHER_DT: f32 = 0.005;

#[derive(Serialize)]
pub struct TeacherFrame {
    time_s: f64,
    observation: Vec<f32>,
    action: [f32; 21],
    target: [f32; 25],
    snapshot: Snapshot,
    before: Vec<lh_model::Pose>,
}
#[derive(Serialize)]
pub struct StudentDecision {
    native_substep: u64,
    observation: Vec<f32>,
    residual: [f32; 21],
    controls: [f32; 21],
    holding: bool,
    before: Vec<lh_model::Pose>,
}
#[derive(Serialize)]
pub struct Controller {
    pub status: lh_contracts::GetUpPolicyObservation,
    pub started_substep: Option<u64>,
    impact_substep: Option<u64>,
    reference: Vec<[f32; 25]>,
    teacher_frames: Vec<TeacherFrame>,
    teacher_initial: Option<Snapshot>,
    teacher_failure: Option<measurement::Failure>,
    decision_observation: Vec<f32>,
    student_decisions: Vec<StudentDecision>,
    residual: [f32; 21],
    reference_pose: [f32; 21],
    holding: bool,
    last_decision: Option<u64>,
    stable_substeps: u64,
    stable_feet: Option<[Vec3; 2]>,
}
impl Controller {
    pub fn new(model: &Model) -> Self {
        let _ = (actor::teacher(), actor::student());
        Self {
            status: lh_contracts::GetUpPolicyObservation {
                phase: "awaiting-physical-fall".into(),
                ..Default::default()
            },
            started_substep: None,
            impact_substep: None,
            reference: Vec::new(),
            teacher_frames: Vec::new(),
            teacher_initial: None,
            teacher_failure: None,
            decision_observation: Vec::new(),
            student_decisions: Vec::new(),
            residual: [0.0; 21],
            reference_pose: standing_target(model)[..21].try_into().unwrap(),
            holding: true,
            last_decision: None,
            stable_substeps: 0,
            stable_feet: None,
        }
    }
    pub fn impact(&mut self, substep: u64) {
        if self.impact_substep.is_none() {
            self.impact_substep = Some(substep);
        }
    }
    pub fn command(&mut self, s: &Simulation, physical: &[f32; 67]) -> Result<[f32; 21], String> {
        if self
            .impact_substep
            .or(self.started_substep)
            .is_some_and(|n| s.substeps_completed - n >= 20000)
            && !self.status.prototype_standing_passed
        {
            self.status.refusal = Some("25-second physical recovery deadline reached".into());
            return Err(self.status.refusal.clone().unwrap());
        }
        let mut new_decision = false;
        if self.started_substep.is_none()
            && s.motion == Motion::Fallen
            && s.striker
                .as_ref()
                .is_none_or(|p| p.snapshot(&s.world).available)
        {
            self.started_substep = Some(s.substeps_completed);
            self.status.phase = "teacher-reference".into();
            self.status.started_at_s = Some(s.substeps_completed as f64 / 800.0);
            if let Err(error) = self.generate_reference(s) {
                self.status.phase = "reference-rejected".into();
                self.status.refusal = Some(error.clone());
                return Err(error);
            }
            self.status.phase = "slow-following".into();
        }
        if let Some(start) = self.started_substep {
            if self.reference.is_empty() {
                return Err("GetUp reference unavailable".into());
            }
            let index = ((s.substeps_completed - start) / 20) as usize;
            if self.last_decision != Some(index as u64) {
                let end = self.reference.len() - 1;
                let now = &self.reference[index.min(end)];
                let ahead = &self.reference[(index + 5).min(end)];
                let input = super::slow_observation(physical, now, ahead);
                self.residual = actor::student().infer(&input)?.action;
                self.reference_pose.copy_from_slice(&now[..21]);
                self.holding = index >= end;
                self.status.phase = if self.holding {
                    "slow-holding"
                } else {
                    "slow-following"
                }
                .into();
                self.decision_observation = input.to_vec();
                self.last_decision = Some(index as u64);
                new_decision = true;
                self.status.policy_decisions += 1;
                self.status.reference_index = index.min(end) as u32;
                self.status.reference_tracking_error_rad = physical[..21]
                    .iter()
                    .zip(&now[..21])
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f32::max);
                self.status.reference_com_error_m = s.center_of_mass().y - now[21];
            }
        }
        let q: &[f32; 21] = physical[..21].try_into().unwrap();
        let dq: &[f32; 21] = physical[46..67].try_into().unwrap();
        let controls = super::slow_controls(
            &s.model,
            &self.reference_pose,
            &self.residual,
            q,
            dq,
            self.holding,
        )?;
        if new_decision {
            if self.student_decisions.len() >= 1100 {
                return Err("GetUp decision archive bound reached".into());
            }
            self.student_decisions.push(StudentDecision {
                native_substep: s.substeps_completed,
                observation: self.decision_observation.clone(),
                residual: self.residual,
                controls,
                holding: self.holding,
                before: s.snapshot().segments,
            });
        }
        Ok(controls)
    }
    fn generate_reference(&mut self, real: &Simulation) -> Result<(), String> {
        let mut shadow = fork(real)?;
        self.teacher_initial = Some(real.snapshot());
        self.status.reference_seed_matches_real = serde_json::to_value(&real.snapshot().segments)
            .unwrap()
            == serde_json::to_value(&shadow.snapshot().segments).unwrap()
            && super::joint_state(real)? == super::joint_state(&shadow)?;
        if !self.status.reference_seed_matches_real {
            return Err("GetUp branch changed its initial state".into());
        }
        shadow.world.integration_parameters.dt = TEACHER_DT;
        shadow.profile.dt_s = TEACHER_DT;
        shadow
            .world
            .integration_parameters
            .num_internal_pgs_iterations = real.profile.fall_internal_pgs_iterations;
        let mut finish_after_step = false;
        let mut usable = false;
        let mut knots = Vec::new();
        for index in 0..MAX_TEACHER_TICKS {
            let physical = super::physical_observation(&shadow)?;
            let before = shadow.snapshot().segments;
            let mut input = [0.0; 68];
            input[..67].copy_from_slice(&physical);
            input[67] = TEACHER_POWER;
            let action = actor::teacher().infer(&input)?.action;
            for substep in 0..5 {
                for h in &shadow.bodies {
                    shadow.world.bodies[*h].reset_forces(false);
                    shadow.world.bodies[*h].reset_torques(false);
                }
                super::apply_controls(&mut shadow, &action)?;
                shadow.world.step_with_events(&shadow.filter, &());
                self.status.forecast_substeps += 1;
                shadow.substeps_completed += 1;
                if let Err(failure) = measurement::inspect(&mut shadow, substep) {
                    self.teacher_failure = Some(failure.clone());
                    self.record_teacher(&shadow, input[..67].to_vec(), action, before)?;
                    return Err(format!(
                        "Teacher Rapier integrity failure: {} {} measured={} limit={}",
                        failure.check, failure.offending, failure.measured, failure.threshold
                    ));
                }
            }
            shadow.tick = index as u64 + 1;
            let target = self.record_teacher(&shadow, physical.to_vec(), action, before)?;
            knots.push(target);
            self.status.teacher_decisions = index as u32 + 1;
            let head = super::physical_observation(&shadow)?[21];
            self.status.teacher_max_head_height_m = self.status.teacher_max_head_height_m.max(head);
            if finish_after_step {
                usable = true;
                break;
            }
            finish_after_step = head > 1.4;
        }
        self.status.teacher_terminal_reached = usable;
        if !usable {
            return Err("WEAK reference did not reach source head-height terminal within 250 decisions; no retry".into());
        }
        self.reference = reference::interpolate(&knots, REFERENCE_SPEED)?;
        self.status.reference_frames = self.reference.len() as u32;
        self.reference.push(standing_target(&real.model));
        Ok(())
    }
    fn record_teacher(
        &mut self,
        s: &Simulation,
        observation: Vec<f32>,
        action: [f32; 21],
        before: Vec<lh_model::Pose>,
    ) -> Result<[f32; 25], String> {
        let physical = super::physical_observation(s)?;
        let mut target = [0.0; 25];
        target[..21].copy_from_slice(&physical[..21]);
        target[21] = s.center_of_mass().y;
        target[22..25].copy_from_slice(&physical[34..37]);
        let time = self.status.forecast_substeps as f64 * 0.005;
        let mut snapshot = s.snapshot();
        snapshot.simulation_time_s = time;
        self.status.teacher_max_head_height_m =
            self.status.teacher_max_head_height_m.max(physical[21]);
        self.teacher_frames.push(TeacherFrame {
            time_s: time,
            observation,
            action,
            target,
            snapshot,
            before,
        });
        Ok(target)
    }
    pub fn observe(&mut self, s: &mut Simulation, loads: &[f32; 25]) {
        if self.started_substep.is_none() {
            return;
        }
        let physical = match super::physical_observation(s) {
            Ok(p) => p,
            Err(_) => return,
        };
        self.status.actual_head_height_m = physical[21];
        let contract = &recovery_contract::contract().recovery_acceptance;
        let ids = ["left_foot", "right_foot"]
            .map(|id| s.model.segments.iter().position(|p| p.id == id).unwrap());
        let feet = ids.map(|i| vec3(s.world.bodies[s.bodies[i]].translation()));
        let foot_load: f32 = ids.iter().map(|i| loads[*i]).sum();
        let only_feet = loads
            .iter()
            .enumerate()
            .all(|(i, l)| ids.contains(&i) || *l < 3.0);
        let mut points = Vec::new();
        for i in ids {
            if let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) {
                for manifold in pair.solver_manifolds() {
                    let sign = if pair.collider2 == s.colliders[i] {
                        1.0
                    } else {
                        -1.0
                    };
                    if manifold.data.normal.y * sign < 0.65 {
                        continue;
                    }
                    for contact in &manifold.data.solver_contacts {
                        let point = &manifold.points[contact.contact_indices()[0] as usize];
                        if point.data.impulse > 0.0 {
                            let (a, b) = manifold
                                .data
                                .solver_contact_world_points(contact, &s.world.bodies);
                            let p = if pair.collider2 == s.colliders[i] {
                                b
                            } else {
                                a
                            };
                            points.push([p.x, p.z]);
                        }
                    }
                }
            }
        }
        let com = s.center_of_mass();
        let vel = Vector::new(physical[38], physical[39], physical[37]);
        let capture = com + vel * (com.y.max(0.0) / 9.81).sqrt();
        let balanced = stepping::inside_support(points.clone(), [com.x, com.z], 0.003)
            && stepping::inside_support(points, [capture.x, capture.z], 0.003);
        let mut stable = physical[21] > 1.4
            && physical[36] >= contract.stable_up_dot
            && ids.iter().all(|i| {
                loads[*i] >= contract.minimum_load_n && s.integrated_support_substeps[*i] >= 40
            })
            && foot_load >= 0.8 * s.total_mass() * 9.81
            && only_feet
            && balanced
            && super::current_twists(s).is_ok_and(|twists| {
                twists.iter().all(|(v, w)| {
                    v.length() <= contract.stable_linear_mps
                        && w.length() <= contract.stable_angular_radps
                })
            });
        if stable {
            let anchors = self.stable_feet.get_or_insert(feet);
            stable = anchors
                .iter()
                .zip(&feet)
                .all(|(a, b)| vector(*a).distance(vector(*b)) <= 0.01);
        }
        if !stable {
            self.stable_feet = None;
        }
        self.stable_substeps = if stable { self.stable_substeps + 1 } else { 0 };
        self.status.stable_standing_s = self.stable_substeps as f64 / 800.0;
        let elapsed = self
            .impact_substep
            .map(|i| (s.substeps_completed - i) as f64 / 800.0);
        self.status.since_impact_s = elapsed;
        if !self.status.prototype_standing_passed
            && self.stable_substeps >= 800
            && elapsed.is_some_and(|t| t <= 25.0)
        {
            self.status.prototype_standing_passed = true;
            s.counters.recoveries += 1;
        }
        s.motion = if self.status.prototype_standing_passed && stable {
            Motion::Upright
        } else {
            Motion::Recovering
        };
    }
}

pub fn standing_target(model: &Model) -> [f32; 25] {
    let mut out = [0.0; 25];
    for (i, q) in out[..21].iter_mut().enumerate() {
        *q = model.reference_metadata["standingPose"][i]
            .as_f64()
            .unwrap() as f32;
    }
    // The source terminal observation explicitly uses 1.4 m CoM and vertical Z.
    out[21] = 1.4;
    out[24] = 1.0;
    out
}

fn fork(real: &Simulation) -> Result<Simulation, String> {
    if real.environment.is_some()
        || !real.world.colliders[real.floor].is_enabled()
        || real.grab.is_some()
    {
        return Err("GetUp prototype requires a free body on the protocol floor".into());
    }
    let mut profile = real.profile.clone();
    profile.getup_policy_enabled = false;
    let mut shadow = Simulation::new(0.0, profile, true)?;
    let w = &real.world;
    shadow.world = PhysicsWorld {
        gravity: w.gravity,
        integration_parameters: w.integration_parameters,
        physics_pipeline: PhysicsPipeline::new(),
        islands: w.islands.clone(),
        broad_phase: w.broad_phase.clone(),
        narrow_phase: w.narrow_phase.clone(),
        bodies: w.bodies.clone(),
        colliders: w.colliders.clone(),
        impulse_joints: w.impulse_joints.clone(),
        multibody_joints: w.multibody_joints.clone(),
        ccd_solver: CCDSolver::new(),
    };
    shadow.bodies = real.bodies.clone();
    shadow.colliders = real.colliders.clone();
    shadow.joints = real.joints.clone();
    shadow.floor = real.floor;
    shadow.filter = Filter {
        exclusions: real.filter.exclusions.clone(),
    };
    shadow.striker = real.striker.clone();
    shadow.contacts = real.contacts.clone();
    shadow.motion = real.motion;
    shadow.diagnostics = Diagnostics::default();
    Ok(shadow)
}

#[cfg(test)]
mod tests {
    #[test]
    fn exact_state_branch_is_independent_and_never_steps_during_construction() {
        let real = crate::Simulation::new_protocol(0.3, crate::Profile::default(), true).unwrap();
        let before = serde_json::to_value(real.snapshot()).unwrap();
        let mut shadow = super::fork(&real).unwrap();
        assert_eq!(
            serde_json::to_value(&shadow.snapshot().segments).unwrap(),
            before["segments"]
        );
        assert_eq!(
            super::super::joint_state(&real).unwrap(),
            super::super::joint_state(&shadow).unwrap()
        );
        assert_eq!(
            serde_json::to_value(&real.world).unwrap(),
            serde_json::to_value(&shadow.world).unwrap()
        );
        assert_eq!(shadow.substeps_completed, 0);
        shadow.world.bodies[shadow.bodies[0]].add_torque(crate::Vector::X, true);
        assert_eq!(serde_json::to_value(real.snapshot()).unwrap(), before);
        assert!(shadow.getup_controller.is_none());
    }
}
