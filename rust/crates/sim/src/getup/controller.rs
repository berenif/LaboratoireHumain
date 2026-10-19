//! Opt-in pretrained recovery. One exact-state teacher branch, then source SLOW.
//! Branch integration is bounded, counted and never copied back to the real body.
use super::{actor, reference};
use crate::*;

pub const TEACHER_POWER: f32 = 0.663420431;
pub const REFERENCE_SPEED: f64 = 0.5;
pub const MAX_TEACHER_TICKS: usize = 250;
const POLICY_DT: f32 = 0.025;

fn policy_substeps(native_dt: f32) -> Result<u32, String> {
    let count = (POLICY_DT / native_dt).round();
    if !native_dt.is_finite()
        || native_dt <= 0.0
        || !(1.0..=20.0).contains(&count)
        || (count * native_dt - POLICY_DT).abs() > 1.0e-7
    {
        return Err("Native step must divide the 40 Hz GetUp policy period within 20 substeps".into());
    }
    Ok(count as u32)
}

#[derive(Serialize, Deserialize)]
pub struct TeacherFrame {
    time_s: f64,
    observation: Vec<f32>,
    action: [f32; 21],
    target: [f32; 25],
    snapshot: Snapshot,
    before: Vec<lh_model::Pose>,
}
#[derive(Serialize, Deserialize)]
pub struct StudentDecision {
    native_substep: u64,
    observation: Vec<f32>,
    residual: [f32; 21],
    controls: [f32; 21],
    holding: bool,
    before: Vec<lh_model::Pose>,
}

#[derive(Serialize, Deserialize)]
struct TerminalBalance {
    activated_native_substep: u64,
    reference: super::supported_standing::StandingReference,
    last_slow_controls: [f32; 21],
    first_balance_controls: Option<[f32; 21]>,
    first_actual_control_jump_max: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    first_actual_torque_jump_max_nm: Option<f32>,
    first_saturated_controls: Option<usize>,
    first_compensation_active: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    first_control_transition: Option<TerminalControlTransition>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present_terminal_bumpless")]
    bumpless: Option<TerminalBumpless>,
}

fn present_terminal_bumpless<'de, D: serde::Deserializer<'de>>(deserializer: D)
    -> Result<Option<TerminalBumpless>, D::Error>
{
    // Absence retains the byte-omitted legacy state. An explicit field must
    // contain V2 state; null must not smuggle a V2-only field into V1/None.
    TerminalBumpless::deserialize(deserializer).map(Some)
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TerminalBumpless {
    admission: TerminalAdmission,
    stable_substeps_at_binding: u64,
    first_intended_controls: Option<[f32; 21]>,
    first_intended_control_jump_max: Option<f32>,
    first_intended_torque_jump_max_nm: Option<f32>,
    /// Final normalized-control validation diagnostic, never a progress clock.
    last_blend_clamped_controls: usize,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
enum TerminalAdmission {
    #[serde(rename = "full-body80")]
    FullBody80,
    #[serde(rename = "early-base80")]
    EarlyBase80,
}

#[derive(Serialize, Deserialize)]
struct TerminalActuatorJump {
    body: String,
    role: String,
    actuator_index: usize,
    normalized_jump: f32,
    torque_jump_nm: f32,
    raw_pd: f32,
    raw_compensated: f32,
    pd_saturated: bool,
    compensated_saturated: bool,
}

#[derive(Serialize, Deserialize)]
struct TerminalControlTransition {
    preview: super::supported_standing::TerminalControlPreview,
    actuator_jumps: Vec<TerminalActuatorJump>,
}

/// Explicit semantic variant, also decoded by builds that must refuse it.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum TerminalBaseReadyFormat {
    #[serde(rename = "terminal-base-ready-v1")]
    V1,
    #[serde(rename = "terminal-base-ready-bumpless-v2")]
    BumplessV2,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TerminalBaseReady {
    format: TerminalBaseReadyFormat,
    consecutive_substeps: u64,
    feet: Option<[Vec3; 2]>,
}

#[derive(Serialize, Deserialize)]
pub struct Controller {
    pub status: lh_contracts::GetUpPolicyObservation,
    pub started_substep: Option<u64>,
    impact_substep: Option<u64>,
    reference: Vec<[f32; 25]>,
    teacher_frames: Vec<TeacherFrame>,
    teacher_initial: Option<Snapshot>,
    teacher_native_dt_s: Option<f32>,
    teacher_failure: Option<measurement::Failure>,
    decision_observation: Vec<f32>,
    student_decisions: Vec<StudentDecision>,
    residual: [f32; 21],
    reference_pose: [f32; 21],
    holding: bool,
    last_decision: Option<u64>,
    stable_substeps: u64,
    stable_feet: Option<[Vec3; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    terminal_balance: Option<TerminalBalance>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    terminal_base_ready: Option<TerminalBaseReady>,
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
            teacher_native_dt_s: None,
            teacher_failure: None,
            decision_observation: Vec::new(),
            student_decisions: Vec::new(),
            residual: [0.0; 21],
            reference_pose: standing_target(model)[..21].try_into().unwrap(),
            holding: true,
            last_decision: None,
            stable_substeps: 0,
            stable_feet: None,
            terminal_balance: None,
            terminal_base_ready: if cfg!(feature = "getup-terminal-base-ready-experiment") {
                Some(TerminalBaseReady {
                    format: if cfg!(feature = "getup-terminal-bumpless-experiment") {
                        TerminalBaseReadyFormat::BumplessV2
                    } else { TerminalBaseReadyFormat::V1 },
                    consecutive_substeps: 0,
                    feet: None,
                })
            } else { None },
        }
    }
    pub(crate) fn controller_label(&self) -> &'static str {
        if self.terminal_balance.is_some() {
            if self.terminal_bumpless_variant() {
                "experimental-recovery-bumpless-joint-balance-hold"
            } else if self.terminal_base_ready.is_some() {
                "experimental-recovery-base-ready-joint-balance-hold"
            } else { "experimental-recovery-joint-balance-hold" }
        } else if cfg!(feature = "getup-terminal-balance-experiment")
            && self.status.phase == "experimental-joint-balance-awaiting-fall"
        {
            if self.terminal_bumpless_variant() {
                "experimental-recovery-bumpless-joint-balance-ready"
            } else if self.terminal_base_ready.is_some() {
                "experimental-recovery-base-ready-joint-balance-ready"
            } else { "experimental-recovery-joint-balance-ready" }
        } else {
            "pretrained-WEAK-SLOW"
        }
    }
    pub(crate) fn checkpoint_variant_supported(&self) -> bool {
        (!self.terminal_base_ready_variant()
            || cfg!(feature = "getup-terminal-base-ready-experiment"))
            && (!self.terminal_bumpless_variant()
                || cfg!(feature = "getup-terminal-bumpless-experiment"))
            && self.terminal_base_ready_variant_valid()
            && ((self.terminal_balance.is_none()
            && self.status.phase != "experimental-joint-balance-awaiting-fall")
            || cfg!(feature = "getup-terminal-balance-experiment"))
    }
    pub(crate) fn terminal_base_ready_variant(&self) -> bool {
        self.terminal_base_ready.is_some()
    }
    pub(crate) fn terminal_bumpless_variant(&self) -> bool {
        self.terminal_base_ready.as_ref().is_some_and(|ready|
            ready.format == TerminalBaseReadyFormat::BumplessV2)
    }
    pub(crate) fn terminal_base_ready_variant_valid(&self) -> bool {
        let bumpless = self.terminal_bumpless_variant();
        let binding_valid = self.terminal_balance.as_ref().is_none_or(|balance| {
            if !bumpless { return balance.bumpless.is_none(); }
            balance.bumpless.as_ref().is_some_and(|blend| {
                let admission_valid = match blend.admission {
                    TerminalAdmission::FullBody80 => blend.stable_substeps_at_binding >= 80,
                    TerminalAdmission::EarlyBase80 => blend.stable_substeps_at_binding < 80
                        && self.terminal_base_ready.as_ref().is_some_and(|ready|
                            ready.consecutive_substeps == 80),
                };
                admission_valid && normalized_controls_valid(&balance.last_slow_controls)
                    && blend.last_blend_clamped_controls <= 21
                    && blend.first_intended_controls.as_ref().is_none_or(normalized_controls_valid)
                    && blend.first_intended_control_jump_max.is_none_or(|x| x.is_finite() && x >= 0.0)
                    && blend.first_intended_torque_jump_max_nm.is_none_or(|x| x.is_finite() && x >= 0.0)
            })
        });
        binding_valid && self.terminal_base_ready.as_ref().is_none_or(|ready| {
            ready.consecutive_substeps <= 80
                && (match ready.feet {
                    None => ready.consecutive_substeps == 0,
                    Some(feet) => ready.consecutive_substeps > 0
                        && feet.iter().all(|p| vector(*p).is_finite()),
                })
                && (bumpless || self.terminal_balance.is_none() || ready.consecutive_substeps == 80)
        })
    }
    pub(crate) fn terminal_bumpless_clock_valid(&self, completed_native_substeps: u64) -> bool {
        !self.terminal_bumpless_variant() || self.terminal_balance.as_ref().is_none_or(|balance|
            completed_native_substeps.checked_sub(balance.activated_native_substep).is_some())
    }
    #[cfg(feature = "getup-terminal-balance-experiment")]
    fn terminal_admission_ready(&self) -> bool {
        self.terminal_base_ready.as_ref().map_or(self.stable_substeps >= 80, |ready| {
            (ready.format == TerminalBaseReadyFormat::BumplessV2 && self.stable_substeps >= 80)
                || (cfg!(feature = "getup-terminal-base-ready-experiment")
                    && ready.consecutive_substeps == 80)
        })
    }

    #[cfg(feature = "getup-terminal-balance-experiment")]
    fn bind_terminal_balance(&mut self, s: &Simulation) {
        // holding starts true before recovery, and reference_frames excludes
        // the appended canonical terminal target. Require the real final index.
        if self.terminal_balance.is_some()
            || self.started_substep.is_none()
            || self.reference.is_empty()
            || !self.holding
            || self.status.reference_index as usize != self.reference.len() - 1
            || self.last_decision.is_none_or(|index| index < (self.reference.len() - 1) as u64)
            || !self.terminal_admission_ready()
        {
            return;
        }
        let Some(reference) = super::supported_standing::recovered_reference(s) else {
            return;
        };
        self.bind_terminal_balance_reference(s, reference);
    }
    #[cfg(feature = "getup-terminal-balance-experiment")]
    fn bind_terminal_balance_reference(&mut self, s: &Simulation, reference: super::supported_standing::StandingReference) {
        if self.terminal_balance.is_some() { return; }
        let Some(last_slow_controls) = s.diagnostics.getup.as_ref()
            .and_then(|g| g.motor_controls.as_slice().try_into().ok()) else {
            return;
        };
        let bumpless = if self.terminal_bumpless_variant() {
            if !normalized_controls_valid(&last_slow_controls) { return; }
            Some(TerminalBumpless {
                admission: if self.stable_substeps >= 80 {
                    TerminalAdmission::FullBody80
                } else { TerminalAdmission::EarlyBase80 },
                stable_substeps_at_binding: self.stable_substeps,
                first_intended_controls: None,
                first_intended_control_jump_max: None,
                first_intended_torque_jump_max_nm: None,
                last_blend_clamped_controls: 0,
            })
        } else { None };
        self.terminal_balance = Some(TerminalBalance {
            activated_native_substep: s.substeps_completed,
            reference,
            last_slow_controls,
            first_balance_controls: None,
            first_actual_control_jump_max: None,
            first_actual_torque_jump_max_nm: None,
            first_saturated_controls: None,
            first_compensation_active: None,
            first_control_transition: None,
            bumpless,
        });
    }

    #[cfg(feature = "getup-terminal-base-ready-experiment")]
    fn reset_terminal_base_ready(&mut self) {
        if let Some(ready) = self.terminal_base_ready.as_mut() {
            ready.consecutive_substeps = 0;
            ready.feet = None;
        }
    }

    /// Distinct control admission. This never changes the frozen acceptance
    /// counter/anchors and never treats a desired reaction as physical support.
    #[cfg(feature = "getup-terminal-base-ready-experiment")]
    fn terminal_base_ready_candidate(&self, s: &Simulation, physical: &[f32; 67], loads: &[f32; 25])
        -> Option<([Vec3; 2], super::supported_standing::StandingReference)>
    {
        if self.started_substep.is_none() || self.reference.is_empty() || !self.holding
            || self.status.reference_index as usize != self.reference.len() - 1
            || self.last_decision.is_none_or(|index| index < (self.reference.len() - 1) as u64)
            || physical.iter().any(|x| !x.is_finite())
        { return None; }
        let contract = &recovery_contract::contract().recovery_acceptance;
        let ids = ["left_foot", "right_foot"]
            .map(|id| s.model.segments.iter().position(|p| p.id == id).unwrap());
        let feet = ids.map(|i| vec3(s.world.bodies[s.bodies[i]].translation()));
        let foot_load: f32 = ids.iter().map(|i| loads[*i]).sum();
        let only_feet = loads.iter().enumerate().all(|(i, l)| ids.contains(&i) || *l < 3.0);
        let mut points = Vec::new();
        for i in ids {
            if let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) {
                for manifold in pair.solver_manifolds() {
                    let sign = if pair.collider2 == s.colliders[i] { 1.0 } else { -1.0 };
                    if manifold.data.normal.y * sign < 0.65 { continue; }
                    for contact in &manifold.data.solver_contacts {
                        let point = &manifold.points[contact.contact_indices()[0] as usize];
                        if point.data.impulse > 0.0 {
                            let (a, b) = manifold.data.solver_contact_world_points(contact, &s.world.bodies);
                            let p = if pair.collider2 == s.colliders[i] { b } else { a };
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
        let twists = super::current_twists(s).ok()?;
        if !(physical[21] > 1.4
            && physical[36] >= contract.stable_up_dot
            && ids.iter().all(|i| loads[*i] >= contract.minimum_load_n && s.integrated_support_substeps[*i] >= 40)
            && foot_load >= 0.8 * s.total_mass() * 9.81
            && only_feet && balanced
            && terminal_base_ready_motion_passes(&s.model, &twists))
        { return None; }
        let reference = super::supported_standing::recovered_reference(s)?;
        let preview = super::supported_standing::terminal_control_preview(
            s, &reference, &self.reference_pose,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(),
        ).ok()?;
        preview.finite_active_unclipped().then_some((feet, reference))
    }

    #[cfg(feature = "getup-terminal-base-ready-experiment")]
    fn observe_terminal_base_ready(&mut self, s: &Simulation, physical: &[f32; 67], loads: &[f32; 25]) {
        if self.terminal_base_ready.is_none() || self.terminal_balance.is_some() { return; }
        let Some((feet, reference)) = self.terminal_base_ready_candidate(s, physical, loads) else {
            self.reset_terminal_base_ready();
            return;
        };
        let ready = self.terminal_base_ready.as_mut().unwrap();
        let anchors = ready.feet.get_or_insert(feet);
        if !anchors.iter().zip(&feet).all(|(a, b)| vector(*a).distance(vector(*b)) <= 0.01) {
            self.reset_terminal_base_ready();
            return;
        }
        ready.consecutive_substeps = (ready.consecutive_substeps + 1).min(80);
        if ready.consecutive_substeps == 80 {
            // Bind exactly this admitted recovered frame, once, without a world
            // write or a second reference capture.
            self.bind_terminal_balance_reference(s, reference);
        }
    }
    pub fn impact(&mut self, substep: u64) {
        if self.impact_substep.is_none() {
            self.impact_substep = Some(substep);
        }
    }
    /// Fixture epoch for the unchanged25s/1s criterion. No impact is fabricated.
    pub fn fixture_trigger(&mut self, substep:u64) -> Result<(),String> {
        if self.started_substep.is_some() || self.impact_substep.is_some() {
            return Err("GetUp controller already triggered".into());
        }
        self.impact_substep=Some(substep);
        Ok(())
    }
    pub fn command(&mut self, s: &Simulation, physical: &[f32; 67]) -> Result<[f32; 21], String> {
        if self.terminal_bumpless_variant() && !cfg!(feature = "getup-terminal-bumpless-experiment") {
            return Err("Terminal bumpless transfer requires its experimental controller build".into());
        }
        if self.terminal_base_ready_variant() && !cfg!(feature = "getup-terminal-base-ready-experiment") {
            return Err("Terminal base readiness requires its experimental controller build".into());
        }
        if !self.checkpoint_variant_supported() {
            return Err("Active terminal balance requires its experimental controller build".into());
        }
        if !self.terminal_bumpless_clock_valid(s.substeps_completed) {
            return Err("Terminal bumpless activation is ahead of the native clock".into());
        }
        if self
            .impact_substep
            .or(self.started_substep)
            .is_some_and(|n| s.substeps_completed - n >= 20000)
            && !self.status.prototype_standing_passed
        {
            self.status.refusal = Some("25-second physical recovery deadline reached".into());
            return Err(self.status.refusal.clone().unwrap());
        }
        #[cfg(feature = "getup-terminal-balance-experiment")]
        if self.started_substep.is_none() && s.motion != Motion::Fallen {
            // Loading the recovery policy must not replace the qualified
            // upright balance with bare source PD while waiting for a fall.
            // Lying fixture preparation bypasses command and remains zero-control.
            self.status.phase = "experimental-joint-balance-awaiting-fall".into();
            return super::supported_standing::controls(
                s, &self.reference_pose,
                physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(),
            );
        }
        #[cfg(feature = "getup-terminal-balance-experiment")]
        if let Some(balance) = self.terminal_balance.as_mut() {
            let q = physical[..21].try_into().unwrap();
            let dq = physical[46..67].try_into().unwrap();
            let (live_controls, compensation_active) = super::supported_standing::controls_with_reference(
                s, &balance.reference, &self.reference_pose, q, dq,
            )?;
            #[cfg(feature = "getup-terminal-bumpless-experiment")]
            let (controls, blending) = if let Some(blend) = balance.bumpless.as_mut() {
                let k = s.substeps_completed.checked_sub(balance.activated_native_substep)
                    .ok_or("Terminal bumpless activation is ahead of the native clock")?;
                let (controls, clamped) = terminal_bumpless_controls(&balance.last_slow_controls, &live_controls, k)?;
                blend.last_blend_clamped_controls = clamped;
                if blend.first_intended_controls.is_none() {
                    blend.first_intended_controls = Some(live_controls);
                    blend.first_intended_control_jump_max = Some(live_controls.iter()
                        .zip(balance.last_slow_controls).map(|(a, b)| (*a - b).abs()).fold(0.0, f32::max));
                    blend.first_intended_torque_jump_max_nm = Some(s.model.segments.iter()
                        .filter_map(|p| p.reference.as_ref()?.hinge.as_ref())
                        .map(|h| h.gear * (live_controls[h.actuator_index] - balance.last_slow_controls[h.actuator_index]).abs())
                        .fold(0.0, f32::max));
                }
                (controls, k < 20)
            } else { (live_controls, false) };
            #[cfg(not(feature = "getup-terminal-bumpless-experiment"))]
            let (controls, blending) = (live_controls, false);
            if balance.first_balance_controls.is_none() {
                balance.first_actual_control_jump_max = Some(controls.iter()
                    .zip(balance.last_slow_controls).map(|(a, b)| (*a - b).abs()).fold(0.0, f32::max));
                balance.first_saturated_controls = Some(controls.iter().filter(|x| x.abs() >= 1.0).count());
                balance.first_balance_controls = Some(controls);
                balance.first_compensation_active = Some(compensation_active);
                #[cfg(feature = "getup-terminal-base-ready-experiment")]
                if self.terminal_base_ready.is_some() {
                    balance.first_actual_torque_jump_max_nm = Some(s.model.segments.iter()
                        .filter_map(|p| p.reference.as_ref()?.hinge.as_ref())
                        .map(|h| h.gear * (controls[h.actuator_index] - balance.last_slow_controls[h.actuator_index]).abs())
                        .fold(0.0, f32::max));
                    let preview = super::supported_standing::terminal_control_preview(
                        s, &balance.reference, &self.reference_pose, q, dq,
                    )?;
                    let actuator_jumps = s.model.segments.iter().enumerate().filter_map(|(i, part)| {
                        let h = part.reference.as_ref()?.hinge.as_ref()?;
                        let jump = controls[h.actuator_index] - balance.last_slow_controls[h.actuator_index];
                        Some(TerminalActuatorJump {
                            body: part.id.clone(),
                            role: if terminal_base_ready_arm_body(&s.model, i) { "arm" } else { "base" }.into(),
                            actuator_index: h.actuator_index,
                            normalized_jump: jump,
                            torque_jump_nm: h.gear * jump,
                            raw_pd: preview.raw_pd[h.actuator_index],
                            raw_compensated: preview.raw_compensated[h.actuator_index],
                            pd_saturated: preview.pd_saturated[h.actuator_index],
                            compensated_saturated: preview.compensated_saturated[h.actuator_index],
                        })
                    }).collect();
                    balance.first_control_transition = Some(TerminalControlTransition { preview, actuator_jumps });
                }
            }
            // Stop running and archiving SLOW after the explicit controller
            // transfer. Its decision counter/archive remain a complete prefix.
            self.status.phase = if blending {
                "experimental-joint-balance-blending"
            } else { "experimental-joint-balance-holding" }.into();
            return Ok(controls);
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
        // Predict the actual native world. Changing its timestep to the source
        // MuJoCo teacher's 5 ms changed Rapier's contact/CCD behavior and broke a
        // welded link in the retained forecast. Keep every cloned parameter;
        // only the actor cadence is inherited from the source (40 Hz).
        if shadow.world.integration_parameters.dt != shadow.profile.dt_s {
            return Err("GetUp branch native clock differs from its profile".into());
        }
        let native_per_policy = policy_substeps(shadow.profile.dt_s)?;
        self.teacher_native_dt_s = Some(shadow.profile.dt_s);
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
            for substep in 0..native_per_policy {
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
        let time = self.status.forecast_substeps as f64 * s.profile.dt_s as f64;
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
            Err(_) => {
                #[cfg(feature = "getup-terminal-base-ready-experiment")]
                if self.terminal_balance.is_none() { self.reset_terminal_base_ready(); }
                return;
            },
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
        #[cfg(feature = "getup-terminal-base-ready-experiment")]
        self.observe_terminal_base_ready(s, &physical, loads);
        #[cfg(feature = "getup-terminal-balance-experiment")]
        self.bind_terminal_balance(s);
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

fn normalized_controls_valid(controls: &[f32; 21]) -> bool {
    controls.iter().all(|x| x.is_finite() && (-1.0..=1.0).contains(x))
}

/// Endpoint-reference crossfade only. Sample-held torques and live feedback
/// need not be C2 through contact switches or the standing controller's clamps.
#[cfg(feature = "getup-terminal-bumpless-experiment")]
fn terminal_bumpless_weight(s: f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    (10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5)).clamp(0.0, 1.0)
}

#[cfg(feature = "getup-terminal-bumpless-experiment")]
fn terminal_bumpless_controls(source: &[f32; 21], live: &[f32; 21], k: u64)
    -> Result<([f32; 21], usize), String>
{
    if !normalized_controls_valid(source) || !normalized_controls_valid(live) {
        return Err("Terminal bumpless controls must be finite normalized actuator commands".into());
    }
    // Copy endpoints, preserving every f32 word including signed zero. Twenty
    // physical intervals at 800 Hz are 25 ms; calls never advance this clock.
    if k == 0 { return Ok((*source, 0)); }
    if k >= 20 { return Ok((*live, 0)); }
    let w = terminal_bumpless_weight(k as f64 / 20.0);
    let mut clamped = 0;
    let mut controls = [0.0; 21];
    for i in 0..21 {
        let value = ((1.0 - w) * f64::from(source[i]) + w * f64::from(live[i])) as f32;
        if !value.is_finite() { return Err("Nonfinite terminal bumpless interpolation".into()); }
        if !(-1.0..=1.0).contains(&value) { clamped += 1; }
        controls[i] = value.clamp(-1.0, 1.0);
    }
    Ok((controls, clamped))
}

/// Source shoulder coordinate frames and all descendants form the arm tree.
/// The complement includes head and every virtual trunk/leg coordinate frame.
#[cfg(feature = "getup-terminal-base-ready-experiment")]
fn terminal_base_ready_arm_body(model: &Model, mut index: usize) -> bool {
    for _ in 0..model.segments.len() {
        let part = &model.segments[index];
        if ["@right_shoulder1", "@left_shoulder1"].contains(&part.id.as_str()) { return true; }
        let Some(parent) = part.parent.as_ref() else { return false; };
        let Some(parent) = model.segments.iter().position(|p| &p.id == parent) else { return false; };
        index = parent;
    }
    false
}

#[cfg(feature = "getup-terminal-base-ready-experiment")]
fn terminal_base_ready_motion_passes(model: &Model, twists: &[(Vector, Vector)]) -> bool {
    let contract = &recovery_contract::contract().recovery_acceptance;
    twists.len() == model.segments.len() && twists.iter().enumerate().all(|(i, (v, w))| {
        let (linear, angular) = if terminal_base_ready_arm_body(model, i) {
            (contract.settle_linear_mps, contract.settle_angular_radps)
        } else { (contract.stable_linear_mps, contract.stable_angular_radps) };
        v.is_finite() && w.is_finite() && v.length() <= linear && w.length() <= angular
    })
}


/// Read-only reconstruction of the exact standing predicates from a checkpoint.
/// Optional anchors belong to an earlier checkpoint in the same stable interval.
pub fn standing_diagnostic(s: &Simulation, anchors: Option<[Vec3; 2]>) -> Result<serde_json::Value, String> {
    let physical = super::physical_observation(s)?;
    let contract = &recovery_contract::contract().recovery_acceptance;
    let ids = ["left_foot", "right_foot"].map(|id| s.model.segments.iter().position(|p| p.id == id).unwrap());
    let feet = ids.map(|i| vec3(s.world.bodies[s.bodies[i]].translation()));
    let mut points = Vec::new();
    for i in ids {
        if let Some(pair) = s.world.narrow_phase.contact_pair(s.floor, s.colliders[i]) {
            for manifold in pair.solver_manifolds() {
                let sign = if pair.collider2 == s.colliders[i] { 1.0 } else { -1.0 };
                if manifold.data.normal.y * sign < 0.65 { continue; }
                for contact in &manifold.data.solver_contacts {
                    let point = &manifold.points[contact.contact_indices()[0] as usize];
                    if point.data.impulse > 0.0 {
                        let (a,b) = manifold.data.solver_contact_world_points(contact, &s.world.bodies);
                        let p = if pair.collider2 == s.colliders[i] { b } else { a };
                        points.push([p.x,p.z]);
                    }
                }
            }
        }
    }
    let loads = s.integrated_support_loads_n;
    let com = s.center_of_mass();
    let velocity = Vector::new(physical[38],physical[39],physical[37]);
    let capture = com + velocity * (com.y.max(0.0)/9.81).sqrt();
    let twists = super::current_twists(s)?;
    let motion: Vec<_> = twists.iter().enumerate().map(|(i,(v,w))| serde_json::json!({
        "body":s.model.segments[i].id,"linearMps":v.length(),"angularRadps":w.length(),
        "linearPassed":v.length()<=contract.stable_linear_mps,"angularPassed":w.length()<=contract.stable_angular_radps
    })).collect();
    let drift = anchors.map(|a| [vector(a[0]).distance(vector(feet[0])),vector(a[1]).distance(vector(feet[1]))]);
    let predicates = serde_json::json!({
        "head":physical[21]>1.4,"torso":physical[36]>=contract.stable_up_dot,
        "bothFeetLoadedAndPersistent":ids.iter().all(|i| loads[*i]>=contract.minimum_load_n && s.integrated_support_substeps[*i]>=40),
        "footLoadFraction":ids.iter().map(|i|loads[*i]).sum::<f32>()>=0.8*s.total_mass()*9.81,
        "onlyFeet":loads.iter().enumerate().all(|(i,l)|ids.contains(&i)||*l<3.0),
        "comInSupport":stepping::inside_support(points.clone(),[com.x,com.z],0.003),
        "captureInSupport":stepping::inside_support(points.clone(),[capture.x,capture.z],0.003),
        "linearMotion":twists.iter().all(|(v,_)|v.length()<=contract.stable_linear_mps),
        "angularMotion":twists.iter().all(|(_,w)|w.length()<=contract.stable_angular_radps),
        "footDriftFromSuppliedAnchors":drift.map(|d|d.iter().all(|v|*v<=0.01))
    });
    Ok(serde_json::json!({"physicsIntegrations":0,"nativeSubsteps":s.substeps_completed,
        "predicates":predicates,"headM":physical[21],"torsoUp":physical[36],"com":vec3(com),
        "comVelocity":vec3(velocity),"capture":vec3(capture),"footIds":ids,"feet":feet,
        "loadsN":loads,"supportPersistence":s.integrated_support_substeps,"solverSupportPoints":points,
        "motion":motion,"footDriftM":drift,"contactMotion":super::standing::contact_diagnostic(s)?,"thresholds":{"linearMps":contract.stable_linear_mps,
        "angularRadps":contract.stable_angular_radps,"torsoUp":contract.stable_up_dot,"footDriftM":0.01}}))
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
#[path = "terminal_balance_tests.rs"]
mod terminal_balance_tests;

#[cfg(all(test, feature = "getup-terminal-base-ready-experiment"))]
#[path = "terminal_base_ready_tests.rs"]
mod terminal_base_ready_tests;

#[cfg(all(test, feature = "getup-terminal-bumpless-experiment"))]
#[path = "terminal_bumpless_tests.rs"]
mod terminal_bumpless_tests;

#[cfg(test)]
mod tests {
    #[test]
    fn teacher_keeps_native_clock_and_bounded_policy_cadence() {
        assert_eq!(super::policy_substeps(0.00125).unwrap(), 20);
        assert_eq!(super::policy_substeps(0.005).unwrap(), 5);
        assert!(super::policy_substeps(0.003).is_err());
        assert!(super::policy_substeps(0.0001).is_err());
        assert!(super::policy_substeps(f32::NAN).is_err());
        assert!(super::policy_substeps(0.0).is_err());
    }

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
