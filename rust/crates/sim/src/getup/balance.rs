//! Native-only, externally driven SAC experiment. No learner or trained balance
//! policy is bundled. The regular simulation loop owns every integration.
use crate::{getup, runtime::Runtime, *};
use serde::{Deserialize, Serialize};
use std::time::Instant;

pub const OBSERVATION_DIM: usize = 120;
pub const ACTION_DIM: usize = 21;
const MAX_EPISODE_SUBSTEPS: u64 = 16_000; // 20 seconds, not a success claim.
const CLEARANCE_M: f32 = 0.015;
fn episode_limit_substeps(seconds: u32) -> Result<u64, String> {
    if !(20..=40).contains(&seconds) { return Err("Diagnostic episode limit must be 20..40 seconds".into()); }
    Ok(u64::from(seconds) * 800)
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Metrics {
    pub anchor_drift_m: f32,
    pub head_height_m: f32,
    pub torso_up: f32,
    pub com_height_m: f32,
    pub horizontal_com_speed_mps: f32,
    pub maximum_body_linear_mps: f32,
    pub maximum_body_angular_radps: f32,
    pub foot_loads_n: [f32; 2],
    pub foot_clearance_m: [f32; 2],
    pub foot_horizontal_speed_mps: [f32; 2],
    pub loaded_contact_slip_mps: [f32; 2],
    pub foot_positions: [Vec3; 2],
    pub controls: [f32; 21],
    pub torques_nm: [f32; 21],
    pub nonfoot_support: bool,
    pub native_standing_without_foot_anchor: bool,
}

#[derive(Default, Serialize)]
pub struct Reward {
    pub total: f32,
    pub upright_stationary: f32,
    pub effort_cost: f32,
    pub control_change_cost: f32,
    pub loaded_slip_cost: f32,
    pub flight_cost: f32,
    pub scuff_cost: f32,
    pub prolonged_unload_cost: f32,
    pub body_motion_cost: f32,
}
impl Reward {
    fn accumulate(&mut self, r: &Self) {
        self.total += r.total;
        self.upright_stationary += r.upright_stationary;
        self.effort_cost += r.effort_cost;
        self.control_change_cost += r.control_change_cost;
        self.loaded_slip_cost += r.loaded_slip_cost;
        self.flight_cost += r.flight_cost;
        self.scuff_cost += r.scuff_cost;
        self.prolonged_unload_cost += r.prolonged_unload_cost;
        self.body_motion_cost += r.body_motion_cost;
    }
    fn average(mut self, samples: u64) -> Self {
        let n = samples.max(1) as f32;
        self.total /= n;
        self.upright_stationary /= n;
        self.effort_cost /= n;
        self.control_change_cost /= n;
        self.loaded_slip_cost /= n;
        self.flight_cost /= n;
        self.scuff_cost /= n;
        self.prolonged_unload_cost /= n;
        self.body_motion_cost /= n;
        self
    }
}

/// No positive clearance/step reward: hovering, hopping and endless stepping
/// cannot farm a bonus. Clearance is a recovery-step *evaluation*, not a goal.
fn reward(m: &Metrics, previous: &[f32; 21], unloaded_s: [f32; 2]) -> Reward {
    let upright = ((m.torso_up - 0.5) / 0.47).clamp(0.0, 1.0);
    let height = (-((m.com_height_m - 0.80326) / 0.15).powi(2)).exp();
    let position = (-(m.anchor_drift_m / 0.10).powi(2)).exp();
    let quiet = (-(m.horizontal_com_speed_mps / 0.20).powi(2)).exp();
    let mut r = Reward {
        upright_stationary: upright * height * position * quiet,
        effort_cost: 0.05 * m.controls.iter().map(|x| x * x).sum::<f32>() / 21.0,
        control_change_cost: 0.10
            * m.controls
                .iter()
                .zip(previous)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f32>()
            / 21.0,
        loaded_slip_cost: m
            .loaded_contact_slip_mps
            .iter()
            .map(|v| 0.25 * (v / 0.05).powi(2).min(4.0))
            .sum(),
        flight_cost: if m.foot_loads_n.iter().all(|v| *v < 3.0) {
            1.0
        } else {
            0.0
        },
        scuff_cost: (0..2)
            .filter(|i| {
                m.foot_loads_n[*i] < 3.0
                    && m.foot_horizontal_speed_mps[*i] > 0.03
                    && m.foot_clearance_m[*i] < CLEARANCE_M
            })
            .count() as f32
            * 0.15,
        prolonged_unload_cost: unloaded_s
            .iter()
            .map(|s| 0.25 * ((*s - 0.6) / 0.4).clamp(0.0, 1.0))
            .sum(),
        // Soft and bounded: recovery is still possible, but counter-moving
        // limbs/torso cannot obtain quiet-stance reward just by cancelling CoM.
        body_motion_cost: 0.05
            * ((m.maximum_body_linear_mps / 0.4).powi(2).min(4.0)
                + (m.maximum_body_angular_radps / 2.0).powi(2).min(4.0)),
        ..Default::default()
    };
    r.total = r.upright_stationary
        - r.effort_cost
        - r.control_change_cost
        - r.loaded_slip_cost
        - r.flight_cost
        - r.scuff_cost
        - r.prolonged_unload_cost;
    r.total -= r.body_motion_cost;
    r
}

/// One decision's acceleration-predicted reference, qpos order. Native lab only.
/// No state writes: the ordinary bounded joint torque path still drives physics.
#[derive(Clone)]
pub(crate) struct MotionProfile {
    pub start_substep: u64,
    pub q: [f32; 21],
    pub dq: [f32; 21],
    pub qdd: [f32; 21],
    pub torques_nm: [f32; 21],
}
impl MotionProfile {
    pub fn control(&self, i: usize, gear: f32, q: f32, dq: f32, t: f32) -> f32 {
        let q_ref = self.q[i] + t * self.dq[i] + 0.5 * t * t * self.qdd[i];
        let dq_ref = self.dq[i] + t * self.qdd[i];
        (self.torques_nm[i] / gear + q_ref - q + 0.2 * (dq_ref - dq)).clamp(-1.0, 1.0)
    }
}

/// Exact source interfaces for one externally driven policy decision.
/// Teacher values are normalized actuator controls. Student references and the
/// separately held residual are qpos-order radians; q/dq refresh every substep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub(crate) enum SourceCommand {
    Teacher { controls: [f32; 21] },
    Student { reference: [f32; 21], holding: bool },
}
impl SourceCommand {
    fn validate(&self) -> Result<(), String> {
        match self {
            Self::Teacher { controls } if controls.iter().any(|x| !x.is_finite() || x.abs() > 1.0) =>
                Err("Source teacher requires finite actuator controls in [-1,1]".into()),
            Self::Student { reference, .. } if reference.iter().any(|x| !x.is_finite()) =>
                Err("Source student requires finite qpos-order references".into()),
            _ => Ok(()),
        }
    }
    pub(crate) fn controls(&self, model: &Model, residual: &[f32;21], q: &[f32;21], dq: &[f32;21]) -> Result<[f32;21],String> {
        self.validate()?;
        match self {
            Self::Teacher { controls } => Ok(*controls),
            Self::Student { reference, holding } => getup::slow_controls(model, reference, residual, q, dq, *holding),
        }
    }
    pub(crate) fn label(&self) -> &'static str {
        match self { Self::Teacher { .. } => "source-GetUp-direct-torque-experiment", Self::Student { .. } => "source-GetUp-moving-reference-experiment" }
    }
}

#[derive(Clone, Serialize, Deserialize)]
struct Memory {
    anchor: [f32; 2],
    elapsed_substeps: u64,
    previous_controls: [f32; 21],
    last_residual: [f32; 21],
    quiet_substeps: u64,
    quiet_feet: Option<[Vec3; 2]>,
    unloaded_substeps: [u64; 2],
    unload_origin: [Vec3; 2],
    maximum_unloaded_clearance_m: [f32; 2],
    swing_supported_by_other: [bool; 2],
    recovery_steps_with_clearance: [u64; 2],
    loaded_slip_distance_m: [f32; 2],
    max_anchor_drift_m: f32,
    torque_change_squared_nm2: f64,
    torque_change_samples: u64,
    previous_torques_nm: [f32; 21],
}

#[derive(Serialize)]
pub struct Observation {
    pub observation: Vec<f32>,
    pub metrics: Metrics,
    pub elapsed_s: f64,
    pub continuous_quiet_s: f64,
    pub recovery_steps_with_clearance: [u64; 2],
    pub loaded_slip_distance_m: [f32; 2],
    pub maximum_anchor_drift_m: f32,
    pub torque_change_rms_nm: f64,
    pub physics_integrations: u64,
    pub trained_stationary_policy: bool,
    pub retained_teacher_forecast_substeps: u64,
}

#[derive(Serialize)]
pub struct NativeFootSample {
    pub native_substep: u64,
    /// Native-rate evidence for the task's stricter upright/height gates.
    pub torso_up: f32,
    pub com_height_m: f32,
    pub loads_n: [f32; 2],
    pub clearance_m: [f32; 2],
    pub positions: [Vec3; 2],
    pub loaded_contact_slip_mps: [f32; 2],
    pub nonfoot_support: bool,
    pub standing_without_foot_anchor: bool,
    pub motor_torques_nm: [f32; 21],
}

#[derive(Serialize)]
pub struct Transition {
    pub state: Observation,
    pub reward: Reward,
    pub terminated: bool,
    pub truncated: bool,
    pub failure: Option<String>,
    pub budget_physics_wall_s: f64,
    /// Actual observed native samples, not interpolated 40 Hz visual evidence.
    pub native_foot_samples: Vec<NativeFootSample>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub native_dynamics_audit: Option<Vec<serde_json::Value>>,
}

/// Shared across resets and checkpoint loads. Serialization never resets budget.
pub struct ComputeBudget {
    limit_s: f64,
    used_s: f64,
    pub native_substeps: u64,
}
impl ComputeBudget {
    pub fn new(limit_s: f64) -> Result<Self, String> {
        if !limit_s.is_finite() || !(0.0..=360.0).contains(&limit_s) {
            return Err(
                "Budget must be finite and within a separately authorized allowance, at most 360 s"
                    .into(),
            );
        }
        Ok(Self {
            limit_s,
            used_s: 0.0,
            native_substeps: 0,
        })
    }
    pub fn used_s(&self) -> f64 {
        self.used_s
    }
}

pub struct Experiment {
    runtime: Runtime,
    memory: Memory,
    episode_limit_substeps: u64,
    source_command: Option<SourceCommand>,
}

fn validate_clock(s: &Simulation) -> Result<(), String> {
    s.profile.validate()?;
    if s.profile.dt_s != lh_contracts::SUBSTEP_S
        || s.world.integration_parameters.dt != s.profile.dt_s
        || s.profile.tick_substeps != lh_contracts::SUBSTEPS
    {
        return Err(
            "Balance experiment requires unchanged 800 Hz native / 40 Hz policy clocks".into(),
        );
    }
    Ok(())
}

fn metrics(s: &Simulation, anchor: [f32; 2]) -> Result<Metrics, String> {
    let p = getup::physical_observation(s)?;
    let diag = getup::controller::standing_diagnostic(s, None)?;
    let feet = ["left_foot", "right_foot"].map(|name| {
        s.model
            .segments
            .iter()
            .position(|part| part.id == name)
            .unwrap()
    });
    let contacts = getup::standing::contacts(s)?;
    let twists = getup::current_twists(s)?;
    let com = s.center_of_mass();
    let controls: [f32; 21] = s
        .diagnostics
        .getup
        .as_ref()
        .and_then(|g| g.motor_controls.as_slice().try_into().ok())
        .unwrap_or([0.0; 21]);
    let mut torques = [0.0; 21];
    for part in &s.model.segments {
        if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            torques[h.actuator_index] = h.gear * controls[h.actuator_index];
        }
    }
    Ok(Metrics {
        anchor_drift_m: ((com.x - anchor[0]).powi(2) + (com.z - anchor[1]).powi(2)).sqrt(),
        head_height_m: p[21],
        torso_up: p[36],
        com_height_m: com.y,
        horizontal_com_speed_mps: (p[37].powi(2) + p[38].powi(2)).sqrt(),
        maximum_body_linear_mps: twists.iter().map(|(v, _)| v.length()).fold(0.0, f32::max),
        maximum_body_angular_radps: twists.iter().map(|(_, w)| w.length()).fold(0.0, f32::max),
        foot_loads_n: feet.map(|i| s.integrated_support_loads_n[i]),
        // Compound AABBs can include empty space below rotated capsules. Use
        // the actual world-space primitive endpoints, including their radii.
        foot_clearance_m: feet.map(|i| primitive_clearance(s, i).max(0.0)),
        foot_horizontal_speed_mps: feet
            .map(|i| Vector::new(twists[i].0.x, 0.0, twists[i].0.z).length()),
        loaded_contact_slip_mps: feet.map(|i| {
            let impulse: f32 = contacts
                .iter()
                .filter(|c| c.body == i)
                .map(|c| c.impulse)
                .sum();
            if s.integrated_support_loads_n[i] < 3.0 || impulse <= 0.0 {
                return 0.0;
            }
            contacts
                .iter()
                .filter(|c| c.body == i)
                .map(|c| (c.velocity - c.normal * c.velocity.dot(c.normal)).length() * c.impulse)
                .sum::<f32>()
                / impulse
        }),
        foot_positions: feet.map(|i| vec3(s.world.bodies[s.bodies[i]].translation())),
        controls,
        torques_nm: torques,
        nonfoot_support: s
            .integrated_support_loads_n
            .iter()
            .enumerate()
            .any(|(i, v)| !feet.contains(&i) && *v >= 3.0),
        native_standing_without_foot_anchor: diag["predicates"]
            .as_object()
            .unwrap()
            .iter()
            .filter(|(k, _)| k.as_str() != "footDriftFromSuppliedAnchors")
            .all(|(_, v)| v == true),
    })
}

fn dynamics_audit_frame(s: &Simulation, m: &Metrics, stage: &str) -> Result<serde_json::Value, String> {
    let mut tangents = Vec::new();
    for name in ["left_foot", "right_foot"] {
        let i = s.model.segments.iter().position(|p| p.id == name).ok_or("Missing audit foot")?;
        tangents.push(vec3(measurement::floor_tangent_impulse(&s.world, s.floor, s.colliders[i])?));
    }
    Ok(serde_json::json!({"stage":stage,"nativeSubstep":s.substeps_completed,
        "physicalObservation":getup::physical_observation(s)?.to_vec(),"snapshot":s.snapshot(),
        "contactMotion":getup::standing::contact_diagnostic(s)?,
        "footTangentImpulsesWorldNs":tangents,"motorTorquesNmActuatorOrder":m.torques_nm,
        "interpretation":"after: final post-solve state; motor torque applied before this native integration; impulses N.s over latest native1/800s frame, after all internal solver substeps; normal points and tangent vectors act on foot; no CCD bodies"}))
}

fn capsule_bottom(a: Vector, b: Vector, radius: f32) -> f32 {
    a.y.min(b.y) - radius
}

fn primitive_clearance(s: &Simulation, body: usize) -> f32 {
    let rb = &s.world.bodies[s.bodies[body]];
    s.model.segments[body]
        .reference
        .as_ref()
        .unwrap()
        .primitives
        .iter()
        .map(|p| {
            let a = rb.translation() + rb.rotation() * vector(p.a);
            let b = rb.translation() + rb.rotation() * vector(p.b);
            capsule_bottom(a, b, p.radius)
        })
        .fold(f32::INFINITY, f32::min)
}

fn observation(s: &Simulation, m: &Memory) -> Result<Vec<f32>, String> {
    let physical = getup::physical_observation(s)?;
    let target = getup::controller::standing_target(&s.model);
    let original = getup::slow_observation(&physical, &target, &target);
    let com = s.center_of_mass();
    let mut result = original[..117].to_vec();
    // Source convention: horizontal Z, X. Power must remain the last input.
    result.extend_from_slice(&[com.z - m.anchor[1], com.x - m.anchor[0], original[117]]);
    Ok(result)
}

impl Experiment {
    /// Reset from an already integrated quiet-standing state; no settling replay,
    /// teacher forecast, state projection, foot pinning or hidden rollout.
    pub fn reset_from_getup_checkpoint(bytes: &[u8]) -> Result<Self, String> {
        Self::reset_checkpoint(bytes, true)
    }
    /// Explicit experiment entry for the recorded pre-quiet shuffling interval.
    /// This changes admission only; all standing, safety and terminal tests stay
    /// unchanged and the observation reports the actual nonstanding condition.
    pub fn reset_from_upright_checkpoint(bytes: &[u8]) -> Result<Self, String> {
        Self::reset_checkpoint(bytes, false)
    }
    fn reset_checkpoint(bytes: &[u8], require_standing: bool) -> Result<Self, String> {
        let mut runtime = Runtime::from_getup_checkpoint(bytes)?;
        let s = runtime.balance_simulation_mut();
        validate_clock(s)?;
        if s.first_failure.is_some() || s.getup_fixture.as_ref().is_some_and(|f| f.preparing) {
            return Err("Balance reset requires an intact, completed fixture".into());
        }
        let com = s.center_of_mass();
        let anchor = [com.x, com.z];
        let initial = metrics(s, anchor)?;
        if require_standing && !initial.native_standing_without_foot_anchor {
            return Err(
                "Balance reset requires native standing predicates at the input checkpoint".into(),
            );
        }
        if initial.head_height_m <= 1.4 || initial.torso_up < 0.9
            || initial.com_height_m < 0.5 || initial.nonfoot_support
        {
            return Err("Balance reset requires intact upright foot-only support".into());
        }
        s.balance_residual = Some([0.0; 21]);
        Ok(Self {
            runtime,
            episode_limit_substeps: MAX_EPISODE_SUBSTEPS,
            source_command: None,
            memory: Memory {
                anchor,
                elapsed_substeps: 0,
                previous_controls: initial.controls,
                last_residual: [0.0; 21],
                quiet_substeps: 0,
                quiet_feet: None,
                unloaded_substeps: [0; 2],
                unload_origin: initial.foot_positions,
                maximum_unloaded_clearance_m: [0.0; 2],
                recovery_steps_with_clearance: [0; 2],
                swing_supported_by_other: [false; 2],
                loaded_slip_distance_m: [0.0; 2],
                max_anchor_drift_m: 0.0,
                torque_change_squared_nm2: 0.0,
                torque_change_samples: 0,
                previous_torques_nm: initial.torques_nm,
            },
        })
    }
    pub fn observe(&self) -> Result<Observation, String> {
        let s = self.runtime.simulation();
        Ok(Observation {
            observation: observation(s, &self.memory)?,
            metrics: metrics(s, self.memory.anchor)?,
            elapsed_s: self.memory.elapsed_substeps as f64 * f64::from(s.profile.dt_s),
            continuous_quiet_s: self.memory.quiet_substeps as f64 * f64::from(s.profile.dt_s),
            recovery_steps_with_clearance: self.memory.recovery_steps_with_clearance,
            loaded_slip_distance_m: self.memory.loaded_slip_distance_m,
            maximum_anchor_drift_m: self.memory.max_anchor_drift_m,
            torque_change_rms_nm: (self.memory.torque_change_squared_nm2
                / self.memory.torque_change_samples.max(1) as f64)
                .sqrt(),
            physics_integrations: self.memory.elapsed_substeps,
            trained_stationary_policy: false,
            retained_teacher_forecast_substeps: s
                .getup_controller
                .as_ref()
                .map_or(0, |c| u64::from(c.status.forecast_substeps)),
        })
    }
    pub fn checkpoint(&self) -> Result<Vec<u8>, String> {
        bincode::serialize(&(3u32, self.runtime.getup_checkpoint()?, &self.memory, self.episode_limit_substeps, &self.source_command))
            .map_err(|e| e.to_string())
    }
    pub fn from_checkpoint(bytes: &[u8]) -> Result<Self, String> {
        use bincode::Options;
        let schema = u32::from_le_bytes(bytes.get(..4).ok_or("Short balance checkpoint")?.try_into().unwrap());
        let options = bincode::DefaultOptions::new().with_fixint_encoding().with_limit(64 * 1024 * 1024).reject_trailing_bytes();
        let (runtime, memory, episode_limit_substeps, source_command) = match schema {
            1 => { let (_, runtime, memory): (u32, Vec<u8>, Memory) = options.deserialize(bytes).map_err(|e|e.to_string())?; (runtime, memory, MAX_EPISODE_SUBSTEPS, None) },
            2 => { let (_, runtime, memory, limit): (u32, Vec<u8>, Memory, u64) = options.deserialize(bytes).map_err(|e|e.to_string())?; (runtime, memory, limit, None) },
            3 => { let (_, runtime, memory, limit, command): (u32, Vec<u8>, Memory, u64, Option<SourceCommand>) = options.deserialize(bytes).map_err(|e|e.to_string())?; (runtime, memory, limit, command) },
            _ => return Err("Unsupported balance checkpoint".into()),
        };
        if !(16_000..=32_000).contains(&episode_limit_substeps) || episode_limit_substeps % 800 != 0 {
            return Err("Invalid checkpoint episode limit".into());
        }
        let mut runtime = Runtime::from_getup_checkpoint(&runtime)?;
        validate_clock(runtime.simulation())?;
        if memory
            .anchor
            .iter()
            .chain(&memory.last_residual)
            .any(|v| !v.is_finite())
        {
            return Err("Nonfinite balance memory".into());
        }
        if let Some(command) = &source_command { command.validate()?; }
        runtime.balance_simulation_mut().balance_residual = Some(memory.last_residual);
        runtime.balance_simulation_mut().balance_source_command = source_command.clone();
        Ok(Self { runtime, memory, episode_limit_substeps, source_command })
    }
    /// Bounded diagnostic horizon only; never changes physical success gates.
    pub fn set_episode_limit_seconds(&mut self, seconds: u32) -> Result<(), String> {
        let limit = episode_limit_substeps(seconds)?;
        if self.memory.elapsed_substeps > limit { return Err("Episode limit precedes elapsed clock".into()); }
        self.episode_limit_substeps = limit;
        Ok(())
    }
    pub fn episode_limit_seconds(&self) -> u64 { self.episode_limit_substeps / 800 }
    pub fn snapshot(&self) -> Snapshot {
        self.runtime.simulation().snapshot()
    }
    pub fn initialized_action(&self) -> Result<[f32; 21], String> {
        static INITIALIZATION: std::sync::OnceLock<getup::actor::Actor> =
            std::sync::OnceLock::new();
        INITIALIZATION
            .get_or_init(getup::actor::Actor::stationary_initialization)
            .infer(&self.observe()?.observation)
            .map(|p| p.action)
    }
    /// Read-only exact source inputs/inference. No standing-nominal shim.
    pub fn source_state(&self, now: [f32;25], ahead: [f32;25], teacher_power: f32, holding: bool) -> Result<serde_json::Value,String> {
        if now.iter().chain(&ahead).any(|x| !x.is_finite()) || !teacher_power.is_finite() || !(0.0..=1.0).contains(&teacher_power) {
            return Err("Invalid source target or teacher power".into());
        }
        let s = self.runtime.simulation();
        let physical = getup::physical_observation(s)?;
        let student_input = getup::slow_observation(&physical, &now, &ahead);
        let mut teacher_input = physical.to_vec(); teacher_input.push(teacher_power);
        let student = getup::actor::student().infer(&student_input)?;
        let teacher = getup::actor::teacher().infer(&teacher_input)?;
        let controls = getup::slow_controls(&s.model, now[..21].try_into().unwrap(), &student.action,
            physical[..21].try_into().unwrap(), physical[46..67].try_into().unwrap(), holding)?;
        Ok(serde_json::json!({"physicalObservation":physical.to_vec(),"studentObservation":student_input.to_vec(),
            "teacherObservation":teacher_input,"student":student,"teacher":teacher,
            "studentMotorControlsActuatorOrder":controls,"holding":holding,"studentPower":1.0,
            "physicsIntegrations":0,"nativeSubstep":s.substeps_completed,
            "comSource":getup::source_vector(s.center_of_mass()),"sourceCommand":self.source_command}))
    }
    /// Source teacher: power-scaled normalized controls already in actuator order.
    pub fn step_source_teacher(&mut self, controls: [f32;21], audit: bool, budget: &mut ComputeBudget) -> Result<Transition,String> {
        let source = SourceCommand::Teacher { controls }; source.validate()?;
        self.step_with_profile([0.;21], [0.;2], None, Some(source), audit, budget)
    }
    /// Source student: reference and residual held for twenty substeps, kp1,
    /// moving kd.1 or holding kd.2. Only ordinary capped motor commands apply.
    pub fn step_source_student(&mut self, reference: [f32;21], residual: [f32;21], holding: bool, audit: bool, budget: &mut ComputeBudget) -> Result<Transition,String> {
        let source = SourceCommand::Student { reference, holding }; source.validate()?;
        self.step_with_profile(residual, [0.;2], None, Some(source), audit, budget)
    }
    /// Frozen source SLOW following an explicitly supplied kinematic reference.
    /// The returned action is expressed against the experiment's fixed nominal
    /// reference. Away from clipping this is algebraically the same PD target.
    /// This does not learn a policy, integrate, or alter the runtime controller.
    pub fn reference_action(
        &self,
        now: [f32; 25],
        ahead: [f32; 25],
    ) -> Result<serde_json::Value, String> {
        if now.iter().chain(&ahead).any(|x| !x.is_finite()) {
            return Err("Nonfinite reference".into());
        }
        let s = self.runtime.simulation();
        let physical = getup::physical_observation(s)?;
        let input = getup::slow_observation(&physical, &now, &ahead);
        let prediction = getup::actor::student().infer(&input)?;
        let nominal = getup::controller::standing_target(&s.model);
        let effective_target: [f32; 21] =
            std::array::from_fn(|i| now[i] + prediction.action[i]);
        let unclipped: [f32; 21] =
            std::array::from_fn(|i| effective_target[i] - nominal[i]);
        let action = unclipped.map(|v| v.clamp(-1.0, 1.0));
        let q = physical[..21].try_into().unwrap();
        let dq = physical[46..67].try_into().unwrap();
        let nominal_q = nominal[..21].try_into().unwrap();
        let now_q = now[..21].try_into().unwrap();
        let represented_controls = getup::slow_controls(&s.model, &nominal_q, &action, &q, &dq, true)?;
        let reference_controls = getup::slow_controls(&s.model, &now_q, &prediction.action, &q, &dq, true)?;
        let max_control_error = represented_controls.iter().zip(reference_controls)
            .map(|(a,b)| (a-b).abs()).fold(0.0_f32, f32::max);
        Ok(serde_json::json!({"action":action,"sourceResidual":prediction.action,
            "effectiveTarget":effective_target,"actionClipped":action!=unclipped,
            "representedMotorControls":represented_controls,
            "referenceMotorControls":reference_controls,
            "referenceControlMaxError":max_control_error,
            "physicsIntegrations":0,"trainedStationaryPolicy":false,
            "source":"frozen-SLOW-hand-supplied-reference"}))
    }
    pub fn diagnostic(&self) -> Result<serde_json::Value, String> {
        getup::controller::standing_diagnostic(self.runtime.simulation(), None)
    }
    pub fn step(
        &mut self,
        action: [f32; 21],
        budget: &mut ComputeBudget,
    ) -> Result<Transition, String> {
        self.step_with_force(action, [0.0; 2], budget)
    }
    /// Optional bounded horizontal force at torso CoM for one 25 ms decision.
    /// A curriculum may repeat pulses; it never writes a pose or velocity.
    pub fn step_with_force(
        &mut self,
        action: [f32; 21],
        torso_force_n: [f32; 2],
        budget: &mut ComputeBudget,
    ) -> Result<Transition, String> {
        self.step_with_profile(action, torso_force_n, None, None, false, budget)
    }
    /// Acceleration-predicted inner-impedance reference for one 25ms decision.
    /// All arrays qpos order; original gears, passive elements, and torque caps.
    pub fn step_motion_profile(
        &mut self, action: [f32; 21], torques_nm: [f32; 21],
        qdd: [f32; 21], budget: &mut ComputeBudget,
    ) -> Result<Transition, String> {
        self.step_motion_profile_with_audit(action, torques_nm, qdd, false, budget)
    }
    /// Same ordinary command, with an opt-in read-only per-native audit.
    pub fn step_motion_profile_audited(&mut self, action: [f32;21], torques_nm: [f32;21], qdd: [f32;21], budget: &mut ComputeBudget) -> Result<Transition,String> {
        self.step_motion_profile_with_audit(action, torques_nm, qdd, true, budget)
    }
    fn step_motion_profile_with_audit(&mut self, action: [f32;21], torques_nm: [f32;21], qdd: [f32;21], audit: bool, budget: &mut ComputeBudget) -> Result<Transition,String> {
        if qdd.iter().any(|v| !v.is_finite() || v.abs() > 50.001)
            || torques_nm.iter().any(|v| !v.is_finite()) {
            return Err("Motion profile requires finite torques and |qdd|<=50 rad/s2".into());
        }
        let physical = getup::physical_observation(self.runtime.simulation())?;
        for p in &self.runtime.simulation().model.segments {
            if let Some(h) = p.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
                if torques_nm[h.qpos_index].abs() > h.gear + 1e-5 {
                    return Err("Motion profile torque exceeds original gear cap".into());
                }
            }
        }
        let profile = MotionProfile {
            start_substep: self.runtime.simulation().substeps_completed,
            q: physical[..21].try_into().unwrap(), dq: physical[46..67].try_into().unwrap(),
            qdd, torques_nm,
        };
        self.step_with_profile(action, [0.0; 2], Some(profile), None, audit, budget)
    }
    fn install_command(&mut self, action: [f32;21], torso_force_n: [f32;2], profile: Option<MotionProfile>, source: Option<SourceCommand>) {
        let s = self.runtime.balance_simulation_mut();
        s.balance_residual = Some(action); s.balance_motion_profile = profile;
        s.balance_source_command = source.clone(); s.balance_torso_force_n = torso_force_n;
        self.source_command = source; self.memory.last_residual = action;
    }
    fn step_with_profile(
        &mut self, action: [f32; 21], torso_force_n: [f32; 2],
        profile: Option<MotionProfile>, source: Option<SourceCommand>, audit: bool, budget: &mut ComputeBudget,
    ) -> Result<Transition, String> {
        if action.iter().any(|v| !v.is_finite() || v.abs() > 1.0) {
            return Err("Action must be 21 finite qpos-order residuals in [-1,1]".into());
        }
        if torso_force_n.iter().any(|v| !v.is_finite())
            || torso_force_n[0].hypot(torso_force_n[1]) > 80.0
        {
            return Err("Horizontal torso force must be finite and at most 80 N".into());
        }
        let current = self.observe()?;
        let already_done = current.metrics.head_height_m < 1.0
            || current.metrics.com_height_m < 0.5
            || current.metrics.anchor_drift_m > 0.6
            || current.metrics.nonfoot_support;
        if already_done || self.memory.elapsed_substeps >= self.episode_limit_substeps {
            return Err("Episode ended; reset or load a checkpoint before stepping".into());
        }
        if budget.used_s + 0.25 >= budget.limit_s {
            return Ok(Transition {
                state: current,
                reward: Reward::default(),
                terminated: false,
                truncated: true,
                failure: Some("physics-budget-exhausted-before-step".into()),
                budget_physics_wall_s: budget.used_s,
                native_foot_samples: Vec::new(),
                native_dynamics_audit: None,
            });
        }
        self.install_command(action, torso_force_n, profile, source);
        let started = Instant::now();
        let mut native_dynamics_audit = if audit { Some(vec![dynamics_audit_frame(self.runtime.simulation(), &current.metrics, "before")?]) } else { None };
        let native_before = self.runtime.simulation().substeps_completed;
        let mut aggregate_reward = Reward::default();
        let mut native_foot_samples = Vec::with_capacity(20);
        let mut terminated = false;
        let used = budget.used_s;
        let limit = budget.limit_s;
        let memory = &mut self.memory;
        let reply = self.runtime.advance_observed(1, |s, substep| {
            let m = metrics(s, memory.anchor).map_err(|e| {
                measurement::Failure::new(s.tick, substep, "balance-observation", e, 1., 0.)
            })?;
            if let Some(frames) = &mut native_dynamics_audit {
                frames.push(dynamics_audit_frame(s, &m, "after").map_err(|e| measurement::Failure::new(s.tick, substep, "balance-dynamics-audit", e, 1., 0.))?);
            }
            memory.max_anchor_drift_m = memory.max_anchor_drift_m.max(m.anchor_drift_m);
            native_foot_samples.push(NativeFootSample {
                native_substep: s.substeps_completed,
                torso_up: m.torso_up,
                com_height_m: m.com_height_m,
                loads_n: m.foot_loads_n,
                clearance_m: m.foot_clearance_m,
                positions: m.foot_positions,
                loaded_contact_slip_mps: m.loaded_contact_slip_mps,
                nonfoot_support: m.nonfoot_support,
                standing_without_foot_anchor: m.native_standing_without_foot_anchor,
                motor_torques_nm: m.torques_nm,
            });
            let mut quiet = m.native_standing_without_foot_anchor
                && m.anchor_drift_m <= 0.10
                && m.horizontal_com_speed_mps <= 0.05;
            if quiet {
                let feet = memory.quiet_feet.get_or_insert(m.foot_positions);
                quiet = feet
                    .iter()
                    .zip(&m.foot_positions)
                    .all(|(a, b)| vector(*a).distance(vector(*b)) <= 0.01);
            }
            if !quiet {
                memory.quiet_feet = None;
            }
            memory.quiet_substeps = if quiet { memory.quiet_substeps + 1 } else { 0 };
            for i in 0..2 {
                memory.loaded_slip_distance_m[i] += m.loaded_contact_slip_mps[i] * s.profile.dt_s;
                if m.foot_loads_n[i] < 3.0 {
                    if memory.unloaded_substeps[i] == 0 {
                        memory.unload_origin[i] = m.foot_positions[i];
                        memory.swing_supported_by_other[i] = true;
                    }
                    memory.swing_supported_by_other[i] &= m.foot_loads_n[1 - i] >= 3.0;
                    memory.unloaded_substeps[i] += 1;
                    memory.maximum_unloaded_clearance_m[i] =
                        memory.maximum_unloaded_clearance_m[i].max(m.foot_clearance_m[i]);
                } else {
                    let a = memory.unload_origin[i];
                    let b = m.foot_positions[i];
                    let movement = ((a.x - b.x).powi(2) + (a.z - b.z).powi(2)).sqrt();
                    let duration = memory.unloaded_substeps[i] as f32 * s.profile.dt_s;
                    if memory.swing_supported_by_other[i]
                        && (0.025..=0.6).contains(&duration)
                        && movement >= 0.02
                        && memory.maximum_unloaded_clearance_m[i] >= CLEARANCE_M
                    {
                        memory.recovery_steps_with_clearance[i] += 1;
                    }
                    memory.unloaded_substeps[i] = 0;
                    memory.maximum_unloaded_clearance_m[i] = 0.0;
                }
            }
            memory.torque_change_squared_nm2 += m
                .torques_nm
                .iter()
                .zip(&memory.previous_torques_nm)
                .map(|(a, b)| f64::from(a - b).powi(2))
                .sum::<f64>();
            memory.torque_change_samples += 21;
            memory.previous_torques_nm = m.torques_nm;
            aggregate_reward.accumulate(&reward(
                &m,
                &memory.previous_controls,
                memory.unloaded_substeps.map(|n| n as f32 * s.profile.dt_s),
            ));
            terminated = m.head_height_m < 1.0
                || m.com_height_m < 0.5
                || m.anchor_drift_m > 0.6
                || m.nonfoot_support;
            if terminated || used + started.elapsed().as_secs_f64() >= limit - 0.10 {
                return Err(measurement::Failure::new(
                    s.tick,
                    substep,
                    if terminated {
                        "balance-episode-ended"
                    } else {
                        "balance-compute-budget"
                    },
                    "No further integration admitted".into(),
                    1.,
                    0.,
                ));
            }
            Ok(())
        });
        budget.used_s += started.elapsed().as_secs_f64();
        // Integrations can fail contact/integrity checks before the observer.
        // Charge actual native clock movement, not successful observer samples.
        let integrated = self.runtime.simulation().substeps_completed - native_before;
        budget.native_substeps += integrated;
        self.memory.elapsed_substeps += integrated;
        let reply = reply.map_err(|e| format!("Runtime rejected balance step: {e:?}"))?;
        self.memory.previous_controls = self.observe()?.metrics.controls;
        // Every component is an average over the same actual native substeps.
        let failure = reply
            .failure
            .as_ref()
            .map(|f| format!("{}: {}", f.check, f.offending));
        Ok(Transition {
            state: self.observe()?,
            // Fixed 40 Hz reward scale, including a partial/failed tick.
            reward: aggregate_reward.average(u64::from(lh_contracts::SUBSTEPS)),
            terminated,
            truncated: !terminated
                && (reply.failure.is_some()
                    || self.memory.elapsed_substeps >= self.episode_limit_substeps),
            failure,
            budget_physics_wall_s: budget.used_s,
            native_foot_samples,
            native_dynamics_audit,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn quiet() -> Metrics {
        Metrics {
            anchor_drift_m: 0.,
            head_height_m: 1.6,
            torso_up: 1.,
            com_height_m: 0.80326,
            horizontal_com_speed_mps: 0.,
            maximum_body_linear_mps: 0.,
            maximum_body_angular_radps: 0.,
            foot_loads_n: [200.; 2],
            foot_clearance_m: [0.; 2],
            foot_horizontal_speed_mps: [0.; 2],
            loaded_contact_slip_mps: [0.; 2],
            foot_positions: [Vec3::default(); 2],
            controls: [0.; 21],
            torques_nm: [0.; 21],
            nonfoot_support: false,
            native_standing_without_foot_anchor: true,
        }
    }
    #[test]
    fn diagnostic_episode_limit_is_bounded_and_default_is_twenty_seconds() {
        assert_eq!(episode_limit_substeps(20).unwrap(),MAX_EPISODE_SUBSTEPS);
        assert_eq!(episode_limit_substeps(40).unwrap(),32_000);
        assert!(episode_limit_substeps(19).is_err());assert!(episode_limit_substeps(41).is_err());
    }
    #[test]
    fn motion_profile_has_exact_feedforward_on_predicted_motion_and_original_caps() {
        let mut p = MotionProfile { start_substep: 123, q: [0.0;21], dq: [0.0;21], qdd:[0.0;21], torques_nm:[0.0;21] };
        p.q[6]=0.3; p.dq[6]=0.7; p.qdd[6]=40.99; p.torques_nm[6]=15.662;
        for k in 0..20 {
            let t=k as f32/800.0;
            let q=p.q[6]+t*p.dq[6]+0.5*t*t*p.qdd[6];
            let dq=p.dq[6]+t*p.qdd[6];
            assert!((80.0*p.control(6,80.0,q,dq,t)-p.torques_nm[6]).abs()<1e-5);
        }
        assert_eq!(p.control(6,80.0,-100.0,-100.0,0.0),1.0);
        assert_eq!(p.control(6,80.0,100.0,100.0,0.0),-1.0);
        assert!((p.control(6,80.0,p.q[6],p.dq[6],0.0)-p.torques_nm[6]/80.0).abs()<1e-6);
    }
    #[test]
    fn source_commands_keep_distinct_orders_damping_and_original_caps() {
        let model = Model::canonical().unwrap();
        let q = [0.0;21]; let dq = [0.5;21]; let residual = [0.1;21];
        let reference = std::array::from_fn(|i| i as f32 * 0.01);
        for holding in [false,true] {
            let source = SourceCommand::Student { reference, holding };
            let actual = source.controls(&model,&residual,&q,&dq).unwrap();
            assert_eq!(actual,getup::slow_controls(&model,&reference,&residual,&q,&dq,holding).unwrap());
        }
        let controls = std::array::from_fn(|i| -0.4 + i as f32 * 0.03);
        let teacher = SourceCommand::Teacher { controls };
        assert_eq!(teacher.controls(&model,&residual,&q,&dq).unwrap(), controls);
        let mut bad=controls; bad[0]=1.001;
        assert!(SourceCommand::Teacher { controls:bad }.validate().is_err());
        let mut bad=reference; bad[0]=f32::NAN;
        assert!(SourceCommand::Student { reference:bad, holding:false }.validate().is_err());
    }
    #[test]
    fn source_experiment_checkpoint_next_control_and_mode_clear_without_integration() {
        let mut profile=Profile::default();profile.getup_policy_enabled=true;
        let runtime = Runtime::new(lh_contracts::Mode::Protocol,0.0,profile).unwrap();
        let bytes = runtime.getup_checkpoint().unwrap();
        let mut experiment = Experiment::reset_from_upright_checkpoint(&bytes).unwrap();
        let residual = std::array::from_fn(|i|i as f32*0.005);
        for source in [SourceCommand::Teacher { controls: [0.12;21] },
            SourceCommand::Student { reference:[0.1;21], holding:false },
            SourceCommand::Student { reference:[-0.1;21], holding:true }] {
            experiment.install_command(residual,[0.;2],None,Some(source.clone()));
            let mut restored = Experiment::from_checkpoint(&experiment.checkpoint().unwrap()).unwrap();
            assert_eq!(restored.source_command,Some(source));
            for e in [&mut experiment,&mut restored] { getup::configure(e.runtime.balance_simulation_mut()).unwrap(); }
            assert_eq!(serde_json::to_value(experiment.runtime.simulation().diagnostics.getup.as_ref().unwrap()).unwrap(),
                serde_json::to_value(restored.runtime.simulation().diagnostics.getup.as_ref().unwrap()).unwrap());
            assert_eq!(experiment.runtime.simulation().substeps_completed,0);
            assert_eq!(restored.runtime.simulation().substeps_completed,0);
        }
        experiment.install_command([0.;21],[0.;2],None,None);
        assert!(experiment.source_command.is_none());assert!(experiment.runtime.simulation().balance_source_command.is_none());
        let p = MotionProfile { start_substep:0,q:[0.;21],dq:[0.;21],qdd:[0.;21],torques_nm:[0.;21] };
        experiment.install_command([0.;21],[0.;2],Some(p),None);
        assert!(experiment.runtime.simulation().balance_source_command.is_none());
        assert!(experiment.runtime.simulation().balance_motion_profile.is_some());
        let reset = Experiment::reset_from_upright_checkpoint(&bytes).unwrap();
        assert!(reset.source_command.is_none());assert!(reset.runtime.simulation().balance_source_command.is_none());
    }
    #[test]
    fn source_command_checkpoint_payload_roundtrips_all_reference_values() {
        for source in [SourceCommand::Teacher { controls: [0.37;21] },
            SourceCommand::Student { reference: std::array::from_fn(|i|i as f32*0.03), holding:false },
            SourceCommand::Student { reference: [-0.12;21], holding:true }] {
            let bytes=bincode::serialize(&Some(source.clone())).unwrap();
            let restored:Option<SourceCommand>=bincode::deserialize(&bytes).unwrap();
            assert_eq!(restored,Some(source));
        }
    }
    #[test]
    fn reward_prefers_quiet_anchored_supported_stance_without_integrations() {
        let m = quiet();
        let best = reward(&m, &[0.; 21], [0.; 2]).total;
        let mut drift = m.clone();
        drift.anchor_drift_m = 0.2;
        let mut slip = m.clone();
        slip.loaded_contact_slip_mps = [0.1; 2];
        let mut flight = m.clone();
        flight.foot_loads_n = [0.; 2];
        flight.foot_clearance_m = [0.2; 2];
        let mut scuff = m.clone();
        scuff.foot_loads_n[0] = 0.;
        scuff.foot_horizontal_speed_mps[0] = 0.2;
        for changed in [drift, slip, flight, scuff] {
            assert!(reward(&changed, &[0.; 21], [0.; 2]).total < best);
        }
        let mut unloaded = m.clone();
        unloaded.foot_loads_n[0] = 0.;
        unloaded.foot_clearance_m[0] = 0.03;
        let step = reward(&unloaded, &[0.; 21], [0.2, 0.]).total;
        let hovering = reward(&unloaded, &[0.; 21], [1.2, 0.]).total;
        assert!(hovering < step);
        unloaded.foot_clearance_m[0] = 0.3;
        assert_eq!(reward(&unloaded, &[0.; 21], [0.2, 0.]).total, step);
    }
    #[test]
    fn expanded_actor_matches_pretrained_initialization_without_integrations() {
        let actor = getup::actor::Actor::stationary_initialization();
        let data: serde_json::Value =
            serde_json::from_str(include_str!("../../data/getup-policy/parity.json")).unwrap();
        for row in data["actors"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|r| r["actor"] == "student")
        {
            let input: Vec<f32> = serde_json::from_value(row["input"].clone()).unwrap();
            let mut extended = input[..117].to_vec();
            extended.extend_from_slice(&[0.35, -0.2, input[117]]);
            let expected = getup::actor::student().infer(&input).unwrap();
            let actual = actor.infer(&extended).unwrap();
            assert_eq!(expected.raw, actual.raw);
            assert_eq!(expected.action, actual.action);
        }
    }
    #[test]
    fn invalid_and_empty_budgets_do_not_integrate() {
        assert!(ComputeBudget::new(f64::NAN).is_err());
        assert!(ComputeBudget::new(360.001).is_err());
        let budget = ComputeBudget::new(0.).unwrap();
        assert_eq!(budget.native_substeps, 0);
    }
    #[test]
    fn capsule_clearance_uses_endpoints_and_radius_not_box_corners() {
        let a = Vector::new(-0.1, 0.05, 0.0);
        let b = Vector::new(0.1, 0.08, 0.0);
        assert!((capsule_bottom(a, b, 0.027) - 0.023).abs() < 1e-6);
        assert_eq!(capsule_bottom(a, b, 0.027), capsule_bottom(b, a, 0.027));
    }
}
