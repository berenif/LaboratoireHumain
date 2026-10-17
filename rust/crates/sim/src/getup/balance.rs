//! Native-only, externally driven SAC experiment. No learner or trained balance
//! policy is bundled. The regular simulation loop owns every integration.
use crate::{getup, runtime::Runtime, *};
use serde::{Deserialize, Serialize};
use std::time::Instant;

pub const OBSERVATION_DIM: usize = 120;
pub const ACTION_DIM: usize = 21;
const MAX_EPISODE_SUBSTEPS: u64 = 16_000; // 20 seconds, not a success claim.
const CLEARANCE_M: f32 = 0.015;

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
        bincode::serialize(&(1u32, self.runtime.getup_checkpoint()?, &self.memory))
            .map_err(|e| e.to_string())
    }
    pub fn from_checkpoint(bytes: &[u8]) -> Result<Self, String> {
        use bincode::Options;
        let (schema, runtime, memory): (u32, Vec<u8>, Memory) = bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .with_limit(64 * 1024 * 1024)
            .reject_trailing_bytes()
            .deserialize(bytes)
            .map_err(|e| e.to_string())?;
        if schema != 1 {
            return Err("Unsupported balance checkpoint".into());
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
        runtime.balance_simulation_mut().balance_residual = Some(memory.last_residual);
        Ok(Self { runtime, memory })
    }
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
        self.step_with_profile(action, torso_force_n, None, budget)
    }
    /// Acceleration-predicted inner-impedance reference for one 25ms decision.
    /// All arrays qpos order; original gears, passive elements, and torque caps.
    pub fn step_motion_profile(
        &mut self, action: [f32; 21], torques_nm: [f32; 21],
        qdd: [f32; 21], budget: &mut ComputeBudget,
    ) -> Result<Transition, String> {
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
        self.step_with_profile(action, [0.0; 2], Some(profile), budget)
    }
    fn step_with_profile(
        &mut self, action: [f32; 21], torso_force_n: [f32; 2],
        profile: Option<MotionProfile>, budget: &mut ComputeBudget,
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
        if already_done || self.memory.elapsed_substeps >= MAX_EPISODE_SUBSTEPS {
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
            });
        }
        self.runtime.balance_simulation_mut().balance_residual = Some(action);
        self.runtime.balance_simulation_mut().balance_motion_profile = profile;
        self.runtime.balance_simulation_mut().balance_torso_force_n = torso_force_n;
        self.memory.last_residual = action;
        let started = Instant::now();
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
            memory.max_anchor_drift_m = memory.max_anchor_drift_m.max(m.anchor_drift_m);
            native_foot_samples.push(NativeFootSample {
                native_substep: s.substeps_completed,
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
                    || self.memory.elapsed_substeps >= MAX_EPISODE_SUBSTEPS),
            failure,
            budget_physics_wall_s: budget.used_s,
            native_foot_samples,
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
