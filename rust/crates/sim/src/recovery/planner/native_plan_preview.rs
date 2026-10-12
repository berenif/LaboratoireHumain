//! Complete static-scene snapshots and a fail-closed preview orchestration shell.
//!
//! The native driver reuses Simulation::advance_tick_observed unchanged, with
//! all four steps reserved before each tick. It never duplicates the integration
//! kernel. Native admission remains closed until a shared experimental owner,
//! frozen prerequisites and measured actual/copy parity have been established.

use crate::Simulation;
use rapier3d::prelude::{PhysicsPipeline, PhysicsWorld};
use serde::Serialize;

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum PreviewError {
    Snapshot(String),
    InvalidHorizon,
    ClockOverflow,
    Reservation(String),
    Kernel(String),
    ClockMismatch { expected: u64, actual: u64 },
    NativeKernelParityUnavailable,
    ParityMismatch(String),
}

/// Constructed only by comparing a complete native four-substep parity trace.
/// This is evidence for the measured kernel/state, not a blanket physical
/// success certificate. The batch ledger separately admits each frozen run.
#[derive(Debug, Serialize)]
pub(crate) struct NativeKernelAdmission {
    initial_substep: u64,
    verified_substeps: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
struct ParityFrame {
    native_substep: u64,
    world: Vec<u8>,
    state: Vec<u8>,
    owner_memory: Vec<u8>,
    /// These numeric fields are compared bitwise rather than through JSON,
    /// which does not distinguish all nonfinite values or signed zeros.
    body_and_motor_bits: Vec<u32>,
}

fn parity_frame(s: &Simulation, memory: &impl Serialize) -> Result<ParityFrame, PreviewError> {
    let serial = |v| {
        serde_json::to_vec(v)
            .map_err(|e| PreviewError::Snapshot(format!("Parity serialization: {e}")))
    };
    let mut bits = Vec::new();
    for handle in &s.bodies {
        let body = &s.world.bodies[*handle];
        bits.extend(body.translation().to_array().map(f32::to_bits));
        let q = body.rotation();
        bits.extend([q.x, q.y, q.z, q.w].map(f32::to_bits));
        bits.extend(body.linvel().to_array().map(f32::to_bits));
        bits.extend(body.angvel().to_array().map(f32::to_bits));
    }
    for handle in s.joints.iter().flatten() {
        let joint = s
            .world
            .impulse_joints
            .get(*handle)
            .ok_or_else(|| PreviewError::Snapshot("Missing native parity joint".into()))?;
        for motor in &joint.data.motors {
            bits.extend(
                [
                    motor.target_pos,
                    motor.target_vel,
                    motor.stiffness,
                    motor.damping,
                    motor.max_force,
                    motor.impulse,
                    motor.delivered_abs_impulse,
                ]
                .map(f32::to_bits),
            );
            bits.extend(
                motor
                    .delivered_angular_impulse_1
                    .to_array()
                    .map(f32::to_bits),
            );
            bits.extend(
                motor
                    .delivered_angular_impulse_2
                    .to_array()
                    .map(f32::to_bits),
            );
        }
    }
    let state = serde_json::json!({
        "snapshot": s.snapshot(), "substeps": s.substeps_completed,
        "targets": s.targets, "restTargets": s.rest_targets,
        "persistence": s.persistence,
        "integratedSupportSubsteps": s.integrated_support_substeps,
        "integratedSupportLoadsN": s.integrated_support_loads_n,
        "recovery": s.recovery_status(),
        "coordination": s.coordination.as_ref().map(|owner| owner.evidence()),
    });
    Ok(ParityFrame {
        native_substep: s.substeps_completed,
        world: serde_json::to_vec(&s.world).map_err(|e| PreviewError::Snapshot(e.to_string()))?,
        state: serial(&state)?,
        owner_memory: serde_json::to_vec(memory)
            .map_err(|e| PreviewError::Snapshot(e.to_string()))?,
        body_and_motor_bits: bits,
    })
}

/// Frames can only be appended from actual Simulation states at consecutive
/// integrated substeps. The caller archives both traces with the frozen receipt.
#[derive(Debug, Serialize)]
pub(crate) struct NativeParityTrace {
    #[serde(skip)]
    simulation_identity: usize,
    initial: ParityFrame,
    frames: Vec<ParityFrame>,
}
impl NativeParityTrace {
    pub(crate) fn begin(s: &Simulation, memory: &impl Serialize) -> Result<Self, PreviewError> {
        if s.tick.checked_mul(4) != Some(s.substeps_completed) {
            return Err(PreviewError::Snapshot(
                "Parity starts at a completed tick".into(),
            ));
        }
        Ok(Self {
            simulation_identity: std::ptr::from_ref(s) as usize,
            initial: parity_frame(s, memory)?,
            frames: Vec::new(),
        })
    }
    pub(crate) fn record(
        &mut self,
        s: &Simulation,
        memory: &impl Serialize,
    ) -> Result<(), PreviewError> {
        if self.simulation_identity != std::ptr::from_ref(s) as usize {
            return Err(PreviewError::ParityMismatch(
                "Parity trace changed Simulation instance".into(),
            ));
        }
        let expected = self
            .initial
            .native_substep
            .checked_add(self.frames.len() as u64 + 1)
            .ok_or(PreviewError::ClockOverflow)?;
        if self.frames.len() >= 4 || s.substeps_completed != expected {
            return Err(PreviewError::ClockMismatch {
                expected,
                actual: s.substeps_completed,
            });
        }
        self.frames.push(parity_frame(s, memory)?);
        Ok(())
    }
}

pub(crate) fn native_kernel_admission(
    predicted: &NativeParityTrace,
    actual: &NativeParityTrace,
) -> Result<NativeKernelAdmission, PreviewError> {
    if predicted.frames.len() != 4 || actual.frames.len() != 4 {
        return Err(PreviewError::NativeKernelParityUnavailable);
    }
    if predicted.simulation_identity == actual.simulation_identity {
        return Err(PreviewError::ParityMismatch(
            "Parity requires two distinct Simulation instances".into(),
        ));
    }
    if predicted.initial != actual.initial || predicted.frames != actual.frames {
        return Err(PreviewError::ParityMismatch(
            "Native world, commands, deliveries, contacts, owner memory or clocks differ".into(),
        ));
    }
    Ok(NativeKernelAdmission {
        initial_substep: actual.initial.native_substep,
        verified_substeps: 4,
    })
}

/// Every field of Simulation is copied explicitly. New fields require a compile
/// error here instead of silently disappearing from the forecast's state.
pub(crate) fn copy_static_snapshot(source: &Simulation) -> Result<Simulation, PreviewError> {
    let refuse = |s: &str| PreviewError::Snapshot(s.into());
    super::contact_observation::require_inactive().map_err(PreviewError::Snapshot)?;
    if source.grab.is_some() || source.striker.is_some() || source.environment.is_some() {
        return Err(refuse(
            "Preview admits only a static scene without interactions",
        ));
    }
    if source.first_failure.is_some() || source.motion == lh_contracts::Motion::Halted {
        return Err(refuse(
            "Interrupted/failed histories are not preview snapshots",
        ));
    }
    if source.profile.dt_s != lh_contracts::SUBSTEP_S || source.profile.tick_substeps != 4 {
        return Err(refuse(
            "Preview requires the original 240 Hz / four-substep clock",
        ));
    }
    if source.tick.checked_mul(4) != Some(source.substeps_completed) {
        return Err(refuse(
            "Snapshot requires a completed tick; intra-tick accumulators are local",
        ));
    }
    if source.model.segments.len() != 25
        || source.bodies.len() != 25
        || source.colliders.len() != 25
        || source.joints.len() != 25
        || source.world.bodies.len() != 25
        || source.world.colliders.len() != 26
        || source.world.multibody_joints.iter().next().is_some()
        || source
            .bodies
            .iter()
            .any(|h| source.world.bodies.get(*h).is_none())
        || source
            .colliders
            .iter()
            .any(|h| source.world.colliders.get(*h).is_none())
        || !source
            .world
            .colliders
            .get(source.floor)
            .is_some_and(|floor| floor.parent().is_none() && floor.is_enabled())
    {
        return Err(refuse(
            "Preview requires the complete native body and one enabled static floor",
        ));
    }
    // The Rapier pipeline contains transient workspace, not persistent physical
    // state. This follows the previously audited copy_world boundary, preserving
    // all contact, joint, CCD, broad-phase and island caches without serde loss.
    let world = PhysicsWorld {
        gravity: source.world.gravity,
        integration_parameters: source.world.integration_parameters,
        physics_pipeline: PhysicsPipeline::new(),
        islands: source.world.islands.clone(),
        broad_phase: source.world.broad_phase.clone(),
        narrow_phase: source.world.narrow_phase.clone(),
        bodies: source.world.bodies.clone(),
        colliders: source.world.colliders.clone(),
        impulse_joints: source.world.impulse_joints.clone(),
        multibody_joints: source.world.multibody_joints.clone(),
        ccd_solver: source.world.ccd_solver.clone(),
    };
    Ok(Simulation {
        world,
        model: source.model.clone(),
        bodies: source.bodies.clone(),
        colliders: source.colliders.clone(),
        joints: source.joints.clone(),
        floor: source.floor,
        filter: source.filter.clone(),
        profile: source.profile.clone(),
        tick: source.tick,
        generation: source.generation,
        motion: source.motion,
        counters: source.counters.clone(),
        targets: source.targets.clone(),
        rest_targets: source.rest_targets.clone(),
        rest_com: source.rest_com,
        rest_com_roundoff: source.rest_com_roundoff,
        heading: source.heading,
        grab: None,
        last_press: source.last_press,
        applied_sequence: source.applied_sequence,
        contacts: source.contacts.clone(),
        persistence: source.persistence.clone(),
        integrated_support_substeps: source.integrated_support_substeps,
        integrated_support_loads_n: source.integrated_support_loads_n,
        diagnostics: source.diagnostics.clone(),
        first_failure: source.first_failure.clone(),
        motor_ceiling_sum: source.motor_ceiling_sum,
        substeps_completed: source.substeps_completed,
        stepping: source.stepping.clone(),
        motion_observation: source.motion_observation.clone(),
        environment: None,
        striker: None,
        recovery: source.recovery.clone(),
        coordination: source.coordination.clone(),
    })
}

/// M must have value semantics: cloning it must not share mutable controller
/// state with the real history. The common runtime Memory is a value-only type;
/// Clone alone cannot guarantee this for an arbitrary Arc<Mutex<_>> supplied
/// by a future caller. Immutable shared plan data is allowed.
pub(crate) struct StaticPreviewBranch<M> {
    simulation: Simulation,
    memory: M,
}

impl<M: Clone> StaticPreviewBranch<M> {
    pub(crate) fn capture(source: &Simulation, memory: &M) -> Result<Self, PreviewError> {
        Ok(Self {
            simulation: copy_static_snapshot(source)?,
            memory: memory.clone(),
        })
    }

    pub(crate) fn simulation(&self) -> &Simulation {
        &self.simulation
    }
    pub(crate) fn memory(&self) -> &M {
        &self.memory
    }

    pub(crate) fn set_feedback_scale(&mut self, scale: f64) -> Result<(), PreviewError> {
        if ![0.0, 0.5, 1.0, 1.5].contains(&scale) {
            return Err(PreviewError::Snapshot(
                "Feedback proposal outside the frozen family".into(),
            ));
        }
        let owner =
            self.simulation.coordination.as_mut().ok_or_else(|| {
                PreviewError::Snapshot("Missing copied coordination owner".into())
            })?;
        owner.feedback_scale = scale;
        Ok(())
    }

    /// Reuse the exact existing native tick kernel, including all original
    /// observers, integrity checks and clocks. The callback only observes the
    /// copied simulation; decision/installation must belong to the same explicit
    /// Simulation owner used in the actual history. No recursive plan search.
    /// This method remains inaccessible without measured native admission.
    pub(crate) fn preview<O>(
        &mut self,
        _admission: &NativeKernelAdmission,
        branch_id: u64,
        steps: u32,
        budget: &mut impl PreviewBudget,
        observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
    ) -> PreviewRun<O> {
        if !(120..=480).contains(&steps) || steps % 4 != 0 {
            return PreviewRun::refused(PreviewError::InvalidHorizon);
        }
        self.preview_ticks(PreviewPurpose::Candidate, branch_id, steps, budget, observe)
    }

    /// Only the ledger's frozen verification phase may use this entrypoint.
    /// Its four steps are charged to technical verification, not free trials.
    /// Histories/candidate selection still require a measured parity receipt.
    pub(crate) fn verify_tick<O>(
        &mut self,
        branch_id: u64,
        budget: &mut impl PreviewBudget,
        observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
    ) -> PreviewRun<O> {
        self.preview_ticks(
            PreviewPurpose::ParityVerification,
            branch_id,
            4,
            budget,
            observe,
        )
    }

    fn preview_ticks<O>(
        &mut self,
        purpose: PreviewPurpose,
        branch_id: u64,
        steps: u32,
        budget: &mut impl PreviewBudget,
        mut observe: impl FnMut(&Simulation, u32, &mut M) -> Result<O, String>,
    ) -> PreviewRun<O> {
        if let Err(error) = budget.admit_preview(purpose, steps) {
            return PreviewRun::refused(PreviewError::Reservation(error));
        }
        let mut run = PreviewRun {
            records: Vec::new(),
            refusal: None,
        };
        for preview_index in (0..steps).step_by(4) {
            if let Err(error) = self.simulation.require_step_context() {
                run.refusal = Some(PreviewError::Snapshot(error));
                break;
            }
            let before_substep = self.simulation.substeps_completed;
            if self.simulation.tick.checked_mul(4) != Some(before_substep) {
                run.refusal = Some(PreviewError::Snapshot(
                    "Native preview requires a complete tick boundary".into(),
                ));
                break;
            }
            let Some(expected) = before_substep.checked_add(4) else {
                run.refusal = Some(PreviewError::ClockOverflow);
                break;
            };
            if let Err(error) = budget.reserve_four_before_tick(StepReservation {
                branch_id,
                before_substep,
                preview_index,
                purpose,
            }) {
                run.refusal = Some(PreviewError::Reservation(error));
                break;
            }
            let memory = &mut self.memory;
            let result = self.simulation.advance_tick_observed(|state, substep| {
                let fail = |check, reason| {
                    crate::measurement::Failure::new(state.tick, substep, check, reason, 1.0, 0.0)
                };
                if state.substeps_completed != before_substep + u64::from(substep) + 1 {
                    return Err(fail(
                        "preview-substep-clock",
                        "Native observer clock mismatch".into(),
                    ));
                }
                state
                    .require_step_context()
                    .map_err(|e| fail("preview-context", e))?;
                let record =
                    observe(state, substep, memory).map_err(|e| fail("preview-observer", e))?;
                run.records.push(record);
                // The first deadline check belongs to the reservation. Later
                // checks occur after each original observer, before continuing
                // to another integration. An in-flight native step cannot be
                // interrupted, and its full reservation is already consumed.
                budget
                    .check_wall_deadline()
                    .map_err(|e| fail("preview-wall-budget", e))?;
                Ok(())
            });
            if let Err(failure) = result {
                run.refusal = Some(PreviewError::Kernel(format!("{failure:?}")));
                break;
            }
            if self.simulation.substeps_completed != expected {
                run.refusal = Some(PreviewError::ClockMismatch {
                    expected,
                    actual: self.simulation.substeps_completed,
                });
                break;
            }
        }
        run
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PreviewPurpose {
    ParityVerification,
    Candidate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct StepReservation {
    pub branch_id: u64,
    /// Absolute physical clock of the copied history, before the next step.
    pub before_substep: u64,
    pub preview_index: u32,
    pub purpose: PreviewPurpose,
}

/// Implement with the ONE durable batch ledger, not a fresh per-copy counter.
/// Check the global wall deadline and reserve one step durably BEFORE returning
/// Ok. No cancellation/refund is provided: failures retain their reservation.
pub(crate) trait PreviewBudget {
    /// Must verify the frozen prerequisites and the batch phase. Verification
    /// requires its dedicated technical allowance; Candidate requires that the
    /// ledger has accepted the measured parity receipt for this same frozen run.
    fn admit_preview(&mut self, purpose: PreviewPurpose, steps: u32) -> Result<(), String>;
    fn reserve_one_before_step(&mut self, request: StepReservation) -> Result<(), String>;
    /// Check only: never reserves again for an already-reserved substep.
    fn check_wall_deadline(&mut self) -> Result<(), String>;
    /// A ledger adapter may override with one atomic four-step reservation.
    /// Default reservations all precede integration; partial reservation failure
    /// executes no tick and intentionally does not refund earlier reservations.
    fn reserve_four_before_tick(&mut self, request: StepReservation) -> Result<(), String> {
        request
            .before_substep
            .checked_add(4)
            .ok_or("Preview clock overflow")?;
        request
            .preview_index
            .checked_add(3)
            .ok_or("Preview index overflow")?;
        self.check_wall_deadline()?;
        for offset in 0..4_u32 {
            self.reserve_one_before_step(StepReservation {
                branch_id: request.branch_id,
                before_substep: request.before_substep + u64::from(offset),
                preview_index: request.preview_index + offset,
                purpose: request.purpose,
            })?;
        }
        Ok(())
    }
}

trait NativeStepClock {
    fn completed_native_substeps(&self) -> u64;
    fn require_step_context(&self) -> Result<(), String> {
        Ok(())
    }
}
impl NativeStepClock for Simulation {
    fn completed_native_substeps(&self) -> u64 {
        self.substeps_completed
    }
    fn require_step_context(&self) -> Result<(), String> {
        super::contact_observation::require_inactive()?;
        if self.grab.is_some()
            || self.striker.is_some()
            || self.environment.is_some()
            || self.first_failure.is_some()
            || self.motion == lh_contracts::Motion::Halted
        {
            return Err("Static preview context changed or history stopped".into());
        }
        Ok(())
    }
}

pub(crate) struct PreviewRun<O> {
    pub records: Vec<O>,
    /// Completed records remain available on refusal; the branch retains the
    /// last state for failure archiving. Nothing is transplanted into reality.
    pub refusal: Option<PreviewError>,
}
impl<O> PreviewRun<O> {
    fn refused(error: PreviewError) -> Self {
        Self {
            records: Vec::new(),
            refusal: Some(error),
        }
    }
}

/// Private generic shell is exercised only with fake clocks in this version.
/// Counter equality detects a broken callback; it cannot undo an over-integrating
/// callback. The closed native gate is essential until an exactly-one-step shared
/// kernel exists and is certified, rather than trusting an arbitrary closure.
fn run_guarded_steps<S: NativeStepClock, M, O>(
    state: &mut S,
    memory: &mut M,
    branch_id: u64,
    steps: u32,
    budget: &mut impl PreviewBudget,
    mut kernel: impl FnMut(&mut S, &mut M) -> Result<O, String>,
) -> PreviewRun<O> {
    if steps == 0 || steps > 480 {
        return PreviewRun::refused(PreviewError::InvalidHorizon);
    }
    let mut run = PreviewRun {
        records: Vec::new(),
        refusal: None,
    };
    for preview_index in 0..steps {
        if let Err(error) = state.require_step_context() {
            run.refusal = Some(PreviewError::Snapshot(error));
            break;
        }
        let before_substep = state.completed_native_substeps();
        let Some(expected) = before_substep.checked_add(1) else {
            run.refusal = Some(PreviewError::ClockOverflow);
            break;
        };
        if let Err(error) = budget.reserve_one_before_step(StepReservation {
            branch_id,
            before_substep,
            preview_index,
            purpose: PreviewPurpose::ParityVerification,
        }) {
            run.refusal = Some(PreviewError::Reservation(error));
            break;
        }
        let result = kernel(state, memory);
        let actual = state.completed_native_substeps();
        if actual != expected && !(actual == before_substep && result.is_err()) {
            run.refusal = Some(PreviewError::ClockMismatch { expected, actual });
            break;
        }
        match result {
            Ok(record) => run.records.push(record),
            Err(error) => {
                run.refusal = Some(PreviewError::Kernel(error));
                break;
            }
        }
    }
    run
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FakeClock(u64);
    impl NativeStepClock for FakeClock {
        fn completed_native_substeps(&self) -> u64 {
            self.0
        }
    }
    #[derive(Default)]
    struct FakeBudget {
        admitted: usize,
        maximum: usize,
        requests: Vec<StepReservation>,
    }
    impl PreviewBudget for FakeBudget {
        fn admit_preview(&mut self, _: PreviewPurpose, _: u32) -> Result<(), String> {
            Ok(())
        }
        fn check_wall_deadline(&mut self) -> Result<(), String> {
            Ok(())
        }
        fn reserve_one_before_step(&mut self, request: StepReservation) -> Result<(), String> {
            if self.admitted >= self.maximum {
                return Err("global ceiling/deadline".into());
            }
            self.admitted += 1;
            self.requests.push(request);
            Ok(())
        }
    }

    #[test]
    fn admission_requires_four_measured_native_frames() {
        let source = Simulation::new(0.0, crate::Profile::default(), true).unwrap();
        let trace = NativeParityTrace::begin(&source, &()).unwrap();
        assert!(matches!(
            native_kernel_admission(&trace, &trace),
            Err(PreviewError::NativeKernelParityUnavailable)
        ));
    }

    #[test]
    fn static_snapshot_copies_world_and_controller_memory_without_integration() {
        // Construction/configuration only. No step, advance or predictor call.
        let mut source = Simulation::new(0.0, crate::Profile::default(), true).unwrap();
        source.recovery = Some(crate::recovery::Controller::new(&source).unwrap());
        source.recovery.as_mut().unwrap().status.phase_time_s = 0.37;
        source.recovery.as_mut().unwrap().stable_substeps = 17;
        source.recovery.as_mut().unwrap().handoff_substeps = 23;
        source.recovery.as_mut().unwrap().settle_substeps = 31;
        source.integrated_support_substeps[3] = 7;
        source.integrated_support_loads_n[3] = 2.5;
        source.persistence[3] = 0.125;
        source.counters.falls = 2;
        let memory = vec![2_u64, 7, 11];
        let before_world = serde_json::to_value(&source.world).unwrap();
        let before_status =
            serde_json::to_value(&source.recovery.as_ref().unwrap().status).unwrap();
        let mut branch = StaticPreviewBranch::capture(&source, &memory).unwrap();
        assert_eq!(
            serde_json::to_value(&branch.simulation().world).unwrap(),
            before_world
        );
        assert_eq!(
            serde_json::to_value(&branch.simulation().recovery.as_ref().unwrap().status).unwrap(),
            before_status
        );
        let copied_controller = branch.simulation().recovery.as_ref().unwrap();
        let source_controller = source.recovery.as_ref().unwrap();
        assert_eq!(copied_controller.targets, source_controller.targets);
        assert_eq!(copied_controller.velocities, source_controller.velocities);
        assert_eq!(
            copied_controller.best_progress.to_bits(),
            source_controller.best_progress.to_bits()
        );
        assert_eq!(
            (
                copied_controller.stable_substeps,
                copied_controller.handoff_substeps,
                copied_controller.settle_substeps
            ),
            (17, 23, 31)
        );
        assert_eq!(
            branch.simulation().filter.exclusions,
            source.filter.exclusions
        );
        assert_eq!(branch.simulation().targets, source.targets);
        assert_eq!(branch.simulation().rest_targets, source.rest_targets);
        assert_eq!(
            branch.simulation().integrated_support_substeps,
            source.integrated_support_substeps
        );
        assert_eq!(
            branch.simulation().integrated_support_loads_n,
            source.integrated_support_loads_n
        );
        assert_eq!(branch.simulation().persistence, source.persistence);
        assert_eq!(branch.memory(), &memory);
        // Changes to copied memory and world data cannot change the source.
        // These are synthetic state edits, still with zero native integrations.
        branch.simulation.world.gravity.x = 1.0;
        branch
            .simulation
            .recovery
            .as_mut()
            .unwrap()
            .status
            .phase_time_s = 9.0;
        branch.simulation.persistence[3] = 0.0;
        branch.memory[0] = 99;
        assert_eq!(serde_json::to_value(&source.world).unwrap(), before_world);
        assert_eq!(
            serde_json::to_value(&source.recovery.as_ref().unwrap().status).unwrap(),
            before_status
        );
        assert_eq!(source.persistence[3], 0.125);
        assert_eq!(memory, [2, 7, 11]);
        assert_eq!(
            (
                source.substeps_completed,
                branch.simulation.substeps_completed
            ),
            (0, 0)
        );
        source.substeps_completed = 1;
        assert!(matches!(
            copy_static_snapshot(&source),
            Err(PreviewError::Snapshot(_))
        ));
    }

    #[test]
    fn fake_kernel_consumes_one_reservation_per_step_and_preserves_partial_trace() {
        let mut state = FakeClock(24);
        let mut memory = 0;
        let mut budget = FakeBudget {
            maximum: 2,
            ..Default::default()
        };
        let run = run_guarded_steps(&mut state, &mut memory, 9, 4, &mut budget, |s, m| {
            s.0 += 1;
            *m += 1;
            Ok(s.0)
        });
        assert_eq!(run.records, [25, 26]);
        assert!(matches!(run.refusal, Some(PreviewError::Reservation(_))));
        assert_eq!((state.0, memory, budget.admitted), (26, 2, 2));
        assert_eq!(
            budget.requests[1],
            StepReservation {
                branch_id: 9,
                before_substep: 25,
                preview_index: 1,
                purpose: PreviewPurpose::ParityVerification
            }
        );
    }

    #[test]
    fn refused_reservation_does_not_invoke_kernel_or_change_memory() {
        let mut state = FakeClock(0);
        let mut memory = 0;
        let mut budget = FakeBudget::default();
        let run: PreviewRun<()> =
            run_guarded_steps(&mut state, &mut memory, 0, 1, &mut budget, |_, _| {
                panic!("unreserved callback")
            });
        assert!(matches!(run.refusal, Some(PreviewError::Reservation(_))));
        assert_eq!((state.0, memory), (0, 0));
    }

    #[test]
    fn partial_four_step_reservation_is_retained_without_starting_a_tick() {
        let mut budget = FakeBudget {
            maximum: 3,
            ..Default::default()
        };
        let request = StepReservation {
            branch_id: 4,
            before_substep: 24,
            preview_index: 0,
            purpose: PreviewPurpose::ParityVerification,
        };
        assert!(budget.reserve_four_before_tick(request).is_err());
        assert_eq!(budget.admitted, 3);
        assert_eq!(
            budget
                .requests
                .iter()
                .map(|r| r.before_substep)
                .collect::<Vec<_>>(),
            [24, 25, 26]
        );
        // No native state or kernel is needed to reject an incomplete batch.
        assert!(budget.reserve_four_before_tick(request).is_err());
        assert_eq!(budget.admitted, 3);
    }

    #[test]
    fn clock_violation_and_kernel_error_never_refund_a_reservation() {
        let mut state = FakeClock(0);
        let mut budget = FakeBudget {
            maximum: 4,
            ..Default::default()
        };
        let run = run_guarded_steps(&mut state, &mut (), 0, 2, &mut budget, |s, _| {
            s.0 += 2;
            Ok(())
        });
        assert_eq!(
            run.refusal,
            Some(PreviewError::ClockMismatch {
                expected: 1,
                actual: 2
            })
        );
        assert_eq!(budget.admitted, 1);
        let run: PreviewRun<()> =
            run_guarded_steps(&mut state, &mut (), 1, 2, &mut budget, |s, _| {
                s.0 += 1;
                Err("observer refuses integrated state".into())
            });
        assert!(matches!(run.refusal, Some(PreviewError::Kernel(_))));
        assert_eq!(budget.admitted, 2);
    }

    #[test]
    fn bad_horizon_and_clock_overflow_refuse_before_reservation() {
        let mut state = FakeClock(u64::MAX);
        let mut budget = FakeBudget {
            maximum: 1,
            ..Default::default()
        };
        for steps in [0, 481, 1] {
            let run: PreviewRun<()> =
                run_guarded_steps(&mut state, &mut (), 0, steps, &mut budget, |_, _| {
                    panic!("invalid clock/horizon callback")
                });
            assert!(run.refusal.is_some());
        }
        assert_eq!(budget.admitted, 0);
    }
}
