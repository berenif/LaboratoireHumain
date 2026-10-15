//! Opt-in command owner for the frozen transition, shared by real and copied worlds.
//! Native prediction selection lives outside this controller, so previews cannot recurse.
use super::{
    coordinated_runtime as runtime, geometric_tasks as tasks, native_observation, support_state,
};
use crate::{Simulation, recovery, recovery::trajectory_program::CompiledProgram};
use rapier3d::prelude::*;
use serde::Deserialize;
use serde_json::{Value, json};

#[derive(Clone, Deserialize, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct ReferenceTasks {
    pub rows: Vec<ReferenceRow>,
    pub material_targets: Vec<MaterialTarget>,
}
#[derive(Clone, Deserialize, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct MaterialTarget {
    pub segment_index: usize,
    pub local_point_m: [f64; 3],
}
#[derive(Clone, Deserialize, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct ReferenceRow {
    pub substep: u64,
    pub com_position_m: [f64; 3],
    pub com_velocity_mps: [f64; 3],
    pub feet: Vec<FootTarget>,
}
#[derive(Clone, Deserialize, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct FootTarget {
    pub side: String,
    pub hindfoot_rotation_xyzw: [f64; 4],
    pub forefoot_rotation_xyzw: [f64; 4],
}

pub(crate) const PARAMETERS: &str = r#"{"version":"coordinated-transition-v2-numeric","solver":"feasible-active-set-newton","commandCompilation":"direct-exact-then-one-ulp-interior","maximumSolvesPerTick":2,"controlHz":60,"previewHz":10,"previewSteps":480,"candidates":[0.0,0.5,1.0,1.5],"comPositionGainPerS":2.0,"comSpeedLimitMps":0.15,"footRotationGainPerS":2.0,"posturePositionGainPerS":2.0,"postureWeight":0.25,"regularization":0.05,"hardMaterialVelocityToleranceMps":0.005,"materialMatchToleranceM":0.012,"maximumTargetOffsetRad":0.12,"iterations":512,"supportProof":"original thresholds unchanged"}"#;

pub(crate) const TRANSFER_PARAMETERS: &str = r#"{"version":"observed-bilateral-transfer-v1","solver":"feasible-active-set-newton","commandCompilation":"direct-exact-then-one-ulp-interior","maximumSolvesPerTick":2,"iterations":512,"controlHz":60,"previewMaximumHz":10,"previewMinimumSteps":120,"previewMaximumSteps":480,"realWindowMaximumSteps":24,"feedbackScale":1.0,"supportPolicy":"ContinueBilateral","localMilestoneDeadlineAfterEntrySteps":480,"originalTerminalDeadlineSteps":6000,"additionalTerminalMinimumSteps":240,"additionalTerminalMaximumSteps":243,"terminalStop":"first-complete-tick-after-minimum","materialRows":"unvalidated-command-guides","materialGuideWeight":1.0,"originalMaterialBandMps":0.005,"comPositionGainPerS":2.0,"comSpeedLimitMps":0.15,"footRotationGainPerS":2.0,"posturePositionGainPerS":2.0,"postureWeight":0.25,"regularization":0.05,"materialMatchToleranceM":0.012,"maximumTargetOffsetRad":0.12,"candidateDriftLimit":"original-contract-idle-planted-foot-drift","supportProof":"native-material-observations-original-diagnostic"}"#;

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
    last_attempt: Value,
    pub(crate) transfer: Option<super::transfer_adapter::Adapter>,
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
            last_attempt: Value::Null,
            transfer: None,
        })
    }
    pub(crate) fn evidence(&self) -> Value {
        json!({"enabled":self.enabled,"feedbackScale":self.feedback_scale,"start":self.start,"previousStep":self.previous_step,
            "q":self.memory.previous,"nextSubstep":self.memory.next_substep,"rates":self.memory.rates.to_vec(),
            "patches":self.patches,"bestProgress":self.best_progress,"stalledS":self.stalled_s,"unsupportedS":self.unsupported_s,"maximumCapNm":self.maximum_cap,"last":self.last,"numericAttempt":self.last_attempt,
            "transfer":self.transfer.as_ref().map(|a|a.evidence())})
    }
    pub(super) fn immutable_transfer_scope(&self)->Value {
        json!({"controllerSchema":"observed-transfer-v1","parameters":TRANSFER_PARAMETERS,
            "candidateDriftLimitM":crate::recovery_contract::contract().acceptance.idle_planted_foot_drift_m,
            "reference":self.reference,"enabled":self.enabled,"feedbackScale":self.feedback_scale,
            "adapter":self.transfer.as_ref().map(|a|a.immutable_scope())})
    }

    /// Rebuild the same numeric task from observed body states and retained patches.
    /// This only reads geometry; it never integrates or changes a body.
    fn problem_observed(
        &self,
        model: &lh_model::Model,
        heading: Rotation,
        observed: &tasks::ObservedState,
        step: u64,
    ) -> Result<super::whole_body_tasks::Problem, String> {
        let nominal = self.program.command(step as u32);
        let reference =
            &self.reference.rows[(step / 4).min(self.reference.rows.len() as u64 - 1) as usize];
        let rotated = |a: [f64; 3]| heading * Vector::from_array(a.map(|v| v as f32));
        let com_ref = rotated(reference.com_position_m);
        let v_ref = rotated(reference.com_velocity_mps);
        let error = com_ref - observed.center_of_mass();
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
                let segment = model
                    .segments
                    .iter()
                    .position(|part| part.id == format!("{}{suffix}", foot.side))
                    .ok_or("Missing reference foot body")?;
                let desired = heading * Rotation::from_array(target.map(|v| v as f32));
                let actual = observed.bodies[segment].rotation();
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
        let layout = runtime::axes(model)?;
        let mut problem = tasks::assemble_observed(model, observed, &self.patches.active, &goals)?;
        for (i, a) in layout.iter().enumerate() {
            let previous = self.memory.previous[a.segment][a.coordinate] as f64;
            let target = nominal.positions[a.segment][a.coordinate] as f64;
            let dt = 4. * lh_contracts::SUBSTEP_S as f64;
            problem.lower[i] =
                problem.lower[i].max(((target - 0.12).max(a.minimum as f64) - previous) / dt);
            problem.upper[i] =
                problem.upper[i].min(((target + 0.12).min(a.maximum as f64) - previous) / dt);
        }
        Ok(problem)
    }

    fn problem(
        &self,
        sim: &Simulation,
        step: u64,
    ) -> Result<super::whole_body_tasks::Problem, String> {
        let observed = tasks::ObservedState::from_simulation(sim)?;
        self.problem_observed(&sim.model, sim.heading, &observed, step)
    }

    /// Static entry point for the reformulated command generator. It shares
    /// this owner's reference, q/dq memory, observed geometry and patch ids,
    /// but does not install commands or replace the closed family's executor.
    /// The returned type explicitly requires a new native preview admission.
    pub(crate) fn prepare_command_proposal_observed(
        &self,
        model: &lh_model::Model,
        heading: Rotation,
        observed: &tasks::ObservedState,
    ) -> Result<super::command_proposal::CommandProposal, String> {
        let step = observed.native_substep.checked_sub(self.start)
            .ok_or("Proposal observation precedes owner start")?;
        if !self.enabled || observed.native_substep != self.memory.next_substep {
            return Err("Proposal requires the enabled owner's current command clock".into());
        }
        let original = self.problem_observed(model, heading, observed, step)?;
        super::command_proposal::prepare(
            model, &self.memory, &original, &self.patches.active, observed.native_substep,
        )
    }

    /// Attach at a completed, already integrated observation. Neither bodies,
    /// targets, recovery clocks nor the existing observer are reset here.
    pub(crate) fn enable_observed_transfer(
        &mut self,
        model: &lh_model::Model,
        observation: &support_state::SupportObservation,
        entry: super::observed_raccord::ObservedCommandEntry,
    ) -> Result<(), String> {
        if self.transfer.is_some() || self.feedback_scale!=1.0 || self.memory.next_substep != observation.native_substep
            || entry.native_substep != observation.native_substep
            || entry.previous_q.iter().flatten().zip(self.memory.previous.iter().flatten()).any(|(a,b)|a.to_bits()!=b.to_bits())
            || self.previous_step.and_then(|s|s.checked_add(self.start+1)) != Some(observation.native_substep)
            || self.program.duration_substeps()!=1920
        { return Err("Transfer attachment differs from existing observed owner history".into()); }
        let previous_dq: crate::recovery::trajectory_program::Coordinates =
            serde_json::from_value(self.last["velocities"].clone()).map_err(|_|"Missing last installed command velocities")?;
        if previous_dq.iter().flatten().zip(entry.previous_dq.iter().flatten()).any(|(a,b)|a.to_bits()!=b.to_bits()) {
            return Err("Transfer entry dq is not the existing owner's last installed dq".into());
        }
        let tracker = if self.patches.active.is_empty() {
            super::support_schedule::SupportSchedule::bootstrap_observed(observation,&[19,24])?.tracker().clone()
        } else { self.patches.clone() };
        let timing = super::continuation::Timing { heels_acquired:168,recentered_hold_started:300,
            recentered_hold_completed:432,standing_reference:1680,hold_completed:1920 };
        let deadline = observation.native_substep.checked_add(480).ok_or("Transfer deadline overflow")?;
        let adapter=super::transfer_adapter::Adapter::new(model,observation,tracker,entry,
            self.program.clone(),timing,
            super::load_transfer::TransferIntent::from_archived_heel_reference(super::load_transfer::AfterHeels::ContinueBilateral,deadline),self.start)?;
        self.transfer=Some(adapter);
        self.enabled=true;
        Ok(())
    }

    fn transfer_problem_observed(
        &self, model:&lh_model::Model, heading:Rotation, observed:&tasks::ObservedState,
        adapter:&super::transfer_adapter::Adapter, nominal:recovery::trajectory_program::Command,
        proposal:&super::load_transfer::TransferProposal,
    ) -> Result<super::whole_body_tasks::Problem,String> {
        let at=adapter.reference_substep(observed.native_substep);
        let row=&self.reference.rows[(at/4).min(self.reference.rows.len() as u64-1) as usize];
        let rotate=|p:[f64;3]|heading*Vector::from_array(p.map(|v|v as f32));
        let velocity=(rotate(row.com_velocity_mps)+(rotate(row.com_position_m)-observed.center_of_mass())*2.).clamp_length_max(0.15);
        let mut posture=nominal.clone();
        for i in 0..25 { for k in 0..3 {
            posture.velocities[i][k]=(nominal.velocities[i][k]+2.*(nominal.positions[i][k]-self.memory.previous[i][k])).clamp(-1.,1.);
        }}
        let mut goals=tasks::TaskGoals {nominal:posture,com_velocity_mps:velocity.to_array().map(f64::from),
            com_weight:1.,point_tasks:vec![],orientation_tasks:vec![],posture_weight:0.25,
            regularization:0.05,retained_velocity_tolerance_mps:0.005};
        for foot in &row.feet { for (suffix,target) in [("Foot",foot.hindfoot_rotation_xyzw),("Forefoot",foot.forefoot_rotation_xyzw)] {
            let segment=model.segments.iter().position(|p|p.id==format!("{}{suffix}",foot.side)).ok_or("Missing reference foot")?;
            let mut delta=heading*Rotation::from_array(target.map(|v|v as f32))*observed.bodies[segment].rotation().inverse();
            if delta.w<0. {delta=-delta;}
            goals.orientation_tasks.push(tasks::OrientationGoal {segment,world_angular_velocity_radps:(delta.to_scaled_axis()*2.).to_array().map(f64::from),weight:1.});
        }}
        proposal.apply_to_goals(&mut goals)?;
        let mut problem=tasks::assemble_observed(model,observed,&adapter.tracker().active,&goals)?;
        for (i,a) in runtime::axes(model)?.iter().enumerate() {
            let old=f64::from(self.memory.previous[a.segment][a.coordinate]);
            let target=f64::from(nominal.positions[a.segment][a.coordinate]);
            let dt=4.*f64::from(lh_contracts::SUBSTEP_S);
            problem.lower[i]=problem.lower[i].max(((target-0.12).max(f64::from(a.minimum))-old)/dt);
            problem.upper[i]=problem.upper[i].min(((target+0.12).min(f64::from(a.maximum))-old)/dt);
        }
        Ok(problem)
    }

    fn configure_transfer(&mut self, sim:&mut Simulation) -> Result<(),String> {
        super::native_plan_preview::require_transfer_native_state(sim)?;
        self.transfer.as_mut().ok_or("Missing transfer adapter")?
            .observe_native_terminal(sim.substeps_completed,sim.counters.recoveries==1)?;
        // Record the current original terminal clock, then capture before the
        // remaining decision changes. Both native branches follow this order.
        let identity=super::native_plan_preview::transfer_identity(sim,self)?;
        let mut adapter=self.transfer.take().ok_or("Missing transfer adapter")?;
        let result=(|| {
            let at=sim.substeps_completed;
            let step=at.checked_sub(self.start).ok_or("Transfer owner clock rewind")?;
            if !self.enabled || self.previous_step.and_then(|p|p.checked_add(1))!=Some(step) || step>=6000
                || sim.recovery.is_none() || sim.recovery.as_ref().is_some_and(|r|r.handed_off()) {
                return Err("Transfer owner discontinuity, deadline, or original observer lost; no restart".into());
            }
            let observation=native_observation::observe(sim,self.start,&[])?;
            let proposal=adapter.observe(&observation)?;
            if at%4==0 {
                let nominal=adapter.nominal(&sim.model,&self.memory,at)?;
                let observed=tasks::ObservedState::from_simulation(sim)?;
                let problem=self.transfer_problem_observed(&sim.model,sim.heading,&observed,&adapter,nominal,&proposal)?;
                adapter.compile_batch(&sim.model,&self.memory,&problem,at)?;
            }
            let command=adapter.proposed_command(&sim.model,&self.memory,at)?;
            adapter.require_native_command(std::ptr::from_ref(sim) as usize,at,identity,&command)?;
            let measured=recovery::support_probe::offline_trajectory::coordinates(sim)?;
            let progress=recovery::support_probe::getup_ab::path_progress(sim,&self.program,&measured);
            if progress>self.best_progress+0.005 { self.best_progress=progress;self.stalled_s=0.; }
            else {self.stalled_s+=sim.profile.dt_s;}
            self.unsupported_s=if recovery::has_persistent_support(sim) {0.} else {self.unsupported_s+sim.profile.dt_s};
            let limits=&crate::recovery_contract::contract().recovery_acceptance;
            if f64::from(self.stalled_s)>=limits.stall_s || f64::from(self.unsupported_s)>=limits.support_loss_s {
                return Err("Original progress/support duration refused transfer".into());
            }
            let anchors=["left","right"].map(|side|recovery::planted_contact(sim,&[
                recovery::index(sim,&format!("{side}Foot")),recovery::index(sim,&format!("{side}Forefoot"))]));
            let controller=sim.recovery.as_mut().ok_or("Original recovery observer vanished")?;
            // Request Stand once; never re-enter or reset settle/handoff counters.
            if adapter.terminal_requested(at) && !controller.standing() {controller.status.phase=recovery::Phase::Stand;}
            controller.status.elapsed_s=step as f32*sim.profile.dt_s;
            controller.status.unsupported_s=self.unsupported_s;
            controller.status.stalled_s=self.stalled_s;
            controller.status.progress=progress;
            controller.foot_anchors=[anchors[0].clone()?,anchors[1].clone()?];
            controller.targets=command.positions.to_vec();controller.velocities=command.velocities.to_vec();
            sim.targets=command.positions.to_vec();
            let (cap,ratio,requests)=recovery::support_probe::offline_trajectory::install_targets(sim,&command.positions,&command.velocities,
                recovery::support_probe::offline_trajectory::Allocation::OwnedRequests)?;
            adapter.commit_command(&mut self.memory,&command,at)?;
            self.maximum_cap=self.maximum_cap.max(cap);self.previous_step=Some(step);
            self.patches=adapter.tracker().clone();
            self.last=json!({"commandSubstep":step,"positions":command.positions,"velocities":command.velocities,
                "capSumNm":cap,"anatomicalCapRatio":ratio,"requests":requests,"transfer":true,"terminalRequested":adapter.terminal_requested(at)});
            Ok(())
        })();
        self.transfer=Some(adapter);
        result
    }

    /// After the original observer grants Handoff, this is a passive admission
    /// monitor only. It does not install targets or modify the recovery state.
    pub(crate) fn observe_transfer_handoff(&mut self,sim:&Simulation)->Result<(),String> {
        super::native_plan_preview::require_transfer_native_state(sim)?;
        let observation=native_observation::observe(sim,self.start,&[])?;
        if observation.segments.iter().any(|s|!matches!(s.role,support_state::SegmentRole::Hindfoot|support_state::SegmentRole::Forefoot)
            &&s.loaded_substeps>=12&&s.persistent_load_n>=3.) {
            return Err("Bilateral continuation gained nonfoot support after handoff".into());
        }
        self.transfer.as_mut().ok_or("Missing passive transfer monitor")?
            .observe_native_terminal(sim.substeps_completed,sim.counters.recoveries==1)?;
        let identity=super::native_plan_preview::transfer_identity(sim,self)?;
        let adapter=self.transfer.as_mut().ok_or("Missing passive transfer monitor")?;
        adapter.observe(&observation)?;
        adapter.require_native_passive(std::ptr::from_ref(sim) as usize,sim.substeps_completed,identity)
    }

    /// Technical verification only: four already reserved native steps, bound
    /// to this instance and immutable candidate. This grants no history lease.
    pub(crate) fn admit_technical_transfer_tick(&mut self,sim:&Simulation,receipt:super::native_plan_preview::NativeTechnicalReceipt)->Result<(),String> {
        super::native_plan_preview::require_transfer_native_state(sim)?;
        let scope=super::native_plan_preview::transfer_scope(sim,self)?;
        let identity=super::native_plan_preview::transfer_identity(sim,self)?;
        self.transfer.as_mut().ok_or("No transfer candidate for technical verification")?
            .admit_technical_tick(receipt,std::ptr::from_ref(sim) as usize,sim.substeps_completed,&identity,&scope)
    }

    pub(crate) fn admit_transfer_window(&mut self,sim:&Simulation,receipt:super::native_plan_preview::NativeWindowReceipt)->Result<(),String> {
        super::native_plan_preview::require_transfer_native_state(sim)?;
        self.transfer.as_ref().ok_or("No transfer candidate to admit")?
            .check_native_terminal(sim.substeps_completed,sim.counters.recoveries==1)?;
        let identity=super::native_plan_preview::transfer_identity(sim,self)?;
        self.transfer.as_mut().ok_or("No transfer candidate to admit")?
            .admit_real_window(receipt,sim.substeps_completed,&identity)
    }

    /// Explicit completed-tick checkpoint for the next 24 already forecast
    /// states, including the tail of the original terminal monitoring second.
    pub(crate) fn continue_transfer_window(&mut self,sim:&Simulation)->Result<(),String> {
        super::native_plan_preview::require_transfer_native_state(sim)?;
        self.transfer.as_ref().ok_or("No transfer candidate to continue")?
            .check_native_terminal(sim.substeps_completed,sim.counters.recoveries==1)?;
        let identity=super::native_plan_preview::transfer_identity(sim,self)?;
        self.transfer.as_mut().unwrap().continue_validated_window(sim.substeps_completed,&identity)
    }

    pub(crate) fn configure(&mut self, sim: &mut Simulation) -> Result<(), String> {
        if self.transfer.is_some() {
            return self.configure_transfer(sim);
        }
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
                let problem = self.problem(sim, step)?;
                let mut trace = Value::Null;
                self.last_attempt = json!({"nativeSubstep":step,"problem":problem_json(&problem),"status":"pending"});
                let compiled = runtime::compile_problem_traced(
                    &sim.model,
                    &self.memory,
                    sim.substeps_completed,
                    &problem,
                    &mut trace,
                );
                self.last_attempt["compilation"] = trace;
                if compiled.is_err() {
                    self.last_attempt["status"] = json!("refused before installation");
                }
                let (next, solution, certificate) = compiled?;
                self.memory = next;
                self.last_attempt["status"] = json!("certified before installation");
                solve_report = json!({"iterations":solution.iterations,"hardError":solution.maximum_constraint_error,"projectedGradientResidual":solution.projected_gradient_residual,
                    "maximumRateRepresentationError":certificate.maximum_observed_rate_error,"rateErrorBounds":certificate.rate_error_bounds.to_vec(),"certifiedCommands":certificate.commands.len()});
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

fn problem_json(problem: &super::whole_body_tasks::Problem) -> Value {
    json!({"lower":problem.lower.to_vec(),"upper":problem.upper.to_vec(),"regularization":problem.regularization,
        "hard":problem.hard.iter().map(|t|json!({"name":t.name,"row":t.row.to_vec(),"lower":t.lower,"upper":t.upper})).collect::<Vec<_>>(),
        "soft":problem.soft.iter().map(|t|json!({"row":t.row.to_vec(),"target":t.target,"weight":t.weight})).collect::<Vec<_>>()})
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
    fn archived_problems() -> (
        lh_model::Model,
        runtime::Memory,
        Vec<(f64, super::super::whole_body_tasks::Problem)>,
        Value,
    ) {
        // Read-only reconstruction: no Simulation or PhysicsWorld is constructed.
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../docs/evidence");
        let fixture: Value = serde_json::from_slice(
            &std::fs::read(
                root.join("coordination-numeric-correction-2026-10-12/archived-step12.json"),
            )
            .unwrap(),
        )
        .unwrap();
        let model: lh_model::Model = serde_json::from_str(lh_model::CANONICAL_JSON).unwrap();
        let poses: Vec<lh_model::Pose> =
            serde_json::from_value(fixture["physical"]["snapshot"]["segments"].clone()).unwrap();
        let bodies = poses
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let properties: MassProperties = serde_json::from_value(
                    fixture["archivedLocalMassProperties"][i]["localMassProperties"].clone(),
                )
                .unwrap();
                let pose = Pose::from_parts(crate::vector(p.position), crate::rotation(p.rotation));
                tasks::ObservedBody {
                    position_m: crate::vector(p.position).to_array(),
                    rotation_xyzw: crate::rotation(p.rotation).to_array(),
                    linear_velocity_mps: crate::vector(p.linear_velocity).to_array(),
                    angular_velocity_radps: crate::vector(p.angular_velocity).to_array(),
                    world_center_of_mass_m: properties.world_com(&pose).to_array(),
                    mass_kg: properties.mass(),
                }
            })
            .collect();
        let observed = tasks::ObservedState {
            native_substep: 12,
            bodies,
        };
        let archived_com: [f64; 3] =
            serde_json::from_value(fixture["physical"]["support"]["center_of_mass_m"].clone())
                .unwrap();
        assert_eq!(
            observed.center_of_mass().to_array().map(f32::to_bits),
            archived_com.map(|v| (v as f32).to_bits()),
            "Native f32 CoM reconstruction must be bit exact"
        );
        let previous =
            serde_json::from_value(fixture["ownerBeforeConfigure"]["q"].clone()).unwrap();
        let memory = runtime::initialize(&model, previous, 12, 0).unwrap();
        let program: recovery::trajectory_program::Program =
            serde_json::from_value(fixture["commandProgram"].clone()).unwrap();
        let reference: ReferenceTasks = serde_json::from_slice(
            &std::fs::read(root.join("coordination-prototype-2026-10-12/continuation-tasks.json"))
                .unwrap(),
        )
        .unwrap();
        let mut problems = Vec::new();
        for case in fixture["cases"].as_array().unwrap() {
            let owner = Controller {
                program: program.clone().compile(&model).unwrap(),
                reference: reference.clone(),
                enabled: true,
                feedback_scale: case["feedbackScale"].as_f64().unwrap(),
                memory: memory.clone(),
                patches: serde_json::from_value(case["terminalOwner"]["patches"].clone()).unwrap(),
                start: 0,
                previous_step: Some(11),
                best_progress: 0.,
                stalled_s: 0.,
                unsupported_s: 0.,
                maximum_cap: 0.,
                last: Value::Null,
                last_attempt: Value::Null,
                transfer: None,
            };
            problems.push((
                owner.feedback_scale,
                owner
                    .problem_observed(&model, Rotation::IDENTITY, &observed, 12)
                    .unwrap(),
            ));
        }
        (model, memory, problems, fixture)
    }
    #[test]
    fn archived_step12_problem_reconstruction_without_integration() {
        let (_, memory, problems, fixture) = archived_problems();
        assert_eq!(problems.len(), 4);
        let hard = problem_json(&problems[0].1)["hard"].clone();
        for (_, problem) in &problems {
            assert_eq!(problem_json(problem)["hard"], hard);
        }
        let output = json!({"scope":"pure archived-state numeric reconstruction; no Simulation, world, FK or native integration",
            "sourceArchivedFixtureSha256":"79c0feb2d4bf3ea4b336b8d23782bf897cecadda826d081a83d2d880209d873d",
            "nativeSubstep":12,"previousQ":memory.previous,"nativeF32ComMatchesArchiveBitwise":true,
            "cases":problems.iter().enumerate().map(|(i,(scale,p))|json!({"feedbackScale":scale,"problem":problem_json(p),"frozenRejectedF64Rates":fixture["cases"][i]["rejectedF64Rates"]})).collect::<Vec<_>>()});
        if let Ok(path) = std::env::var("LH_NUMERIC_REPLAY_EXPORT") {
            std::fs::write(path, serde_json::to_vec_pretty(&output).unwrap()).unwrap();
        }
    }
    mod frozen_solver {
        include!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-numeric-correction-2026-10-12/frozen-solver-replay.rs"
        ));
    }
    #[test]
    fn archived_step12_reproduces_all_four_frozen_refusals_without_integration() {
        let (model, memory, problems, fixture) = archived_problems();
        let mut cases = Vec::new();
        let layout = runtime::axes(&model).unwrap();
        for (case_index, (scale, p)) in problems.iter().enumerate() {
            let old = frozen_solver::Problem {
                lower: p.lower,
                upper: p.upper,
                regularization: p.regularization,
                hard: p
                    .hard
                    .iter()
                    .map(|t| frozen_solver::HardTask {
                        name: t.name.clone(),
                        row: t.row,
                        lower: t.lower,
                        upper: t.upper,
                    })
                    .collect(),
                soft: p
                    .soft
                    .iter()
                    .map(|t| frozen_solver::Task {
                        row: t.row,
                        target: t.target,
                        weight: t.weight,
                    })
                    .collect(),
            };
            match old.solve() {
                Ok(solution) => {
                    assert_eq!(case_index, 0);
                    let archived: Vec<f64> =
                        serde_json::from_value(fixture["cases"][0]["rejectedF64Rates"].clone())
                            .unwrap();
                    assert_eq!(
                        solution.rates.map(f64::to_bits).to_vec(),
                        archived.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                        "Frozen f64 solution must be reproduced bitwise"
                    );
                    let mut trial = memory.clone();
                    trial.rates = solution.rates;
                    let mut emitted = Vec::new();
                    for step in 12..16 {
                        let (command, next) = runtime::command(&model, &trial, step).unwrap();
                        let rates: [f64; 46] = std::array::from_fn(|i| {
                            f64::from(command.velocities[layout[i].segment][layout[i].coordinate])
                        });
                        let mut violations = Vec::new();
                        for (i, rate) in rates.iter().enumerate() {
                            if *rate < p.lower[i] - 1e-8 || *rate > p.upper[i] + 1e-8 {
                                violations.push(json!({"kind":"bound","axis":i,"value":rate,"lower":p.lower[i],"upper":p.upper[i]}));
                            }
                        }
                        for task in &p.hard {
                            let value = task.row.iter().zip(rates).map(|(a, x)| a * x).sum::<f64>();
                            let error = (task.lower - value).max(value - task.upper).max(0.);
                            if error > 1e-8 {
                                violations.push(json!({"kind":"hard","name":task.name,"value":value,"lower":task.lower,"upper":task.upper,"excess":error,"tolerance":1e-8}));
                            }
                        }
                        emitted.push(json!({"nativeSubstep":step,"positions":command.positions,"velocities":command.velocities,"violations":violations}));
                        trial = next;
                    }
                    assert!(
                        !emitted[0]["violations"].as_array().unwrap().is_empty(),
                        "The first rejected compiled command must be identified"
                    );
                    cases.push(json!({"feedbackScale":scale,"frozenSolutionMatchesArchiveBitwise":true,"continuousRates":solution.rates.to_vec(),"continuousIterations":solution.iterations,"continuousProjectedGradientResidual":solution.projected_gradient_residual,"commands":emitted}));
                }
                Err(error) => {
                    assert!(case_index > 0);
                    assert!(error.contains("did not converge"));
                    cases.push(json!({"feedbackScale":scale,"refusal":error}));
                }
            }
        }
        if let Ok(path) = std::env::var("LH_NUMERIC_REFUSAL_EXPORT") {
            std::fs::write(
                path,
                serde_json::to_vec_pretty(&json!({"nativeIntegrations":0,"cases":cases})).unwrap(),
            )
            .unwrap();
        }
    }
    #[test]
    fn archived_step12_all_scales_emit_certified_predictable_f32_commands_without_integration() {
        let (model, memory, problems, _) = archived_problems();
        let mut cases = Vec::new();
        let layout = runtime::axes(&model).unwrap();
        for (scale, problem) in problems {
            let mut audit = Value::Null;
            let (decision, solution, certificate) =
                runtime::compile_problem_traced(&model, &memory, 12, &problem, &mut audit)
                    .unwrap_or_else(|e| panic!("scale {scale}: {e}; audit {audit}"));
            assert_eq!(certificate.commands.len(), 4);
            assert_eq!(memory.next_substep, 12);
            let mut real = decision.clone();
            let mut preview = decision.clone();
            for (index, expected) in certificate.commands.iter().enumerate() {
                let step = 12 + index as u64;
                let (a, anext) = runtime::command(&model, &real, step).unwrap();
                let (b, bnext) = runtime::command(&model, &preview, step).unwrap();
                let bits = |q: [[f32; 3]; 25]| q.map(|row| row.map(f32::to_bits));
                assert_eq!(bits(a.positions), bits(b.positions));
                assert_eq!(bits(a.velocities), bits(b.velocities));
                assert_eq!(bits(a.positions), bits(expected.positions));
                assert_eq!(bits(a.velocities), bits(expected.velocities));
                let rates: [f64; 46] = std::array::from_fn(|i| {
                    f64::from(a.velocities[layout[i].segment][layout[i].coordinate])
                });
                problem.verify_rates(&rates).unwrap();
                for axis in &layout {
                    assert_eq!(
                        a.velocities[axis.segment][axis.coordinate],
                        (a.positions[axis.segment][axis.coordinate]
                            - real.previous[axis.segment][axis.coordinate])
                            / lh_contracts::SUBSTEP_S
                    );
                }
                real = anext;
                preview = bnext;
            }
            assert_eq!(real.next_substep, 16);
            cases.push(json!({"feedbackScale":scale,"iterations":solution.iterations,"hardError":solution.maximum_constraint_error,"projectedGradientResidual":solution.projected_gradient_residual,"maximumObservedRateRepresentationError":certificate.maximum_observed_rate_error,"actualF32CommandsCertified":4,"replayedCommandBitsEqual":true,"audit":audit}));
        }
        if let Ok(path) = std::env::var("LH_NUMERIC_CORRECTION_EXPORT") {
            std::fs::write(path,serde_json::to_vec_pretty(&json!({"scope":"pure command replay, no native dynamics qualification","nativeIntegrations":0,"cases":cases})).unwrap()).unwrap();
        }
    }
}


#[cfg(test)]
mod transfer_archive_tests {
    use super::*;
    use std::{fs::{self,OpenOptions},io::Write,time::{SystemTime,UNIX_EPOCH}};

    /// Ignored by ordinary test runs: each invocation needs its own durable,
    /// single-use inspection ticket and must finish within the frozen window.
    #[test]
    #[ignore = "requires an unexpired separately charged geometry ticket"]
    fn archived_a36_common_command_proposal() {
        let root=std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
        let dir=root.join("docs/evidence/coordination-transfer-candidate-2026-10-13");
        let ticket=std::env::var("LH_TRANSFER_GEOMETRY_TICKET").expect("No geometry authority");
        let ticket_path=std::path::Path::new(&ticket);
        let authorization:Value=serde_json::from_slice(&fs::read(ticket_path).unwrap()).unwrap();
        let deadline=authorization["deadlineUnixMs"].as_u64().unwrap();
        let now=||SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64;
        assert!(now()<deadline,"Inspection deadline expired");
        assert_eq!(authorization["fixtureNativeSubstep"],36);
        let model:lh_model::Model=serde_json::from_slice(&fs::read(root.join("rust/crates/model/data/canonical-v1.json")).unwrap()).unwrap();
        let input:Value=serde_json::from_slice(&fs::read(dir.join("observed-entry-step36.json")).unwrap()).unwrap();
        let observed:Value=serde_json::from_slice(&fs::read(dir.join("observed-entry-task-state.json")).unwrap()).unwrap();
        let observed:tasks::ObservedState=serde_json::from_value(observed["observed_task_state"].clone()).unwrap();
        let observation:support_state::SupportObservation=serde_json::from_value(input["archivedNativeRow"]["support"].clone()).unwrap();
        let entry=serde_json::from_value(input["commandEntry"].clone()).unwrap();
        let source:recovery::trajectory_program::Program=serde_json::from_slice(&fs::read(root.join("docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json")).unwrap()).unwrap();
        let reference=serde_json::from_slice(&fs::read(root.join("docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json")).unwrap()).unwrap();
        let o=&input["ownerBeforeConfigure36"];
        let rates:Vec<f64>=serde_json::from_value(o["rates"].clone()).unwrap();
        let mut owner=Controller { program:source.compile(&model).unwrap(),reference,
            enabled:o["enabled"].as_bool().unwrap(),feedback_scale:o["feedbackScale"].as_f64().unwrap(),
            memory:runtime::Memory {previous:serde_json::from_value(o["q"].clone()).unwrap(),next_substep:36,rates:rates.try_into().unwrap(),mode_signature:0},
            patches:serde_json::from_value(o["patches"].clone()).unwrap(),start:o["start"].as_u64().unwrap(),
            previous_step:Some(o["previousStep"].as_u64().unwrap()),best_progress:o["bestProgress"].as_f64().unwrap() as f32,
            stalled_s:o["stalledS"].as_f64().unwrap() as f32,unsupported_s:o["unsupportedS"].as_f64().unwrap() as f32,
            maximum_cap:o["maximumCapNm"].as_f64().unwrap(),last:o["last"].clone(),last_attempt:o["numericAttempt"].clone(),transfer:None };
        owner.enable_observed_transfer(&model,&observation,entry).unwrap();
        let mut adapter=owner.transfer.take().unwrap();
        let proposal=adapter.observe(&observation).unwrap();
        let nominal=adapter.nominal(&model,&owner.memory,36).unwrap();
        // Consume durably immediately BEFORE the sole geometry assembly call.
        let mut used=OpenOptions::new().create_new(true).write(true).open(ticket_path.with_extension("used.json")).unwrap();
        let receipt=json!({"nativeSubsteps":0,"fkEvaluations":0,"assembleObservedCallsCharged":1,"startedUnixMs":now(),"deadlineUnixMs":deadline});
        used.write_all(serde_json::to_string_pretty(&receipt).unwrap().as_bytes()).unwrap();used.sync_all().unwrap();
        assert!(now()<deadline);
        let problem=owner.transfer_problem_observed(&model,Rotation::IDENTITY,&observed,&adapter,nominal.clone(),&proposal).unwrap();
        let compilation=adapter.compile_batch(&model,&owner.memory,&problem,36);
        let mut commands=vec![];
        if compilation.is_ok() { for at in 36..40 {
            let command=adapter.proposed_command(&model,&owner.memory,at).unwrap();
            commands.push(json!({"at":at,"positions":command.positions,"velocities":command.velocities}));
            adapter.commit_command(&mut owner.memory,&command,at).unwrap();
        }}
        let evidence=json!({"status":if compilation.is_ok(){"pure-command-pass-native-admission-required"}else{"pure-command-rejection"},
            "nativeSubsteps":0,"newWorlds":0,"fkEvaluations":0,"assembleObservedCalls":1,
            "completedUnixMs":now(),"proposal":proposal,"nominal":{"positions":nominal.positions,"velocities":nominal.velocities},
            "problem":{"lower":problem.lower.to_vec(),"upper":problem.upper.to_vec(),"regularization":problem.regularization,
                "hard":problem.hard.iter().map(|h|json!({"name":h.name,"row":h.row.to_vec(),"lower":h.lower,"upper":h.upper})).collect::<Vec<_>>(),
                "soft":problem.soft.iter().map(|t|json!({"row":t.row.to_vec(),"target":t.target,"weight":t.weight})).collect::<Vec<_>>()},"compilationError":compilation.as_ref().err(),"commands":commands,"adapter":adapter.evidence()});
        fs::write(dir.join("archived-a36-command-result.json"),serde_json::to_vec_pretty(&evidence).unwrap()).unwrap();
        assert!(now()<deadline,"Inspection completed outside window");
        assert!(compilation.is_ok(),"Frozen candidate rejected: {:?}",compilation.err());
        assert_eq!(commands.len(),4);
        assert_eq!(adapter.tracker().active.iter().map(|p|p.segment).collect::<Vec<_>>(),[19,24]);
    }
}
