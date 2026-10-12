//! Explicit NDJSON worker. No test or production entrypoint invokes it.
//! Every integration requires a parent reservation and a local four-step debit.
use super::native_plan_preview::{
    NativeKernelAdmission, NativeParityTrace, PreviewBudget, PreviewPurpose, StaticPreviewBranch,
    StepReservation, native_kernel_admission,
};
use super::whole_body_controller::{self, ReferenceTasks};
use super::{contact_observation as journal, native_observation, support_state};
use crate::{Simulation, measurement, recovery::trajectory_program::Program};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeSet,
    fs::{File, OpenOptions},
    io::{BufRead, BufWriter, Write},
    path::Path,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Permit {
    reservation_id: String,
    reserved_steps: u64,
    deadline_unix_s: f64,
    remaining_wall_seconds: f64,
    worker_pid: u32,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Inputs {
    program_path: String,
    tasks_path: String,
}
#[derive(Clone)]
struct FrozenInputs {
    program: Program,
    tasks: ReferenceTasks,
    program_bytes: Vec<u8>,
    tasks_bytes: Vec<u8>,
}

struct LocalBudget {
    permit: Permit,
    consumed: u64,
    deadline: Instant,
    purpose: PreviewPurpose,
}
impl PreviewBudget for LocalBudget {
    fn admit_preview(&mut self, purpose: PreviewPurpose, steps: u32) -> Result<(), String> {
        self.check_wall_deadline()?;
        if purpose != self.purpose
            || (purpose == PreviewPurpose::ParityVerification && steps != 4)
            || (purpose == PreviewPurpose::Candidate && steps != 480)
        {
            return Err("Preview differs from the frozen reservation purpose/horizon".into());
        }
        Ok(())
    }
    fn reserve_one_before_step(&mut self, request: StepReservation) -> Result<(), String> {
        self.check_wall_deadline()?;
        if request.purpose != self.purpose || self.consumed >= self.permit.reserved_steps {
            return Err("Local parent reservation exhausted or wrong purpose".into());
        }
        self.consumed += 1;
        Ok(())
    }
    fn reserve_four_before_tick(&mut self, request: StepReservation) -> Result<(), String> {
        self.check_wall_deadline()?;
        if request.purpose != self.purpose
            || self
                .consumed
                .checked_add(4)
                .is_none_or(|end| end > self.permit.reserved_steps)
        {
            return Err("Cannot reserve the complete next native tick".into());
        }
        self.consumed += 4;
        Ok(())
    }
    fn check_wall_deadline(&mut self) -> Result<(), String> {
        if Instant::now() >= self.deadline || unix_now()? >= self.permit.deadline_unix_s {
            Err("Original batch wall deadline exhausted".into())
        } else {
            Ok(())
        }
    }
}

struct Worker {
    inputs: Option<FrozenInputs>,
    a_receipt: Option<NativeKernelAdmission>,
    b_receipt: Option<NativeKernelAdmission>,
    reservation_ids: BTreeSet<String>,
    reserved_total: u64,
    actual_total: u64,
    deadline: Option<Instant>,
    histories: usize,
    prior_b_success: bool,
}
impl Worker {
    fn new() -> Self {
        Self {
            inputs: None,
            a_receipt: None,
            b_receipt: None,
            reservation_ids: BTreeSet::new(),
            reserved_total: 0,
            actual_total: 0,
            deadline: None,
            histories: 0,
            prior_b_success: false,
        }
    }

    fn accept_permit(
        &mut self,
        raw: &Value,
        expected: u64,
        purpose: PreviewPurpose,
    ) -> Result<LocalBudget, String> {
        let permit: Permit = serde_json::from_value(raw.clone()).map_err(|e| e.to_string())?;
        if permit.worker_pid != std::process::id()
            || permit.reserved_steps != expected
            || permit.reservation_id.is_empty()
            || !permit.deadline_unix_s.is_finite()
            || !permit.remaining_wall_seconds.is_finite()
            || permit.remaining_wall_seconds <= 0.
            || permit.remaining_wall_seconds > 7200.
        {
            return Err("Invalid native permit, worker identity, or wall limit".into());
        }
        let remaining = permit
            .remaining_wall_seconds
            .min(permit.deadline_unix_s - unix_now()?);
        if remaining <= 0. {
            return Err("Expired parent permit".into());
        }
        let deadline = Instant::now()
            .checked_add(Duration::from_secs_f64(remaining))
            .ok_or("Invalid deadline")?;
        let deadline = self.deadline.map_or(deadline, |prior| prior.min(deadline));
        self.deadline = Some(deadline);
        if !self.reservation_ids.insert(permit.reservation_id.clone()) {
            return Err("Reservation replay refused".into());
        }
        self.reserved_total = self
            .reserved_total
            .checked_add(expected)
            .ok_or("Reservation overflow")?;
        if self.reserved_total > 300_000 {
            return Err("Global native reservation ceiling exceeded".into());
        }
        Ok(LocalBudget {
            permit,
            consumed: 0,
            deadline,
            purpose,
        })
    }
    fn count(&mut self, steps: u64) -> Result<(), String> {
        self.actual_total = self
            .actual_total
            .checked_add(steps)
            .ok_or("Native count overflow")?;
        if self.actual_total > self.reserved_total || self.actual_total > 300_000 {
            return Err("Native accounting exceeded reservations".into());
        }
        Ok(())
    }
    fn inputs(&mut self, raw: &Value) -> Result<FrozenInputs, String> {
        let input: Inputs = serde_json::from_value(raw.clone()).map_err(|e| e.to_string())?;
        let program_bytes = std::fs::read(input.program_path).map_err(|e| e.to_string())?;
        let tasks_bytes = std::fs::read(input.tasks_path).map_err(|e| e.to_string())?;
        if let Some(prior) = &self.inputs {
            if prior.program_bytes != program_bytes || prior.tasks_bytes != tasks_bytes {
                return Err("Frozen reference changed between operations".into());
            }
            return Ok(prior.clone());
        }
        let value = FrozenInputs {
            program: serde_json::from_slice(&program_bytes).map_err(|e| e.to_string())?,
            tasks: serde_json::from_slice(&tasks_bytes).map_err(|e| e.to_string())?,
            program_bytes,
            tasks_bytes,
        };
        self.inputs = Some(value.clone());
        Ok(value)
    }
}

fn unix_now() -> Result<f64, String> {
    Ok(SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_secs_f64())
}
fn required_string<'a>(v: &'a Value, key: &str) -> Result<&'a str, String> {
    v[key].as_str().ok_or_else(|| format!("Missing {key}"))
}
fn new_file(path: &str) -> Result<BufWriter<File>, String> {
    let p = Path::new(path);
    if let Some(parent) = p.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(p)
        .map(BufWriter::new)
        .map_err(|e| e.to_string())
}
fn line(writer: &mut impl Write, value: &Value) -> Result<(), String> {
    serde_json::to_writer(&mut *writer, value).map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())
}
fn send(writer: &mut impl Write, value: &Value) -> Result<(), String> {
    line(writer, value)?;
    writer.flush().map_err(|e| e.to_string())
}
fn read(reader: &mut impl BufRead) -> Result<Option<Value>, String> {
    let mut text = String::new();
    let bytes = reader.read_line(&mut text).map_err(|e| e.to_string())?;
    if bytes == 0 {
        return Ok(None);
    }
    if bytes > 1_048_576 {
        return Err("Parent message exceeds protocol cap".into());
    }
    let value: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    if value["op"] == "abort" {
        return Err("Parent aborted the frozen batch".into());
    }
    Ok(Some(value))
}
fn failure(s: &Simulation, substep: u32, reason: String) -> measurement::Failure {
    measurement::Failure::new(s.tick, substep, "coordination-worker", reason, 1., 0.)
}
fn native_frame(
    s: &Simulation,
    observation: &support_state::SupportObservation,
    diagnostic: &support_state::LocalSupportSample,
    ccd: &[journal::SolveRecord],
    kind: &str,
) -> Value {
    let motors: Vec<_> = s
        .joints
        .iter()
        .enumerate()
        .filter_map(|(segment, handle)| {
            let joint = s.world.impulse_joints.get((*handle)?)?;
            Some(json!({"segment":segment,"motors":joint.data.motors}))
        })
        .collect();
    json!({"kind":kind,"nativeSubstep":s.substeps_completed,"snapshot":s.snapshot(),
        "owner":s.coordination.as_ref().map(|owner|owner.evidence()),"support":observation,
        "targets":s.targets,"restTargets":s.rest_targets,"motors":motors,
        "persistence":s.persistence,"integratedSupportSubsteps":s.integrated_support_substeps,
        "integratedSupportLoadsN":s.integrated_support_loads_n,
        "localDiagnostic":diagnostic,"ccd":ccd,
        "ccdJournalCollected":!kind.starts_with("preview-") && !kind.ends_with("-copy"),
        "angularBalanceCertified":false})
}
fn report_result(
    stdout: &mut impl Write,
    op: &str,
    actual: u64,
    outcome: Value,
    path: &str,
) -> Result<(), String> {
    send(
        stdout,
        &json!({"event":"result","op":op,"actualNativeSteps":actual,"outcome":outcome,"evidencePath":path}),
    )
}
fn outcome(ok: bool, success: bool) -> Value {
    json!({"integrity_ok":ok,"support_feasible":ok,"predictions_usable":ok,"diagnostic_present":ok,"local_success":success})
}

fn local_deadline_irreversible(
    diagnostic: &support_state::LocalSupportDiagnostic,
    time_s: f64,
) -> bool {
    time_s >= 2.
        && !diagnostic.passed
        && !diagnostic.stable_started_s.is_some_and(|start| start < 2.)
}

fn actual_tick(
    sim: &mut Simulation,
    budget: &mut LocalBudget,
    diagnostic: &mut support_state::LocalSupportDiagnostic,
    trace: &mut impl Write,
    kind: &str,
    mut parity: Option<&mut NativeParityTrace>,
    enforce_local_deadline: bool,
) -> Result<(), String> {
    journal::require_inactive()?;
    budget.reserve_four_before_tick(StepReservation {
        branch_id: 0,
        before_substep: sim.substeps_completed,
        preview_index: 0,
        purpose: budget.purpose,
    })?;
    journal::activate(sim);
    let before = sim.substeps_completed;
    let mut pending_ccd = Vec::new();
    let result = sim.advance_tick_observed(|s, substep| {
        pending_ccd = journal::take_records();
        if s.substeps_completed != before + u64::from(substep) + 1 {
            return Err(failure(
                s,
                substep,
                "Actual native step clock mismatch".into(),
            ));
        }
        let observed =
            native_observation::observe(s, 0, &pending_ccd).map_err(|e| failure(s, substep, e))?;
        let sample = diagnostic
            .observe(&observed)
            .map_err(|e| failure(s, substep, e))?;
        if let Some(p) = parity.as_deref_mut() {
            p.record(s, &())
                .map_err(|e| failure(s, substep, format!("{e:?}")))?;
        }
        line(
            trace,
            &native_frame(s, &observed, &sample, &pending_ccd, kind),
        )
        .map_err(|e| failure(s, substep, e))?;
        pending_ccd.clear();
        if enforce_local_deadline && local_deadline_irreversible(diagnostic, observed.time_s) {
            return Err(failure(
                s,
                substep,
                "Local diagnostic can no longer begin its stable interval before 2 s".into(),
            ));
        }
        budget
            .check_wall_deadline()
            .map_err(|e| failure(s, substep, e))?;
        Ok(())
    });
    pending_ccd.extend(journal::take_records());
    journal::finish();
    if let Err(error) = result {
        line(
            trace,
            &json!({"kind":"last-failed-state","source":kind,"nativeSubstep":sim.substeps_completed,
            "snapshot":sim.snapshot(),"owner":sim.coordination.as_ref().map(|c|c.evidence()),
            "firstFailure":error,"unobservedCcd":pending_ccd}),
        )?;
        return Err(format!("{error:?}"));
    }
    Ok(())
}

fn verify(worker: &mut Worker, request: &Value, stdout: &mut impl Write) -> Result<(), String> {
    if worker.a_receipt.is_some() || worker.b_receipt.is_some() {
        return Err("Native verification may not be renewed".into());
    }
    let inputs = worker.inputs(&request["inputs"])?;
    let output = required_string(request, "outputPath")?;
    let mut report = new_file(output)?;
    let trace_path = required_string(request, "tracePath")?;
    let mut trace = new_file(trace_path)?;
    let mut budget =
        worker.accept_permit(&request["permit"], 16, PreviewPurpose::ParityVerification)?;
    let mut actual = 0;
    let mut cases = Vec::new();
    let mut refusal = None;
    for enabled in [false, true] {
        let name = if enabled { "B" } else { "A" };
        let mut real = whole_body_controller::scene(
            inputs.program.clone(),
            inputs.tasks.clone(),
            enabled,
            0.,
        )?;
        let mut copy = StaticPreviewBranch::capture(&real, &()).map_err(|e| format!("{e:?}"))?;
        let mut predicted =
            NativeParityTrace::begin(copy.simulation(), &()).map_err(|e| format!("{e:?}"))?;
        let mut actual_trace =
            NativeParityTrace::begin(&real, &()).map_err(|e| format!("{e:?}"))?;
        let mut copy_diagnostic = support_state::LocalSupportDiagnostic::default();
        let copied = copy.verify_tick(u64::from(enabled), &mut budget, |s, _, _| {
            predicted.record(s, &()).map_err(|e| format!("{e:?}"))?;
            let observed = native_observation::observe(s, 0, &[])?;
            let sample = copy_diagnostic.observe(&observed)?;
            line(
                &mut trace,
                &native_frame(s, &observed, &sample, &[], &format!("verify-{name}-copy")),
            )?;
            Ok(())
        });
        let copied_steps = copy.simulation().substeps_completed;
        actual += copied_steps;
        worker.count(copied_steps)?;
        if let Some(error) = copied.refusal {
            cases.push(json!({"name":name,"predicted":predicted,
                "copyLastState":copy.simulation().snapshot(),
                "copyLastOwner":copy.simulation().coordination.as_ref().map(|o|o.evidence()),
                "copyFirstFailure":copy.simulation().first_failure,"copyDiagnostic":copy_diagnostic}));
            refusal = Some(format!("{name} copy: {error:?}"));
            break;
        }
        let mut real_diagnostic = support_state::LocalSupportDiagnostic::default();
        let result = actual_tick(
            &mut real,
            &mut budget,
            &mut real_diagnostic,
            &mut trace,
            &format!("verify-{name}-actual"),
            Some(&mut actual_trace),
            false,
        );
        actual += real.substeps_completed;
        worker.count(real.substeps_completed)?;
        if let Err(error) = result {
            cases.push(
                json!({"name":name,"predicted":predicted,"actual":actual_trace,
                "actualLastState":real.snapshot(),"actualFirstFailure":real.first_failure}),
            );
            refusal = Some(error);
            break;
        }
        match native_kernel_admission(&predicted, &actual_trace) {
            Ok(receipt) => {
                cases.push(json!({"name":name,"receipt":receipt,"predicted":predicted,"actual":actual_trace}));
                if enabled {
                    worker.b_receipt = Some(receipt);
                } else {
                    worker.a_receipt = Some(receipt);
                }
            }
            Err(error) => {
                refusal = Some(format!("{error:?}"));
                break;
            }
        }
    }
    let ok = refusal.is_none()
        && actual == 16
        && budget.consumed == 16
        && worker.a_receipt.is_some()
        && worker.b_receipt.is_some();
    let mut result = outcome(ok, false);
    result["native_accounting_verified"] = json!(ok);
    result["prediction_state_and_motor_semantics_verified"] = json!(ok);
    line(
        &mut report,
        &json!({"scope":"native-parity-verification","actualNativeSteps":actual,
        "reservedNativeSteps":16,"consumedLocalReservations":budget.consumed,"refusal":refusal,
        "cases":cases,"tracePath":trace_path,"outcome":result,"recoveryQualified":false}),
    )?;
    report.flush().map_err(|e| e.to_string())?;
    trace.flush().map_err(|e| e.to_string())?;
    report_result(stdout, "verify", actual, result, output)
}

fn planning(
    worker: &mut Worker,
    sim: &Simulation,
    diagnostic: &support_state::LocalSupportDiagnostic,
    receipt: &NativeKernelAdmission,
    reader: &mut impl BufRead,
    stdout: &mut impl Write,
    trace: &mut impl Write,
) -> Result<(f64, NativeParityTrace, u64), String> {
    send(
        stdout,
        &json!({"event":"planning-request","historyStep":sim.substeps_completed,
        "longSteps":[480,480,480,480],"shortProbes":0,"requiredReservedSteps":1920}),
    )?;
    let request = read(reader)?.ok_or("Parent disconnected before a planning permit")?;
    if request["op"] != "planning-permit" {
        return Err("Expected planning permit".into());
    }
    let output = required_string(&request, "outputPath")?;
    let mut report = new_file(output)?;
    let mut budget = worker.accept_permit(&request["permit"], 1920, PreviewPurpose::Candidate)?;
    let mut cases = Vec::new();
    let mut best: Option<([f64; 5], f64, NativeParityTrace)> = None;
    let mut actual = 0;
    for (index, scale) in [0., 0.5, 1., 1.5].into_iter().enumerate() {
        let mut branch = StaticPreviewBranch::capture(sim, &()).map_err(|e| format!("{e:?}"))?;
        branch
            .set_feedback_scale(scale)
            .map_err(|e| format!("{e:?}"))?;
        let mut first_four =
            NativeParityTrace::begin(branch.simulation(), &()).map_err(|e| format!("{e:?}"))?;
        let mut state = diagnostic.clone();
        let mut last = None;
        let start = sim.substeps_completed;
        let predicted = branch.preview(receipt, index as u64, 480, &mut budget, |s, _, _| {
            if s.substeps_completed <= start + 4 {
                first_four.record(s, &()).map_err(|e| format!("{e:?}"))?;
            }
            let observation = native_observation::observe(s, 0, &[])?;
            let sample = state.observe(&observation)?;
            line(
                trace,
                &native_frame(
                    s,
                    &observation,
                    &sample,
                    &[],
                    &format!("preview-{start}-{index}"),
                ),
            )?;
            let velocity = observation
                .center_of_mass_velocity_mps
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            last = Some((sample, velocity));
            if local_deadline_irreversible(&state, observation.time_s) {
                return Err(
                    "Local diagnostic can no longer begin its stable interval before 2 s".into(),
                );
            }
            Ok(())
        });
        let integrated = branch.simulation().substeps_completed - start;
        actual += integrated;
        worker.count(integrated)?;
        let usable = predicted.refusal.is_none() && integrated == 480 && last.is_some();
        let score = last.as_ref().map(|(s, speed)| {
            [
                f64::from(u8::from(s.passed)),
                f64::from(u8::from(s.com_inside && s.capture_inside)),
                s.heel_loads_n[0].min(s.heel_loads_n[1]),
                s.support_fraction,
                -speed,
            ]
        });
        cases.push(json!({"feedbackScale":scale,"usable":usable,"actualNativeSteps":integrated,
            "refusal":predicted.refusal.as_ref().map(|e|format!("{e:?}")),"score":score,
            "terminalDiagnostic":last.as_ref().map(|v|&v.0),"terminalState":branch.simulation().snapshot(),
            "terminalOwner":branch.simulation().coordination.as_ref().map(|o|o.evidence()),
            "firstFailure":branch.simulation().first_failure,
            "firstPredictedSteps":first_four}));
        if usable
            && score.is_some_and(|score| best.as_ref().is_none_or(|(prior, _, _)| score > *prior))
        {
            best = Some((score.unwrap(), scale, first_four));
        }
    }
    let ok = best.is_some();
    let result = outcome(ok, false);
    line(
        &mut report,
        &json!({"scope":"four-frozen-native-previews","historyStep":sim.substeps_completed,
        "actualNativeSteps":actual,"reservedNativeSteps":1920,"consumedLocalReservations":budget.consumed,
        "selectedScale":best.as_ref().map(|(_,s,_)|s),"cases":cases,"outcome":result}),
    )?;
    report.flush().map_err(|e| e.to_string())?;
    trace.flush().map_err(|e| e.to_string())?;
    report_result(stdout, "planning", actual, result, output)?;
    let (_, scale, predicted) =
        best.ok_or("All four native predictions refused; no substitute proposal")?;
    Ok((scale, predicted, actual))
}

fn history(
    worker: &mut Worker,
    request: &Value,
    reader: &mut impl BufRead,
    stdout: &mut impl Write,
) -> Result<(), String> {
    if worker.a_receipt.is_none() || worker.b_receipt.is_none() {
        return Err("Histories require successful native verification".into());
    }
    let id = required_string(request, "id")?;
    let expected = ["A", "B", "B-repeat", "B60"]
        .get(worker.histories)
        .ok_or("Four-history ceiling")?;
    let enabled = request["enabled"]
        .as_bool()
        .ok_or("Missing history owner mode")?;
    let heading = request["headingDegrees"]
        .as_f64()
        .ok_or("Missing history heading")?;
    if id != *expected
        || enabled != (id != "A")
        || heading != (if id == "B60" { 60. } else { 0. })
        || worker.histories >= 2 && !worker.prior_b_success
    {
        return Err("History differs from frozen order/conditional followups".into());
    }
    let inputs = worker.inputs(&request["inputs"])?;
    let output = required_string(request, "outputPath")?;
    let mut report = new_file(output)?;
    let trace_path = required_string(request, "tracePath")?;
    let mut trace = new_file(trace_path)?;
    let mut budget = worker.accept_permit(&request["permit"], 960, PreviewPurpose::Candidate)?;
    let mut sim = whole_body_controller::scene(
        inputs.program,
        inputs.tasks,
        enabled,
        (heading as f32).to_radians(),
    )?;
    let initial = sim.snapshot();
    let mut diagnostic = support_state::LocalSupportDiagnostic::default();
    let mut actual = 0;
    let mut copies = 0;
    let mut planning_calls = 0;
    let mut refusal = None;
    let mut divergence = None;
    let mut expected_first: Option<NativeParityTrace> = None;
    let mut actual_first: Option<NativeParityTrace> = None;
    let receipt = worker.b_receipt.take().ok_or("Missing B parity receipt")?;
    while sim.substeps_completed < 960 {
        if enabled && sim.substeps_completed % 24 == 0 {
            if planning_calls >= 40 {
                refusal = Some("Planning call ceiling".into());
                break;
            }
            planning_calls += 1;
            let prior_count = worker.actual_total;
            let planned = planning(
                worker,
                &sim,
                &diagnostic,
                &receipt,
                reader,
                stdout,
                &mut trace,
            );
            copies += worker.actual_total - prior_count;
            match planned {
                Ok((scale, predicted, _)) => {
                    // Parent must durably accept the planning result before the
                    // real history resumes. This ACK grants no new history credit.
                    let resumed = (|| -> Result<NativeParityTrace, String> {
                        let ack =
                            read(reader)?.ok_or("Parent disconnected before continuation ACK")?;
                        if ack["op"] != "continue-history" {
                            return Err("Expected continuation ACK".into());
                        }
                        let permit: Permit = serde_json::from_value(ack["permit"].clone())
                            .map_err(|e| e.to_string())?;
                        if permit.reservation_id != budget.permit.reservation_id
                            || permit.reserved_steps != 960
                            || permit.worker_pid != std::process::id()
                            || permit.deadline_unix_s != budget.permit.deadline_unix_s
                        {
                            return Err(
                                "Continuation attempted to renew or alter history permission"
                                    .into(),
                            );
                        }
                        budget.check_wall_deadline()?;
                        sim.coordination
                            .as_mut()
                            .ok_or("Lost real command owner")?
                            .feedback_scale = scale;
                        NativeParityTrace::begin(&sim, &()).map_err(|e| format!("{e:?}"))
                    })();
                    match resumed {
                        Ok(observed) => {
                            actual_first = Some(observed);
                            expected_first = Some(predicted);
                        }
                        Err(error) => {
                            refusal = Some(error);
                            break;
                        }
                    }
                }
                Err(error) => {
                    refusal = Some(error);
                    break;
                }
            }
        }
        let before = sim.substeps_completed;
        let result = actual_tick(
            &mut sim,
            &mut budget,
            &mut diagnostic,
            &mut trace,
            "actual-history",
            actual_first.as_mut(),
            enabled,
        );
        let integrated = sim.substeps_completed - before;
        actual += integrated;
        if let Err(error) = worker.count(integrated) {
            refusal = Some(error);
            break;
        }
        if let Err(error) = result {
            refusal = Some(error);
            break;
        }
        if let (Some(predicted), Some(observed)) = (expected_first.take(), actual_first.take()) {
            if let Err(error) = native_kernel_admission(&predicted, &observed) {
                divergence = Some(
                    json!({"substep":sim.substeps_completed,"error":format!("{error:?}"),"predicted":predicted,"actual":observed}),
                );
                refusal = Some("First native prediction/actual mismatch".into());
                break;
            }
        }
    }
    worker.b_receipt = Some(receipt);
    worker.histories += 1;
    let ok = refusal.is_none() && actual == 960;
    let local_success = ok && diagnostic.passed;
    if id == "B" {
        worker.prior_b_success = local_success;
    }
    let result = outcome(ok, local_success);
    line(
        &mut report,
        &json!({"id":id,"headingDegrees":heading,"scope":"local-transfer-only",
        "actualNativeSteps":actual,"predictionNativeSteps":copies,"allNativeSteps":actual+copies,
        "reservedRealSteps":960,"localReservedRealSteps":budget.consumed,"planningCalls":planning_calls,
        "initial":initial,"final":sim.snapshot(),"lastOwner":sim.coordination.as_ref().map(|o|o.evidence()),
        "firstFailure":sim.first_failure,"refusal":refusal,"firstPredictionDivergence":divergence,
        "diagnostic":diagnostic,"recoveriesObserved":sim.counters.recoveries,"outcome":result,
        "localSuccessDefinition":"At least 132 consecutive qualifying native samples, starting before 2 s, with the full 4 s real history completed without refusal; passed is an interval latch, not a terminal stance certificate",
        "tracePath":trace_path,"workerActualAllOperations":worker.actual_total,"workerReservedAllOperations":worker.reserved_total}),
    )?;
    report.flush().map_err(|e| e.to_string())?;
    trace.flush().map_err(|e| e.to_string())?;
    report_result(stdout, "history", actual, result, output)
}

pub(crate) fn run() -> Result<(), String> {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut reader = stdin.lock();
    let mut writer = stdout.lock();
    send(
        &mut writer,
        &json!({"event":"ready","pid":std::process::id()}),
    )?;
    let mut worker = Worker::new();
    while let Some(request) = read(&mut reader)? {
        match request["op"].as_str() {
            Some("verify") => verify(&mut worker, &request, &mut writer)?,
            Some("history") => history(&mut worker, &request, &mut reader, &mut writer)?,
            _ => return Err("Unexpected worker operation".into()),
        }
    }
    Ok(())
}

pub(crate) fn main() {
    if let Err(error) = run() {
        eprintln!("coordination worker refused: {error}");
        std::process::exit(1);
    }
}
