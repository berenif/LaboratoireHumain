//! Explicit NDJSON worker. No test or production entrypoint invokes it.
//! Every integration requires a parent reservation and a local four-step debit.
use super::native_plan_preview::{
    NativeKernelAdmission, NativeParityTrace, PreviewBudget, PreviewPurpose, StaticPreviewBranch,
    StepReservation, compare_prefix, native_kernel_admission_complete,
};
use super::whole_body_controller::{self, ReferenceTasks};
use super::{contact_observation as journal, native_observation, support_state};
use crate::{Simulation, measurement, recovery::trajectory_program::Program};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    cell::RefCell,
    collections::BTreeSet,
    fs::{File, OpenOptions},
    io::{BufRead, BufWriter, Write},
    panic::{AssertUnwindSafe, catch_unwind},
    path::Path,
    rc::Rc,
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
    gate: Option<Rc<RefCell<IntegrationGate>>>,
}

/// A separate append-only checkpoint journal remains writable even if rich
/// state serialization fails. Counts distinguish admitted and completed calls;
/// an interrupted call is never reported as an exact native count.
struct IntegrationGate {
    reservation_id: String,
    deadline: Instant,
    deadline_unix_s: f64,
    pending: u64,
    abandoned: u64,
    next_clock: u64,
    admitted: u64,
    completed: u64,
    in_flight: bool,
    journal: BufWriter<File>,
}
impl IntegrationGate {
    fn check(&self) -> Result<(), String> {
        if Instant::now() >= self.deadline || unix_now()? >= self.deadline_unix_s {
            Err("Original batch wall deadline exhausted before integration".into())
        } else {
            Ok(())
        }
    }
    fn checkpoint(&mut self, event: &str) -> Result<(), String> {
        line(
            &mut self.journal,
            &json!({"event":event,"reservationId":self.reservation_id,
            "admittedNativeSteps":self.admitted,"completedNativeSteps":self.completed,
            "pendingNativeSlots":self.pending,"abandonedNativeSlots":self.abandoned,
            "actualNativeStepsExact":!self.in_flight && self.admitted==self.completed,
            "nextNativeClock":self.next_clock}),
        )?;
        self.journal.flush().map_err(|e| e.to_string())?;
        self.journal
            .get_ref()
            .sync_data()
            .map_err(|e| e.to_string())
    }
    fn reserve(&mut self, before: u64, steps: u64) -> Result<(), String> {
        self.check()?;
        if self.pending != 0 || self.in_flight {
            return Err("Previous native tick is incomplete".into());
        }
        self.next_clock = before;
        self.pending = steps;
        Ok(())
    }
    fn before(&mut self, clock: u64) -> Result<(), String> {
        self.check()?;
        if self.pending == 0 || self.in_flight || clock != self.next_clock {
            return Err("Native integration has no reserved exact clock".into());
        }
        self.pending -= 1;
        self.admitted += 1;
        self.in_flight = true;
        self.checkpoint("before-integration")?;
        // This is the last action before the original world.step call.
        self.check()
    }
    fn after(&mut self, clock: u64) -> Result<(), String> {
        if !self.in_flight || clock != self.next_clock {
            return Err("Completed native call has no matching admission".into());
        }
        self.completed += 1;
        self.next_clock += 1;
        self.in_flight = false;
        self.checkpoint("after-integration")
    }
}
thread_local! { static ACTIVE: RefCell<Option<Rc<RefCell<IntegrationGate>>>> = const { RefCell::new(None) }; }
struct ActiveScope(Option<Rc<RefCell<IntegrationGate>>>);
impl Drop for ActiveScope {
    fn drop(&mut self) {
        ACTIVE.with(|slot| *slot.borrow_mut() = self.0.take());
    }
}
impl LocalBudget {
    fn activate(&mut self, output: &str) -> Result<ActiveScope, String> {
        if self.gate.is_some() {
            return Err("Native permit activation replay".into());
        }
        let mut gate = IntegrationGate {
            reservation_id: self.permit.reservation_id.clone(),
            deadline: self.deadline,
            deadline_unix_s: self.permit.deadline_unix_s,
            pending: 0,
            abandoned: 0,
            next_clock: 0,
            admitted: 0,
            completed: 0,
            in_flight: false,
            journal: new_file(&format!("{output}.accounting.jsonl"))?,
        };
        gate.checkpoint("permit-activated")?;
        let gate = Rc::new(RefCell::new(gate));
        self.gate = Some(gate.clone());
        Ok(ActiveScope(ACTIVE.with(|slot| slot.replace(Some(gate)))))
    }
    fn completed(&self) -> u64 {
        self.gate.as_ref().map_or(0, |g| g.borrow().completed)
    }
    fn finish_branch(&mut self) -> Result<(), String> {
        let mut gate = self
            .gate
            .as_ref()
            .ok_or("Native accounting not active")?
            .borrow_mut();
        if gate.in_flight || gate.admitted != gate.completed {
            return Err("Cannot abandon an uncertain native integration".into());
        }
        // A refused branch abandons its unused tick slots. The full four-step
        // debit in `consumed` is retained; the next branch needs a fresh debit.
        if gate.pending != 0 {
            gate.abandoned += gate.pending;
            gate.pending = 0;
            gate.checkpoint("abandon-unused-tick-slots-without-refund")?;
        }
        Ok(())
    }
    fn exact(&self) -> bool {
        self.gate.as_ref().is_some_and(|g| {
            let g = g.borrow();
            !g.in_flight && g.completed == g.admitted
        })
    }
    fn actual_json(&self, count: u64) -> Value {
        if self.exact() {
            json!(count)
        } else {
            Value::Null
        }
    }
    fn accounting(&self) -> Value {
        self.gate.as_ref().map_or(Value::Null, |g| {
            let g = g.borrow();
            json!({"admittedNativeSteps":g.admitted,"completedNativeSteps":g.completed,
            "actualNativeStepsExact":!g.in_flight && g.admitted==g.completed,
                "abandonedNativeSlots":g.abandoned})
        })
    }
}
pub(crate) fn before_integration(s: &Simulation) -> Result<(), String> {
    ACTIVE.with(|slot| {
        slot.borrow()
            .as_ref()
            .ok_or("No active coordination permit")?
            .borrow_mut()
            .before(s.substeps_completed)
    })
}
pub(crate) fn after_integration(s: &Simulation) -> Result<(), String> {
    ACTIVE.with(|slot| {
        slot.borrow()
            .as_ref()
            .ok_or("Lost active coordination permit")?
            .borrow_mut()
            .after(s.substeps_completed)
    })
}
fn catch_failure<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    catch_unwind(AssertUnwindSafe(f)).map_err(|panic| {
        let text = panic
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| panic.downcast_ref::<&str>().copied())
            .unwrap_or("non-string panic");
        format!("Unwound native worker failure; no further integration: {text}")
    })
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
        self.gate
            .as_ref()
            .ok_or("Native accounting not active")?
            .borrow_mut()
            .reserve(request.before_substep, 1)?;
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
        self.gate
            .as_ref()
            .ok_or("Native accounting not active")?
            .borrow_mut()
            .reserve(request.before_substep, 4)?;
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
    fatal: bool,
    accounting_uncertain: bool,
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
            fatal: false,
            accounting_uncertain: false,
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
            gate: None,
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
    actual: Value,
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

/// Observation memory participates in parity exactly like controller memory.
/// Keeping the derived sample and speed also covers every input to selection.
#[derive(Clone, Default, Serialize)]
struct DiagnosticMemory {
    diagnostic: support_state::LocalSupportDiagnostic,
    sample: Option<support_state::LocalSupportSample>,
    center_of_mass_speed_mps: Option<f64>,
}
impl DiagnosticMemory {
    fn observe(
        &mut self,
        observation: &support_state::SupportObservation,
    ) -> Result<support_state::LocalSupportSample, String> {
        let sample = self.diagnostic.observe(observation)?;
        self.center_of_mass_speed_mps = Some(
            observation
                .center_of_mass_velocity_mps
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt(),
        );
        self.sample = Some(sample.clone());
        Ok(sample)
    }
}

fn actual_tick(
    sim: &mut Simulation,
    budget: &mut LocalBudget,
    diagnostic: &mut DiagnosticMemory,
    trace: &mut impl Write,
    kind: &str,
    mut parity: Option<&mut NativeParityTrace>,
    predicted: Option<&NativeParityTrace>,
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
    let result = catch_failure(|| {
        sim.advance_tick_observed(|s, substep| {
            pending_ccd = journal::take_records();
            if s.substeps_completed != before + u64::from(substep) + 1 {
                return Err(failure(
                    s,
                    substep,
                    "Actual native step clock mismatch".into(),
                ));
            }
            let observed = native_observation::observe(s, 0, &pending_ccd)
                .map_err(|e| failure(s, substep, e))?;
            let sample = diagnostic
                .observe(&observed)
                .map_err(|e| failure(s, substep, e))?;
            if let Some(p) = parity.as_deref_mut() {
                p.record(s, &*diagnostic)
                    .map_err(|e| failure(s, substep, format!("{e:?}")))?;
                if let Some(expected) = predicted {
                    compare_prefix(expected, p)
                        .map_err(|e| failure(s, substep, format!("{e:?}")))?;
                }
            }
            line(
                trace,
                &native_frame(s, &observed, &sample, &pending_ccd, kind),
            )
            .map_err(|e| failure(s, substep, e))?;
            pending_ccd.clear();
            if enforce_local_deadline
                && local_deadline_irreversible(&diagnostic.diagnostic, observed.time_s)
            {
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
        })
    })
    .and_then(|r| r.map_err(|e| format!("{e:?}")));
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
    if let Some(p) = parity {
        p.record_post_tick(sim, &*diagnostic)
            .map_err(|e| format!("{e:?}"))?;
        if let Some(expected) = predicted {
            compare_prefix(expected, p).map_err(|e| format!("{e:?}"))?;
        }
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
    let _scope = budget.activate(output)?;
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
        let mut real_diagnostic = DiagnosticMemory::default();
        let mut copy =
            StaticPreviewBranch::capture(&real, &real_diagnostic).map_err(|e| format!("{e:?}"))?;
        let predicted = RefCell::new(
            NativeParityTrace::begin(copy.simulation(), copy.memory())
                .map_err(|e| format!("{e:?}"))?,
        );
        let mut actual_trace =
            NativeParityTrace::begin(&real, &real_diagnostic).map_err(|e| format!("{e:?}"))?;
        let before_copy = budget.completed();
        let copied = catch_failure(|| {
            copy.verify_tick_with_post_tick(
                u64::from(enabled),
                &mut budget,
                |s, _, memory| {
                    let observed = native_observation::observe(s, 0, &[])?;
                    let sample = memory.observe(&observed)?;
                    predicted
                        .borrow_mut()
                        .record(s, memory)
                        .map_err(|e| format!("{e:?}"))?;
                    line(
                        &mut trace,
                        &native_frame(s, &observed, &sample, &[], &format!("verify-{name}-copy")),
                    )?;
                    Ok(())
                },
                |s, memory| {
                    predicted
                        .borrow_mut()
                        .record_post_tick(s, memory)
                        .map_err(|e| format!("{e:?}"))
                },
            )
        });
        let predicted = predicted.into_inner();
        let copied_steps = budget.completed() - before_copy;
        actual += copied_steps;
        worker.count(copied_steps)?;
        let copied_refusal = match copied {
            Ok(run) => {
                if let Err(error) = budget.finish_branch() {
                    worker.fatal = true;
                    Some(error)
                } else {
                    run.refusal.map(|e| format!("{e:?}"))
                }
            }
            Err(error) => {
                worker.fatal = true;
                Some(error)
            }
        };
        worker.fatal |= !budget.exact();
        worker.accounting_uncertain |= !budget.exact();
        if let Some(error) = copied_refusal {
            cases.push(json!({"name":name,"predicted":predicted,
                "copyLastState":copy.simulation().snapshot(),
                "copyLastOwner":copy.simulation().coordination.as_ref().map(|o|o.evidence()),
                "copyFirstFailure":copy.simulation().first_failure,"copyDiagnostic":copy.memory()}));
            refusal = Some(format!("{name} copy: {error:?}"));
            break;
        }
        if let Err(error) = compare_prefix(&predicted, &actual_trace) {
            refusal = Some(format!("Initial native parity mismatch: {error:?}"));
            break;
        }
        let before_real = budget.completed();
        let result = actual_tick(
            &mut real,
            &mut budget,
            &mut real_diagnostic,
            &mut trace,
            &format!("verify-{name}-actual"),
            Some(&mut actual_trace),
            Some(&predicted),
            false,
        );
        let real_steps = budget.completed() - before_real;
        actual += real_steps;
        worker.count(real_steps)?;
        worker.fatal |= !budget.exact()
            || result
                .as_ref()
                .err()
                .is_some_and(|e| e.contains("Unwound native worker failure"));
        worker.accounting_uncertain |= !budget.exact();
        if let Err(error) = result {
            cases.push(
                json!({"name":name,"predicted":predicted,"actual":actual_trace,
                "actualLastState":real.snapshot(),"actualFirstFailure":real.first_failure}),
            );
            refusal = Some(error);
            break;
        }
        match native_kernel_admission_complete(&predicted, &actual_trace) {
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
        &json!({"scope":"native-parity-verification","actualNativeSteps":budget.actual_json(actual),
        "nativeAccounting":budget.accounting(),
        "reservedNativeSteps":16,"consumedLocalReservations":budget.consumed,"refusal":refusal,
        "cases":cases,"tracePath":trace_path,"outcome":result,"recoveryQualified":false}),
    )?;
    report.flush().map_err(|e| e.to_string())?;
    report.get_ref().sync_data().map_err(|e| e.to_string())?;
    trace.flush().map_err(|e| e.to_string())?;
    report_result(stdout, "verify", budget.actual_json(actual), result, output)
}

fn planning(
    worker: &mut Worker,
    sim: &Simulation,
    diagnostic: &DiagnosticMemory,
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
    let _scope = budget.activate(output)?;
    let mut cases = Vec::new();
    let mut best: Option<([f64; 5], f64, NativeParityTrace)> = None;
    let mut actual = 0;
    for (index, scale) in [0., 0.5, 1., 1.5].into_iter().enumerate() {
        let mut branch =
            StaticPreviewBranch::capture(sim, diagnostic).map_err(|e| format!("{e:?}"))?;
        branch
            .set_feedback_scale(scale)
            .map_err(|e| format!("{e:?}"))?;
        let first_four = RefCell::new(
            NativeParityTrace::begin(branch.simulation(), branch.memory())
                .map_err(|e| format!("{e:?}"))?,
        );
        let start = sim.substeps_completed;
        let before_copy = budget.completed();
        let predicted = catch_failure(|| {
            branch.preview_with_post_tick(
                receipt,
                index as u64,
                480,
                &mut budget,
                |s, _, memory| {
                    let observation = native_observation::observe(s, 0, &[])?;
                    let sample = memory.observe(&observation)?;
                    if s.substeps_completed <= start + 4 {
                        first_four
                            .borrow_mut()
                            .record(s, memory)
                            .map_err(|e| format!("{e:?}"))?;
                    }
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
                    if local_deadline_irreversible(&memory.diagnostic, observation.time_s) {
                        return Err(
                            "Local diagnostic can no longer begin its stable interval before 2 s"
                                .into(),
                        );
                    }
                    Ok(())
                },
                |s, memory| {
                    if s.substeps_completed == start + 4 {
                        first_four
                            .borrow_mut()
                            .record_post_tick(s, memory)
                            .map_err(|e| format!("{e:?}"))?;
                    }
                    Ok(())
                },
            )
        });
        let first_four = first_four.into_inner();
        let integrated = budget.completed() - before_copy;
        let predicted_refusal = match predicted {
            Ok(run) => {
                // Uncertain accounting is fatal and preserves its upper bound.
                if budget.finish_branch().is_err() {
                    worker.fatal = true;
                }
                run.refusal.map(|e| format!("{e:?}"))
            }
            Err(error) => {
                worker.fatal = true;
                Some(error)
            }
        };
        worker.fatal |= !budget.exact();
        worker.accounting_uncertain |= !budget.exact();
        actual += integrated;
        worker.count(integrated)?;
        let last = branch
            .memory()
            .sample
            .as_ref()
            .zip(branch.memory().center_of_mass_speed_mps);
        let usable =
            predicted_refusal.is_none() && integrated == 480 && last.is_some() && !worker.fatal;
        let score = last.as_ref().map(|(s, speed)| {
            [
                f64::from(u8::from(s.passed)),
                f64::from(u8::from(s.com_inside && s.capture_inside)),
                s.heel_loads_n[0].min(s.heel_loads_n[1]),
                s.support_fraction,
                -speed,
            ]
        });
        cases.push(json!({"feedbackScale":scale,"usable":usable,"actualNativeSteps":budget.actual_json(integrated),
            "knownCompletedNativeSteps":integrated,
            "refusal":predicted_refusal,"score":score,
            "terminalDiagnostic":last.as_ref().map(|v|&v.0),"terminalState":branch.simulation().snapshot(),
            "terminalOwner":branch.simulation().coordination.as_ref().map(|o|o.evidence()),
            "firstFailure":branch.simulation().first_failure,
            "firstPredictedSteps":first_four}));
        if usable
            && score.is_some_and(|score| best.as_ref().is_none_or(|(prior, _, _)| score > *prior))
        {
            best = Some((score.unwrap(), scale, first_four));
        }
        if worker.fatal {
            best = None;
            break;
        }
    }
    let ok = best.is_some();
    let result = outcome(ok, false);
    line(
        &mut report,
        &json!({"scope":"four-frozen-native-previews","historyStep":sim.substeps_completed,
        "actualNativeSteps":budget.actual_json(actual),"nativeAccounting":budget.accounting(),"reservedNativeSteps":1920,"consumedLocalReservations":budget.consumed,
        "selectedScale":best.as_ref().map(|(_,s,_)|s),"cases":cases,"outcome":result}),
    )?;
    report.flush().map_err(|e| e.to_string())?;
    report.get_ref().sync_data().map_err(|e| e.to_string())?;
    trace.flush().map_err(|e| e.to_string())?;
    report_result(
        stdout,
        "planning",
        budget.actual_json(actual),
        result,
        output,
    )?;
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
    let _scope = budget.activate(output)?;
    let mut sim = whole_body_controller::scene(
        inputs.program,
        inputs.tasks,
        enabled,
        (heading as f32).to_radians(),
    )?;
    let initial = sim.snapshot();
    let mut diagnostic = DiagnosticMemory::default();
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
                        let observed = NativeParityTrace::begin(&sim, &diagnostic)
                            .map_err(|e| format!("{e:?}"))?;
                        compare_prefix(&predicted, &observed).map_err(|e| {
                            format!("Initial selected native parity mismatch: {e:?}")
                        })?;
                        Ok(observed)
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
        let before = budget.completed();
        let result = actual_tick(
            &mut sim,
            &mut budget,
            &mut diagnostic,
            &mut trace,
            "actual-history",
            actual_first.as_mut(),
            expected_first.as_ref(),
            enabled,
        );
        let integrated = budget.completed() - before;
        worker.fatal |= !budget.exact()
            || result
                .as_ref()
                .err()
                .is_some_and(|e| e.contains("Unwound native worker failure"));
        worker.accounting_uncertain |= !budget.exact();
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
            if let Err(error) = native_kernel_admission_complete(&predicted, &observed) {
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
    let local_success = ok && diagnostic.diagnostic.passed;
    if id == "B" {
        worker.prior_b_success = local_success;
    }
    let result = outcome(ok, local_success);
    line(
        &mut report,
        &json!({"id":id,"headingDegrees":heading,"scope":"local-transfer-only",
        "actualNativeSteps":budget.actual_json(actual),"nativeAccounting":budget.accounting(),"predictionNativeSteps":if worker.accounting_uncertain {Value::Null}else{json!(copies)},
        "allNativeSteps":if worker.accounting_uncertain {Value::Null}else{json!(actual+copies)},
        "knownCompletedPredictionSteps":copies,"knownCompletedAllSteps":actual+copies,
        "reservedRealSteps":960,"localReservedRealSteps":budget.consumed,"planningCalls":planning_calls,
        "initial":initial,"final":sim.snapshot(),"lastOwner":sim.coordination.as_ref().map(|o|o.evidence()),
        "firstFailure":sim.first_failure,"refusal":refusal,"firstPredictionDivergence":divergence,
        "diagnostic":diagnostic.diagnostic,"diagnosticMemory":diagnostic,"recoveriesObserved":sim.counters.recoveries,"outcome":result,
        "localSuccessDefinition":"At least 132 consecutive qualifying native samples, starting before 2 s, with the full 4 s real history completed without refusal; passed is an interval latch, not a terminal stance certificate",
        "tracePath":trace_path,"workerActualAllOperations":if worker.accounting_uncertain {Value::Null}else{json!(worker.actual_total)},
        "workerKnownCompletedAllOperations":worker.actual_total,"workerAccountingExact":!worker.accounting_uncertain,
        "workerReservedAllOperations":worker.reserved_total}),
    )?;
    report.flush().map_err(|e| e.to_string())?;
    report.get_ref().sync_data().map_err(|e| e.to_string())?;
    trace.flush().map_err(|e| e.to_string())?;
    report_result(
        stdout,
        "history",
        budget.actual_json(actual),
        result,
        output,
    )
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
        if worker.fatal {
            return Err("Native worker halted after a technical or accounting failure".into());
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

#[cfg(test)]
mod pure_tests {
    // These tests use only scalar accounting, JSON, and temporary files. No
    // Simulation/Model is constructed and no native kernel or FK is invoked.
    use super::*;
    use serde::{Deserialize, Serialize};

    fn permit(id: &str, steps: u64) -> Value {
        json!({"reservationId":id,"reservedSteps":steps,"deadlineUnixS":unix_now().unwrap()+60.,
            "remainingWallSeconds":60.,"workerPid":std::process::id()})
    }
    fn path(name: &str) -> String {
        std::env::temp_dir()
            .join(format!(
                "lh-local-pair-pure-{name}-{}-{}",
                std::process::id(),
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ))
            .to_str()
            .unwrap()
            .to_owned()
    }
    fn request(clock: u64) -> StepReservation {
        StepReservation {
            branch_id: 0,
            before_substep: clock,
            preview_index: 0,
            purpose: PreviewPurpose::ParityVerification,
        }
    }
    #[test]
    fn direct_world_bytes_preserve_u128_signed_zero_and_adjacent_values() {
        #[derive(Serialize, Deserialize)]
        struct State {
            mask: u128,
            zero: f32,
        }
        for mask in [u128::MAX, u128::MAX - 1, u128::from(u64::MAX) + 1] {
            // This is the actual NativeParityTrace world representation:
            // direct typed serialization, then an opaque byte array in Value.
            let bytes = serde_json::to_vec(&State { mask, zero: -0.0 }).unwrap();
            let report = json!({"world":bytes});
            let recovered: Vec<u8> = serde_json::from_value(report["world"].clone()).unwrap();
            let state: State = serde_json::from_slice(&recovered).unwrap();
            assert_eq!(state.mask, mask);
            assert_eq!(state.zero.to_bits(), (-0.0_f32).to_bits());
        }
    }
    #[test]
    fn archived_initial_json_roundtrip_without_world_construction() {
        let archived:Value=serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-native-serialization-2026-10-13/archived-initial-world.json"))).unwrap();
        let bytes = serde_json::to_vec(&archived).unwrap();
        let envelope = json!({"world":bytes});
        let recovered: Vec<u8> = serde_json::from_value(envelope["world"].clone()).unwrap();
        assert_eq!(bytes, recovered);
    }
    #[test]
    fn gate_requires_exact_clock_and_complete_tick_reservation() {
        let mut worker = Worker::new();
        let mut budget = worker
            .accept_permit(&permit("clock", 4), 4, PreviewPurpose::ParityVerification)
            .unwrap();
        let output = path("clock");
        let scope = budget.activate(&output).unwrap();
        budget.reserve_four_before_tick(request(12)).unwrap();
        let gate = budget.gate.as_ref().unwrap();
        assert!(gate.borrow_mut().before(13).is_err());
        assert_eq!(gate.borrow().admitted, 0);
        for clock in 12..16 {
            gate.borrow_mut().before(clock).unwrap();
            gate.borrow_mut().after(clock).unwrap();
        }
        assert_eq!(budget.completed(), 4);
        assert!(budget.exact());
        assert!(gate.borrow_mut().before(16).is_err());
        assert!(budget.reserve_four_before_tick(request(16)).is_err());
        let rows = std::fs::read_to_string(format!("{output}.accounting.jsonl")).unwrap();
        assert_eq!(rows.lines().count(), 9);
        drop(scope);
        drop(budget);
        std::fs::remove_file(format!("{output}.accounting.jsonl")).unwrap();
    }
    #[test]
    fn refused_branch_discards_slots_without_refunding_tick_debit() {
        let mut worker = Worker::new();
        let mut budget = worker
            .accept_permit(&permit("discard", 8), 8, PreviewPurpose::ParityVerification)
            .unwrap();
        let output = path("discard");
        let scope = budget.activate(&output).unwrap();
        budget.reserve_four_before_tick(request(12)).unwrap();
        let gate = budget.gate.as_ref().unwrap().clone();
        gate.borrow_mut().before(12).unwrap();
        gate.borrow_mut().after(12).unwrap();
        budget.finish_branch().unwrap();
        assert_eq!(budget.consumed, 4);
        assert_eq!(gate.borrow().abandoned, 3);
        assert!(
            std::fs::read_to_string(format!("{output}.accounting.jsonl"))
                .unwrap()
                .contains("abandon-unused-tick-slots-without-refund")
        );
        assert!(gate.borrow_mut().before(13).is_err());
        budget.reserve_four_before_tick(request(12)).unwrap();
        assert_eq!(budget.consumed, 8);
        gate.borrow_mut().before(12).unwrap();
        gate.borrow_mut().after(12).unwrap();
        assert_eq!(budget.completed(), 2);
        budget.finish_branch().unwrap();
        assert!(budget.reserve_four_before_tick(request(12)).is_err());
        drop(scope);
        drop(budget);
        drop(gate);
        std::fs::remove_file(format!("{output}.accounting.jsonl")).unwrap();
    }
    #[test]
    fn expired_gate_denies_before_any_admission() {
        let mut worker = Worker::new();
        let mut budget = worker
            .accept_permit(&permit("expired", 4), 4, PreviewPurpose::ParityVerification)
            .unwrap();
        let output = path("expired");
        let scope = budget.activate(&output).unwrap();
        budget.reserve_four_before_tick(request(0)).unwrap();
        let gate = budget.gate.as_ref().unwrap();
        gate.borrow_mut().deadline = Instant::now();
        assert!(gate.borrow_mut().before(0).is_err());
        assert_eq!(gate.borrow().admitted, 0);
        drop(scope);
        drop(budget);
        std::fs::remove_file(format!("{output}.accounting.jsonl")).unwrap();
    }
    #[test]
    fn interrupted_admission_is_unknown_and_stops_reuse() {
        let mut worker = Worker::new();
        let mut budget = worker
            .accept_permit(&permit("partial", 4), 4, PreviewPurpose::ParityVerification)
            .unwrap();
        let output = path("partial");
        let scope = budget.activate(&output).unwrap();
        budget.reserve_four_before_tick(request(0)).unwrap();
        budget
            .gate
            .as_ref()
            .unwrap()
            .borrow_mut()
            .before(0)
            .unwrap();
        assert!(!budget.exact());
        assert_eq!(budget.actual_json(0), Value::Null);
        assert!(
            budget
                .gate
                .as_ref()
                .unwrap()
                .borrow_mut()
                .before(1)
                .is_err()
        );
        assert_eq!(budget.accounting()["admittedNativeSteps"], 1);
        assert_eq!(budget.accounting()["completedNativeSteps"], 0);
        drop(scope);
        drop(budget);
        std::fs::remove_file(format!("{output}.accounting.jsonl")).unwrap();
    }
    #[test]
    fn completion_checkpoint_survives_later_serialization_panic() {
        let mut worker = Worker::new();
        let mut budget = worker
            .accept_permit(&permit("panic", 4), 4, PreviewPurpose::ParityVerification)
            .unwrap();
        let output = path("panic");
        let scope = budget.activate(&output).unwrap();
        budget.reserve_four_before_tick(request(0)).unwrap();
        let result = catch_failure(|| {
            let gate = budget.gate.as_ref().unwrap();
            gate.borrow_mut().before(0).unwrap();
            gate.borrow_mut().after(0).unwrap(); // scalar checkpoint; no native call
            let _ = json!({"mask":u128::MAX}); // reproduces former Value failure
        });
        assert!(result.is_err());
        assert_eq!(budget.completed(), 1);
        assert!(budget.exact());
        let text = std::fs::read_to_string(format!("{output}.accounting.jsonl")).unwrap();
        let last: Value = serde_json::from_str(text.lines().last().unwrap()).unwrap();
        assert_eq!(last["completedNativeSteps"], 1);
        assert_eq!(last["actualNativeStepsExact"], true);
        drop(scope);
        drop(budget);
        std::fs::remove_file(format!("{output}.accounting.jsonl")).unwrap();
    }
    #[test]
    fn permit_replay_purpose_and_global_count_are_bounded() {
        let mut worker = Worker::new();
        let raw = permit("once", 16);
        let mut budget = worker
            .accept_permit(&raw, 16, PreviewPurpose::ParityVerification)
            .unwrap();
        assert!(
            worker
                .accept_permit(&raw, 16, PreviewPurpose::ParityVerification)
                .is_err()
        );
        assert!(
            budget
                .admit_preview(PreviewPurpose::Candidate, 480)
                .is_err()
        );
        assert!(
            budget
                .admit_preview(PreviewPurpose::ParityVerification, 8)
                .is_err()
        );
        assert!(
            budget
                .admit_preview(PreviewPurpose::ParityVerification, 4)
                .is_ok()
        );
        worker.count(16).unwrap();
        assert!(worker.count(1).is_err());
    }
    #[test]
    fn local_deadline_keeps_interval_started_before_two_seconds() {
        let mut diagnostic = support_state::LocalSupportDiagnostic::default();
        assert!(!local_deadline_irreversible(&diagnostic, 1.999));
        assert!(local_deadline_irreversible(&diagnostic, 2.));
        diagnostic.stable_started_s = Some(1.999);
        assert!(!local_deadline_irreversible(&diagnostic, 2.4));
        diagnostic.stable_started_s = None;
        assert!(local_deadline_irreversible(&diagnostic, 2.4));
        diagnostic.passed = true;
        assert!(!local_deadline_irreversible(&diagnostic, 4.));
    }
}
