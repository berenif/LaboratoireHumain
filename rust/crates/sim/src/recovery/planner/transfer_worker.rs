//! Future single-history driver. Merely linking this module performs no work.
//! A separate explicit entrypoint accepts exactly one durable parent operation.
//! This is a child of transition_worker to reuse its integration accounting gate.
use super::super::{native_plan_preview, observed_raccord::ObservedCommandEntry};
use super::*;

const PREFIX: u64 = 36;
const TECHNICAL: u64 = PREFIX + 16;
const REAL_CAP: u64 = 6244;
const FORECAST_CAP: u64 = 260;
const HORIZON: u32 = 480;
const REAL_WINDOW: u64 = 24;
const HISTORY: u64 = REAL_CAP + FORECAST_CAP * HORIZON as u64;
const TOTAL: u64 = TECHNICAL + HISTORY;
const LOCAL_DEADLINE: u64 = PREFIX + 480;

#[derive(Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct TransferInputs {
    program_path: String,
    tasks_path: String,
    entry_path: String,
    prefix_frames_path: String,
}

struct PreparedInputs {
    frozen: FrozenInputs,
    entry: ObservedCommandEntry,
    prefix: Vec<Value>,
}

/// Only the inert schema addition is allowed. Every existing decision, motor,
/// contact, CCD and snapshot field remains part of the comparison.
fn archived_projection(mut value: Value) -> Result<Value, String> {
    let object = value
        .as_object_mut()
        .ok_or("Native archive row is not an object")?;
    object.remove("kind");
    let owner = object
        .get_mut("owner")
        .and_then(Value::as_object_mut)
        .ok_or("Missing archived owner")?;
    if owner.get("transfer").is_some_and(|v| !v.is_null()) {
        return Err("Prefix already contains an active transfer adapter".into());
    }
    owner.remove("transfer");
    Ok(value)
}

/// serde_json Value equality merges +0.0 and -0.0. Preserve those bits and
/// integer spelling separately, including the archived motor impulses.
fn exact_json(expected: &Value, actual: &Value, path: &str) -> Result<(), String> {
    let mismatch = || format!("Archived prefix mismatch at {path}");
    match (expected, actual) {
        (Value::Number(a), Value::Number(b)) => {
            if a.is_f64() && b.is_f64() {
                if a.as_f64().unwrap().to_bits() != b.as_f64().unwrap().to_bits() {
                    return Err(mismatch());
                }
            } else if a.to_string() != b.to_string() {
                return Err(mismatch());
            }
        }
        (Value::Array(a), Value::Array(b)) if a.len() == b.len() => {
            for (i, (x, y)) in a.iter().zip(b).enumerate() {
                exact_json(x, y, &format!("{path}[{i}]"))?;
            }
        }
        (Value::Object(a), Value::Object(b)) if a.len() == b.len() => {
            for (key, x) in a {
                let y = b.get(key).ok_or_else(mismatch)?;
                exact_json(x, y, &format!("{path}.{key}"))?;
            }
        }
        _ if expected == actual => (),
        _ => return Err(mismatch()),
    }
    Ok(())
}

fn compare_archived(expected: &Value, actual: &Value) -> Result<(), String> {
    exact_json(
        &archived_projection(expected.clone())?,
        &archived_projection(actual.clone())?,
        "nativeFrame",
    )
}

fn prepare_inputs(worker: &mut Worker, raw: &Value) -> Result<PreparedInputs, String> {
    let input: TransferInputs = serde_json::from_value(raw.clone()).map_err(|e| e.to_string())?;
    let frozen =
        worker.inputs(&json!({"programPath":input.program_path,"tasksPath":input.tasks_path}))?;
    let entry_bytes = std::fs::read(&input.entry_path).map_err(|e| e.to_string())?;
    let prefix_bytes = std::fs::read(&input.prefix_frames_path).map_err(|e| e.to_string())?;
    let archive: Value = serde_json::from_slice(&entry_bytes).map_err(|e| e.to_string())?;
    let prefix: Value = serde_json::from_slice(&prefix_bytes).map_err(|e| e.to_string())?;
    let rows = prefix["prefixFrames"]
        .as_array()
        .ok_or("Missing full archived prefix")?
        .clone();
    if archive["nativeSubstep"] != PREFIX
        || archive["headingDegrees"] != 0
        || archive["physicalPrefixStepsRequiredForExactReplay"] != PREFIX
        || prefix["schema"] != 1
        || rows.len() != PREFIX as usize
        || rows
            .iter()
            .enumerate()
            .any(|(i, r)| r["nativeSubstep"] != (i as u64 + 1))
    {
        return Err("Transfer inputs differ from the sole archived A36 entry".into());
    }
    compare_archived(&archive["archivedNativeRow"], rows.last().unwrap())?;
    let entry: ObservedCommandEntry =
        serde_json::from_value(archive["commandEntry"].clone()).map_err(|e| e.to_string())?;
    if entry.native_substep != PREFIX {
        return Err("Command history entry is not native boundary36".into());
    }
    Ok(PreparedInputs {
        frozen,
        entry,
        prefix: rows,
    })
}

fn accept_both(worker: &mut Worker, request: &Value) -> Result<(LocalBudget, LocalBudget), String> {
    let a: Permit =
        serde_json::from_value(request["technicalPermit"].clone()).map_err(|e| e.to_string())?;
    let b: Permit =
        serde_json::from_value(request["historyPermit"].clone()).map_err(|e| e.to_string())?;
    if a.reservation_id == b.reservation_id
        || a.deadline_unix_s.to_bits() != b.deadline_unix_s.to_bits()
        || a.worker_pid != b.worker_pid
        || worker.reserved_total != 0
    {
        return Err(
            "Transfer permits need distinct IDs, one PID and the same original deadline".into(),
        );
    }
    let mut technical = worker.accept_permit(
        &request["technicalPermit"],
        TECHNICAL,
        PreviewPurpose::ParityVerification,
    )?;
    let history = worker.accept_permit(
        &request["historyPermit"],
        HISTORY,
        PreviewPurpose::Candidate,
    )?;
    if worker.reserved_total != TOTAL {
        return Err("Wrong transfer reservation total".into());
    }
    // The second acceptance may shorten the monotonic deadline, never extend it.
    technical.deadline = history.deadline;
    Ok((technical, history))
}

#[derive(Default, Serialize)]
#[serde(rename_all = "camelCase")]
struct Audit {
    technical_prefix_native_steps: u64,
    technical_parity_native_steps: u64,
    real_history_native_steps: u64,
    forecast_calls: u64,
    forecast_requested_steps: u64,
    forecast_actual_steps: u64,
    technical_parity_passed: bool,
    local_success: bool,
    original_terminal_passed: bool,
    history_completed: bool,
    first_local_ready_substep: Option<u64>,
    forecast_summaries: Vec<Value>,
    technical_pairs: Vec<Value>,
    phase: String,
}
impl Audit {
    fn reserve_forecast(&mut self, at: u64) -> Result<(), String> {
        if !self.technical_parity_passed
            || at < PREFIX
            || at % 4 != 0
            || at >= REAL_CAP
            || self.forecast_calls >= FORECAST_CAP
        {
            return Err("Forecast phase, clock, or frozen call cap refused".into());
        }
        if at != PREFIX + self.forecast_calls * REAL_WINDOW {
            return Err(
                "Forecast must follow exactly24 real substeps; no retries or faster cadence".into(),
            );
        }
        self.forecast_calls += 1;
        self.forecast_requested_steps += u64::from(HORIZON);
        Ok(())
    }
    fn reserve_real_tick(&self, at: u64) -> Result<(), String> {
        if at % 4 != 0 || at.checked_add(4).is_none_or(|n| n > REAL_CAP) {
            return Err("Original global history cap reached before the next tick".into());
        }
        Ok(())
    }
}

/// Observe and compare inside the unchanged kernel callback, including step36
/// before its outer tick stamp changes from8 to9. A mismatch stops that tick.
fn prefix_tick(
    sim: &mut Simulation,
    budget: &mut LocalBudget,
    memory: &mut DiagnosticMemory,
    trace: &mut impl Write,
    expected: &[Value],
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
    let result = catch_failure(|| {
        sim.advance_tick_observed(|s, substep| {
            let checked = (|| {
                let ccd = journal::take_records();
                if s.substeps_completed != before + u64::from(substep) + 1 {
                    return Err("Prefix clock mismatch".into());
                }
                let observation = native_observation::observe(s, 0, &ccd)?;
                let sample = memory.observe(&observation)?;
                let frame = native_frame(s, &observation, &sample, &ccd, "actual-history");
                line(trace, &frame)?;
                let expected = expected
                    .get((s.substeps_completed - 1) as usize)
                    .ok_or("Prefix exceeded archive")?;
                compare_archived(expected, &frame)?;
                budget.check_wall_deadline()
            })();
            checked.map_err(|e| failure(s, substep, e))
        })
    })
    .and_then(|result| result.map_err(|e| format!("{e:?}")));
    let ccd = journal::take_records();
    journal::finish();
    if let Err(ref error) = result {
        line(
            trace,
            &json!({"kind":"prefix-refusal","nativeSubstep":sim.substeps_completed,
            "error":error,"unobservedCcd":ccd,"snapshot":sim.snapshot()}),
        )?;
    }
    result
}

fn replay_prefix(
    inputs: &PreparedInputs,
    budget: &mut LocalBudget,
    audit: &mut Audit,
    trace: &mut impl Write,
    technical: bool,
) -> Result<(Box<Simulation>, DiagnosticMemory), String> {
    budget.check_wall_deadline()?;
    let mut sim = Box::new(whole_body_controller::scene(
        inputs.frozen.program.clone(),
        inputs.frozen.tasks.clone(),
        false,
        0.,
    )?);
    let mut memory = DiagnosticMemory::default();
    line(
        trace,
        &json!({"event":"prefix-start","technical":technical,"expectedSteps":PREFIX}),
    )?;
    for _ in 0..PREFIX / 4 {
        let before = budget.completed();
        let result = prefix_tick(&mut sim, budget, &mut memory, trace, &inputs.prefix);
        let actual = budget.completed() - before;
        if technical {
            audit.technical_prefix_native_steps += actual;
        } else {
            audit.real_history_native_steps += actual;
        }
        result?;
    }
    if sim.substeps_completed != PREFIX || sim.tick * 4 != PREFIX {
        return Err("A36 prefix did not finish exactly nine native ticks".into());
    }
    Ok((sim, memory))
}

fn attach(sim: &mut Simulation, entry: &ObservedCommandEntry) -> Result<(), String> {
    let observation = native_observation::observe(sim, 0, &[])?;
    let mut owner = sim
        .coordination
        .take()
        .ok_or("Missing observed prefix owner")?;
    let result = owner.enable_observed_transfer(&sim.model, &observation, entry.clone());
    sim.coordination = Some(owner);
    result
}

fn technical(
    inputs: &PreparedInputs,
    budget: &mut LocalBudget,
    audit: &mut Audit,
    trace: &mut impl Write,
) -> Result<NativeKernelAdmission, String> {
    audit.phase = "technical-prefix".into();
    let (mut base, base_memory) = replay_prefix(inputs, budget, audit, trace, true)?;
    attach(&mut base, &inputs.entry)?;
    let mut kernel = None;
    for pair in 0..2 {
        audit.phase = format!("technical-pair-{pair}");
        // Box fixes the address used by the one-shot counterpart capability.
        let mut actual = Box::new(
            native_plan_preview::copy_static_snapshot(&base).map_err(|e| format!("{e:?}"))?,
        );
        let mut actual_memory = base_memory.clone();
        let mut copy =
            StaticPreviewBranch::capture(&actual, &actual_memory).map_err(|e| format!("{e:?}"))?;
        let predicted = RefCell::new(
            NativeParityTrace::begin(copy.simulation(), copy.memory())
                .map_err(|e| format!("{e:?}"))?,
        );
        let mut real_trace =
            NativeParityTrace::begin(&actual, &actual_memory).map_err(|e| format!("{e:?}"))?;
        let before = budget.completed();
        let copied = catch_failure(|| {
            copy.verify_tick_with_post_tick(
                pair,
                budget,
                |s, _, memory| {
                    let observation = native_observation::observe(s, 0, &[])?;
                    let sample = memory.observe(&observation)?;
                    predicted
                        .borrow_mut()
                        .record(s, memory)
                        .map_err(|e| format!("{e:?}"))?;
                    line(
                        trace,
                        &native_frame(s, &observation, &sample, &[], "transfer-verify-copy"),
                    )
                },
                |s, memory| {
                    predicted
                        .borrow_mut()
                        .record_post_tick(s, memory)
                        .map_err(|e| format!("{e:?}"))
                },
            )
        });
        audit.technical_parity_native_steps += budget.completed() - before;
        budget.finish_branch()?;
        if let Some(error) = copied?.refusal {
            return Err(format!("Technical copy refused: {error:?}"));
        }
        let predicted = predicted.into_inner();
        compare_prefix(&predicted, &real_trace).map_err(|e| format!("{e:?}"))?;
        let receipt = copy
            .certify_technical_transfer_tick(&actual)
            .map_err(|e| format!("{e:?}"))?;
        let mut owner = actual
            .coordination
            .take()
            .ok_or("Missing technical owner")?;
        let admitted = owner.admit_technical_transfer_tick(&actual, receipt);
        actual.coordination = Some(owner);
        admitted?;
        let before = budget.completed();
        let result = actual_tick(
            &mut actual,
            budget,
            &mut actual_memory,
            trace,
            "transfer-verify-actual",
            Some(&mut real_trace),
            Some(&predicted),
            false,
        );
        audit.technical_parity_native_steps += budget.completed() - before;
        budget.finish_branch()?;
        result?;
        let current = native_kernel_admission_complete(&predicted, &real_trace)
            .map_err(|e| format!("{e:?}"))?;
        if let Some(ref previous) = kernel {
            exact_json(
                &serde_json::to_value(previous).map_err(|e| e.to_string())?,
                &serde_json::to_value(&current).map_err(|e| e.to_string())?,
                "kernelScope",
            )?;
        }
        audit
            .technical_pairs
            .push(json!({"pair":pair,"predicted":predicted,"actual":real_trace,"passed":true}));
        kernel = Some(current);
    }
    if budget.completed() != TECHNICAL
        || audit.technical_prefix_native_steps != PREFIX
        || audit.technical_parity_native_steps != 16
    {
        return Err("Technical52-step accounting mismatch".into());
    }
    audit.technical_parity_passed = true;
    kernel.ok_or("Missing newly measured kernel capability".into())
}

fn terminal_complete(sim: &Simulation) -> Result<bool, String> {
    native_plan_preview::require_transfer_native_state(sim)?;
    sim.coordination
        .as_ref()
        .and_then(|o| o.transfer.as_ref())
        .ok_or("Missing transfer monitor")?
        .terminal_monitoring_complete(sim.substeps_completed, sim.counters.recoveries == 1)
}

fn observe_local_endpoint(sim: &Simulation, audit: &mut Audit) -> Result<(), String> {
    let mut adapter = sim
        .coordination
        .as_ref()
        .and_then(|o| o.transfer.as_ref())
        .ok_or("Missing transfer policy")?
        .clone();
    // Clone-only endpoint inspection leaves the next before-step identity intact.
    let observed = native_observation::observe(sim, 0, &[])?;
    adapter.observe(&observed)?;
    if let Some(ready) = adapter.policy().continuation_event() {
        if ready > LOCAL_DEADLINE {
            return Err("Local ready event arrived after516".into());
        }
        audit.local_success = true;
        audit.first_local_ready_substep.get_or_insert(ready);
    }
    if sim.substeps_completed >= LOCAL_DEADLINE && !audit.local_success {
        return Err("Local milestone was not complete by inclusive native step516".into());
    }
    Ok(())
}

fn history(
    inputs: &PreparedInputs,
    kernel: &NativeKernelAdmission,
    budget: &mut LocalBudget,
    audit: &mut Audit,
    trace: &mut impl Write,
) -> Result<(), String> {
    audit.phase = "main-prefix".into();
    let (mut actual, mut memory) = replay_prefix(inputs, budget, audit, trace, false)?;
    attach(&mut actual, &inputs.entry)?;
    loop {
        let start = actual.substeps_completed;
        audit.reserve_forecast(start)?;
        audit.phase = format!("forecast-{}", audit.forecast_calls);
        let mut copy =
            StaticPreviewBranch::capture(&actual, &memory).map_err(|e| format!("{e:?}"))?;
        let before = budget.completed();
        let preview = catch_failure(|| {
            copy.preview(
                kernel,
                audit.forecast_calls,
                HORIZON,
                budget,
                |s, _, memory| {
                    let observed = native_observation::observe(s, 0, &[])?;
                    let sample = memory.observe(&observed)?;
                    let owner = s.coordination.as_ref().ok_or("Missing copied owner")?;
                    let evidence = owner.evidence();
                    line(
                        trace,
                        &json!({"kind":"transfer-preview-step","forecast":audit.forecast_calls,
                "nativeSubstep":s.substeps_completed,"installedCommand":evidence["last"],
                "numericAttempt":evidence["numericAttempt"],"localDiagnostic":sample,
                "centerOfMassM":observed.center_of_mass_m,
                "support":observed,
                "transferProposal":evidence["transfer"]["decision"]["lastProposal"]}),
                    )
                },
            )
        });
        audit.forecast_actual_steps += budget.completed() - before;
        budget.finish_branch()?;
        // Full endpoint persists even when the copied branch refuses.
        let endpoint_observed = native_observation::observe(copy.simulation(), 0, &[]);
        if let Ok(observed) = endpoint_observed {
            if let Some(sample) = &copy.memory().sample {
                line(
                    trace,
                    &native_frame(
                        copy.simulation(),
                        &observed,
                        sample,
                        &[],
                        "preview-transfer-end",
                    ),
                )?;
            }
        }
        let summary = copy.transfer_horizon_evidence();
        audit.forecast_summaries.push(summary.clone());
        line(
            trace,
            &json!({"event":"forecast-end","forecast":audit.forecast_calls,"horizon":summary}),
        )?;
        if let Some(error) = preview?.refusal {
            return Err(format!("Fixed forecast refused: {error:?}"));
        }
        let terminal_tail = terminal_complete(copy.simulation())?;
        let certified_end = copy.simulation().substeps_completed;
        let receipt = copy
            .certify_transfer_horizon(kernel, start, HORIZON)
            .map_err(|e| format!("{e:?}"))?;
        let mut owner = actual.coordination.take().ok_or("Missing actual owner")?;
        let admitted = owner.admit_transfer_window(&actual, receipt);
        actual.coordination = Some(owner);
        admitted?;
        // Normal receding horizon:24 actual steps. A certified terminal tail
        // consumes this SAME complete receipt in checked windows, with no retry.
        let real_end = if terminal_tail {
            certified_end
        } else {
            start + REAL_WINDOW
        };
        let mut window_start = start;
        while actual.substeps_completed < real_end {
            audit.phase = "actual-certified-window".into();
            audit.reserve_real_tick(actual.substeps_completed)?;
            if actual.substeps_completed == window_start + REAL_WINDOW {
                let mut owner = actual
                    .coordination
                    .take()
                    .ok_or("Missing checkpoint owner")?;
                let continued = owner.continue_transfer_window(&actual);
                actual.coordination = Some(owner);
                continued?;
                window_start = actual.substeps_completed;
            }
            let before = budget.completed();
            let result = actual_tick(
                &mut actual,
                budget,
                &mut memory,
                trace,
                "transfer-actual-history",
                None,
                None,
                false,
            );
            audit.real_history_native_steps += budget.completed() - before;
            budget.finish_branch()?;
            result?;
            observe_local_endpoint(&actual, audit)?;
            if terminal_complete(&actual)? {
                if actual.substeps_completed != certified_end || !terminal_tail {
                    return Err(
                        "Actual terminal endpoint differs from the certified full horizon".into(),
                    );
                }
                // No later command exists to compare this final integration.
                // Certify its outer completed-tick state before reporting success.
                let owner = actual
                    .coordination
                    .as_ref()
                    .ok_or("Missing terminal owner")?;
                let identity = native_plan_preview::transfer_identity(&actual, owner)?;
                owner
                    .transfer
                    .as_ref()
                    .ok_or("Missing terminal monitor")?
                    .require_window_checkpoint(actual.substeps_completed, &identity)?;
                audit.original_terminal_passed = true;
                audit.history_completed = true;
                audit.phase = "closed-terminal".into();
                return Ok(());
            }
        }
        if terminal_tail {
            return Err("Certified terminal tail did not complete original stability".into());
        }
    }
}

fn execute(request: &Value, stdout: &mut impl Write) -> Result<(), String> {
    if request["op"] != "transfer-run" {
        return Err("Only transfer-run is supported".into());
    }
    let operation = required_string(request, "operationId")?;
    if operation.is_empty() {
        return Err("Missing durable operation identity".into());
    }
    let output = required_string(request, "outputPath")?;
    let trace_path = required_string(request, "tracePath")?;
    let mut worker = Worker::new();
    let inputs = prepare_inputs(&mut worker, &request["inputs"])?;
    let (mut technical_budget, mut history_budget) = accept_both(&mut worker, request)?;
    let mut report = new_file(output)?;
    let mut trace = new_file(trace_path)?;
    let mut audit = Audit::default();
    let result = catch_failure(|| {
        let kernel = {
            let _scope = technical_budget.activate(&format!("{output}.technical"))?;
            technical(&inputs, &mut technical_budget, &mut audit, &mut trace)?
        };
        let _scope = history_budget.activate(&format!("{output}.history"))?;
        history(
            &inputs,
            &kernel,
            &mut history_budget,
            &mut audit,
            &mut trace,
        )
    })
    .and_then(|r| r);
    let technical_actual = if technical_budget.gate.is_none() {
        json!(0)
    } else {
        technical_budget.actual_json(technical_budget.completed())
    };
    let history_actual = if history_budget.gate.is_none() {
        json!(0)
    } else {
        history_budget.actual_json(history_budget.completed())
    };
    let exact = technical_actual.is_u64() && history_actual.is_u64();
    let actual = if exact {
        json!(technical_budget.completed() + history_budget.completed())
    } else {
        Value::Null
    };
    let reconciled = technical_budget.completed()
        == audit.technical_prefix_native_steps + audit.technical_parity_native_steps
        && history_budget.completed()
            == audit.real_history_native_steps + audit.forecast_actual_steps;
    let outcome = json!({"integrity_ok":result.is_ok() && exact && reconciled,
        "technical_parity_passed":audit.technical_parity_passed,
        "history_completed":audit.history_completed,"local_success":audit.local_success,
        "original_terminal_passed":audit.original_terminal_passed});
    let evidence = json!({"schema":1,"event":"result","op":"transfer-run","pid":std::process::id(),
        "operationId":operation,"technicalReservationId":technical_budget.permit.reservation_id,
        "historyReservationId":history_budget.permit.reservation_id,
        "reservedNativeSteps":TOTAL,"technicalReservedNativeSteps":TECHNICAL,"historyReservedNativeSteps":HISTORY,
        "technicalActualNativeSteps":technical_actual,"historyActualNativeSteps":history_actual,"actualNativeSteps":actual,
        "outcome":outcome,"evidencePath":output,"refusal":result.err(),"countsReconciled":reconciled,
        "audit":audit,"technicalAccounting":technical_budget.accounting(),"historyAccounting":history_budget.accounting(),
        "prefixExclusions":["kind label only","owner.transfer absent or null only"],
        "noRefund":true,"automaticRenewalAllowed":false});
    line(&mut report, &evidence)?;
    trace.flush().map_err(|e| e.to_string())?;
    trace.get_ref().sync_data().map_err(|e| e.to_string())?;
    report.flush().map_err(|e| e.to_string())?;
    report.get_ref().sync_data().map_err(|e| e.to_string())?;
    // The potentially large native parity frames remain in evidence on disk.
    let mut response = evidence;
    response.as_object_mut().unwrap().remove("audit");
    send(stdout, &response)
}

pub(super) fn run() -> Result<(), String> {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut reader = stdin.lock();
    let mut writer = stdout.lock();
    send(
        &mut writer,
        &json!({"event":"ready","pid":std::process::id(),"protocol":"transfer-run-v1"}),
    )?;
    let request = read(&mut reader)?.ok_or("Missing sole transfer request")?;
    // Exactly one operation. Remain alive for the supervisor's final PID/build
    // audit until it closes stdin; any second request refuses before execution.
    execute(&request, &mut writer)?;
    require_parent_eof(&mut reader)
}

fn require_parent_eof(reader: &mut impl BufRead) -> Result<(), String> {
    if !reader.fill_buf().map_err(|e| e.to_string())?.is_empty() {
        return Err("Transfer worker accepts exactly one operation".into());
    }
    Ok(())
}

#[cfg(test)]
mod pure_tests {
    // Scalar/JSON checks only. These tests construct no model or native world.
    use super::*;
    fn permits() -> Value {
        let deadline = unix_now().unwrap() + 60.;
        let p = |id, steps| {
            json!({"reservationId":id,"reservedSteps":steps,"deadlineUnixS":deadline,
            "remainingWallSeconds":60.,"workerPid":std::process::id()})
        };
        json!({"technicalPermit":p("technical",TECHNICAL),"historyPermit":p("history",HISTORY)})
    }
    #[test]
    fn pure_transfer_budget_formula_and_dual_permits() {
        assert_eq!((TECHNICAL, HISTORY, TOTAL), (52, 131044, 131096));
        let mut w = Worker::new();
        let (a, b) = accept_both(&mut w, &permits()).unwrap();
        assert_eq!(a.deadline, b.deadline);
        assert_eq!(w.reserved_total, TOTAL);
        assert!(accept_both(&mut w, &permits()).is_err());
    }
    #[test]
    fn pure_transfer_wrong_ids_deadlines_pid_or_size_refuse() {
        for field in [
            "reservationId",
            "deadlineUnixS",
            "workerPid",
            "reservedSteps",
        ] {
            let mut v = permits();
            v["historyPermit"][field] = match field {
                "reservationId" => v["technicalPermit"][field].clone(),
                "deadlineUnixS" => json!(unix_now().unwrap() + 120.),
                "workerPid" => json!(0),
                _ => json!(131045),
            };
            assert!(accept_both(&mut Worker::new(), &v).is_err(), "{field}");
        }
    }
    #[test]
    fn pure_transfer_forecast_cadence_and_nonrenewable_cap() {
        let mut a = Audit::default();
        assert!(a.reserve_forecast(PREFIX).is_err());
        a.technical_parity_passed = true;
        // The reserved forecast cap is conservative: the independent global
        // real clock ends before the 260th forecast could start.
        let possible_starts = (REAL_CAP - 1 - PREFIX) / REAL_WINDOW + 1;
        assert_eq!(possible_starts, 259);
        for i in 0..possible_starts {
            a.reserve_forecast(PREFIX + i * REAL_WINDOW).unwrap();
        }
        assert_eq!(a.forecast_requested_steps, 124320);
        assert!(
            a.reserve_forecast(PREFIX + possible_starts * REAL_WINDOW)
                .is_err()
        );
        assert_eq!(FORECAST_CAP * u64::from(HORIZON), 124800);
        let mut exhausted = Audit {
            technical_parity_passed: true,
            forecast_calls: FORECAST_CAP,
            ..Default::default()
        };
        assert!(exhausted.reserve_forecast(PREFIX).is_err());
        let mut a = Audit {
            technical_parity_passed: true,
            ..Default::default()
        };
        a.reserve_forecast(PREFIX).unwrap();
        for at in [PREFIX, PREFIX + 4, PREFIX + 28] {
            assert!(a.reserve_forecast(at).is_err());
        }
    }
    #[test]
    fn pure_transfer_global_real_cap_includes_prefix() {
        let a = Audit::default();
        assert!(a.reserve_real_tick(6240).is_ok());
        assert!(a.reserve_real_tick(6244).is_err());
        assert!(a.reserve_real_tick(37).is_err());
        assert_eq!(LOCAL_DEADLINE, 516);
    }
    #[test]
    fn pure_transfer_prefix_preserves_signed_zero_and_unknown_fields() {
        let old = json!({"kind":"actual-history","owner":{"q":[0.0]},"impulse":-0.0});
        let mut new = old.clone();
        new["kind"] = json!("technical-prefix");
        new["owner"]["transfer"] = Value::Null;
        compare_archived(&old, &new).unwrap();
        new["impulse"] = json!(0.0);
        assert!(compare_archived(&old, &new).is_err());
        new = old.clone();
        new["owner"]["futureDecisionMemory"] = json!(1);
        assert!(compare_archived(&old, &new).is_err());
        new = old.clone();
        new["owner"]["transfer"] = json!({"active":true});
        assert!(compare_archived(&old, &new).is_err());
    }
    #[test]
    fn pure_transfer_inputs_archive_shapes_and_binding() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
        let proposal = root.join("docs/evidence/coordination-transfer-native-proposal-2026-10-13");
        let input = json!({"programPath":root.join("docs/evidence/coordination-prototype-2026-10-12/continuation-candidate-program.json"),
            "tasksPath":root.join("docs/evidence/coordination-prototype-2026-10-12/continuation-tasks.json"),
            "entryPath":root.join("docs/evidence/coordination-transfer-candidate-2026-10-13/observed-entry-step36.json"),
            "prefixFramesPath":proposal.join("worker-prefix-frames.json")});
        let prepared = prepare_inputs(&mut Worker::new(), &input).unwrap();
        assert_eq!(prepared.prefix.len(), 36);
        assert_eq!(prepared.entry.native_substep, 36);
        assert_eq!(prepared.prefix[35]["snapshot"]["stamp"]["tick"], 8);
    }
    #[test]
    fn pure_transfer_waits_for_eof_but_refuses_any_second_request() {
        assert!(require_parent_eof(&mut std::io::Cursor::new(b"")).is_ok());
        for bytes in [b"{\"op\":\"transfer-run\"}\n".as_slice(), b"\n", b"invalid"] {
            assert!(require_parent_eof(&mut std::io::Cursor::new(bytes)).is_err());
        }
    }
}
