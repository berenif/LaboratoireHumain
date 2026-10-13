//! One-shot 44-substep numeric parity experiment. Never invoked by a test.
//! The separate parent ledger reserves all 44 steps before this process runs.
//! This worker may only replay twelve archived steps and four independent
//! four-step actual/copy pairs. It cannot continue a history or refill a permit.
use super::native_plan_preview::{
    NativeParityTrace, PreviewBudget, PreviewPurpose, StaticPreviewBranch, StepReservation,
    copy_static_snapshot, native_kernel_admission,
};
use super::{
    contact_observation as journal, coordinated_runtime, native_observation, support_state,
    whole_body_controller::{self, ReferenceTasks},
};
use crate::{Simulation, measurement, recovery::trajectory_program::Program};
use rapier3d::prelude::MotorModel;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    cell::RefCell,
    fs::{File, OpenOptions},
    io::{BufRead, BufWriter, Read, Write},
    path::Path,
    rc::Rc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

type Coordinates = [[f32; 3]; 25];
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Permit {
    reserved_steps: u64,
    deadline_unix_s: f64,
    remaining_wall_seconds: f64,
    worker_pid: u32,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Inputs {
    fixture_path: String,
    program_path: String,
    tasks_path: String,
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct ExpectedCommand {
    native_substep: u64,
    positions: Coordinates,
    velocities: Coordinates,
}
#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Case {
    feedback_scale: f64,
    iterations: u64,
    commands: Vec<ExpectedCommand>,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Fixture {
    schema: u32,
    prefix_frames: Vec<Value>,
    prefix_body_and_motor_bits: Vec<Vec<u32>>,
    cases: Vec<Case>,
    source_sha256: Value,
}

struct Gate {
    permit: Permit,
    deadline: Instant,
    reserved: u64,
    started: u64,
    pending: u32,
    next_clock: u64,
    phase: String,
    oracle: Option<Case>,
    expected_installations: Option<Vec<Value>>,
    previous: Coordinates,
    installed: Vec<Value>,
}
impl Gate {
    fn new(permit: Permit) -> Result<Self, String> {
        if permit.reserved_steps != 44
            || permit.worker_pid != std::process::id()
            || !permit.deadline_unix_s.is_finite()
            || !permit.remaining_wall_seconds.is_finite()
            || permit.remaining_wall_seconds <= 0.
            || permit.remaining_wall_seconds > 120.
        {
            return Err("Invalid one-shot 44-step / 120-second permit".into());
        }
        let remaining = permit
            .remaining_wall_seconds
            .min(permit.deadline_unix_s - unix_now()?);
        if remaining <= 0. {
            return Err("Expired permit".into());
        }
        Ok(Self {
            permit,
            deadline: Instant::now() + Duration::from_secs_f64(remaining),
            reserved: 0,
            started: 0,
            pending: 0,
            next_clock: 0,
            phase: "prefix".into(),
            oracle: None,
            expected_installations: None,
            previous: [[0.; 3]; 25],
            installed: Vec::new(),
        })
    }
    fn check(&self) -> Result<(), String> {
        if Instant::now() >= self.deadline || unix_now()? >= self.permit.deadline_unix_s {
            Err("Original one-shot wall deadline exhausted".into())
        } else {
            Ok(())
        }
    }
}
#[derive(Clone)]
struct Budget(Rc<RefCell<Gate>>);
impl PreviewBudget for Budget {
    fn admit_preview(&mut self, purpose: PreviewPurpose, steps: u32) -> Result<(), String> {
        self.check_wall_deadline()?;
        if purpose != PreviewPurpose::ParityVerification || steps != 4 {
            return Err("Only a four-step numeric verification is permitted".into());
        }
        Ok(())
    }
    fn reserve_one_before_step(&mut self, _: StepReservation) -> Result<(), String> {
        Err("Only complete original ticks may be reserved".into())
    }
    fn reserve_four_before_tick(&mut self, r: StepReservation) -> Result<(), String> {
        let mut g = self.0.borrow_mut();
        g.check()?;
        if r.purpose != PreviewPurpose::ParityVerification || g.pending != 0 || g.reserved + 4 > 44
        {
            return Err("Incomplete preceding tick or exhausted 44-step reservation".into());
        }
        g.reserved += 4;
        g.pending = 4;
        g.next_clock = r.before_substep;
        Ok(())
    }
    fn check_wall_deadline(&mut self) -> Result<(), String> {
        self.0.borrow().check()
    }
}

thread_local! { static ACTIVE: RefCell<Option<Rc<RefCell<Gate>>>> = const { RefCell::new(None) }; }
struct ActiveScope;
impl ActiveScope {
    fn enter(gate: Rc<RefCell<Gate>>) -> Result<Self, String> {
        ACTIVE.with(|slot| {
            let mut slot = slot.borrow_mut();
            if slot.is_some() {
                return Err("Nested numeric permit refused".into());
            }
            *slot = Some(gate);
            Ok(Self)
        })
    }
}
impl Drop for ActiveScope {
    fn drop(&mut self) {
        ACTIVE.with(|v| *v.borrow_mut() = None);
    }
}

/// Called by the original kernel after configuration and immediately before
/// Rapier. The hook is compiled only into this dedicated worker feature.
/// Failed oracle checks spend no integration; the configured failed state is
/// still archived by the caller. No motor/body/controller is changed here.
pub(crate) fn before_integration(s: &Simulation) -> Result<(), String> {
    ACTIVE.with(|slot| {
        let gate = slot
            .borrow()
            .clone()
            .ok_or("No active numeric native permit")?;
        let mut g = gate.borrow_mut();
        g.check()?;
        if g.pending == 0 || g.started >= 44 || s.substeps_completed != g.next_clock {
            return Err("Native integration lacks its exact reserved clock".into());
        }
        let observed = installed_evidence(s);
        // Preserve the attempted installation even if the distinct oracle fails.
        let phase = g.phase.clone();
        g.installed
            .push(json!({"phase":phase,"nativeSubstep":s.substeps_completed,
            "installed":observed}));
        if let Some(expected) = &g.expected_installations {
            let i = s
                .substeps_completed
                .checked_sub(12)
                .ok_or("Invalid actual installation clock")? as usize;
            compare_json(
                &g.installed.last().unwrap()["installed"],
                expected.get(i).ok_or("Missing predicted installation")?,
                "before-integration-actual-copy",
            )?;
        }
        if let Some(case) = &g.oracle {
            verify_command(s, case, &g.previous)?;
        } else if s.substeps_completed >= 12 {
            return Err("Unverified prefix cannot integrate step thirteen".into());
        }
        g.previous = s
            .coordination
            .as_ref()
            .ok_or("Missing explicit owner")?
            .memory
            .previous;
        g.check()?;
        g.pending -= 1;
        g.started += 1;
        g.next_clock += 1;
        Ok(())
    })
}

fn coordinates_bits(q: &[[f32; 3]]) -> Vec<[u32; 3]> {
    q.iter().map(|v| v.map(f32::to_bits)).collect()
}
fn equal_coordinates(a: &[[f32; 3]], b: &[[f32; 3]]) -> bool {
    coordinates_bits(a) == coordinates_bits(b)
}
fn verify_command(s: &Simulation, case: &Case, previous: &Coordinates) -> Result<(), String> {
    let i = s
        .substeps_completed
        .checked_sub(12)
        .ok_or("Command clock before QP")? as usize;
    let expected = case
        .commands
        .get(i)
        .ok_or("Command clock past frozen four-step oracle")?;
    if expected.native_substep != s.substeps_completed {
        return Err("Oracle command clock differs".into());
    }
    let owner = s.coordination.as_ref().ok_or("Missing coordinated owner")?;
    let evidence = owner.evidence();
    let positions: Coordinates =
        serde_json::from_value(evidence["last"]["positions"].clone()).map_err(|e| e.to_string())?;
    let velocities: Coordinates = serde_json::from_value(evidence["last"]["velocities"].clone())
        .map_err(|e| e.to_string())?;
    if !equal_coordinates(&positions, &expected.positions)
        || !equal_coordinates(&velocities, &expected.velocities)
        || !equal_coordinates(&s.targets, &expected.positions)
        || !equal_coordinates(&owner.memory.previous, &expected.positions)
        || owner.memory.next_substep != s.substeps_completed + 1
    {
        return Err(format!(
            "Compiled command differs bitwise from frozen oracle at {}",
            s.substeps_completed
        ));
    }
    let recovery = s
        .recovery
        .as_ref()
        .ok_or("Missing original recovery memory")?;
    if !equal_coordinates(&recovery.targets, &expected.positions)
        || !equal_coordinates(&recovery.velocities, &expected.velocities)
    {
        return Err("Recovery installation memory differs from compiled command".into());
    }
    let gain_scale = s
        .profile
        .motor_scale
        .min(s.profile.aggregate_motor_cap_nm / s.motor_ceiling_sum);
    let mut cap_sum = 0_f64;
    for a in coordinated_runtime::axes(&s.model)? {
        let q = expected.positions[a.segment][a.coordinate];
        let dq = expected.velocities[a.segment][a.coordinate];
        if ((q - previous[a.segment][a.coordinate]) / s.profile.dt_s).to_bits() != dq.to_bits() {
            return Err(format!(
                "Oracle dq is not the exact f32 finite difference at {i}/{}",
                a.segment
            ));
        }
        let joint = s
            .world
            .impulse_joints
            .get(s.joints[a.segment].ok_or("Missing joint handle")?)
            .ok_or("Missing motor joint")?;
        let motor = &joint.data.motors[3 + a.coordinate];
        let axis = s.model.segments[a.segment]
            .joint_profile
            .as_ref()
            .ok_or("Missing canonical axis profile")?
            .axes
            .iter()
            .find(|v| v.coordinate == ['x', 'y', 'z'][a.coordinate])
            .ok_or("Missing canonical anatomical axis")?;
        let stiffness =
            axis.passive_stiffness_nm_per_rad * s.profile.posture_stiffness_multiplier * gain_scale;
        let damping = axis.damping_nms_per_rad * s.profile.posture_damping_multiplier * gain_scale;
        let anatomical_cap = axis.max_motor_torque_nm * s.profile.motor_scale.min(1.);
        if motor.target_pos.to_bits() != q.to_bits()
            || motor.target_vel.to_bits() != dq.to_bits()
            || motor.model != MotorModel::ForceBased
            || !joint.data.enforce_motor_vector_limit
            || !motor.max_force.is_finite()
            || motor.max_force < 0.
            || !motor.stiffness.is_finite()
            || motor.stiffness < 0.
            || !motor.damping.is_finite()
            || motor.damping < 0.
            || motor.stiffness.to_bits() != stiffness.to_bits()
            || motor.damping.to_bits() != damping.to_bits()
            || motor.max_force > anatomical_cap
            || joint.data.motor_axes.bits() & (1 << (3 + a.coordinate)) == 0
        {
            return Err(format!(
                "Actually installed motor differs at segment {} axis {}",
                a.segment, a.coordinate
            ));
        }
        cap_sum += f64::from(motor.max_force);
    }
    if cap_sum > f64::from(s.profile.aggregate_motor_cap_nm) {
        return Err(format!(
            "Actually installed aggregate cap {cap_sum} exceeds original cap"
        ));
    }
    if i == 0
        && (evidence["numericAttempt"]["status"] != "certified before installation"
            || evidence["numericAttempt"]["compilation"]["stage"]
                != "certified guarded exact f32 commands"
            || evidence["last"]["solver"]["iterations"].as_u64() != Some(case.iterations)
            || evidence["patches"]["active"]
                .as_array()
                .is_none_or(Vec::is_empty))
    {
        return Err(
            "Frozen constrained solve, active patches, or expected iteration count absent".into(),
        );
    }
    Ok(())
}

fn installed_evidence(s: &Simulation) -> Value {
    let joints: Vec<_> = s
        .joints
        .iter()
        .enumerate()
        .filter_map(|(segment, h)| {
            let j = s.world.impulse_joints.get((*h)?)?;
            Some(json!({"segment":segment,"data":j.data}))
        })
        .collect();
    json!({"joints":joints,"bodyAndMotorBits":body_motor_bits(s),
        "owner":s.coordination.as_ref().map(|o|o.evidence()),"memory":memory(s),
        "targetsBits":coordinates_bits(&s.targets)})
}
fn body_motor_bits(s: &Simulation) -> Vec<u32> {
    let mut bits = Vec::with_capacity(2341);
    for h in &s.bodies {
        let b = &s.world.bodies[*h];
        bits.extend(b.translation().to_array().map(f32::to_bits));
        bits.extend(b.rotation().to_array().map(f32::to_bits));
        bits.extend(b.linvel().to_array().map(f32::to_bits));
        bits.extend(b.angvel().to_array().map(f32::to_bits));
    }
    for h in s.joints.iter().flatten() {
        let j = s
            .world
            .impulse_joints
            .get(*h)
            .expect("existing native joints");
        for m in &j.data.motors {
            bits.extend(
                [
                    m.target_pos,
                    m.target_vel,
                    m.stiffness,
                    m.damping,
                    m.max_force,
                    m.impulse,
                    m.delivered_abs_impulse,
                    m.delivered_impulse,
                ]
                .map(f32::to_bits),
            );
            bits.extend(m.delivered_angular_impulse_1.to_array().map(f32::to_bits));
            bits.extend(m.delivered_angular_impulse_2.to_array().map(f32::to_bits));
        }
    }
    bits
}
fn memory(s: &Simulation) -> Value {
    let recovery = s.recovery.as_ref().map(|r| {
        // Exhaustive destructuring makes added persistent fields a build error.
        let crate::recovery::Controller { status, targets, hands, lead, stand_feet, stand_root,
            stand_goal, best_progress, velocities, foot_anchors, hand_anchors, knee_anchors,
            handoff_from, stable_substeps, handoff_substeps, settle_substeps } = r;
        let anchors = |values: &[Option<crate::recovery::PlantedAnchor>; 2]| values.map(|a| a.map(|a|
            json!({"segment":a.segment,"localBits":a.local_point.to_array().map(f32::to_bits),
                "worldBits":a.world_point.to_array().map(f32::to_bits)})));
        json!({"status":status,"targetsBits":coordinates_bits(targets),
            "velocityBits":coordinates_bits(velocities),"handsBits":hands.map(|v|v.to_array().map(f32::to_bits)),
            "lead":lead,"standFeetBits":stand_feet.map(|v|v.map(|x|x.to_array().map(f32::to_bits))),
            "standRoot":stand_root,"standGoalBits":stand_goal.map(|q|q.to_array().map(f32::to_bits)),
            "bestProgressBits":best_progress.to_bits(),"footAnchors":anchors(foot_anchors),
            "handAnchors":anchors(hand_anchors),"kneeAnchors":anchors(knee_anchors),
            "handoffFromBits":handoff_from.as_ref().map(|q|coordinates_bits(q)),
            "stableSubsteps":stable_substeps,"handoffSubsteps":handoff_substeps,"settleSubsteps":settle_substeps})
    });
    json!({"recovery":recovery,"modeSignature":s.coordination.as_ref().map(|o|o.memory.mode_signature)})
}
fn common_state(s: &Simulation) -> Value {
    json!({"world":s.world,"snapshot":s.snapshot(),"nativeSubstep":s.substeps_completed,
        "tick":s.tick,"owner":s.coordination.as_ref().map(|o|o.evidence()),"memory":memory(s),
        "bodyAndMotorBits":body_motor_bits(s),"targetsBits":coordinates_bits(&s.targets),
        "restTargetsBits":coordinates_bits(&s.rest_targets),"persistence":s.persistence,
        "contacts":s.contacts,"integratedSupportSubsteps":s.integrated_support_substeps,
        "integratedSupportLoadsN":s.integrated_support_loads_n,
        "firstFailure":s.first_failure})
}
fn native_frame(
    s: &Simulation,
    observation: &support_state::SupportObservation,
    diagnostic: &support_state::LocalSupportSample,
    ccd: &[journal::SolveRecord],
    kind: &str,
    collected: bool,
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
        "owner":s.coordination.as_ref().map(|o|o.evidence()),"support":observation,
        "targets":s.targets,"restTargets":s.rest_targets,"motors":motors,
        "persistence":s.persistence,"integratedSupportSubsteps":s.integrated_support_substeps,
        "integratedSupportLoadsN":s.integrated_support_loads_n,"localDiagnostic":diagnostic,
        "ccd":ccd,"ccdJournalCollected":collected,"angularBalanceCertified":false})
}

/// JSON numeric comparison deliberately distinguishes signed zero. Native f32
/// state is additionally compared through separately frozen u32 bit arrays.
fn compare_json(a: &Value, b: &Value, path: &str) -> Result<(), String> {
    match (a, b) {
        (Value::Number(a), Value::Number(b)) => {
            let same = if a.is_f64() || b.is_f64() {
                a.as_f64().map(f64::to_bits) == b.as_f64().map(f64::to_bits)
            } else {
                a == b
            };
            if same {
                Ok(())
            } else {
                Err(format!("Numeric bits differ at {path}: {a} / {b}"))
            }
        }
        (Value::Array(a), Value::Array(b)) if a.len() == b.len() => {
            for (i, (x, y)) in a.iter().zip(b).enumerate() {
                compare_json(x, y, &format!("{path}/{i}"))?;
            }
            Ok(())
        }
        (Value::Object(a), Value::Object(b)) if a.len() == b.len() => {
            for (key, x) in a {
                compare_json(
                    x,
                    b.get(key).ok_or_else(|| format!("Missing {path}/{key}"))?,
                    &format!("{path}/{key}"),
                )?;
            }
            Ok(())
        }
        _ if a == b => Ok(()),
        _ => Err(format!("State differs at {path}")),
    }
}
fn prefix_projection(mut v: Value, new: bool) -> Result<Value, String> {
    let object = v.as_object_mut().ok_or("Invalid prefix frame")?;
    for key in ["kind", "ccd", "ccdJournalCollected"] {
        object.remove(key);
    }
    if new {
        let owner = object
            .get_mut("owner")
            .and_then(Value::as_object_mut)
            .ok_or("Missing prefix owner")?;
        if owner.remove("numericAttempt") != Some(Value::Null) {
            return Err("Unexpected constrained attempt in archived prefix".into());
        }
    }
    object
        .get_mut("support")
        .and_then(Value::as_object_mut)
        .ok_or("Missing prefix support")?
        .remove("ccd_impulses");
    Ok(v)
}
fn check_prefix(s: &Simulation, frame: &Value, fixture: &Fixture) -> Result<(), String> {
    let i = s
        .substeps_completed
        .checked_sub(1)
        .ok_or("Prefix clock zero")? as usize;
    if body_motor_bits(s)
        != *fixture
            .prefix_body_and_motor_bits
            .get(i)
            .ok_or("Prefix bits absent")?
    {
        return Err(format!(
            "Archived native f32 body/motor bits differ at completed step {}",
            i + 1
        ));
    }
    // Apply the same JSON parser boundary to newly emitted and archived rows.
    // Native f32 exactness does not depend on this decimal round trip.
    let emitted: Value =
        serde_json::from_slice(&serde_json::to_vec(frame).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    compare_json(
        &prefix_projection(emitted, true)?,
        &prefix_projection(fixture.prefix_frames[i].clone(), false)?,
        "prefix",
    )
}

#[derive(Default)]
struct Evidence {
    actual: u64,
    prefix: bool,
    cases: Vec<Value>,
    last: Value,
    observed: u64,
}
fn fail(s: &Simulation, substep: u32, reason: String) -> measurement::Failure {
    measurement::Failure::new(s.tick, substep, "numeric-parity-observer", reason, 1., 0.)
}
fn real_tick(
    s: &mut Simulation,
    budget: &mut Budget,
    diagnostic: &mut support_state::LocalSupportDiagnostic,
    trace: &mut impl Write,
    kind: &str,
    fixture: Option<&Fixture>,
    mut parity: Option<&mut NativeParityTrace>,
    expected_states: Option<&[Value]>,
    evidence: &mut Evidence,
) -> Result<(), String> {
    journal::require_inactive()?;
    budget.reserve_four_before_tick(StepReservation {
        branch_id: 0,
        before_substep: s.substeps_completed,
        preview_index: 0,
        purpose: PreviewPurpose::ParityVerification,
    })?;
    let before = s.substeps_completed;
    journal::activate(s);
    let mut pending = Vec::new();
    let result = s.advance_tick_observed(|s, substep| {
        pending = journal::take_records();
        let observed =
            native_observation::observe(s, 0, &pending).map_err(|e| fail(s, substep, e))?;
        let sample = diagnostic
            .observe(&observed)
            .map_err(|e| fail(s, substep, e))?;
        let frame = native_frame(s, &observed, &sample, &pending, kind, true);
        evidence.last = common_state(s);
        evidence.observed += 1;
        line(
            trace,
            &json!({"kind":kind,"frame":frame,"state":evidence.last}),
        )
        .map_err(|e| fail(s, substep, e))?;
        if let Some(expected) = expected_states {
            let expected = expected
                .get(substep as usize)
                .ok_or_else(|| fail(s, substep, "Missing predicted frame".into()))?;
            compare_json(&evidence.last, expected, "after-step-actual-copy")
                .map_err(|e| fail(s, substep, e))?;
        }
        if let Some(f) = fixture {
            check_prefix(s, &frame, f).map_err(|e| fail(s, substep, e))?;
        }
        if let Some(p) = parity.as_deref_mut() {
            p.record(s, &memory(s))
                .map_err(|e| fail(s, substep, format!("{e:?}")))?;
        }
        pending.clear();
        budget
            .check_wall_deadline()
            .map_err(|e| fail(s, substep, e))?;
        Ok(())
    });
    evidence.actual += s.substeps_completed - before;
    pending.extend(journal::take_records());
    journal::finish();
    evidence.last = common_state(s);
    line(
        trace,
        &json!({"kind":"post-tick","source":kind,"state":evidence.last,
        "unobservedCcd":pending,"failure":result.as_ref().err()}),
    )?;
    result.map_err(|e| format!("{e:?}"))
}

fn validate_fixture(f: &Fixture) -> Result<(), String> {
    if f.schema != 1
        || f.prefix_frames.len() != 12
        || f.prefix_body_and_motor_bits.len() != 12
        || f.prefix_body_and_motor_bits.iter().any(|v| v.len() != 2341)
        || f.cases.len() != 4
    {
        return Err("Incomplete frozen prefix/oracle fixture".into());
    }
    for (i, c) in f.cases.iter().enumerate() {
        if c.feedback_scale.to_bits() != [0_f64, 0.5, 1., 1.5][i].to_bits()
            || c.iterations != [8, 8, 12, 12][i]
            || c.commands.len() != 4
            || c.commands.iter().enumerate().any(|(j, q)| {
                q.native_substep != 12 + j as u64
                    || q.positions
                        .iter()
                        .chain(q.velocities.iter())
                        .flatten()
                        .any(|x| !x.is_finite())
            })
        {
            return Err("Frozen four-case command oracle differs from authorization".into());
        }
    }
    Ok(())
}
fn execute(
    input: &Inputs,
    budget: &mut Budget,
    trace: &mut impl Write,
    evidence: &mut Evidence,
) -> Result<Value, String> {
    let fixture: Fixture = read_json(&input.fixture_path)?;
    validate_fixture(&fixture)?;
    let program: Program = read_json(&input.program_path)?;
    let tasks: ReferenceTasks = read_json(&input.tasks_path)?;
    let mut prefix = whole_body_controller::scene(program, tasks, true, 0.)?;
    let mut diagnostic = support_state::LocalSupportDiagnostic::default();
    budget.0.borrow_mut().previous = prefix
        .coordination
        .as_ref()
        .ok_or("Missing prefix owner")?
        .memory
        .previous;
    line(
        trace,
        &json!({"kind":"initial","state":common_state(&prefix)}),
    )?;
    for _ in 0..3 {
        real_tick(
            &mut prefix,
            budget,
            &mut diagnostic,
            trace,
            "prefix",
            Some(&fixture),
            None,
            None,
            evidence,
        )?;
    }
    if prefix.substeps_completed != 12 || evidence.actual != 12 {
        return Err("Shared prefix clock/accounting mismatch".into());
    }
    evidence.prefix = true;
    line(
        trace,
        &json!({"kind":"prefix-certified","completedNativeSteps":12,"beforeAnyStep13":true}),
    )?;
    trace.flush().map_err(|e| e.to_string())?;
    for (i, case) in fixture.cases.iter().enumerate() {
        let mut real = copy_static_snapshot(&prefix).map_err(|e| format!("{e:?}"))?;
        real.coordination
            .as_mut()
            .ok_or("Missing copied owner")?
            .feedback_scale = case.feedback_scale;
        let mut copy = StaticPreviewBranch::capture(&real, &()).map_err(|e| format!("{e:?}"))?;
        let mut predicted = NativeParityTrace::begin(copy.simulation(), &memory(copy.simulation()))
            .map_err(|e| format!("{e:?}"))?;
        let mut actual =
            NativeParityTrace::begin(&real, &memory(&real)).map_err(|e| format!("{e:?}"))?;
        let mut copy_diag = diagnostic.clone();
        let mut real_diag = diagnostic.clone();
        let start_installed = budget.0.borrow().installed.len();
        {
            let mut gate = budget.0.borrow_mut();
            gate.oracle = Some(case.clone());
            gate.expected_installations = None;
            gate.previous = prefix.coordination.as_ref().unwrap().memory.previous;
            gate.phase = format!("case-{i}-copy");
        }
        let before = copy.simulation().substeps_completed;
        let mut predicted_states = Vec::new();
        let run = copy.verify_tick(i as u64+1,budget,|s,_,_| {
            let observed = native_observation::observe(s,0,&[])?;
            let sample = copy_diag.observe(&observed)?;
            predicted.record(s,&memory(s)).map_err(|e|format!("{e:?}"))?;
            evidence.observed += 1;
            evidence.last = common_state(s);
            predicted_states.push(evidence.last.clone());
            line(trace,&json!({"kind":"predicted","case":i,"frame":native_frame(s,&observed,&sample,&[],"predicted",false),"state":evidence.last}))?;
            Ok(())
        });
        evidence.actual += copy.simulation().substeps_completed - before;
        evidence.last = common_state(copy.simulation());
        line(
            trace,
            &json!({"kind":"post-tick","source":"predicted","case":i,"state":evidence.last,
            "refusal":run.refusal.as_ref().map(|e|format!("{e:?}"))}),
        )?;
        if let Some(error) = run.refusal {
            return Err(format!("Case {i} predicted: {error:?}"));
        }
        let final_predicted = evidence.last.clone();
        {
            let mut gate = budget.0.borrow_mut();
            gate.previous = prefix.coordination.as_ref().unwrap().memory.previous;
            gate.phase = format!("case-{i}-real");
            gate.expected_installations = Some(
                gate.installed[start_installed..]
                    .iter()
                    .map(|v| v["installed"].clone())
                    .collect(),
            );
        }
        real_tick(
            &mut real,
            budget,
            &mut real_diag,
            trace,
            "actual",
            None,
            Some(&mut actual),
            Some(&predicted_states),
            evidence,
        )?;
        let admission =
            native_kernel_admission(&predicted, &actual).map_err(|e| format!("Case {i}: {e:?}"))?;
        compare_json(&final_predicted, &evidence.last, "post-tick-final-state")?;
        compare_json(&json!(copy_diag), &json!(real_diag), "diagnostic-memory")?;
        let installed = budget.0.borrow();
        let records = &installed.installed[start_installed..];
        if records.len() != 8 {
            return Err("Missing one of eight before-integration records".into());
        }
        for j in 0..4 {
            compare_json(
                &records[j]["installed"],
                &records[j + 4]["installed"],
                "installed-original-motors-and-memory",
            )?;
        }
        evidence.cases.push(
            json!({"feedbackScale":case.feedback_scale,"expectedIterations":case.iterations,
            "actualNativeSteps":8,"frozenCommandsBitwiseVerifiedPerPath":4,"nativeParity":admission,
            "postTickStateVerified":true,"installedMotorsVerified":true}),
        );
    }
    if evidence.actual != 44 || budget.0.borrow().started != 44 || evidence.observed != 44 {
        return Err("Final 44-step integrated/started/observed accounting differs".into());
    }
    Ok(fixture.source_sha256)
}

fn unix_now() -> Result<f64, String> {
    Ok(SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_secs_f64())
}
fn read_json<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, String> {
    serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}
fn required<'a>(v: &'a Value, key: &str) -> Result<&'a str, String> {
    v[key].as_str().ok_or_else(|| format!("Missing {key}"))
}
fn new_file(path: &str) -> Result<BufWriter<File>, String> {
    if let Some(p) = Path::new(path).parent() {
        std::fs::create_dir_all(p).map_err(|e| e.to_string())?;
    }
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map(BufWriter::new)
        .map_err(|e| e.to_string())
}
fn line(w: &mut impl Write, v: &Value) -> Result<(), String> {
    serde_json::to_writer(&mut *w, v).map_err(|e| e.to_string())?;
    w.write_all(b"\n").map_err(|e| e.to_string())
}
fn send(w: &mut impl Write, v: &Value) -> Result<(), String> {
    line(w, v)?;
    w.flush().map_err(|e| e.to_string())
}

fn run() -> Result<(), String> {
    let stdin = std::io::stdin();
    let mut stdin = stdin.lock();
    let stdout = std::io::stdout();
    let mut stdout = stdout.lock();
    send(
        &mut stdout,
        &json!({"event":"ready","pid":std::process::id()}),
    )?;
    let mut request = String::new();
    if stdin.read_line(&mut request).map_err(|e| e.to_string())? == 0 {
        return Ok(());
    }
    let request: Value = serde_json::from_str(&request).map_err(|e| e.to_string())?;
    if request["op"] != "run" {
        return Err("Only one run command is supported".into());
    }
    let output = required(&request, "outputPath")?;
    let trace_path = required(&request, "tracePath")?;
    let mut report = new_file(output)?;
    let mut trace = new_file(trace_path)?;
    let mut evidence = Evidence::default();
    let mut gate_saved = None;
    let result = (|| {
        let inputs: Inputs =
            serde_json::from_value(request["inputs"].clone()).map_err(|e| e.to_string())?;
        let permit: Permit =
            serde_json::from_value(request["permit"].clone()).map_err(|e| e.to_string())?;
        let gate = Rc::new(RefCell::new(Gate::new(permit)?));
        gate_saved = Some(gate.clone());
        let _scope = ActiveScope::enter(gate.clone())?;
        execute(&inputs, &mut Budget(gate), &mut trace, &mut evidence)
    })();
    let ok = result.is_ok();
    let outcome = json!({"integrity_ok":ok,"prefix_matches_archive":evidence.prefix,
        "parity_verified":ok && evidence.cases.len()==4,"diagnostic_present":evidence.observed>0});
    let budget_evidence=gate_saved.as_ref().map(|g| {
        let g=g.borrow();
        json!({"reservedSteps":44,"locallyReservedSteps":g.reserved,"admittedIntegrations":g.started,
            "pendingReservedSteps":g.pending,"installations":g.installed})
    });
    let document = json!({"schema":1,"op":"numeric-parity","actualNativeSteps":evidence.actual,"outcome":outcome,
        "reason":result.as_ref().err(),"sourceSha256":result.as_ref().ok(),"tracePath":trace_path,
        "cases":evidence.cases,"lastState":evidence.last,"budget":budget_evidence,
        "scope":"Twelve archived prefix states plus four independent four-step native actual/copy comparisons. No recovery or continuation qualification.",
        "ccdScope":"Real CCD journals collected; preview journals inactive. Persistent world/contact states and signed delivered motor impulses compared. CCD workspace omitted by world serde is not certified by serialization.",
        "prefixProjection":"Ignore kind, CCD journal fields, and the newly added null numericAttempt only; native f32 body/motor bits and archived JSON numeric representations compared exactly.",
        "recoveryQualified":false});
    line(&mut report, &document)?;
    trace.flush().map_err(|e| e.to_string())?;
    trace.get_ref().sync_all().map_err(|e| e.to_string())?;
    report.flush().map_err(|e| e.to_string())?;
    report.get_ref().sync_all().map_err(|e| e.to_string())?;
    send(
        &mut stdout,
        &json!({"event":"result","op":"numeric-parity","actualNativeSteps":evidence.actual,
        "outcome":outcome,"evidencePath":output}),
    )?;
    // Keep the single-thread process alive for the parent's final PID/executable
    // check. EOF ends it; no second command can renew or use the closed permit.
    let mut byte = [0u8; 1];
    while stdin.read(&mut byte).map_err(|e| e.to_string())? != 0 {}
    Ok(())
}
pub(crate) fn main() {
    if let Err(error) = run() {
        eprintln!("numeric-parity-worker: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn numeric_archive_comparison_preserves_signed_zero() {
        assert!(compare_json(&json!(-0.0), &json!(0.0), "zero").is_err());
        assert!(compare_json(&json!([1.25, -0.0]), &json!([1.25, -0.0]), "same").is_ok());
    }
    #[test]
    fn command_comparison_preserves_f32_bits() {
        let a = [[0_f32; 3]; 25];
        let mut b = a;
        b[1][2] = -0.;
        assert!(!equal_coordinates(&a, &b));
    }
    #[test]
    fn permit_cannot_expand_fixed_authorization() {
        let p = Permit {
            reserved_steps: 45,
            deadline_unix_s: unix_now().unwrap() + 120.,
            remaining_wall_seconds: 120.,
            worker_pid: std::process::id(),
        };
        assert!(Gate::new(p).is_err());
    }
    #[test]
    fn incomplete_tick_and_expired_permit_cannot_be_reserved_again() {
        let p = Permit {
            reserved_steps: 44,
            deadline_unix_s: unix_now().unwrap() + 120.,
            remaining_wall_seconds: 120.,
            worker_pid: std::process::id(),
        };
        let gate = Rc::new(RefCell::new(Gate::new(p).unwrap()));
        let mut budget = Budget(gate.clone());
        let request = StepReservation {
            branch_id: 0,
            before_substep: 0,
            preview_index: 0,
            purpose: PreviewPurpose::ParityVerification,
        };
        budget.reserve_four_before_tick(request).unwrap();
        assert!(budget.reserve_four_before_tick(request).is_err());
        assert_eq!(gate.borrow().reserved, 4);
        assert_eq!(gate.borrow().started, 0);
        gate.borrow_mut().deadline = Instant::now();
        assert!(budget.check_wall_deadline().is_err());
        assert_eq!(gate.borrow().reserved, 4);
    }
    #[test]
    fn frozen_fixture_shape_and_command_finite_differences_are_pure() {
        let fixture: Fixture = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../docs/evidence/coordination-native-parity-2026-10-13/expected.json"
        )))
        .unwrap();
        validate_fixture(&fixture).unwrap();
        for case in &fixture.cases {
            let mut prior: Coordinates =
                serde_json::from_value(fixture.prefix_frames[11]["owner"]["q"].clone()).unwrap();
            for command in &case.commands {
                for i in 0..25 {
                    for k in 0..3 {
                        assert_eq!(
                            ((command.positions[i][k] - prior[i][k]) / lh_contracts::SUBSTEP_S)
                                .to_bits(),
                            command.velocities[i][k].to_bits(),
                            "scale={} step={} axis={i}/{k}",
                            case.feedback_scale,
                            command.native_substep
                        );
                    }
                }
                prior = command.positions;
            }
        }
    }
}
