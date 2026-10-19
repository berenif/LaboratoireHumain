//! One bounded forecast-only experiment. Nothing here is a real controller,
//! reference generator, handoff, or persistent checkpoint state.
use super::*;

const REQUIRED_NATIVE_SAMPLES: u64 = 80;
const STANDING_KEYS: [&str; 10] = ["head", "torso", "bothFeetLoadedAndPersistent",
    "footLoadFraction", "onlyFeet", "comInSupport", "captureInSupport",
    "linearMotion", "angularMotion", "footDriftFromSuppliedAnchors"];

#[derive(Default)]
struct StandingInterval {
    anchors: Option<[Vec3; 2]>,
    consecutive: u64,
    maximum: u64,
}

impl StandingInterval {
    /// The input is the existing native standing checker, not a weaker
    /// teacher-specific posture test. A new interval includes its first sample.
    fn observe(&mut self, mut current: serde_json::Value) -> Result<(serde_json::Value, bool, Option<[Vec3; 2]>), String> {
        let predicates = current["predicates"].as_object().ok_or("Missing full standing predicates")?;
        if predicates.len() != STANDING_KEYS.len() || !STANDING_KEYS.iter().all(|key|predicates.contains_key(*key)) {
            return Err("Full standing predicate contract differs".into());
        }
        let drift = &predicates[STANDING_KEYS[9]];
        if STANDING_KEYS[..9].iter().any(|key|predicates[*key].as_bool().is_none())
            || !(drift.is_boolean() || drift.is_null())
            || self.anchors.is_some() && predicates[STANDING_KEYS[9]].as_bool().is_none()
        {
            return Err("Invalid full standing predicate value".into());
        }
        let base = STANDING_KEYS[..9].iter().all(|key| predicates[*key] == true);
        let mut used = self.anchors;
        if base && used.is_none() {
            let feet: [Vec3; 2] = serde_json::from_value(current["feet"].clone()).map_err(|e|e.to_string())?;
            used = Some(feet);
            // The same-current-sample anchor distance is exactly zero. All
            // later samples use standing_diagnostic's original 10 mm check.
            current["footDriftM"] = serde_json::json!([0.0, 0.0]);
            current["predicates"]["footDriftFromSuppliedAnchors"] = true.into();
        }
        let passed = base && current["predicates"]["footDriftFromSuppliedAnchors"] == true;
        if passed {
            self.anchors = used;
            self.consecutive += 1;
            self.maximum = self.maximum.max(self.consecutive);
        } else {
            // Never re-anchor on the failing sample itself.
            self.anchors = None;
            self.consecutive = 0;
        }
        Ok((current, passed, used))
    }
}

fn next_support_persistence(previous: u64, load: f32, minimum: f32) -> Result<u64, String> {
    if !load.is_finite() || load < 0.0 || !minimum.is_finite() || minimum <= 0.0 {
        return Err("Invalid teacher support sample".into());
    }
    if load >= minimum { previous.checked_add(1).ok_or("Teacher support clock overflow".into()) }
    else { Ok(0) }
}

fn refresh_current_support(shadow: &mut Simulation) -> Result<(), String> {
    let minimum = recovery_contract::contract().recovery_acceptance.minimum_load_n;
    let mut loads = [0.0; 25];
    let mut persistence = [0; 25];
    for i in 0..shadow.colliders.len() {
        loads[i] = shadow.support_normal_impulse(shadow.colliders[i])? / shadow.profile.dt_s;
        persistence[i] = next_support_persistence(shadow.integrated_support_substeps[i], loads[i], minimum)?;
    }
    shadow.integrated_support_loads_n = loads;
    shadow.integrated_support_substeps = persistence;
    Ok(())
}

fn original_cut_after_frame(finish: &mut bool, cut: &mut Option<usize>, decisions: usize, head: f32) {
    if cut.is_none() {
        if *finish { *cut = Some(decisions); }
        *finish = head > 1.4;
    }
}

pub(crate) fn probe(real: &Simulation) -> Result<serde_json::Value, String> {
    let fixture = real.getup_fixture.as_ref().ok_or("Teacher feasibility requires the original BACK fixture")?;
    let controller = real.getup_controller.as_ref().ok_or("Missing untouched source controller")?;
    if fixture.kind != "back" || fixture.preparing || !fixture.ready
        || real.substeps_completed != 480 || fixture.trigger_substep != Some(480)
        || real.motion != Motion::Fallen || controller.started_substep.is_some()
        || controller.impact_substep != Some(480) || !controller.teacher_frames.is_empty()
        || !controller.reference.is_empty()
    {
        return Err("Teacher feasibility requires fresh triggered BACK at native480 before source control".into());
    }
    if real.profile.getup_solver_budget != Some(GetUpSolverBudget::ConsistentRecoveryPhaseV1) {
        return Err("Teacher feasibility requires the explicit consistent recovery solver budget".into());
    }
    let mut untouched = Controller::new(&real.model);
    // Simulation construction evaluates the original upright command once;
    // fixture preparation then bypasses all controller commands/decisions.
    if cfg!(feature = "getup-terminal-balance-experiment") {
        untouched.status.phase = "experimental-joint-balance-awaiting-fall".into();
    }
    untouched.fixture_trigger(480)?;
    if serde_json::to_value(controller).map_err(|e|e.to_string())?
        != serde_json::to_value(&untouched).map_err(|e|e.to_string())?
    {
        return Err("Teacher feasibility controller differs from untouched construction plus trigger".into());
    }
    let teacher_initial = real.snapshot();
    let mut shadow = fork(real)?;
    if serde_json::to_value(&teacher_initial.segments).map_err(|e|e.to_string())?
        != serde_json::to_value(&shadow.snapshot().segments).map_err(|e|e.to_string())?
        || super::super::joint_state(real)? != super::super::joint_state(&shadow)?
    {
        return Err("Teacher feasibility fork changed the exact seed".into());
    }
    select_forecast_solver_budget(&mut shadow);
    let native_per_policy = policy_substeps(shadow.profile.dt_s)?;
    if native_per_policy != 20 || shadow.world.integration_parameters.dt != shadow.profile.dt_s
        || shadow.world.integration_parameters.num_internal_pgs_iterations != 32
        || shadow.profile.getup_policy_enabled || shadow.getup_controller.is_some()
    {
        return Err("Teacher feasibility requires the original 40Hz/800Hz/32-pass disabled-controller fork".into());
    }
    let mut archive = Controller::new(&shadow.model);
    let mut interval = StandingInterval::default();
    let mut trace = Vec::with_capacity(MAX_TEACHER_TICKS * native_per_policy as usize);
    let mut finish_after_step = false;
    let mut original_cut = None;
    let mut endpoint = None;
    let mut failure = None;
    let mut decisions = 0;
    let mut last_decision_native_count = 0;
    'decisions: for index in 0..MAX_TEACHER_TICKS {
        let physical = super::super::physical_observation(&shadow)?;
        let before = shadow.snapshot().segments;
        let mut input = [0.0; 68];
        input[..67].copy_from_slice(&physical);
        input[67] = TEACHER_POWER;
        let action = actor::teacher().infer(&input)?.action;
        decisions = index + 1;
        for substep in 0..native_per_policy {
            if cfg!(feature = "unit-root-rotation-experiment") || shadow.profile.root_rotation_normalization.is_some() {
                shadow.validate_unit_root_rotation_policy().map_err(|e|format!("Teacher unit root rotation: {e}"))?;
            }
            for h in &shadow.bodies {
                shadow.world.bodies[*h].reset_forces(false);
                shadow.world.bodies[*h].reset_torques(false);
            }
            super::super::apply_controls(&mut shadow, &action)?;
            shadow.world.step_with_events(&shadow.filter, &());
            archive.status.forecast_substeps += 1;
            shadow.substeps_completed += 1;
            last_decision_native_count = substep + 1;
            if let Err(f) = measurement::inspect(&mut shadow, substep) {
                failure = Some(f);
                archive.record_teacher(&shadow, physical.to_vec(), action, before)?;
                break 'decisions;
            }
            // Same post-solve load and persistence semantics as the real
            // native observer, without calling its controller/lifecycle.
            refresh_current_support(&mut shadow)?;
            let current = standing_diagnostic(&shadow, interval.anchors)?;
            let (current, passed, used) = interval.observe(current)?;
            let clearance = super::super::foot_floor_clearance(&shadow)?;
            trace.push(serde_json::json!({"forecastNativeSubstep":shadow.substeps_completed,
                "decisionIndex":index,"withinDecisionNative":substep+1,"action":action,
                "internalPgsIterations":shadow.world.integration_parameters.num_internal_pgs_iterations,
                "current":current,"footFloorClearanceM":clearance,"standingPassed":passed,
                "prospectiveAnchorsUsed":used,"prospectiveAnchorsForNext":interval.anchors,
                "consecutiveFullStandingSamples":interval.consecutive}));
            if original_cut.is_some() && interval.consecutive >= REQUIRED_NATIVE_SAMPLES {
                endpoint = Some(serde_json::json!({"forecastNativeSubstep":shadow.substeps_completed,
                    "decisionIndex":index,"withinDecisionNative":substep+1,
                    "consecutiveFullStandingSamples":interval.consecutive,
                    "anchors":interval.anchors,"current":trace.last().unwrap()["current"]}));
                if substep + 1 == native_per_policy { shadow.tick = index as u64 + 1; }
                archive.record_teacher(&shadow, physical.to_vec(), action, before)?;
                break 'decisions;
            }
        }
        shadow.tick = index as u64 + 1;
        archive.record_teacher(&shadow, physical.to_vec(), action, before)?;
        let head = super::super::physical_observation(&shadow)?[21];
        original_cut_after_frame(&mut finish_after_step, &mut original_cut, index + 1, head);
        if original_cut.is_some() && interval.consecutive >= REQUIRED_NATIVE_SAMPLES {
            endpoint = Some(serde_json::json!({"forecastNativeSubstep":shadow.substeps_completed,
                "decisionIndex":index,"withinDecisionNative":native_per_policy,
                "consecutiveFullStandingSamples":interval.consecutive,
                "anchors":interval.anchors,"current":trace.last().unwrap()["current"]}));
            break;
        }
    }
    let teacher_initial_json = serde_json::to_string(&teacher_initial).map_err(|e|e.to_string())?;
    let teacher_frames_json = serde_json::to_string(&archive.teacher_frames).map_err(|e|e.to_string())?;
    let last_observed = trace.last().and_then(|row|row["forecastNativeSubstep"].as_u64()).unwrap_or(0);
    let accounting_valid = shadow.substeps_completed == (decisions as u64 - 1) * u64::from(native_per_policy)
        + u64::from(last_decision_native_count)
        && decisions <= MAX_TEACHER_TICKS && shadow.substeps_completed <= 5000;
    if !accounting_valid { return Err("Teacher feasibility accounting differs".into()); }
    Ok(serde_json::json!({"experiment":"original-back-weak-teacher-feasibility-v1",
        "status":if failure.is_some(){"integrity-failure"}else if endpoint.is_some(){"feasible-full-standing-endpoint"}else{"no-full-standing-endpoint-within-ceiling"},
        "realNativeSubsteps":real.substeps_completed,"realPhysicsIntegrations":0,
        "totalDecisionCeiling":MAX_TEACHER_TICKS,"nativePerDecision":native_per_policy,
        "teacherPower":TEACHER_POWER,"teacherNativeDtS":shadow.profile.dt_s,
        "decisionsStarted":decisions,"forecastNativeIntegrations":shadow.substeps_completed,
        "lastDecisionNativeCount":last_decision_native_count,"originalHeightCutDecisions":original_cut,
        "requiredConsecutiveFullStandingSamples":REQUIRED_NATIVE_SAMPLES,
        "standingIntervalSemantics":"Continuous original prospective anchors/streak from forecast native1; endpoint acceptance only after the complete inherited height-cut prefix. No reset or re-anchoring at that cut.",
        "maximumConsecutiveFullStandingSamples":interval.maximum,
        "teacherInitialJson":teacher_initial_json,"teacherFramesJson":teacher_frames_json,
        "teacherFrameCount":archive.teacher_frames.len(),"nativeAccountingValid":accounting_valid,
        "unobservedNativeTail":shadow.substeps_completed-last_observed,
        "nativeTrace":trace,"endpoint":endpoint,"failure":failure,
        "referenceGenerated":false,"canonicalAppended":false,"realHandoffActivated":false,
        "forecastCounters":shadow.counters,
        "archiveContactSemantics":"TeacherFrame snapshot.contacts remains the original stale seed cache for exact source-prefix audit; never use it as current load evidence. nativeTrace.current contains refreshed solved loads, persistence, geometry and current generalized motion on every successful forecast native step."}))
}

#[cfg(test)]
#[path = "teacher_feasibility_tests.rs"]
mod tests;
