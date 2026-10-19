//! Tick-indexed inputs exported from the repository fixtures; no wall clock.
use crate::{Profile, Simulation, measurement};
use lh_contracts::{Action, Command, Motion, Stamp, Timeline};
use lh_model::Vec3;
use serde::Deserialize;
use serde_json::json;

pub const INPUTS: &str = include_str!("../data/scenarios-v1.json");
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Inputs {
    schema: u32,
    command_tick_hz: u32,
    pulls: Vec<Pull>,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Pull {
    id: String,
    initial: Initial,
    region: String,
    local_anchor: Vec3,
    targets: Vec<Target>,
    release_frame: u64,
    observe_until_frame: u64,
    outcome: String,
    minimum_steps: u32,
    #[serde(default)]
    release_at_first_swing: bool,
}
#[derive(Deserialize)]
struct Initial {
    heading: f32,
}
#[derive(Deserialize)]
struct Target {
    frame: u64,
    offset: Vec3,
}
fn offset(p: &Pull, tick: u64) -> Result<rapier3d::prelude::Vector, String> {
    let last = p.targets.last().ok_or("Empty target schedule")?;
    let Some(next) = p.targets.iter().position(|t| t.frame >= tick) else {
        return Ok(crate::vector(last.offset));
    };
    if next == 0 {
        return Ok(crate::vector(p.targets[0].offset));
    }
    let a = &p.targets[next - 1];
    let b = &p.targets[next];
    if b.frame <= a.frame {
        return Err("Unordered target schedule".into());
    }
    let blend = (tick - a.frame) as f32 / (b.frame - a.frame) as f32;
    Ok(crate::vector(a.offset) + (crate::vector(b.offset) - crate::vector(a.offset)) * blend)
}
pub fn disturbances() -> Result<Vec<serde_json::Value>, String> {
    disturbances_with_profile(&Profile::default(), None)
}
pub fn disturbances_with_profile(
    profile: &Profile,
    selected: Option<&str>,
) -> Result<Vec<serde_json::Value>, String> {
    run_disturbances(profile, selected, Scope::Disturbances)
}
/// The existing eight recoverable pull probes used by the native foundation.
/// Overpower and verified-swing fixtures remain required by the release gates.
pub fn foundation_with_profile(profile: &Profile) -> Result<Vec<serde_json::Value>, String> {
    run_disturbances(profile, None, Scope::Foundation)
}
/// Behavioral diagnostic that preserves the fixture's minimum completed steps
/// and releases the pointer only after measured liftoff in the swing phase.
pub fn steps_with_profile(
    profile: &Profile,
    selected: Option<&str>,
) -> Result<Vec<serde_json::Value>, String> {
    run_disturbances(profile, selected, Scope::Steps)
}
#[derive(Clone, Copy, PartialEq)]
enum Scope {
    Foundation,
    Disturbances,
    Steps,
}
fn includes_fixture(scope: Scope, fixture: &Pull) -> bool {
    (scope == Scope::Disturbances || fixture.outcome == "recoverable")
        && (scope == Scope::Steps || !fixture.release_at_first_swing)
}
fn run_disturbances(
    profile: &Profile,
    selected: Option<&str>,
    scope: Scope,
) -> Result<Vec<serde_json::Value>, String> {
    let require_steps = scope == Scope::Steps;
    let inputs: Inputs = serde_json::from_str(INPUTS).map_err(|e| e.to_string())?;
    if inputs.schema != 1 {
        return Err("Fixture schema".into());
    }
    if inputs.command_tick_hz != lh_contracts::TICK_HZ {
        return Err(format!(
            "Legacy disturbance fixtures require a {} Hz anatomical model; the current GetUp contract is {} Hz and is not qualified by these fixtures. Re-export and qualify model-specific inputs instead of implicitly remapping them.",
            inputs.command_tick_hz,
            lh_contracts::TICK_HZ,
        ));
    }
    let mut reports = Vec::new();
    for p in inputs
        .pulls
        .iter()
        .filter(|p| includes_fixture(scope, p))
        .filter(|p| selected.is_none_or(|id| p.id == id))
    {
        let mut s = Simulation::new(p.initial.heading, profile.clone(), true)?;
        let segment = s
            .model
            .segments
            .iter()
            .position(|s| s.id == p.region)
            .ok_or("Fixture segment")?;
        let body = &s.world.bodies[s.bodies[segment]];
        let start = body
            .position()
            .transform_point(crate::vector(p.local_anchor));
        let heading = rapier3d::prelude::Rotation::from_rotation_y(p.initial.heading);
        let mut timeline = Timeline::new(s.generation);
        let mut replay = Vec::new();
        let mut failure = None;
        let mut command_rejections = Vec::new();
        let mut trace = Vec::new();
        let swing_trace = std::cell::RefCell::new(Vec::<serde_json::Value>::new());
        let mut swing_diagnostic_errors = Vec::new();
        let mut release_tick = None;
        let mut step_events = Vec::new();
        let mut last_step_event = ("idle", false, 0);
        let mut minimum_pelvis_y = f32::INFINITY;
        for tick in 0..=p.observe_until_frame {
            let release = release_tick.is_none()
                && if p.release_at_first_swing {
                    s.stepping.as_ref().is_some_and(|controller| {
                        controller.status.phase == "swing" && controller.status.verified_liftoff
                    })
                } else {
                    tick == p.release_frame
                };
            let action = if tick == 0 {
                Some(Action::GrabBegin {
                    press: 41,
                    segment: segment as u8,
                    local_anchor: p.local_anchor,
                    target: crate::vec3(start),
                })
            } else if release {
                release_tick = Some(tick);
                Some(Action::GrabEnd { press: 41 })
            } else if release_tick.is_none() {
                Some(Action::GrabMove {
                    press: 41,
                    target: crate::vec3(start + heading * offset(p, tick)?),
                })
            } else {
                None
            };
            if let Some(action) = action {
                let command = Command {
                    stamp: Stamp::new(s.generation, tick + 1, tick),
                    action,
                };
                timeline
                    .enqueue(command.clone())
                    .map_err(|e| format!("Fixture enqueue: {e:?}"))?;
                replay.push(command);
            }
            for command in timeline.boundary() {
                if let Err(rejected) = s.apply_grab_command(&command) {
                    command_rejections.push(json!({"command":command,"rejected":rejected}));
                    // A physical fall cancels the press. Its remaining exported
                    // moves still arrive and must never resurrect that grab.
                    let cancelled_move = p.outcome != "recoverable"
                        && matches!(
                            s.motion,
                            Motion::Falling | Motion::Fallen | Motion::Recovering
                        )
                        && s.grab.is_none()
                        && matches!(
                            command.action,
                            Action::GrabMove { .. } | Action::GrabEnd { .. }
                        );
                    if !cancelled_move {
                        failure = Some(measurement::Failure::new(
                            s.tick,
                            0,
                            "fixture-command-rejection",
                            format!("{rejected:?}"),
                            1.0,
                            0.0,
                        ));
                    }
                }
            }
            if failure.is_some() {
                break;
            }
            if let Err(f) = s.advance_tick_traced(|s, substep| {
                if let Some(sample) = swing_trace.borrow_mut().last_mut()
                    && sample["tick"] == s.tick && sample["substep"] == substep
                {
                    let side = sample["side"].as_u64().expect("diagnostic side") as usize;
                    let mut loads = [0.0_f32; 2];
                    let mut motors = Vec::new();
                    for (i, part) in s.model.segments.iter().enumerate() {
                        let part_side = usize::from(part.id.starts_with("right"));
                        if matches!(part.role.as_str(), "hindfoot" | "forefoot") {
                            loads[part_side] += s.support_normal_impulse(s.colliders[i])
                                .expect("owned diagnostic collider") / s.profile.dt_s;
                        }
                        if part_side != side || !matches!(part.role.as_str(), "thigh" | "shin" | "ankle" | "hindfoot") {
                            continue;
                        }
                        let profile = part.joint_profile.as_ref().expect("leg profile");
                        let joint = s.world.impulse_joints.get(s.joints[i].expect("leg handle")).expect("leg joint");
                        let measured = measurement::coordinates(
                            *s.world.bodies[joint.body1()].rotation(),
                            *s.world.bodies[joint.body2()].rotation(), profile);
                        for axis in &profile.axes {
                            let k = ['x', 'y', 'z'].iter().position(|c| *c == axis.coordinate).expect("leg coordinate");
                            let motor = &joint.data.motors[3 + k];
                            motors.push(json!({"segment":i,"coordinate":axis.coordinate,
                                "measuredRad":measured[k],"deliveredAbsImpulseNmS":motor.delivered_abs_impulse,
                                "capImpulseNmS":motor.max_force * s.profile.dt_s}));
                        }
                        if part.role == "hindfoot" {
                            sample["footAfter"] = json!(crate::vec3(s.world.bodies[s.bodies[i]].translation()));
                        }
                    }
                    sample["loadsAfterN"] = json!(loads);
                    sample["pelvisAfter"] = json!(s.snapshot().segments[0]);
                    sample["axesAfter"] = json!(motors);
                }
                if let Some(controller) = &s.stepping {
                    let key = (controller.status.phase, controller.status.verified_liftoff, s.counters.steps);
                    if key != last_step_event {
                        let foot = controller.status.side.map(|side| {
                            let id = if side == 0 { "leftFoot" } else { "rightFoot" };
                            s.model.segments.iter().position(|p| p.id == id).map(|i| crate::vec3(s.world.bodies[s.bodies[i]].translation()))
                        });
                        step_events.push(json!({"tick":s.tick,"substep":substep,"step":controller.status,"foot":foot,"completedSteps":s.counters.steps}));
                        last_step_event = key;
                    }
                }
                let pelvis = &s.world.bodies[s.bodies[0]];
                minimum_pelvis_y = minimum_pelvis_y.min(pelvis.translation().y);
                if p.outcome == "recoverable" && pelvis.translation().y < 0.56 {
                    return Err(measurement::Failure::new(
                        s.tick,
                        substep,
                        "recoverable-pelvis-height",
                        "pelvis".into(),
                        pelvis.translation().y,
                        0.56,
                    ));
                }
                if p.outcome == "recoverable" && s.tick >= p.observe_until_frame.saturating_sub(60) {
                    measurement::standing_support(s, substep)?;
                }
                Ok(())
            }, |s, event| {
                if event.phase == crate::profiling::Phase::Controller && !event.begin
                    && let Some(controller) = &s.stepping
                {
                    match controller.diagnose_swing(s) {
                        Ok(Some(mut sample)) => {
                            sample["tick"] = json!(event.tick);
                            sample["substep"] = json!(event.substep);
                            swing_trace.borrow_mut().push(sample);
                        }
                        Ok(None) => {}
                        Err(error) => swing_diagnostic_errors.push(error),
                    }
                }
            }) {
                failure = Some(f);
                s.cancel_grab();
                break;
            }
            timeline.tick = s.tick;
            if tick % 15 == 0 {
                let limb_motors: Vec<_> = s
                    .model
                    .segments
                    .iter()
                    .enumerate()
                    .filter(|(_, segment)| {
                        matches!(
                            segment.role.as_str(),
                            "thigh" | "shin" | "ankle" | "hindfoot"
                        )
                    })
                    .filter_map(|(i, segment)| {
                        let profile = segment.joint_profile.as_ref()?;
                        let joint = s.world.impulse_joints.get(s.joints[i]?)?;
                        let coordinates = measurement::coordinates(
                            *s.world.bodies[joint.body1()].rotation(),
                            *s.world.bodies[joint.body2()].rotation(),
                            profile,
                        );
                        let caps: [f32; 3] = std::array::from_fn(|k| {
                            if profile
                                .axes
                                .iter()
                                .any(|axis| axis.coordinate == ['x', 'y', 'z'][k])
                            {
                                joint.data.motors[3 + k].max_force
                            } else {
                                0.0
                            }
                        });
                        Some(json!({"segment":i,"coordinates":coordinates,"axisCapsNm":caps}))
                    })
                    .collect();
                trace.push(json!({"tick":s.tick,"com":crate::vec3(s.center_of_mass()),
                    "pelvis":s.snapshot().segments[0],"contacts":s.contacts,"targets":s.targets,
                    "limbMotors":limb_motors,"deliveredMotors":s.diagnostics.delivered_motors,
                    "step":s.stepping.as_ref().map(|controller| &controller.status)}));
            }
        }
        if !swing_diagnostic_errors.is_empty() {
            return Err(format!("Swing diagnostic: {swing_diagnostic_errors:?}"));
        }
        let physical_passed = failure.is_none()
            && s.tick == p.observe_until_frame + 1
            && (p.outcome == "recoverable" || s.counters.falls > 0 && minimum_pelvis_y < 0.55);
        let steps_passed = s.counters.steps >= p.minimum_steps
            && (!p.release_at_first_swing || release_tick.is_some());
        let passed = physical_passed && (!require_steps || steps_passed);
        let mut behavior_failures = Vec::new();
        if s.counters.steps < p.minimum_steps {
            behavior_failures.push(json!({"check":"minimum-completed-steps","measured":s.counters.steps,"threshold":p.minimum_steps}));
        }
        if p.release_at_first_swing && release_tick.is_none() {
            behavior_failures.push(
                json!({"check":"release-during-verified-swing","measured":false,"threshold":true}),
            );
        }
        let scope = if require_steps {
            "Recoverable fixture diagnostic with unchanged minimum completed steps, measured liftoff/touchdown, no-fall and final standing. Release after verified swing when required. Other Gate D and E requirements remain untested."
        } else if scope == Scope::Foundation {
            "Existing recoverable pull inputs, physical integrity/no fall and final standing evidence. Corrective-step counts and release-during-swing are not admitted by this disturbance probe."
        } else {
            "Full disturbance physical matrix: recoverable standing and overpower falls with physical integrity. Corrective-step counts and release-during-swing require the separate strict-step gate."
        };
        reports.push(json!({"id":p.id,"scope":scope,
            "passed":passed,"physicalPassed":physical_passed,"requiresSteps":require_steps,"stepsPassed":steps_passed,
            "minimumSteps":p.minimum_steps,"releaseAtFirstSwing":p.release_at_first_swing,"releasedAtTick":release_tick,
            "behaviorFailures":behavior_failures,"stepEvents":step_events,"swingTrace":swing_trace.into_inner(),
            "completedTicks":s.tick,"firstFailure":failure,"commands":replay,"commandRejections":command_rejections,"trace":trace,"minimumPelvisY":minimum_pelvis_y,"final":s.snapshot()}));
    }
    if reports.is_empty() {
        return Err("No disturbance fixtures selected".into());
    }
    Ok(reports)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_inputs_are_rejected_before_constructing_a_different_physical_model() {
        let inputs: Inputs = serde_json::from_str(INPUTS).unwrap();
        assert_eq!(
            inputs.command_tick_hz, 60,
            "historical inputs remain unchanged"
        );
        assert_eq!(lh_contracts::TICK_HZ, 40);
        for scope in [Scope::Foundation, Scope::Disturbances, Scope::Steps] {
            let error = run_disturbances(&Profile::default(), Some("slow-hand-forward"), scope)
                .expect_err("historical model/clock cannot qualify GetUp");
            assert!(error.contains("Legacy disturbance fixtures require a 60 Hz anatomical model"));
            assert!(error.contains("current GetUp contract is 40 Hz"));
        }
    }

    #[test]
    fn foundation_scope_preserves_recoverable_probes_and_full_release_coverage() {
        let inputs: Inputs = serde_json::from_str(INPUTS).unwrap();
        let selected = |scope| {
            inputs
                .pulls
                .iter()
                .filter(|fixture| includes_fixture(scope, fixture))
                .map(|fixture| fixture.id.as_str())
                .collect::<Vec<_>>()
        };
        let foundation = selected(Scope::Foundation);
        assert_eq!(foundation.len(), 8);
        assert!(
            foundation
                .iter()
                .all(|id| !id.starts_with("fast-") && !id.starts_with("sustained-"))
        );
        let disturbances = selected(Scope::Disturbances);
        assert_eq!(disturbances.len(), 14);
        assert!(disturbances.contains(&"fast-right-heading"));
        assert!(disturbances.contains(&"sustained-backward-heading"));
        assert!(foundation.iter().all(|id| disturbances.contains(id)));
        let steps = selected(Scope::Steps);
        assert_eq!(steps.len(), 9);
        assert!(steps.contains(&"release-during-corrective-step"));
        assert!(!foundation.contains(&"release-during-corrective-step"));
        assert!(foundation.iter().all(|id| steps.contains(id)));
    }
}
