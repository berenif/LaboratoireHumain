//! Tick-indexed inputs exported from the repository fixtures; no wall clock.
use crate::{Profile, Simulation, measurement};
use lh_contracts::{Action, Command, Stamp, Timeline};
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
    run_disturbances(profile, selected, false)
}
/// Behavioral diagnostic that preserves the fixture's minimum completed steps
/// and releases the pointer only after measured liftoff in the swing phase.
pub fn steps_with_profile(
    profile: &Profile,
    selected: Option<&str>,
) -> Result<Vec<serde_json::Value>, String> {
    run_disturbances(profile, selected, true)
}
fn run_disturbances(
    profile: &Profile,
    selected: Option<&str>,
    require_steps: bool,
) -> Result<Vec<serde_json::Value>, String> {
    let inputs: Inputs = serde_json::from_str(INPUTS).map_err(|e| e.to_string())?;
    if inputs.schema != 1 || inputs.command_tick_hz != 60 {
        return Err("Fixture schema/rate".into());
    }
    let mut reports = Vec::new();
    for p in inputs
        .pulls
        .iter()
        .filter(|p| p.outcome == "recoverable" && (require_steps || !p.release_at_first_swing))
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
        let mut trace = Vec::new();
        let mut release_tick = None;
        let mut step_events = Vec::new();
        let mut last_step_event = ("idle", false, 0);
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
                s.apply_grab_command(&command)
                    .map_err(|e| format!("Fixture apply: {e:?}"))?;
            }
            if let Err(f) = s.advance_tick_observed(|s, substep| {
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
                if pelvis.translation().y < 0.56 {
                    return Err(measurement::Failure::new(
                        s.tick,
                        substep,
                        "recoverable-pelvis-height",
                        "pelvis".into(),
                        pelvis.translation().y,
                        0.56,
                    ));
                }
                if s.tick >= p.observe_until_frame.saturating_sub(60) {
                    measurement::standing_support(s, substep)?;
                }
                Ok(())
            }) {
                failure = Some(f);
                s.cancel_grab();
                break;
            }
            timeline.tick = s.tick;
            if tick % 15 == 0 {
                trace.push(json!({"tick":s.tick,"com":crate::vec3(s.center_of_mass()),
                    "pelvis":s.snapshot().segments[0],"contacts":s.contacts,"targets":s.targets,
                    "step":s.stepping.as_ref().map(|controller| &controller.status)}));
            }
        }
        let physical_passed = failure.is_none() && s.tick == p.observe_until_frame + 1;
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
        } else {
            "Existing recoverable pull inputs, physical integrity/no fall and final standing evidence. Corrective-step counts and release-during-swing are not admitted by this disturbance probe."
        };
        reports.push(json!({"id":p.id,"scope":scope,
            "passed":passed,"physicalPassed":physical_passed,"requiresSteps":require_steps,"stepsPassed":steps_passed,
            "minimumSteps":p.minimum_steps,"releaseAtFirstSwing":p.release_at_first_swing,"releasedAtTick":release_tick,
            "behaviorFailures":behavior_failures,"stepEvents":step_events,
            "completedTicks":s.tick,"firstFailure":failure,"commands":replay,"trace":trace,"final":s.snapshot()}));
    }
    if reports.is_empty() {
        return Err("No disturbance fixtures selected".into());
    }
    Ok(reports)
}
