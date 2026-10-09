//! Exact tick schedules and behavioral assertions retained from the four legacy
//! regression tests. These are mandatory release fixtures, not general pulls.
use crate::{Profile, Simulation, measurement, vec3, vector};
use lh_contracts::{Action, Command, Mode, Motion, Stamp};
use lh_model::Vec3;
use rapier3d::prelude::*;
use serde_json::{Value, json};

fn locked(motion: Motion) -> bool {
    matches!(
        motion,
        Motion::Falling | Motion::Fallen | Motion::Recovering
    )
}
fn hand(s: &Simulation) -> usize {
    s.model
        .segments
        .iter()
        .position(|p| p.id == "rightHand")
        .unwrap()
}
fn failure(
    s: &Simulation,
    substep: u32,
    check: &str,
    measured: f32,
    threshold: f32,
) -> measurement::Failure {
    measurement::Failure::new(
        s.tick,
        substep,
        check,
        "legacy behavioral regression".into(),
        measured,
        threshold,
    )
}

fn pull(profile: &Profile, heading: f32, reversal: bool, strong: bool) -> Result<Value, String> {
    let mut s = Simulation::new(heading, profile.clone(), true)?;
    let original = s.bodies.clone();
    let segment = hand(&s);
    let anchor = if reversal || strong {
        Vec3 {
            x: 0.025,
            y: 0.015,
            z: 0.01,
        }
    } else {
        Vec3::default()
    };
    let start = s.world.bodies[s.bodies[segment]]
        .position()
        .transform_point(vector(anchor));
    s.begin_grab(
        segment,
        if strong {
            52
        } else if reversal {
            41
        } else {
            3
        },
        anchor,
        vec3(start),
    )
    .map_err(|e| format!("{e:?}"))?;
    let mut first_failure = s.advance_tick().err(); // fixedUpdate(begin), tick zero
    let mut first_fall_tick = None;
    let mut minimum_pelvis_y = s.world.bodies[s.bodies[0]].translation().y;
    let mut trace = Vec::new();
    let limit = if strong { 180 } else { 510 };
    for tick in 1..=limit {
        if first_failure.is_some() {
            break;
        }
        let offset = if strong && tick < 50 {
            let fraction = (tick as f32 / 5.0).min(1.0);
            Some(Vector::new(1.25 * fraction, 0.12 * fraction, 0.0))
        } else if reversal && tick < 300 {
            let x = if tick <= 90 {
                0.75 * tick as f32 / 90.0
            } else if tick <= 180 {
                0.75 - 1.5 * (tick - 90) as f32 / 90.0
            } else {
                -0.75
            };
            Some(Vector::new(x, 0.02 * (tick as f32).min(90.0) / 90.0, 0.0))
        } else if !strong && !reversal && tick <= 180 {
            Some(if tick <= 90 {
                Vector::new(0.60 * tick as f32 / 90.0, 0.0, 0.08 * tick as f32 / 90.0)
            } else {
                Vector::new(0.6 - (tick - 90) as f32 * 0.9 / 90.0, 0.0, 0.08)
            })
        } else {
            None
        };
        if let Some(offset) = offset {
            // The legacy fixedUpdate ignores moves after a cancelled fall press.
            if s.grab.is_some() {
                s.move_grab(
                    if strong {
                        52
                    } else if reversal {
                        41
                    } else {
                        3
                    },
                    vec3(start + s.heading * offset),
                )
                .map_err(|e| format!("{e:?}"))?;
            }
        } else if tick
            == if strong {
                50
            } else if reversal {
                300
            } else {
                181
            }
        {
            s.cancel_grab();
        }
        first_failure = s
            .advance_tick_observed(|state, substep| {
                if state.bodies != original
                    || state
                        .bodies
                        .iter()
                        .any(|h| !state.world.bodies[*h].is_dynamic())
                {
                    return Err(failure(
                        state,
                        substep,
                        "legacy-dynamic-ownership",
                        0.0,
                        1.0,
                    ));
                }
                if !strong && locked(state.motion) {
                    return Err(failure(state, substep, "legacy-unexpected-fall", 1.0, 0.0));
                }
                if locked(state.motion) && state.grab.is_some() {
                    return Err(failure(
                        state,
                        substep,
                        "legacy-fall-grab-cancellation",
                        1.0,
                        0.0,
                    ));
                }
                // advance_tick already inspects finite states, actual transforms,
                // anchors, collisions, penetration and canonical joint limits.
                Ok(())
            })
            .err();
        if locked(s.motion) {
            first_fall_tick.get_or_insert(tick);
        }
        minimum_pelvis_y = minimum_pelvis_y.min(s.world.bodies[s.bodies[0]].translation().y);
        if tick % 15 == 0 {
            trace.push(json!({"tick":tick,"snapshot":s.snapshot()}));
        }
    }
    let behavior = if strong {
        first_fall_tick.is_some() && minimum_pelvis_y < 0.55
    } else {
        s.counters.steps >= if reversal { 2 } else { 1 } && s.motion == Motion::Upright
    };
    Ok(
        json!({"heading":heading,"passed":first_failure.is_none() && s.tick == limit + 1 && behavior,
        "firstFailure":first_failure,"firstFallTick":first_fall_tick,"minimumPelvisY":minimum_pelvis_y,
        "minimumSteps":if reversal {2} else if strong {0} else {1},"final":s.snapshot(),"trace":trace}),
    )
}

fn input_lockout(profile: &Profile) -> Result<Value, String> {
    let mut runtime = crate::runtime::Runtime::new(Mode::Protocol, 0.0, profile.clone())?;
    let initial = runtime.advance(0).map_err(|e| format!("{e:?}"))?.snapshot;
    let segment = initial
        .segments
        .iter()
        .position(|p| p.id == "rightHand")
        .unwrap() as u8;
    let start = vector(initial.segments[segment as usize].position);
    let mut sequence = 0;
    let mut snapshot = initial;
    let mut commands = Vec::new();
    let mut issue = |runtime: &mut crate::runtime::Runtime,
                     snapshot: &mut lh_contracts::Snapshot,
                     action: Option<Action>|
     -> Result<(crate::runtime::Reply, Option<lh_contracts::Reject>), String> {
        if let Some(action) = action {
            sequence += 1;
            let command = Command {
                stamp: Stamp::new(snapshot.stamp.generation, sequence, snapshot.stamp.tick),
                action,
            };
            match runtime.enqueue(command.clone()) {
                Ok(()) => commands.push(command),
                Err(lh_contracts::Reject::Cancelled) => {
                    let reply = runtime.advance(1).map_err(|e| format!("{e:?}"))?;
                    commands.push(command);
                    *snapshot = reply.snapshot.clone();
                    return Ok((reply, Some(lh_contracts::Reject::Cancelled)));
                }
                Err(e) => return Err(format!("{e:?}")),
            }
        }
        let reply = runtime.advance(1).map_err(|e| format!("{e:?}"))?;
        *snapshot = reply.snapshot.clone();
        Ok((reply, None))
    };
    issue(
        &mut runtime,
        &mut snapshot,
        Some(Action::GrabBegin {
            press: 81,
            segment,
            local_anchor: Vec3::default(),
            target: vec3(start),
        }),
    )?;
    let mut first_failure = None;
    for tick in 1..=180 {
        if locked(snapshot.motion) {
            break;
        }
        let action = (tick <= 5).then(|| Action::GrabMove {
            press: 81,
            target: vec3(
                start
                    + Vector::new(
                        1.25 * tick as f32 / 5.0,
                        0.15 * tick as f32 / 5.0,
                        0.2 * tick as f32 / 5.0,
                    ),
            ),
        });
        let reply = issue(&mut runtime, &mut snapshot, action)?;
        if reply.0.failure.is_some() {
            first_failure = reply.0.failure;
            break;
        }
    }
    let fell = locked(snapshot.motion);
    let target = snapshot.segments[segment as usize].position;
    let rejected = issue(
        &mut runtime,
        &mut snapshot,
        Some(Action::GrabBegin {
            press: 82,
            segment,
            local_anchor: Vec3::default(),
            target,
        }),
    )?;
    let locked_press_rejected = rejected.1 == Some(lh_contracts::Reject::Cancelled)
        || rejected
            .0
            .acknowledgements
            .iter()
            .any(|a| a.rejected == Some(lh_contracts::Reject::Cancelled));
    // The legacy test explicitly requests reset here. This is observable input,
    // never a hidden reset used to pass a physical get-up or cycle fixture.
    issue(&mut runtime, &mut snapshot, Some(Action::Reset))?;
    let reset_upright = snapshot.motion == Motion::Upright;
    let target = snapshot.segments[segment as usize].position;
    let fresh = issue(
        &mut runtime,
        &mut snapshot,
        Some(Action::GrabBegin {
            press: 83,
            segment,
            local_anchor: Vec3::default(),
            target,
        }),
    )?;
    let fresh_accepted = fresh.1.is_none()
        && fresh
            .0
            .acknowledgements
            .iter()
            .any(|a| a.rejected.is_none());
    Ok(
        json!({"passed":fell && first_failure.is_none() && locked_press_rejected && reset_upright && fresh_accepted,
        "fallObserved":fell,"lockedPressRejected":locked_press_rejected,"explicitResetUpright":reset_upright,
        "freshPressAccepted":fresh_accepted,"firstFailure":first_failure,"commands":commands,"final":snapshot}),
    )
}

pub fn trials(profile: &Profile) -> Result<Vec<Value>, String> {
    let slow = [0.0, 1.1, -1.7]
        .into_iter()
        .map(|heading| pull(profile, heading, false, false))
        .collect::<Result<Vec<_>, _>>()?;
    let reversal = pull(profile, 0.0, true, false)?;
    let lockout = input_lockout(profile)?;
    let descent = pull(profile, 0.0, false, true)?;
    Ok(vec![
        json!({"id":"slow-pulls-release-reversal","source":"tests/balance-controller.test.mjs","passed":slow.iter().all(|r| r["passed"] == true),"cases":slow}),
        json!({"id":"planted-reversal","source":"tests/balance-controller.test.mjs","passed":reversal["passed"],"case":reversal}),
        json!({"id":"fall-recovery-lockout-fresh-press","source":"tests/character-domain.test.mjs","passed":lockout["passed"],"case":lockout}),
        json!({"id":"strong-pull-connected-descent","source":"tests/physical-chain-integrity.test.mjs","passed":descent["passed"],"case":descent}),
    ])
}
